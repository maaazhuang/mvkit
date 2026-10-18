#include "runtime/memory/builtin_pool_resource.hpp"

#include <algorithm>
#include <cstring>
#include <new>
#include <stdexcept>
#include <thread>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#endif

namespace mv::runtime {
void reclaim_thread_caches(std::size_t token) noexcept;
struct PoolThreadLife {
  std::size_t token;
  std::atomic<bool> exited{false};
  explicit PoolThreadLife(std::size_t id) : token(id) {}
};
namespace {
constexpr std::uint64_t kFrontCanary = UINT64_C(0x4d564b4954504f4f);
constexpr std::uint64_t kBackCanary = UINT64_C(0x4c53414645424c4b);
struct alignas(std::max_align_t) AllocationHeader {
  void* base;
  std::size_t raw_bytes, requested_bytes, requested_alignment;
  std::size_t raw_alignment;
  int list;
  std::uint16_t shard;
  bool large;
  bool virtual_memory;
#ifndef NDEBUG
  std::uint64_t front_canary;
#endif
};
struct Registry {
  std::mutex mutex;
  BuiltinPoolMemoryResource* head = nullptr;
  std::atomic<std::size_t> credit{0}, limit{256u * kSizeClassMiB};
  std::atomic<std::size_t> thread_id{0};
};
Registry& registry() noexcept {
  alignas(Registry) static std::byte storage[sizeof(Registry)];
  static auto* p = std::construct_at(reinterpret_cast<Registry*>(storage));
  return *p;
}
thread_local std::size_t current_token = 0;
thread_local bool thread_exiting = false;
struct ThreadOwner {
  std::shared_ptr<PoolThreadLife> life;
  ~ThreadOwner() {
    thread_exiting = true;
    current_token = 0;
    if (life) {
      life->exited.store(true, std::memory_order_release);
      reclaim_thread_caches(life->token);
    }
  }
};
ThreadOwner& thread_owner() {
  thread_local ThreadOwner owner;
  return owner;
}
void ensure_thread_owner() {
  if (current_token || thread_exiting) return;
  auto& owner = thread_owner();
  const auto token =
      registry().thread_id.fetch_add(1, std::memory_order_relaxed) + 1;
  owner.life = std::make_shared<PoolThreadLife>(token);
  current_token = token;
}
bool charge(std::atomic<std::size_t>& counter, std::size_t limit,
            std::size_t bytes) noexcept {
  auto current = counter.load(std::memory_order_relaxed);
  while (bytes <= limit && current <= limit - bytes) {
    if (counter.compare_exchange_weak(current, current + bytes,
                                      std::memory_order_relaxed))
      return true;
  }
  return false;
}
bool charge_idle(std::size_t bytes) noexcept {
  return charge(registry().credit,
                registry().limit.load(std::memory_order_relaxed), bytes);
}
void release_idle(std::size_t bytes) noexcept {
  registry().credit.fetch_sub(bytes, std::memory_order_relaxed);
}
bool is_power_of_two(std::size_t value) noexcept {
  return value && !(value & (value - 1));
}
AllocationHeader* header_of(void* p) noexcept {
  return reinterpret_cast<AllocationHeader*>(static_cast<std::byte*>(p) -
                                             sizeof(AllocationHeader));
}
void system_free(AllocationHeader* h) noexcept {
#ifdef _WIN32
  if (h->virtual_memory) {
    VirtualFree(h->base, 0, MEM_RELEASE);
    return;
  }
#endif
  std::pmr::new_delete_resource()->deallocate(h->base, h->raw_bytes,
                                              h->raw_alignment);
}
struct Layout {
  std::size_t payload, alignment, offset, total;
  int list;
};
Layout layout_for(PoolPurpose purpose, std::size_t bytes, std::size_t alignment,
                  bool large) {
  if (!is_power_of_two(alignment))
    throw std::invalid_argument("alignment must be a non-zero power of two");
  const auto normalized = normalize_alignment(alignment);
  const int index = bytes <= kMaxPooledPayload || large
                        ? rounded_payload_index(purpose, bytes)
                        : -1;
  const auto payload = index >= 0 ? payload_at_index(purpose, index) : bytes;
  const auto capacity = (std::max)(payload, std::size_t{64});
  const auto raw_alignment = (std::max)(normalized, std::size_t{64});
  // 隔离相邻有效分配，避免相机 worker 间伪共享。
  if (size_add_overflows(sizeof(AllocationHeader), raw_alignment - 1))
    throw std::bad_alloc();
  const auto offset =
      (sizeof(AllocationHeader) + raw_alignment - 1) & ~(raw_alignment - 1);
  constexpr std::size_t tail =
#ifndef NDEBUG
      sizeof(kBackCanary);
#else
      0;
#endif
  if (size_add_overflows(offset, capacity) ||
      size_add_overflows(offset + capacity, tail))
    throw std::bad_alloc();
  const int ai = alignment_class_index(alignment);
  return {payload, raw_alignment, offset, offset + capacity + tail,
          index >= 0 && ai >= 0 ? index * 3 + ai : -1};
}
}  // namespace

void BuiltinPoolMemoryResource::SpinMutex::lock() noexcept {
  while (flag.test_and_set(std::memory_order_acquire)) {
    while (flag.test(std::memory_order_relaxed)) {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
      _mm_pause();
#elif defined(__x86_64__) || defined(__i386__)
      __builtin_ia32_pause();
#else
      std::this_thread::yield();
#endif
    }
  }
}
BuiltinPoolMemoryResource::BuiltinPoolMemoryResource(
    PoolPurpose purpose) noexcept
    : purpose_(purpose) {
  large_cache_.store(purpose == PoolPurpose::Persist,
                     std::memory_order_relaxed);
  large_budget_.store(
      purpose == PoolPurpose::Persist ? 128u * kSizeClassMiB : 0,
      std::memory_order_relaxed);
  auto& r = registry();
  std::lock_guard lock(r.mutex);
  registry_next_ = r.head;
  r.head = this;
}
BuiltinPoolMemoryResource::~BuiltinPoolMemoryResource() {
  auto& r = registry();
  std::lock_guard lock(r.mutex);
  auto** link = &r.head;
  while (*link != this) link = &(*link)->registry_next_;
  *link = registry_next_;
  request_trim(true);
}
#if defined(_MSC_VER)
#define MV_POOL_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define MV_POOL_INLINE inline __attribute__((always_inline))
#else
#define MV_POOL_INLINE inline
#endif
MV_POOL_INLINE std::size_t BuiltinPoolMemoryResource::select_shard(
    bool initialize) {
  if (!current_token) {
    if (!initialize || thread_exiting) return kLocalShards;
    ensure_thread_owner();
  }
  const auto index = (current_token - 1) % kLocalShards;
  if (shards_[index].owner_token.load(std::memory_order_acquire) ==
      current_token)
    return index;
  return claim_shard(index);
}
std::size_t BuiltinPoolMemoryResource::claim_shard(std::size_t index) {
  auto& s = shards_[index];
  std::lock_guard lock(s.mutex);
  if (s.owner_life && !s.owner_life->exited.load(std::memory_order_acquire))
    return kLocalShards;
  s.owner_life = thread_owner().life;
  s.owner_token.store(current_token, std::memory_order_release);
  return index;
}
MV_POOL_INLINE void BuiltinPoolMemoryResource::service_trim(Shard& s) noexcept {
  if (s.trim_requested.load(std::memory_order_acquire) &&
      s.trim_requested.exchange(false, std::memory_order_acq_rel))
    release_shard(s);
}
void BuiltinPoolMemoryResource::request_trim(bool force) noexcept {
  for (std::size_t i = 0; i < kShardCount; ++i) {
    auto& s = shards_[i];
    std::lock_guard lock(s.mutex);
    const bool safe =
        force || i == kLocalShards || !s.owner_life ||
        s.owner_token.load(std::memory_order_relaxed) == current_token ||
        s.owner_life->exited.load(std::memory_order_acquire);
    if (safe) {
      s.trim_requested.store(false, std::memory_order_relaxed);
      release_shard(s);
    } else
      s.trim_requested.store(true, std::memory_order_release);
  }
}
void BuiltinPoolMemoryResource::release_shard(Shard& s,
                                              bool large_only) noexcept {
  for (std::size_t i = 0; i < s.lists.size(); ++i) {
    if (large_only &&
        !is_large_payload(payload_at_index(purpose_, static_cast<int>(i / 3))))
      continue;
    while (auto* p = s.lists[i]) {
      s.lists[i] = p->next;
      auto* h = header_of(p);
      s.stats.cached_bytes -= h->raw_bytes;
      s.stats.released_raw += h->raw_bytes;
      if (h->large) s.stats.large_cached_bytes -= h->raw_bytes;
      system_free(h);
    }
    s.list_sizes[i] = 0;
  }
  const auto release = s.credit - s.stats.cached_bytes;
  s.credit -= release;
  credits_.fetch_sub(release, std::memory_order_relaxed);
  release_idle(release);
}
bool BuiltinPoolMemoryResource::reserve_credit(Shard& s,
                                               std::size_t bytes) noexcept {
  if (bytes <= s.credit - s.stats.cached_bytes) return true;
  const auto needed = bytes - (s.credit - s.stats.cached_bytes);
  // 仅 refill 更新共享缓存行；credit 为闲置缓存上限。
  auto amount = &s == &shards_[kLocalShards]
                    ? needed
                    : (std::min)((std::max)(needed, std::size_t{65536}),
                                 2u * kSizeClassMiB - s.credit);
  for (;;) {
    if (charge(credits_, cache_limit_.load(std::memory_order_relaxed),
               amount)) {
      if (charge_idle(amount)) {
        s.credit += amount;
        return true;
      }
      credits_.fetch_sub(amount, std::memory_order_relaxed);
    }
    if (amount == needed) return false;
    amount = needed;
  }
}
void* BuiltinPoolMemoryResource::do_allocate(std::size_t bytes,
                                             std::size_t alignment) {
  if (bytes <= kMaxCachedPayload &&
      (bytes <= kMaxPooledPayload ||
       large_cache_.load(std::memory_order_relaxed)) &&
      alignment <= 64 && is_power_of_two(alignment)) {
    const auto index = rounded_payload_index(purpose_, bytes);
    const int list = index * 3 + (alignment <= 16   ? 0
                                  : alignment == 32 ? 1
                                                    : 2);
    const auto slot = bytes <= 65536 ? select_shard(true) : kLocalShards;
    auto& s = shards_[slot];
    ShardGuard lock(s, slot == kLocalShards);
    service_trim(s);
    if (auto* p = s.lists[list]) {
      s.lists[list] = p->next;
      if (bytes > 65536) --s.list_sizes[list];
      auto* h = header_of(p);
      h->requested_bytes = bytes;
      h->requested_alignment = alignment;
#ifndef NDEBUG
      h->front_canary = kFrontCanary;
      std::memcpy(static_cast<std::byte*>(static_cast<void*>(p)) + bytes,
                  &kBackCanary, sizeof(kBackCanary));
#endif
      s.stats.cached_bytes -= h->raw_bytes;
      if (h->large) s.stats.large_cached_bytes -= h->raw_bytes;
      s.stats.requested_bytes += bytes;
      ++s.stats.allocation_count;
      ++s.stats.cache_hit_count;
      return p;
    }
  }
  return allocate_slow(bytes, alignment);
}
void BuiltinPoolMemoryResource::do_deallocate(void* p, std::size_t bytes,
                                              std::size_t alignment) {
  if (!p) return;
  auto* h = header_of(p);
  if (h->requested_bytes != bytes || h->requested_alignment != alignment)
    std::terminate();
#ifndef NDEBUG
  std::uint64_t tail;
  std::memcpy(&tail, static_cast<std::byte*>(p) + bytes, sizeof(tail));
  if (h->front_canary != kFrontCanary || tail != kBackCanary) std::terminate();
#endif
  if (h->list >= 0) {
    const auto slot = bytes <= 65536 ? select_shard(false) : kLocalShards;
    auto& s = shards_[slot];
    ShardGuard lock(s, slot == kLocalShards);
    service_trim(s);
    const auto cap =
        h->large ? kMaxBlocksPerLargeClass : kMaxBlocksPerSmallClass;
    if (h->raw_bytes <= s.credit - s.stats.cached_bytes &&
        (bytes <= 65536 || s.list_sizes[h->list] < cap) &&
        (!h->large || (large_cache_.load(std::memory_order_relaxed) &&
                       s.stats.large_cached_bytes + h->raw_bytes <=
                           large_budget_.load(std::memory_order_relaxed)))) {
      auto* node = static_cast<FreeNode*>(p);
      node->next = s.lists[h->list];
      s.lists[h->list] = node;
      if (bytes > 65536) ++s.list_sizes[h->list];
      s.stats.freed_bytes += bytes;
      s.stats.cached_bytes += h->raw_bytes;
      if (h->large) s.stats.large_cached_bytes += h->raw_bytes;
      return;
    }
  }
  deallocate_slow(p, bytes, alignment);
}
void* BuiltinPoolMemoryResource::allocate_slow(std::size_t bytes,
                                               std::size_t alignment) {
  const auto l = layout_for(purpose_, bytes, alignment,
                            large_cache_.load(std::memory_order_relaxed));
  const auto shard_index =
      l.payload <= 65536 ? select_shard(true) : kLocalShards;
  auto& s = shards_[shard_index];
  auto finish = [&](void* p) {
    auto* h = header_of(p);
    h->requested_bytes = bytes;
    h->requested_alignment = alignment;
    h->shard = static_cast<std::uint16_t>(shard_index);
#ifndef NDEBUG
    h->front_canary = kFrontCanary;
    std::memcpy(static_cast<std::byte*>(p) + bytes, &kBackCanary,
                sizeof(kBackCanary));
#endif
    s.stats.requested_bytes += bytes;
    ++s.stats.allocation_count;
    return p;
  };
  {
    ShardGuard lock(s, shard_index == kLocalShards);
    service_trim(s);
    if (l.list >= 0 && s.lists[l.list]) {
      auto* p = s.lists[l.list];
      s.lists[l.list] = p->next;
      if (bytes > 65536) --s.list_sizes[l.list];
      auto* h = header_of(p);
      s.stats.cached_bytes -= h->raw_bytes;
      if (h->large) s.stats.large_cached_bytes -= h->raw_bytes;
      ++s.stats.cache_hit_count;
      return finish(p);
    }
  }
  // 通过共享缓存或已退出线程缓存转移批次/块，其他线程不能检查或修改仍存活的外部拥有者。
  if (l.list >= 0 && shard_index != kLocalShards &&
      credits_.load(std::memory_order_relaxed) != 0) {
    for (std::size_t n = 0; n <= kLocalShards; ++n) {
      const auto donor_index = n == 0 ? kLocalShards : n - 1;
      if (donor_index == shard_index) continue;
      auto& donor = shards_[donor_index];
      if (donor.stats.cached_bytes == 0) continue;
      void* p = nullptr;
      {
        std::lock_guard lock(donor.mutex);
        if (donor_index != kLocalShards && donor.owner_life &&
            !donor.owner_life->exited.load(std::memory_order_acquire))
          continue;
        service_trim(donor);
        if (auto* node = donor.lists[l.list]) {
          donor.lists[l.list] = node->next;
          // 此处仅转移 TLS 尺寸块，不使用数量配额。
          p = node;
          donor.stats.cached_bytes -= header_of(p)->raw_bytes;
          // 原始存储仍归此资源所有，仅缓存位置移动。
        }
      }
      if (p) {
        ++s.stats.cache_hit_count;
        return finish(p);
      }
    }
  }
  bool virtual_memory = false;
#ifdef _WIN32
  virtual_memory = l.total >= kSizeClassMiB && l.alignment <= 65536;
#endif
  auto acquire_raw = [&]() -> void* {
#ifdef _WIN32
    if (virtual_memory) {
      if (auto* p = VirtualAlloc(nullptr, l.total, MEM_RESERVE | MEM_COMMIT,
                                 PAGE_READWRITE))
        return p;
      throw std::bad_alloc();
    }
#endif
    return std::pmr::new_delete_resource()->allocate(l.total, l.alignment);
  };
  void* base;
  try {
    base = acquire_raw();
  } catch (const std::bad_alloc&) {
    trim();
    base = acquire_raw();
  }
  auto* p = static_cast<std::byte*>(base) + l.offset;
  auto* h = std::construct_at(header_of(p));
  h->base = base;
  h->raw_bytes = l.total;
  h->raw_alignment = l.alignment;
  h->list = l.list;
  h->large = is_large_payload(l.payload);
  h->virtual_memory = virtual_memory;
  ShardGuard lock(s, shard_index == kLocalShards);
  s.stats.acquired_raw += l.total;
  return finish(p);
}
void BuiltinPoolMemoryResource::deallocate_slow(void* p, std::size_t bytes,
                                                std::size_t alignment) {
  if (!p) return;
  auto* h = header_of(p);
  if (h->requested_bytes != bytes || h->requested_alignment != alignment)
    std::terminate();
#ifndef NDEBUG
  std::uint64_t tail;
  std::memcpy(&tail, static_cast<std::byte*>(p) + bytes, sizeof(tail));
  if (h->front_canary != kFrontCanary || tail != kBackCanary) std::terminate();
#endif
  const auto shard_index = bytes <= 65536 ? select_shard(false) : kLocalShards;
  auto& s = shards_[shard_index];
  {
    ShardGuard lock(s, shard_index == kLocalShards);
    service_trim(s);
    s.stats.freed_bytes += bytes;
    const auto local_limit = shard_index == kLocalShards
                                 ? cache_limit_.load(std::memory_order_relaxed)
                                 : 2u * kSizeClassMiB;
    bool allow = h->list >= 0 &&
                 (bytes <= 65536 || s.list_sizes[h->list] <
                                        (h->large ? kMaxBlocksPerLargeClass
                                                  : kMaxBlocksPerSmallClass)) &&
                 h->raw_bytes <= local_limit &&
                 s.stats.cached_bytes <= local_limit - h->raw_bytes;
    if (allow && h->large) {
      const auto budget = large_budget_.load(std::memory_order_relaxed);
      allow = large_cache_.load(std::memory_order_relaxed) &&
              h->raw_bytes <= budget &&
              s.stats.large_cached_bytes <= budget - h->raw_bytes;
    }
    if (allow && reserve_credit(s, h->raw_bytes)) {
      auto* node = static_cast<FreeNode*>(p);
      node->next = s.lists[h->list];
      s.lists[h->list] = node;
      if (bytes > 65536) ++s.list_sizes[h->list];
      s.stats.cached_bytes += h->raw_bytes;
      if (h->large) s.stats.large_cached_bytes += h->raw_bytes;
      return;
    }
    s.stats.released_raw += h->raw_bytes;
  }
  system_free(h);
}
bool BuiltinPoolMemoryResource::do_is_equal(
    const std::pmr::memory_resource& other) const noexcept {
  return this == &other;
}
PoolPurpose BuiltinPoolMemoryResource::purpose() const noexcept {
  return purpose_;
}
void BuiltinPoolMemoryResource::trim() noexcept { request_trim(); }
void BuiltinPoolMemoryResource::set_cache_limit(std::size_t bytes) noexcept {
  const auto old = cache_limit_.exchange(bytes, std::memory_order_relaxed);
  if (bytes < old) request_trim();
}
std::size_t BuiltinPoolMemoryResource::cache_limit() const noexcept {
  return cache_limit_.load(std::memory_order_relaxed);
}
std::size_t BuiltinPoolMemoryResource::cache_budget_reserved_bytes()
    const noexcept {
  return credits_.load(std::memory_order_relaxed);
}
void BuiltinPoolMemoryResource::set_large_block_cache(bool enabled) noexcept {
  auto& s = shards_[kLocalShards];
  std::lock_guard lock(s.mutex);
  large_cache_.store(enabled, std::memory_order_relaxed);
  if (!enabled) release_shard(s, true);
}
bool BuiltinPoolMemoryResource::large_block_cache() const noexcept {
  return large_cache_.load(std::memory_order_relaxed);
}
void BuiltinPoolMemoryResource::set_large_block_budget(
    std::size_t bytes) noexcept {
  auto& s = shards_[kLocalShards];
  std::lock_guard lock(s.mutex);
  large_budget_.store(bytes, std::memory_order_relaxed);
  if (s.stats.large_cached_bytes > bytes) release_shard(s, true);
}
std::size_t BuiltinPoolMemoryResource::large_block_budget() const noexcept {
  return large_budget_.load(std::memory_order_relaxed);
}
MemoryPoolStats BuiltinPoolMemoryResource::stats() const noexcept {
  MemoryPoolStats result;
  std::size_t acquired = 0, released = 0, requested = 0, freed = 0;
  for (const auto& s : shards_) {
    acquired += s.stats.acquired_raw;
    released += s.stats.released_raw;
    requested += s.stats.requested_bytes;
    freed += s.stats.freed_bytes;
    result.cached_bytes += s.stats.cached_bytes;
    result.large_cached_bytes += s.stats.large_cached_bytes;
    result.allocation_count += s.stats.allocation_count;
    result.cache_hit_count += s.stats.cache_hit_count;
  }
  // 并发观测可能跨越转移时刻；静止后的总数精确。
  result.live_bytes = requested - (std::min)(requested, freed);
  result.reserved_bytes = (std::max)(result.cached_bytes,
                                     acquired - (std::min)(acquired, released));
  auto peak = observed_peak_.load(std::memory_order_relaxed);
  while (peak < result.live_bytes &&
         !observed_peak_.compare_exchange_weak(peak, result.live_bytes,
                                               std::memory_order_relaxed)) {
  }
  result.observed_peak_live_bytes = (std::max)(peak, result.live_bytes);
  return result;
}
namespace {
struct PoolCacheShutdown {
  BuiltinPoolMemoryResource* pool;
  ~PoolCacheShutdown() { pool->set_cache_limit(0); }
};

template <PoolPurpose purpose>
BuiltinPoolMemoryResource* process_pool() noexcept {
  alignas(BuiltinPoolMemoryResource) static std::byte
      storage[sizeof(BuiltinPoolMemoryResource)];
  static auto* pool = std::construct_at(
      reinterpret_cast<BuiltinPoolMemoryResource*>(storage), purpose);
  // 关闭时清空缓存，但资源/vtable/mutex 保持存活以支持后续全局拥有者释放；其释放绕过已禁用缓存。
  static PoolCacheShutdown shutdown{pool};
  return pool;
}
}  // namespace

BuiltinPoolMemoryResource* builtin_persist_pool() noexcept {
  return process_pool<PoolPurpose::Persist>();
}

BuiltinPoolMemoryResource* builtin_scratch_pool() noexcept {
  return process_pool<PoolPurpose::Scratch>();
}

BuiltinPoolMemoryResource* builtin_pool_memory_resource() noexcept {
  return builtin_scratch_pool();
}

void drain_idle_caches() noexcept {
  builtin_persist_pool()->trim();
  builtin_scratch_pool()->trim();
}

void reclaim_thread_caches(std::size_t token) noexcept {
  auto& r = registry();
  std::lock_guard lock(r.mutex);
  for (auto* p = r.head; p; p = p->registry_next_)
    for (auto& s : p->shards_) {
      if (s.owner_token.load(std::memory_order_acquire) != token) continue;
      std::lock_guard shard_lock(s.mutex);
      if (s.owner_token.load(std::memory_order_relaxed) == token &&
          s.owner_life->exited.load(std::memory_order_acquire)) {
        s.trim_requested.store(false, std::memory_order_relaxed);
        p->release_shard(s);
      }
    }
}
void set_global_cache_limit(std::size_t bytes) noexcept {
  auto& r = registry();
  std::lock_guard lock(r.mutex);
  const auto old = r.limit.exchange(bytes, std::memory_order_relaxed);
  if (bytes < old)
    for (auto* p = r.head; p; p = p->registry_next_) p->request_trim();
}
std::size_t global_cache_limit() noexcept {
  return registry().limit.load(std::memory_order_relaxed);
}
std::size_t global_cache_budget_reserved_bytes() noexcept {
  return registry().credit.load(std::memory_order_relaxed);
}
std::size_t global_cached_bytes() noexcept {
  auto& r = registry();
  std::lock_guard lock(r.mutex);
  std::size_t sum = 0;
  for (auto* p = r.head; p; p = p->registry_next_)
    for (const auto& s : p->shards_) sum += s.stats.cached_bytes;
  return sum;
}
}  // namespace mv::runtime
