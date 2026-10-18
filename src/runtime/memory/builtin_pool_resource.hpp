#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <memory_resource>
#include <mutex>

#include "runtime/memory/size_class.hpp"

namespace mv::runtime {

struct PoolThreadLife;

struct MemoryPoolStats {
  std::size_t reserved_bytes = 0;
  std::size_t cached_bytes = 0;
  std::size_t live_bytes = 0;
  // stats() 采样的高水位，不是分配时精确峰值。
  std::size_t observed_peak_live_bytes = 0;
  std::size_t allocation_count = 0;
  std::size_t cache_hit_count = 0;
  std::size_t large_cached_bytes = 0;
};

class BuiltinPoolMemoryResource final : public std::pmr::memory_resource {
 public:
  explicit BuiltinPoolMemoryResource(PoolPurpose purpose) noexcept;
  ~BuiltinPoolMemoryResource() override;

  BuiltinPoolMemoryResource(const BuiltinPoolMemoryResource&) = delete;
  BuiltinPoolMemoryResource& operator=(const BuiltinPoolMemoryResource&) =
      delete;

  void set_cache_limit(std::size_t bytes) noexcept;
  [[nodiscard]] std::size_t cache_limit() const noexcept;
  // 准入额度包含未使用预留，不表示已分配内存。
  [[nodiscard]] std::size_t cache_budget_reserved_bytes() const noexcept;
  void set_large_block_cache(bool enabled) noexcept;
  [[nodiscard]] bool large_block_cache() const noexcept;
  void set_large_block_budget(std::size_t bytes) noexcept;
  [[nodiscard]] std::size_t large_block_budget() const noexcept;
  void trim() noexcept;
  void drain_idle_cache() noexcept { trim(); }
  [[nodiscard]] MemoryPoolStats stats() const noexcept;
  [[nodiscard]] PoolPurpose purpose() const noexcept;

 private:
  void* do_allocate(std::size_t bytes, std::size_t alignment) override;
  void* allocate_slow(std::size_t bytes, std::size_t alignment);
  void deallocate_slow(void* pointer, std::size_t bytes, std::size_t alignment);
  void do_deallocate(void* pointer, std::size_t bytes,
                     std::size_t alignment) override;
  bool do_is_equal(
      const std::pmr::memory_resource& other) const noexcept override;

  // TLS 仅持有线程生命周期 token，不持有资源/缓存指针；资源拥有全部分片，线程退出/复用不会遗留拥有者。
  static constexpr std::size_t kLocalShards = 32;
  static constexpr std::size_t kShardCount = kLocalShards + 1;
  struct SpinMutex {
    std::atomic_flag flag = ATOMIC_FLAG_INIT;
    void lock() noexcept;
    void unlock() noexcept { flag.clear(std::memory_order_release); }
  };
  struct FreeNode {
    FreeNode* next;
  };
  // 单写者计数器：TLS 拥有者写入，观测者读取；共享分片写入串行化，不需要逐对象全局原子 RMW。
  struct OwnerCounter {
    std::atomic<std::size_t> value{0};
    operator std::size_t() const noexcept {
      return value.load(std::memory_order_relaxed);
    }
    OwnerCounter& operator+=(std::size_t n) noexcept {
      value.store(value.load(std::memory_order_relaxed) + n,
                  std::memory_order_relaxed);
      return *this;
    }
    OwnerCounter& operator-=(std::size_t n) noexcept {
      value.store(value.load(std::memory_order_relaxed) - n,
                  std::memory_order_relaxed);
      return *this;
    }
    OwnerCounter& operator++() noexcept { return *this += 1; }
  };
  struct ShardStats {
    OwnerCounter acquired_raw, released_raw, requested_bytes, freed_bytes;
    OwnerCounter cached_bytes, allocation_count, cache_hit_count,
        large_cached_bytes;
  };
  struct alignas(64) Shard {
    mutable SpinMutex mutex;
    std::atomic<std::size_t> owner_token{0};
    std::shared_ptr<PoolThreadLife>
        owner_life;  // 受 mutex 保护，不存入块头。
    std::atomic<bool> trim_requested{false};
    std::array<FreeNode*, kMaxSizeClassLists> lists{};
    std::array<std::uint32_t, kMaxSizeClassLists> list_sizes{};
    ShardStats stats;
    std::size_t credit = 0;
    std::byte cache_color[64]{};
  };
  struct ShardGuard {
    Shard& shard;
    bool locked;
    ShardGuard(Shard& s, bool shared) noexcept : shard(s), locked(shared) {
      if (locked) shard.mutex.lock();
    }
    ~ShardGuard() {
      if (locked) shard.mutex.unlock();
    }
  };
  std::size_t select_shard(bool initialize);
  std::size_t claim_shard(std::size_t index);
  void service_trim(Shard& shard) noexcept;
  void request_trim(bool force = false) noexcept;
  void release_shard(Shard& shard, bool large_only = false) noexcept;
  bool reserve_credit(Shard& shard, std::size_t bytes) noexcept;
  PoolPurpose purpose_;
  mutable std::array<Shard, kShardCount> shards_{};
  std::atomic<std::size_t> cache_limit_{128u * kSizeClassMiB};
  std::atomic<std::size_t> credits_{0};
  mutable std::atomic<std::size_t> observed_peak_{0};
  std::atomic<std::size_t> large_budget_{0};
  std::atomic<bool> large_cache_{false};
  // 侵入式登记表无需堆分配，支持后续释放的拥有者。
  BuiltinPoolMemoryResource* registry_next_ = nullptr;
  friend void set_global_cache_limit(std::size_t) noexcept;
  friend void reclaim_thread_caches(std::size_t) noexcept;
  friend std::size_t global_cached_bytes() noexcept;
};

[[nodiscard]] BuiltinPoolMemoryResource* builtin_persist_pool() noexcept;
[[nodiscard]] BuiltinPoolMemoryResource* builtin_scratch_pool() noexcept;
[[nodiscard]] BuiltinPoolMemoryResource*
builtin_pool_memory_resource() noexcept;

void drain_idle_caches() noexcept;

// 包含全部资源实例。降低限制请求 TLS 回收，活跃拥有者在下次分配调用处理。
// trim() 可直接回收调用线程、已退出线程及共享缓存；闲置活线程可保留已预算缓存至安全点。
// 统计为采样观测，静止时精确；不影响有效分配。
void set_global_cache_limit(std::size_t bytes) noexcept;
[[nodiscard]] std::size_t global_cache_limit() noexcept;
[[nodiscard]] std::size_t global_cached_bytes() noexcept;
[[nodiscard]] std::size_t global_cache_budget_reserved_bytes() noexcept;

}  // namespace mv::runtime
