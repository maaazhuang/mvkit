#include "runtime/parallel/parallel_state.hpp"

#include <atomic>
#include <array>
#include <memory>

namespace mv::runtime::detail {

thread_local unsigned parallel_depth = 0;
thread_local std::size_t worker_index = kNoWorkerIndex;

void set_worker_index(std::size_t index) noexcept { worker_index = index; }

std::size_t get_worker_index() noexcept { return worker_index; }

namespace {

struct alignas(64) Counters {
  std::atomic<bool> owned{false};
  std::atomic<std::uint64_t> queued{0}, run{0}, wait{0};
};
struct Registry {
  std::array<Counters, 128> slots;
  Counters overflow;
};
Registry& registry() noexcept {
  // 进程生命周期，包含晚期静态析构及 TLS 清理。
  alignas(Registry) static std::byte storage[sizeof(Registry)];
  static auto* value = std::construct_at(reinterpret_cast<Registry*>(storage));
  return *value;
}
thread_local bool exiting = false;
struct ThreadCounters {
  Counters* value = nullptr;
  ThreadCounters() noexcept {
    for (auto& slot : registry().slots) {
      bool expected = false;
      if (slot.owned.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        value = &slot;
        break;
      }
    }
  }
  ~ThreadCounters() {
    exiting = true;
    if (value) value->owned.store(false, std::memory_order_release);
  }
};

void add(std::atomic<std::uint64_t> Counters::* field, std::uint64_t count) noexcept {
  if (count == 0) return;
  // 后续 TLS 析构仍可能提交工作，不能复用已释放槽位。
  if (exiting) {
    (registry().overflow.*field).fetch_add(count, std::memory_order_relaxed);
    return;
  }
  thread_local ThreadCounters local;
  if (local.value) {
    auto& counter = local.value->*field;
    counter.store(counter.load(std::memory_order_relaxed) + count,
                  std::memory_order_relaxed);
  } else {
    (registry().overflow.*field).fetch_add(count, std::memory_order_relaxed);
  }
}
std::atomic<void (*)(const char*, const char*)> diagnostic_sink{nullptr};

}  // namespace

void record_batches_queued(std::size_t count) noexcept {
  add(&Counters::queued, count);
}

void record_batches_run(std::size_t count) noexcept {
  add(&Counters::run, count);
}

void record_wait_ns(std::uint64_t nanoseconds) noexcept {
  add(&Counters::wait, nanoseconds);
}

SchedulerSnapshot scheduler_snapshot() noexcept {
  SchedulerSnapshot snapshot;
  const auto read = [&](const Counters& slot) {
    snapshot.batches_queued += slot.queued.load(std::memory_order_relaxed);
    snapshot.batches_run += slot.run.load(std::memory_order_relaxed);
    snapshot.wait_ns += slot.wait.load(std::memory_order_relaxed);
  };
  for (const auto& slot : registry().slots) read(slot);
  read(registry().overflow);
  return snapshot;
}

void set_scheduler_diagnostic_sink(
    void (*sink)(const char*, const char*)) noexcept {
  diagnostic_sink.store(sink, std::memory_order_release);
}

void emit_scheduler_diagnostic(const char* event) noexcept {
  auto* sink = diagnostic_sink.load(std::memory_order_acquire);
  if (sink != nullptr) {
    // 诊断不能终止进程或使成功工作失败。
    try { sink("parallel", event); } catch (...) {}
  }
}

}  // namespace mv::runtime::detail
