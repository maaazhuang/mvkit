#pragma once

#include <cstddef>
#include <cstdint>

namespace mv::runtime::detail {

inline constexpr std::size_t kNoWorkerIndex =
    static_cast<std::size_t>(-1);

void set_worker_index(std::size_t index) noexcept;
[[nodiscard]] std::size_t get_worker_index() noexcept;

extern thread_local unsigned parallel_depth;

class ParallelDepthGuard {
 public:
  ParallelDepthGuard() noexcept { ++parallel_depth; }
  ~ParallelDepthGuard() { --parallel_depth; }

  ParallelDepthGuard(const ParallelDepthGuard&) = delete;
  ParallelDepthGuard& operator=(const ParallelDepthGuard&) = delete;
};

void record_batches_queued(std::size_t count) noexcept;
void record_batches_run(std::size_t count) noexcept;
void record_wait_ns(std::uint64_t nanoseconds) noexcept;

struct SchedulerSnapshot {
  std::uint64_t batches_queued = 0;
  std::uint64_t batches_run = 0;
  std::uint64_t wait_ns = 0;
};

// 累计计数跨线程退出保留，静止时精确；并发读取采样独立分片，不是单一线性化快照。
[[nodiscard]] SchedulerSnapshot scheduler_snapshot() noexcept;
void set_scheduler_diagnostic_sink(void (*sink)(const char*, const char*)) noexcept;
void emit_scheduler_diagnostic(const char* event) noexcept;

}  // namespace mv::runtime::detail
