#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>

#include "runtime/cpu/cpu_features.hpp"
#include "runtime/memory/scratch_arena.hpp"
#include "runtime/parallel/parallel_for.hpp"

namespace mv::runtime {

enum class StopReason : unsigned char { None, Cancelled, DeadlineExceeded };

class ExecutionContext {
 public:
  using clock_type = std::chrono::steady_clock;
  using time_point = clock_type::time_point;
  using ClockFn = time_point (*)();

  ExecutionContext();
  explicit ExecutionContext(time_point deadline);
  ExecutionContext(time_point deadline, ClockFn clock);

  ExecutionContext(const ExecutionContext&) = delete;
  ExecutionContext& operator=(const ExecutionContext&) = delete;
  ExecutionContext(ExecutionContext&&) = delete;
  ExecutionContext& operator=(ExecutionContext&&) = delete;

  [[nodiscard]] std::pmr::memory_resource* scratch_resource() noexcept;
  void reset_scratch() noexcept;

  [[nodiscard]] bool has_deadline() const noexcept { return has_deadline_; }
  [[nodiscard]] time_point deadline() const noexcept { return deadline_; }
  void request_cancel() noexcept;
  [[nodiscard]] bool cancelled() const noexcept;
  [[nodiscard]] StopReason stop_reason() const;
  // 协作安全点：返回当前停止原因，由算法映射到既有错误。运行时不抛异常。
  [[nodiscard]] StopReason poll() const;

  [[nodiscard]] std::unique_ptr<ExecutionContext> nest() const;
  [[nodiscard]] std::unique_ptr<ExecutionContext> nest(
      time_point child_deadline) const;

  [[nodiscard]] const CpuFeatures& cpu() const noexcept;
  [[nodiscard]] const char* selected_isa() const noexcept;

  template <class Body>
  static void for_each_partition(std::size_t count, std::size_t work,
                                 std::size_t parallel_threshold, Body&& body,
                                 std::size_t max_partitions = 0) {
    if (count == 0) return;
    if (work < parallel_threshold) {
      body(IndexRange{0, count});
      return;
    }
    // 已知成本的算法可限制均匀轻任务；零保留自适应划分，该预算不调整线程池大小。
    const auto grain = max_partitions
        ? 1 + (count - 1) / (std::min)(saturated_work(max_parallelism(), 4), max_partitions) : 0;
    parallel_for(0, count, grain, std::forward<Body>(body));
  }

  [[nodiscard]] static std::size_t saturated_work(
      std::size_t left, std::size_t right) noexcept {
    if (left != 0 &&
        right > (std::numeric_limits<std::size_t>::max)() / left) {
      return (std::numeric_limits<std::size_t>::max)();
    }
    return left * right;
  }

  static time_point default_clock();

 private:
  explicit ExecutionContext(std::shared_ptr<std::atomic<bool>> cancelled,
                            time_point deadline, bool has_deadline,
                            ClockFn clock);

  [[nodiscard]] time_point now() const;

  ScratchArena scratch_;
  std::shared_ptr<std::atomic<bool>> cancelled_;
  time_point deadline_{};
  bool has_deadline_ = false;
  ClockFn clock_ = &ExecutionContext::default_clock;
};

}  // namespace mv::runtime
