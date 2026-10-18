#pragma once

#include <cstddef>
#include <utility>

#include <runtime/execution/execution_context.hpp>

namespace mv::imageproc {

class ExecutionContext {
 public:
  static constexpr std::size_t kArithmeticParallelWork = 1024u * 1024u;
  static constexpr std::size_t kFilterParallelWork = 1024u * 1024u;

  ExecutionContext() = default;

  ExecutionContext(const ExecutionContext&) = delete;
  ExecutionContext& operator=(const ExecutionContext&) = delete;

  [[nodiscard]] std::pmr::memory_resource* scratch_resource() noexcept {
    return runtime_.scratch_resource();
  }
  void reset_scratch() noexcept { runtime_.reset_scratch(); }
  [[nodiscard]] mv::runtime::ExecutionContext& runtime() noexcept {
    return runtime_;
  }
  [[nodiscard]] const mv::runtime::ExecutionContext& runtime() const noexcept {
    return runtime_;
  }

  template <class Body>
  static void for_each_partition(std::size_t count, std::size_t work,
                                 std::size_t parallel_threshold, Body&& body,
                                 std::size_t max_partitions = 0) {
    mv::runtime::ExecutionContext::for_each_partition(
        count, work, parallel_threshold, std::forward<Body>(body), max_partitions);
  }

  [[nodiscard]] static std::size_t saturated_work(
      std::size_t left, std::size_t right) noexcept {
    return mv::runtime::ExecutionContext::saturated_work(left, right);
  }

 private:
  mv::runtime::ExecutionContext runtime_;
};

}  // namespace mv::imageproc
