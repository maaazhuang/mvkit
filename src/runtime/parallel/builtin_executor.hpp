#pragma once

#include <cstddef>
#include <memory>

#include "runtime/parallel/executor.hpp"

namespace mv::runtime {

class BuiltinExecutor final : public Executor {
 public:
  BuiltinExecutor();
  ~BuiltinExecutor() override;

  BuiltinExecutor(const BuiltinExecutor&) = delete;
  BuiltinExecutor& operator=(const BuiltinExecutor&) = delete;

  void execute(IndexRange range, std::size_t grain_size,
               ParallelBodyView body) override;
  [[nodiscard]] std::size_t max_parallelism() const noexcept override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] Executor& builtin_executor();
[[nodiscard]] std::size_t parallel_thread_count() noexcept;
[[nodiscard]] bool builtin_executor_started() noexcept;
[[nodiscard]] std::size_t builtin_started_worker_count() noexcept;
void set_parallel_thread_count(std::size_t count);
// 调用线程执行仍保持 builtin 显式 grain 边界及嵌套规则。
void execute_builtin_inline(IndexRange range, std::size_t grain,
                            ParallelBodyView body);

}  // namespace mv::runtime
