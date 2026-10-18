#pragma once

#include "runtime/parallel/executor.hpp"

namespace mv::runtime {

class SerialExecutor final : public Executor {
 public:
  void execute(IndexRange range, std::size_t grain_size,
               ParallelBodyView body) override;
  [[nodiscard]] std::size_t max_parallelism() const noexcept override;
};

[[nodiscard]] Executor& serial_executor() noexcept;

}  // namespace mv::runtime
