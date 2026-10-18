#include "runtime/parallel/serial_executor.hpp"

#include "runtime/parallel/parallel_state.hpp"

namespace mv::runtime {

void SerialExecutor::execute(IndexRange range, std::size_t,
                             ParallelBodyView body) {
  if (range.begin >= range.end) return;
  detail::record_batches_queued(1);
  try {
    body.invoke(body.context, range);
  } catch (...) {
    detail::record_batches_run(1);
    throw;
  }
  detail::record_batches_run(1);
}

std::size_t SerialExecutor::max_parallelism() const noexcept { return 1; }

Executor& serial_executor() noexcept {
  static SerialExecutor executor;
  return executor;
}

}  // namespace mv::runtime
