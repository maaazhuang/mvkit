#include "runtime/parallel/parallel_for.hpp"

#include <mvkit/runtime/generated_backend_config.hpp>
#include <stdexcept>

#if MVKIT_PARALLEL_BACKEND_BUILTIN
#include "runtime/parallel/builtin_executor.hpp"
#endif
#include "runtime/parallel/parallel_state.hpp"
#include "runtime/parallel/serial_executor.hpp"

namespace mv::runtime {
namespace detail {

void parallel_for_erased(std::size_t begin, std::size_t end,
                         std::size_t grain_size, ParallelBodyView body) {
  if (begin >= end) return;
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  if (parallel_depth != 0) {
    serial_executor().execute({begin, end}, grain_size, body);
    return;
  }
  if (end - begin == 1 || grain_size >= end - begin ||
      parallel_thread_count() == 1) {
    execute_builtin_inline({begin, end}, grain_size, body);
    return;
  }
  builtin_executor().execute({begin, end}, grain_size, body);
#else
  serial_executor().execute({begin, end}, grain_size, body);
#endif
}

}  // namespace detail

std::size_t max_parallelism() noexcept {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  return parallel_thread_count();
#else
  return serial_executor().max_parallelism();
#endif
}

std::size_t current_worker_index() noexcept {
  return detail::get_worker_index();
}

std::size_t configured_parallelism() noexcept {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  return parallel_thread_count();
#else
  return 1;
#endif
}

void set_parallelism(std::size_t count) {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  set_parallel_thread_count(count);
#else
  if (count > 1) throw std::invalid_argument("serial backend accepts only zero or one worker");
#endif
}

bool parallel_executor_started() noexcept {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  return builtin_executor_started();
#else
  return false;
#endif
}

std::size_t started_parallel_worker_count() noexcept {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  return builtin_started_worker_count();
#else
  return 0;
#endif
}

}  // namespace mv::runtime
