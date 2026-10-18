#pragma once

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include "runtime/parallel/executor.hpp"

namespace mv::runtime {

namespace detail {
void parallel_for_erased(std::size_t begin, std::size_t end,
                         std::size_t grain_size, ParallelBodyView body);
}

template <class Body>
void parallel_for(std::size_t begin, std::size_t end, std::size_t grain_size,
                  Body&& body) {
  using Callable = std::remove_reference_t<Body>;
  auto invoke = [](void* context, IndexRange range) {
    (*static_cast<Callable*>(context))(range);
  };
  detail::parallel_for_erased(
      begin, end, grain_size,
      {const_cast<void*>(static_cast<const void*>(std::addressof(body))), invoke});
}

[[nodiscard]] std::size_t max_parallelism() noexcept;
[[nodiscard]] std::size_t configured_parallelism() noexcept;
// 进程级 worker 预算，零使用捕获的可用 CPU 数；builtin 在池启动时冻结，serial 仅接受零或一。
void set_parallelism(std::size_t count);
[[nodiscard]] bool parallel_executor_started() noexcept;
[[nodiscard]] std::size_t started_parallel_worker_count() noexcept;
[[nodiscard]] std::size_t current_worker_index() noexcept;
inline constexpr std::size_t kNoWorkerIndex =
    static_cast<std::size_t>(-1);

}  // namespace mv::runtime
