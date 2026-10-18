#pragma once

#include <memory_resource>

namespace mv::runtime {

[[nodiscard]] std::pmr::memory_resource* system_memory_resource() noexcept;

}  // namespace mv::runtime
