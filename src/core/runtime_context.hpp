#pragma once

#include <core/tuple.hpp>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace mv::core {

enum class Neighborhood : int { Four = 4, Eight = 8 };

struct RuntimeContext {
  Neighborhood neighborhood{Neighborhood::Eight};
  std::size_t max_connection{0};
};

[[nodiscard]] RuntimeContext current_runtime_context() noexcept;
void initialize_runtime_context() noexcept;
void set_system(std::string_view system_parameter, const Tuple& value);
void set_system(std::string_view system_parameter, const Value& value);
[[nodiscard]] Tuple get_system(std::string_view query);

}  // namespace mv::core
