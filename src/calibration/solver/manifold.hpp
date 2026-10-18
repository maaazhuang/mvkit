#pragma once

#include <cstddef>
#include <span>

#include <calibration/solver/problem.hpp>

namespace mv::calibration::solver {

[[nodiscard]] std::size_t expected_tangent_size(const ParameterBlock& block) noexcept;
[[nodiscard]] core::Status validate_parameter_block(const ParameterBlock& block) noexcept;
[[nodiscard]] core::Status apply_local_delta(ParameterBlock& block, std::span<const double> delta) noexcept;

}  // namespace mv::calibration::solver
