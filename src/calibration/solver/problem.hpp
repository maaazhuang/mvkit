#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <core/status.hpp>

namespace mv::calibration::solver {

using ParameterBlockId = std::uint32_t;
using ResidualBlockId = std::uint32_t;

enum class ManifoldKind {
  Euclidean,
  UnitQuaternion,
  PoseSE3,
};

struct ParameterBlock {
  ParameterBlockId id{};
  std::vector<double> values;
  std::size_t tangent_size{};
  ManifoldKind manifold{ManifoldKind::Euclidean};
  bool constant{false};
  std::vector<std::optional<double>> lower_bounds;
  std::vector<std::optional<double>> upper_bounds;
};

struct ParameterBlockView {
  ParameterBlockId id{};
  std::span<const double> values;
  std::size_t tangent_size{};
  ManifoldKind manifold{ManifoldKind::Euclidean};
};

struct JacobianView {
  std::span<double> values;
  std::size_t rows{};
  std::size_t cols{};

  double& operator()(std::size_t row, std::size_t col) {
    return values[row * cols + col];
  }
  const double& operator()(std::size_t row, std::size_t col) const {
    return values[row * cols + col];
  }
};

class ResidualFunction {
 public:
  virtual ~ResidualFunction() = default;
  [[nodiscard]] virtual std::size_t residual_size() const noexcept = 0;
  [[nodiscard]] virtual std::span<const ParameterBlockId> parameter_blocks() const noexcept = 0;
  virtual core::Status evaluate(std::span<const ParameterBlockView> parameters,
                                std::span<double> residuals,
                                std::span<JacobianView> local_jacobians) const = 0;
};

struct ResidualBlock {
  ResidualBlockId id{};
  std::unique_ptr<ResidualFunction> function;
};

struct SolverProblem {
  std::vector<ParameterBlock> parameter_blocks;
  std::vector<ResidualBlock> residual_blocks;
};

}  // namespace mv::calibration::solver
