#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <calibration/solver/problem.hpp>
#include <calibration/numeric/linear_backend.hpp>

namespace mv::calibration::solver {

enum class SolverTermination {
  Converged,
  MaxIterations,
  NumericalFailure,
  InvalidProblem,
};

struct SolverOptions {
  std::size_t max_iterations{100};
  double function_tolerance{1e-12};
  double gradient_tolerance{1e-12};
  double parameter_tolerance{1e-12};
  double initial_damping{1e-3};
  numeric::LinearBackendKind linear_backend{numeric::default_backend_kind()};
};

struct SolverSummary {
  SolverTermination termination{SolverTermination::InvalidProblem};
  std::size_t iterations{};
  std::size_t successful_steps{};
  std::size_t unsuccessful_steps{};
  std::size_t parameter_blocks{};
  std::size_t effective_parameters{};
  std::size_t residual_blocks{};
  std::size_t residuals{};
  double initial_cost{};
  double final_cost{};
  double gradient_max_norm{};
  std::string message;
};

class CalibrationOptimizerBackend {
 public:
  virtual ~CalibrationOptimizerBackend() = default;
  virtual core::Status solve(SolverProblem& problem,
                             const SolverOptions& options,
                             SolverSummary* summary) = 0;
};

enum class OptimizerBackendKind {
  BuiltinLm,
};

[[nodiscard]] std::unique_ptr<CalibrationOptimizerBackend>
make_optimizer_backend(OptimizerBackendKind kind);

}  // namespace mv::calibration::solver
