#pragma once

#include <calibration/solver/backend.hpp>

namespace mv::calibration::solver {

class BuiltinLmBackend final : public CalibrationOptimizerBackend {
 public:
  core::Status solve(SolverProblem& problem,
                     const SolverOptions& options,
                     SolverSummary* summary) override;
};

}  // namespace mv::calibration::solver
