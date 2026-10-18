#pragma once

#include <span>

#include <calibration/calibration/residual.hpp>
#include <calibration/model.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <calibration/solver/problem.hpp>

namespace mv::calibration {

void compute_calibration_uncertainty(const solver::SolverProblem& problem,
                                     std::span<const IntrinsicBinding> camera_bindings,
                                     CalibrationResult* result,
                                     numeric::LinearBackendKind backend_kind =
                                         numeric::default_backend_kind());

}  // namespace mv::calibration
