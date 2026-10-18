#pragma once

#include <vector>

#include <calibration/numeric/linear_backend.hpp>
#include <core/geometry_types.hpp>

namespace mv::calibration {

Pose3d solve_ax_xb(
    const std::vector<Pose3d>& a,
    const std::vector<Pose3d>& b,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind());

}  // namespace mv::calibration
