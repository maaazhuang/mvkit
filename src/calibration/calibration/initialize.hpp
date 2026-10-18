#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <calibration/camera_parameters.hpp>
#include <calibration/model.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <core/geometry_types.hpp>

namespace mv::calibration {

[[nodiscard]] CameraParameters initialize_perspective_intrinsics_planar(
    const CameraParameters& initial,
    const CalibrationDataModel& data,
    const std::set<std::string>& excluded_settings,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind());

[[nodiscard]] CameraParameters initialize_telecentric_intrinsics_planar(
    const CameraParameters& initial,
    const CalibrationDataModel& data,
    const std::set<std::string>& excluded_settings,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind());

[[nodiscard]] Pose3d estimate_planar_pose_telecentric(
    const CameraParameters& camera,
    const std::vector<Point3d>& object_points,
    const std::vector<double>& rows,
    const std::vector<double>& columns,
    const std::vector<std::int64_t>& indices,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind());

}  // namespace mv::calibration
