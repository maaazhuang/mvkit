#pragma once

#include <array>

#include <core/tuple.hpp>
#include <core/geometry_types.hpp>

namespace mv::calibration {

[[nodiscard]] Quaterniond normalize_quaternion(Quaterniond q);
[[nodiscard]] Quaterniond quaternion_from_gba(double rot_x, double rot_y, double rot_z);
[[nodiscard]] std::array<double, 9> rotation_matrix(const Quaterniond& q);
[[nodiscard]] Quaterniond quaternion_from_rotation_matrix(const std::array<double, 9>& r);
[[nodiscard]] Point3d rotate_point(const Quaterniond& q, const Point3d& p);
[[nodiscard]] Point3d transform_point(const Pose3d& pose, const Point3d& p);
[[nodiscard]] Pose3d compose_pose(const Pose3d& left, const Pose3d& right);
[[nodiscard]] Pose3d inverse_pose(const Pose3d& pose);
[[nodiscard]] core::Tuple pose_to_halcon_tuple(const Pose3d& pose);
[[nodiscard]] Pose3d pose_from_halcon_tuple(const core::Tuple& tuple);

}  // namespace mv::calibration
