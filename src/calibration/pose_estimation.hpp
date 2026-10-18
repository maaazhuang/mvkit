#pragma once

#include <core/tuple.hpp>
#include <string_view>

namespace mv::calibration {
struct PoseEstimate {
  core::Tuple pose;
  double error{};
};
// 正式方法内核；旧加权/RANSAC 估计保持独立。
[[nodiscard]] PoseEstimate estimate_pose(const core::Tuple& world_x, const core::Tuple& world_y,
                                         const core::Tuple& world_z, const core::Tuple& row,
                                         const core::Tuple& column, const core::Tuple& camera,
                                         std::string_view method);
}  // namespace mv::calibration
