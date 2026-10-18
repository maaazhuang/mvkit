#pragma once
#include <reconstruction/stereo/reconstruct.hpp>
#include <reconstruction/stereo/rectified_points.hpp>
namespace mv::reconstruction {
struct RectifiedXyzGeometry {
  RectifiedCamera first, second;
  Pose3d pose;
  double baseline{};
  bool nonfinite_rotation{};
};
void validate_rectified_xyz_geometry(const RectifiedXyzGeometry& geometry);
[[nodiscard]] XyzImages rectified_disparity_xyz(const core::Image& disparity,
                                                const RectifiedXyzGeometry& geometry);
}  // namespace mv::reconstruction
