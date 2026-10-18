#pragma once
#include <core/tuple.hpp>
#include <core/geometry_types.hpp>
namespace mv::reconstruction {
struct RectifiedCamera {
  bool perspective{true};
  double scale{}, sx{}, sy{}, cx{}, cy{};
};
[[nodiscard]] RectifiedCamera parse_rectified_camera(const core::Tuple& input);
[[nodiscard]] Pose3d parse_rectified_pose(const core::Tuple& input, bool* nonfinite_rotation);
[[nodiscard]] Point3d rectified_disparity_point(const RectifiedCamera& first, const RectifiedCamera& second,
                                                const Pose3d& second_in_first, bool nonfinite_rotation,
                                                double row, double column, double disparity);
}  // namespace mv::reconstruction
