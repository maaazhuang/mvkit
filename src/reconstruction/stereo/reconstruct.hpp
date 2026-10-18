#pragma once

#include <cstdint>

#include <core/image.hpp>
#include <core/tuple.hpp>
#include <core/geometry_types.hpp>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/stereo/rectification.hpp>

namespace mv::reconstruction {

[[nodiscard]] Point3d disparity_to_point_3d(const StereoRectification& rectification,
                                             double row,
                                             double column,
                                             double disparity,
                                             double max_ray_gap = 1e-4);

[[nodiscard]] XyzImages disparity_image_to_xyz(const StereoRectification& rectification,
                                                const core::Image& disparity,
                                                double max_ray_gap = 1e-4);

[[nodiscard]] ObjectModel3DData reconstruct_points_stereo(
    const core::Image& rectified_left,
    const core::Image& rectified_right,
    const StereoRectification& rectification,
    std::int64_t min_disparity,
    std::int64_t max_disparity,
    std::int64_t smoothness,
    std::int64_t edge_preservation,
    const core::Tuple& gen_param_name,
    const core::Tuple& gen_param_value,
    double max_ray_gap = 1e-4);

}  // namespace mv::reconstruction
