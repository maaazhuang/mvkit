#pragma once

#include <array>

#include <core/tuple.hpp>

namespace mv::calibration {

/** HALCON 风格 12 元 3x4 齐次矩阵：[r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz]。 */
[[nodiscard]] core::Tuple pose_to_hom_mat3d(const core::Tuple& pose);
[[nodiscard]] core::Tuple hom_mat3d_to_pose(const core::Tuple& hom_mat3d);
[[nodiscard]] core::Tuple hom_mat3d_compose(const core::Tuple& left, const core::Tuple& right);
[[nodiscard]] core::Tuple hom_mat3d_invert(const core::Tuple& hom_mat3d);
void affine_trans_point_3d(const core::Tuple& hom_mat3d, const core::Tuple& px, const core::Tuple& py,
                           const core::Tuple& pz, core::Tuple* qx, core::Tuple* qy, core::Tuple* qz);

[[nodiscard]] std::array<double, 12> parse_hom_mat3d(const core::Tuple& hom_mat3d, const char* op,
                                                 bool require_inverse = true);
[[nodiscard]] core::Tuple hom_mat3d_from_values(const std::array<double, 12>& values);

}  // namespace mv::calibration
