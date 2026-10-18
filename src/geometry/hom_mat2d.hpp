#pragma once

#include <array>

#include <core/tuple.hpp>

namespace mv::geometry {

/** 二维仿射矩阵为 6 元 Tuple：[a, b, tx, c, d, ty]，作用于 x=column、y=row。 */
[[nodiscard]] core::Tuple hom_mat2d_identity();
[[nodiscard]] core::Tuple hom_mat2d_translate(const core::Tuple& hom_mat2d, double tx, double ty);
[[nodiscard]] core::Tuple hom_mat2d_rotate(const core::Tuple& hom_mat2d, double phi, double px,
                                           double py);
[[nodiscard]] core::Tuple hom_mat2d_scale(const core::Tuple& hom_mat2d, double sx, double sy,
                                          double px, double py);
[[nodiscard]] core::Tuple hom_mat2d_compose(const core::Tuple& left, const core::Tuple& right);
[[nodiscard]] core::Tuple hom_mat2d_invert(const core::Tuple& hom_mat2d);
void affine_trans_point_2d(const core::Tuple& hom_mat2d, const core::Tuple& px,
                           const core::Tuple& py, core::Tuple* qx, core::Tuple* qy);
[[nodiscard]] core::Tuple vector_angle_to_rigid(double row1, double column1, double angle1,
                                                double row2, double column2, double angle2);

[[nodiscard]] std::array<double, 6> parse_hom_mat2d(const core::Tuple& hom_mat2d, const char* op);
[[nodiscard]] core::Tuple hom_mat2d_from_values(const std::array<double, 6>& values);

}  // namespace mv::geometry
