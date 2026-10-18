#pragma once
#include <array>
#include <core/xld.hpp>
#include <span>
namespace mv::xld {
void validate_distance_points_avx2(std::span<const core::Point2d> points);
std::array<double, 2>
distance_point_contour_avx2(std::span<const core::Point2d> points, bool closed,
                            core::Point2d point);
} // namespace mv::xld
