#pragma once
#include <array>
#include <memory_resource>
#include <core/geometry_types.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <vector>

namespace mv::reconstruction {
// 不可变原语框架及有限支撑，副本共享直至变换生成新支撑。
struct PrimitiveSupport {
  PrimitiveSupport() = default;
  PrimitiveSupport(const PrimitiveSupport& other)
      : origin(other.origin),
        axis_x(other.axis_x),
        axis_y(other.axis_y),
        axis_z(other.axis_z),
        plane_boundary(other.plane_boundary, runtime::persist_resource()),
        cylinder_extent(other.cylinder_extent) {}
  PrimitiveSupport(PrimitiveSupport&&) noexcept = default;
  PrimitiveSupport& operator=(const PrimitiveSupport&) = default;
  PrimitiveSupport& operator=(PrimitiveSupport&&) noexcept = default;
  Point3d origin{}, axis_x{1, 0, 0}, axis_y{0, 1, 0}, axis_z{0, 0, 1};
  std::pmr::vector<Point3d> plane_boundary{runtime::persist_resource()};
  std::array<double, 2> cylinder_extent{};
};
}  // namespace mv::reconstruction
