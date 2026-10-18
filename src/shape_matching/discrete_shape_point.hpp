#pragma once
#include <cstddef>
#include <cstdint>

namespace mv::shape_matching {
struct DiscreteShapePoint {
  std::ptrdiff_t image_offset{};
  std::int16_t row_offset{};
  std::int16_t col_offset{};
  float normal_row{};
  float normal_column{};
  std::int8_t qn_row{};
  std::int8_t qn_col{};
};

// 热态整数评分记录，浮点几何保留在 DiscreteShapePoint。
struct PackedI8ShapePoint {
  std::ptrdiff_t image_offset{};
  std::uint32_t normal_pair{};
};
static_assert(sizeof(PackedI8ShapePoint) <= 16);

inline PackedI8ShapePoint pack_i8_shape_point(const DiscreteShapePoint& point) noexcept {
  return {point.image_offset,
          (static_cast<std::uint32_t>(static_cast<std::uint16_t>(point.qn_col)) << 16) |
              static_cast<std::uint16_t>(point.qn_row)};
}

}
