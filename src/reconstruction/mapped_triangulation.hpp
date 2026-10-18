#pragma once

#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {
struct MappedTriangulationOptions {
  double maximum_hole_area{10};
  double maximum_view_angle{1.57079632679489661923};
  bool filter_view_angle{false};
  Point3d view_direction{0, 0, 1};
  bool output_all_points{false};
};

// 私有二维映射内核，公开 C2 协议由适配层实现。
ObjectModel3DData triangulate_mapping(const ObjectModel3DData& model,
                                      const MappedTriangulationOptions& options = {});
}  // namespace mv::reconstruction
