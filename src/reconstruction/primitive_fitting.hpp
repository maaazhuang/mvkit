#pragma once

#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {
enum class PrimitiveFitLoss { LeastSquares, Huber, Tukey };
struct PrimitiveFitOptions {
  // 位 0/1/2 分别表示 plane/sphere/cylinder，HALCON 默认 cylinder。
  unsigned types = 4;
  PrimitiveFitLoss loss = PrimitiveFitLoss::LeastSquares;
  double minimum_radius = .01, maximum_radius = .2;
};
struct PrimitiveFitResult : PrimitiveSupport {
  PrimitiveKind kind = PrimitiveKind::None;
  // 平面使用 HALCON N·P=D，区别于旧 N·P+d=0。
  std::array<double, 7> parameters{};
  double rms{};
  std::size_t iterations{};
};
// 数值候选内核；公开适配及输出所有权由各自层处理。
[[nodiscard]] PrimitiveFitResult fit_primitive_geometry(const ObjectModel3DData& model,
                                                        const PrimitiveFitOptions& options);
[[nodiscard]] double fitted_primitive_distance(Point3d point, const PrimitiveFitResult& fit);
[[nodiscard]] double model_primitive_distance(Point3d point, const ObjectModel3DData& model);
[[nodiscard]] core::Tuple primitive_frame_pose(const ObjectModel3DData& model);
[[nodiscard]] ObjectModel3DData fit_primitive_model(const ObjectModel3DData& model,
                                                    const PrimitiveFitOptions& options,
                                                    bool output_point_coord, bool output_xyz_mapping);
}  // namespace mv::reconstruction
