#pragma once

#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <core/tuple.hpp>
#include <vector>

namespace mv::calibration {

struct GeometryEstimationOptions {
  bool robust{};
  std::size_t max_trials{512}, max_iterations{30};
  std::uint64_t seed{23063};
  double threshold{2}, confidence{.99}, min_inlier_ratio{.5};
};
struct GeometryEstimationResult {
  core::Tuple model, residuals, inliers;
  double rms{}, all_rms{}, weighted_rms{};
  std::size_t samples{}, inlier_count{}, trials{}, iterations{};
  int geometry_rank{};
  bool converged{};
};
[[nodiscard]] GeometryEstimationResult vector_to_proj_hom_mat2d(const core::Tuple&, const core::Tuple&,
                                                                const core::Tuple&, const core::Tuple&,
                                                                const core::Tuple& weights,
                                                                const GeometryEstimationOptions&);
[[nodiscard]] GeometryEstimationResult vector_to_pose(const core::Tuple&, const core::Tuple&,
                                                      const core::Tuple&, const core::Tuple&,
                                                      const core::Tuple&, const core::Tuple& camera,
                                                      const core::Tuple& weights,
                                                      const GeometryEstimationOptions&);
// 复用既有 DLT 初始化，调用方提供归一化坐标及正权重。
[[nodiscard]] Pose3d initialize_nonplanar_pose(const CameraParameters&, const std::vector<Point3d>&,
                                               const std::vector<double>&, const std::vector<double>&,
                                               std::span<const double> weights);

void vector_to_pose(const core::Tuple& world_x, const core::Tuple& world_y, const core::Tuple& world_z,
                    const core::Tuple& image_row, const core::Tuple& image_column,
                    const core::Tuple& camera_param, const core::Tuple& method, core::Tuple* pose,
                    core::Tuple* quality);
void vector_to_hom_mat2d(const core::Tuple& px, const core::Tuple& py, const core::Tuple& qx,
                         const core::Tuple& qy, core::Tuple* hom_mat2d, bool halcon_contract = false,
                         bool coordinates_normalized = false);

}  // namespace mv::calibration
