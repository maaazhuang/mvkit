#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <span>

#include <calibration/camera_parameters.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <core/geometry_types.hpp>

namespace mv::calibration {

[[nodiscard]] std::array<double, 9> fit_planar_homography_2d(
    std::span<const double> x,
    std::span<const double> y,
    std::span<const double> u,
    std::span<const double> v,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind(),
    std::span<const double> weights = {}, bool coordinates_normalized = false);

// 采用目标坐标的正向残差目标，不宣称等同 HALCON gold_standard。
void refine_planar_homography(std::array<double, 9>* matrix,
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& u, const std::vector<double>& v,
    const std::vector<std::size_t>& indices, std::span<const double> weights,
    std::size_t max_iterations, std::size_t* iterations, bool* converged);

[[nodiscard]] inline std::array<double,9> fit_planar_homography_2d(
    const std::vector<double>& x,const std::vector<double>& y,
    const std::vector<double>& u,const std::vector<double>& v,
    numeric::LinearBackendKind kind=numeric::default_backend_kind()) {
  return fit_planar_homography_2d(std::span<const double>{x},std::span<const double>{y},
                                 std::span<const double>{u},std::span<const double>{v},kind);
}

struct RobustHomographyResult {
  std::array<double, 9> matrix{};
  std::size_t inliers{};
  double inlier_rmse{};
  double linear_rmse{};
  double refined_rmse{};
};

struct RobustPlanarPoseOptions {
  double threshold_pixels{};
  std::size_t max_trials{};
  std::uint64_t seed{};
};

// 阈值为目标坐标中的欧氏重投影距离。
[[nodiscard]] RobustHomographyResult fit_planar_homography_ransac_2d(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& u, const std::vector<double>& v,
    double threshold, std::size_t max_trials, std::uint64_t seed,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind());

[[nodiscard]] Pose3d estimate_planar_pose_perspective(
    const CameraParameters& camera,
    const std::vector<Point3d>& object_points,
    const std::vector<double>& rows,
    const std::vector<double>& columns,
    const std::vector<std::int64_t>& indices,
    numeric::LinearBackendKind backend_kind = numeric::default_backend_kind(),
    const RobustPlanarPoseOptions* robust = nullptr, std::span<const double> weights = {});

}  // namespace mv::calibration
