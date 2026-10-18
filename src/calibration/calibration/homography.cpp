#include <calibration/calibration/homography.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

struct Normalization2d {
  double center_x{};
  double center_y{};
  double scale{1.0};
};

Normalization2d normalization_for(std::span<const double> x,
                                  std::span<const double> y) {
  Normalization2d result;
  for (std::size_t i = 0; i < x.size(); ++i) {
    result.center_x += x[i];
    result.center_y += y[i];
  }
  result.center_x /= static_cast<double>(x.size());
  result.center_y /= static_cast<double>(y.size());

  double mean_squared_radius = 0.0;
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double dx = x[i] - result.center_x;
    const double dy = y[i] - result.center_y;
    mean_squared_radius += dx * dx + dy * dy;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }
  mean_squared_radius /= static_cast<double>(x.size());
  if (!std::isfinite(mean_squared_radius) || mean_squared_radius < 1e-24) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "homography points have degenerate spread");
  }
  // 点云归一化面积不受缩放和平移影响；近共线配置不能可靠确定平面单应。
  const double trace = xx + yy;
  const double normalized_area = (xx * yy - xy * xy) / (trace * trace);
  if (!std::isfinite(normalized_area) || normalized_area < 1e-12) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "homography points are nearly collinear");
  }
  result.scale = std::sqrt(2.0 / mean_squared_radius);
  return result;
}

std::array<double, 9> multiply3(const std::array<double, 9>& a,
                               const std::array<double, 9>& b) {
  std::array<double, 9> result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t k = 0; k < 3; ++k) {
        result[row * 3 + column] += a[row * 3 + k] * b[k * 3 + column];
      }
    }
  }
  return result;
}

}  // namespace

std::array<double, 9> fit_planar_homography_2d(std::span<const double> x,
                                                std::span<const double> y,
                                                std::span<const double> u,
                                                std::span<const double> v,
                                                numeric::LinearBackendKind backend_kind,
                                                std::span<const double> weights,
                                                bool coordinates_normalized) {
  if (x.size() < 4 || x.size() != y.size() || x.size() != u.size() ||
      x.size() != v.size()) {
    throw core::Exception(core::Status::InvalidArgument,
                          "estimate_planar_pose_perspective",
                          "homography requires four or more paired points");
  }
  for (std::size_t i = 0; !coordinates_normalized && i < x.size(); ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(y[i]) ||
        !std::isfinite(u[i]) || !std::isfinite(v[i])) {
      throw core::Exception(core::Status::InvalidArgument,
                            "estimate_planar_pose_perspective",
                            "homography coordinates must be finite");
    }
  }
  if (!weights.empty() && weights.size() != x.size())
    throw core::Exception(core::Status::InvalidArgument, "fit_planar_homography_2d", "weights length mismatch");
  // 内部快路径已由调用方验证有限归一化坐标和几何跨度；最小 RANSAC 子集仍独立归一化。
  const auto source = coordinates_normalized ? Normalization2d{} : normalization_for(x, y);
  const auto destination = coordinates_normalized ? Normalization2d{} : normalization_for(u, v);

  std::vector<double> design;
  design.reserve(x.size() * 18);
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double xn = source.scale * (x[i] - source.center_x);
    const double yn = source.scale * (y[i] - source.center_y);
    const double un = destination.scale * (u[i] - destination.center_x);
    const double vn = destination.scale * (v[i] - destination.center_y);
    std::array<double, 9> row_u{-xn, -yn, -1.0, 0.0, 0.0, 0.0,
                                      un * xn, un * yn, un};
    std::array<double, 9> row_v{0.0, 0.0, 0.0, -xn, -yn, -1.0,
                                      vn * xn, vn * yn, vn};
    if (!weights.empty()) {
      if (!std::isfinite(weights[i]) || weights[i] <= 0)
        throw core::Exception(core::Status::InvalidArgument, "fit_planar_homography_2d", "weights must be positive and finite");
      const double factor = std::sqrt(weights[i]);
      for (auto& value : row_u) value *= factor;
      for (auto& value : row_v) value *= factor;
    }
    design.insert(design.end(), row_u.begin(), row_u.end());
    design.insert(design.end(), row_v.begin(), row_v.end());
  }

  auto backend = numeric::make_backend(backend_kind);
  std::vector<double> h;
  if (!backend->smallest_right_singular_vector(design, x.size() * 2, 9, &h) || h.size() != 9) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "normalized homography solve is rank deficient");
  }
  const std::array<double, 9> normalized_h{
      h[0], h[1], h[2],
      h[3], h[4], h[5],
      h[6], h[7], h[8]};
  const std::array<double, 9> source_transform{
      source.scale, 0.0, -source.scale * source.center_x,
      0.0, source.scale, -source.scale * source.center_y,
      0.0, 0.0, 1.0};
  const std::array<double, 9> destination_inverse{
      1.0 / destination.scale, 0.0, destination.center_x,
      0.0, 1.0 / destination.scale, destination.center_y,
      0.0, 0.0, 1.0};

  auto result = multiply3(destination_inverse, multiply3(normalized_h, source_transform));
  double scale = result[8];
  if (std::abs(scale) < 1e-18) {
    double norm = 0.0;
    for (double value : result) norm += value * value;
    scale = std::sqrt(norm);
  }
  if (!std::isfinite(scale) || std::abs(scale) < 1e-18) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "homography normalization failed");
  }
  if (scale < 0.0) scale = -scale;
  for (double& value : result) value /= scale;
  return result;
}

namespace {

bool project2(const std::array<double, 9>& h, double x, double y,
              double* u, double* v) {
  const double d = h[6] * x + h[7] * y + h[8];
  if (!std::isfinite(d) || std::abs(d) < 1e-12) return false;
  *u = (h[0] * x + h[1] * y + h[2]) / d;
  *v = (h[3] * x + h[4] * y + h[5]) / d;
  return std::isfinite(*u) && std::isfinite(*v);
}

bool solve8(std::array<double, 64> a, std::array<double, 8> b,
            std::array<double, 8>* x) {
  for (std::size_t col = 0; col < 8; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 8; ++row)
      if (std::abs(a[row * 8 + col]) > std::abs(a[pivot * 8 + col])) pivot = row;
    if (std::abs(a[pivot * 8 + col]) < 1e-14) return false;
    if (pivot != col) {
      for (std::size_t k = col; k < 8; ++k)
        std::swap(a[pivot * 8 + k], a[col * 8 + k]);
      std::swap(b[pivot], b[col]);
    }
    const double diagonal = a[col * 8 + col];
    for (std::size_t row = col + 1; row < 8; ++row) {
      const double factor = a[row * 8 + col] / diagonal;
      for (std::size_t k = col + 1; k < 8; ++k)
        a[row * 8 + k] -= factor * a[col * 8 + k];
      b[row] -= factor * b[col];
    }
  }
  for (std::size_t row = 8; row-- > 0;) {
    double value = b[row];
    for (std::size_t col = row + 1; col < 8; ++col)
      value -= a[row * 8 + col] * (*x)[col];
    (*x)[row] = value / a[row * 8 + row];
  }
  return true;
}

double reprojection_cost(const std::array<double, 9>& h,
                         const std::vector<double>& x, const std::vector<double>& y,
                         const std::vector<double>& u, const std::vector<double>& v,
                         const std::vector<std::size_t>& indices,
                         std::span<const double> weights = {}) {
  double cost = 0;
  for (const auto i : indices) {
    double pu = 0, pv = 0;
    if (!project2(h, x[i], y[i], &pu, &pv))
      return std::numeric_limits<double>::infinity();
    const double du = pu - u[i], dv = pv - v[i];
    cost += (weights.empty() ? 1 : weights[i]) * (du * du + dv * dv);
  }
  return cost;
}

void refine_homography(std::array<double, 9>* h,
                       const std::vector<double>& x, const std::vector<double>& y,
                       const std::vector<double>& u, const std::vector<double>& v,
                       const std::vector<std::size_t>& indices,
                       std::span<const double> weights = {}, std::size_t max_iterations = 30,
                       std::size_t* iterations = nullptr, bool* converged = nullptr) {
  if (iterations) *iterations = 0;
  if (converged) *converged = false;
  if (std::abs((*h)[8]) < 1e-12) return;
  for (double& value : *h) value /= (*h)[8];
  double cost = reprojection_cost(*h, x, y, u, v, indices, weights);
  double lambda = 1e-3;
  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    if (iterations) *iterations = iteration + 1;
    std::array<double, 64> normal{};
    std::array<double, 8> gradient{};
    for (const auto i : indices) {
      const double d = (*h)[6] * x[i] + (*h)[7] * y[i] + 1.0;
      const double pu = ((*h)[0] * x[i] + (*h)[1] * y[i] + (*h)[2]) / d;
      const double pv = ((*h)[3] * x[i] + (*h)[4] * y[i] + (*h)[5]) / d;
      const std::array<double, 8> ju{x[i]/d, y[i]/d, 1.0/d, 0, 0, 0,
                                     -pu*x[i]/d, -pu*y[i]/d};
      const std::array<double, 8> jv{0, 0, 0, x[i]/d, y[i]/d, 1.0/d,
                                     -pv*x[i]/d, -pv*y[i]/d};
      for (std::size_t row = 0; row < 8; ++row) {
        const double weight = weights.empty() ? 1 : weights[i];
        gradient[row] += weight * (ju[row] * (u[i] - pu) + jv[row] * (v[i] - pv));
        for (std::size_t col = 0; col < 8; ++col)
          normal[row * 8 + col] += weight * (ju[row] * ju[col] + jv[row] * jv[col]);
      }
    }
    double gradient_max = 0;
    for (double value : gradient) gradient_max = std::max(gradient_max, std::abs(value));
    if (gradient_max <= 1e-10) { if (converged) *converged = true; break; }
    for (std::size_t i = 0; i < 8; ++i)
      normal[i * 8 + i] += lambda * (normal[i * 8 + i] + 1.0);
    std::array<double, 8> step{};
    if (!solve8(normal, gradient, &step)) break;
    auto candidate = *h;
    double step_norm = 0;
    for (std::size_t i = 0; i < 8; ++i) {
      candidate[i] += step[i];
      step_norm += step[i] * step[i];
    }
    const double next = reprojection_cost(candidate, x, y, u, v, indices, weights);
    if (lambda <= 1e-3 && (step_norm < 1e-20 ||
        (std::isfinite(next) && next <= cost && std::abs(next-cost) <= 1e-12 * (1+cost)))) {
      if (next < cost) *h = candidate;
      if (converged) *converged = true;
      break;
    }
    if (next < cost) {
      *h = candidate;
      cost = next;
      lambda = (std::max)(lambda * 0.3, 1e-9);
      if (step_norm < 1e-20) break;
    } else {
      lambda = (std::min)(lambda * 10.0, 1e10);
    }
  }
}

double norm3(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

void refine_planar_homography(std::array<double, 9>* matrix,
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& u, const std::vector<double>& v,
    const std::vector<std::size_t>& indices, std::span<const double> weights,
    std::size_t max_iterations, std::size_t* iterations, bool* converged) {
  refine_homography(matrix, x, y, u, v, indices, weights, max_iterations, iterations, converged);
}

RobustHomographyResult fit_planar_homography_ransac_2d(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& u, const std::vector<double>& v,
    double threshold, std::size_t max_trials, std::uint64_t seed,
    numeric::LinearBackendKind backend_kind) {
  constexpr auto op = "fit_planar_homography_ransac_2d";
  if (x.size() < 4 || x.size() != y.size() || x.size() != u.size() ||
      x.size() != v.size() || !std::isfinite(threshold) || threshold <= 0 ||
      max_trials == 0 || max_trials > 100000) {
    throw core::Exception(core::Status::InvalidArgument, op,
                          "invalid correspondences, threshold, or trial count");
  }
  for (std::size_t i = 0; i < x.size(); ++i)
    if (!std::isfinite(x[i]) || !std::isfinite(y[i]) ||
        !std::isfinite(u[i]) || !std::isfinite(v[i]))
      throw core::Exception(core::Status::InvalidArgument, op,
                            "correspondences must be finite");
  const auto source = normalization_for(x, y);
  const auto destination = normalization_for(u, v);
  std::vector<double> xn(x.size()), yn(x.size()), un(x.size()), vn(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    xn[i] = source.scale * (x[i] - source.center_x);
    yn[i] = source.scale * (y[i] - source.center_y);
    un[i] = destination.scale * (u[i] - destination.center_x);
    vn[i] = destination.scale * (v[i] - destination.center_y);
  }
  const double threshold2 = std::pow(threshold * destination.scale, 2);
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<std::size_t> choice(0, x.size() - 1);
  std::array<double, 9> best{};
  std::size_t best_count = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t trial = 0; trial < max_trials; ++trial) {
    std::array<std::size_t, 4> picked{};
    for (std::size_t j = 0; j < picked.size(); ++j) {
      do { picked[j] = choice(rng); }
      while (std::find(picked.begin(), picked.begin() + j, picked[j]) != picked.begin() + j);
    }
    std::vector<double> sx, sy, su, sv;
    for (const auto i : picked) {
      sx.push_back(xn[i]); sy.push_back(yn[i]);
      su.push_back(un[i]); sv.push_back(vn[i]);
    }
    std::array<double, 9> candidate{};
    try {
      candidate = fit_planar_homography_2d(sx, sy, su, sv, backend_kind);
    } catch (const core::Exception&) {
      continue;
    }
    std::size_t count = 0;
    double cost = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
      double pu = 0, pv = 0;
      if (!project2(candidate, xn[i], yn[i], &pu, &pv)) continue;
      const double du = pu - un[i], dv = pv - vn[i];
      const double error2 = du * du + dv * dv;
      if (error2 <= threshold2) { ++count; cost += error2; }
    }
    if (count > best_count || (count == best_count && cost < best_cost)) {
      best = candidate;
      best_count = count;
      best_cost = cost;
    }
  }
  if (best_count < 4)
    throw core::Exception(core::Status::InvalidState, op,
                          "no nondegenerate four-point consensus found");
  std::vector<std::size_t> inliers;
  for (int round = 0; round < 2; ++round) {
    inliers.clear();
    std::vector<double> sx, sy, su, sv;
    for (std::size_t i = 0; i < x.size(); ++i) {
      double pu = 0, pv = 0;
      if (!project2(best, xn[i], yn[i], &pu, &pv)) continue;
      const double du = pu - un[i], dv = pv - vn[i];
      if (du * du + dv * dv <= threshold2) {
        inliers.push_back(i);
        sx.push_back(xn[i]); sy.push_back(yn[i]);
        su.push_back(un[i]); sv.push_back(vn[i]);
      }
    }
    if (inliers.size() < 4) break;
    best = fit_planar_homography_2d(sx, sy, su, sv, backend_kind);
  }
  if (inliers.size() < 4)
    throw core::Exception(core::Status::InvalidState, op,
                          "homography consensus collapsed during refit");
  const double linear_rmse = std::sqrt(
      reprojection_cost(best, xn, yn, un, vn, inliers) / inliers.size()) /
      destination.scale;
  refine_homography(&best, xn, yn, un, vn, inliers);
  const double refined_rmse = std::sqrt(
      reprojection_cost(best, xn, yn, un, vn, inliers) / inliers.size()) /
      destination.scale;
  const std::array<double, 9> source_transform{
      source.scale, 0, -source.scale * source.center_x,
      0, source.scale, -source.scale * source.center_y, 0, 0, 1};
  const std::array<double, 9> destination_inverse{
      1.0 / destination.scale, 0, destination.center_x,
      0, 1.0 / destination.scale, destination.center_y, 0, 0, 1};
  auto matrix = multiply3(destination_inverse, multiply3(best, source_transform));
  if (!std::isfinite(matrix[8]) || std::abs(matrix[8]) < 1e-18)
    throw core::Exception(core::Status::InvalidState, op,
                          "refined homography normalization failed");
  const double scale = matrix[8];
  for (auto& value : matrix) value /= scale;
  std::size_t count = 0;
  double error2 = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    double pu = 0, pv = 0;
    if (!project2(matrix, x[i], y[i], &pu, &pv)) continue;
    const double du = pu - u[i], dv = pv - v[i];
    const double residual2 = du * du + dv * dv;
    if (residual2 <= threshold * threshold) {
      ++count;
      error2 += residual2;
    }
  }
  if (count < 4)
    throw core::Exception(core::Status::InvalidState, op,
                          "refined homography has insufficient inliers");
  return {matrix, count, std::sqrt(error2 / static_cast<double>(count)),
          linear_rmse, refined_rmse};
}

Pose3d estimate_planar_pose_perspective(const CameraParameters& camera,
                                        const std::vector<Point3d>& object_points,
                                        const std::vector<double>& rows,
                                        const std::vector<double>& columns,
                                        const std::vector<std::int64_t>& indices,
                                        numeric::LinearBackendKind backend_kind,
                                        const RobustPlanarPoseOptions* robust,
                                        std::span<const double> weights) {
  if (!is_perspective(camera) || rows.size() < 4 || rows.size() != columns.size() ||
      rows.size() != indices.size()) {
    throw core::Exception(core::Status::InvalidArgument,
                          "estimate_planar_pose_perspective",
                          "invalid planar pose input");
  }

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> u;
  std::vector<double> v;
  x.reserve(rows.size());
  y.reserve(rows.size());
  u.reserve(rows.size());
  v.reserve(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& point = object_points.at(static_cast<std::size_t>(indices[i]));
    if (std::abs(point.z) > 1e-10) {
      throw core::Exception(core::Status::InvalidState,
                            "estimate_planar_pose_perspective",
                            "target is not planar");
    }
    const auto ray = image_point_to_ray(camera, {rows[i], columns[i]});
    x.push_back(point.x);
    y.push_back(point.y);
    u.push_back(ray.direction.x / ray.direction.z);
    v.push_back(ray.direction.y / ray.direction.z);
  }

  std::array<double, 9> h{};
  if (robust) {
    const auto& common = common_parameters(camera);
    const double focus = camera_parameter_value(camera, "focus");
    const double threshold_ray = robust->threshold_pixels *
        std::sqrt(common.sx * common.sy) / focus;
    const auto fit = fit_planar_homography_ransac_2d(x, y, u, v, threshold_ray,
                                                     robust->max_trials, robust->seed,
                                                     backend_kind);
    if (fit.inliers < std::max<std::size_t>(6, (x.size() + 1) / 2))
      throw core::Exception(core::Status::InvalidState,
                            "estimate_planar_pose_perspective",
                            "ransac pose requires at least six and a majority of inliers");
    h = fit.matrix;
  } else {
    h = fit_planar_homography_2d(std::span<const double>(x),y,u,v,backend_kind,weights);
  }
  double pose_scale = 2.0 /
                      (norm3(h[0], h[3], h[6]) + norm3(h[1], h[4], h[7]));
  if (h[8] * pose_scale < 0.0) pose_scale = -pose_scale;

  double r1[3]{pose_scale * h[0], pose_scale * h[3], pose_scale * h[6]};
  const double n1 = norm3(r1[0], r1[1], r1[2]);
  for (double& value : r1) value /= n1;

  double r2[3]{pose_scale * h[1], pose_scale * h[4], pose_scale * h[7]};
  const double dot = r1[0] * r2[0] + r1[1] * r2[1] + r1[2] * r2[2];
  for (std::size_t i = 0; i < 3; ++i) r2[i] -= dot * r1[i];
  const double n2 = norm3(r2[0], r2[1], r2[2]);
  if (n2 < 1e-10) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "homography rotation is degenerate");
  }
  for (double& value : r2) value /= n2;

  const double r3[3]{r1[1] * r2[2] - r1[2] * r2[1],
                     r1[2] * r2[0] - r1[0] * r2[2],
                     r1[0] * r2[1] - r1[1] * r2[0]};
  const std::array<double, 9> rotation{
      r1[0], r2[0], r3[0],
      r1[1], r2[1], r3[1],
      r1[2], r2[2], r3[2]};
  Pose3d pose{{pose_scale * h[2], pose_scale * h[5], pose_scale * h[8]},
              quaternion_from_rotation_matrix(rotation)};
  if (pose.translation.z <= 0.0) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_perspective",
                          "estimated target is behind camera");
  }
  return pose;
}

}  // namespace mv::calibration
