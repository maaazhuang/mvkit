#include <calibration/calibration/initialize.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <calibration/calibration/homography.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

std::array<double, 3> fit_affine(const std::vector<double>& x,
                                 const std::vector<double>& y,
                                 const std::vector<double>& value,
                                 const numeric::LinearAlgebraBackend& backend) {
  std::vector<double> normal(9, 0.0);
  std::vector<double> rhs(3, 0.0);
  for (std::size_t i = 0; i < x.size(); ++i) {
    const std::array<double, 3> row{x[i], y[i], 1.0};
    for (std::size_t c = 0; c < 3; ++c) {
      rhs[c] += row[c] * value[i];
      for (std::size_t d = 0; d < 3; ++d) normal[c * 3 + d] += row[c] * row[d];
    }
  }
  std::vector<double> solution;
  if (!backend.solve_symmetric(normal, rhs, 3, &solution) || solution.size() != 3) {
    throw core::Exception(core::Status::InvalidState,
                          "telecentric_planar_initialization",
                          "affine initialization is rank deficient");
  }
  return {solution[0], solution[1], solution[2]};
}

std::array<double, 6> zhang_v(const std::array<double, 9>& h,
                              std::size_t i,
                              std::size_t j) {
  const auto hc = [&](std::size_t row, std::size_t col) { return h[row * 3 + col]; };
  return {
      hc(0, i) * hc(0, j),
      hc(0, i) * hc(1, j) + hc(1, i) * hc(0, j),
      hc(1, i) * hc(1, j),
      hc(2, i) * hc(0, j) + hc(0, i) * hc(2, j),
      hc(2, i) * hc(1, j) + hc(1, i) * hc(2, j),
      hc(2, i) * hc(2, j),
  };
}

void accumulate_outer(std::array<double, 36>* normal, const std::array<double, 6>& row) {
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < 6; ++c) (*normal)[r * 6 + c] += row[r] * row[c];
  }
}

std::vector<double> object_x(const CalibrationTarget& target, const CalibrationObservation& observation) {
  std::vector<double> result;
  result.reserve(observation.indices.size());
  for (const auto index : observation.indices) {
    const auto& point = target.points.at(static_cast<std::size_t>(index));
    if (std::abs(point.z) > 1e-10) {
      throw core::Exception(core::Status::InvalidState,
                            "planar_initialization",
                            "calibration target is not planar");
    }
    result.push_back(point.x);
  }
  return result;
}

std::vector<double> object_y(const CalibrationTarget& target, const CalibrationObservation& observation) {
  std::vector<double> result;
  result.reserve(observation.indices.size());
  for (const auto index : observation.indices) result.push_back(target.points.at(static_cast<std::size_t>(index)).y);
  return result;
}

double telecentric_magnification(const CameraParameters& camera) {
  return std::visit([](const auto& p) -> double {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanTelecentricDivisionParameters> ||
                  std::is_same_v<T, AreaScanTelecentricPolynomialParameters>) {
      return p.magnification;
    } else {
      return 0.0;
    }
  }, camera);
}

double largest_singular_value_2x2(double a, double b, double c, double d) {
  const double trace = a * a + b * b + c * c + d * d;
  const double determinant = (a * d - b * c) * (a * d - b * c);
  const double discriminant = std::max(0.0, trace * trace - 4.0 * determinant);
  return std::sqrt(0.5 * (trace + std::sqrt(discriminant)));
}

}  // namespace

CameraParameters initialize_perspective_intrinsics_planar(
    const CameraParameters& initial,
    const CalibrationDataModel& data,
    const std::set<std::string>& excluded_settings,
    numeric::LinearBackendKind backend_kind) {
  if (!is_perspective(initial) || data.observations().size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_perspective_intrinsics_planar",
                          "perspective planar initialization requires at least three observations");
  }

  std::array<double, 36> normal{};
  auto backend = numeric::make_backend(backend_kind);
  for (const auto& [key, observation] : data.observations()) {
    const auto& target = data.target(key.calib_object_index);
    const auto x = object_x(target, observation);
    const auto y = object_y(target, observation);
    const auto h = fit_planar_homography_2d(x, y, observation.columns, observation.rows, backend_kind);
    const auto v12 = zhang_v(h, 0, 1);
    const auto v11 = zhang_v(h, 0, 0);
    const auto v22 = zhang_v(h, 1, 1);
    std::array<double, 6> difference{};
    for (std::size_t i = 0; i < 6; ++i) difference[i] = v11[i] - v22[i];
    accumulate_outer(&normal, v12);
    accumulate_outer(&normal, difference);
  }

  std::vector<double> normal_values(normal.begin(), normal.end());
  std::vector<double> eigenvalues;
  std::vector<double> eigenvectors;
  if (!backend->symmetric_eigen(normal_values, 6, &eigenvalues, &eigenvectors) ||
      eigenvalues.size() != 6 || eigenvectors.size() != 36) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_perspective_intrinsics_planar",
                          "Zhang initialization eigen decomposition failed");
  }
  std::array<double, 6> b{};
  for (std::size_t i = 0; i < 6; ++i) b[i] = eigenvectors[i * 6];
  if (b[0] < 0.0) for (double& value : b) value = -value;

  const double denominator = b[0] * b[2] - b[1] * b[1];
  if (b[0] <= 0.0 || denominator <= 1e-20) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_perspective_intrinsics_planar",
                          "Zhang initialization is degenerate");
  }
  const double cy = (b[1] * b[3] - b[0] * b[4]) / denominator;
  const double lambda = b[5] -
                        (b[3] * b[3] + cy * (b[1] * b[3] - b[0] * b[4])) / b[0];
  if (lambda <= 0.0 || !std::isfinite(lambda)) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_perspective_intrinsics_planar",
                          "Zhang initialization produced invalid focal scale");
  }
  const double fx = std::sqrt(lambda / b[0]);
  const double fy = std::sqrt(lambda * b[0] / denominator);
  const double skew = -b[1] * fx * fx * fy / lambda;
  const double cx = skew * cy / fy - b[3] * fx * fx / lambda;
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
      fx <= 0.0 || fy <= 0.0) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_perspective_intrinsics_planar",
                          "Zhang initialization produced non-finite intrinsics");
  }

  CameraParameters result = initial;
  const auto& common = common_parameters(initial);
  double focus = camera_parameter_value(result, "focus");
  if (!excluded_settings.count("focus")) {
    focus = fx * common.sx;
    set_camera_parameter_value(result, "focus", focus);
  }
  if (!excluded_settings.count("sy")) set_camera_parameter_value(result, "sy", focus / fy);
  if (!excluded_settings.count("cx")) set_camera_parameter_value(result, "cx", cx);
  if (!excluded_settings.count("cy")) set_camera_parameter_value(result, "cy", cy);
  return result;
}

CameraParameters initialize_telecentric_intrinsics_planar(
    const CameraParameters& initial,
    const CalibrationDataModel& data,
    const std::set<std::string>& excluded_settings,
    numeric::LinearBackendKind backend_kind) {
  if (is_perspective(initial) || data.observations().size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "initialize_telecentric_intrinsics_planar",
                          "telecentric planar initialization requires at least three observations");
  }
  if (excluded_settings.count("magnification")) return initial;

  auto backend = numeric::make_backend(backend_kind);
  std::vector<double> estimates;
  estimates.reserve(data.observations().size());
  const double initial_magnification = telecentric_magnification(initial);
  for (const auto& [key, observation] : data.observations()) {
    const auto& target = data.target(key.calib_object_index);
    const auto x = object_x(target, observation);
    const auto y = object_y(target, observation);
    std::vector<double> camera_x;
    std::vector<double> camera_y;
    camera_x.reserve(observation.rows.size());
    camera_y.reserve(observation.rows.size());
    for (std::size_t i = 0; i < observation.rows.size(); ++i) {
      const auto ray = image_point_to_ray(initial, {observation.rows[i], observation.columns[i]});
      camera_x.push_back(ray.origin.x);
      camera_y.push_back(ray.origin.y);
    }
    const auto ax = fit_affine(x, y, camera_x, *backend);
    const auto ay = fit_affine(x, y, camera_y, *backend);
    const double ratio = largest_singular_value_2x2(ax[0], ax[1], ay[0], ay[1]);
    if (ratio <= 1e-12 || !std::isfinite(ratio)) {
      throw core::Exception(core::Status::InvalidState,
                            "initialize_telecentric_intrinsics_planar",
                            "telecentric affine scale is degenerate");
    }
    estimates.push_back(initial_magnification * ratio);
  }
  std::sort(estimates.begin(), estimates.end());
  const double magnification = estimates[estimates.size() / 2];
  CameraParameters result = initial;
  set_camera_parameter_value(result, "magnification", magnification);
  return result;
}

Pose3d estimate_planar_pose_telecentric(const CameraParameters& camera,
                                        const std::vector<Point3d>& object_points,
                                        const std::vector<double>& rows,
                                        const std::vector<double>& columns,
                                        const std::vector<std::int64_t>& indices,
                                        numeric::LinearBackendKind backend_kind) {
  if (is_perspective(camera) || rows.size() < 4 || rows.size() != columns.size() ||
      rows.size() != indices.size()) {
    throw core::Exception(core::Status::InvalidArgument,
                          "estimate_planar_pose_telecentric",
                          "invalid telecentric planar pose input");
  }
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> camera_x;
  std::vector<double> camera_y;
  x.reserve(rows.size());
  y.reserve(rows.size());
  camera_x.reserve(rows.size());
  camera_y.reserve(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& point = object_points.at(static_cast<std::size_t>(indices[i]));
    if (std::abs(point.z) > 1e-10) {
      throw core::Exception(core::Status::InvalidState,
                            "estimate_planar_pose_telecentric",
                            "target is not planar");
    }
    const auto ray = image_point_to_ray(camera, {rows[i], columns[i]});
    x.push_back(point.x);
    y.push_back(point.y);
    camera_x.push_back(ray.origin.x);
    camera_y.push_back(ray.origin.y);
  }

  auto backend = numeric::make_backend(backend_kind);
  const auto ax = fit_affine(x, y, camera_x, *backend);
  const auto ay = fit_affine(x, y, camera_y, *backend);
  const double ratio = largest_singular_value_2x2(ax[0], ax[1], ay[0], ay[1]);
  if (ratio <= 1e-12) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_telecentric",
                          "telecentric affine rotation is degenerate");
  }

  double r11 = ax[0] / ratio;
  double r12 = ax[1] / ratio;
  double r21 = ay[0] / ratio;
  double r22 = ay[1] / ratio;
  const double z1_abs = std::sqrt(std::max(0.0, 1.0 - r11 * r11 - r12 * r12));
  const double z2_abs = std::sqrt(std::max(0.0, 1.0 - r21 * r21 - r22 * r22));
  const double dot_xy = r11 * r21 + r12 * r22;
  double r13 = z1_abs;
  double r23 = z2_abs;
  if (std::abs(dot_xy + r13 * r23) > std::abs(dot_xy - r13 * r23)) r23 = -r23;

  double n1 = std::sqrt(r11 * r11 + r12 * r12 + r13 * r13);
  r11 /= n1; r12 /= n1; r13 /= n1;
  const double dot = r11 * r21 + r12 * r22 + r13 * r23;
  r21 -= dot * r11; r22 -= dot * r12; r23 -= dot * r13;
  const double n2 = std::sqrt(r21 * r21 + r22 * r22 + r23 * r23);
  if (n2 < 1e-10) {
    throw core::Exception(core::Status::InvalidState,
                          "estimate_planar_pose_telecentric",
                          "telecentric affine rows are degenerate");
  }
  r21 /= n2; r22 /= n2; r23 /= n2;
  const double r31 = r12 * r23 - r13 * r22;
  const double r32 = r13 * r21 - r11 * r23;
  const double r33 = r11 * r22 - r12 * r21;
  const std::array<double, 9> rotation{
      r11, r12, r13,
      r21, r22, r23,
      r31, r32, r33};
  return {{ax[2] / ratio, ay[2] / ratio, 0.0}, quaternion_from_rotation_matrix(rotation)};
}

}  // namespace mv::calibration
