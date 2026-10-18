#include "sparse_model.hpp"

#include <algorithm>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <map>
#include <set>
namespace mv::reconstruction {
namespace {
constexpr auto op = "reconstruct_points_stereo";
[[noreturn]] void bad(const char* operation, const char* parameter, core::ErrorCode code, const char* message,
                      std::int64_t index = -1) {
  throw core::Exception(
      code == core::ErrorCode::InvalidType ? core::Status::TypeMismatch : core::Status::InvalidArgument, code,
      operation, parameter, index, message);
}
double number(const core::Tuple& tuple, std::size_t i, const char* parameter, const char* operation = op) {
  const auto value = tuple.at(i);
  if (value.kind() != core::ValueKind::Int64 && value.kind() != core::ValueKind::Real64)
    bad(operation, parameter, core::ErrorCode::InvalidType, "numeric control required", i);
  const auto result = value.as_real64();
  if (!std::isfinite(result))
    bad(operation, parameter, core::ErrorCode::InvalidValue, "finite control required", i);
  return result;
}
std::int64_t integer(const core::Tuple& tuple, std::size_t i, const char* parameter,
                     const char* operation = op) {
  if (tuple.at(i).kind() != core::ValueKind::Int64)
    bad(operation, parameter, core::ErrorCode::InvalidType, "integer control required", i);
  return tuple.as_int64(i);
}
struct Observation {
  double row, column;
  std::size_t camera, input;
};
struct WorldRay {
  std::array<double, 3> origin;
  std::array<std::array<double, 3>, 2> planes;
};
// 求最近直线最小二乘，秩不足时保留最小范数解；此处允许负深度。
std::array<double, 3> intersect(const std::vector<WorldRay>& rays) {
  std::vector<double> a(9, 0.), b(3, 0.), values, vectors;
  for (const auto& ray : rays)
    for (const auto& n : ray.planes) {
      double rhs = 0.;
      for (std::size_t i = 0; i < 3; ++i) rhs += n[i] * ray.origin[i];
      for (std::size_t i = 0; i < 3; ++i) {
        b[i] += n[i] * rhs;
        for (std::size_t j = 0; j < 3; ++j) a[3 * i + j] += n[i] * n[j];
      }
    }
  const auto& backend = calibration::numeric::shared_backend(calibration::numeric::default_backend_kind());
  if (!backend.symmetric_eigen(a, 3, &values, &vectors))
    throw core::Exception(core::Status::InvalidState, op, "could not solve lines of sight");
  std::array<double, 3> result{};
  const auto threshold = std::max(1., values.back()) * 1e-14;
  for (std::size_t k = 0; k < 3; ++k)
    if (values[k] > threshold) {
      double coefficient = 0.;
      for (std::size_t i = 0; i < 3; ++i) coefficient += vectors[3 * i + k] * b[i];
      coefficient /= values[k];
      for (std::size_t i = 0; i < 3; ++i) result[i] += coefficient * vectors[3 * i + k];
    }
  for (const auto v : result)
    if (!std::isfinite(v))
      throw core::Exception(core::Status::InvalidState, op, "nonfinite least-squares result");
  return result;
}
}  // namespace
SparseStereoModel::SparseStereoModel(const CameraSetupModel& setup, std::string_view method,
                                     const core::Tuple& names, const core::Tuple& values) {
  constexpr auto create = "create_stereo_model";
  if (method != "points_3d") {
    if (method == "surface_pairwise" || method == "surface_fusion")
      throw core::Exception(core::Status::Unsupported, core::ErrorCode::InvalidValue, create, "Method", 0,
                            "surface stereo models are not implemented");
    bad(create, "Method", core::ErrorCode::InvalidValue, "invalid stereo model method");
  }
  cameras_.reserve(setup.camera_count());
  for (std::size_t i = 0; i < setup.camera_count(); ++i)
    cameras_.push_back({setup.camera(i), calibration::pose_from_halcon_tuple(setup.camera_pose(i))});
  set_parameters_unlocked(names, values);
}
void SparseStereoModel::require_open(const char* operation) const {
  if (closed_)
    throw core::Exception(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, operation,
                          "StereoModelID", 0, "stereo model was cleared");
}
void SparseStereoModel::clear() {
  std::lock_guard lock(mutex_);
  require_open("clear_stereo_model");
  closed_ = true;
  cameras_.clear();
}
void SparseStereoModel::set_parameters(const core::Tuple& names, const core::Tuple& values) {
  std::lock_guard lock(mutex_);
  require_open("set_stereo_model_param");
  set_parameters_unlocked(names, values);
}
void SparseStereoModel::set_parameters_unlocked(const core::Tuple& names, const core::Tuple& values) {
  constexpr auto set = "set_stereo_model_param";
  auto bounds = bounds_;
  auto persistence = persistence_;
  if (names.empty()) {
    if (!values.empty())
      bad(set, "GenParamValue", core::ErrorCode::InvalidCardinality, "empty names require empty values");
    return;
  }
  for (std::size_t i = 0; i < names.size(); ++i)
    if (names.at(i).kind() != core::ValueKind::String)
      bad(set, "GenParamName", core::ErrorCode::InvalidType, "parameter name must be string", i);
  if (names.size() == 1 && names.string_view(0) == "bounding_box") {
    if (values.size() != 6)
      bad(set, "GenParamValue", core::ErrorCode::InvalidCardinality, "bounding box requires six values");
    for (std::size_t i = 0; i < 6; ++i) bounds[i] = number(values, i, "GenParamValue", set);
  } else {
    if (names.size() != values.size())
      bad(set, "GenParamValue", core::ErrorCode::InvalidCardinality,
          "parameter names and values must have equal length");
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (names.string_view(i) != "persistence")
        bad(set, "GenParamName", core::ErrorCode::InvalidValue,
            "unsupported parameter for points_3d stereo model", i);
      const auto v = integer(values, i, "GenParamValue", set);
      if (v != 0 && v != 1)
        bad(set, "GenParamValue", core::ErrorCode::InvalidValue, "persistence must be zero or one", i);
      persistence = v != 0;
    }
  }
  bounds_ = bounds;
  persistence_ = persistence;
}
SparseStereoPoints SparseStereoModel::reconstruct(const core::Tuple& rows, const core::Tuple& columns,
                                                  const core::Tuple& covariance, const core::Tuple& cameras,
                                                  const core::Tuple& points) const {
  std::lock_guard lock(mutex_);
  require_open(op);
  const auto count = rows.size();
  if (columns.size() != count)
    bad(op, "Column", core::ErrorCode::InvalidCardinality, "image coordinates must have equal length");
  if (cameras.size() != count)
    bad(op, "CameraIdx", core::ErrorCode::InvalidCardinality, "one camera index required per observation");
  if (points.size() != count)
    bad(op, "PointIdx", core::ErrorCode::InvalidCardinality, "one point index required per observation");
  if (!covariance.empty() && (count > static_cast<std::size_t>(-1) / 4 || covariance.size() != 4 * count))
    bad(op, "CovIP", core::ErrorCode::InvalidCardinality,
        "one four-element covariance required per observation");
  std::vector<std::array<double, 4>> covariances(count);
  for (std::size_t i = 0; i < covariance.size(); ++i)
    covariances[i / 4][i % 4] = number(covariance, i, "CovIP");
  for (std::size_t i = 0; i < count && !covariance.empty(); ++i)
    if (covariances[i][0] < 0 || covariances[i][3] < 0)
      bad(op, "CovIP", core::ErrorCode::InvalidValue, "image variance must be nonnegative", i);
  std::map<std::int64_t, std::vector<Observation>> grouped;
  for (std::size_t i = 0; i < count; ++i) {
    const auto camera = integer(cameras, i, "CameraIdx"), point = integer(points, i, "PointIdx");
    if (camera < 0 || static_cast<std::uint64_t>(camera) >= cameras_.size())
      bad(op, "CameraIdx", core::ErrorCode::IndexOutOfRange, "camera index outside setup", i);
    auto& group = grouped[point];
    for (const auto& observation : group)
      if (observation.camera == static_cast<std::size_t>(camera))
        bad(op, "CameraIdx", core::ErrorCode::InvalidValue,
            "a point may have only one observation per camera", i);
    group.push_back(
        {number(rows, i, "Row"), number(columns, i, "Column"), static_cast<std::size_t>(camera), i});
  }
  auto ray = [&](const Observation& observation) {
    const auto& camera = cameras_[observation.camera];
    const auto local =
        calibration::image_point_to_ray(camera.parameters, {observation.row, observation.column});
    const auto origin = calibration::transform_point(camera.pose, local.origin);
    // HALCON 求定义各射线的两个相机坐标平面交集，尺度为归一化图像坐标而非单位射线长度；
    // 倾斜/噪声多视图及有限协方差步可显现差异。
    const auto nx = calibration::rotate_point(camera.pose.rotation,
                                              {1., 0., -local.direction.x / local.direction.z}),
               ny = calibration::rotate_point(camera.pose.rotation,
                                              {0., 1., -local.direction.y / local.direction.z});
    return WorldRay{{origin.x, origin.y, origin.z}, {{{nx.x, nx.y, nx.z}, {ny.x, ny.y, ny.z}}}};
  };
  const bool clipped = bounds_[0] < bounds_[3] && bounds_[1] < bounds_[4] && bounds_[2] < bounds_[5];
  SparseStereoPoints result;
  for (const auto& [point, observations] : grouped) {
    if (observations.size() < 2) continue;
    std::vector<WorldRay> rays;
    rays.reserve(observations.size());
    for (const auto& observation : observations) rays.push_back(ray(observation));
    const auto position = intersect(rays);
    if (clipped && (position[0] < bounds_[0] || position[0] > bounds_[3] || position[1] < bounds_[1] ||
                    position[1] > bounds_[4] || position[2] < bounds_[2] || position[2] > bounds_[5]))
      continue;
    result.x.append(position[0]);
    result.y.append(position[1]);
    result.z.append(position[2]);
    result.indices.append(point);
    if (covariance.empty()) continue;
    std::array<double, 9> world_cov{};
    for (std::size_t i = 0; i < observations.size(); ++i) {
      const auto& observation = observations[i];
      const auto& c = covariances[observation.input];
      std::array<std::array<double, 3>, 2> jac{};
      for (std::size_t j = 0; j < 2; ++j) {
        const auto step = c[j * 3] > 0 ? std::sqrt(c[j * 3]) : 1.;
        auto perturbed = observation;
        if (j == 0)
          perturbed.row += step;
        else
          perturbed.column += step;
        const auto original = rays[i];
        rays[i] = ray(perturbed);
        const auto shifted = intersect(rays);
        rays[i] = original;
        for (std::size_t k = 0; k < 3; ++k) jac[j][k] = (shifted[k] - position[k]) / step;
      }
      for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t s = 0; s < 3; ++s)
          world_cov[3 * r + s] += jac[0][r] * (c[0] * jac[0][s] + c[1] * jac[1][s]) +
                                  jac[1][r] * (c[2] * jac[0][s] + c[3] * jac[1][s]);
    }
    for (auto v : world_cov) result.covariance.append(v);
  }
  return result;
}
}  // namespace mv::reconstruction
