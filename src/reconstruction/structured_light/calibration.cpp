#include <reconstruction/structured_light/calibration.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include <calibration/model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

std::vector<double> numeric(const core::Tuple& tuple, const char* operation) {
  std::vector<double> out;
  out.reserve(tuple.size());
  for (std::size_t i = 0; i < tuple.size(); ++i) {
    const double value = tuple.at(i).as_real64();
    if (!std::isfinite(value)) {
      throw core::Exception(core::Status::InvalidArgument, operation, "coordinates must be finite");
    }
    out.push_back(value);
  }
  return out;
}

std::vector<std::int64_t> indices(const core::Tuple& tuple, const char* operation) {
  std::vector<std::int64_t> out;
  out.reserve(tuple.size());
  std::set<std::int64_t> seen;
  for (std::size_t i = 0; i < tuple.size(); ++i) {
    const auto value = tuple.at(i).as_int64();
    if (value < 0 || !seen.insert(value).second) {
      throw core::Exception(core::Status::InvalidArgument,
                            operation,
                            "indices must be unique nonnegative integers");
    }
    out.push_back(value);
  }
  return out;
}

core::Tuple real_tuple(const std::vector<double>& values) {
  core::Tuple out;
  for (double value : values) {
    out.append(value);
  }
  return out;
}

core::Tuple int_tuple(const std::vector<std::int64_t>& values) {
  core::Tuple out;
  for (auto value : values) {
    out.append(value);
  }
  return out;
}

Quaterniond average_quaternions(const std::vector<Quaterniond>& quaternions) {
  if (quaternions.empty()) {
    return {};
  }

  const auto reference = calibration::normalize_quaternion(quaternions.front());
  Quaterniond sum{};
  sum.w = 0.0;
  sum.x = 0.0;
  sum.y = 0.0;
  sum.z = 0.0;

  for (auto quaternion : quaternions) {
    quaternion = calibration::normalize_quaternion(quaternion);
    const double dot = reference.w * quaternion.w + reference.x * quaternion.x +
                       reference.y * quaternion.y + reference.z * quaternion.z;
    const double sign = dot < 0.0 ? -1.0 : 1.0;
    sum.w += sign * quaternion.w;
    sum.x += sign * quaternion.x;
    sum.y += sign * quaternion.y;
    sum.z += sign * quaternion.z;
  }
  return calibration::normalize_quaternion(sum);
}

float f32(const core::Image& image, std::size_t row, std::size_t col) {
  return reinterpret_cast<const float*>(
      static_cast<const std::byte*>(image.data()) + row * image.stride())[col];
}

std::uint8_t g8(const core::Image& image, std::size_t row, std::size_t col) {
  return *(static_cast<const std::uint8_t*>(image.data()) + row * image.stride() + col);
}

std::optional<double> bilinear_float(const core::Image& image, double row, double col) {
  const int r0 = static_cast<int>(std::floor(row));
  const int c0 = static_cast<int>(std::floor(col));
  if (r0 < 0 || c0 < 0 || r0 + 1 >= static_cast<int>(image.height()) ||
      c0 + 1 >= static_cast<int>(image.width())) {
    return std::nullopt;
  }
  const double fr = row - static_cast<double>(r0);
  const double fc = col - static_cast<double>(c0);
  const double p00 = f32(image, static_cast<std::size_t>(r0), static_cast<std::size_t>(c0));
  const double p01 = f32(image, static_cast<std::size_t>(r0), static_cast<std::size_t>(c0 + 1));
  const double p10 = f32(image, static_cast<std::size_t>(r0 + 1), static_cast<std::size_t>(c0));
  const double p11 = f32(image, static_cast<std::size_t>(r0 + 1), static_cast<std::size_t>(c0 + 1));
  if (!std::isfinite(p00) || !std::isfinite(p01) || !std::isfinite(p10) || !std::isfinite(p11)) {
    return std::nullopt;
  }
  return (1.0 - fr) * ((1.0 - fc) * p00 + fc * p01) +
         fr * ((1.0 - fc) * p10 + fc * p11);
}

}  // namespace

void add_structured_light_calibration_observation(StructuredLightModel& model,
                                                  std::int64_t calib_object_index,
                                                  std::int64_t pose_index,
                                                  const core::Tuple& projector_row,
                                                  const core::Tuple& projector_column,
                                                  const core::Tuple& index) {
  if (calib_object_index < 0 || pose_index < 0) {
    throw core::Exception(core::Status::InvalidArgument,
                          "add_structured_light_calibration_observation",
                          "indices must be nonnegative");
  }
  if (projector_row.size() != projector_column.size() || projector_row.size() != index.size() ||
      projector_row.size() < 4) {
    throw core::Exception(core::Status::InvalidArgument,
                          "add_structured_light_calibration_observation",
                          "row/column/index must have equal length of at least four");
  }

  StructuredLightCalibrationObservation candidate;
  candidate.calib_object_index = calib_object_index;
  candidate.pose_index = pose_index;
  candidate.projector_rows = numeric(projector_row, "add_structured_light_calibration_observation");
  candidate.projector_columns = numeric(projector_column, "add_structured_light_calibration_observation");
  candidate.indices = indices(index, "add_structured_light_calibration_observation");
  model.calibration_observations_[{calib_object_index, pose_index}] = std::move(candidate);
}

void sample_structured_light_calibration_observation(
    StructuredLightModel& model,
    const calibration::CalibrationDataModel& camera_calibration,
    std::int64_t calib_object_index,
    std::int64_t pose_index) {
  if (!model.has_decoded_state_ || model.decoded_correspondence_.size() != 2) {
    throw core::Exception(core::Status::InvalidState,
                          "sample_structured_light_calibration_observation",
                          "structured-light calibration requires decoded both-orientation correspondence");
  }

  const auto& observation = camera_calibration.observation({0, calib_object_index, pose_index});
  std::vector<double> projector_rows;
  std::vector<double> projector_columns;
  std::vector<std::int64_t> point_indices;
  projector_rows.reserve(observation.rows.size());
  projector_columns.reserve(observation.rows.size());
  point_indices.reserve(observation.rows.size());

  for (std::size_t i = 0; i < observation.rows.size(); ++i) {
    const int row = static_cast<int>(std::lround(observation.rows[i]));
    const int col = static_cast<int>(std::lround(observation.columns[i]));
    if (row < 0 || col < 0 || row >= static_cast<int>(model.decoded_validity_.height()) ||
        col >= static_cast<int>(model.decoded_validity_.width()) ||
        g8(model.decoded_validity_, static_cast<std::size_t>(row), static_cast<std::size_t>(col)) == 0) {
      continue;
    }
    const auto projector_column = bilinear_float(
        model.decoded_correspondence_.at(0).as_image(), observation.rows[i], observation.columns[i]);
    const auto projector_row = bilinear_float(
        model.decoded_correspondence_.at(1).as_image(), observation.rows[i], observation.columns[i]);
    if (!projector_column.has_value() || !projector_row.has_value()) continue;
    projector_columns.push_back(*projector_column);
    projector_rows.push_back(*projector_row);
    point_indices.push_back(observation.indices[i]);
  }

  add_structured_light_calibration_observation(model,
                                               calib_object_index,
                                               pose_index,
                                               real_tuple(projector_rows),
                                               real_tuple(projector_columns),
                                               int_tuple(point_indices));
}

CameraSetupModel calibrate_structured_light_setup(
    StructuredLightModel& model,
    const calibration::CalibrationDataModel& camera_calibration,
    calibration::CameraParameters projector_initial,
    std::int64_t calib_object_index,
    double* reprojection_rmse) {
  if (!reprojection_rmse) {
    throw core::Exception(core::Status::InvalidArgument,
                          "calibrate_structured_light_setup",
                          "reprojection output must not be null");
  }
  if (!calibration::is_perspective(projector_initial)) {
    throw core::Exception(core::Status::Unsupported,
                          "calibrate_structured_light_setup",
                          "projector must use a perspective area-scan camera model");
  }
  if (!camera_calibration.camera(0).parameters || !camera_calibration.calibration_result()) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_structured_light_setup",
                          "calibrated camera parameters and poses are required");
  }

  const auto& target = camera_calibration.target(calib_object_index);
  if (target.points.size() < 4) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_structured_light_setup",
                          "calibration target is not configured");
  }

  std::vector<const StructuredLightCalibrationObservation*> observations;
  for (const auto& [key, observation] : model.calibration_observations_) {
    if (key.first == calib_object_index) {
      observations.push_back(&observation);
    }
  }
  if (observations.size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_structured_light_setup",
                          "at least three projector observations are required");
  }

  auto projector = calibration::CalibrationDataModel::create("calibration_object", 1, 1);
  calibration::set_camera_parameters(projector,
                                     0,
                                     calibration::camera_parameters_to_tuple(projector_initial));

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  x.reserve(target.points.size());
  y.reserve(target.points.size());
  z.reserve(target.points.size());
  for (const auto& point : target.points) {
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }
  calibration::set_custom_target_points(projector,
                                        0,
                                        real_tuple(x),
                                        real_tuple(y),
                                        real_tuple(z));

  for (const auto* observation : observations) {
    calibration::set_observation_points(projector,
                                        0,
                                        0,
                                        observation->pose_index,
                                        real_tuple(observation->projector_rows),
                                        real_tuple(observation->projector_columns),
                                        int_tuple(observation->indices),
                                        {});
  }

  double rmse = 0.0;
  calibration::calibrate_cameras(projector, &rmse);

  std::map<std::int64_t, Pose3d> camera_poses;
  for (const auto& [key, pose] : camera_calibration.calibration_result()->poses) {
    if (key.camera_index == 0 && key.calib_object_index == calib_object_index) {
      camera_poses[key.pose_index] = pose;
    }
  }

  std::vector<Pose3d> candidates;
  for (const auto& [key, projector_pose] : projector.calibration_result()->poses) {
    const auto it = camera_poses.find(key.pose_index);
    if (it != camera_poses.end()) {
      candidates.push_back(
          calibration::compose_pose(it->second, calibration::inverse_pose(projector_pose)));
    }
  }
  if (candidates.size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_structured_light_setup",
                          "fewer than three paired camera/projector poses were recovered");
  }

  Point3d translation{};
  std::vector<Quaterniond> rotations;
  rotations.reserve(candidates.size());
  for (const auto& pose : candidates) {
    translation.x += pose.translation.x;
    translation.y += pose.translation.y;
    translation.z += pose.translation.z;
    rotations.push_back(pose.rotation);
  }
  const double inv = 1.0 / static_cast<double>(candidates.size());
  translation.x *= inv;
  translation.y *= inv;
  translation.z *= inv;

  auto setup = CameraSetupModel::create("camera_projector");
  setup.set_camera(0, *camera_calibration.camera(0).parameters);
  setup.set_camera(1, *projector.camera(0).parameters);
  setup.set_device_1_in_device_0_pose({translation, average_quaternions(rotations)});

  model.camera_setup_ = setup;
  *reprojection_rmse = rmse;
  return setup;
}

void set_structured_light_camera_setup(StructuredLightModel& model,
                                       const CameraSetupModel& setup) {
  if (setup.kind() != CameraSetupKind::CameraProjector || !setup.complete()) {
    throw core::Exception(core::Status::InvalidArgument,
                          "set_structured_light_camera_setup",
                          "complete camera_projector setup required");
  }
  if (!calibration::is_perspective(setup.camera(1))) {
    throw core::Exception(core::Status::Unsupported,
                          "set_structured_light_camera_setup",
                          "projector must be perspective");
  }
  model.camera_setup_ = setup;
}

}  // namespace mv::reconstruction
