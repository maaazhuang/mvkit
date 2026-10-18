#include <array>
#include <calibration/geometry3d.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <reconstruction/camera_setup.hpp>

namespace mv::reconstruction {
namespace {
double normalized_angle(double value) {
  value = std::fmod(value, 360.);
  return value < 0 ? value + 360. : value;
}

// 存储/查询遵循 IEEE 传播；非有限位姿不提供重建所需有限几何缓存。
core::Tuple nonfinite_standard_pose(const core::Tuple& pose, std::int64_t code) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> angles{nan, nan, nan};
  std::array<double, 9> rotation;
  rotation.fill(nan);
  bool finite_angles = true;
  for (std::size_t j = 3; j < 6; ++j) finite_angles = std::isfinite(pose.as_real64(j)) && finite_angles;
  if (finite_angles) {
    core::Tuple sanitized;
    for (int j = 0; j < 3; ++j) sanitized.append(0.);
    for (std::size_t j = 3; j < 7; ++j) sanitized.append(pose.at(j));
    const auto converted = calibration::pose_from_halcon_tuple(sanitized);
    rotation = calibration::rotation_matrix(converted.rotation);
    const auto standard = calibration::pose_to_halcon_tuple(converted);
    for (std::size_t j = 0; j < 3; ++j) angles[j] = normalized_angle(standard.as_real64(j + 3));
  } else if ((code & 6) == 0) {
    constexpr auto radians = 3.14159265358979323846 / 180.;
    for (std::size_t j = 0; j < 3; ++j) {
      const auto value = pose.as_real64(j + 3);
      angles[j] =
          code & 1 ? normalized_angle(-value) : (std::isfinite(value) ? normalized_angle(value) : value);
    }
    const auto sx = std::sin(angles[0] * radians), cx = std::cos(angles[0] * radians);
    const auto sy = std::sin(angles[1] * radians), cy = std::cos(angles[1] * radians);
    const auto sz = std::sin(angles[2] * radians), cz = std::cos(angles[2] * radians);
    rotation = {cy * cz,
                -cy * sz,
                sy,
                cx * sz + sx * sy * cz,
                cx * cz - sx * sy * sz,
                -sx * cy,
                sx * sz - cx * sy * cz,
                sx * cz + cx * sy * sz,
                cx * cy};
  }
  core::Tuple result;
  for (std::size_t j = 0; j < 3; ++j) {
    double value = pose.as_real64(j);
    if (code & 8)
      value = -(rotation[j * 3] * pose.as_real64(0) + rotation[j * 3 + 1] * pose.as_real64(1) +
                rotation[j * 3 + 2] * pose.as_real64(2));
    result.append(value);
  }
  for (const auto angle : angles) result.append(angle);
  result.append(std::int64_t{0});
  return result;
}
}  // namespace

CameraSetupModel CameraSetupModel::create(std::string_view setup_type) {
  if (setup_type == "camera_camera") return CameraSetupModel(CameraSetupKind::CameraCamera);
  if (setup_type == "camera_projector") return CameraSetupModel(CameraSetupKind::CameraProjector);
  throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::create", "unsupported setup type");
}

CameraSetupModel CameraSetupModel::create(std::int64_t num_cameras) {
  if (num_cameras <= 0)
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::create",
                          "camera count must be positive");
  return CameraSetupModel(CameraSetupKind::CameraCamera, static_cast<std::size_t>(num_cameras));
}

std::size_t CameraSetupModel::checked_index(std::int64_t index) const {
  if (index < 0 || static_cast<std::uint64_t>(index) >= cameras_.size()) {
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::camera",
                          "camera index outside setup");
  }
  return static_cast<std::size_t>(index);
}

void CameraSetupModel::set_camera(std::int64_t index, calibration::CameraParameters params,
                                  const core::Tuple& pose) {
  const auto i = checked_index(index);
  if (pose.size() != 7)
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::set_camera",
                          "pose requires seven values");
  const auto code = pose.as_int64(6);
  if (code < 0 || code > 13 || (code & 6) == 6)
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::set_camera",
                          "invalid pose representation code");
  bool finite = true;
  for (std::size_t j = 0; j < 6; ++j) finite = std::isfinite(pose.as_real64(j)) && finite;
  std::optional<Pose3d> converted;
  if (finite) converted = calibration::pose_from_halcon_tuple(pose);
  core::Tuple reported;
  if (code == 0 || code == 1) {
    std::array<double,6> values;
    for (std::size_t j = 0; j < 6; ++j) {
      auto value = pose.as_real64(j);
      if (j >= 3) {
        value = std::fmod(code == 1 ? -value : value, 360.);
        if (value < 0) value += 360.;
      }
      values[j]=value;
    }
    // 六个实数和一个整数一次建立混合元组，避免追加整数时复制整组实数。
    reported=core::Tuple{core::Value{values[0]},core::Value{values[1]},core::Value{values[2]},
                        core::Value{values[3]},core::Value{values[4]},core::Value{values[5]},core::Value{std::int64_t{0}}};
  } else if (finite) {
    const auto standard = calibration::pose_to_halcon_tuple(*converted);
    for (std::size_t j = 0; j < 6; ++j) {
      auto value = standard.as_real64(j);
      if (j >= 3) {
        value = std::fmod(value, 360.);
        if (value < 0) value += 360.;
      }
      reported.append(value);
    }
    reported.append(std::int64_t{0});
  } else {
    reported = nonfinite_standard_pose(pose, code);
  }
  std::optional<Pose3d> relative = relative_pose_;
  if (i < 2 && cameras_.size() >= 2) {
    const auto& p0 = i == 0 ? converted : poses_[0];
    const auto& p1 = i == 1 ? converted : poses_[1];
    relative.reset();
    if (p0 && p1) relative = calibration::compose_pose(calibration::inverse_pose(*p0), *p1);
  }
  // 全部验证及分配先于共享模型修改。
  set_camera(index, std::move(params));
  poses_[i] = converted;
  reported_poses_[i] = std::move(reported);
  relative_pose_ = relative;
}

void CameraSetupModel::set_camera(std::int64_t index, calibration::CameraParameters params) {
  const auto i = checked_index(index);
  if (kind_ == CameraSetupKind::CameraProjector && i == 1 && !calibration::is_perspective(params)) {
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::set_camera",
                          "projector must use a perspective area-scan model");
  }
  cameras_[i] = std::move(params);
}

void CameraSetupModel::set_device_1_in_device_0_pose(Pose3d pose) {
  pose.rotation = calibration::normalize_quaternion(pose.rotation);
  if (!std::isfinite(pose.translation.x) || !std::isfinite(pose.translation.y) ||
      !std::isfinite(pose.translation.z)) {
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::set_device_1_in_device_0_pose",
                          "relative translation must be finite");
  }
  if (cameras_.size() != 2)
    throw core::Exception(core::Status::InvalidArgument, "CameraSetupModel::set_device_1_in_device_0_pose",
                          "relative-pose convenience interface requires two cameras");
  auto first = calibration::pose_to_halcon_tuple(Pose3d{});
  auto second = calibration::pose_to_halcon_tuple(pose);
  poses_[0] = Pose3d{};
  poses_[1] = pose;
  reported_poses_[0] = std::move(first);
  reported_poses_[1] = std::move(second);
  relative_pose_ = pose;
}

const core::Tuple& CameraSetupModel::camera_pose(std::int64_t index) const {
  const auto i = checked_index(index);
  (void)camera(index);
  if (reported_poses_[i].empty())
    throw core::Exception(core::Status::InvalidState, "CameraSetupModel::camera_pose",
                          "camera pose is not set");
  return reported_poses_[i];
}

const calibration::CameraParameters& CameraSetupModel::camera(std::int64_t index) const {
  const auto i = checked_index(index);
  if (!cameras_[i].has_value()) {
    throw core::Exception(core::Status::InvalidState, "CameraSetupModel::camera",
                          "camera parameters are not set");
  }
  return *cameras_[i];
}

const Pose3d& CameraSetupModel::device_1_in_device_0_pose() const {
  if (!relative_pose_.has_value()) {
    throw core::Exception(core::Status::InvalidState, "CameraSetupModel::device_1_in_device_0_pose",
                          "relative pose is not set");
  }
  return *relative_pose_;
}

bool CameraSetupModel::complete() const noexcept {
  return cameras_.size() == 2 && cameras_[0].has_value() && cameras_[1].has_value() &&
         relative_pose_.has_value();
}

}  // namespace mv::reconstruction
