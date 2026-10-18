#include <reconstruction/stereo/setup_calibration.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

double dot(const Point3d& a, const Point3d& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }

Point3d normal_in_camera(const Pose3d& pose) {
  return calibration::rotate_point(pose.rotation, {0.0,0.0,1.0});
}

void require_view_diversity(const std::vector<Pose3d>& poses) {
  double min_dot = 1.0;
  for (std::size_t i=0;i<poses.size();++i) {
    const auto ni = normal_in_camera(poses[i]);
    for (std::size_t j=i+1;j<poses.size();++j) {
      min_dot = std::min(min_dot, dot(ni, normal_in_camera(poses[j])));
    }
  }
  if (min_dot > std::cos(1.0 * 3.14159265358979323846 / 180.0)) {
    throw core::Exception(core::Status::InvalidState, "calibrate_binocular_camera_setup",
                          "paired calibration views lack orientation diversity");
  }
}

Quaterniond average_quaternions(const std::vector<Quaterniond>& qs) {
  if (qs.empty()) return {};
  const auto q0 = calibration::normalize_quaternion(qs.front());
  Quaterniond sum{};
  sum.w = sum.x = sum.y = sum.z = 0.0;
  for (auto q : qs) {
    q = calibration::normalize_quaternion(q);
    const double sign = (q0.w*q.w + q0.x*q.x + q0.y*q.y + q0.z*q.z) < 0.0 ? -1.0 : 1.0;
    sum.w += sign*q.w; sum.x += sign*q.x; sum.y += sign*q.y; sum.z += sign*q.z;
  }
  return calibration::normalize_quaternion(sum);
}

}  // namespace

CameraSetupModel calibrate_binocular_camera_setup(
    const calibration::CalibrationDataModel& camera0,
    const calibration::CalibrationDataModel& camera1,
    std::int64_t calib_object_index) {
  if (!camera0.camera(0).parameters || !camera1.camera(0).parameters) {
    throw core::Exception(core::Status::InvalidState, "calibrate_binocular_camera_setup",
                          "both camera parameters must be calibrated");
  }
  if (!camera0.calibration_result() || !camera1.calibration_result()) {
    throw core::Exception(core::Status::InvalidState, "calibrate_binocular_camera_setup",
                          "both calibration results are required");
  }

  std::map<std::int64_t, Pose3d> left_poses;
  for (const auto& [key, value] : camera0.calibration_result()->poses) {
    if (key.camera_index == 0 && key.calib_object_index == calib_object_index) left_poses[key.pose_index] = value;
  }

  std::vector<Pose3d> paired_left;
  std::vector<Pose3d> candidates;
  for (const auto& [key, right_pose] : camera1.calibration_result()->poses) {
    if (key.camera_index != 0 || key.calib_object_index != calib_object_index) continue;
    const auto it = left_poses.find(key.pose_index);
    if (it == left_poses.end()) continue;
    paired_left.push_back(it->second);
    candidates.push_back(calibration::compose_pose(it->second, calibration::inverse_pose(right_pose)));
  }
  if (candidates.size() < 3) {
    throw core::Exception(core::Status::InvalidState, "calibrate_binocular_camera_setup",
                          "at least three paired calibration poses are required");
  }
  require_view_diversity(paired_left);

  Point3d t{};
  std::vector<Quaterniond> qs;
  qs.reserve(candidates.size());
  for (const auto& p : candidates) {
    t.x += p.translation.x; t.y += p.translation.y; t.z += p.translation.z;
    qs.push_back(p.rotation);
  }
  const double inv = 1.0 / static_cast<double>(candidates.size());
  t.x*=inv; t.y*=inv; t.z*=inv;

  auto setup = CameraSetupModel::create("camera_camera");
  setup.set_camera(0, *camera0.camera(0).parameters);
  setup.set_camera(1, *camera1.camera(0).parameters);
  setup.set_device_1_in_device_0_pose({t, average_quaternions(qs)});
  return setup;
}

}  // namespace mv::reconstruction
