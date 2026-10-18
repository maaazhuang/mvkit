#pragma once

#include <calibration/camera_parameters.hpp>
#include <cstdint>
#include <core/geometry_types.hpp>
#include <optional>
#include <string_view>
#include <vector>

namespace mv::reconstruction {

enum class CameraSetupKind { CameraCamera, CameraProjector };

class CameraSetupModel {
 public:
  static CameraSetupModel create(std::string_view setup_type);
  static CameraSetupModel create(std::int64_t num_cameras);

  void set_camera(std::int64_t index, calibration::CameraParameters params);
  void set_camera(std::int64_t index, calibration::CameraParameters params, const core::Tuple& pose);
  void set_device_1_in_device_0_pose(Pose3d pose);

  [[nodiscard]] CameraSetupKind kind() const noexcept { return kind_; }
  [[nodiscard]] const calibration::CameraParameters& camera(std::int64_t index) const;
  [[nodiscard]] const core::Tuple& camera_pose(std::int64_t index) const;
  [[nodiscard]] std::size_t camera_count() const noexcept { return cameras_.size(); }
  [[nodiscard]] const Pose3d& device_1_in_device_0_pose() const;
  [[nodiscard]] bool complete() const noexcept;

 private:
  explicit CameraSetupModel(CameraSetupKind kind, std::size_t count = 2)
      : kind_(kind), cameras_(count), poses_(count), reported_poses_(count) {}
  std::size_t checked_index(std::int64_t index) const;

  CameraSetupKind kind_;
  std::vector<std::optional<calibration::CameraParameters>> cameras_;
  std::vector<std::optional<Pose3d>> poses_;
  std::vector<core::Tuple> reported_poses_;
  std::optional<Pose3d> relative_pose_;
};

}  // namespace mv::reconstruction
