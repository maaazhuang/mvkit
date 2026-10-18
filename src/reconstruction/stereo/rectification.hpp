#pragma once

#include <array>
#include <cstdint>

#include <calibration/camera_parameters.hpp>
#include <core/image.hpp>
#include <core/geometry_types.hpp>
#include <reconstruction/camera_setup.hpp>

namespace mv::reconstruction {

class StereoRectification {
 public:
  StereoRectification(calibration::CameraParameters rect0,
                      calibration::CameraParameters rect1,
                      Pose3d rectified_to_device0,
                      Pose3d rect_rel,
                      CameraSetupModel original_setup,
                      std::array<core::Image,2> map_row,
                      std::array<core::Image,2> map_col)
      : rectified_cameras_{std::move(rect0),std::move(rect1)},
        rectified_to_device0_(rectified_to_device0), rect_rel_(rect_rel),
        original_setup_(std::move(original_setup)), map_row_(std::move(map_row)), map_col_(std::move(map_col)) {}

  [[nodiscard]] const calibration::CameraParameters& camera(std::int64_t index) const;
  [[nodiscard]] const Pose3d& device_1_in_device_0_pose() const noexcept { return rect_rel_; }
  [[nodiscard]] const Pose3d& rectified_to_original_device0_pose() const noexcept { return rectified_to_device0_; }
  [[nodiscard]] const CameraSetupModel& original_setup() const noexcept { return original_setup_; }
  [[nodiscard]] const core::Image& map_row(std::int64_t index) const;
  [[nodiscard]] const core::Image& map_col(std::int64_t index) const;

 private:
  static std::size_t checked_index(std::int64_t index);
  std::array<calibration::CameraParameters,2> rectified_cameras_;
  Pose3d rectified_to_device0_;
  Pose3d rect_rel_;
  CameraSetupModel original_setup_;
  std::array<core::Image,2> map_row_;
  std::array<core::Image,2> map_col_;
};

[[nodiscard]] StereoRectification gen_binocular_rectification_map(const CameraSetupModel& setup);
[[nodiscard]] Point2d rectify_image_point(const StereoRectification& rect,
                                          std::int64_t camera_index,
                                          const Point2d& raw_point);
[[nodiscard]] core::Image remap_image(const core::Image& source,
                                      const core::Image& map_row,
                                      const core::Image& map_col);
void map_binocular_images(const core::Image& left, const core::Image& right,
                          const StereoRectification& rect,
                          core::Image* left_rect, core::Image* right_rect);

}  // namespace mv::reconstruction
