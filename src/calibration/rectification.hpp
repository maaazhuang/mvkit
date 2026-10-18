#pragma once

#include <calibration/camera_parameters.hpp>
#include <core/iconic_tuple.hpp>
#include <core/image.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <cstddef>

namespace mv::calibration {
class WorldPlaneProjection;

void gen_radial_distortion_map(const core::Tuple& camera_param_in, const core::Tuple& camera_param_out,
                               core::Image* row_map, core::Image* col_map);
[[nodiscard]] core::Tuple change_radial_distortion_cam_par(const core::Tuple& camera_param_in,
                                                           const core::Tuple& distortion);
void gen_image_to_world_plane_map(const core::Tuple& camera_param, const core::Tuple& world_pose,
                                  const core::Tuple& scale, std::size_t width, std::size_t height,
                                  core::Image* row_map, core::Image* col_map);
[[nodiscard]] core::XldContours contour_to_world_plane_xld(const core::XldContours& contours,
                                                           const core::Tuple& camera_param,
                                                           const core::Tuple& world_pose,
                                                           const core::Tuple& scale);
[[nodiscard]] core::XldContours change_radial_distortion_contours_xld(const core::XldContours& contours,
                                                                      const core::Tuple& camera_param_in,
                                                                      const core::Tuple& camera_param_out);
// 正式适配一次准备控制，空 iconic 数组也如此。
[[nodiscard]] core::XldContours contour_to_world_plane_xld(const core::XldContours& contours,
                                                           const WorldPlaneProjection& project);
[[nodiscard]] core::XldContours change_radial_distortion_contours_xld(const core::XldContours& contours,
                                                                      const CameraParameters& camera_in,
                                                                      const CameraParameters& camera_out);
[[nodiscard]] double world_plane_meters_per_pixel(const core::Tuple& scale, const char* op);

// 正式编码 Map 和直接校正使用 Double 逆坐标；上方旧双 Float32 Map 保持独立契约。
[[nodiscard]] core::Image gen_radial_distortion_map_halcon(const core::Tuple& in, const core::Tuple& out,
                                                           std::string_view map_type);
[[nodiscard]] core::Image gen_image_to_world_plane_map_halcon(const core::Tuple& camera,
                                                              const core::Tuple& pose, std::size_t width_in,
                                                              std::size_t height_in, std::size_t width,
                                                              std::size_t height, const core::Tuple& scale,
                                                              std::string_view map_type);
[[nodiscard]] core::IconicTuple change_radial_distortion_image_halcon(const core::IconicTuple& images,
                                                                      const core::IconicTuple& regions,
                                                                      const core::Tuple& in,
                                                                      const core::Tuple& out);
[[nodiscard]] core::IconicTuple image_to_world_plane_halcon(const core::IconicTuple& images,
                                                            const core::Tuple& camera,
                                                            const core::Tuple& pose, std::size_t width,
                                                            std::size_t height, const core::Tuple& scale,
                                                            std::string_view interpolation);

}  // namespace mv::calibration
