#pragma once
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <core/geometry_types.hpp>
namespace mv::calibration {
// 每调用一次不可变准备，供 Tuple 与 XLD 点工作流共享。
class WorldPlaneProjection {
 public:
  WorldPlaneProjection(CameraParameters camera, Pose3d camera_in_world, double factor,
                       bool halcon_contract = false);
  static WorldPlaneProjection prepare(const core::Tuple& camera_param, const core::Tuple& world_pose,
                                      const core::Tuple& scale, bool halcon_contract = false);
  [[nodiscard]] Point2d operator()(Point2d point) const;
  void map_tuples(const core::Tuple& row, const core::Tuple& column, core::Tuple* x, core::Tuple* y) const;
  void map_points(std::span<const core::Point2d> input, std::span<core::Point2d> output,
                  std::string_view operation) const;

 private:
  CameraParameters camera_;
  Pose3d camera_in_world_;
  double factor_;
  std::array<double, 9> rotation_;
  bool perspective_;
  bool halcon_contract_;
  Point3d perspective_origin_;
};
void project_3d_point(const core::Tuple& x, const core::Tuple& y, const core::Tuple& z,
                      const core::Tuple& camera_param, const core::Tuple& object_in_camera_pose,
                      core::Tuple* row, core::Tuple* column);
void change_radial_distortion_points(const core::Tuple& row, const core::Tuple& column,
                                     const core::Tuple& camera_param_in, const core::Tuple& camera_param_out,
                                     core::Tuple* row_changed, core::Tuple* column_changed);
void image_points_to_world_plane(const core::Tuple& camera_param, const core::Tuple& world_in_camera_pose,
                                 const core::Tuple& row, const core::Tuple& column, const core::Tuple& scale,
                                 core::Tuple* x, core::Tuple* y);
}  // namespace mv::calibration
