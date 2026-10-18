#pragma once

#include <calibration/camera_model.hpp>
#include <core/geometry_types.hpp>

namespace mv::reconstruction {

struct Ray3d {
  Point3d origin{};
  Point3d direction{0.0, 0.0, 1.0};
};

struct TriangulationResult {
  Point3d point{};
  double ray_gap{};
  double depth0{};
  double depth1{};
};

[[nodiscard]] Ray3d transform_ray(const Pose3d& pose, const Ray3d& ray);
[[nodiscard]] Ray3d transform_ray(const Pose3d& pose, const calibration::Ray3d& ray);
[[nodiscard]] TriangulationResult triangulate_rays(const Ray3d& ray0,
                                                   const Ray3d& ray1,
                                                   double max_ray_gap);

}  // namespace mv::reconstruction
