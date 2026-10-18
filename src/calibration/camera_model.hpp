#pragma once

#include <calibration/camera_parameters.hpp>
#include <core/geometry_types.hpp>

namespace mv::calibration {

struct MetricPoint {
  double u{};
  double v{};
};

struct Ray3d {
  Point3d origin{};
  Point3d direction{0.0, 0.0, 1.0};
};

[[nodiscard]] MetricPoint distort_metric_point(const CameraParameters& params, const MetricPoint& undistorted);
[[nodiscard]] MetricPoint undistort_metric_point(const CameraParameters& params, const MetricPoint& distorted);
[[nodiscard]] Point2d project_camera_point(const CameraParameters& params, const Point3d& camera_point);
[[nodiscard]] Ray3d image_point_to_ray(const CameraParameters& params, const Point2d& image_point);
[[nodiscard]] MetricPoint image_point_to_undistorted_metric(const CameraParameters& params, const Point2d& image_point);
[[nodiscard]] Point2d undistorted_metric_to_image_point(const CameraParameters& params, const MetricPoint& metric);

}  // namespace mv::calibration
