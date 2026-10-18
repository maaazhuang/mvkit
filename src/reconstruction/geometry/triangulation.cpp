#include <reconstruction/geometry/triangulation.hpp>

#include <cmath>

#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

double dot(const Point3d& a, const Point3d& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Point3d sub(const Point3d& a, const Point3d& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Point3d add(const Point3d& a, const Point3d& b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Point3d mul(const Point3d& a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double norm(const Point3d& a) { return std::sqrt(dot(a,a)); }
Point3d normalized(const Point3d& a) {
  const double n = norm(a);
  if (!std::isfinite(n) || n <= 1e-15) {
    throw core::Exception(core::Status::InvalidArgument, "triangulate_rays", "ray direction must be nonzero");
  }
  return mul(a, 1.0/n);
}

}  // namespace

Ray3d transform_ray(const Pose3d& pose, const Ray3d& ray) {
  return {calibration::transform_point(pose, ray.origin),
          calibration::rotate_point(pose.rotation, ray.direction)};
}

Ray3d transform_ray(const Pose3d& pose, const calibration::Ray3d& ray) {
  return transform_ray(pose, Ray3d{ray.origin, ray.direction});
}

TriangulationResult triangulate_rays(const Ray3d& input0,
                                     const Ray3d& input1,
                                     double max_ray_gap) {
  if (!std::isfinite(max_ray_gap) || max_ray_gap < 0.0) {
    throw core::Exception(core::Status::InvalidArgument, "triangulate_rays", "max ray gap must be finite and nonnegative");
  }
  const Point3d u = normalized(input0.direction);
  const Point3d v = normalized(input1.direction);
  const Point3d w0 = sub(input0.origin, input1.origin);
  const double a = 1.0;
  const double b = dot(u, v);
  const double c = 1.0;
  const double d = dot(u, w0);
  const double e = dot(v, w0);
  const double den = a*c - b*b;
  if (!std::isfinite(den) || std::abs(den) < 1e-12) {
    throw core::Exception(core::Status::InvalidState, "triangulate_rays", "rays are parallel or nearly parallel");
  }
  const double s = (b*e - c*d) / den;
  const double t = (a*e - b*d) / den;
  if (!(s > 0.0) || !(t > 0.0)) {
    throw core::Exception(core::Status::InvalidState, "triangulate_rays", "triangulated point is behind a ray origin");
  }
  const Point3d p0 = add(input0.origin, mul(u, s));
  const Point3d p1 = add(input1.origin, mul(v, t));
  const double gap = norm(sub(p0, p1));
  if (!std::isfinite(gap) || gap > max_ray_gap) {
    throw core::Exception(core::Status::InvalidState, "triangulate_rays", "closest-ray gap exceeds tolerance");
  }
  return {mul(add(p0,p1),0.5), gap, s, t};
}

}  // namespace mv::reconstruction
