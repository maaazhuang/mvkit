#include <calibration/camera_model.hpp>

#include <algorithm>
#include <cmath>
#include <type_traits>

#include <core/exception.hpp>

namespace mv::calibration {
namespace {
constexpr double kPerspectiveZEps = 1e-12;
constexpr int kNewtonIterations = 30;
constexpr double kNewtonTolerance = 1e-14;

struct PolynomialCoefficients {
  double k1{}, k2{}, k3{}, p1{}, p2{};
};

PolynomialCoefficients polynomial_coefficients(const CameraParameters& params) {
  return std::visit([](const auto& p) -> PolynomialCoefficients {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanPolynomialParameters> ||
                  std::is_same_v<T, AreaScanTelecentricPolynomialParameters>) {
      return {p.k1, p.k2, p.k3, p.p1, p.p2};
    } else {
      return {};
    }
  }, params);
}

double division_kappa(const CameraParameters& params) {
  return std::visit([](const auto& p) -> double {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                  std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
      return p.kappa;
    } else {
      return 0.0;
    }
  }, params);
}

bool is_polynomial(const CameraParameters& params) {
  return std::holds_alternative<AreaScanPolynomialParameters>(params) ||
         std::holds_alternative<AreaScanTelecentricPolynomialParameters>(params);
}

MetricPoint polynomial_undistort(const MetricPoint& d, const PolynomialCoefficients& c) {
  const double r2 = d.u * d.u + d.v * d.v;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double radial = c.k1 * r2 + c.k2 * r4 + c.k3 * r6;
  // HALCON 按图像轴命名偏心系数：p1 作用于水平度量坐标，p2 作用于垂直坐标，
  // 元组顺序与常用 Brown/OpenCV 约定相反。
  return {
      d.u + d.u * radial + c.p1 * (r2 + 2.0 * d.u * d.u) + 2.0 * c.p2 * d.u * d.v,
      d.v + d.v * radial + 2.0 * c.p1 * d.u * d.v + c.p2 * (r2 + 2.0 * d.v * d.v),
  };
}

MetricPoint polynomial_distort(const MetricPoint& target, const PolynomialCoefficients& c) {
  MetricPoint x = target;
  for (int iter = 0; iter < kNewtonIterations; ++iter) {
    const auto f = polynomial_undistort(x, c);
    const double e0 = f.u - target.u;
    const double e1 = f.v - target.v;
    if (std::max(std::abs(e0), std::abs(e1)) <= kNewtonTolerance) return x;

    const double scale = std::max({1.0, std::abs(x.u), std::abs(x.v)});
    const double h = 1e-8 * scale;
    const auto fu = polynomial_undistort({x.u + h, x.v}, c);
    const auto fv = polynomial_undistort({x.u, x.v + h}, c);
    const double j00 = (fu.u - f.u) / h;
    const double j10 = (fu.v - f.v) / h;
    const double j01 = (fv.u - f.u) / h;
    const double j11 = (fv.v - f.v) / h;
    const double det = j00 * j11 - j01 * j10;
    if (!std::isfinite(det) || std::abs(det) < 1e-20) {
      throw core::Exception(core::Status::InvalidState, "distort_metric_point", "polynomial distortion inversion is singular");
    }
    const double du = ( j11 * e0 - j01 * e1) / det;
    const double dv = (-j10 * e0 + j00 * e1) / det;
    x.u -= du;
    x.v -= dv;
    if (!std::isfinite(x.u) || !std::isfinite(x.v)) {
      throw core::Exception(core::Status::InvalidState, "distort_metric_point", "polynomial distortion inversion diverged");
    }
  }
  throw core::Exception(core::Status::InvalidState, "distort_metric_point", "polynomial distortion inversion did not converge");
}

MetricPoint pixel_to_distorted_metric(const CommonCameraParameters& c, const Point2d& image) {
  return {(image.col - c.cx) * c.sx, (image.row - c.cy) * c.sy};
}

Point2d distorted_metric_to_pixel(const CommonCameraParameters& c, const MetricPoint& metric) {
  return {metric.v / c.sy + c.cy, metric.u / c.sx + c.cx};
}
}  // namespace

MetricPoint undistort_metric_point(const CameraParameters& params, const MetricPoint& distorted) {
  if (!std::isfinite(distorted.u) || !std::isfinite(distorted.v)) {
    throw core::Exception(core::Status::InvalidArgument, "undistort_metric_point", "metric point must be finite");
  }
  if (is_polynomial(params)) return polynomial_undistort(distorted, polynomial_coefficients(params));
  const double kappa = division_kappa(params);
  const double r2 = distorted.u * distorted.u + distorted.v * distorted.v;
  const double denom = 1.0 + kappa * r2;
  if (!std::isfinite(denom) || std::abs(denom) < 1e-15) {
    throw core::Exception(core::Status::InvalidState, "undistort_metric_point", "division distortion is outside its valid domain");
  }
  return {distorted.u / denom, distorted.v / denom};
}

MetricPoint distort_metric_point(const CameraParameters& params, const MetricPoint& undistorted) {
  if (!std::isfinite(undistorted.u) || !std::isfinite(undistorted.v)) {
    throw core::Exception(core::Status::InvalidArgument, "distort_metric_point", "metric point must be finite");
  }
  if (is_polynomial(params)) return polynomial_distort(undistorted, polynomial_coefficients(params));
  const double kappa = division_kappa(params);
  if (std::abs(kappa) < 1e-30) return undistorted;
  const double r2 = undistorted.u * undistorted.u + undistorted.v * undistorted.v;
  const double radicand = 1.0 - 4.0 * kappa * r2;
  if (radicand < 0.0 || !std::isfinite(radicand)) {
    throw core::Exception(core::Status::InvalidState, "distort_metric_point", "division distortion is outside its valid domain");
  }
  const double denom = 1.0 + std::sqrt(radicand);
  if (denom <= 0.0) {
    throw core::Exception(core::Status::InvalidState, "distort_metric_point", "division distortion has no finite inverse");
  }
  const double scale = 2.0 / denom;
  return {undistorted.u * scale, undistorted.v * scale};
}


MetricPoint image_point_to_undistorted_metric(const CameraParameters& params, const Point2d& image_point) {
  if (!std::isfinite(image_point.row) || !std::isfinite(image_point.col)) {
    throw core::Exception(core::Status::InvalidArgument, "image_point_to_undistorted_metric", "image point must be finite");
  }
  return undistort_metric_point(params, pixel_to_distorted_metric(common_parameters(params), image_point));
}

Point2d undistorted_metric_to_image_point(const CameraParameters& params, const MetricPoint& metric) {
  return distorted_metric_to_pixel(common_parameters(params), distort_metric_point(params, metric));
}

Point2d project_camera_point(const CameraParameters& params, const Point3d& camera_point) {
  if (!std::isfinite(camera_point.x) || !std::isfinite(camera_point.y) || !std::isfinite(camera_point.z)) {
    throw core::Exception(core::Status::InvalidArgument, "project_camera_point", "camera point must be finite");
  }
  MetricPoint ideal;
  std::visit([&](const auto& p) {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters> || std::is_same_v<T, AreaScanPolynomialParameters>) {
      if (camera_point.z <= kPerspectiveZEps) {
        throw core::Exception(core::Status::InvalidArgument, "project_camera_point", "perspective camera point must be in front of camera");
      }
      ideal = {p.focus * camera_point.x / camera_point.z, p.focus * camera_point.y / camera_point.z};
    } else {
      ideal = {p.magnification * camera_point.x, p.magnification * camera_point.y};
    }
  }, params);
  return distorted_metric_to_pixel(common_parameters(params), distort_metric_point(params, ideal));
}

Ray3d image_point_to_ray(const CameraParameters& params, const Point2d& image_point) {
  if (!std::isfinite(image_point.row) || !std::isfinite(image_point.col)) {
    throw core::Exception(core::Status::InvalidArgument, "image_point_to_ray", "image point must be finite");
  }
  const auto distorted = pixel_to_distorted_metric(common_parameters(params), image_point);
  const auto ideal = undistort_metric_point(params, distorted);
  return std::visit([&](const auto& p) -> Ray3d {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters> || std::is_same_v<T, AreaScanPolynomialParameters>) {
      return {{0.0, 0.0, 0.0}, {ideal.u / p.focus, ideal.v / p.focus, 1.0}};
    } else {
      return {{ideal.u / p.magnification, ideal.v / p.magnification, 0.0}, {0.0, 0.0, 1.0}};
    }
  }, params);
}

}  // namespace mv::calibration
