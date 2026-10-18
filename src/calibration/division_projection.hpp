#pragma once

#include <calibration/camera_model.hpp>
#include <cmath>
#include <optional>
#include <type_traits>

namespace mv::calibration {

// 私有逐点内核：参数只准备一次，数值顺序与通用 division 投影相同。
// 映射调用将投影异常转为空像素，此处直接返回空值，避免为孔洞构造异常。
class DivisionProjection {
public:
  explicit DivisionProjection(const CameraParameters& camera)
      : common_(common_parameters(camera)) {
    std::visit([&](const auto& p) {
      using T = std::decay_t<decltype(p)>;
      if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                    std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
        enabled_ = true;
        kappa_ = p.kappa;
        perspective_ = std::is_same_v<T, AreaScanDivisionParameters>;
        if constexpr (std::is_same_v<T, AreaScanDivisionParameters>) scale_ = p.focus;
        else scale_ = p.magnification;
      }
    }, camera);
  }

  bool enabled() const noexcept { return enabled_; }
  bool perspective() const noexcept { return perspective_; }
  bool linear_distortion() const noexcept { return enabled_ && std::abs(kappa_) < 1e-30; }
  bool zero_undistortion() const noexcept { return enabled_ && kappa_ == 0.; }

  std::optional<MetricPoint> undistort(Point2d image) const noexcept {
    if (!std::isfinite(image.row) || !std::isfinite(image.col)) return {};
    const MetricPoint metric{(image.col - common_.cx) * common_.sx,
                             (image.row - common_.cy) * common_.sy};
    if (!finite(metric)) return {};
    const double r2 = metric.u * metric.u + metric.v * metric.v;
    const double denominator = 1.0 + kappa_ * r2;
    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-15) return {};
    return MetricPoint{metric.u / denominator, metric.v / denominator};
  }

  std::optional<Point2d> distort(MetricPoint metric) const noexcept {
    if (!finite(metric)) return {};
    if (!(std::abs(kappa_) < 1e-30)) {
      const double r2 = metric.u * metric.u + metric.v * metric.v;
      const double radicand = 1.0 - 4.0 * kappa_ * r2;
      if (radicand < 0.0 || !std::isfinite(radicand)) return {};
      const double denominator = 1.0 + std::sqrt(radicand);
      if (denominator <= 0.0) return {};
      const double factor = 2.0 / denominator;
      metric = {metric.u * factor, metric.v * factor};
    }
    return Point2d{metric.v / common_.sy + common_.cy,
                   metric.u / common_.sx + common_.cx};
  }

  std::optional<Point2d> project(Point3d point) const noexcept {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return {};
    if (perspective_) {
      if (point.z <= 1e-12) return {};
      return distort({scale_ * point.x / point.z, scale_ * point.y / point.z});
    }
    return distort({scale_ * point.x, scale_ * point.y});
  }

private:
  static bool finite(MetricPoint p) noexcept {
    return std::isfinite(p.u) && std::isfinite(p.v);
  }
  CommonCameraParameters common_;
  double kappa_{}, scale_{};
  bool enabled_{}, perspective_{};
};
} // namespace mv::calibration
