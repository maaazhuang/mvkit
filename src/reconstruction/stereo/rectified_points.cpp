#include <calibration/geometry3d.hpp>
#include <array>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <reconstruction/stereo/rectified_points.hpp>
#include <string_view>
namespace mv::reconstruction {
namespace {
constexpr auto op = "rectified_disparity_point";
double number(const core::Tuple& t, std::size_t i) { return t.as_real64(i); }
void invalid(std::string_view message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "Camera", -1,
                        message);
}
Point3d cross(Point3d a, Point3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double dot(Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Point3d sub(Point3d a, Point3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point3d ray_direction(const RectifiedCamera& c, double row, double column) {
  return c.perspective ? Point3d{(column - c.cx) * c.sx / c.scale, (row - c.cy) * c.sy / c.scale, 1.}
                       : Point3d{0, 0, 1};
}
Point3d ray_origin(const RectifiedCamera& c, double row, double column) {
  return c.perspective ? Point3d{}
                       : Point3d{(column - c.cx) * c.sx / c.scale, (row - c.cy) * c.sy / c.scale, 0};
}
Point3d nan_point() {
  const auto n = std::numeric_limits<double>::quiet_NaN();
  return {n, n, n};
}
}  // namespace
RectifiedCamera parse_rectified_camera(const core::Tuple& input) {
  if (input.empty()) invalid("camera parameters must not be empty");
  std::size_t start = 0, common = 2, distortion = 1, count = 1;
  bool perspective = true, legacy = false;
  if (input.at(0).kind() == core::ValueKind::String) {
    const auto type = input.string_view(0);
    start = 1;
    if (type == "area_scan_division" || type == "area_scan_telecentric_division") {
      common = 3;
      distortion = 2;
      count = 1;
    } else if (type == "area_scan_polynomial" || type == "area_scan_telecentric_polynomial") {
      common = 7;
      distortion = 2;
      count = 5;
    } else
      throw core::Exception(core::Status::Unsupported, core::ErrorCode::UnsupportedValue, op, "Camera", 0,
                            "unsupported camera model");
    perspective = type == "area_scan_division" || type == "area_scan_polynomial";
  } else {
    legacy = true;
  }
  if (input.size() != common + 6) invalid("camera parameter tuple length does not match the model");
  double scale = number(input, start);
  if (legacy && scale == 0) {
    perspective = false;
    scale = 1.;
  }
  if (scale <= 0) invalid("positive focus or magnification required");
  for (std::size_t i = 0; i < count; ++i)
    if (number(input, distortion + i) != 0.) invalid("rectified cameras must have zero distortion");
  RectifiedCamera c{perspective,
                    scale,
                    number(input, common),
                    number(input, common + 1),
                    number(input, common + 2),
                    number(input, common + 3)};
  if (c.sx <= 0 || c.sy <= 0 || number(input, common + 4) <= 0 || number(input, common + 5) <= 0)
    invalid("positive pixel size and image dimensions required");
  return c;
}
Pose3d parse_rectified_pose(const core::Tuple& input, bool* nonfinite_rotation) {
  if (input.size() != 7)
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, op, "Pose", -1,
                          "seven pose controls required");
  const double code = number(input, 6);
  if (!std::isfinite(code) || code < 0 || code >= 14) invalid("valid pose representation required");
  std::array<double,6> values;
  *nonfinite_rotation = false;
  for (std::size_t i = 0; i < 6; ++i) {
    const auto value = number(input, i);
    if (i >= 3 && !std::isfinite(value)) *nonfinite_rotation = true;
    values[i]=value;
  }
  const auto finite_value=[&](std::size_t i){return std::isfinite(values[i])?values[i]:0.;};
  // 一次构造六实数、一整数，避免反复扩容以及实数存储转混合存储时整组复制。
  const core::Tuple sanitized{core::Value{finite_value(0)},core::Value{finite_value(1)},
    core::Value{finite_value(2)},core::Value{finite_value(3)},core::Value{finite_value(4)},
    core::Value{finite_value(5)},core::Value{static_cast<std::int64_t>(code)}};
  auto result = calibration::pose_from_halcon_tuple(sanitized);
  // 有限位姿保留正式表示约定；本算子传播非有限平移/旋转，不拒绝。
  bool nonfinite_translation = false;
  for (std::size_t i = 0; i < 3; ++i) nonfinite_translation |= !std::isfinite(values[i]);
  if (nonfinite_translation) result.translation = {values[0],values[1],values[2]};
  return result;
}
Point3d rectified_disparity_point(const RectifiedCamera& first, const RectifiedCamera& second,
                                  const Pose3d& pose, bool nonfinite_rotation, double row, double column,
                                  double disparity) {
  const auto finite_camera = [](const RectifiedCamera& c) {
    return std::isfinite(c.scale) && std::isfinite(c.sx) && std::isfinite(c.sy) && std::isfinite(c.cx) &&
           std::isfinite(c.cy);
  };
  if (nonfinite_rotation || !finite_camera(first) || !finite_camera(second) || !std::isfinite(row) ||
      !std::isfinite(column) || !std::isfinite(disparity))
    return nan_point();
  const auto u = ray_direction(first, row, column);
  const auto v = calibration::rotate_point(pose.rotation, ray_direction(second, row, column + disparity));
  const auto a = ray_origin(first, row, column);
  const auto b = calibration::transform_point(pose, ray_origin(second, row, column + disparity));
  const auto n = cross(u, v), w = sub(b, a);
  const auto denominator = dot(n, n);
  if (denominator == 0 || !std::isfinite(denominator)) return nan_point();
  // 叉积保留小视差，1-dot(u,v)^2 会因相消损失它。
  const auto s = dot(cross(w, v), n) / denominator, t = dot(cross(w, u), n) / denominator;
  if ((first.perspective && s < 0) || (second.perspective && t < 0))
    throw core::Exception(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, op, "Disparity",
                          -1, "intersection behind a perspective ray origin");
  const Point3d result{(a.x + s * u.x + b.x + t * v.x) * .5, (a.y + s * u.y + b.y + t * v.y) * .5,
                       (a.z + s * u.z + b.z + t * v.z) * .5};
  if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) return nan_point();
  return result;
}
}  // namespace mv::reconstruction
