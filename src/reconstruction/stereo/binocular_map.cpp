#include <algorithm>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/division_projection.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <reconstruction/stereo/binocular_map.hpp>
#include <runtime/execution/execution_context.hpp>
#include <type_traits>

namespace mv::reconstruction {
namespace {
constexpr auto op = "gen_binocular_rectification_map";
using Camera = calibration::CameraParameters;
using Matrix = std::array<double, 9>;
Point3d add(Point3d a, Point3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Point3d mul(Point3d a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Point3d cross(Point3d a, Point3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Point3d unit(Point3d a) {
  const auto n = std::sqrt(dot(a, a));
  if (!(n > 1e-14) || !std::isfinite(n))
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "RelPose", -1,
                          "degenerate stereo geometry");
  return mul(a, 1 / n);
}
Point3d apply(const Matrix& m, Point3d p) {
  return {m[0] * p.x + m[1] * p.y + m[2] * p.z, m[3] * p.x + m[4] * p.y + m[5] * p.z,
          m[6] * p.x + m[7] * p.y + m[8] * p.z};
}
Matrix transpose(const Matrix& m) { return {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]}; }
Matrix inverse(const Matrix& m) {
  Matrix adj{m[4] * m[8] - m[5] * m[7], m[2] * m[7] - m[1] * m[8], m[1] * m[5] - m[2] * m[4],
             m[5] * m[6] - m[3] * m[8], m[0] * m[8] - m[2] * m[6], m[2] * m[3] - m[0] * m[5],
             m[3] * m[7] - m[4] * m[6], m[1] * m[6] - m[0] * m[7], m[0] * m[4] - m[1] * m[3]};
  const auto determinant = m[0] * adj[0] + m[1] * adj[3] + m[2] * adj[6];
  if (std::abs(determinant) < 1e-14)
    throw core::Exception(core::Status::InvalidArgument, op, "degenerate rectification basis");
  for (auto& value : adj) value /= determinant;
  return adj;
}
Matrix product(const Matrix& a, const Matrix& b) {
  Matrix m{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      for (int k = 0; k < 3; ++k) m[r * 3 + c] += a[r * 3 + k] * b[k * 3 + c];
  return m;
}
Camera parse_camera(const core::Tuple& input, const char* parameter) {
  try {
    core::Tuple t;
    if (!input.empty() && input.at(0).kind() != core::ValueKind::String) t.append("area_scan_division");
    for (std::size_t i = 0; i < input.size(); ++i) t.append(input.at(i));
    if (t.size() == 9 || t.size() == 13) {
      core::Tuple integers;
      for (std::size_t i = 0; i < t.size(); ++i)
        if (i >= t.size() - 2) {
          const auto d = t.as_real64(i);
          if (!std::isfinite(d) || d <= 0 || d > double(INT32_MAX))
            throw core::Exception(core::Status::InvalidArgument, op, "invalid camera dimensions");
          integers.append(static_cast<std::int64_t>(d));
        } else
          integers.append(t.at(i));
      t = std::move(integers);
    }
    auto camera = calibration::parse_camera_parameters(t);
    if (!calibration::is_perspective(camera))
      throw core::Exception(core::Status::Unsupported, op, "telecentric rectification is not implemented");
    return camera;
  } catch (const core::Exception& e) {
    throw core::Exception(e.status(), e.error_code(), op, parameter, e.parameter_index(), e.message());
  }
}
double focus(const Camera& c) {
  return std::visit(
      [](const auto& p) -> double {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, calibration::AreaScanDivisionParameters> ||
                      std::is_same_v<T, calibration::AreaScanPolynomialParameters>)
          return p.focus;
        else
          return 0.;
      },
      c);
}
struct Bounds {
  double left = std::numeric_limits<double>::infinity(), right = -left, top = left, bottom = -left;
  std::vector<Point2d> perimeter;
};
Bounds bounds(const Camera& camera, const Matrix& original_to_rect) {
  Bounds b;
  const auto& size = calibration::common_parameters(camera);
  b.perimeter.reserve(static_cast<std::size_t>(2*(size.image_width+size.image_height)));
  const calibration::DivisionProjection projection(camera);
  const double focal=focus(camera);
  const auto include = [&](double row, double col) {
    const auto metric=projection.enabled()?projection.undistort({row,col}):
        std::optional<calibration::MetricPoint>{};
    const auto ray=metric?Point3d{metric->u/focal,metric->v/focal,1.}:
        calibration::image_point_to_ray(camera,{row,col}).direction;
    const auto p = apply(original_to_rect,ray);
    if (!(p.z > 0))
      throw core::Exception(core::Status::Unsupported, op, "rectified image plane crosses a camera horizon");
    const double x = p.x / p.z, y = p.y / p.z;
    b.perimeter.push_back({y, x});
    b.left = std::min(b.left, x);
    b.right = std::max(b.right, x);
    b.top = std::min(b.top, y);
    b.bottom = std::max(b.bottom, y);
  };
  // 连续视场由像素边缘而非中心限定，沿各边采样以包含角点间畸变极值。
  constexpr std::int64_t subdivisions = 1;
  for (std::int64_t i = 0; i <= size.image_width * subdivisions; ++i) {
    const double c = double(i) / subdivisions - .5;
    include(-.5, c);
    include(size.image_height - .5, c);
  }
  for (std::int64_t i = 1; i < size.image_height * subdivisions; ++i) {
    const double r = double(i) / subdivisions - .5;
    include(r, -.5);
    include(r, size.image_width - .5);
  }
  return b;
}
core::Tuple camera_controls(const Camera& original, double f, double pitch, double cx, double cy,
                            std::int64_t width, std::int64_t height) {
  auto camera = original;
  std::visit(
      [&](auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, calibration::AreaScanDivisionParameters> ||
                      std::is_same_v<T, calibration::AreaScanPolynomialParameters>)
          p.focus = f;
        if constexpr (std::is_same_v<T, calibration::AreaScanDivisionParameters> ||
                      std::is_same_v<T, calibration::AreaScanTelecentricDivisionParameters>)
          p.kappa = 0;
        else {
          p.k1 = 0;
          p.k2 = 0;
          p.k3 = 0;
          p.p1 = 0;
          p.p2 = 0;
        }
        p.sx = p.sy = pitch;
        p.cx = cx;
        p.cy = cy;
        p.image_width = width;
        p.image_height = height;
      },
      camera);
  return calibration::camera_parameters_to_tuple(camera);
}
core::Tuple pose_controls(const Matrix& rotation) {
  const auto beta = std::asin(std::clamp(rotation[2], -1., 1.));
  const bool regular = std::abs(std::cos(beta)) > 1e-12;
  const double degrees = 180. / 3.141592653589793238462643383279502884;
  const core::Tuple p{
      0.,
      0.,
      0.,
      (regular ? std::atan2(-rotation[5], rotation[8]) : std::atan2(rotation[3], rotation[4])) * degrees,
      beta * degrees,
      (regular ? std::atan2(-rotation[1], rotation[0]) : 0.) * degrees};
  core::Tuple result;
  for (std::size_t i = 0; i < 6; ++i) {
    double value = p.as_real64(i);
    if (i >= 3 && value < 0) value += 360.;
    result.append(value);
  }
  result.append(0);
  return result;
}
core::Region domain(const std::vector<std::uint8_t>& valid, std::size_t width, std::size_t height) {
  std::vector<core::RegionRun> runs;
  for (std::size_t r = 0; r < height; ++r)
    for (std::size_t c = 0; c < width;) {
      while (c < width && !valid[r * width + c]) ++c;
      const auto first = c;
      while (c < width && valid[r * width + c]) ++c;
      if (first < c) runs.push_back({std::int64_t(r), std::int64_t(first), std::int64_t(c - 1)});
    }
  return core::Region::from_runs(std::span<const core::RegionRun>(runs));
}
core::Image encode_map(const Camera& original, const core::Tuple& controls, const Matrix& rect_to_original,
                       bool nearest) {
  const auto rectified = calibration::parse_camera_parameters(controls);
  const auto& source = calibration::common_parameters(original);
  const auto& target = calibration::common_parameters(rectified);
  if (source.image_width > INT32_MAX / source.image_height)
    throw core::Exception(core::Status::Unsupported, op, "encoded source indices exceed int4");
  const std::array types{core::PixelType::Int32, core::PixelType::UInt16, core::PixelType::UInt16,
                         core::PixelType::UInt16, core::PixelType::UInt16};
  auto output = core::Image::create(target.image_width, target.image_height,
                                    std::span<const core::PixelType>(types.data(), nearest ? 1 : 5));
  const auto w = output.width(), h = output.height();
  std::vector<std::uint8_t> valid(w * h);
  std::array<std::byte*, 5> planes{};
  std::array<std::size_t, 5> strides{};
  for (std::size_t ch = 0; ch < output.channel_count(); ++ch) {
    planes[ch] = static_cast<std::byte*>(output.mutable_data(ch));
    strides[ch] = output.stride(ch);
  }
  if (!nearest && (source.image_width < 2 || source.image_height < 2))
    return output.reduce_domain(core::Region{});
  const double f = focus(rectified);
  const calibration::DivisionProjection projection(original);
  runtime::ExecutionContext::for_each_partition(
      h, runtime::ExecutionContext::saturated_work(w, h), 65536, [&](runtime::IndexRange range) {
        for (auto r = range.begin; r < range.end; ++r)
          for (std::size_t c = 0; c < w; ++c) {
            const auto p = apply(rect_to_original, {(double(c) - target.cx) * target.sx,
                                                    (double(r) - target.cy) * target.sy, f});
            if (!(p.z > 0)) continue;
            // division 参数在循环外准备；异常情形仍交回原投影器，保留整次调用的错误行为。
            const auto prepared=projection.enabled()?projection.project(p):std::optional<Point2d>{};
            const auto pixel=prepared?*prepared:calibration::project_camera_point(original,p);
            if (!std::isfinite(pixel.row) || !std::isfinite(pixel.col)) continue;
            // HALCON nearest 在精确半值时选择较小像素。
            const auto shifted_row=pixel.row-.5,shifted_column=pixel.col-.5;
            if(!(shifted_row>-1 && shifted_column>-1 && shifted_row<=double(source.image_height-1) &&
                 shifted_column<=double(source.image_width-1)))continue;
            auto nr=static_cast<std::int64_t>(shifted_row),nc=static_cast<std::int64_t>(shifted_column);
            nr+=double(nr)<shifted_row;nc+=double(nc)<shifted_column;
            valid[r * w + c] = 1;
            auto* index = reinterpret_cast<std::int32_t*>(planes[0] + r * strides[0]) + c;
            if (nearest) {
              *index = std::int32_t(nr * source.image_width + nc);
              continue;
            }
            const auto rr = std::clamp(pixel.row, 0., double(source.image_height - 1)),
                       cc = std::clamp(pixel.col, 0., double(source.image_width - 1));
            const auto y = std::min(std::int64_t(rr), source.image_height - 2),
                       x = std::min(std::int64_t(cc), source.image_width - 2);
            const double dy = rr - y, dx = cc - x;
            *index = std::int32_t(y * source.image_width + x);
            const std::array weights{(1 - dy) * (1 - dx), (1 - dy) * dx, dy * (1 - dx), dy * dx};
            for (std::size_t ch = 0; ch < 4; ++ch)
              reinterpret_cast<std::uint16_t*>(planes[ch + 1] + r * strides[ch + 1])[c] =
                  std::uint16_t(std::clamp(weights[ch] * 65535., 0., 65535.));
          }
      });
  return output.reduce_domain(domain(valid, w, h));
}
}  // namespace
BinocularMaps binocular_rectification_maps(const core::Tuple& camera1, const core::Tuple& camera2,
                                           const core::Tuple& relative_pose, double subsampling,
                                           std::string_view method, std::string_view map_type) {
  if (!std::isfinite(subsampling) || subsampling <= 0)
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "SubSampling", 0,
                          "positive finite subsampling required");
  if (method != "viewing_direction" && method != "geometric")
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::UnsupportedValue, op, "Method", 0,
                          "unknown rectification method");
  if (map_type != "nearest_neighbor" && map_type != "bilinear")
    throw core::Exception(
        map_type == "coord_map_sub_pix" ? core::Status::Unsupported : core::Status::InvalidArgument,
        core::ErrorCode::UnsupportedValue, op, "MapType", 0,
        "nearest_neighbor or bilinear required; vector_field storage is not implemented");
  const std::array<Camera, 2> cameras{parse_camera(camera1, "CamParam1"), parse_camera(camera2, "CamParam2")};
  Pose3d pose;
  try {
    pose = calibration::pose_from_halcon_tuple(relative_pose);
  } catch (const core::Exception& e) {
    throw core::Exception(e.status(), e.error_code(), op, "RelPose", e.parameter_index(), e.message());
  }
  const auto rel = calibration::rotation_matrix(pose.rotation);
  const Point3d n0{0, 0, 1}, n1{rel[2], rel[5], rel[8]};
  const auto x = unit(pose.translation);
  auto z = add(n0, n1);
  z = unit(add(z, mul(x, -dot(z, x))));
  if (method == "geometric") {
    const auto candidate = cross(x, cross(n0, n1));
    if (dot(candidate, candidate) > 1e-28) {
      auto normal = unit(candidate);
      if (dot(normal, z) < 0) normal = mul(normal, -1);
      z = normal;
    } else
      z = n0;
  }
  // 平行平面 geometric 分支保留叉积尺度，投影基不正交，不能作为旋转求逆。
  const auto y = method == "geometric" ? cross(z, x) : unit(cross(z, x));
  const Matrix rect{x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z};
  const auto inverse_rect = inverse(rect);
  const std::array<Matrix, 2> to_rect{inverse_rect, product(inverse_rect, rel)};
  const std::array<Matrix, 2> from_rect{rect, product(transpose(rel), rect)};
  // 焦距测量经过各图像中心，中心可偏离主点；虽校正方向使用原光轴，偏轴中心仍影响距离。
  const auto center_point = [&](std::size_t i) {
    const auto& size = calibration::common_parameters(cameras[i]);
    return mul(calibration::image_point_to_ray(
                   cameras[i], {.5 * double(size.image_height), .5 * double(size.image_width)})
                   .direction,
               focus(cameras[i]));
  };
  const double f = .5 * (apply(to_rect[0], center_point(0)).z + apply(to_rect[1], center_point(1)).z);
  const double pitch =
      .5 * (calibration::common_parameters(cameras[0]).sy + calibration::common_parameters(cameras[1]).sy);
  if (!(f > 0)) throw core::Exception(core::Status::Unsupported, op, "positive common focal length required");
  const double scale = f / pitch;
  std::array<Bounds, 2> fields{bounds(cameras[0], to_rect[0]), bounds(cameras[1], to_rect[1])};
  const double top = std::max(fields[0].top, fields[1].top),
               bottom = std::min(fields[0].bottom, fields[1].bottom);
  if (!(bottom > top))
    throw core::Exception(core::Status::InvalidState, op,
                          "rectified views have no common vertical field of view");
  const double clipped_bottom = std::min(bottom, top + std::round((bottom - top) * scale) / scale);
  // 水平范围只取共同垂直视场内边界样本；膨胀镜头不能用已丢弃行扩大 Map。
  for (std::size_t i = 0; i < fields.size(); ++i) {
    auto& field = fields[i];
    field.left = std::numeric_limits<double>::infinity();
    field.right = -field.left;
    for (const auto& point : field.perimeter)
      if (point.row >= top - 1e-14 && point.row <= clipped_bottom + 1e-14) {
        field.left = std::min(field.left, point.col);
        field.right = std::max(field.right, point.col);
      }
  }
  const auto dimension = [&](double extent) {
    const double n = std::floor(std::round(extent) / subsampling);
    if (n < 1 || !std::isfinite(n) || n > INT32_MAX)
      throw core::Exception(core::Status::InvalidArgument, op, "invalid rectified dimensions");
    return std::int64_t(n);
  };
  const auto height = dimension((bottom - top) * scale);
  const double cy = (-top * scale - .5) / subsampling;
  BinocularMaps result;
  for (std::size_t i = 0; i < 2; ++i) {
    const auto width = dimension((fields[i].right - fields[i].left) * scale);
    result.cameras[i] = camera_controls(cameras[i], f, pitch * subsampling,
                                        (-fields[i].left * scale - .5) / subsampling, cy, width, height);
    result.camera_poses[i] = pose_controls(from_rect[i]);
    result.maps[i] = encode_map(cameras[i], result.cameras[i], from_rect[i], map_type == "nearest_neighbor");
  }
  result.relative_pose = core::Tuple{core::Value(std::sqrt(dot(pose.translation, pose.translation))),
                                     core::Value(0.),
                                     core::Value(0.),
                                     core::Value(0.),
                                     core::Value(0.),
                                     core::Value(0.),
                                     core::Value(0)};
  return result;
}
}  // namespace mv::reconstruction
