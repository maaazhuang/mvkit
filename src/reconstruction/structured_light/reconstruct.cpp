#include <reconstruction/structured_light/reconstruct.hpp>
#include <reconstruction/quality.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>
#include <reconstruction/geometry/triangulation.hpp>
#include <runtime/execution/execution_context.hpp>
#include <runtime/cpu/cpu_features.hpp>

#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_SL_PAIR_SSE2 1
#else
#define MVKIT_SL_PAIR_SSE2 0
#endif

namespace mv::reconstruction {
namespace {

// 批量独立点保持标量运算序及精度；不支持/非法点对恢复原逐点错误处理。
bool triangulate_pinhole_pair(const calibration::AreaScanDivisionParameters& cp,
                              const calibration::AreaScanDivisionParameters& pp,
                              const std::array<double, 9>& rotation, const Point3d& origin,
                              std::size_t row, std::size_t col, const float* projector_cols,
                              const float* projector_rows, double max_gap,
                              std::array<Point3d, 2>& out) {
#if MVKIT_SL_PAIR_SSE2
  const auto one = _mm_set1_pd(1.), zero = _mm_setzero_pd();
  const auto finite = [](__m128d x) {
    return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), x),
                        _mm_set1_pd(std::numeric_limits<double>::max()));
  };
  const auto dot = [](__m128d ax, __m128d ay, __m128d az, __m128d bx, __m128d by, __m128d bz) {
    return _mm_add_pd(_mm_add_pd(_mm_mul_pd(ax, bx), _mm_mul_pd(ay, by)), _mm_mul_pd(az, bz));
  };
  const auto x = _mm_set_pd(double(col + 1), double(col));
  auto ux = _mm_div_pd(_mm_mul_pd(_mm_sub_pd(x, _mm_set1_pd(cp.cx)), _mm_set1_pd(cp.sx)),
                       _mm_set1_pd(cp.focus));
  auto uy = _mm_set1_pd((double(row) - cp.cy) * cp.sy / cp.focus), uz = one;
  const auto pc =
      _mm_cvtps_pd(_mm_loadl_pi(_mm_setzero_ps(), reinterpret_cast<const __m64*>(projector_cols)));
  const auto pr =
      _mm_cvtps_pd(_mm_loadl_pi(_mm_setzero_ps(), reinterpret_cast<const __m64*>(projector_rows)));
  const auto rx = _mm_div_pd(_mm_mul_pd(_mm_sub_pd(pc, _mm_set1_pd(pp.cx)), _mm_set1_pd(pp.sx)),
                             _mm_set1_pd(pp.focus));
  const auto ry = _mm_div_pd(_mm_mul_pd(_mm_sub_pd(pr, _mm_set1_pd(pp.cy)), _mm_set1_pd(pp.sy)),
                             _mm_set1_pd(pp.focus));
  const auto rotate = [&](int k) {
    return _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(rotation[k]), rx),
                                 _mm_mul_pd(_mm_set1_pd(rotation[k + 1]), ry)),
                      _mm_mul_pd(_mm_set1_pd(rotation[k + 2]), one));
  };
  auto vx = rotate(0), vy = rotate(3), vz = rotate(6);
  const auto nu = _mm_sqrt_pd(dot(ux, uy, uz, ux, uy, uz));
  if (_mm_movemask_pd(_mm_and_pd(finite(nu), _mm_cmpgt_pd(nu, _mm_set1_pd(1e-15)))) != 3)
    return false;
  const auto su = _mm_div_pd(one, nu);
  ux = _mm_mul_pd(ux, su);
  uy = _mm_mul_pd(uy, su);
  uz = _mm_mul_pd(uz, su);
  const auto nv = _mm_sqrt_pd(dot(vx, vy, vz, vx, vy, vz));
  if (_mm_movemask_pd(_mm_and_pd(finite(nv), _mm_cmpgt_pd(nv, _mm_set1_pd(1e-15)))) != 3)
    return false;
  const auto sv = _mm_div_pd(one, nv);
  vx = _mm_mul_pd(vx, sv);
  vy = _mm_mul_pd(vy, sv);
  vz = _mm_mul_pd(vz, sv);
  const auto wx = _mm_sub_pd(zero, _mm_set1_pd(origin.x)),
             wy = _mm_sub_pd(zero, _mm_set1_pd(origin.y)),
             wz = _mm_sub_pd(zero, _mm_set1_pd(origin.z));
  const auto b = dot(ux, uy, uz, vx, vy, vz), d = dot(ux, uy, uz, wx, wy, wz),
             e = dot(vx, vy, vz, wx, wy, wz);
  const auto denominator = _mm_sub_pd(_mm_mul_pd(one, one), _mm_mul_pd(b, b));
  if (_mm_movemask_pd(_mm_and_pd(
          finite(denominator),
          _mm_cmpge_pd(_mm_andnot_pd(_mm_set1_pd(-0.), denominator), _mm_set1_pd(1e-12)))) != 3)
    return false;
  const auto ds = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(b, e), _mm_mul_pd(one, d)), denominator);
  const auto dt = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(one, e), _mm_mul_pd(b, d)), denominator);
  if (_mm_movemask_pd(_mm_and_pd(_mm_cmpgt_pd(ds, zero), _mm_cmpgt_pd(dt, zero))) != 3)
    return false;
  const auto p0x = _mm_add_pd(zero, _mm_mul_pd(ux, ds)), p0y = _mm_add_pd(zero, _mm_mul_pd(uy, ds)),
             p0z = _mm_add_pd(zero, _mm_mul_pd(uz, ds));
  const auto p1x = _mm_add_pd(_mm_set1_pd(origin.x), _mm_mul_pd(vx, dt)),
             p1y = _mm_add_pd(_mm_set1_pd(origin.y), _mm_mul_pd(vy, dt)),
             p1z = _mm_add_pd(_mm_set1_pd(origin.z), _mm_mul_pd(vz, dt));
  const auto dx = _mm_sub_pd(p0x, p1x), dy = _mm_sub_pd(p0y, p1y), dz = _mm_sub_pd(p0z, p1z);
  const auto gap = _mm_sqrt_pd(dot(dx, dy, dz, dx, dy, dz));
  if (_mm_movemask_pd(_mm_and_pd(finite(gap), _mm_cmple_pd(gap, _mm_set1_pd(max_gap)))) != 3)
    return false;
  const auto px = _mm_mul_pd(_mm_add_pd(p0x, p1x), _mm_set1_pd(.5)),
             py = _mm_mul_pd(_mm_add_pd(p0y, p1y), _mm_set1_pd(.5)),
             pz = _mm_mul_pd(_mm_add_pd(p0z, p1z), _mm_set1_pd(.5));
  out[0] = {_mm_cvtsd_f64(px), _mm_cvtsd_f64(py), _mm_cvtsd_f64(pz)};
  out[1] = {_mm_cvtsd_f64(_mm_unpackhi_pd(px, px)), _mm_cvtsd_f64(_mm_unpackhi_pd(py, py)),
            _mm_cvtsd_f64(_mm_unpackhi_pd(pz, pz))};
  return true;
#else
  (void)cp;
  (void)pp;
  (void)rotation;
  (void)origin;
  (void)row;
  (void)col;
  (void)projector_cols;
  (void)projector_rows;
  (void)max_gap;
  (void)out;
  return false;
#endif
}

// 在原首次使用位置、同一分区浮点环境中准备。
Ray3d transform_ray_prepared(const Pose3d& pose, const calibration::Ray3d& ray,
                             std::optional<std::array<double, 9>>& rotation) {
  if (!rotation) rotation = calibration::rotation_matrix(pose.rotation);
  const auto& r = *rotation;
  const auto rotate = [&](const Point3d& p) {
    return Point3d{r[0] * p.x + r[1] * p.y + r[2] * p.z, r[3] * p.x + r[4] * p.y + r[5] * p.z,
                   r[6] * p.x + r[7] * p.y + r[8] * p.z};
  };
  const auto origin = rotate(ray.origin);
  // 保留旋转原点加平移，包含原带符号零算术。
  return {
      {origin.x + pose.translation.x, origin.y + pose.translation.y, origin.z + pose.translation.z},
      rotate(ray.direction)};
}

// 数值求解器在原 transform-point 位置调用。
Point3d transform_point_prepared(const Pose3d& pose, const Point3d& point,
                                 std::optional<std::array<double, 9>>& rotation) {
  if (!rotation) rotation = calibration::rotation_matrix(pose.rotation);
  const auto& r = *rotation;
  const Point3d rotated{r[0] * point.x + r[1] * point.y + r[2] * point.z,
                        r[3] * point.x + r[4] * point.y + r[5] * point.z,
                        r[6] * point.x + r[7] * point.y + r[8] * point.z};
  return {rotated.x + pose.translation.x, rotated.y + pose.translation.y,
          rotated.z + pose.translation.z};
}

Point3d point_on_ray(const calibration::Ray3d& ray, double distance) {
  return {ray.origin.x + distance * ray.direction.x,
          ray.origin.y + distance * ray.direction.y,
          ray.origin.z + distance * ray.direction.z};
}

Point3d single_coordinate_point(const calibration::CameraParameters& camera,
                                const calibration::CameraParameters& projector,
                                const Pose3d& projector_in_camera,
                                const Pose3d& camera_in_projector, double row, double col,
                                double projector_coordinate, PatternOrientation orientation) {
  const auto camera_ray = calibration::image_point_to_ray(camera, {row, col});
  const auto& common = calibration::common_parameters(projector);
  std::optional<std::array<double, 9>> inverse_rotation;

  const auto coordinate_error = [&](double distance) {
    const auto camera_point = point_on_ray(camera_ray, distance);
    const auto projector_point =
        transform_point_prepared(camera_in_projector, camera_point, inverse_rotation);
    const auto image_point = calibration::project_camera_point(projector, projector_point);
    return (orientation == PatternOrientation::Vertical ? image_point.col : image_point.row) -
           projector_coordinate;
  };
  const auto safe_coordinate_error = [&](double distance) -> std::optional<double> {
    try {
      const double error = coordinate_error(distance);
      if (std::isfinite(error)) return error;
    } catch (const core::Exception&) {
    }
    return std::nullopt;
  };

  // 无畸变透视投影仪坐标定义过光心的平面，直接与相机射线求交；
  // 畸变/远心投影仪仍使用下方数值求解器。
  if (const auto* perspective = std::get_if<calibration::AreaScanDivisionParameters>(&projector);
      perspective && perspective->kappa == 0.0) {
    const auto normal = calibration::rotate_point(
        projector_in_camera.rotation,
        orientation == PatternOrientation::Vertical
            ? Point3d{perspective->focus, 0, -(projector_coordinate - common.cx) * common.sx}
            : Point3d{0, perspective->focus, -(projector_coordinate - common.cy) * common.sy});
    const auto& origin = camera_ray.origin;
    const auto& direction = camera_ray.direction;
    const auto& center = projector_in_camera.translation;
    const double denominator =
        normal.x * direction.x + normal.y * direction.y + normal.z * direction.z;
    const double numerator = normal.x * (center.x - origin.x) + normal.y * (center.y - origin.y) +
                             normal.z * (center.z - origin.z);
    const double candidate = numerator / denominator;
    if (std::isfinite(candidate) && candidate > 1e-7) {
      const auto error = safe_coordinate_error(candidate);
      if (error && std::abs(*error) < 1e-5) return point_on_ray(camera_ray, candidate);
    }
  }

  double distance = 1.0;
  try {
    const Point2d projector_image_point = orientation == PatternOrientation::Vertical
                                              ? Point2d{common.cy, projector_coordinate}
                                              : Point2d{projector_coordinate, common.cx};
    const auto projector_ray = calibration::image_point_to_ray(projector, projector_image_point);
    const auto projector_ray_in_camera = transform_ray(projector_in_camera, projector_ray);
    const auto estimate = triangulate_rays(Ray3d{camera_ray.origin, camera_ray.direction},
                                           projector_ray_in_camera, 1e6);
    const double dx = estimate.point.x - camera_ray.origin.x;
    const double dy = estimate.point.y - camera_ray.origin.y;
    const double dz = estimate.point.z - camera_ray.origin.z;
    const double denominator = camera_ray.direction.x * camera_ray.direction.x +
                               camera_ray.direction.y * camera_ray.direction.y +
                               camera_ray.direction.z * camera_ray.direction.z;
    distance =
        (dx * camera_ray.direction.x + dy * camera_ray.direction.y + dz * camera_ray.direction.z) /
        denominator;
  } catch (const core::Exception&) {
    // 下方 Newton 求解仍为单坐标权威路径。
  }

  distance = std::max(distance, 1e-6);
  for (int iteration = 0; iteration < 30; ++iteration) {
    const auto error_value = safe_coordinate_error(distance);
    if (!error_value.has_value()) break;
    const double error = *error_value;
    if (std::abs(error) < 1e-5) {
      return point_on_ray(camera_ray, distance);
    }
    const double h = std::max(1e-7, std::abs(distance) * 1e-5);
    const double minus = std::max(1e-7, distance - h);
    const double plus = distance + h;
    const auto plus_error = safe_coordinate_error(plus);
    const auto minus_error = safe_coordinate_error(minus);
    if (!plus_error.has_value() || !minus_error.has_value()) break;
    const double derivative = (*plus_error - *minus_error) / (plus - minus);
    if (!std::isfinite(derivative) || std::abs(derivative) < 1e-12) {
      break;
    }
    const double next = distance - error / derivative;
    if (!std::isfinite(next) || next <= 1e-7) {
      break;
    }
    distance = next;
  }

  const auto final_error = safe_coordinate_error(distance);
  if (final_error.has_value() && std::abs(*final_error) <= 0.05) {
    return point_on_ray(camera_ray, distance);
  }

  // 强畸变斜向配置可能使 Newton 离开有效投影仪半空间，回退正距离区间及二分。
  std::optional<double> previous_distance;
  std::optional<double> previous_error;
  for (int sample = 0; sample <= 96; ++sample) {
    const double candidate = std::pow(10.0, -4.0 + 6.0 * sample / 96.0);
    const auto candidate_error = safe_coordinate_error(candidate);
    if (!candidate_error.has_value()) continue;
    if (std::abs(*candidate_error) <= 1e-5) return point_on_ray(camera_ray, candidate);
    if (previous_error.has_value() && (*previous_error < 0.0) != (*candidate_error < 0.0)) {
      double lower = *previous_distance;
      double upper = candidate;
      double lower_error = *previous_error;
      for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = 0.5 * (lower + upper);
        const auto middle_error = safe_coordinate_error(middle);
        if (!middle_error.has_value()) {
          lower = middle;
          continue;
        }
        if (std::abs(*middle_error) <= 1e-5) return point_on_ray(camera_ray, middle);
        if ((lower_error < 0.0) != (*middle_error < 0.0)) {
          upper = middle;
        } else {
          lower = middle;
          lower_error = *middle_error;
        }
      }
      return point_on_ray(camera_ray, 0.5 * (lower + upper));
    }
    previous_distance = candidate;
    previous_error = candidate_error;
  }

  throw core::Exception(core::Status::InvalidState, "reconstruct_surface_structured_light",
                        "single-coordinate projector triangulation did not converge");
}

}  // namespace

ObjectModel3DData reconstruct_surface_structured_light(const StructuredLightModel& model,
                                                       double max_ray_gap) {
  if (!std::isfinite(max_ray_gap) || max_ray_gap < 0.0) {
    throw core::Exception(core::Status::InvalidArgument, "reconstruct_surface_structured_light",
                          "max_ray_gap must be finite and nonnegative");
  }
  if (!model.has_decoded_state_ || !model.camera_setup_) {
    throw core::Exception(core::Status::InvalidState, "reconstruct_surface_structured_light",
                          "decoded correspondences and camera_projector setup are required");
  }

  const auto& setup = *model.camera_setup_;
  if (setup.kind() != CameraSetupKind::CameraProjector) {
    throw core::Exception(core::Status::InvalidState, "reconstruct_surface_structured_light",
                          "camera_projector setup required");
  }

  const bool both = model.parameters_.orientation == PatternOrientation::Both;
  if (both && model.decoded_correspondence_.size() != 2) {
    throw core::Exception(core::Status::InvalidState, "reconstruct_surface_structured_light",
                          "both orientation requires two correspondence images");
  }
  const auto& validity = model.decoded_validity_;
  ObjectModel3DData output;
  output.width = validity.width();
  output.height = validity.height();
  const std::size_t count = output.width * output.height;
  output.points.resize(count);
  output.valid.assign(count, 0);
  output.confidence.assign(count, 0.0f);
  const bool quality = model.parameters_.quality_output;
  if (quality) allocate_quality(output);

  const auto first = model.decoded_correspondence_.at(0).as_image();
  const auto second = both ? model.decoded_correspondence_.at(1).as_image() : core::Image{};
  const auto& camera = setup.camera(0);
  const auto& projector = setup.camera(1);
  const auto& relative = setup.device_1_in_device_0_pose();
  const auto inverse = calibration::inverse_pose(relative);
  const auto* cp = std::get_if<calibration::AreaScanDivisionParameters>(&camera);
  const auto* pp = std::get_if<calibration::AreaScanDivisionParameters>(&projector);
  const bool pinhole = cp && pp && cp->kappa == 0 && pp->kappa == 0;
  const auto rotation = calibration::rotation_matrix(relative.rotation);
  bool pair_enabled = false;
#if MVKIT_SL_PAIR_SSE2
  if (pinhole && both && output.width >= 2) {
    const auto& cpu = runtime::cpu_features();
    pair_enabled =
        runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                {true, false, false, false});
  }
#endif

  const auto rotate = [&](const Point3d& v) {
    return Point3d{rotation[0] * v.x + rotation[1] * v.y + rotation[2] * v.z,
                   rotation[3] * v.x + rotation[4] * v.y + rotation[5] * v.z,
                   rotation[6] * v.x + rotation[7] * v.y + rotation[8] * v.z};
  };
  const auto* valid_data = static_cast<const std::uint8_t*>(validity.data());
  const auto valid_stride = validity.stride();
  const auto* first_data = static_cast<const std::byte*>(first.data());
  const auto first_stride = first.stride();
  const auto* second_data = both ? static_cast<const std::byte*>(second.data()) : nullptr;
  const auto second_stride = both ? second.stride() : 0;
  const auto* contrast_data = model.decoded_contrast_.empty()
                                  ? nullptr
                                  : static_cast<const std::byte*>(model.decoded_contrast_.data());
  const auto contrast_stride = contrast_data ? model.decoded_contrast_.stride() : 0;

  runtime::ExecutionContext::for_each_partition(
      output.height, runtime::ExecutionContext::saturated_work(output.width, output.height), 2048,
      [&](auto range) {
        std::optional<std::array<double, 9>> worker_rotation;
        for (std::size_t row = range.begin; row < range.end; ++row) {
          std::array<Point3d, 2> paired_points{};
          bool have_pair = false;
          std::size_t paired_col = 0;
          for (std::size_t col = 0; col < output.width; ++col) {
            const auto index = row * output.width + col;
            if (quality)
              output.quality_reason[index] = reinterpret_cast<const std::uint16_t*>(
                  static_cast<const std::byte*>(model.decoded_reason_.data()) +
                  row * model.decoded_reason_.stride())[col];
            if (valid_data[row * valid_stride + col] == 0) {
              continue;
            }

            try {
              const double first_projector_coordinate =
                  reinterpret_cast<const float*>(first_data + row * first_stride)[col];
              Point3d point;
              if (pinhole) {
                if (both) {
                  if (have_pair && col == paired_col + 1)
                    point = paired_points[1];
                  else if (pair_enabled && output.width - col >= 2 &&
                           valid_data[row * valid_stride + col + 1] != 0 &&
                           triangulate_pinhole_pair(
                               *cp, *pp, rotation, relative.translation, row, col,
                               reinterpret_cast<const float*>(first_data + row * first_stride) +
                                   col,
                               reinterpret_cast<const float*>(second_data + row * second_stride) +
                                   col,
                               max_ray_gap, paired_points)) {
                    have_pair = true;
                    paired_col = col;
                    point = paired_points[0];
                  } else {
                    have_pair = false;
                    const Point3d ray{(double(col) - cp->cx) * cp->sx / cp->focus,
                                      (double(row) - cp->cy) * cp->sy / cp->focus, 1};
                    const double pcrow =
                        reinterpret_cast<const float*>(second_data + row * second_stride)[col];
                    const Point3d pr{(first_projector_coordinate - pp->cx) * pp->sx / pp->focus,
                                     (pcrow - pp->cy) * pp->sy / pp->focus, 1};
                    point = triangulate_rays(Ray3d{{0, 0, 0}, ray},
                                             Ray3d{relative.translation, rotate(pr)}, max_ray_gap)
                                .point;
                  }
                } else {
                  const Point3d ray{(double(col) - cp->cx) * cp->sx / cp->focus,
                                    (double(row) - cp->cy) * cp->sy / cp->focus, 1};
                  const bool vertical =
                      model.parameters_.orientation == PatternOrientation::Vertical;
                  const auto n = rotate(
                      vertical
                          ? Point3d{pp->focus, 0, -(first_projector_coordinate - pp->cx) * pp->sx}
                          : Point3d{0, pp->focus, -(first_projector_coordinate - pp->cy) * pp->sy});
                  const auto& t = relative.translation;
                  const double depth =
                      (n.x * t.x + n.y * t.y + n.z * t.z) / (n.x * ray.x + n.y * ray.y + n.z);
                  point = {ray.x * depth, ray.y * depth, depth};
                  // 保持原正向射线及投影仪投影守卫。
                  const double x = point.x - t.x, y = point.y - t.y, z = point.z - t.z;
                  const double pz = rotation[2] * x + rotation[5] * y + rotation[8] * z;
                  const double transverse =
                      vertical ? rotation[0] * x + rotation[3] * y + rotation[6] * z
                               : rotation[1] * x + rotation[4] * y + rotation[7] * z;
                  const double reprojected =
                      pp->focus * transverse / pz / (vertical ? pp->sx : pp->sy) +
                      (vertical ? pp->cx : pp->cy);
                  if (!(depth > 1e-7) || !(pz > 1e-12) || !std::isfinite(depth) ||
                      !std::isfinite(reprojected) ||
                      std::abs(reprojected - first_projector_coordinate) >= 1e-5)
                    point = single_coordinate_point(
                        camera, projector, relative, inverse, static_cast<double>(row),
                        static_cast<double>(col), first_projector_coordinate,
                        model.parameters_.orientation);
                }
              } else if (both) {
                const double projector_col = first_projector_coordinate;
                const double projector_row =
                    reinterpret_cast<const float*>(second_data + row * second_stride)[col];
                const auto camera_ray = calibration::image_point_to_ray(
                    camera, {static_cast<double>(row), static_cast<double>(col)});
                const auto projector_ray =
                    calibration::image_point_to_ray(projector, {projector_row, projector_col});
                point = triangulate_rays(
                            Ray3d{camera_ray.origin, camera_ray.direction},
                            transform_ray_prepared(relative, projector_ray, worker_rotation),
                            max_ray_gap)
                            .point;
              } else {
                point = single_coordinate_point(camera, projector, relative, inverse,
                                                static_cast<double>(row), static_cast<double>(col),
                                                first_projector_coordinate,
                                                model.parameters_.orientation);
              }

              if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw core::Exception(core::Status::InvalidState,
                                      "reconstruct_surface_structured_light",
                                      "nonfinite triangulation");
              output.points[index] = point;
              output.valid[index] = 1;
              const float contrast =
                  contrast_data
                      ? reinterpret_cast<const float*>(contrast_data + row * contrast_stride)[col]
                      : 255.0f;
              output.confidence[index] = std::clamp(contrast / 255.0f, 0.0f, 1.0f);
              if (quality) {
                output.observed_valid[index] = 1;
                output.quality_reason[index] = 0;
                double variance = 0;
                std::optional<calibration::Ray3d> quality_camera_ray;
                bool known = true;
                for (std::size_t di = 0; di < (both ? 2U : 1U); ++di) {
                  const auto si = model.decoded_coordinate_stddev_.at(di).as_image();
                  const double sigma = reinterpret_cast<const float*>(
                      static_cast<const std::byte*>(si.data()) + row * si.stride())[col];
                  if (!(sigma > 0)) {
                    known = false;
                    break;
                  }
                  const double first_coordinate = first_projector_coordinate;
                  const double second_coordinate =
                      both ? reinterpret_cast<const float*>(second_data + row * second_stride)[col]
                           : 0;
                  const auto evaluate = [&](double coordinate) {
                    if (!both && pinhole) {
                      const Point3d ray{(double(col) - cp->cx) * cp->sx / cp->focus,
                                        (double(row) - cp->cy) * cp->sy / cp->focus, 1};
                      const bool vertical =
                          model.parameters_.orientation == PatternOrientation::Vertical;
                      const auto n =
                          rotate(vertical ? Point3d{pp->focus, 0, -(coordinate - pp->cx) * pp->sx}
                                          : Point3d{0, pp->focus, -(coordinate - pp->cy) * pp->sy});
                      const auto t = relative.translation;
                      const double z =
                          (n.x * t.x + n.y * t.y + n.z * t.z) / (n.x * ray.x + n.y * ray.y + n.z);
                      return Point3d{ray.x * z, ray.y * z, z};
                    }
                    if (!both)
                      return single_coordinate_point(camera, projector, relative, inverse,
                                                     double(row), double(col), coordinate,
                                                     model.parameters_.orientation);
                    // 延迟复用留在 depth_sigma 的原异常边界内。
                    if (!quality_camera_ray)
                      quality_camera_ray =
                          calibration::image_point_to_ray(camera, {double(row), double(col)});
                    const auto& cr = *quality_camera_ray;
                    const auto pr = calibration::image_point_to_ray(
                        projector, {di == 1 ? coordinate : second_coordinate,
                                    di == 0 ? coordinate : first_coordinate});
                    return triangulate_rays(Ray3d{cr.origin, cr.direction},
                                            transform_ray_prepared(relative, pr, worker_rotation),
                                            std::numeric_limits<double>::max())
                        .point;
                  };
                  const float propagated =
                      depth_sigma(evaluate, di == 0 ? first_coordinate : second_coordinate, sigma);
                  if (!std::isfinite(propagated)) {
                    known = false;
                    break;
                  }
                  variance += double(propagated) * propagated;
                }
                if (known)
                  output.depth_uncertainty[index] = static_cast<float>(std::sqrt(variance));
              }
            } catch (const core::Exception&) {
              if (quality) output.quality_reason[index] |= GeometryFailure;
              // 无效点保持无效；遮挡/边缘附近逐像素几何失败属于预期行为。
            }
          }
        }
      });

  if (quality)
    fill_quality_holes(output, model.parameters_.hole_max_width,
                       model.parameters_.hole_max_depth_jump);
  validate_object_model_3d(output);
  return output;
}

}  // namespace mv::reconstruction
