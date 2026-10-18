#include <array>
#include <calibration/camera_model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/division_projection.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/projection.hpp>
#include <calibration/rectification.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <core/iconic_tuple.hpp>
#include <cstring>
#include <limits>
#include <optional>
#include <runtime/execution/execution_context.hpp>
#include <string_view>
#include <utility>
#include <type_traits>
#include <vector>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_RECTIFICATION_SSE2 1
#else
#define MVKIT_RECTIFICATION_SSE2 0
#endif

namespace mv::calibration {
namespace {

void require_outputs(core::Image* row_map, core::Image* col_map, const char* op) {
  if (row_map == nullptr || col_map == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op, "outputs", -1,
                          "map outputs must not be null");
  }
  if (row_map == col_map) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "outputs", -1,
                          "row and column maps must be distinct");
  }
}

std::optional<Point2d> try_source_from_output(const CameraParameters& camera_in,
                                              const CameraParameters& camera_out, Point2d out) {
  try {
    const auto metric = image_point_to_undistorted_metric(camera_out, out);
    return undistorted_metric_to_image_point(camera_in, metric);
  } catch (const core::Exception&) {
    return std::nullopt;
  }
}

void fill_maps(std::size_t width, std::size_t height, core::Image* row_map, core::Image* col_map,
               const auto& sample) {
  constexpr std::array types{core::PixelType::Float32};
  const auto create_map = [&] {
    try {
      return core::Image::create_uninitialized(width, height, types);
    } catch (const core::Exception& e) {
      // 保留已公开的分配错误契约。
      if (e.where() == "Image::create_uninitialized")
        throw core::Exception(e.status(), e.error_code(), "Image::create", e.parameter(), e.parameter_index(),
                              e.message());
      throw;
    }
  };
  *row_map = create_map();
  *col_map = create_map();
  auto* rows = static_cast<std::byte*>(row_map->mutable_data());
  auto* cols = static_cast<std::byte*>(col_map->mutable_data());
  const auto row_stride = row_map->stride();
  const auto col_stride = col_map->stride();
  runtime::ExecutionContext::for_each_partition(
      height, runtime::ExecutionContext::saturated_work(width, height), 65536,
      [&](runtime::IndexRange range) {
        for (auto r = range.begin; r < range.end; ++r) {
          auto* row_ptr = reinterpret_cast<float*>(rows + r * row_stride);
          auto* col_ptr = reinterpret_cast<float*>(cols + r * col_stride);
          std::size_t first = 0;
          if constexpr (requires { sample.fill_row(r, width, row_ptr, col_ptr); })
            first = sample.fill_row(r, width, row_ptr, col_ptr);
          for (std::size_t c = first; c < width; ++c) {
            const auto src = sample(r, c);
            row_ptr[c] = src ? static_cast<float>(src->row) : std::numeric_limits<float>::quiet_NaN();
            col_ptr[c] = src ? static_cast<float>(src->col) : std::numeric_limits<float>::quiet_NaN();
          }
        }
      });
}

void map_distortion_points(const CameraParameters& camera_in, const CameraParameters& camera_out,
                           std::span<const core::Point2d> input, std::span<core::Point2d> output,
                           const char* op) {
  std::size_t index = 0;
#if MVKIT_RECTIFICATION_SSE2
  const auto* in = std::get_if<AreaScanDivisionParameters>(&camera_in);
  const auto* out = std::get_if<AreaScanDivisionParameters>(&camera_out);
  const auto& cpu = runtime::cpu_features();
  if (in && out &&
      runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                              {true, false, false, false})) {
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    const auto cx = _mm_set1_pd(in->cx), cy = _mm_set1_pd(in->cy);
    const auto sx = _mm_set1_pd(in->sx), sy = _mm_set1_pd(in->sy), kappa = _mm_set1_pd(in->kappa);
    const auto ox = _mm_set1_pd(out->cx), oy = _mm_set1_pd(out->cy);
    const auto osx = _mm_set1_pd(out->sx), osy = _mm_set1_pd(out->sy);
    const bool inverse = std::abs(out->kappa) >= 1e-30;
    const auto four_kappa = _mm_set1_pd(4. * out->kappa), one = _mm_set1_pd(1.);
    for (; input.size() - index >= 2; index += 2) {
      const auto first = _mm_loadu_pd(&input[index].row), second = _mm_loadu_pd(&input[index + 1].row);
      const auto rows = _mm_unpacklo_pd(first, second), cols = _mm_unpackhi_pd(first, second);
      auto valid = _mm_and_pd(finite(rows), finite(cols));
      const auto u = _mm_mul_pd(_mm_sub_pd(cols, cx), sx), v = _mm_mul_pd(_mm_sub_pd(rows, cy), sy);
      const auto radius = _mm_add_pd(_mm_mul_pd(u, u), _mm_mul_pd(v, v));
      const auto denominator = _mm_add_pd(one, _mm_mul_pd(kappa, radius));
      valid = _mm_and_pd(valid, _mm_and_pd(_mm_and_pd(finite(u), finite(v)), finite(denominator)));
      valid =
          _mm_and_pd(valid, _mm_cmpge_pd(_mm_andnot_pd(_mm_set1_pd(-0.), denominator), _mm_set1_pd(1e-15)));
      if (_mm_movemask_pd(valid) != 3) break;
      auto x = _mm_div_pd(u, denominator), y = _mm_div_pd(v, denominator);
      if (_mm_movemask_pd(_mm_and_pd(finite(x), finite(y))) != 3) break;
      if (inverse) {
        const auto squared = _mm_add_pd(_mm_mul_pd(x, x), _mm_mul_pd(y, y));
        const auto radicand = _mm_sub_pd(one, _mm_mul_pd(four_kappa, squared));
        if (_mm_movemask_pd(_mm_and_pd(finite(radicand), _mm_cmpge_pd(radicand, _mm_setzero_pd()))) != 3)
          break;
        const auto scale = _mm_div_pd(_mm_set1_pd(2.), _mm_add_pd(one, _mm_sqrt_pd(radicand)));
        x = _mm_mul_pd(x, scale);
        y = _mm_mul_pd(y, scale);
      }
      const auto result_rows = _mm_add_pd(_mm_div_pd(y, osy), oy);
      const auto result_cols = _mm_add_pd(_mm_div_pd(x, osx), ox);
      _mm_storeu_pd(&output[index].row, _mm_unpacklo_pd(result_rows, result_cols));
      _mm_storeu_pd(&output[index + 1].row, _mm_unpackhi_pd(result_rows, result_cols));
    }
  }
#endif
  for (; index < input.size(); ++index) {
    const auto p = input[index];
    if (!std::isfinite(p.row) || !std::isfinite(p.col))
      throw core::Exception(core::Status::InvalidArgument, op, "contour coordinates must be finite");
    const auto metric = image_point_to_undistorted_metric(camera_in, {p.row, p.col});
    const auto point = undistorted_metric_to_image_point(camera_out, metric);
    output[index] = {point.row, point.col};
  }
}

core::XldContours map_contours(const core::XldContours& contours, const auto& map_points, const char* op) {
  std::vector<core::IconicValue> values;
  values.reserve(contours.contour_count());
  for (std::size_t i = 0; i < contours.contour_count(); ++i) {
    const auto& contour = contours.contour(i);
    core::XldContour::OwnedPointPayload points;
    std::array<core::Point2d, 16> inline_points;
    const bool use_inline = contour.size() <= inline_points.size();
    if (!use_inline) points.reset_for_overwrite(contour.size());
    auto* output = use_inline ? inline_points.data() : points.data();
    const auto input = contour.points();
    if (!input.empty()) {
      const auto first = input.front();
      if (!std::isfinite(first.row) || !std::isfinite(first.col))
        throw core::Exception(core::Status::InvalidArgument, op, "contour coordinates must be finite");
      map_points(input, std::span<core::Point2d>{output, contour.size()});
    }
    values.push_back(core::IconicValue::from_xld_contour(
        use_inline
            ? core::XldContour::from_inline_points({output, contour.size()}, contour.closed(),
                                                   core::ContourApproximation::Unknown, {}, false, nullptr)
            : core::XldContour::from_owned_point_payload(std::move(points), contour.closed(),
                                                         core::ContourApproximation::Unknown, {}, false,
                                                         nullptr)));
  }
  return core::XldContours{core::IconicTuple{std::move(values)}};
}

bool same_family(const CameraParameters& a, const CameraParameters& b) { return a.index() == b.index(); }

}  // namespace

double world_plane_meters_per_pixel(const core::Tuple& scale, const char* op) {
  if (scale.size() != 1) {
    throw core::Exception(core::Status::InvalidArgument, op, "scale must contain one value");
  }
  if (scale.at(0).kind() == core::ValueKind::String) {
    const auto unit = scale.string_view(0);
    if (unit == "m") return 1.0;
    if (unit == "cm") return 0.01;
    if (unit == "mm") return 0.001;
    if (unit == "um") return 1e-6;
    throw core::Exception(core::Status::InvalidArgument, op, "invalid world-plane scale");
  }
  try {
    const double value = scale.as_real64(0);
    if (!std::isfinite(value) || value <= 0.0) {
      throw core::Exception(core::Status::InvalidArgument, op,
                            "numeric scale must be positive meters per pixel");
    }
    return value;
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::InvalidArgument) throw;
    throw core::Exception(core::Status::InvalidArgument, op, "invalid world-plane scale");
  }
}

namespace {
template <class Scalar>
struct RadialGridSample {
  const Scalar& scalar;
  const AreaScanDivisionParameters& in;
  const AreaScanDivisionParameters& out;
  auto operator()(std::size_t r, std::size_t c) const { return scalar(r, c); }
  std::size_t fill_row(std::size_t r, std::size_t width, float* rows, float* cols) const {
    std::size_t c = 0;
#if MVKIT_RECTIFICATION_SSE2
    if (width < 2 || width > std::size_t(INT32_MAX)) return 0;
    const auto& cpu = runtime::cpu_features();
    if (!runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                 {true, false, false, false}))
      return 0;
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    const auto cx = _mm_set1_pd(out.cx), sx = _mm_set1_pd(out.sx);
    const auto v = _mm_set1_pd((double(r) - out.cy) * out.sy);
    const auto kappa = _mm_set1_pd(out.kappa), one = _mm_set1_pd(1.);
    const auto isx = _mm_set1_pd(in.sx), isy = _mm_set1_pd(in.sy);
    const auto icx = _mm_set1_pd(in.cx), icy = _mm_set1_pd(in.cy);
    const bool inverse = std::abs(in.kappa) >= 1e-30;
    for (; width - c >= 2; c += 2) {
      const auto column = _mm_cvtepi32_pd(_mm_set_epi32(0, 0, int(c + 1), int(c)));
      const auto u = _mm_mul_pd(_mm_sub_pd(column, cx), sx);
      const auto radius = _mm_add_pd(_mm_mul_pd(u, u), _mm_mul_pd(v, v));
      const auto denominator = _mm_add_pd(one, _mm_mul_pd(kappa, radius));
      auto valid = _mm_and_pd(_mm_and_pd(finite(u), finite(v)), finite(denominator));
      valid =
          _mm_and_pd(valid, _mm_cmpge_pd(_mm_andnot_pd(_mm_set1_pd(-0.), denominator), _mm_set1_pd(1e-15)));
      if (_mm_movemask_pd(valid) != 3) break;
      auto x = _mm_div_pd(u, denominator), y = _mm_div_pd(v, denominator);
      if (_mm_movemask_pd(_mm_and_pd(finite(x), finite(y))) != 3) break;
      if (inverse) {
        const auto squared = _mm_add_pd(_mm_mul_pd(x, x), _mm_mul_pd(y, y));
        const auto radicand = _mm_sub_pd(one, _mm_mul_pd(_mm_set1_pd(4. * in.kappa), squared));
        valid = _mm_and_pd(finite(radicand), _mm_cmpge_pd(radicand, _mm_setzero_pd()));
        if (_mm_movemask_pd(valid) != 3) break;
        const auto scale = _mm_div_pd(_mm_set1_pd(2.), _mm_add_pd(one, _mm_sqrt_pd(radicand)));
        x = _mm_mul_pd(x, scale);
        y = _mm_mul_pd(y, scale);
      }
      // 保留最终坐标允许溢出的原行为。
      const auto rr = _mm_add_pd(_mm_div_pd(y, isy), icy);
      const auto cc = _mm_add_pd(_mm_div_pd(x, isx), icx);
      _mm_storel_pi(reinterpret_cast<__m64*>(rows + c), _mm_cvtpd_ps(rr));
      _mm_storel_pi(reinterpret_cast<__m64*>(cols + c), _mm_cvtpd_ps(cc));
    }
#endif
    return c;
  }
};
}  // namespace

void gen_radial_distortion_map(const core::Tuple& camera_param_in, const core::Tuple& camera_param_out,
                               core::Image* row_map, core::Image* col_map) {
  constexpr auto op = "gen_radial_distortion_map";
  require_outputs(row_map, col_map, op);
  const auto camera_in = parse_camera_parameters(camera_param_in);
  const auto camera_out = parse_camera_parameters(camera_param_out);
  if (!same_family(camera_in, camera_out)) {
    throw core::Exception(core::Status::InvalidArgument, op,
                          "input and output cameras must use the same model family");
  }
  const auto& common = common_parameters(camera_out);
  const auto width = static_cast<std::size_t>(common.image_width);
  const auto height = static_cast<std::size_t>(common.image_height);
  const auto* in = std::get_if<AreaScanDivisionParameters>(&camera_in);
  const auto* out = std::get_if<AreaScanDivisionParameters>(&camera_out);
  if (in && out) {
    const auto scalar = [&](std::size_t r, std::size_t c) -> std::optional<Point2d> {
      const double u = (double(c) - out->cx) * out->sx, v = (double(r) - out->cy) * out->sy;
      const double denominator = 1. + out->kappa * (u * u + v * v);
      if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(denominator) ||
          std::abs(denominator) < 1e-15)
        return std::nullopt;
      double x = u / denominator, y = v / denominator;
      if (!std::isfinite(x) || !std::isfinite(y)) return std::nullopt;
      if (std::abs(in->kappa) >= 1e-30) {
        const double radicand = 1. - 4. * in->kappa * (x * x + y * y);
        if (radicand < 0 || !std::isfinite(radicand)) return std::nullopt;
        const double scale = 2. / (1. + std::sqrt(radicand));
        x *= scale;
        y *= scale;
      }
      return Point2d{y / in->sy + in->cy, x / in->sx + in->cx};
    };
    fill_maps(width, height, row_map, col_map, RadialGridSample{scalar, *in, *out});
    return;
  }
  fill_maps(width, height, row_map, col_map, [&](std::size_t r, std::size_t c) {
    return try_source_from_output(camera_in, camera_out, {static_cast<double>(r), static_cast<double>(c)});
  });
}

core::Tuple change_radial_distortion_cam_par(const core::Tuple& camera_param_in,
                                             const core::Tuple& distortion) {
  constexpr auto op = "change_radial_distortion_cam_par";
  auto camera = parse_camera_parameters(camera_param_in);
  if (distortion.empty()) {
    std::visit(
        [&](auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                        std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
            p.kappa = 0.0;
          } else {
            p.k1 = p.k2 = p.k3 = p.p1 = p.p2 = 0.0;
          }
        },
        camera);
    return camera_parameters_to_tuple(camera);
  }
  try {
    const auto source = parse_camera_parameters(distortion);
    if (!same_family(camera, source)) {
      throw core::Exception(core::Status::InvalidArgument, op,
                            "distortion camera must use the same model family");
    }
    std::visit(
        [&](auto& p) {
          using T = std::decay_t<decltype(p)>;
          std::visit(
              [&](const auto& s) {
                using S = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<T, S>) {
                  if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                                std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
                    p.kappa = s.kappa;
                  } else {
                    p.k1 = s.k1;
                    p.k2 = s.k2;
                    p.k3 = s.k3;
                    p.p1 = s.p1;
                    p.p2 = s.p2;
                  }
                }
              },
              source);
        },
        camera);
    return camera_parameters_to_tuple(camera);
  } catch (const core::Exception& ex) {
    if (std::string_view(ex.where()) == op) throw;
  }
  auto set_kappa = [&](double kappa) {
    if (!std::isfinite(kappa)) {
      throw core::Exception(core::Status::InvalidArgument, op, "distortion must be finite");
    }
    std::visit(
        [&](auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                        std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
            p.kappa = kappa;
          } else {
            throw core::Exception(core::Status::InvalidArgument, op,
                                  "polynomial cameras require five distortion coefficients");
          }
        },
        camera);
  };
  if (distortion.size() == 1) {
    try {
      set_kappa(distortion.as_real64(0));
    } catch (const core::Exception& ex) {
      if (ex.status() == core::Status::InvalidArgument && std::string_view(ex.where()) == op) {
        throw;
      }
      throw core::Exception(core::Status::InvalidArgument, op, "distortion must be numeric");
    }
    return camera_parameters_to_tuple(camera);
  }
  if (distortion.size() == 5) {
    double k[5]{};
    for (std::size_t i = 0; i < 5; ++i) {
      try {
        k[i] = distortion.as_real64(i);
      } catch (const core::Exception&) {
        throw core::Exception(core::Status::InvalidArgument, op, "distortion must be numeric");
      }
      if (!std::isfinite(k[i])) {
        throw core::Exception(core::Status::InvalidArgument, op, "distortion must be finite");
      }
    }
    std::visit(
        [&](auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, AreaScanPolynomialParameters> ||
                        std::is_same_v<T, AreaScanTelecentricPolynomialParameters>) {
            p.k1 = k[0];
            p.k2 = k[1];
            p.k3 = k[2];
            p.p1 = k[3];
            p.p2 = k[4];
          } else {
            throw core::Exception(core::Status::InvalidArgument, op,
                                  "division cameras require one kappa coefficient");
          }
        },
        camera);
    return camera_parameters_to_tuple(camera);
  }
  throw core::Exception(core::Status::InvalidArgument, op, "unsupported distortion specification");
}

namespace {
// 批量处理独立坐标列，保持标量运算序；非法点对及不支持相机恢复原标量路径。
template <class Scalar>
struct ForwardGridSample {
  const Scalar& scalar;
  const AreaScanDivisionParameters* camera;
  const std::array<double, 9>& rot;
  const Pose3d& pose;
  double meters;
  auto operator()(std::size_t r, std::size_t c) const { return scalar(r, c); }
  std::size_t fill_row(std::size_t r, std::size_t width, float* rows, float* cols) const {
    std::size_t c = 0;
#if MVKIT_RECTIFICATION_SSE2
    if (!camera || camera->kappa != 0 || width < 2 || width > std::size_t(INT32_MAX)) return 0;
    const auto& cpu = runtime::cpu_features();
    if (!runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                 {true, false, false, false}))
      return 0;
    const auto finite = [](__m128d x) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), x),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    const auto y = _mm_set1_pd(static_cast<double>(r) * meters), zero = _mm_setzero_pd();
    const auto m = _mm_set1_pd(meters), focus = _mm_set1_pd(camera->focus);
    const auto sx = _mm_set1_pd(camera->sx), sy = _mm_set1_pd(camera->sy);
    const auto cx = _mm_set1_pd(camera->cx), cy = _mm_set1_pd(camera->cy);
    const auto coordinate = [&](__m128d x, int k, double t) {
      return _mm_add_pd(
          _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(rot[k]), x), _mm_mul_pd(_mm_set1_pd(rot[k + 1]), y)),
                     _mm_mul_pd(_mm_set1_pd(rot[k + 2]), zero)),
          _mm_set1_pd(t));
    };
    for (; width - c >= 2; c += 2) {
      const auto x = _mm_mul_pd(_mm_cvtepi32_pd(_mm_set_epi32(0, 0, int(c + 1), int(c))), m);
      const auto px = coordinate(x, 0, pose.translation.x), py = coordinate(x, 3, pose.translation.y);
      const auto pz = coordinate(x, 6, pose.translation.z);
      const auto valid = _mm_and_pd(_mm_and_pd(finite(px), finite(py)),
                                    _mm_and_pd(finite(pz), _mm_cmpgt_pd(pz, _mm_set1_pd(1e-12))));
      if (_mm_movemask_pd(valid) != 3) break;
      const auto u = _mm_div_pd(_mm_mul_pd(focus, px), pz), v = _mm_div_pd(_mm_mul_pd(focus, py), pz);
      if (_mm_movemask_pd(_mm_and_pd(finite(u), finite(v))) != 3) break;
      // 旧快路径允许最终坐标溢出，保留此行为。
      const auto rr = _mm_add_pd(_mm_div_pd(v, sy), cy), cc = _mm_add_pd(_mm_div_pd(u, sx), cx);
      _mm_storel_pi(reinterpret_cast<__m64*>(rows + c), _mm_cvtpd_ps(rr));
      _mm_storel_pi(reinterpret_cast<__m64*>(cols + c), _mm_cvtpd_ps(cc));
    }
#endif
    return c;
  }
};

}  // namespace

void gen_image_to_world_plane_map(const core::Tuple& camera_param, const core::Tuple& world_pose,
                                  const core::Tuple& scale, std::size_t width, std::size_t height,
                                  core::Image* row_map, core::Image* col_map) {
  constexpr auto op = "gen_image_to_world_plane_map";
  require_outputs(row_map, col_map, op);
  if (width == 0 || height == 0) {
    throw core::Exception(core::Status::InvalidArgument, op, "output dimensions must be positive");
  }
  const auto camera = parse_camera_parameters(camera_param);
  const auto pose = pose_from_halcon_tuple(world_pose);
  const double meters = world_plane_meters_per_pixel(scale, op);
  const auto rotation = rotation_matrix(pose.rotation);
  const auto* division = std::get_if<AreaScanDivisionParameters>(&camera);
  const auto scalar = [&](std::size_t r, std::size_t c) -> std::optional<Point2d> {
    try {
      const Point3d world{static_cast<double>(c) * meters, static_cast<double>(r) * meters, 0.0};
      const Point3d point{
          rotation[0] * world.x + rotation[1] * world.y + rotation[2] * world.z + pose.translation.x,
          rotation[3] * world.x + rotation[4] * world.y + rotation[5] * world.z + pose.translation.y,
          rotation[6] * world.x + rotation[7] * world.y + rotation[8] * world.z + pose.translation.z};
      if (division && division->kappa == 0 && std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && point.z > 1e-12) {
        const auto u = division->focus * point.x / point.z, v = division->focus * point.y / point.z;
        if (std::isfinite(u) && std::isfinite(v))
          return Point2d{v / division->sy + division->cy, u / division->sx + division->cx};
      }
      return project_camera_point(camera, point);
    } catch (const core::Exception&) {
      return std::nullopt;
    }
  };
  fill_maps(width, height, row_map, col_map, ForwardGridSample{scalar, division, rotation, pose, meters});
}

core::XldContours contour_to_world_plane_xld(const core::XldContours& contours,
                                             const WorldPlaneProjection& project) {
  constexpr auto op = "contour_to_world_plane_xld";
  return map_contours(contours, [&](auto input, auto output) { project.map_points(input, output, op); }, op);
}

core::XldContours change_radial_distortion_contours_xld(const core::XldContours& contours,
                                                        const CameraParameters& camera_in,
                                                        const CameraParameters& camera_out) {
  constexpr auto op = "change_radial_distortion_contours_xld";
  return map_contours(
      contours,
      [&](auto input, auto output) { map_distortion_points(camera_in, camera_out, input, output, op); }, op);
}

core::XldContours contour_to_world_plane_xld(const core::XldContours& contours,
                                             const core::Tuple& camera_param, const core::Tuple& world_pose,
                                             const core::Tuple& scale) {
  constexpr auto op = "contour_to_world_plane_xld";
  // 空轮廓不解析参数；首个非法坐标仍先于相机/位姿校验，在首个有限输入点准备。
  std::optional<WorldPlaneProjection> project;
  return map_contours(
      contours,
      [&](auto input, auto output) {
        if (!project) project.emplace(WorldPlaneProjection::prepare(camera_param, world_pose, scale));
        project->map_points(input, output, op);
      },
      op);
}

core::XldContours change_radial_distortion_contours_xld(const core::XldContours& contours,
                                                        const core::Tuple& camera_param_in,
                                                        const core::Tuple& camera_param_out) {
  constexpr auto op = "change_radial_distortion_contours_xld";
  std::optional<std::pair<CameraParameters, CameraParameters>> cameras;
  return map_contours(
      contours,
      [&](auto input, auto output) {
        if (!cameras) {
          auto camera_in = parse_camera_parameters(camera_param_in);
          auto camera_out = parse_camera_parameters(camera_param_out);
          cameras.emplace(std::move(camera_in), std::move(camera_out));
        }
        map_distortion_points(cameras->first, cameras->second, input, output, op);
      },
      op);
}

}  // namespace mv::calibration

namespace mv::calibration {
namespace {
struct PreparedGrid {
  CameraParameters camera, target;
  DivisionProjection from, to;
  Pose3d pose{};
  std::array<double, 9> rotation{};
  double meters = 1.;
  bool world = false;
  PreparedGrid(CameraParameters source, CameraParameters destination)
      : camera(std::move(source)), target(std::move(destination)), from(camera), to(target) {}
  std::optional<Point2d> operator()(std::size_t row, std::size_t col) const {
    if (from.enabled() && (world || to.enabled())) {
      if (!world) {
        const auto metric = to.undistort({double(row), double(col)});
        return metric ? from.distort(*metric) : std::nullopt;
      }
      const double x = double(col) * meters, y = double(row) * meters;
      Point3d point{rotation[0] * x + rotation[1] * y + pose.translation.x,
                    rotation[3] * x + rotation[4] * y + pose.translation.y,
                    rotation[6] * x + rotation[7] * y + pose.translation.z};
      if (from.perspective() && point.z < 0) point = {-point.x, -point.y, -point.z};
      return from.project(point);
    }
    try {
      if (!world) {
        auto metric = image_point_to_undistorted_metric(target, {double(row), double(col)});
        // HALCON 映射度量畸变坐标时不重缩放焦距。
        return undistorted_metric_to_image_point(camera, metric);
      }
      const double x = double(col) * meters, y = double(row) * meters;
      Point3d point{rotation[0] * x + rotation[1] * y + pose.translation.x,
                    rotation[3] * x + rotation[4] * y + pose.translation.y,
                    rotation[6] * x + rotation[7] * y + pose.translation.z};
      if (is_perspective(camera) && point.z < 0) point = {-point.x, -point.y, -point.z};
      return project_camera_point(camera, point);
    } catch (const core::Exception&) {
      return std::nullopt;
    }
  }
};
PreparedGrid radial_grid(const core::Tuple& in, const core::Tuple& out) {
  PreparedGrid grid{parse_camera_parameters(in), parse_camera_parameters(out)};
  if (!same_family(grid.camera, grid.target))
    throw core::Exception(core::Status::InvalidArgument, "gen_radial_distortion_map",
                          "input/output cameras require the same model family");
  return grid;
}
PreparedGrid world_grid(const core::Tuple& camera, const core::Tuple& pose, const core::Tuple& scale) {
  PreparedGrid grid{parse_camera_parameters(camera), {}};
  grid.world = true;
  grid.pose = pose_from_halcon_tuple(pose);
  grid.rotation = rotation_matrix(grid.pose.rotation);
  if (scale.size() == 1 && scale.at(0).kind() == core::ValueKind::Real64 &&
      std::isnan(scale.at(0).as_real64())) {
    grid.meters = scale.at(0).as_real64();
    return grid;  // NaN scale 产生空定义域。
  }
  grid.meters = world_plane_meters_per_pixel(
      scale.size() == 1 && scale.at(0).kind() == core::ValueKind::String && scale.string_view() == "microns"
          ? core::Tuple("um")
          : scale,
      "image_to_world_plane");
  return grid;
}
bool map_nearest(std::string_view mode, const char* op) {
  if (mode == "nearest_neighbor") return true;
  if (mode == "bilinear") return false;
  const auto parameter = std::string_view(op) == "image_to_world_plane" ? "Interpolation" : "MapType";
  throw core::Exception(
      mode == "coord_map_sub_pix" && std::string_view(parameter) == "MapType" ? core::Status::Unsupported
                                                                              : core::Status::InvalidArgument,
      core::ErrorCode::UnsupportedValue, op, parameter, 0,
      "nearest_neighbor or bilinear map required; vector_field image storage is not implemented");
}
struct GridSampler {
  const PreparedGrid& grid;
  std::vector<double> rows, columns;
  bool separable = false, batch_world = false;
  double projection_scale = 1.;
  GridSampler(const PreparedGrid& prepared, std::size_t width, std::size_t height) : grid(prepared) {
#if MVKIT_RECTIFICATION_SSE2
    const auto& cpu = runtime::cpu_features();
    batch_world = grid.world && grid.from.linear_distortion() && std::isfinite(grid.meters) &&
        runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                {true, false, false, false});
    if (batch_world) {
      std::visit([&](const auto& camera) {
        if constexpr (requires { camera.focus; }) projection_scale = camera.focus;
        else projection_scale = camera.magnification;
      }, grid.camera);
    }
#endif
    if (grid.world || !grid.from.linear_distortion() || !grid.to.zero_undistortion()) return;
    const auto& from = common_parameters(grid.camera);
    const auto& to = common_parameters(grid.target);
    rows.resize(height); columns.resize(width);
    double max_u2 = 0., max_v2 = 0.;
    for (std::size_t c = 0; c < width; ++c) {
      const double metric = (double(c) - to.cx) * to.sx;
      if (!std::isfinite(metric)) return;
      max_u2 = std::max(max_u2, metric * metric);
      columns[c] = metric / from.sx + from.cx;
    }
    for (std::size_t r = 0; r < height; ++r) {
      const double metric = (double(r) - to.cy) * to.sy;
      if (!std::isfinite(metric)) return;
      max_v2 = std::max(max_v2, metric * metric);
      rows[r] = metric / from.sy + from.cy;
    }
    // 零系数仍有 0*Inf 的域检查；证明全部 r2 有限后才分离两轴。
    // 不取理想整数坐标，保留原乘除及主点加法产生的浮点残差。
    separable = std::isfinite(max_u2 + max_v2);
  }
  std::optional<Point2d> operator()(std::size_t row, std::size_t column) const {
    if (separable) return Point2d{rows[row], columns[column]};
    return grid(row, column);
  }
  void fill_world_row(std::size_t row, std::span<core::Point2d> output) const {
    std::size_t column = 0;
    const auto replay = [&](std::size_t first, std::size_t count) {
      for (std::size_t i = first; i < first + count; ++i) {
        const auto point = grid(row, i);
        output[i] = point ? core::Point2d{point->row, point->col}
            : core::Point2d{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
      }
    };
#if MVKIT_RECTIFICATION_SSE2
    const auto& camera = common_parameters(grid.camera);
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    const double y = double(row) * grid.meters;
    const auto transform = [&](std::size_t axis, __m128d x, double translation) {
      return _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(grid.rotation[axis]), x),
                                    _mm_set1_pd(grid.rotation[axis + 1] * y)),
                         _mm_set1_pd(translation));
    };
    for (; output.size() - column >= 2; column += 2) {
      const auto x = _mm_mul_pd(_mm_set_pd(double(column + 1), double(column)), _mm_set1_pd(grid.meters));
      auto px = transform(0, x, grid.pose.translation.x);
      auto py = transform(3, x, grid.pose.translation.y);
      auto pz = transform(6, x, grid.pose.translation.z);
      auto valid = _mm_and_pd(_mm_and_pd(finite(px), finite(py)), finite(pz));
      if (grid.from.perspective()) {
        const auto sign = _mm_and_pd(_mm_cmplt_pd(pz, _mm_setzero_pd()), _mm_set1_pd(-0.));
        px = _mm_xor_pd(px, sign); py = _mm_xor_pd(py, sign); pz = _mm_xor_pd(pz, sign);
        valid = _mm_and_pd(valid, _mm_cmpgt_pd(pz, _mm_set1_pd(1e-12)));
      }
      if (_mm_movemask_pd(valid) != 3) { replay(column, 2); continue; }
      auto u = _mm_mul_pd(_mm_set1_pd(projection_scale), px);
      auto v = _mm_mul_pd(_mm_set1_pd(projection_scale), py);
      if (grid.from.perspective()) { u = _mm_div_pd(u, pz); v = _mm_div_pd(v, pz); }
      valid = _mm_and_pd(valid, _mm_and_pd(finite(u), finite(v)));
      if (_mm_movemask_pd(valid) != 3) { replay(column, 2); continue; }
      const auto rr = _mm_add_pd(_mm_div_pd(v, _mm_set1_pd(camera.sy)), _mm_set1_pd(camera.cy));
      const auto cc = _mm_add_pd(_mm_div_pd(u, _mm_set1_pd(camera.sx)), _mm_set1_pd(camera.cx));
      _mm_storeu_pd(&output[column].row, _mm_unpacklo_pd(rr, cc));
      _mm_storeu_pd(&output[column + 1].row, _mm_unpackhi_pd(rr, cc));
    }
#endif
    replay(column, output.size() - column);
  }
};
core::Region pixel_domain(const std::vector<std::uint8_t>& valid, std::size_t width, std::size_t height) {
  std::pmr::vector<core::RegionRun> runs{runtime::persist_resource()};
  runs.reserve(height);
  for (std::size_t r = 0; r < height; ++r) {
    std::size_t c = 0;
    while (c < width) {
      while (c < width && !valid[r * width + c]) ++c;
      const auto first = c;
      while (c < width && valid[r * width + c]) ++c;
      if (first < c)
        runs.push_back({static_cast<std::int64_t>(r), static_cast<std::int64_t>(first),
                        static_cast<std::int64_t>(c - 1)});
    }
  }
  return core::Region::from_canonical_runs(std::move(runs));
}
core::Image encode_grid(const PreparedGrid& grid, std::size_t width_in, std::size_t height_in,
                        std::size_t width, std::size_t height, bool nearest) {
  if (width_in == 0 || height_in == 0 || width_in > std::size_t(INT32_MAX) / height_in)
    throw core::Exception(core::Status::InvalidArgument, "encode_grid",
                          "positive int4-representable input dimensions required");

  const std::array types{core::PixelType::Int32, core::PixelType::UInt16, core::PixelType::UInt16,
                         core::PixelType::UInt16, core::PixelType::UInt16};
  auto image =
      core::Image::create(width, height, std::span<const core::PixelType>(types.data(), nearest ? 1 : 5));
  if (!nearest && (width_in < 2 || height_in < 2)) return image.reduce_domain(core::Region{});
  auto* indices = static_cast<std::byte*>(image.mutable_data());
  std::array<std::byte*, 4> weight{};
  std::array<std::size_t, 4> strides{};
  if (!nearest)
    for (std::size_t ch = 0; ch < 4; ++ch) {
      weight[ch] = static_cast<std::byte*>(image.mutable_data(ch + 1));
      strides[ch] = image.stride(ch + 1);
    }
  std::vector<std::uint8_t> valid(width * height);
  const GridSampler sample_grid(grid, width, height);
  runtime::ExecutionContext::for_each_partition(
      height, runtime::ExecutionContext::saturated_work(width, height), 65536,
      [&](runtime::IndexRange range) {
        std::vector<core::Point2d> projected;
        if (sample_grid.batch_world) projected.resize(width);
        for (auto r = range.begin; r < range.end; ++r) {
          if (sample_grid.batch_world) sample_grid.fill_world_row(r, projected);
          for (std::size_t c = 0; c < width; ++c) {
            const auto point = sample_grid.batch_world ? std::optional<Point2d>{Point2d{projected[c].row, projected[c].col}} : sample_grid(r, c);
            if (!point || !std::isfinite(point->row) || !std::isfinite(point->col)) continue;
            const double nr = std::round(point->row), nc = std::round(point->col);
            bool inside = nr >= 0 && nc >= 0 && nr < double(height_in) && nc < double(width_in);
            if (nearest && !grid.world)
              inside = inside && std::trunc(point->row) >= 0 && std::trunc(point->col) >= 0 &&
                       std::trunc(point->row) < double(height_in - 1) &&
                       std::trunc(point->col) < double(width_in - 1);
            if (!inside) continue;
            valid[r * width + c] = 1;
            auto* index = reinterpret_cast<std::int32_t*>(indices + r * image.stride()) + c;
            if (nearest) {
              *index = static_cast<std::int32_t>(nr * width_in + nc);
              continue;
            }
            const double rr = std::clamp(point->row, 0., double(height_in - 1)),
                         cc = std::clamp(point->col, 0., double(width_in - 1));
            const auto y = std::min(static_cast<std::size_t>(rr), height_in - 2),
                       x = std::min(static_cast<std::size_t>(cc), width_in - 2);
            const double dy = rr - y, dx = cc - x;
            *index = static_cast<std::int32_t>(y * width_in + x);
            const std::array weights{(1 - dy) * (1 - dx), (1 - dy) * dx, dy * (1 - dx), dy * dx};
            for (std::size_t ch = 0; ch < 4; ++ch)
              reinterpret_cast<std::uint16_t*>(weight[ch] + r * strides[ch])[c] =
                  static_cast<std::uint16_t>(std::clamp(weights[ch] * 65535., 0., 65535.));
          }
        }
      });
  return image.reduce_domain(pixel_domain(valid, width, height));
}
struct RectificationPlane {
  const std::byte* source;
  std::byte* output;
  std::size_t source_stride, output_stride;
  core::PixelType type;
};

template <class T>
void sample_plane(const RectificationPlane& plane, std::size_t row, std::size_t column,
                   std::size_t y, std::size_t x, std::size_t y1, std::size_t x1,
                   double dy, double dx, bool nearest, std::size_t nr, std::size_t nc) {
  const auto read = [&](std::size_t r, std::size_t c) {
    T value;
    std::memcpy(&value, plane.source + r * plane.source_stride + c * sizeof(T), sizeof(T));
    return double(value);
  };
  if constexpr (std::is_integral_v<T>) {
    if (nearest) {
      // 整数样本的原 Double/+0.5/floor/饱和结果就是自身，保持非对齐地址支持。
      T value;
      std::memcpy(&value, plane.source + nr * plane.source_stride + nc * sizeof(T), sizeof(T));
      std::memcpy(plane.output + row * plane.output_stride + column * sizeof(T), &value, sizeof(T));
      return;
    }
  }
  // 与原双线性表达式完全同序，只将类型和布局从四次邻域读取中移出。
  const double value = nearest ? read(nr, nc)
      : (1 - dy) * ((1 - dx) * read(y, x) + dx * read(y, x1)) +
             dy * ((1 - dx) * read(y1, x) + dx * read(y1, x1));
  T result;
  if constexpr (std::is_floating_point_v<T>) result = static_cast<T>(value);
  else result = static_cast<T>(std::clamp(std::floor(value + .5), 0., double(std::numeric_limits<T>::max())));
  std::memcpy(plane.output + row * plane.output_stride + column * sizeof(T), &result, sizeof(T));
}

core::Image rectify_image(const core::Image& source, const PreparedGrid& grid, std::size_t width,
                          std::size_t height, bool nearest, const core::Region* region) {
  std::vector<core::PixelType> types;
  for (std::size_t ch = 0; ch < source.channel_count(); ++ch) {
    const auto type = source.pixel_type(ch);
    if (type != core::PixelType::UInt8 && type != core::PixelType::UInt16 && type != core::PixelType::Float32)
      throw core::Exception(core::Status::TypeMismatch, "rectify_image",
                            "byte, uint2 or real input required");
    types.push_back(type);
  }
  auto result = core::Image::create(width, height, std::span<const core::PixelType>(types));
  std::vector<RectificationPlane> planes;
  for (std::size_t ch = 0; ch < types.size(); ++ch)
    planes.push_back({static_cast<const std::byte*>(source.data(ch)),
                      static_cast<std::byte*>(result.mutable_data(ch)), source.stride(ch), result.stride(ch), types[ch]});
  const auto source_width = source.width(), source_height = source.height();
  const auto runs = region ? region->runs() : std::span<const core::RegionRun>{};
  std::vector<std::uint8_t> valid(width * height);
  const GridSampler sample_grid(grid, width, height);
  runtime::ExecutionContext::for_each_partition(
      height, runtime::ExecutionContext::saturated_work(width, height), 65536,
      [&](runtime::IndexRange range) {
        std::vector<core::Point2d> projected;
        if (sample_grid.batch_world) projected.resize(width);
        auto cursor = runs.begin();
        if (!runs.empty()) cursor = std::lower_bound(runs.begin(), runs.end(), range.begin,
            [](const core::RegionRun& run, std::size_t row) { return std::int64_t(run.row) < std::int64_t(row); });
        for (auto r = range.begin; r < range.end; ++r) {
          if (sample_grid.batch_world) sample_grid.fill_world_row(r, projected);
          const auto process = [&](std::size_t begin, std::size_t end) {
            for (std::size_t c = begin; c < end; ++c) {
              const auto point = sample_grid.batch_world ? std::optional<Point2d>{Point2d{projected[c].row, projected[c].col}} : sample_grid(r, c);
              if (!point || !std::isfinite(point->row) || !std::isfinite(point->col)) continue;
              const double nr = std::round(point->row), nc = std::round(point->col);
              if (nr < 0 || nc < 0 || nr >= double(source_height) || nc >= double(source_width)) continue;
              valid[r * width + c] = 1;
              const double rr = std::clamp(point->row, 0., double(source_height - 1)),
                           cc = std::clamp(point->col, 0., double(source_width - 1));
              const auto y = static_cast<std::size_t>(rr), x = static_cast<std::size_t>(cc);
              const auto y1 = std::min(y + 1, source_height - 1), x1 = std::min(x + 1, source_width - 1);
              const double dy = rr - y, dx = cc - x;
              for (const auto& plane : planes) {
                switch (plane.type) {
                case core::PixelType::UInt8:
                  sample_plane<std::uint8_t>(plane, r, c, y, x, y1, x1, dy, dx, nearest, std::size_t(nr), std::size_t(nc)); break;
                case core::PixelType::UInt16:
                  sample_plane<std::uint16_t>(plane, r, c, y, x, y1, x1, dy, dx, nearest, std::size_t(nr), std::size_t(nc)); break;
                case core::PixelType::Float32:
                  sample_plane<float>(plane, r, c, y, x, y1, x1, dy, dx, nearest, std::size_t(nr), std::size_t(nc)); break;
                default: break; // 类型已在输出分配前验证。
                }
              }
            }
          };
          if (!region) process(0, width);
          else {
            // Region 行程有序；每个 worker 独立游标，每个有效行程只访问一次。
            while (cursor != runs.end() && std::int64_t(cursor->row) < std::int64_t(r)) ++cursor;
            while (cursor != runs.end() && std::int64_t(cursor->row) == std::int64_t(r)) {
              const auto begin = std::max<std::int64_t>(0, cursor->col_begin);
              const auto end = std::min<std::int64_t>(width, std::int64_t(cursor->col_end) + 1);
              if (begin < end) process(std::size_t(begin), std::size_t(end));
              ++cursor;
            }
          }
        }
      });
  return result.reduce_domain(pixel_domain(valid, width, height));
}
}  // namespace
core::Image gen_radial_distortion_map_halcon(const core::Tuple& in, const core::Tuple& out,
                                             std::string_view type) {
  const auto grid = radial_grid(in, out);
  const auto& from = common_parameters(grid.camera);
  const auto& to = common_parameters(grid.target);
  return encode_grid(grid, from.image_width, from.image_height, to.image_width, to.image_height,
                     map_nearest(type, "gen_radial_distortion_map"));
}
core::Image gen_image_to_world_plane_map_halcon(const core::Tuple& camera, const core::Tuple& pose,
                                                std::size_t wi, std::size_t hi, std::size_t width,
                                                std::size_t height, const core::Tuple& scale,
                                                std::string_view type) {
  return encode_grid(world_grid(camera, pose, scale), wi, hi, width, height,
                     map_nearest(type, "gen_image_to_world_plane_map"));
}
core::IconicTuple change_radial_distortion_image_halcon(const core::IconicTuple& images,
                                                        const core::IconicTuple& regions,
                                                        const core::Tuple& in, const core::Tuple& out) {
  const auto grid = radial_grid(in, out);
  const auto& size = common_parameters(grid.target);
  const auto* region = regions.empty() ? nullptr : &regions.at(0).as_region();
  std::vector<core::Image> output;
  output.reserve(images.size());
  for (const auto& value : images.values())
    output.push_back(
        rectify_image(value.as_image(), grid, size.image_width, size.image_height, false, region));
  return core::make_image_tuple(std::move(output));
}
core::IconicTuple image_to_world_plane_halcon(const core::IconicTuple& images, const core::Tuple& camera,
                                              const core::Tuple& pose, std::size_t width, std::size_t height,
                                              const core::Tuple& scale, std::string_view interpolation) {
  const auto grid = world_grid(camera, pose, scale);
  const auto nearest = map_nearest(interpolation, "image_to_world_plane");
  std::vector<core::Image> output;
  output.reserve(images.size());
  for (const auto& value : images.values())
    output.push_back(rectify_image(value.as_image(), grid, width, height, nearest, nullptr));
  return core::make_image_tuple(std::move(output));
}
}  // namespace mv::calibration
