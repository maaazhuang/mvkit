#include <algorithm>
#include <calibration/camera_model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/projection.hpp>
#include <cmath>
#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <cstddef>
#include <limits>
#include <runtime/execution/execution_context.hpp>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_PROJECT_SSE2 1
#else
#define MVKIT_PROJECT_SSE2 0
#endif

namespace mv::calibration {
namespace {

struct NumericValues {
  NumericValues() = default;
  NumericValues(const NumericValues&) = delete;
  NumericValues(NumericValues&&) = default;
  std::vector<double> owned;
  std::span<const double> values;
  const double* data() const noexcept { return values.data(); }
  double operator[](std::size_t index) const noexcept { return values[index]; }
  std::size_t size() const noexcept { return values.size(); }
};

NumericValues broadcast_numeric(const core::Tuple& tuple, const core::BroadcastPlan& plan,
                                std::size_t argument, std::string_view operation,
                                bool allow_nonfinite = false) {
  if (tuple.kind() == core::TupleKind::Real64 && tuple.size() == plan.size()) {
    const auto values = tuple.real64_span();
    for (double value : values)
      if (!allow_nonfinite && !std::isfinite(value))
        throw core::Exception(core::Status::InvalidArgument, operation, "point values must be finite");
    NumericValues result;
    result.values = values;
    return result;
  }
  std::vector<double> values(plan.size());
  if (tuple.kind() == core::TupleKind::Real64 || tuple.kind() == core::TupleKind::Int64) {
    const bool scalar = tuple.size() == 1;
    if (tuple.kind() == core::TupleKind::Real64) {
      const auto source = tuple.real64_span();
      if (scalar)
        std::fill(values.begin(), values.end(), source.front());
      else
        std::copy(source.begin(), source.end(), values.begin());
    } else {
      const auto source = tuple.int64_span();
      if (scalar)
        std::fill(values.begin(), values.end(), double(source.front()));
      else
        std::transform(source.begin(), source.end(), values.begin(), [](auto v) { return double(v); });
    }
    for (double value : values)
      if (!allow_nonfinite && !std::isfinite(value))
        throw core::Exception(core::Status::InvalidArgument, operation, "point values must be finite");
  } else {
    for (std::size_t index = 0; index < plan.size(); ++index) {
      try {
        values[index] = tuple.at(plan.source_index(argument, index)).as_real64();
      } catch (const core::Exception&) {
        throw core::Exception(core::Status::InvalidArgument, operation,
                              "point tuples must contain numeric values");
      }
      if (!allow_nonfinite && !std::isfinite(values[index])) {
        throw core::Exception(core::Status::InvalidArgument, operation, "point values must be finite");
      }
    }
  }
  NumericValues result;
  result.owned = std::move(values);
  result.values = result.owned;
  return result;
}

double scale_factor(const core::Tuple& scale) {
  constexpr std::string_view kOperation = "image_points_to_world_plane";
  if (scale.size() != 1) {
    throw core::Exception(core::Status::InvalidArgument, kOperation, "scale must contain one value");
  }
  try {
    const double value = scale.at(0).as_real64();
    if (!std::isfinite(value) || value <= 0) {
      throw core::Exception(core::Status::InvalidArgument, kOperation, "scale must be positive");
    }
    return value;
  } catch (const core::Exception&) {
    try {
      const auto value = scale.string_view(0);
      if (value == "m") return 1.0;
      if (value == "cm") return 100.0;
      if (value == "mm") return 1000.0;
      if (value == "um") return 1e6;
    } catch (const core::Exception&) {
    }
    throw core::Exception(core::Status::InvalidArgument, kOperation, "invalid scale");
  }
}

}  // namespace

WorldPlaneProjection::WorldPlaneProjection(CameraParameters camera, Pose3d camera_in_world, double factor,
                                           bool halcon_contract)
    : camera_(std::move(camera)),
      camera_in_world_(camera_in_world),
      factor_(factor),
      rotation_(rotation_matrix(camera_in_world_.rotation)),
      perspective_(is_perspective(camera_)),
      halcon_contract_(halcon_contract),
      perspective_origin_(transform_point(camera_in_world_, {0, 0, 0})) {}

WorldPlaneProjection WorldPlaneProjection::prepare(const core::Tuple& camera_param,
                                                   const core::Tuple& world_pose, const core::Tuple& scale,
                                                   bool halcon_contract) {
  // 显式语句保留相机→位姿→尺度的原错误顺序。
  auto camera = parse_camera_parameters(camera_param);
  const auto world_in_camera = pose_from_halcon_tuple(world_pose);
  const auto camera_in_world = inverse_pose(world_in_camera);
  auto factor = scale_factor(scale);
  // HALCON 数值 Scale 指定以米表示的输出单位；私有历史前向射线辅助函数将其解释为乘数。
  if (halcon_contract && scale.at(0).kind() != core::ValueKind::String) factor = 1.0 / factor;
  return WorldPlaneProjection(std::move(camera), camera_in_world, factor, halcon_contract);
}

Point2d WorldPlaneProjection::operator()(Point2d point) const {
  constexpr std::string_view kOperation = "image_points_to_world_plane";
  const auto rotate = [&](const Point3d& p) {
    return Point3d{rotation_[0] * p.x + rotation_[1] * p.y + rotation_[2] * p.z,
                   rotation_[3] * p.x + rotation_[4] * p.y + rotation_[5] * p.z,
                   rotation_[6] * p.x + rotation_[7] * p.y + rotation_[8] * p.z};
  };
  const auto* division = std::get_if<AreaScanDivisionParameters>(&camera_);
  Ray3d ray;
  if (division && division->kappa == 0) {
    const auto u = (point.col - division->cx) * division->sx;
    const auto v = (point.row - division->cy) * division->sy;
    if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(u * u + v * v))
      ray = image_point_to_ray(camera_, point);
    else
      ray = {{0, 0, 0}, {u / division->focus, v / division->focus, 1}};
  } else
    ray = image_point_to_ray(camera_, point);
  auto origin = perspective_ ? perspective_origin_ : camera_in_world_.translation;
  if (!perspective_) {
    const auto p = rotate(ray.origin);
    origin = {p.x + origin.x, p.y + origin.y, p.z + origin.z};
  }
  const auto direction = rotate(ray.direction);
  if (std::abs(direction.z) < 1e-14)
    throw core::Exception(core::Status::InvalidState, kOperation, "ray is parallel to world plane");
  const double distance = -origin.z / direction.z;
  if (!std::isfinite(distance) || (!halcon_contract_ && distance <= 0))
    throw core::Exception(core::Status::InvalidState, kOperation,
                          "world plane intersection is behind camera ray");
  return {(origin.y + distance * direction.y) * factor_, (origin.x + distance * direction.x) * factor_};
}

void WorldPlaneProjection::map_points(std::span<const core::Point2d> input, std::span<core::Point2d> output,
                                      std::string_view operation) const {
  std::size_t index = 0;
#if MVKIT_PROJECT_SSE2
  const auto* division = std::get_if<AreaScanDivisionParameters>(&camera_);
  const auto& cpu = runtime::cpu_features();
  if (division && division->kappa == 0 &&
      runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                              {true, false, false, false})) {
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    __m128d rotation[9];
    for (int k = 0; k < 9; ++k) rotation[k] = _mm_set1_pd(rotation_[k]);
    const auto cx = _mm_set1_pd(division->cx), cy = _mm_set1_pd(division->cy);
    const auto sx = _mm_set1_pd(division->sx), sy = _mm_set1_pd(division->sy),
               focus = _mm_set1_pd(division->focus);
    const auto ox = _mm_set1_pd(perspective_origin_.x), oy = _mm_set1_pd(perspective_origin_.y);
    const auto minus_z = _mm_set1_pd(-perspective_origin_.z), factor = _mm_set1_pd(factor_);
    for (; input.size() - index >= 2; index += 2) {
      const auto first = _mm_loadu_pd(&input[index].row), second = _mm_loadu_pd(&input[index + 1].row);
      const auto rows = _mm_unpacklo_pd(first, second), cols = _mm_unpackhi_pd(first, second);
      auto valid = _mm_and_pd(finite(rows), finite(cols));
      const auto u = _mm_mul_pd(_mm_sub_pd(cols, cx), sx), v = _mm_mul_pd(_mm_sub_pd(rows, cy), sy);
      valid = _mm_and_pd(valid, _mm_and_pd(_mm_and_pd(finite(u), finite(v)),
                                           finite(_mm_add_pd(_mm_mul_pd(u, u), _mm_mul_pd(v, v)))));
      if (_mm_movemask_pd(valid) != 3) break;
      const auto x = _mm_div_pd(u, focus), y = _mm_div_pd(v, focus);
      const auto rotate = [&](int k) {
        return _mm_add_pd(_mm_add_pd(_mm_mul_pd(rotation[k], x), _mm_mul_pd(rotation[k + 1], y)),
                          rotation[k + 2]);
      };
      const auto dx = rotate(0), dy = rotate(3), dz = rotate(6);
      if (_mm_movemask_pd(_mm_cmpge_pd(_mm_andnot_pd(_mm_set1_pd(-0.), dz), _mm_set1_pd(1e-14))) != 3) break;
      const auto distance = _mm_div_pd(minus_z, dz);
      if (_mm_movemask_pd(finite(distance)) != 3 ||
          (!halcon_contract_ && _mm_movemask_pd(_mm_cmpgt_pd(distance, _mm_setzero_pd())) != 3))
        break;
      const auto result_x = _mm_mul_pd(_mm_add_pd(ox, _mm_mul_pd(distance, dx)), factor);
      const auto result_y = _mm_mul_pd(_mm_add_pd(oy, _mm_mul_pd(distance, dy)), factor);
      _mm_storeu_pd(&output[index].row, _mm_unpacklo_pd(result_y, result_x));
      _mm_storeu_pd(&output[index + 1].row, _mm_unpackhi_pd(result_y, result_x));
    }
  }
#endif
  // 可疑点对按原输入顺序恢复逐点计算，保持首个几何/坐标异常。
  for (; index < input.size(); ++index) {
    const auto p = input[index];
    if (!std::isfinite(p.row) || !std::isfinite(p.col))
      throw core::Exception(core::Status::InvalidArgument, operation, "contour coordinates must be finite");
    const auto point = (*this)({p.row, p.col});
    output[index] = {point.row, point.col};
  }
}

void project_3d_point(const core::Tuple& x, const core::Tuple& y, const core::Tuple& z,
                      const core::Tuple& camera_param, const core::Tuple& object_in_camera_pose,
                      core::Tuple* row, core::Tuple* column) {
  constexpr std::string_view kOperation = "project_3d_point";
  if (row == nullptr || column == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, kOperation, "outputs must not be null");
  }
  const auto camera = parse_camera_parameters(camera_param);
  const auto pose = pose_from_halcon_tuple(object_in_camera_pose);
  const auto plan = core::BroadcastPlan::make(
      {{x.size(), "X", true}, {y.size(), "Y", true}, {z.size(), "Z", true}}, kOperation);
  const auto xs = broadcast_numeric(x, plan, 0, kOperation);
  const auto ys = broadcast_numeric(y, plan, 1, kOperation);
  const auto zs = broadcast_numeric(z, plan, 2, kOperation);

  const auto rotation = rotation_matrix(pose.rotation);
  const auto& translation = pose.translation;
  const auto& common = common_parameters(camera);
  const bool perspective = is_perspective(camera);
  double factor = 0;
  const bool undistorted = std::visit(
      [&](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                      std::is_same_v<T, AreaScanPolynomialParameters>)
          factor = p.focus;
        else
          factor = p.magnification;
        if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                      std::is_same_v<T, AreaScanTelecentricDivisionParameters>)
          return p.kappa == 0;
        else
          return p.k1 == 0 && p.k2 == 0 && p.k3 == 0 && p.p1 == 0 && p.p2 == 0;
      },
      camera);
  std::vector<double> result_row(plan.size()), result_column(plan.size());
  std::size_t index = 0;
#if MVKIT_PROJECT_SSE2
  const auto& cpu = mv::runtime::cpu_features();
  if (undistorted &&
      mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                  {true, false, false, false})) {
    __m128d r[9];
    for (int k = 0; k < 9; ++k) r[k] = _mm_set1_pd(rotation[k]);
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    for (; plan.size() - index >= 2; index += 2) {
      const auto vx = _mm_loadu_pd(xs.data() + index), vy = _mm_loadu_pd(ys.data() + index),
                 vz = _mm_loadu_pd(zs.data() + index);
      const auto transform = [&](int k, double t) {
        return _mm_add_pd(
            _mm_add_pd(_mm_add_pd(_mm_mul_pd(r[k], vx), _mm_mul_pd(r[k + 1], vy)), _mm_mul_pd(r[k + 2], vz)),
            _mm_set1_pd(t));
      };
      const auto tx = transform(0, translation.x), ty = transform(3, translation.y),
                 tz = transform(6, translation.z);
      auto valid = _mm_and_pd(_mm_and_pd(finite(tx), finite(ty)), finite(tz));
      if (perspective) valid = _mm_and_pd(valid, _mm_cmpgt_pd(tz, _mm_set1_pd(1e-12)));
      if (_mm_movemask_pd(valid) != 3) break;
      auto u = _mm_mul_pd(_mm_set1_pd(factor), tx), v = _mm_mul_pd(_mm_set1_pd(factor), ty);
      if (perspective) {
        u = _mm_div_pd(u, tz);
        v = _mm_div_pd(v, tz);
      }
      if (_mm_movemask_pd(_mm_and_pd(finite(u), finite(v))) != 3) break;
      _mm_storeu_pd(result_row.data() + index,
                    _mm_add_pd(_mm_div_pd(v, _mm_set1_pd(common.sy)), _mm_set1_pd(common.cy)));
      _mm_storeu_pd(result_column.data() + index,
                    _mm_add_pd(_mm_div_pd(u, _mm_set1_pd(common.sx)), _mm_set1_pd(common.cx)));
    }
  }
#endif
  for (; index < plan.size(); ++index) {
    const auto x0 = xs[index], y0 = ys[index], z0 = zs[index];
    const Point3d transformed{rotation[0] * x0 + rotation[1] * y0 + rotation[2] * z0 + translation.x,
                              rotation[3] * x0 + rotation[4] * y0 + rotation[5] * z0 + translation.y,
                              rotation[6] * x0 + rotation[7] * y0 + rotation[8] * z0 + translation.z};
    Point2d point;
    if (undistorted && std::isfinite(transformed.x) && std::isfinite(transformed.y) &&
        std::isfinite(transformed.z) && (!perspective || transformed.z > 1e-12)) {
      const auto u = perspective ? factor * transformed.x / transformed.z : factor * transformed.x;
      const auto v = perspective ? factor * transformed.y / transformed.z : factor * transformed.y;
      if (std::isfinite(u) && std::isfinite(v))
        point = {v / common.sy + common.cy, u / common.sx + common.cx};
      else
        point = project_camera_point(camera, transformed);
    } else
      point = project_camera_point(camera, transformed);
    result_row[index] = point.row;
    result_column[index] = point.col;
  }
  *row = core::Tuple{std::move(result_row)};
  *column = core::Tuple{std::move(result_column)};
}

void change_radial_distortion_points(const core::Tuple& row, const core::Tuple& column,
                                     const core::Tuple& camera_param_in, const core::Tuple& camera_param_out,
                                     core::Tuple* row_changed, core::Tuple* column_changed) {
  constexpr std::string_view kOperation = "change_radial_distortion_points";
  if (row_changed == nullptr || column_changed == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, kOperation, "outputs must not be null");
  }
  const auto camera_in = parse_camera_parameters(camera_param_in);
  const auto camera_out = parse_camera_parameters(camera_param_out);
  const auto plan =
      core::BroadcastPlan::make({{row.size(), "Row", true}, {column.size(), "Column", true}}, kOperation);
  const auto rows = broadcast_numeric(row, plan, 0, kOperation);
  const auto columns = broadcast_numeric(column, plan, 1, kOperation);

  std::vector<double> result_row(plan.size()), result_column(plan.size());
  // 既有 XLD 畸变算术直接处理 Tuple 坐标列，避免交错 scratch 缓冲；可疑点对按原序重放。
  std::size_t index = 0;
#if MVKIT_PROJECT_SSE2
  const auto* in = std::get_if<AreaScanDivisionParameters>(&camera_in);
  const auto* out = std::get_if<AreaScanDivisionParameters>(&camera_out);
  const auto& cpu = runtime::cpu_features();
  if (plan.size() >= 256 && in && out &&
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
    for (; plan.size() - index >= 2; index += 2) {
      const auto point_rows = _mm_loadu_pd(rows.data() + index),
                 point_cols = _mm_loadu_pd(columns.data() + index);
      auto valid = _mm_and_pd(finite(point_rows), finite(point_cols));
      const auto u = _mm_mul_pd(_mm_sub_pd(point_cols, cx), sx),
                 v = _mm_mul_pd(_mm_sub_pd(point_rows, cy), sy);
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
      _mm_storeu_pd(result_row.data() + index, result_rows);
      _mm_storeu_pd(result_column.data() + index, result_cols);
    }
  }
#endif
  for (; index < plan.size(); ++index) {
    const auto metric = image_point_to_undistorted_metric(camera_in, {rows[index], columns[index]});
    const auto point = undistorted_metric_to_image_point(camera_out, metric);
    result_row[index] = point.row;
    result_column[index] = point.col;
  }
  *row_changed = result_row.empty() ? core::Tuple{} : core::Tuple{std::move(result_row)};
  *column_changed = result_column.empty() ? core::Tuple{} : core::Tuple{std::move(result_column)};
}

void image_points_to_world_plane(const core::Tuple& camera_param, const core::Tuple& world_in_camera_pose,
                                 const core::Tuple& row, const core::Tuple& column, const core::Tuple& scale,
                                 core::Tuple* x, core::Tuple* y) {
  constexpr std::string_view kOperation = "image_points_to_world_plane";
  if (x == nullptr || y == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, kOperation, "outputs must not be null");
  }
  const auto camera = parse_camera_parameters(camera_param);
  const auto world_in_camera = pose_from_halcon_tuple(world_in_camera_pose);
  const auto camera_in_world = inverse_pose(world_in_camera);
  const double factor = scale_factor(scale);
  const WorldPlaneProjection project(camera, camera_in_world, factor);
  project.map_tuples(row, column, x, y);
}

void WorldPlaneProjection::map_tuples(const core::Tuple& row, const core::Tuple& column, core::Tuple* x,
                                      core::Tuple* y) const {
  constexpr std::string_view kOperation = "image_points_to_world_plane";
  if (halcon_contract_ && row.size() == 0 && column.size() == 0) {
    *x = core::Tuple{};
    *y = core::Tuple{};
    return;
  }
  const auto plan =
      core::BroadcastPlan::make({{row.size(), "Row", true}, {column.size(), "Column", true}}, kOperation);
  const auto rows = broadcast_numeric(row, plan, 0, kOperation, halcon_contract_);
  const auto columns = broadcast_numeric(column, plan, 1, kOperation, halcon_contract_);

  std::vector<double> result_x_values(plan.size()), result_y_values(plan.size());
  // 既有 XLD 投影算术直接处理 Tuple 坐标列，避免交错 scratch 缓冲；可疑点对按原序重放。
  std::size_t index = 0;
#if MVKIT_PROJECT_SSE2
  const auto* division = std::get_if<AreaScanDivisionParameters>(&camera_);
  const auto& cpu = runtime::cpu_features();
  if (plan.size() >= 256 && division && division->kappa == 0 &&
      runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                              {true, false, false, false})) {
    const auto rotation_values = rotation_matrix(camera_in_world_.rotation);
    const auto origin = transform_point(camera_in_world_, {0, 0, 0});
    const auto scale_factor_value = factor_;
    const auto finite = [](__m128d value) {
      return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.), value),
                          _mm_set1_pd(std::numeric_limits<double>::max()));
    };
    __m128d rotation[9];
    for (int k = 0; k < 9; ++k) rotation[k] = _mm_set1_pd(rotation_values[k]);
    const auto cx = _mm_set1_pd(division->cx), cy = _mm_set1_pd(division->cy);
    const auto sx = _mm_set1_pd(division->sx), sy = _mm_set1_pd(division->sy),
               focus = _mm_set1_pd(division->focus);
    const auto ox = _mm_set1_pd(origin.x), oy = _mm_set1_pd(origin.y);
    const auto minus_z = _mm_set1_pd(-origin.z), factor = _mm_set1_pd(scale_factor_value);
    for (; plan.size() - index >= 2; index += 2) {
      const auto point_rows = _mm_loadu_pd(rows.data() + index),
                 point_cols = _mm_loadu_pd(columns.data() + index);
      auto valid = _mm_and_pd(finite(point_rows), finite(point_cols));
      const auto u = _mm_mul_pd(_mm_sub_pd(point_cols, cx), sx),
                 v = _mm_mul_pd(_mm_sub_pd(point_rows, cy), sy);
      valid = _mm_and_pd(valid, _mm_and_pd(_mm_and_pd(finite(u), finite(v)),
                                           finite(_mm_add_pd(_mm_mul_pd(u, u), _mm_mul_pd(v, v)))));
      if (_mm_movemask_pd(valid) != 3) break;
      const auto x = _mm_div_pd(u, focus), y = _mm_div_pd(v, focus);
      const auto rotate = [&](int k) {
        return _mm_add_pd(_mm_add_pd(_mm_mul_pd(rotation[k], x), _mm_mul_pd(rotation[k + 1], y)),
                          rotation[k + 2]);
      };
      const auto dx = rotate(0), dy = rotate(3), dz = rotate(6);
      if (_mm_movemask_pd(_mm_cmpge_pd(_mm_andnot_pd(_mm_set1_pd(-0.), dz), _mm_set1_pd(1e-14))) != 3) break;
      const auto distance = _mm_div_pd(minus_z, dz);
      if (_mm_movemask_pd(finite(distance)) != 3 ||
          (!halcon_contract_ && _mm_movemask_pd(_mm_cmpgt_pd(distance, _mm_setzero_pd())) != 3))
        break;
      const auto result_x = _mm_mul_pd(_mm_add_pd(ox, _mm_mul_pd(distance, dx)), factor);
      const auto result_y = _mm_mul_pd(_mm_add_pd(oy, _mm_mul_pd(distance, dy)), factor);
      _mm_storeu_pd(result_x_values.data() + index, result_x);
      _mm_storeu_pd(result_y_values.data() + index, result_y);
    }
  }
#endif
  for (; index < plan.size(); ++index) {
    if (halcon_contract_ && (!std::isfinite(rows[index]) || !std::isfinite(columns[index]))) {
      result_x_values[index] = result_y_values[index] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    const auto point = (*this)({rows[index], columns[index]});
    result_x_values[index] = point.col;
    result_y_values[index] = point.row;
  }
  if (halcon_contract_)
    for (std::size_t i = 0; i < plan.size(); ++i) {
      result_x_values[i] = static_cast<float>(result_x_values[i]);
      result_y_values[i] = static_cast<float>(result_y_values[i]);
    }
  *x = core::Tuple{std::move(result_x_values)};
  *y = core::Tuple{std::move(result_y_values)};
}

}  // namespace mv::calibration
