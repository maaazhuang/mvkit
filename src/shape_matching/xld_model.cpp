#include <shape_matching/operators.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <core/exception.hpp>
#include <runtime/memory/persist_container.hpp>

namespace mv::shape_matching {
namespace {
constexpr auto operation = "create_shape_model_xld";
constexpr std::size_t point_budget = 65536;
void require(bool ok, const char* parameter) {
  if (!ok) throw core::Exception(core::Status::InvalidArgument,
      core::ErrorCode::InvalidValue, operation, parameter, -1, "invalid XLD model input");
}
double number(const core::Tuple& t) {
  require(t.size() == 1, "parameter");
  if (t.kind() == core::TupleKind::Int64) return static_cast<double>(t.as_int64());
  require(t.kind() == core::TupleKind::Real64, "parameter");
  return t.as_real64();
}
double automatic_number(const core::Tuple& t) {
  if (t.size() == 1 && t.kind() == core::TupleKind::String && t.string_view() == "auto") return 0;
  return number(t);
}
XldCreateParameters parameters(const core::Tuple& levels, double start, double extent,
    const core::Tuple& step, double scale_min, double scale_max, const core::Tuple* scale_step,
    const core::Tuple& optimization, std::string_view metric, double min_contrast) {
  XldCreateParameters p;
  const double n = automatic_number(levels);
  require(std::isfinite(n) && n >= 0 && n <= 30 && n == std::floor(n), "num_levels");
  p.levels = static_cast<int>(n);
  p.angle_start = start; p.angle_extent = extent; p.angle_step = automatic_number(step);
  p.scale_min = scale_min; p.scale_max = scale_max;
  if (scale_step) p.scale_step = automatic_number(*scale_step);
  p.min_contrast = min_contrast;
  if (metric == "use_polarity") p.metric = 0;
  else if (metric == "ignore_global_polarity") p.metric = 1;
  else { require(metric == "ignore_local_polarity", "metric"); p.metric = 2; }
  require(!optimization.empty(), "optimization");
  bool have_reduction = false;
  p.reduction = 0;
  for (std::size_t i = 0; i < optimization.size(); ++i) {
    const auto v = optimization.at(i);
    require(v.kind() == core::ValueKind::String, "optimization");
    const auto s = v.as_string().view();
    // 共享搜索缓存中的旋转准备仍延迟执行。
    require(s != "pregeneration", "optimization");
    if (s == "no_pregeneration") continue;
    require(!have_reduction, "optimization");
    have_reduction = true;
    if (s == "none") p.reduction = 0;
    else if (s == "auto") p.reduction = 1;
    else if (s == "point_reduction_low") p.reduction = 2;
    else if (s == "point_reduction_medium") p.reduction = 3;
    else { require(s == "point_reduction_high", "optimization"); p.reduction = 4; }
  }
  return p;
}
std::vector<ShapeModelPoint> coverage(std::span<const ShapeModelPoint> input, std::size_t cap) {
  const auto n = std::min(input.size(), cap);
  std::vector<ShapeModelPoint> out;
  out.reserve(n);
  // greedy 前缀须覆盖完整对象而非首条边；互素排列保留完全相同的均匀选点。
  std::size_t stride = std::max<std::size_t>(1, static_cast<std::size_t>(n * .6180339887498949));
  while (n && std::gcd(stride, n) != 1) ++stride;
  for (std::size_t i = 0; i < n; ++i) out.push_back(input[((i * stride) % n) * input.size() / n]);
  return out;
}
core::XldContours relative_contours(const core::XldContours& input, double row, double col, double div) {
  core::XldContours out;
  for (std::size_t i = 0; i < input.contour_count(); ++i) {
    const auto& c = input.contour(i);
    std::vector<core::Point2d> points;
    points.reserve(c.size());
    for (const auto& p : c.points()) points.push_back({(p.row - row) / div, (p.col - col) / div});
    out.append(core::XldContour::from_points_and_attributes(points, c.closed(), c.approximation(),
        c.point_attribute(core::XldPointAttribute::Response),
        c.point_attribute(core::XldPointAttribute::EdgeDirection),
        c.point_attribute(core::XldPointAttribute::NormalAngle)));
  }
  return out;
}
}

ShapeModel create_model_xld(const core::XldContours& input, const XldCreateParameters& p) {
  require(p.levels >= 0 && p.levels <= 30, "levels");
  require(p.metric >= 0 && p.metric <= 2, "metric");
  require(p.reduction >= 0 && p.reduction <= 4, "reduction");
  require(std::isfinite(p.angle_start) && std::isfinite(p.angle_extent) &&
      p.angle_extent >= 0 && p.angle_extent <= 2 * std::numbers::pi, "angle_range");
  require(std::isfinite(p.angle_step) && p.angle_step >= 0, "angle_step");
  require(std::isfinite(p.scale_min) && std::isfinite(p.scale_max) &&
      p.scale_min > 0 && p.scale_max >= p.scale_min, "scale_range");
  require(std::isfinite(p.scale_step) && p.scale_step >= 0, "scale_step");
  require(std::isfinite(p.min_contrast) && p.min_contrast >= 0, "min_contrast");
  require(!input.empty() && input.contour_count() <= 4096, "contours");
  double rmin = std::numeric_limits<double>::infinity(), cmin = rmin;
  double rmax = -rmin, cmax = -rmin;
  std::size_t input_count = 0;
  double arc_length = 0;
  for (std::size_t i = 0; i < input.contour_count(); ++i) {
    const auto& c = input.contour(i);
    require(c.size() >= 2 && c.size() <= point_budget - input_count, "points");
    input_count += c.size();
    const auto points = c.points();
    for (const auto& q : points) {
      require(std::isfinite(q.row) && std::isfinite(q.col), "coordinates");
      rmin = std::min(rmin, q.row); rmax = std::max(rmax, q.row);
      cmin = std::min(cmin, q.col); cmax = std::max(cmax, q.col);
    }
    for (auto attr : {core::XldPointAttribute::Response, core::XldPointAttribute::EdgeDirection,
                      core::XldPointAttribute::NormalAngle}) {
      for (double value : c.point_attribute(attr)) require(std::isfinite(value), "attributes");
    }
    if (p.metric != 2) require(c.has_point_attribute(core::XldPointAttribute::EdgeDirection), "edge_direction");
    double length = 0;
    for (std::size_t j = 0; j < points.size() - (c.closed() ? 0 : 1); ++j) {
      const auto& a = points[j]; const auto& b = points[(j + 1) % points.size()];
      length += std::hypot(b.row - a.row, b.col - a.col);
    }
    require(std::isfinite(length) && length > 0, "geometry");
    arc_length += length;
  }
  require(std::isfinite(arc_length) && arc_length <= point_budget, "sampling_budget");
  const double height = rmax - rmin, width = cmax - cmin;
  require(std::isfinite(height) && std::isfinite(width), "bounds");
  // 独立短轮廓仍可能跨巨大坐标范围，离散计划将偏移窄化到 int16 前
  // 须限制缩放跨度，弧长预算不足以保证安全。
  require(std::hypot(height, width) * .5 * p.scale_max <= 32760, "scaled_bounds");
  ShapeModel model;
  model.xld_model_ = true;
  model.search_cache_ = make_shape_model_search_cache();
  model.kind_ = p.scale_min == 1 && p.scale_max == 1 ? ShapeModelKind::Rigid : ShapeModelKind::IsotropicScale;
  model.metric_ = static_cast<ShapeMetric>(p.metric);
  const char* metrics[]{"use_polarity", "ignore_global_polarity", "ignore_local_polarity"};
  model.metric_name_ = metrics[p.metric];
  model.default_origin_row_ = model.origin_row_ = std::midpoint(rmin, rmax);
  model.default_origin_column_ = model.origin_column_ = std::midpoint(cmin, cmax);
  model.angle_start_ = p.angle_start; model.angle_extent_ = p.angle_extent;
  model.scale_min_ = p.scale_min; model.scale_max_ = p.scale_max;
  model.min_contrast_ = p.min_contrast;
  int requested = p.levels;
  if (!requested) {
    requested = 1;
    double side = std::max(height, width);
    while (requested < 6 && side / 2 >= 6 && arc_length / std::ldexp(1., requested) >= 16) {
      ++requested; side *= .5;
    }
  }
  for (int level = 0; level < requested; ++level) {
    ShapeModelLevel next;
    next.level = level; next.pyramid_scale = std::ldexp(1., level);
    std::vector<ShapeModelPoint> samples;
    for (std::size_t i = 0; i < input.contour_count(); ++i) {
      const auto& c = input.contour(i);
      const auto points = c.points();
      const auto directions = c.point_attribute(core::XldPointAttribute::EdgeDirection);
      double distance_to_sample = .5 * next.pyramid_scale;
      for (std::size_t j = 0; j < points.size() - (c.closed() ? 0 : 1); ++j) {
        const auto k = (j + 1) % points.size();
        const auto& a = points[j]; const auto& b = points[k];
        const double dr = b.row - a.row, dc = b.col - a.col;
        const double length = std::hypot(dr, dc);
        if (length <= 1e-12) continue;
        for (; distance_to_sample < length; distance_to_sample += next.pyramid_scale) {
          require(samples.size() < point_budget, "sampling_budget");
          const double t = distance_to_sample / length;
          double nr = dc / length, nc = -dr / length;
          if (p.metric != 2) {
            nr = -(1 - t) * std::sin(directions[j]) - t * std::sin(directions[k]);
            nc = (1 - t) * std::cos(directions[j]) + t * std::cos(directions[k]);
            const double norm = std::hypot(nr, nc);
            require(norm > 1e-12, "edge_direction");
            nr /= norm; nc /= norm;
          }
          samples.push_back({(a.row - model.origin_row_ + t * dr) / next.pyramid_scale,
              (a.col - model.origin_column_ + t * dc) / next.pyramid_scale, nr, nc, 1});
        }
        distance_to_sample -= length;
      }
    }
    if (samples.size() < 4) break;
    if (level == 0) mv::runtime::assign_persist(model.refinement_points_, coverage(samples,
        p.reduction == 0 ? 1200 : std::max<std::size_t>(16, std::min<std::size_t>(1200, samples.size()/2))));
    const std::size_t cap = p.reduction == 0 ? point_budget : p.reduction == 1 ? 64 :
        p.reduction == 4 ? 160 : p.reduction == 3 ? 240 : 400;
    mv::runtime::assign_persist(next.points, coverage(samples, cap));
    // 包含全部采样几何，包括独立细化点。
    for (const auto& q : samples) next.radius = std::max(next.radius, std::hypot(q.row, q.column));
    next.contours = relative_contours(input, model.origin_row_, model.origin_column_, next.pyramid_scale);
    model.levels_.push_back(std::move(next));
  }
  require(!model.levels_.empty(), "model_points");
  const auto radius = model.levels_.front().radius;
  model.angle_step_ = p.angle_step ? p.angle_step : std::clamp(1. / std::max(1., radius), .002, .2);
  model.scale_step_ = p.scale_step ? p.scale_step : std::clamp(1. / std::max(1., radius), .005, .1);
  return model;
}

ShapeModel create_shape_model_xld(const core::XldContours& c, const core::Tuple& l,
    double a, double e, const core::Tuple& s, const core::Tuple& o, std::string m, double contrast) {
  return create_model_xld(c, parameters(l, a, e, s, 1, 1, nullptr, o, m, contrast));
}
ShapeModel create_scaled_shape_model_xld(const core::XldContours& c, const core::Tuple& l,
    double a, double e, const core::Tuple& s, double lo, double hi, const core::Tuple& ss,
    const core::Tuple& o, std::string m, double contrast) {
  return create_model_xld(c, parameters(l, a, e, s, lo, hi, &ss, o, m, contrast));
}
}  // namespace mv::shape_matching
