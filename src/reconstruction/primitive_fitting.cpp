#include "primitive_fitting.hpp"

#include <algorithm>
#include <calibration/hom_mat3d.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <reconstruction/mesh_index.hpp>
#include <runtime/execution/execution_context.hpp>

namespace mv::reconstruction {
namespace {
Point3d add(Point3d a, Point3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
using mesh_detail::cross;
using mesh_detail::dot;
using mesh_detail::sub;
Point3d multiply(Point3d p, double s) { return {p.x * s, p.y * s, p.z * s}; }
double length(Point3d p) { return std::hypot(p.x, p.y, p.z); }
Point3d unit(Point3d p) {
  const auto n = length(p);
  return n > 0 ? multiply(p, 1 / n) : Point3d{};
}
const auto& backend() {
  return calibration::numeric::shared_backend(calibration::numeric::default_backend_kind());
}
void basis(Point3d axis, Point3d& u, Point3d& v) {
  u = unit(cross(axis, std::abs(axis.z) < .8 ? Point3d{0, 0, 1} : Point3d{0, 1, 0}));
  v = cross(axis, u);
}
void pose_basis(Point3d normal, Point3d& u, Point3d& v) {
  const double a = std::atan2(-normal.y, normal.z);
  const double b = std::atan2(normal.x, std::hypot(normal.y, normal.z));
  u = {std::cos(b), std::sin(a) * std::sin(b), -std::cos(a) * std::sin(b)};
  v = {0, std::cos(a), std::sin(a)};
}
struct Surface {
  Point3d center{}, axis{0, 0, 1};
  double radius{};
  std::size_t iterations{};
};
double residual(Point3d p, const Surface& s, bool cylinder) {
  p = sub(p, s.center);
  if (cylinder) p = sub(p, multiply(s.axis, dot(p, s.axis)));
  return length(p) - s.radius;
}
bool eigen(std::span<const Point3d> points, std::vector<double>& values, std::vector<double>& vectors) {
  std::vector<double> covariance(9, 0);
  Point3d mean{};
  for (auto p : points) mean = add(mean, p);
  mean = multiply(mean, 1. / points.size());
  for (auto p : points) {
    p = sub(p, mean);
    const double q[]{p.x, p.y, p.z};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) covariance[i * 3 + j] += q[i] * q[j];
  }
  return backend().symmetric_eigen(covariance, 3, &values, &vectors);
}
bool seed(std::span<const Point3d> points, bool cylinder, Point3d axis, Surface& s) {
  const std::size_t n = cylinder ? 3 : 4;
  std::vector<double> matrix(n * n, 0), rhs(n, 0), solution;
  Point3d u{}, v{};
  if (cylinder) basis(axis, u, v);
  for (auto p : points) {
    const double x = cylinder ? dot(p, u) : p.x, y = cylinder ? dot(p, v) : p.y;
    const double j[]{2 * x, 2 * y, cylinder ? 1 : 2 * p.z, 1};
    const double q = x * x + y * y + (cylinder ? 0 : p.z * p.z);
    for (std::size_t a = 0; a < n; ++a) {
      rhs[a] += j[a] * q;
      for (std::size_t b = 0; b < n; ++b) matrix[a * n + b] += j[a] * j[b];
    }
  }
  std::vector<double> ev, evec;
  if (!backend().symmetric_eigen(matrix, n, &ev, &evec) || ev.front() <= ev.back() * 1e-12 ||
      !backend().solve_symmetric(matrix, rhs, n, &solution))
    return false;
  s.center = cylinder ? add(multiply(u, solution[0]), multiply(v, solution[1]))
                      : Point3d{solution[0], solution[1], solution[2]};
  const double r2 = dot(s.center, s.center) + solution.back();
  if (!(r2 > 0) || !std::isfinite(r2)) return false;
  s.radius = std::sqrt(r2);
  s.axis = axis;
  return true;
}
double squared_error(std::span<const Point3d> points, const Surface& s, bool cylinder,
                     std::span<const double> weights = {}) {
  double sum = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto r = residual(points[i], s, cylinder);
    sum += r * r * (weights.empty() ? 1 : weights[i]);
  }
  return sum;
}
bool refine(std::span<const Point3d> points, bool cylinder, std::span<const double> weights, Surface& s) {
  const std::size_t n = cylinder ? 5 : 4;
  double lambda = 1e-5;
  for (std::size_t iteration = 0; iteration < 60; ++iteration) {
    Point3d u{}, v{};
    if (cylinder) basis(s.axis, u, v);
    std::vector<double> matrix(n * n, 0), rhs(n, 0), step;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto q = sub(points[i], s.center);
      const double t = dot(q, s.axis);
      const auto radial = cylinder ? sub(q, multiply(s.axis, t)) : q;
      const double norm = length(radial);
      if (norm < 1e-14) continue;
      const auto g = multiply(radial, 1 / norm);
      const double r = norm - s.radius;
      const double j[]{cylinder ? -dot(g, u) : -g.x, cylinder ? -dot(g, v) : -g.y,
                       cylinder ? -t * dot(g, u) : -g.z, cylinder ? -t * dot(g, v) : -1., -1.};
      const double weight = weights.empty() ? 1 : weights[i];
      for (std::size_t a = 0; a < n; ++a) {
        rhs[a] -= weight * j[a] * r;
        for (std::size_t b = 0; b < n; ++b) matrix[a * n + b] += weight * j[a] * j[b];
      }
    }
    ++s.iterations;
    double gradient = 0;
    for (auto r : rhs) gradient = std::max(gradient, std::abs(r));
    if (gradient < 1e-13 * points.size()) return true;
    const double old = squared_error(points, s, cylinder, weights);
    bool accepted = false;
    for (int attempt = 0; attempt < 12; ++attempt) {
      auto damped = matrix;
      for (std::size_t a = 0; a < n; ++a) damped[a * n + a] += lambda * std::max(matrix[a * n + a], 1e-8);
      if (!backend().solve_symmetric(damped, rhs, n, &step)) return false;
      auto next = s;
      if (cylinder) {
        next.center = add(s.center, add(multiply(u, step[0]), multiply(v, step[1])));
        next.axis = unit(add(s.axis, add(multiply(u, step[2]), multiply(v, step[3]))));
        next.center = sub(next.center, multiply(next.axis, dot(next.center, next.axis)));
        next.radius += step[4];
      } else {
        next.center = add(s.center, {step[0], step[1], step[2]});
        next.radius += step[3];
      }
      const double newer = next.radius > 0 ? squared_error(points, next, cylinder, weights) : INFINITY;
      if (std::isfinite(newer) && newer <= old) {
        s = next;
        accepted = true;
        lambda = std::max(1e-12, lambda * .2);
        double delta = 0;
        for (auto value : step) delta += value * value;
        if (delta < 1e-22 || old - newer < 1e-16 * std::max(1., old)) return true;
        break;
      }
      lambda *= 10;
    }
    if (!accepted) return std::isfinite(old);
  }
  return std::isfinite(s.radius);
}
double median(std::vector<double>& values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2) return *middle;
  return .5 * (*middle + *std::max_element(values.begin(), middle));
}
bool robust_refine(std::span<const Point3d> points, bool cylinder, PrimitiveFitLoss loss, Surface& s) {
  if (loss == PrimitiveFitLoss::LeastSquares) return true;
  std::vector<double> weights(points.size()), errors(points.size()), work(points.size());
  for (int iteration = 0; iteration < 20; ++iteration) {
    for (std::size_t i = 0; i < points.size(); ++i) errors[i] = residual(points[i], s, cylinder);
    work = errors;
    const auto center = median(work);
    for (std::size_t i = 0; i < points.size(); ++i) work[i] = std::abs(errors[i] - center);
    const double scale = 1.482602218505602 * median(work);
    // 精确表面无需稳健更新，避免放大舍入误差。
    if (scale < 1e-12) break;
    const double cutoff = scale * (loss == PrimitiveFitLoss::Huber ? 1.345 : 4.685);
    std::size_t retained = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double t = std::abs(errors[i]) / cutoff;
      weights[i] =
          loss == PrimitiveFitLoss::Huber ? (t <= 1 ? 1 : 1 / t) : (t >= 1 ? 0 : (1 - t * t) * (1 - t * t));
      retained += weights[i] > 0;
    }
    if (retained < (cylinder ? 5u : 4u)) return false;
    const auto previous = s;
    if (!refine(points, cylinder, weights, s)) return false;
    if (length(sub(s.center, previous.center)) + std::abs(s.radius - previous.radius) +
            length(sub(s.axis, previous.axis)) <
        1e-11)
      break;
  }
  return true;
}
bool observable(std::span<const Point3d> points, const Surface& s, bool cylinder) {
  const std::size_t n = cylinder ? 5 : 4;
  Point3d u{}, v{};
  if (cylinder) basis(s.axis, u, v);
  std::vector<double> matrix(n * n, 0), ev, evec;
  for (auto p : points) {
    const auto q = sub(p, s.center);
    const double t = dot(q, s.axis);
    const auto g = unit(cylinder ? sub(q, multiply(s.axis, t)) : q);
    const double j[]{cylinder ? -dot(g, u) : -g.x, cylinder ? -dot(g, v) : -g.y,
                     cylinder ? -t * dot(g, u) : -g.z, cylinder ? -t * dot(g, v) : -1., -1.};
    for (std::size_t a = 0; a < n; ++a)
      for (std::size_t b = 0; b < n; ++b) matrix[a * n + b] += j[a] * j[b];
  }
  return backend().symmetric_eigen(matrix, n, &ev, &evec) && ev.front() > ev.back() * 1e-12;
}
void boundary(std::span<const Point3d> points, PrimitiveFitResult& fit) {
  struct XY {
    double x, y;
  };
  std::vector<XY> projected;
  projected.reserve(points.size());
  for (auto p : points) {
    p = sub(p, fit.origin);
    // 匹配已安装 HALCON 支撑多边形实测 mm/Float 分辨率；
    // 此规则由输出坐标推断，仅用于 hull。
    const auto support_coordinate = [](double coordinate) {
      if (std::abs(coordinate) > std::numeric_limits<float>::max() / 1000.) return coordinate;
      const double rounded = static_cast<double>(static_cast<float>(coordinate * 1000)) * .001;
      return rounded == 0 && coordinate != 0 ? coordinate : rounded;
    };
    projected.push_back({support_coordinate(dot(p, fit.axis_x)), support_coordinate(dot(p, fit.axis_y))});
  }
  std::sort(projected.begin(), projected.end(),
            [](auto a, auto b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  projected.erase(std::unique(projected.begin(), projected.end(),
                              [](auto a, auto b) { return a.x == b.x && a.y == b.y; }),
                  projected.end());
  std::vector<XY> hull;
  const auto turn = [](XY a, XY b, XY c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
  for (auto p : projected) {
    while (hull.size() >= 2 && turn(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
    hull.push_back(p);
  }
  const auto lower = hull.size();
  for (auto i = projected.rbegin() + 1; i != projected.rend(); ++i) {
    while (hull.size() > lower && turn(hull[hull.size() - 2], hull.back(), *i) <= 0) hull.pop_back();
    hull.push_back(*i);
  }
  if (hull.size() > 1) hull.pop_back();
  for (auto p : hull)
    fit.plane_boundary.push_back(add(fit.origin, add(multiply(fit.axis_x, p.x), multiply(fit.axis_y, p.y))));
}
}  // namespace

PrimitiveFitResult fit_primitive_geometry(const ObjectModel3DData& model,
                                          const PrimitiveFitOptions& options) {
  validate_object_model_3d(model);
  if (!options.types || (options.types & ~7u) ||
      ((options.types & 6u) &&
       (options.maximum_radius <= 0 || options.minimum_radius > options.maximum_radius)))
    throw core::Exception(core::Status::InvalidArgument, "fit_primitives_object_model_3d",
                          "invalid fit options");
  runtime::ExecutionContext context;
  std::pmr::vector<Point3d> points(context.scratch_resource());
  Point3d origin{};
  for (std::size_t i = 0; i < model.points.size(); ++i)
    if (mesh_detail::valid(model, i)) {
      points.push_back(model.points[i]);
      origin = add(origin, model.points[i]);
    }
  PrimitiveFitResult best;
  if (points.empty()) return best;
  origin = multiply(origin, 1. / points.size());
  double scale = 0;
  for (auto p : points) scale = std::max(scale, length(sub(p, origin)));
  if (!(scale > 0) || !std::isfinite(scale)) return best;
  for (auto& p : points) p = multiply(sub(p, origin), 1 / scale);
  std::vector<double> ev, vec;
  if (!eigen(points, ev, vec)) return best;
  double best_error = INFINITY;
  if ((options.types & 1) && points.size() >= 3 && ev[1] > ev[2] * 1e-12) {
    Point3d normal = unit({vec[0], vec[3], vec[6]});
    if (dot(normal, origin) > 0) normal = multiply(normal, -1);
    double squared = 0;
    for (auto p : points) {
      const auto r = dot(p, normal);
      squared += r * r;
    }
    best.kind = PrimitiveKind::Plane;
    best.parameters = {normal.x, normal.y, normal.z, dot(normal, origin), 0, 0, 0};
    best.origin = origin;
    best.axis_z = normal;
    pose_basis(normal, best.axis_x, best.axis_y);
    best.rms = scale * std::sqrt(squared / points.size());
    best_error = squared;
    std::pmr::vector<Point3d> world(context.scratch_resource());
    world.reserve(points.size());
    for (auto p : points) world.push_back(add(origin, multiply(p, scale)));
    boundary(world, best);
  }
  for (bool cylinder : {false, true}) {
    if (!(options.types & (cylinder ? 4u : 2u)) || points.size() < (cylinder ? 5u : 4u)) continue;
    Surface selected;
    double error = INFINITY;
    std::vector<Surface> starts;
    for (int axis_index = 0; axis_index < (cylinder ? 3 : 1); ++axis_index) {
      const Point3d axis =
          cylinder ? Point3d{vec[axis_index], vec[3 + axis_index], vec[6 + axis_index]} : Point3d{0, 0, 1};
      Surface candidate;
      if (seed(points, cylinder, axis, candidate)) starts.push_back(candidate);
    }
    // 优先最可能种子，精确表面无需其他局部拟合。
    std::stable_sort(starts.begin(), starts.end(), [&](const auto& a, const auto& b) {
      return squared_error(points, a, cylinder) < squared_error(points, b, cylinder);
    });
    for (auto candidate : starts) {
      if (!refine(points, cylinder, {}, candidate) || !observable(points, candidate, cylinder) ||
          !robust_refine(points, cylinder, options.loss, candidate))
        continue;
      const double rss = squared_error(points, candidate, cylinder);
      if (rss < error) {
        error = rss;
        selected = candidate;
      }
      if (error < 1e-24 * points.size()) break;
    }
    const double radius = selected.radius * scale;
    if (error >= best_error || radius < options.minimum_radius || radius > options.maximum_radius) continue;
    PrimitiveFitResult candidate;
    candidate.kind = cylinder ? PrimitiveKind::Cylinder : PrimitiveKind::Sphere;
    auto center = add(origin, multiply(selected.center, scale));
    auto axis = selected.axis;
    if (cylinder) {
      // HALCON 沿拟合轴将锚点设在点云质心。
      center = add(center, multiply(axis, dot(sub(origin, center), axis)));
      if (dot(axis, center) > 0) axis = multiply(axis, -1);
      double lo = INFINITY, hi = -INFINITY;
      for (auto p : points) {
        const auto axial = scale * dot(p, axis);
        lo = std::min(lo, axial);
        hi = std::max(hi, axial);
      }
      candidate.cylinder_extent = {lo, hi};
      candidate.parameters = {center.x, center.y, center.z, axis.x, axis.y, axis.z, radius};
      candidate.axis_z = axis;
      pose_basis(axis, candidate.axis_x, candidate.axis_y);
    } else
      candidate.parameters = {center.x, center.y, center.z, radius, 0, 0, 0};
    candidate.origin = center;
    candidate.rms = scale * std::sqrt(error / points.size());
    candidate.iterations = selected.iterations;
    best = std::move(candidate);
    best_error = error;
  }
  return best;
}

namespace {
double supported_distance(Point3d p, PrimitiveKind kind, const std::array<double, 7>& a,
                          const PrimitiveSupport* support) {
  if (kind == PrimitiveKind::Sphere) return length(sub(p, {a[0], a[1], a[2]})) - a[3];
  if (kind == PrimitiveKind::Cylinder) {
    const auto q = sub(p, {a[0], a[1], a[2]});
    const Point3d axis{a[3], a[4], a[5]};
    const double axial = dot(q, axis), radial = length(sub(q, multiply(axis, axial))) - a[6];
    const double beyond =
        support ? std::max({support->cylinder_extent[0] - axial, axial - support->cylinder_extent[1], 0.})
                : 0;
    return beyond > 0 ? std::hypot(radial, beyond) : radial;
  }
  if (kind == PrimitiveKind::Plane) {
    const Point3d normal{a[0], a[1], a[2]};
    const double signed_distance = dot(p, normal) - a[3];
    if (!support || support->plane_boundary.empty()) return signed_distance;
    const auto projected = sub(p, multiply(normal, signed_distance));
    bool inside = true;
    double minimum = INFINITY;
    for (std::size_t i = 0; i < support->plane_boundary.size(); ++i) {
      const auto start = support->plane_boundary[i],
                 end = support->plane_boundary[(i + 1) % support->plane_boundary.size()];
      const auto edge = sub(end, start), delta = sub(projected, start);
      if (dot(cross(edge, delta), normal) < -1e-14) inside = false;
      const double denominator = dot(edge, edge);
      const double t = denominator > 0 ? std::clamp(dot(edge, delta) / denominator, 0., 1.) : 0;
      minimum = std::min(minimum, length(sub(p, add(start, multiply(edge, t)))));
    }
    return inside ? signed_distance : std::copysign(minimum, signed_distance);
  }
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace
double fitted_primitive_distance(Point3d p, const PrimitiveFitResult& fit) {
  return supported_distance(p, fit.kind, fit.parameters, &fit);
}
double model_primitive_distance(Point3d p, const ObjectModel3DData& model) {
  auto parameters = model.primitive;
  if (model.primitive_kind == PrimitiveKind::Plane)
    parameters = {model.plane[0], model.plane[1], model.plane[2], -model.plane[3], 0, 0, 0};
  return supported_distance(p, model.primitive_kind, parameters, model.primitive_support.get());
}
core::Tuple primitive_frame_pose(const ObjectModel3DData& model) {
  if (!model.primitive_support)
    throw core::Exception(core::Status::Unsupported, "get_object_model_3d_params",
                          "primitive pose metadata unavailable");
  const auto& s = *model.primitive_support;
  return calibration::hom_mat3d_to_pose(core::Tuple(
      std::vector<double>{s.axis_x.x, s.axis_y.x, s.axis_z.x, s.origin.x, s.axis_x.y, s.axis_y.y, s.axis_z.y,
                          s.origin.y, s.axis_x.z, s.axis_y.z, s.axis_z.z, s.origin.z}));
}
ObjectModel3DData fit_primitive_model(const ObjectModel3DData& input, const PrimitiveFitOptions& options,
                                      bool copy_points, bool copy_mapping) {
  const auto fit = fit_primitive_geometry(input, options);
  ObjectModel3DData output;
  if (copy_points) {
    if (copy_mapping) {
      output.mapping_width = input.mapping_width ? input.mapping_width : input.width;
      output.mapping_height = input.mapping_height ? input.mapping_height : input.height;
    }
    output.points.reserve(input.points.size());
    for (std::size_t i = 0; i < input.points.size(); ++i)
      if (mesh_detail::valid(input, i)) {
        output.points.push_back(input.points[i]);
        if (output.mapping_width)
          output.source_indices.push_back(input.source_indices.empty() ? i : input.source_indices[i]);
      }
  }
  output.primitive_kind = fit.kind;
  if (fit.kind != PrimitiveKind::None) {
    if (fit.kind == PrimitiveKind::Plane)
      output.plane = {fit.parameters[0], fit.parameters[1], fit.parameters[2], -fit.parameters[3]};
    else
      output.primitive = fit.parameters;
    output.primitive_support = std::allocate_shared<PrimitiveSupport>(
        std::pmr::polymorphic_allocator<PrimitiveSupport>{runtime::persist_resource()},
        static_cast<const PrimitiveSupport&>(fit));
    output.primitive_rms = fit.rms;
    output.fit_iterations = fit.iterations;
    output.fit_converged = true;
  }
  validate_object_model_3d(output);
  return output;
}
}  // namespace mv::reconstruction
