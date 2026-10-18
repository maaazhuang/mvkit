#include <algorithm>
#include <calibration/numeric/linear_backend.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <numeric>
#include <reconstruction/mesh_index.hpp>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/primitive_fitting.hpp>
#include <runtime/execution/execution_context.hpp>
#include <unordered_set>

namespace mv::reconstruction {
namespace {
constexpr auto fit_op = "fit_primitives_object_model_3d";
[[noreturn]] void bad(const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, fit_op, parameter, -1,
                        message);
}
[[noreturn]] void degenerate(const char* message) {
  throw core::Exception(core::Status::InvalidState, fit_op, message);
}
Point3d add(Point3d a, Point3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Point3d sub(Point3d a, Point3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point3d mul(Point3d a, double b) { return {a.x * b, a.y * b, a.z * b}; }
double dot(Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Point3d cross(Point3d a, Point3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(Point3d a) { return std::sqrt(dot(a, a)); }
Point3d unit(Point3d a) {
  const double n = norm(a);
  return n > 0 ? mul(a, 1 / n) : Point3d{};
}
void basis(Point3d axis, Point3d& u, Point3d& v) {
  u = unit(cross(axis, std::abs(axis.z) < .8 ? Point3d{0, 0, 1} : Point3d{0, 1, 0}));
  v = cross(axis, u);
}
bool valid(const ObjectModel3DData& m, size_t i) { return m.valid.empty() || m.valid[i]; }
const auto& backend() {
  return calibration::numeric::shared_backend(calibration::numeric::default_backend_kind());
}
bool solve(const std::vector<double>& h, const std::vector<double>& b, std::vector<double>& x) {
  return backend().solve_symmetric(h, b, b.size(), &x);
}
template <size_t N>
void accumulate(std::vector<double>& h, std::vector<double>& b, const double* j, double r, double w) {
  for (size_t a = 0; a < N; ++a) {
    b[a] -= w * j[a] * r;
    for (size_t c = a; c < N; ++c) h[a * N + c] += w * j[a] * j[c];
  }
}
double information(const std::vector<double>& h, size_t n) {
  std::vector<double> values, vectors;
  if (!backend().symmetric_eigen(h, n, &values, &vectors) || !(values.back() > 0)) return 0;
  return std::max(0., values.front() / values.back());
}
struct Options {
  size_t iterations = 100, refine = 40;
  std::uint64_t seed = 1;
  double ratio = 0;
  bool irls = false;
};
Options options(const core::Tuple& names, const core::Tuple& values, bool robust) {
  if (names.size() != values.size()) bad("GenParamValue", "generic parameter lengths must match");
  Options o;
  o.irls = robust;
  std::unordered_set<std::string> seen;
  for (size_t i = 0; i < names.size(); ++i) {
    if (names.at(i).kind() != core::ValueKind::String) bad("GenParamName", "parameter names must be strings");
    const auto key = std::string(names.string_view(i));
    if (!seen.insert(key).second) bad("GenParamName", "duplicate parameter");
    const auto& value = values.at(i);
    if (key == "seed" || key == "iterations" || key == "refine_iterations") {
      if (value.kind() != core::ValueKind::Int64) bad(key.c_str(), "integer required");
      const auto n = value.as_int64();
      if (n < (key == "seed" ? 0 : 1) || (key != "seed" && n > 10000))
        bad(key.c_str(), "integer out of range");
      if (key == "seed")
        o.seed = static_cast<uint64_t>(n);
      else if (key == "iterations")
        o.iterations = static_cast<size_t>(n);
      else
        o.refine = static_cast<size_t>(n);
    } else if (key == "min_inlier_ratio") {
      if (value.kind() != core::ValueKind::Int64 && value.kind() != core::ValueKind::Real64)
        bad(key.c_str(), "numeric ratio required");
      o.ratio = value.as_real64();
      if (!std::isfinite(o.ratio) || o.ratio < 0 || o.ratio > 1) bad(key.c_str(), "ratio must be in [0,1]");
    } else if (key == "irls") {
      if (value.kind() == core::ValueKind::Int64) {
        const auto n = value.as_int64();
        if (n != 0 && n != 1) bad("irls", "expected 0 or 1");
        o.irls = n != 0;
      } else if (value.kind() == core::ValueKind::String) {
        const auto text = value.as_string().view();
        if (text != "true" && text != "false") bad("irls", "expected true or false");
        o.irls = text == "true";
      } else
        bad("irls", "expected boolean integer or string");
    } else
      bad("GenParamName", "unknown curved fit parameter");
  }
  return o;
}
struct Surface {
  Point3d center{}, axis{0, 0, 1};
  double radius{};
  size_t iterations{};
  bool converged{};
  double info{};
};
double residual(Point3d p, const Surface& s, bool cylinder) {
  const auto q = sub(p, s.center);
  return norm(cylinder ? sub(q, mul(s.axis, dot(q, s.axis))) : q) - s.radius;
}
bool sphere_seed(std::span<const Point3d> pts, Surface& s) {
  std::vector<double> h(16, 0), b(4, 0), x;
  for (auto p : pts) {
    const double j[4] = {2 * p.x, 2 * p.y, 2 * p.z, 1}, r = dot(p, p);
    accumulate<4>(h, b, j, -r, 1);
  }
  for (size_t a = 0; a < 4; ++a)
    for (size_t c = 0; c < a; ++c) h[a * 4 + c] = h[c * 4 + a];
  if (information(h, 4) < 1e-12 || !solve(h, b, x)) return false;
  s.center = {x[0], x[1], x[2]};
  const double r2 = dot(s.center, s.center) + x[3];
  if (!(r2 > 0) || !std::isfinite(r2)) return false;
  s.radius = std::sqrt(r2);
  return true;
}
bool cylinder_seed(std::span<const Point3d> pts, Point3d axis, Surface& s) {
  Point3d u, v;
  basis(axis, u, v);
  std::vector<double> h(9, 0), b(3, 0), x;
  Point3d mean{};
  for (auto p : pts) mean = add(mean, p);
  mean = mul(mean, 1. / pts.size());
  for (auto p : pts) {
    p = sub(p, mean);
    const double px = dot(p, u), py = dot(p, v), j[3] = {2 * px, 2 * py, 1}, r = px * px + py * py;
    for (int a = 0; a < 3; ++a) {
      b[a] += j[a] * r;
      for (int c = 0; c < 3; ++c) h[a * 3 + c] += j[a] * j[c];
    }
  }
  if (information(h, 3) < 1e-12 || !solve(h, b, x)) return false;
  const double r2 = x[0] * x[0] + x[1] * x[1] + x[2];
  if (!(r2 > 0) || !std::isfinite(r2)) return false;
  s.center = add(mean, add(mul(u, x[0]), mul(v, x[1])));
  s.axis = axis;
  s.center = sub(s.center, mul(axis, dot(s.center, axis)));
  s.radius = std::sqrt(r2);
  return true;
}
std::array<Point3d, 3> axes(std::span<const Point3d> pts) {
  Point3d mean{};
  for (auto p : pts) mean = add(mean, p);
  mean = mul(mean, 1. / pts.size());
  std::vector<double> cov(9, 0), ev, vec;
  for (auto p : pts) {
    p = sub(p, mean);
    const double q[3] = {p.x, p.y, p.z};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) cov[i * 3 + j] += q[i] * q[j];
  }
  if (!backend().symmetric_eigen(cov, 3, &ev, &vec)) return {};
  return {Point3d{vec[0], vec[3], vec[6]}, Point3d{vec[1], vec[4], vec[7]}, Point3d{vec[2], vec[5], vec[8]}};
}
// 点遍历前解析模型类型及损失；各特化保留原 Double 表达式、点序、求解及收敛规则。
template <bool Cylinder, bool Robust>
double cost_t(std::span<const Point3d> pts, const Surface& s, double cutoff) {
  constexpr bool cylinder = Cylinder;
  constexpr bool robust = Robust;
  double sum = 0;
  for (auto p : pts) {
    const double r = std::abs(residual(p, s, cylinder));
    sum += robust && r > cutoff ? 2 * cutoff * r - cutoff * cutoff : r * r;
  }
  return sum;
}
double cost(std::span<const Point3d> pts, const Surface& s, bool cylinder, double cutoff, bool robust) {
  if (cylinder) return robust ? cost_t<true, true>(pts, s, cutoff) : cost_t<true, false>(pts, s, cutoff);
  return robust ? cost_t<false, true>(pts, s, cutoff) : cost_t<false, false>(pts, s, cutoff);
}
template <bool Cylinder>
double geometric_information_t(std::span<const Point3d> pts, const Surface& s) {
  constexpr bool cylinder = Cylinder;
  constexpr size_t n = cylinder ? 5 : 4;
  Point3d u, v;
  if (cylinder) basis(s.axis, u, v);
  std::vector<double> h(n * n, 0);
  for (auto p : pts) {
    const auto q = sub(p, s.center);
    const double t = dot(q, s.axis);
    const auto g = unit(cylinder ? sub(q, mul(s.axis, t)) : q);
    const double j[5] = {cylinder ? -dot(g, u) : -g.x, cylinder ? -dot(g, v) : -g.y,
                         cylinder ? -t * dot(g, u) : -g.z, cylinder ? -t * dot(g, v) : -1, -1};
    for (size_t a = 0; a < n; ++a)
      for (size_t b = 0; b < n; ++b) h[a * n + b] += j[a] * j[b];
  }
  return information(h, n);
}
double geometric_information(std::span<const Point3d> pts, const Surface& s, bool cylinder) {
  return cylinder ? geometric_information_t<true>(pts, s) : geometric_information_t<false>(pts, s);
}
template <bool Cylinder, bool Robust>
bool refine_t(std::span<const Point3d> pts, Surface& s, double cutoff, size_t budget) {
  constexpr bool cylinder = Cylinder;
  constexpr bool robust = Robust;
  constexpr size_t n = cylinder ? 5 : 4;
  s.converged = false;
  double lambda = 1e-5;
  for (size_t it = 0; it < budget; ++it) {
    Point3d u, v;
    if (cylinder) basis(s.axis, u, v);
    std::vector<double> h(n * n, 0), b(n, 0), step;
    for (auto p : pts) {
      const auto q = sub(p, s.center);
      const double axial = dot(q, s.axis);
      const auto radial = cylinder ? sub(q, mul(s.axis, axial)) : q;
      const double len = norm(radial);
      if (len < 1e-14) continue;
      const auto g = mul(radial, 1 / len);
      const double r = len - s.radius, w = robust && std::abs(r) > cutoff ? cutoff / std::abs(r) : 1;
      const double j[5] = {cylinder ? -dot(g, u) : -g.x, cylinder ? -dot(g, v) : -g.y,
                           cylinder ? -axial * dot(g, u) : -g.z, cylinder ? -axial * dot(g, v) : -1, -1};
      if (cylinder)
        accumulate<5>(h, b, j, r, w);
      else
        accumulate<4>(h, b, j, r, w);
    }
    for (size_t a = 0; a < n; ++a)
      for (size_t c = 0; c < a; ++c) h[a * n + c] = h[c * n + a];
    s.iterations = it + 1;
    double maxgrad = 0;
    for (double x : b) maxgrad = std::max(maxgrad, std::abs(x));
    if (maxgrad < 1e-12 * pts.size()) {
      s.converged = true;
      if (!robust) {
        s.info = information(h, n);
        return std::isfinite(s.radius) && s.info >= 1e-10;
      }
      break;
    }
    const double old = cost_t<Cylinder, Robust>(pts, s, cutoff);
    bool accepted = false;
    for (int attempt = 0; attempt < 12; ++attempt) {
      auto damped = h;
      for (size_t a = 0; a < n; ++a) damped[a * n + a] += lambda * std::max(h[a * n + a], 1e-8);
      if (!solve(damped, b, step)) return false;
      Surface next = s;
      if (cylinder) {
        next.center = add(s.center, add(mul(u, step[0]), mul(v, step[1])));
        next.axis = unit(add(s.axis, add(mul(u, step[2]), mul(v, step[3]))));
        next.center = sub(next.center, mul(next.axis, dot(next.center, next.axis)));
        next.radius += step[4];
      } else {
        next.center = add(s.center, {step[0], step[1], step[2]});
        next.radius += step[3];
      }
      const double newer = next.radius > 0 ? cost_t<Cylinder, Robust>(pts, next, cutoff) : INFINITY;
      if (std::isfinite(newer) && newer <= old) {
        s = next;
        lambda = std::max(1e-12, lambda * .2);
        accepted = true;
        double delta = 0;
        for (double x : step) delta += x * x;
        if (delta < 1e-20 || old - newer < 1e-15 * std::max(1., old)) s.converged = true;
        break;
      }
      lambda *= 10;
    }
    if (s.converged) break;
    if (!accepted) return false;
  }
  // 几何信息使用归一化坐标中的无权 Jacobian。
  s.info = geometric_information_t<Cylinder>(pts, s);
  return std::isfinite(s.radius) && s.info >= 1e-10;
}
bool refine(std::span<const Point3d> pts, Surface& s, bool cylinder, double cutoff, bool robust,
            size_t budget) {
  if (cylinder)
    return robust ? refine_t<true, true>(pts, s, cutoff, budget)
                  : refine_t<true, false>(pts, s, cutoff, budget);
  return robust ? refine_t<false, true>(pts, s, cutoff, budget)
                : refine_t<false, false>(pts, s, cutoff, budget);
}

// 闭合三角形的精确最近点，退化三角形归约到其边。
}  // namespace

namespace {
// 评分和最终内点/RMS 遍历也特化，不仅细化；输出压缩、元数据验证及稳健采样不变。
template <bool Cylinder, bool Robust>
ObjectModel3DData fit_curved_primitive_t(const ObjectModel3DData& model, std::string_view type,
                                         double threshold, const core::Tuple& names,
                                         const core::Tuple& values) {
  if (!std::isfinite(threshold) || threshold <= 0) bad("MaxDistance", "must be finite and positive");
  constexpr bool cylinder = Cylinder, robust = Robust;
  const auto o = options(names, values, robust);
  runtime::ExecutionContext context;
  std::pmr::vector<size_t> ids(context.scratch_resource());
  std::pmr::vector<Point3d> points(context.scratch_resource());
  Point3d origin{};
  // 全有效输入已知数量，掩码输入仍稀疏扩容。
  if (model.valid.empty()) ids.reserve(model.points.size());
  for (size_t i = 0; i < model.points.size(); ++i)
    if (valid(model, i)) {
      ids.push_back(i);
      origin = add(origin, model.points[i]);
    }
  if (ids.size() < (cylinder ? 6u : 4u)) degenerate("insufficient valid points");
  origin = mul(origin, 1. / ids.size());
  double scale = 0;
  for (auto i : ids) {
    const auto q = sub(model.points[i], origin);
    scale = std::max(scale, dot(q, q));
  }
  scale = std::sqrt(scale);
  if (!(scale > 0) || !std::isfinite(scale)) degenerate("zero or nonfinite coordinate spread");
  points.reserve(ids.size());
  for (auto i : ids) points.push_back(mul(sub(model.points[i], origin), 1 / scale));
  const double cutoff = threshold / scale;
  Surface best;
  size_t bestcount = 0;
  double bestrss = INFINITY;
  auto score = [&](Surface s) {
    size_t count = 0;
    double rss = 0;
    for (auto p : points) {
      const double r = residual(p, s, cylinder);
      if (std::abs(r) <= cutoff) {
        ++count;
        rss += r * r;
      }
    }
    if (count > bestcount || (count == bestcount && rss < bestrss)) {
      best = s;
      bestcount = count;
      bestrss = rss;
    }
  };
  auto seed = [&](std::span<const Point3d> pts, bool weighted) {
    if (cylinder) {
      std::vector<std::pair<double, Surface>> candidates;
      for (auto axis : axes(pts)) {
        Surface s;
        if (norm(axis) > 0 && cylinder_seed(pts, axis, s))
          candidates.emplace_back(cost(pts, s, true, cutoff, weighted), s);
      }
      std::stable_sort(candidates.begin(), candidates.end(),
                       [](const auto& a, const auto& b) { return a.first < b.first; });
      for (auto& candidate : candidates) {
        auto s = candidate.second;
        if (refine(pts, s, true, cutoff, weighted, o.refine)) score(s);
        if (bestcount == points.size()) break;
      }
    } else {
      Surface s;
      if (sphere_seed(pts, s) && refine(pts, s, false, cutoff, weighted, o.refine)) score(s);
    }
  };
  seed(points, o.irls);
  if (robust) {
    uint64_t state = o.seed;
    const size_t n = cylinder ? 9 : 4;
    std::array<Point3d, 9> sample{};
    std::array<size_t, 9> selected{};
    for (size_t it = 0; it < o.iterations && bestcount < points.size(); ++it) {
      const size_t count = std::min(n, points.size());
      for (size_t j = 0; j < count; ++j) {
        size_t at;
        bool duplicate;
        do {
          state = state * 6364136223846793005ULL + 1442695040888963407ULL;
          at = static_cast<size_t>((state >> 32) % points.size());
          duplicate = false;
          for (size_t k = 0; k < j; ++k) duplicate |= selected[k] == at;
        } while (duplicate);
        selected[j] = at;
        sample[j] = points[at];
      }
      seed(std::span<const Point3d>(sample.data(), count), false);
    }
  }
  const size_t minimum =
      std::max(cylinder ? size_t{6} : size_t{4}, static_cast<size_t>(std::ceil(o.ratio * points.size())));
  if (bestcount < minimum) degenerate("no observable primitive satisfies the inlier requirement");
  std::pmr::vector<Point3d> inliers(context.scratch_resource());
  if (bestcount != points.size())
    for (auto p : points)
      if (std::abs(residual(p, best, cylinder)) <= cutoff) inliers.push_back(p);
  if (bestcount != points.size() && !refine(inliers, best, cylinder, cutoff, o.irls, o.refine))
    degenerate("inlier geometry does not constrain the primitive");
  std::pmr::vector<uint8_t> keep(model.points.size(), 0, context.scratch_resource());
  double rss = 0;
  size_t count = 0;
  for (size_t j = 0; j < ids.size(); ++j) {
    const double r = residual(points[j], best, cylinder);
    if (std::abs(r) <= cutoff) {
      keep[ids[j]] = 1;
      rss += r * r;
      ++count;
    }
  }
  if (count < minimum) degenerate("refined primitive has insufficient inliers");
  // 在最终接受集合重算条件性，不使用外点。
  double info = best.info;
  if (count != points.size()) {
    inliers.clear();
    for (auto p : points)
      if (std::abs(residual(p, best, cylinder)) <= cutoff) inliers.push_back(p);
    info = geometric_information(inliers, best, cylinder);
    if (info < 1e-10) degenerate("final inliers are degenerate");
  }
  auto out = compact_object_model_3d(model, keep.data());
  out.primitive_kind = cylinder ? PrimitiveKind::Cylinder : PrimitiveKind::Sphere;
  if (out.source_indices.empty()) {
    out.source_indices.reserve(count);
    for (auto i : ids)
      if (keep[i]) out.source_indices.push_back(i);
  }
  const auto center = add(origin, mul(best.center, scale));
  const auto axis = best.axis;
  out.primitive = {center.x,
                   center.y,
                   center.z,
                   cylinder ? axis.x : best.radius * scale,
                   cylinder ? axis.y : 0,
                   cylinder ? axis.z : 0,
                   cylinder ? best.radius * scale : 0};
  if (cylinder) {
    Point3d direction = axis;
    if (direction.z < 0 || (direction.z == 0 && (direction.y < 0 || (direction.y == 0 && direction.x < 0))))
      direction = mul(direction, -1);
    const auto anchor = sub(center, mul(direction, dot(center, direction)));
    out.primitive = {anchor.x,    anchor.y,    anchor.z,           direction.x,
                     direction.y, direction.z, best.radius * scale};
  }
  out.primitive_rms = scale * std::sqrt(rss / count);
  out.primitive_num_inliers = count;
  out.fit_converged = best.converged;
  out.fit_iterations = best.iterations;
  out.information_ratio = info;
  validate_object_model_3d(out);
  return out;
}

}  // namespace

ObjectModel3DData fit_curved_primitive(const ObjectModel3DData& model, std::string_view type,
                                       double threshold, const core::Tuple& names,
                                       const core::Tuple& values) {
  const bool cylinder = type.starts_with("cylinder"), robust = type.ends_with("_ransac");
  if (cylinder)
    return robust ? fit_curved_primitive_t<true, true>(model, type, threshold, names, values)
                  : fit_curved_primitive_t<true, false>(model, type, threshold, names, values);
  return robust ? fit_curved_primitive_t<false, true>(model, type, threshold, names, values)
                : fit_curved_primitive_t<false, false>(model, type, threshold, names, values);
}

core::Tuple distance_to_surface(const ObjectModel3DData& model, const ObjectModel3DData& reference,
                                std::string_view method) {
  runtime::ExecutionContext context;
  std::vector<double> result;
  result.reserve(model.points.size());
  if (method == "triangles") {
    MeshIndex index(reference);
    for (size_t i = 0; i < model.points.size(); ++i)
      if (valid(model, i)) result.push_back(index.distance(model.points[i]));
    return core::Tuple(std::move(result));
  }
  if (reference.primitive_support) {
    for (size_t i = 0; i < model.points.size(); ++i)
      if (valid(model, i)) result.push_back(model_primitive_distance(model.points[i], reference));
    return core::Tuple(std::move(result));
  }
  const auto kind = reference.primitive_kind;
  if (kind == PrimitiveKind::None)
    throw core::Exception(core::Status::InvalidState, "distance_object_model_3d",
                          "reference has no primitive");
  const auto& a = reference.primitive;
  Surface s;
  s.center = {a[0], a[1], a[2]};
  s.axis = {a[3], a[4], a[5]};
  s.radius = kind == PrimitiveKind::Cylinder ? a[6] : a[3];
  if (model.valid.empty() && kind == PrimitiveKind::Sphere) {
    result.resize(model.points.size());
    for (size_t i = 0; i < model.points.size(); ++i) {
      const auto p = model.points[i];
      const double x = p.x - a[0], y = p.y - a[1], z = p.z - a[2];
      result[i] = std::abs(std::sqrt(x * x + y * y + z * z) - a[3]);
    }
    return core::Tuple(std::move(result));
  }
  if (model.valid.empty() && kind == PrimitiveKind::Cylinder) {
    result.resize(model.points.size());
    for (size_t i = 0; i < model.points.size(); ++i) {
      const auto p = model.points[i];
      const double x = p.x - a[0], y = p.y - a[1], z = p.z - a[2], t = x * a[3] + y * a[4] + z * a[5];
      const double dx = x - t * a[3], dy = y - t * a[4], dz = z - t * a[5];
      result[i] = std::abs(std::sqrt(dx * dx + dy * dy + dz * dz) - a[6]);
    }
    return core::Tuple(std::move(result));
  }
  for (size_t i = 0; i < model.points.size(); ++i)
    if (valid(model, i)) {
      const auto p = model.points[i];
      result.push_back(kind == PrimitiveKind::Plane
                           ? std::abs(p.x * reference.plane[0] + p.y * reference.plane[1] +
                                      p.z * reference.plane[2] + reference.plane[3])
                           : std::abs(residual(p, s, kind == PrimitiveKind::Cylinder)));
    }
  return core::Tuple(std::move(result));
}
}  // namespace mv::reconstruction
