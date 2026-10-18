#include <algorithm>
#include <array>
#include <calibration/calibration/homography.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <calibration/pose_estimation.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <type_traits>
#include <vector>

namespace mv::calibration {
namespace {
constexpr auto op = "vector_to_pose";
using V = std::array<double, 3>;
using M = std::array<double, 9>;
[[noreturn]] void fail(core::Status status, core::ErrorCode code, std::string_view parameter,
                       std::string_view message) {
  throw core::Exception(status, code, op, parameter, -1, message);
}
const auto& backend() { return numeric::shared_backend(numeric::default_backend_kind()); }
V add(V a, V b) {
  for (int i = 0; i < 3; ++i) a[i] += b[i];
  return a;
}
V sub(V a, V b) {
  for (int i = 0; i < 3; ++i) a[i] -= b[i];
  return a;
}
V mul(V a, double s) {
  for (auto& v : a) v *= s;
  return a;
}
double dot(V a, V b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V cross(V a, V b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double norm(V a) { return std::sqrt(dot(a, a)); }
V unit(V a) {
  double n = norm(a);
  if (!(n > 0.))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "World", "zero direction");
  return mul(a, 1 / n);
}
V vec(Point3d p) { return {p.x, p.y, p.z}; }
Point3d point(V p) { return {p[0], p[1], p[2]}; }
struct Frame {
  V center{};
  M basis{};
  std::array<double, 3> radius{};
  int rank{};
};
Frame frame(const std::vector<V>& points) {
  Frame out;
  auto anchor = points[0];
  for (auto p : points) out.center = add(out.center, sub(p, anchor));
  out.center = add(anchor, mul(out.center, 1. / points.size()));
  std::vector<double> normal(9), values, vectors;
  for (auto p : points) {
    p = sub(p, out.center);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) normal[r * 3 + c] += p[r] * p[c];
  }
  if (!backend().symmetric_eigen(normal, 3, &values, &vectors) || !(values[2] > 0.))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "World", "zero world spread");
  for (int k = 0; k < 3; ++k) {
    out.radius[k] = std::sqrt(std::max(0., values[2 - k]) / points.size());
    if (values[2 - k] > values[2] * 1e-12) ++out.rank;
    for (int r = 0; r < 3; ++r) out.basis[r * 3 + k] = vectors[r * 3 + 2 - k];
  }
  if (out.rank < 2)
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "World", "collinear world points");
  auto c = cross({out.basis[0], out.basis[3], out.basis[6]}, {out.basis[1], out.basis[4], out.basis[7]});
  for (int r = 0; r < 3; ++r) out.basis[r * 3 + 2] = c[r];
  return out;
}
Pose3d align(const std::vector<V>& world, const std::vector<V>& camera) {
  V a{}, b{};
  for (std::size_t i = 0; i < world.size(); ++i) {
    a = add(a, world[i]);
    b = add(b, camera[i]);
  }
  a = mul(a, 1. / world.size());
  b = mul(b, 1. / world.size());
  M s{};
  for (std::size_t i = 0; i < world.size(); ++i) {
    auto p = sub(world[i], a), q = sub(camera[i], b);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) s[r * 3 + c] += p[r] * q[c];
  }
  const double trace = s[0] + s[4] + s[8];
  std::vector<double> n{trace,       s[5] - s[7],        s[6] - s[2],         s[1] - s[3],
                        s[5] - s[7], s[0] - s[4] - s[8], s[1] + s[3],         s[2] + s[6],
                        s[6] - s[2], s[1] + s[3],        -s[0] + s[4] - s[8], s[5] + s[7],
                        s[1] - s[3], s[2] + s[6],        s[5] + s[7],         -s[0] - s[4] + s[8]},
      values, vectors;
  if (!backend().symmetric_eigen(n, 4, &values, &vectors))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Pose", "rigid alignment failed");
  Quaterniond q{vectors[3], vectors[7], vectors[11], vectors[15]};
  return {point(sub(b, vec(rotate_point(q, point(a))))), q};
}
double cost(const Pose3d& pose, const CameraParameters& camera, const std::vector<V>& world,
            const std::vector<Point2d>& observations) {
  double sum = 0.;
  try {
    for (std::size_t i = 0; i < world.size(); ++i) {
      auto p = transform_point(pose, point(world[i]));
      if (is_perspective(camera) && p.z <= 0.) return std::numeric_limits<double>::infinity();
      auto q = project_camera_point(camera, p);
      double dr = q.row - observations[i].row, dc = q.col - observations[i].col;
      sum += dr * dr + dc * dc;
    }
  } catch (const core::Exception&) {
    return std::numeric_limits<double>::infinity();
  }
  return sum;
}
Pose3d step_pose(Pose3d pose, const std::array<double, 6>& step, bool tele) {
  double a = std::hypot(step[0], std::hypot(step[1], step[2])),
         factor = a < 1e-12 ? .5 : std::sin(a * .5) / a;
  pose = compose_pose(
      {{step[3], step[4], step[5]}, {std::cos(a * .5), step[0] * factor, step[1] * factor, step[2] * factor}},
      pose);
  if (tele) pose.translation.z = 0.;
  return pose;
}
struct ProjectionDerivative {
  Point2d image;
  V row, col;
};
ProjectionDerivative projection_derivative(const CameraParameters& camera, V point_camera) {
  ProjectionDerivative out{project_camera_point(camera, point(point_camera))};
  std::visit(
      [&](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        const double x = (out.image.col - p.cx) * p.sx, y = (out.image.row - p.cy) * p.sy, r2 = x * x + y * y;
        double j00, j01, j11;
        // HALCON 将畸变度量坐标映射到理想坐标；直接求逆解析 2x2 Jacobian，避免重复逆畸变投影。
        if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                      std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
          const double d = 1 + p.kappa * r2, a = 1 / d, b = -2 * p.kappa * a * a;
          j00 = a + b * x * x;
          j01 = b * x * y;
          j11 = a + b * y * y;
        } else {
          const double r4 = r2 * r2, radial = 1 + p.k1 * r2 + p.k2 * r4 + p.k3 * r4 * r2,
                       derivative = 2 * p.k1 + 4 * p.k2 * r2 + 6 * p.k3 * r4;
          j00 = radial + x * x * derivative + 6 * p.p1 * x + 2 * p.p2 * y;
          j01 = x * y * derivative + 2 * p.p1 * y + 2 * p.p2 * x;
          j11 = radial + y * y * derivative + 2 * p.p1 * x + 6 * p.p2 * y;
        }
        const double determinant = j00 * j11 - j01 * j01;
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-20)
          fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "CameraParam",
               "singular distortion derivative");
        const double col_x = j11 / (determinant * p.sx), col_y = -j01 / (determinant * p.sx),
                     row_x = -j01 / (determinant * p.sy), row_y = j00 / (determinant * p.sy);
        if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                      std::is_same_v<T, AreaScanPolynomialParameters>) {
          const double scale = p.focus / point_camera[2];
          out.row = {scale * row_x, scale * row_y,
                     -scale * (row_x * point_camera[0] + row_y * point_camera[1]) / point_camera[2]};
          out.col = {scale * col_x, scale * col_y,
                     -scale * (col_x * point_camera[0] + col_y * point_camera[1]) / point_camera[2]};
        } else {
          out.row = {p.magnification * row_x, p.magnification * row_y, 0.};
          out.col = {p.magnification * col_x, p.magnification * col_y, 0.};
        }
      },
      camera);
  return out;
}
Pose3d refine(Pose3d pose, const CameraParameters& camera, const std::vector<V>& world,
              const std::vector<Point2d>& observed) {
  const bool tele = !is_perspective(camera);
  const int n = tele ? 5 : 6;
  double current = cost(pose, camera, world, observed), lambda = 1e-4;
  if (!std::isfinite(current) || current < 1e-20) return pose;
  for (int iteration = 0; iteration < 100; ++iteration) {
    std::vector<double> normal(n * n), rhs(n), solution(n);
    bool valid = true;
    for (std::size_t i = 0; i < world.size() && valid; ++i) {
      try {
        const auto p = vec(transform_point(pose, point(world[i])));
        const auto derivative = projection_derivative(camera, p);
        const auto q = derivative.image;
        const auto& gr = derivative.row;
        const auto& gc = derivative.col;
        const std::array<double, 6> jr{-p[2] * gr[1] + p[1] * gr[2],
                                       p[2] * gr[0] - p[0] * gr[2],
                                       -p[1] * gr[0] + p[0] * gr[1],
                                       gr[0],
                                       gr[1],
                                       gr[2]},
            jc{-p[2] * gc[1] + p[1] * gc[2],
               p[2] * gc[0] - p[0] * gc[2],
               -p[1] * gc[0] + p[0] * gc[1],
               gc[0],
               gc[1],
               gc[2]};
        for (int r = 0; r < n; ++r) {
          rhs[r] += jr[r] * (observed[i].row - q.row) + jc[r] * (observed[i].col - q.col);
          for (int c = 0; c < n; ++c) normal[r * n + c] += jr[r] * jr[c] + jc[r] * jc[c];
        }
      } catch (const core::Exception&) {
        valid = false;
      }
    }
    if (!valid) break;
    bool accepted = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
      auto damped = normal;
      for (int k = 0; k < n; ++k) damped[k * n + k] += lambda * std::max(normal[k * n + k], 1e-20);
      if (!backend().solve_symmetric_into(damped, rhs, solution)) {
        lambda *= 10.;
        continue;
      }
      std::array<double, 6> step{};
      for (int k = 0; k < n; ++k) step[k] = solution[k];
      auto candidate = step_pose(pose, step, tele);
      double next = cost(candidate, camera, world, observed);
      if (next < current) {
        double improvement = current - next;
        pose = candidate;
        current = next;
        lambda = std::max(1e-15, lambda * .25);
        accepted = true;
        if (improvement < 1e-14 * (1 + current)) iteration = 100;
        break;
      }
      lambda *= 10.;
    }
    if (!accepted) break;
  }
  return pose;
}
// 按导数区间隔离 P3P 四次方程的正实根。
using Polynomial = std::vector<long double>;
long double evaluate(const Polynomial& p, long double x) {
  long double v = 0;
  for (auto i = p.rbegin(); i != p.rend(); ++i) v = v * x + *i;
  return v;
}
Polynomial product(const Polynomial& a, const Polynomial& b) {
  Polynomial p(a.size() + b.size() - 1);
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = 0; j < b.size(); ++j) p[i + j] += a[i] * b[j];
  return p;
}
std::vector<long double> roots(Polynomial p) {
  long double scale = 0;
  for (auto v : p) scale = std::max(scale, std::abs(v));
  if (scale == 0) return {};
  for (auto& v : p) v /= scale;
  while (p.size() > 1 && std::abs(p.back()) < 1e-16L) p.pop_back();
  if (p.size() == 1) return {};
  if (p.size() == 2) return {-p[0] / p[1]};
  Polynomial derivative(p.size() - 1);
  for (std::size_t k = 1; k < p.size(); ++k) derivative[k - 1] = k * p[k];
  auto critical = roots(derivative);
  long double bound = 1;
  for (std::size_t k = 0; k + 1 < p.size(); ++k) bound = std::max(bound, 1 + std::abs(p[k] / p.back()));
  std::vector<long double> cuts{-bound}, out;
  for (auto r : critical)
    if (r > -bound && r < bound) cuts.push_back(r);
  cuts.push_back(bound);
  std::sort(cuts.begin(), cuts.end());
  for (auto r : critical)
    if (std::abs(evaluate(p, r)) < 1e-13L) out.push_back(r);
  for (std::size_t k = 1; k < cuts.size(); ++k) {
    auto lo = cuts[k - 1], hi = cuts[k], a = evaluate(p, lo), b = evaluate(p, hi);
    if (a * b >= 0) continue;
    for (int j = 0; j < 100; ++j) {
      auto mid = (lo + hi) * .5L, v = evaluate(p, mid);
      if (a * v <= 0)
        hi = mid;
      else {
        lo = mid;
        a = v;
      }
    }
    out.push_back((lo + hi) * .5L);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end(),
                        [](auto a, auto b) { return std::abs(a - b) < 1e-9L * (1 + std::abs(a)); }),
            out.end());
  return out;
}
std::vector<Pose3d> p3p(const std::vector<V>& world, const std::vector<V>& rays) {
  std::array<std::size_t, 3> ids{0, 1, 2};
  double distance = 0;
  for (std::size_t i = 1; i < world.size(); ++i) {
    double d = dot(sub(world[i], world[0]), sub(world[i], world[0]));
    if (d > distance) {
      distance = d;
      ids[1] = i;
    }
  }
  double area = 0;
  for (std::size_t i = 1; i < world.size(); ++i) {
    auto c = cross(sub(world[ids[1]], world[0]), sub(world[i], world[0]));
    double d = dot(c, c);
    if (d > area) {
      area = d;
      ids[2] = i;
    }
  }
  std::vector<Pose3d> out;
  if (area <= distance * distance * 1e-14) return out;
  for (int permutation = 0; permutation < 3; ++permutation) {
    auto i = ids[permutation], j = ids[(permutation + 1) % 3], k = ids[(permutation + 2) % 3];
    double a = dot(sub(world[j], world[k]), sub(world[j], world[k])),
           b = dot(sub(world[i], world[k]), sub(world[i], world[k])),
           c = dot(sub(world[i], world[j]), sub(world[i], world[j]));
    a /= c;
    b /= c;
    double alpha = dot(rays[j], rays[k]), beta = dot(rays[i], rays[k]), gamma = dot(rays[i], rays[j]);
    Polynomial d{1., -2 * gamma, 1.}, n{a - b + 1, -2 * gamma * (a - b), a - b - 1},
        den{2 * beta, -2 * alpha};
    auto nn = product(n, n), dd = product(den, den), nd = product(n, den), ddd = product(d, dd);
    Polynomial p(5);
    for (std::size_t r = 0; r < p.size(); ++r)
      p[r] = (r < nn.size() ? nn[r] : 0) + (r < dd.size() ? dd[r] : 0) -
             2 * beta * (r < nd.size() ? nd[r] : 0) - b * ddd[r];
    for (auto u : roots(p)) {
      if (u <= 0) continue;
      auto divisor = evaluate(den, u);
      if (std::abs(divisor) < 1e-12) continue;
      auto v = evaluate(n, u) / divisor, D = evaluate(d, u);
      if (v <= 0 || D <= 0) continue;
      double s = std::sqrt(c / double(D));
      std::vector<V> w{world[i], world[j], world[k]},
          q{mul(rays[i], s), mul(rays[j], s * double(u)), mul(rays[k], s * double(v))};
      out.push_back(align(w, q));
    }
  }
  return out;
}
// EPnP 控制点/特征基构造为独立实现；算法参考：https://www.epfl.ch/labs/cvlab/software/multi-view-stereo/epnp/
std::vector<Pose3d> epnp(const std::vector<V>& world, const std::vector<V>& rays, const Frame& frame) {
  const int controls = frame.rank == 2 ? 3 : 4, dimension = controls * 3, bases = controls;
  std::vector<V> points{frame.center};
  for (int k = 0; k < controls - 1; ++k)
    points.push_back(
        add(frame.center, mul({frame.basis[k], frame.basis[3 + k], frame.basis[6 + k]}, frame.radius[k])));
  std::vector<std::array<double, 4>> weights(world.size());
  std::vector<double> normal(dimension * dimension);
  for (std::size_t i = 0; i < world.size(); ++i) {
    auto p = sub(world[i], frame.center);
    auto& w = weights[i];
    w[0] = 1.;
    for (int k = 1; k < controls; ++k) {
      w[k] =
          dot(p, {frame.basis[k - 1], frame.basis[3 + k - 1], frame.basis[6 + k - 1]}) / frame.radius[k - 1];
      w[0] -= w[k];
    }
    std::vector<double> a(dimension), b(dimension);
    double u = rays[i][0] / rays[i][2], v = rays[i][1] / rays[i][2];
    for (int k = 0; k < controls; ++k) {
      a[k * 3] = w[k];
      a[k * 3 + 2] = -u * w[k];
      b[k * 3 + 1] = w[k];
      b[k * 3 + 2] = -v * w[k];
    }
    for (int r = 0; r < dimension; ++r)
      for (int c = 0; c < dimension; ++c) normal[r * dimension + c] += a[r] * a[c] + b[r] * b[c];
  }
  std::vector<double> values, vectors;
  if (!backend().symmetric_eigen(normal, dimension, &values, &vectors)) return {};
  struct Edge {
    std::array<V, 4> directions{};
    double squared;
  };
  std::vector<Edge> edges;
  for (int a = 0; a < controls; ++a)
    for (int b = a + 1; b < controls; ++b) {
      Edge e;
      e.squared = dot(sub(points[a], points[b]), sub(points[a], points[b]));
      for (int k = 0; k < bases; ++k)
        for (int r = 0; r < 3; ++r)
          e.directions[k][r] = vectors[(a * 3 + r) * dimension + k] - vectors[(b * 3 + r) * dimension + k];
      edges.push_back(e);
    }
  std::vector<std::array<double, 4>> seeds;
  for (int count = 1; count <= std::min(3, bases); ++count) {
    int n = count * (count + 1) / 2;
    std::vector<double> gram(n * n), rhs(n), solution(n);
    for (auto& e : edges) {
      std::vector<double> row;
      for (int a = 0; a < count; ++a)
        for (int b = a; b < count; ++b)
          row.push_back(dot(e.directions[a], e.directions[b]) * (a == b ? 1 : 2));
      for (int r = 0; r < n; ++r) {
        rhs[r] += row[r] * e.squared;
        for (int c = 0; c < n; ++c) gram[r * n + c] += row[r] * row[c];
      }
    }
    if (!backend().solve_symmetric_into(gram, rhs, solution)) continue;
    std::vector<double> square(count * count);
    int at = 0;
    for (int a = 0; a < count; ++a)
      for (int b = a; b < count; ++b) square[a * count + b] = square[b * count + a] = solution[at++];
    std::vector<double> eigen, basis;
    if (!backend().symmetric_eigen(square, count, &eigen, &basis)) continue;
    std::array<double, 4> seed{};
    for (int k = 0; k < count; ++k)
      seed[k] = basis[k * count + count - 1] * std::sqrt(std::abs(eigen.back()));
    seeds.push_back(seed);
  }
  std::vector<Pose3d> out;
  for (auto beta : seeds) {
    auto error = [&](const auto& b) {
      double sum = 0;
      for (auto& e : edges) {
        V q{};
        for (int k = 0; k < bases; ++k) q = add(q, mul(e.directions[k], b[k]));
        double r = dot(q, q) - e.squared;
        sum += r * r;
      }
      return sum;
    };
    double current = error(beta), lambda = 1e-6;
    for (int iteration = 0; iteration < 50; ++iteration) {
      std::vector<double> gram(bases * bases), rhs(bases), solution(bases);
      for (auto& e : edges) {
        V q{};
        for (int k = 0; k < bases; ++k) q = add(q, mul(e.directions[k], beta[k]));
        double r = e.squared - dot(q, q);
        std::array<double, 4> j{};
        for (int k = 0; k < bases; ++k) j[k] = 2 * dot(q, e.directions[k]);
        for (int a = 0; a < bases; ++a) {
          rhs[a] += j[a] * r;
          for (int b = 0; b < bases; ++b) gram[a * bases + b] += j[a] * j[b];
        }
      }
      bool accepted = false;
      for (int attempt = 0; attempt < 15; ++attempt) {
        auto damped = gram;
        for (int k = 0; k < bases; ++k)
          damped[k * bases + k] += lambda * std::max(gram[k * bases + k], 1e-20);
        if (!backend().solve_symmetric_into(damped, rhs, solution)) {
          lambda *= 10.;
          continue;
        }
        auto candidate = beta;
        for (int k = 0; k < bases; ++k) candidate[k] += solution[k];
        double next = error(candidate);
        if (next < current) {
          beta = candidate;
          double improvement = current - next;
          current = next;
          lambda = std::max(1e-15, lambda * .2);
          accepted = true;
          if (improvement < 1e-24) iteration = 50;
          break;
        }
        lambda *= 10.;
      }
      if (!accepted) break;
    }
    std::vector<V> cp(controls);
    for (int a = 0; a < controls; ++a)
      for (int r = 0; r < 3; ++r)
        for (int k = 0; k < bases; ++k) cp[a][r] += vectors[(a * 3 + r) * dimension + k] * beta[k];
    std::vector<V> camera(world.size());
    for (std::size_t i = 0; i < world.size(); ++i)
      for (int a = 0; a < controls; ++a) camera[i] = add(camera[i], mul(cp[a], weights[i][a]));
    if (camera[0][2] < 0)
      for (auto& p : camera) p = mul(p, -1.);
    out.push_back(align(world, camera));
  }
  return out;
}
Pose3d orthographic(const std::vector<V>& world, const std::vector<V>& rays, const Frame& f) {
  int rank = f.rank, n = rank + 1;
  std::vector<double> gram(n * n), bx(n), by(n), ax(n), ay(n);
  V mean{};
  for (std::size_t i = 0; i < world.size(); ++i) {
    std::array<double, 4> row{};
    auto p = sub(world[i], f.center);
    for (int k = 0; k < rank; ++k)
      row[k] = dot(p, {f.basis[k], f.basis[3 + k], f.basis[6 + k]}) / f.radius[k];
    row[rank] = 1.;
    mean = add(mean, rays[i]);
    for (int r = 0; r < n; ++r) {
      bx[r] += row[r] * rays[i][0];
      by[r] += row[r] * rays[i][1];
      for (int c = 0; c < n; ++c) gram[r * n + c] += row[r] * row[c];
    }
  }
  if (!backend().solve_symmetric_into(gram, bx, ax) || !backend().solve_symmetric_into(gram, by, ay))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Pose", "orthographic fit failed");
  V a{}, b{};
  for (int k = 0; k < rank; ++k) {
    a[k] = ax[k] / f.radius[k];
    b[k] = ay[k] / f.radius[k];
  }
  if (rank == 2) {
    a[2] = std::sqrt(std::max(0., 1 - dot(a, a)));
    b[2] = std::sqrt(std::max(0., 1 - dot(b, b)));
    if (dot(a, b) > 0) b[2] = -b[2];
  }
  a = unit(a);
  b = unit(sub(b, mul(a, dot(a, b))));
  auto c = cross(a, b);
  M local{a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]}, rotation{};
  for (int r = 0; r < 3; ++r)
    for (int col = 0; col < 3; ++col)
      for (int k = 0; k < 3; ++k) rotation[r * 3 + col] += local[r * 3 + k] * f.basis[col * 3 + k];
  auto q = quaternion_from_rotation_matrix(rotation);
  auto center = vec(rotate_point(q, point(f.center)));
  mean = mul(mean, 1. / world.size());
  return {{mean[0] - center[0], mean[1] - center[1], 0.}, q};
}
}  // namespace
PoseEstimate estimate_pose(const core::Tuple& wx, const core::Tuple& wy, const core::Tuple& wz,
                           const core::Tuple& row, const core::Tuple& col, const core::Tuple& camera_tuple,
                           std::string_view method) {
  const bool tele_method = method == "telecentric" || method == "telecentric_robust" ||
                           method == "telecentric_planar" || method == "telecentric_planar_robust";
  const bool planar_method =
      method == "planar_analytic" || method == "telecentric_planar" || method == "telecentric_planar_robust";
  if (!tele_method && method != "analytic" && method != "iterative" && method != "planar_analytic" &&
      method != "line_scan")
    fail(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, "Method", "unknown method");
  auto camera = parse_camera_parameters(camera_tuple);
  if (method == "line_scan" || tele_method == is_perspective(camera))
    fail(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, "Method",
         "method does not match camera model");
  const std::size_t n = wx.size(), minimum = method == "iterative" || method == "telecentric_planar" ||
                                                     method == "telecentric_planar_robust"
                                                 ? 3
                                                 : 4;
  if (n < minimum || (method == "analytic" && n > 32767))
    fail(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, "WorldX",
         "invalid point count for method");
  const std::array<const core::Tuple*, 5> controls{&wx, &wy, &wz, &row, &col};
  const std::array<std::string_view, 5> names{"WorldX", "WorldY", "WorldZ", "ImageRow", "ImageColumn"};
  std::array<std::vector<double>, 5> data;
  for (int k = 0; k < 5; ++k) {
    if (k == 2 && controls[k]->empty()) {
      data[k].assign(n, 0.);
      continue;
    }
    if (controls[k]->size() != n)
      fail(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, names[k],
           "coordinate lengths must match");
    data[k].reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      auto v = controls[k]->at(i);
      if (v.kind() != core::ValueKind::Int64 && v.kind() != core::ValueKind::Real64)
        fail(core::Status::TypeMismatch, core::ErrorCode::InvalidType, names[k],
             "numeric coordinates required");
      double x = v.as_real64();
      if (!std::isfinite(x))
        fail(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, names[k],
             "finite coordinates required");
      data[k].push_back(x);
    }
  }
  std::vector<V> world, rays;
  std::vector<Point2d> observed;
  for (std::size_t i = 0; i < n; ++i) {
    world.push_back({data[0][i], data[1][i], planar_method ? 0. : data[2][i]});
    observed.push_back({data[3][i], data[4][i]});
    auto ray = image_point_to_ray(camera, observed.back());
    rays.push_back(tele_method ? vec(ray.origin) : unit(vec(ray.direction)));
  }
  const auto f = frame(world);
  auto optimization_camera = camera;
  auto optimization_observed = observed;
  if (tele_method) {
    // HALCON 正交拟合在理想相机平面进行，Quality 在原畸变图像中测量；
    // 即使畸变/噪声很小，两项目标也有可测差异。
    std::visit(
        [](auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, AreaScanTelecentricDivisionParameters>)
            p.kappa = 0.;
          else if constexpr (std::is_same_v<T, AreaScanTelecentricPolynomialParameters>)
            p.k1 = p.k2 = p.k3 = p.p1 = p.p2 = 0.;
        },
        optimization_camera);
    for (std::size_t i = 0; i < n; ++i)
      optimization_observed[i] = project_camera_point(optimization_camera, point(rays[i]));
  }
  std::vector<Pose3d> candidates;
  if (tele_method) {
    if (method == "telecentric" && f.rank < 3)
      fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "World",
           "general telecentric method requires spatial points");
    candidates.push_back(orthographic(world, rays, f));
  } else {
    if (n >= 4) candidates = epnp(world, rays, f);
    auto minimal = p3p(world, rays);
    candidates.insert(candidates.end(), minimal.begin(), minimal.end());
    if (f.rank == 2 && n >= 4) {
      std::vector<Point3d> local;
      std::vector<double> rows, cols;
      std::vector<std::int64_t> ids;
      for (std::size_t i = 0; i < n; ++i) {
        auto p = sub(world[i], f.center);
        local.push_back(
            {dot(p, {f.basis[0], f.basis[3], f.basis[6]}), dot(p, {f.basis[1], f.basis[4], f.basis[7]}), 0.});
        rows.push_back(observed[i].row);
        cols.push_back(observed[i].col);
        ids.push_back(i);
      }
      try {
        auto pose = estimate_planar_pose_perspective(camera, local, rows, cols, ids);
        auto r = rotation_matrix(pose.rotation);
        M global{};
        for (int a = 0; a < 3; ++a)
          for (int b = 0; b < 3; ++b)
            for (int k = 0; k < 3; ++k) global[a * 3 + b] += r[a * 3 + k] * f.basis[b * 3 + k];
        pose.rotation = quaternion_from_rotation_matrix(global);
        pose.translation =
            point(sub(vec(pose.translation), vec(rotate_point(pose.rotation, point(f.center)))));
        candidates.push_back(pose);
      } catch (const core::Exception&) {
      }
    }
  }
  double best = std::numeric_limits<double>::infinity();
  Pose3d pose;
  // 固定数量代数候选统一排序，仅细化前三个；P3P 排列可能重复同一物理解，
  // 不对每个副本重复完整 N 点 LM 拟合。
  std::vector<std::pair<double, Pose3d>> ranked;
  for (auto candidate : candidates) {
    const auto initial = cost(candidate, camera, world, observed);
    if (std::isfinite(initial)) ranked.emplace_back(initial, candidate);
  }
  std::stable_sort(ranked.begin(), ranked.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });
  for (std::size_t i = 0; i < std::min<std::size_t>(3, ranked.size()); ++i) {
    auto candidate = ranked[i].second;
    candidate = refine(candidate, optimization_camera, world, optimization_observed);
    double error = cost(candidate, camera, world, observed);
    if (error < best) {
      best = error;
      pose = candidate;
    }
  }
  if (!std::isfinite(best))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Pose", "no valid pose solution");
  return {pose_to_halcon_tuple(pose), std::sqrt(best / n)};
}
}  // namespace mv::calibration
