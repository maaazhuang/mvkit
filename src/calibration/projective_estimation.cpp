#include <algorithm>
#include <array>
#include <calibration/numeric/linear_backend.hpp>
#include <calibration/projective_estimation.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <vector>

namespace mv::calibration {
namespace {
constexpr auto op = "vector_to_proj_hom_mat2d";
using Matrix = std::array<double, 9>;
using Normal = std::array<double, 64>;
using Gradient = std::array<double, 8>;
struct Point {
  double x, y;
};
struct Symmetric2 {
  double xx = 1., xy = 0., yy = 1.;
};
struct Normalization {
  double x = 0., y = 0., scale = 1.;
};

[[noreturn]] void fail(core::Status status, core::ErrorCode code, std::string_view parameter,
                       std::string_view message) {
  throw core::Exception(status, code, op, parameter, -1, message);
}
std::vector<double> values(const core::Tuple& tuple, std::string_view parameter) {
  std::vector<double> output;
  output.reserve(tuple.size());
  for (std::size_t i = 0; i < tuple.size(); ++i) {
    const auto value = tuple.at(i);
    if (value.kind() != core::ValueKind::Int64 && value.kind() != core::ValueKind::Real64)
      fail(core::Status::TypeMismatch, core::ErrorCode::InvalidType, parameter, "numeric values required");
    const auto number = value.as_real64();
    if (!std::isfinite(number))
      fail(core::Status::InvalidState, core::ErrorCode::InvalidValue, parameter,
           "finite coordinates required");
    output.push_back(number);
  }
  return output;
}
Normalization normalize(const std::vector<Point>& points) {
  Normalization out;
  const bool constant_x =
      std::all_of(points.begin(), points.end(), [&](auto p) { return p.x == points[0].x; });
  const bool constant_y =
      std::all_of(points.begin(), points.end(), [&](auto p) { return p.y == points[0].y; });
  if (constant_x || constant_y)
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "points",
         "normalization requires spread in both axes");
  for (auto p : points) {
    out.x += p.x;
    out.y += p.y;
  }
  out.x /= double(points.size());
  out.y /= double(points.size());
  double radius = 0.;
  for (auto p : points) radius += std::hypot(p.x - out.x, p.y - out.y);
  if (!std::isfinite(radius) || radius == 0.)
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "points",
         "degenerate point spread");
  out.scale = std::sqrt(2.) * double(points.size()) / radius;
  return out;
}
Matrix multiply(const Matrix& a, const Matrix& b) {
  Matrix out{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      for (int k = 0; k < 3; ++k) out[r * 3 + c] += a[r * 3 + k] * b[k * 3 + c];
  return out;
}
Matrix denormalize(const Matrix& h, const Normalization& p, const Normalization& q) {
  const Matrix left{1. / q.scale, 0., q.x, 0., 1. / q.scale, q.y, 0., 0., 1.};
  const Matrix right{p.scale, 0., -p.scale * p.x, 0., p.scale, -p.scale * p.y, 0., 0., 1.};
  return multiply(left, multiply(h, right));
}
Point normalized(Point p, const Normalization& n) { return {(p.x - n.x) * n.scale, (p.y - n.y) * n.scale}; }
bool project(const Matrix& h, Point p, Point* q, std::array<double, 4>* a = nullptr,
             std::array<double, 18>* j = nullptr) {
  const double w = h[6] * p.x + h[7] * p.y + h[8];
  if (w == 0. || !std::isfinite(w)) return false;
  q->x = (h[0] * p.x + h[1] * p.y + h[2]) / w;
  q->y = (h[3] * p.x + h[4] * p.y + h[5]) / w;
  if (!std::isfinite(q->x) || !std::isfinite(q->y)) return false;
  if (a)
    *a = {(h[0] - q->x * h[6]) / w, (h[1] - q->x * h[7]) / w, (h[3] - q->y * h[6]) / w,
          (h[4] - q->y * h[7]) / w};
  if (j)
    *j = {p.x / w, p.y / w, 1. / w, 0.,      0.,      0.,     -q->x * p.x / w, -q->x * p.y / w, -q->x / w,
          0.,      0.,      0.,     p.x / w, p.y / w, 1. / w, -q->y * p.x / w, -q->y * p.y / w, -q->y / w};
  return true;
}
bool inverse(Symmetric2 c, Symmetric2* out) {
  const double determinant = c.xx * c.yy - c.xy * c.xy;
  if (!(determinant > 0.) || !std::isfinite(determinant)) return false;
  *out = {c.yy / determinant, -c.xy / determinant, c.xx / determinant};
  return true;
}
// 按独立 HALCON 探针行为，归一化前将奇异输入协方差替换为按 trace 缩放的各向同性方差。
Symmetric2 precision(Symmetric2 c) {
  Symmetric2 out;
  if (inverse(c, &out)) return out;
  const double trace = c.xx + c.yy;
  if (trace == 0.) return {0., 0., 0.};
  return {c.xx / (trace * trace), c.xy / (trace * trace), c.yy / (trace * trace)};
}
double quadratic(Symmetric2 w, Point p) {
  return w.xx * p.x * p.x + 2. * w.xy * p.x * p.y + w.yy * p.y * p.y;
}
Symmetric2 propagated(Symmetric2 p, Symmetric2 q, const std::array<double, 4>& a) {
  return {q.xx + a[0] * a[0] * p.xx + 2. * a[0] * a[1] * p.xy + a[1] * a[1] * p.yy,
          q.xy + a[0] * a[2] * p.xx + (a[0] * a[3] + a[1] * a[2]) * p.xy + a[1] * a[3] * p.yy,
          q.yy + a[2] * a[2] * p.xx + 2. * a[2] * a[3] * p.xy + a[3] * a[3] * p.yy};
}
struct Observation {
  Point p, q;
  Symmetric2 source, target, source_precision, target_precision;
};
double cost(const Matrix& h, const std::vector<Point>& corrected,
            const std::vector<Observation>& observations) {
  double sum = 0.;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto& obs = observations[i];
    Point q;
    if (!project(h, corrected[i], &q)) return std::numeric_limits<double>::infinity();
    sum += quadratic(obs.source_precision, {corrected[i].x - obs.p.x, corrected[i].y - obs.p.y}) +
           quadratic(obs.target_precision, {q.x - obs.q.x, q.y - obs.q.y});
  }
  return sum;
}
bool equations(const Matrix& h, const std::vector<Point>& corrected,
               const std::vector<Observation>& observations, const std::array<int, 8>& active, Normal* normal,
               Gradient* rhs) {
  normal->fill(0.);
  rhs->fill(0.);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto& obs = observations[i];
    Point q;
    std::array<double, 4> a;
    std::array<double, 18> j;
    if (!project(h, corrected[i], &q, &a, &j)) return false;
    Symmetric2 w;
    if (!inverse(propagated(obs.source, obs.target, a), &w)) return false;
    const Point e{corrected[i].x - obs.p.x, corrected[i].y - obs.p.y};
    const Point r{q.x - obs.q.x - a[0] * e.x - a[1] * e.y, q.y - obs.q.y - a[2] * e.x - a[3] * e.y};
    const Point weighted{w.xx * r.x + w.xy * r.y, w.xy * r.x + w.yy * r.y};
    for (int row = 0; row < 8; ++row) {
      const int u = active[row];
      (*rhs)[row] -= j[u] * weighted.x + j[9 + u] * weighted.y;
      for (int col = 0; col < 8; ++col) {
        const int v = active[col];
        (*normal)[row * 8 + col] +=
            j[u] * (w.xx * j[v] + w.xy * j[9 + v]) + j[9 + u] * (w.xy * j[v] + w.yy * j[9 + v]);
      }
    }
  }
  return true;
}
std::vector<Point> point_step(const Matrix& h, const std::vector<Point>& corrected,
                              const std::vector<Observation>& observations, const Matrix& step) {
  std::vector<Point> result;
  result.reserve(observations.size());
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto& obs = observations[i];
    Point q;
    std::array<double, 4> a;
    std::array<double, 18> j;
    if (!project(h, corrected[i], &q, &a, &j)) return {};
    Symmetric2 w;
    if (!inverse(propagated(obs.source, obs.target, a), &w)) return {};
    const Point e{corrected[i].x - obs.p.x, corrected[i].y - obs.p.y};
    Point r{q.x - obs.q.x - a[0] * e.x - a[1] * e.y, q.y - obs.q.y - a[2] * e.x - a[3] * e.y};
    for (int k = 0; k < 9; ++k) {
      r.x += j[k] * step[k];
      r.y += j[9 + k] * step[k];
    }
    const Point weighted{w.xx * r.x + w.xy * r.y, w.xy * r.x + w.yy * r.y};
    const Point adjoint{a[0] * weighted.x + a[2] * weighted.y, a[1] * weighted.x + a[3] * weighted.y};
    result.push_back({obs.p.x - obs.source.xx * adjoint.x - obs.source.xy * adjoint.y,
                      obs.p.y - obs.source.xy * adjoint.x - obs.source.yy * adjoint.y});
  }
  return result;
}
Matrix gold_standard(Matrix h, const std::vector<Observation>& observations, std::vector<Point>* fitted) {
  const int fixed =
      int(std::max_element(h.begin(), h.end(), [](double a, double b) { return std::abs(a) < std::abs(b); }) -
          h.begin());
  std::array<int, 8> active;
  int n = 0;
  for (int i = 0; i < 9; ++i)
    if (i != fixed) active[n++] = i;
  std::vector<Point> corrected;
  corrected.reserve(observations.size());
  for (auto& obs : observations) corrected.push_back(obs.p);
  double current = cost(h, corrected, observations), lambda = 1e-4;
  if (!std::isfinite(current))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "points",
         "invalid initial projection");
  const auto& backend = numeric::shared_backend(numeric::default_backend_kind());
  for (int iteration = 0; iteration < 100; ++iteration) {
    Normal normal;
    Gradient rhs, solution;
    if (!equations(h, corrected, observations, active, &normal, &rhs))
      fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Covariance",
           "singular measurement information");
    if (current < 1e-24 ||
        *std::max_element(
            rhs.begin(), rhs.end(), [](double a, double b) { return std::abs(a) < std::abs(b); }) == 0.)
      break;
    bool accepted = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
      auto damped = normal;
      for (int k = 0; k < 8; ++k) damped[k * 8 + k] += lambda * std::max(normal[k * 8 + k], 1e-15);
      if (!backend.solve_symmetric_into(damped, rhs, solution)) {
        lambda *= 10.;
        continue;
      }
      Matrix step{};
      for (int k = 0; k < 8; ++k) step[active[k]] = solution[k];
      auto candidate_points = point_step(h, corrected, observations, step);
      auto candidate = h;
      for (int k = 0; k < 9; ++k) candidate[k] += step[k];
      const double next = candidate_points.size() == observations.size()
                              ? cost(candidate, candidate_points, observations)
                              : std::numeric_limits<double>::infinity();
      if (next < current) {
        const double improvement = current - next;
        h = candidate;
        corrected = std::move(candidate_points);
        current = next;
        lambda = std::max(lambda * .25, 1e-15);
        accepted = true;
        if (improvement <= 1e-12 * (1. + current)) iteration = 100;
        break;
      }
      lambda *= 10.;
    }
    if (!accepted) break;
  }
  *fitted = std::move(corrected);
  return h;
}
std::vector<double> covariance(const std::vector<Observation>& observations, const std::vector<Point>& fitted,
                               const Normalization& p, const Normalization& q, const Matrix& raw) {
  const auto& backend = numeric::shared_backend(numeric::default_backend_kind());
  std::vector<double> information(81, 0.);
  for (std::size_t i = 0; i < fitted.size(); ++i) {
    const auto& obs = observations[i];
    const Symmetric2 source{obs.source.xx / (p.scale * p.scale), obs.source.xy / (p.scale * p.scale),
                            obs.source.yy / (p.scale * p.scale)};
    const Symmetric2 target{obs.target.xx / (q.scale * q.scale), obs.target.xy / (q.scale * q.scale),
                            obs.target.yy / (q.scale * q.scale)};
    const Point point{fitted[i].x / p.scale + p.x, fitted[i].y / p.scale + p.y};
    Point projected;
    std::array<double, 4> a;
    std::array<double, 18> j;
    Symmetric2 w;
    if (!project(raw, point, &projected, &a, &j) || !inverse(propagated(source, target, a), &w))
      fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Covariance",
           "invalid information matrix");
    for (int r = 0; r < 9; ++r)
      for (int c = 0; c < 9; ++c)
        information[r * 9 + c] +=
            j[r] * (w.xx * j[c] + w.xy * j[9 + c]) + j[9 + r] * (w.xy * j[c] + w.yy * j[9 + c]);
  }
  std::vector<double> eigenvalues, eigenvectors;
  if (!backend.symmetric_eigen(information, 9, &eigenvalues, &eigenvectors))
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "Covariance",
         "covariance solve failed");
  std::vector<double> out(81, 0.);
  // HALCON 协方差中的原坐标信息截断可观测；独立仿射/投影探针将相对截断夹在
  // [9.5846e-8,1.2517e-7]。明确保留推断的 1e-7；归一化求逆会改变平移后的数值秩。
  const double tolerance = std::max(1e-30, std::abs(eigenvalues.back()) * 1e-7);
  for (int k = 0; k < 9; ++k) {
    if (eigenvalues[k] <= tolerance) continue;
    for (int r = 0; r < 9; ++r)
      for (int c = 0; c < 9; ++c)
        out[r * 9 + c] += eigenvectors[r * 9 + k] * eigenvectors[c * 9 + k] / eigenvalues[k];
  }
  return out;
}
}  // namespace

ProjectiveEstimate estimate_projective_transform(const core::Tuple& px, const core::Tuple& py,
                                                 const core::Tuple& qx, const core::Tuple& qy,
                                                 std::string_view method,
                                                 const std::array<core::Tuple, 6>& covariances) {
  if (method != "dlt" && method != "normalized_dlt" && method != "gold_standard")
    fail(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, "Method", "unknown estimation method");
  const std::array<const core::Tuple*, 4> controls{&px, &py, &qx, &qy};
  const std::array<std::string_view, 4> names{"Px", "Py", "Qx", "Qy"};
  std::array<std::vector<double>, 4> coordinates;
  for (int i = 0; i < 4; ++i) {
    if (controls[i]->size() < 4 || controls[i]->size() != px.size())
      fail(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, names[i],
           "four or more equally sized coordinate arrays required");
    coordinates[i] = values(*controls[i], names[i]);
  }
  std::vector<Point> source, target;
  source.reserve(px.size());
  target.reserve(px.size());
  for (std::size_t i = 0; i < px.size(); ++i) {
    source.push_back({coordinates[0][i], coordinates[1][i]});
    target.push_back({coordinates[2][i], coordinates[3][i]});
  }
  const auto p = method == "dlt" ? Normalization{} : normalize(source);
  const auto q = method == "dlt" ? Normalization{} : normalize(target);
  std::vector<double> design;
  design.reserve(source.size() * 18);
  for (std::size_t i = 0; i < source.size(); ++i) {
    source[i] = normalized(source[i], p);
    target[i] = normalized(target[i], q);
    const auto a = source[i], b = target[i];
    const Matrix row_x{-a.x, -a.y, -1., 0., 0., 0., b.x * a.x, b.x * a.y, b.x};
    const Matrix row_y{0., 0., 0., -a.x, -a.y, -1., b.y * a.x, b.y * a.y, b.y};
    design.insert(design.end(), row_x.begin(), row_x.end());
    design.insert(design.end(), row_y.begin(), row_y.end());
  }
  const auto& backend = numeric::shared_backend(numeric::default_backend_kind());
  std::vector<double> solution;
  if (!backend.smallest_right_singular_vector(design, source.size() * 2, 9, &solution) ||
      solution.size() != 9)
    fail(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, "points",
         "homography estimation failed");
  Matrix h;
  std::copy(solution.begin(), solution.end(), h.begin());
  std::vector<Observation> observations;
  std::vector<Point> fitted;
  if (method == "gold_standard") {
    constexpr std::array<std::string_view, 6> covariance_names{"CovXX1", "CovYY1", "CovXY1",
                                                               "CovXX2", "CovYY2", "CovXY2"};
    const bool empty =
        std::all_of(covariances.begin(), covariances.end(), [](const auto& a) { return a.empty(); });
    std::array<std::vector<double>, 6> controls_values;
    if (!empty)
      for (int k = 0; k < 6; ++k) {
        if (covariances[k].size() != source.size())
          fail(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, covariance_names[k],
               "one covariance value per point required");
        controls_values[k] = values(covariances[k], covariance_names[k]);
      }
    observations.reserve(source.size());
    for (std::size_t i = 0; i < source.size(); ++i) {
      Symmetric2 c1, c2;
      if (!empty) {
        c1 = {controls_values[0][i], controls_values[2][i], controls_values[1][i]};
        c2 = {controls_values[3][i], controls_values[5][i], controls_values[4][i]};
      }
      for (auto c : {c1, c2}) {
        if (c.xx < 0. || c.yy < 0. || c.xx * c.yy < c.xy * c.xy)
          fail(core::Status::Unsupported, core::ErrorCode::UnsupportedValue, "Covariance",
               "indefinite measurement covariance is not supported");
      }
      if (c1.xx * c1.yy == c1.xy * c1.xy) {
        const double trace = c1.xx + c1.yy;
        c1 = {trace == 0. ? 1. : trace, 0., trace == 0. ? 1. : trace};
      }
      if (c2.xx * c2.yy == c2.xy * c2.xy) {
        const double trace = c2.xx + c2.yy;
        c2 = {trace == 0. ? 1. : trace, 0., trace == 0. ? 1. : trace};
      }
      c1 = {c1.xx * p.scale * p.scale, c1.xy * p.scale * p.scale, c1.yy * p.scale * p.scale};
      c2 = {c2.xx * q.scale * q.scale, c2.xy * q.scale * q.scale, c2.yy * q.scale * q.scale};
      observations.push_back({source[i], target[i], c1, c2, precision(c1), precision(c2)});
    }
    h = gold_standard(h, observations, &fitted);
  }
  double norm = 0.;
  for (auto value : h) norm += value * value;
  norm = std::sqrt(norm);
  Matrix unit = h;
  for (auto& value : unit) value /= norm;
  const auto raw = denormalize(unit, p, q);
  ProjectiveEstimate out{core::Tuple(std::vector<double>(raw.begin(), raw.end())), {}};
  if (method == "gold_standard") out.covariance = core::Tuple(covariance(observations, fitted, p, q, raw));
  return out;
}
}  // namespace mv::calibration
