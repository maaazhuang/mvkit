#pragma once
#include <xld/operators.hpp>
#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory_resource>
#include <numbers>
namespace mv::xld {
// atan2 已在 [-pi,pi] 内，不用 fmod，以保留带符号零及 NaN。
inline double positive_atan2(double y,double x) noexcept {
  const double value=std::atan2(y,x);
  return value<0 ? value+2*std::numbers::pi_v<double> : value;
}
}  // namespace mv::xld

namespace mv::xld::detail {
using namespace mv::core;
using mv::runtime::ExecutionContext;
inline constexpr double pi = std::numbers::pi, tau = 2 * pi;
[[noreturn]] inline void fail_requirement(const char* op, const char* parameter, const char* message) {
  throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op, parameter, -1, message);
}
inline void require(bool ok, const char* op, const char* parameter, const char* message) {
  if (!ok) fail_requirement(op,parameter,message);
}
inline void cardinality(bool ok, const char* op, const char* parameter) {
  if (!ok)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, op,
                    parameter, -1, "invalid object or control cardinality");
}
inline void coordinate(double v, const char* op) {
  require(std::abs(v) <= 0x1p30, op, "Coordinates",
          "finite coordinates in [-2^30,2^30] required");
}
inline void validate(const XldContour& c, const char* op) {
  for (auto p : c.points()) {
    coordinate(p.row, op);
    coordinate(p.col, op);
  }
}
inline double angle(double v) {
  v = std::fmod(v, tau);
  return v < 0 ? v + tau : v;
}
inline XldContours objects(std::span<const XldContour> contours) {
  std::vector<IconicValue> v;
  v.reserve(contours.size());
  for (auto c : contours) v.push_back(IconicValue::from_xld_contour(c));
  return XldContours{IconicTuple{std::move(v)}};
}
inline std::size_t work(const XldContours& c) {
  std::size_t w = 0;
  for (std::size_t i = 0; i < c.contour_count(); ++i)
    w += std::min(c.contour(i).size(), SIZE_MAX - w);
  return w;
}
inline std::size_t segments(const XldContour& c) {
  return c.size() < 2
             ? c.size()
             : c.size() - 1 +
                   (c.closed() && c.points().front() != c.points().back());
}
inline std::pair<Point2d, Point2d> segment(const XldContour& c, std::size_t i) {
  return {c.points()[i], c.points()[(i + 1) % c.size()]};
}
inline double point_distance(Point2d a, Point2d b) {
  return std::hypot(a.row - b.row, a.col - b.col);
}
inline double point_segment(Point2d p, Point2d a, Point2d b) {
  double dr = b.row - a.row, dc = b.col - a.col, den = dr * dr + dc * dc;
  double t =
      den == 0 ? 0
               : std::clamp(((p.row - a.row) * dr + (p.col - a.col) * dc) / den,
                            0., 1.);
  return std::hypot(p.row - std::lerp(a.row, b.row, t),
                    p.col - std::lerp(a.col, b.col, t));
}
struct Crossings {
  std::array<Point2d, 2> points{};
  int count = 0;
  bool overlap = false;
};
inline Crossings crossings(Point2d a, Point2d b, Point2d c, Point2d d) {
  const double x = b.col - a.col, y = b.row - a.row, u = d.col - c.col,
               v = d.row - c.row, dx = c.col - a.col, dy = c.row - a.row,
               det = x * v - y * u;
  Crossings result;
  if (det != 0) {
    double t = (dx * v - dy * u) / det, s = (dx * y - dy * x) / det;
    if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
      result.points[0] = {std::lerp(a.row, b.row, t),
                          std::lerp(a.col, b.col, t)};
      result.count = 1;
    }
    return result;
  }
  if (dx * y - dy * x != 0) return result;
  if (x == 0 && y == 0) {
    if (point_segment(a, c, d) == 0) {
      result.points[0] = a;
      result.count = 1;
    }
    return result;
  }
  double t0 = std::abs(x) >= std::abs(y) ? dx / x : dy / y,
         t1 = std::abs(x) >= std::abs(y) ? (d.col - a.col) / x
                                         : (d.row - a.row) / y;
  if (t0 > t1) std::swap(t0, t1);
  t0 = std::max(0., t0);
  t1 = std::min(1., t1);
  if (t0 <= t1) {
    result.points[0] = {std::lerp(a.row, b.row, t0),
                        std::lerp(a.col, b.col, t0)};
    result.count = 1;
    if (t1 > t0) {
      result.points[1] = {std::lerp(a.row, b.row, t1),
                          std::lerp(a.col, b.col, t1)};
      result.count = 2;
      result.overlap = true;
    }
  }
  return result;
}
template <std::size_t N>
bool solve(std::array<std::array<double, N>, N> a, std::array<double, N> b,
           std::array<double, N>& x) {
  double scale = 0;
  for (auto row : a)
    for (auto v : row) scale = std::max(scale, std::abs(v));
  if (scale == 0) return false;
  for (std::size_t k = 0; k < N; ++k) {
    auto pivot = k;
    for (auto i = k + 1; i < N; ++i)
      if (std::abs(a[i][k]) > std::abs(a[pivot][k])) pivot = i;
    if (std::abs(a[pivot][k]) < scale * 1e-13) return false;
    std::swap(a[k], a[pivot]);
    std::swap(b[k], b[pivot]);
    double den = a[k][k];
    for (auto j = k; j < N; ++j) a[k][j] /= den;
    b[k] /= den;
    for (std::size_t i = 0; i < N; ++i)
      if (i != k) {
        auto f = a[i][k];
        for (auto j = k; j < N; ++j) a[i][j] -= f * a[k][j];
        b[i] -= f * b[k];
      }
  }
  x = b;
  return true;
}
}  // namespace mv::xld::detail
