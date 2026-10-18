#include <xld/operators.hpp>
#include <core/exception.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory_resource>
#include <numeric>
#include <numbers>
#include <tuple>

namespace mv::xld {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
constexpr auto tau = 2 * std::numbers::pi;
constexpr std::array attributes{XldPointAttribute::Response,
                                XldPointAttribute::EdgeDirection,
                                XldPointAttribute::NormalAngle};
void check(bool ok, const char* op, const char* parameter,
           const char* message) {
  if (!ok)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    parameter, -1, message);
}
void bounded(double v, const char* op, const char* parameter) {
  check(std::isfinite(v) && std::abs(v) <= 0x1p30, op, parameter,
        "finite coordinates in [-2^30,2^30] required");
}
void validate(const XldContour& c, const char* op, bool attrs) {
  if(!c.geometry_summary().bounded)
    for (auto p : c.points()) {
      bounded(p.row, op, "Contours");
      bounded(p.col, op, "Contours");
    }
  if (attrs)
    for (auto a : attributes)
      for (auto v : c.point_attribute(a))
        check(std::isfinite(v), op, "Attributes", "finite attributes required");
}
struct Sample {
  Point2d p;
  std::array<double, 3> a{};
};
Sample sample(const XldContour& c, std::size_t i) {
  Sample s{c.points()[i]};
  for (std::size_t k = 0; k < 3; ++k)
    if (c.has_point_attribute(attributes[k]))
      s.a[k] = c.point_attribute(attributes[k])[i];
  return s;
}
Sample interpolate(const Sample& a, const Sample& b, double t) {
  if (t == 0) return a;
  if (t == 1) return b;
  Sample s{{std::lerp(a.p.row, b.p.row, t), std::lerp(a.p.col, b.p.col, t)}};
  s.a[0] = std::lerp(a.a[0], b.a[0], t);
  for (int k = 1; k < 3; ++k) {
    auto start = std::fmod(a.a[k], tau), end = std::fmod(b.a[k], tau);
    auto d = std::remainder(end - start, tau);
    if (d == -std::numbers::pi) d = std::numbers::pi;
    s.a[k] = std::fmod(start + t * d, tau);
    if (s.a[k] < 0) s.a[k] += tau;
  }
  return s;
}
struct Box {
  double r1, c1, r2, c2;
  bool contains(Point2d p) const {
    return p.row >= r1 && p.row <= r2 && p.col >= c1 && p.col <= c2;
  }
  double perimeter(Point2d p) const {
    const auto w = c2 - c1, h = r2 - r1;
    if (p.row == r1) return p.col - c1;
    if (p.col == c2) return w + p.row - r1;
    if (p.row == r2) return w + h + c2 - p.col;
    return 2 * w + h + r2 - p.row;
  }
  bool boundary(Point2d p) const {
    return p.row == r1 || p.row == r2 || p.col == c1 || p.col == c2;
  }
};
bool clip(Sample a, Sample b, Box box, Sample& first, Sample& last, double& lo,
          double& hi) {
  lo = 0;
  hi = 1;
  int entering = -1, leaving = -1;
  const std::array origin{a.p.row, a.p.col},
      delta{b.p.row - a.p.row, b.p.col - a.p.col};
  const std::array lower{box.r1, box.c1}, upper{box.r2, box.c2};
  for (int k = 0; k < 2; ++k) {
    if (delta[k] == 0) {
      if (origin[k] < lower[k] || origin[k] > upper[k]) return false;
      continue;
    }
    double x = (lower[k] - origin[k]) / delta[k],
           y = (upper[k] - origin[k]) / delta[k];
    int ex = 2 * k, ey = 2 * k + 1;
    if (x > y) {
      std::swap(x, y);
      std::swap(ex, ey);
    }
    if (x > lo) {
      lo = x;
      entering = ex;
    }
    if (y < hi) {
      hi = y;
      leaving = ey;
    }
    if (lo > hi) return false;
  }
  first = interpolate(a, b, lo);
  last = interpolate(a, b, hi);
  const auto snap = [&](Sample& s, int side) {
    if (side == 0) s.p.row = box.r1;
    if (side == 1) s.p.row = box.r2;
    if (side == 2) s.p.col = box.c1;
    if (side == 3) s.p.col = box.c2;
    s.p.row = std::clamp(s.p.row, box.r1, box.r2);
    s.p.col = std::clamp(s.p.col, box.c1, box.c2);
  };
  snap(first, entering);
  snap(last, leaving);
  return true;
}
XldContour materialize(std::span<const Sample> samples,
                       const XldContour& source, bool closed, bool retain,
                       std::pmr::memory_resource* memory) {
  std::pmr::vector<Point2d> points{memory};
  std::pmr::vector<double> response{memory}, edge{memory}, normal{memory};
  const std::array out{&response, &edge, &normal};
  for (const auto& s : samples) {
    points.push_back(s.p);
    for (std::size_t k = 0; k < 3; ++k)
      if (retain && source.has_point_attribute(attributes[k]))
        out[k]->push_back(s.a[k]);
  }
  auto approximation = source.approximation() == ContourApproximation::Line
                           ? ContourApproximation::Line
                           : ContourApproximation::Unknown;
  return XldContour::from_points_and_attributes(points, closed, approximation,
                                                response, edge, normal);
}
double cross(Point2d a, Point2d b, Point2d c) {
  return (b.col - a.col) * (c.row - a.row) - (b.row - a.row) * (c.col - a.col);
}
bool on_segment(Point2d p, Point2d a, Point2d b) {
  return cross(a, b, p) == 0 && p.row >= std::min(a.row, b.row) &&
         p.row <= std::max(a.row, b.row) && p.col >= std::min(a.col, b.col) &&
         p.col <= std::max(a.col, b.col);
}
bool intersect(Point2d a, Point2d b, Point2d c, Point2d d) {
  const auto x = cross(a, b, c), y = cross(a, b, d), z = cross(c, d, a),
             w = cross(c, d, b);
  return ((x < 0) != (y < 0) && x != 0 && y != 0 && (z < 0) != (w < 0) &&
          z != 0 && w != 0) ||
         on_segment(c, a, b) || on_segment(d, a, b) || on_segment(a, c, d) ||
         on_segment(b, c, d);
}
bool inside_polygon(Point2d p, std::span<const Sample> polygon) {
  bool inside = false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    auto a = polygon[i].p, b = polygon[(i + 1) % polygon.size()].p;
    if (on_segment(p, a, b))
      return false;  // 共享边界归原轮廓边所有。
    if ((a.row > p.row) != (b.row > p.row) &&
        p.col < (b.col - a.col) * (p.row - a.row) / (b.row - a.row) + a.col)
      inside = !inside;
  }
  return inside;
}
bool strictly_convex(std::span<const Sample> polygon) {
  const auto n = polygon.size();
  const auto first = cross(polygon[0].p, polygon[1].p, polygon[2].p);
  if (first == 0)
    return false;
  const bool positive = first > 0;
  const auto half = [](Point2d d) {
    return d.row > 0 || (d.row == 0 && d.col >= 0) ? 0 : 1;
  };
  std::size_t wraps = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const auto a = polygon[i].p, b = polygon[(i + 1) % n].p,
               c = polygon[(i + 2) % n].p;
    const auto turn = cross(a, b, c);
    if (turn == 0 || (turn > 0) != positive)
      return false;
    const int from = half({b.row - a.row, b.col - a.col}),
              to = half({c.row - b.row, c.col - b.col});
    wraps += positive ? from > to : from < to;
  }
  // 同向转角还需恰好绕一周；五角星等多绕自交输入不能走凸轮廓快路径。
  return wraps == 1;
}
void crop_closed(const XldContour &source, Box box,
                 std::pmr::vector<XldContour> &result,
                 std::pmr::memory_resource *memory) {
  const auto& summary=source.geometry_summary();
  if(source.closed() && summary.strictly_convex && summary.row_min>=box.r1 && summary.row_max<=box.r2 &&
      summary.column_min>=box.c1 && summary.column_max<=box.c2) {
    // 严格凸性已经证明简单且非退化；完整包含时原闭合轮廓即为结果。
    result.push_back(source);return;
  }
  std::pmr::vector<Sample> polygon{memory};
  const auto points = source.points();
  const std::array attr{source.point_attribute(attributes[0]),
                        source.point_attribute(attributes[1]),
                        source.point_attribute(attributes[2])};
  for (std::size_t i = 0; i < source.size(); ++i)
    if (polygon.empty() || polygon.back().p != points[i]) {
      Sample value{points[i]};
      for (std::size_t k = 0; k < 3; ++k)
        if (!attr[k].empty())
          value.a[k] = attr[k][i];
      polygon.push_back(value);
    }
  if (polygon.size() > 1 && polygon.front().p == polygon.back().p)
    polygon.pop_back();
  const auto n = polygon.size();
  if (n < 3)
    return;
  double area = 0;
  struct Bounds {
    double r1, r2, c1, c2;
    std::size_t index;
  };
  std::pmr::vector<Bounds> bounds{memory};
  bounds.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    area += cross(polygon[0].p, polygon[i].p, polygon[(i + 1) % n].p);
    const auto a = polygon[i].p, b = polygon[(i + 1) % n].p;
    bounds.push_back({std::min(a.row, b.row), std::max(a.row, b.row),
                      std::min(a.col, b.col), std::max(a.col, b.col), i});
  }
  // 行区间扫掠只筛选可能相交的边；精确相交判定、邻边豁免及错误行为不变。
  if (!strictly_convex(polygon)) {
    std::sort(bounds.begin(), bounds.end(), [](const auto &a, const auto &b) {
      return std::pair{a.r1, a.index} < std::pair{b.r1, b.index};
    });
    for (std::size_t a = 0; a < n; ++a) {
      const auto &first = bounds[a];
      for (std::size_t b = a + 1; b < n && bounds[b].r1 <= first.r2; ++b) {
        const auto &second = bounds[b];
        const auto i = first.index, j = second.index;
        if ((i + 1) % n == j || (j + 1) % n == i || first.c2 < second.c1 ||
            second.c2 < first.c1)
          continue;
        check(!intersect(polygon[i].p, polygon[(i + 1) % n].p, polygon[j].p,
                         polygon[(j + 1) % n].p),
              "crop_contours_xld", "Contours",
              "closed reconstruction requires a simple polygon");
      }
    }
  }
  if (area == 0 || box.r1 == box.r2 || box.c1 == box.c2)
    return;
  if (std::all_of(polygon.begin(), polygon.end(),
                  [&](auto s) { return box.contains(s.p); })) {
    if (source.closed())
      result.push_back(source);
    else
      result.push_back(XldContour::from_points_and_attributes(
          source.points(), true, source.approximation(),
          source.point_attribute(XldPointAttribute::Response),
          source.point_attribute(XldPointAttribute::EdgeDirection),
          source.point_attribute(XldPointAttribute::NormalAngle)));
    return;
  }
  struct Edge {
    Sample a, b;
    bool used = false;
  };
  std::pmr::vector<Edge> edges{memory};
  std::pmr::vector<Sample> boundary{memory};
  for (auto p : {Point2d{box.r1, box.c1}, Point2d{box.r1, box.c2},
                 Point2d{box.r2, box.c2}, Point2d{box.r2, box.c1}})
    boundary.push_back({p});
  for (std::size_t i = 0; i < n; ++i) {
    Sample a, b;
    double lo, hi;
    if (!clip(polygon[i], polygon[(i + 1) % n], box, a, b, lo, hi)) continue;
    if (box.boundary(a.p)) boundary.push_back(a);
    if (box.boundary(b.p)) boundary.push_back(b);
    if (a.p == b.p) continue;
    // 位于框上的多边形边仅在内部朝向框内时贡献。
    bool shared = false, clockwise = false;
    if (a.p.row == box.r1 && b.p.row == box.r1) {
      shared = true;
      clockwise = b.p.col > a.p.col;
    }
    if (a.p.col == box.c2 && b.p.col == box.c2) {
      shared = true;
      clockwise = b.p.row > a.p.row;
    }
    if (a.p.row == box.r2 && b.p.row == box.r2) {
      shared = true;
      clockwise = b.p.col < a.p.col;
    }
    if (a.p.col == box.c1 && b.p.col == box.c1) {
      shared = true;
      clockwise = b.p.row < a.p.row;
    }
    if (!shared || clockwise == (area > 0)) edges.push_back({a, b});
  }
  std::sort(boundary.begin(), boundary.end(), [&](auto a, auto b) {
    return box.perimeter(a.p) < box.perimeter(b.p);
  });
  boundary.erase(std::unique(boundary.begin(), boundary.end(),
                             [](auto a, auto b) { return a.p == b.p; }),
                 boundary.end());
  bool synthetic = false;
  for (std::size_t i = 0; i < boundary.size(); ++i) {
    auto a = boundary[i], b = boundary[(i + 1) % boundary.size()];
    if (inside_polygon(
            {std::midpoint(a.p.row, b.p.row), std::midpoint(a.p.col, b.p.col)},
            polygon)) {
      if (area < 0) std::swap(a, b);
      edges.push_back({a, b});
      synthetic = true;
    }
  }
  std::pmr::vector<Sample> path{memory};
  for (std::size_t i = 0; i < edges.size(); ++i)
    if (!edges[i].used) {
      path.clear();
      auto index = i;
      const auto start = edges[i].a.p;
      while (true) {
        auto& e = edges[index];
        e.used = true;
        path.push_back(e.a);
        if (e.b.p == start) break;
        auto next = edges.size();
        for (std::size_t j = 0; j < edges.size(); ++j)
          if (!edges[j].used && edges[j].a.p == e.b.p) {
            next = j;
            break;
          }
        check(next < edges.size(), "crop_contours_xld", "Contours",
              "numerically ambiguous boundary topology");
        index = next;
      }
      if (path.size() >= 3)
        result.push_back(materialize(path, source, true, !synthetic, memory));
    }
}
XldContours crop_one(const XldContour& c, Box box, bool close) {
  validate(c, "crop_contours_xld", true);
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  std::pmr::vector<XldContour> pieces{memory};
  const bool cyclic =
      c.closed() || (c.size() > 1 && c.points().front() == c.points().back());
  if (close && cyclic)
    crop_closed(c, box, pieces, memory);
  else if (!c.empty()) {
    if (std::all_of(c.points().begin(), c.points().end(),
                    [&](auto p) { return box.contains(p); }))
      pieces.push_back(c);
    else {
      std::pmr::vector<Sample> path{memory};
      const auto flush = [&] {
        if (!path.empty()) {
          pieces.push_back(materialize(path, c, false, true, memory));
          path.clear();
        }
      };
      const auto segments =
          c.size() - 1 +
          (c.closed() && c.size() > 1 && c.points().front() != c.points().back()
               ? 1
               : 0);
      for (std::size_t i = 0; i < segments; ++i) {
        Sample a, b;
        double lo, hi;
        if (!clip(sample(c, i % c.size()), sample(c, (i + 1) % c.size()), box,
                  a, b, lo, hi)) {
          flush();
          continue;
        }
        if (lo > 0) flush();
        if (path.empty()) path.push_back(a);
        if (a.p != b.p) path.push_back(b);
        if (hi < 1) flush();
      }
      flush();
      // 闭合输入可能有一条存活弧跨越存储首点。
      if (cyclic && pieces.size() > 1 && box.contains(c.points().front())) {
        auto first = pieces.front(), last = pieces.back();
        if (last.points().back() == first.points().front()) {
          path.clear();
          for (std::size_t i = 0; i < last.size(); ++i)
            path.push_back(sample(last, i));
          for (std::size_t i = 1; i < first.size(); ++i)
            path.push_back(sample(first, i));
          pieces.front() = materialize(path, c, false, true, memory);
          pieces.pop_back();
        }
      }
    }
  }
  std::vector<IconicValue> values;
  for (auto& p : pieces)
    values.push_back(IconicValue::from_xld_contour(std::move(p)));
  return XldContours{IconicTuple{std::move(values)}};
}
std::size_t work_size(const XldContours& c) {
  std::size_t work = 0;
  for (std::size_t i = 0; i < c.contour_count(); ++i)
    work += std::min(c.contour(i).size(), SIZE_MAX - work);
  return work;
}
}  // namespace
mv::core::XldContours crop_contours_xld(const mv::core::XldContours& c,
                                        double r1, double c1, double r2,
                                        double c2, bool close) {
  for (auto v : {r1, c1, r2, c2}) bounded(v, "crop_contours_xld", "Rectangle");
  check(r1 <= r2 && c1 <= c2, "crop_contours_xld", "Rectangle",
        "ordered rectangle bounds required");
  ExecutionContext context;
  std::pmr::vector<XldContours> result(c.contour_count(),
                                       context.scratch_resource());
  ExecutionContext::for_each_partition(
      result.size(), work_size(c), 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          result[i] = crop_one(c.contour(i), {r1, c1, r2, c2}, close);
      });
  std::vector<IconicValue> values;
  for (auto& group : result)
    for (std::size_t i = 0; i < group.contour_count(); ++i)
      values.push_back(IconicValue::from_xld_contour(group.contour(i)));
  return XldContours{IconicTuple{std::move(values)}};
}
mv::core::XldContours sort_contours_xld(const mv::core::XldContours& c,
                                        XldSortMode mode, XldSortOrder order,
                                        XldSortAxis axis) {
  constexpr auto op = "sort_contours_xld";
  check(mode >= XldSortMode::UpperLeft && mode <= XldSortMode::Character, op,
        "SortMode", "invalid mode");
  check(order == XldSortOrder::Ascending || order == XldSortOrder::Descending,
        op, "Order", "invalid order");
  check(axis == XldSortAxis::Row || axis == XldSortAxis::Column, op, "RowOrCol",
        "invalid axis");
  if(c.contour_count()==1){validate(c.contour(0),op,false);return c;}
  struct Key {
    double primary = 0, secondary = 0, upper = 0;
    std::size_t index = 0, group = 0;
    bool empty = true;
  };
  ExecutionContext context;
  std::pmr::vector<Key> keys(c.contour_count(), context.scratch_resource());
  ExecutionContext::for_each_partition(
      keys.size(), work_size(c), 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) {
          const auto& contour = c.contour(i);
          validate(contour, op, false);
          auto& k = keys[i];
          k.index = i;
          if (contour.empty()) continue;
          k.empty = false;
          const auto& summary=contour.geometry_summary();
          const Point2d lo{summary.row_min,summary.column_min},hi{summary.row_max,summary.column_max};
          double r = (mode == XldSortMode::LowerLeft ||
                      mode == XldSortMode::LowerRight)
                         ? hi.row
                         : lo.row;
          double col = (mode == XldSortMode::UpperRight ||
                        mode == XldSortMode::LowerRight)
                           ? hi.col
                           : lo.col;
          k.primary = axis == XldSortAxis::Row ? r : col;
          k.secondary = axis == XldSortAxis::Row ? col : r;
          k.upper = axis == XldSortAxis::Row ? hi.row : hi.col;
        }
      });
  std::sort(keys.begin(), keys.end(), [](auto a, auto b) {
    return std::tuple{a.empty, a.primary, a.secondary, a.index} <
           std::tuple{b.empty, b.primary, b.secondary, b.index};
  });
  if (mode == XldSortMode::Character) {
    double end = -INFINITY;
    std::size_t group = 0;
    for (auto& k : keys)
      if (!k.empty) {
        if (k.primary > end) {
          ++group;
          end = k.upper;
        } else
          end = std::max(end, k.upper);
        k.group = group;
      }
  }
  const bool descending = order == XldSortOrder::Descending;
  std::sort(keys.begin(), keys.end(), [&](auto a, auto b) {
    if (a.empty != b.empty) return !a.empty;
    if (a.empty) return a.index < b.index;
    const auto ka = std::tuple{
        mode == XldSortMode::Character ? double(a.group) : a.primary,
        a.secondary, mode == XldSortMode::Character ? a.primary : 0.};
    const auto kb = std::tuple{
        mode == XldSortMode::Character ? double(b.group) : b.primary,
        b.secondary, mode == XldSortMode::Character ? b.primary : 0.};
    if (ka == kb) return a.index < b.index;
    return descending ? ka > kb : ka < kb;
  });
  std::vector<IconicValue> result;
  for (auto k : keys)
    result.push_back(IconicValue::from_xld_contour(c.contour(k.index)));
  return XldContours{IconicTuple{std::move(result)}};
}
}  // namespace mv::xld
