#pragma once
#include <xld/contour_utils.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory_resource>
#include <span>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
namespace mv::xld::detail {
inline std::uint64_t cell_key(std::int32_t row, std::int32_t col) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32) |
         static_cast<std::uint32_t>(col);
}
inline std::int32_t cell_coord(double value, double origin, double cell) {
  return static_cast<std::int32_t>(std::floor((value - origin) / cell));
}
inline std::pmr::vector<Point2d> convex_hull(std::span<const Point2d> input,
                                             std::pmr::memory_resource* memory) {
  if (input.size() <= 2) return {input.begin(), input.end(), memory};
  std::pmr::vector<Point2d> points(input.begin(), input.end(), memory);
  const auto less=[](Point2d a,Point2d b){return std::tie(a.col,a.row)<std::tie(b.col,b.row);};
  if(!std::is_sorted(points.begin(),points.end(),less))std::sort(points.begin(),points.end(),less);
  points.erase(std::unique(points.begin(), points.end(),
                           [](Point2d a, Point2d b) {
                             return a.row == b.row && a.col == b.col;
                           }),
               points.end());
  if (points.size() <= 2) return points;
  const auto cross = [](Point2d o, Point2d a, Point2d b) {
    return (a.col - o.col) * (b.row - o.row) - (a.row - o.row) * (b.col - o.col);
  };
  std::pmr::vector<Point2d> hull{memory};
  hull.reserve(points.size() + 1);
  for (int pass = 0; pass < 2; ++pass) {
    const auto start = hull.size();
    for (auto p : points) {
      while (hull.size() >= start + 2 &&
             cross(hull[hull.size() - 2], hull.back(), p) <= 0)
        hull.pop_back();
      hull.push_back(p);
    }
    hull.pop_back();
    std::reverse(points.begin(), points.end());
  }
  return hull;
}
inline double max_point_point(std::span<const Point2d> a,
                              std::span<const Point2d> b,
                              std::pmr::memory_resource* memory) {
  const auto ha = convex_hull(a, memory);
  const auto hb = convex_hull(b, memory);
  double high = 0;
  for (auto p : ha)
    for (auto q : hb) high = std::max(high, point_distance(p, q));
  return high;
}
inline std::array<double,4> point_bounds(std::span<const Point2d> p) {
  std::array<double,4> b{p[0].row,p[0].row,p[0].col,p[0].col};
  for(auto q:p){b[0]=std::min(b[0],q.row);b[1]=std::max(b[1],q.row);b[2]=std::min(b[2],q.col);b[3]=std::max(b[3],q.col);}return b;
}
inline double bounds_distance(const std::array<double,4>& a,const std::array<double,4>& b) {
  return std::hypot(std::max({b[0]-a[1],a[0]-b[1],0.}),std::max({b[2]-a[3],a[2]-b[3],0.}));
}
inline double min_point_point(std::span<const Point2d> a,
                              std::span<const Point2d> b) {
  if (a.empty() || b.empty()) return INFINITY;
  double best = point_distance(a.front(), b.front());
  best = std::min(best, point_distance(a.front(), b.back()));
  best = std::min(best, point_distance(a.back(), b.front()));
  best = std::min(best, point_distance(a.back(), b.back()));
  if (best == 0) return 0;
  if(bounds_distance(point_bounds(a),point_bounds(b))>=best)return best;
  if (a.size() * b.size() <= 4096) {
    for (auto p : a)
      for (auto q : b) {
        best = std::min(best, point_distance(p, q));
        if (best == 0) return 0;
      }
    return best;
  }
  double row_min = a.front().row, row_max = a.front().row;
  double col_min = a.front().col, col_max = a.front().col;
  auto stretch = [&](std::span<const Point2d> pts) {
    for (auto p : pts) {
      row_min = std::min(row_min, p.row);
      row_max = std::max(row_max, p.row);
      col_min = std::min(col_min, p.col);
      col_max = std::max(col_max, p.col);
    }
  };
  stretch(a);
  stretch(b);
  const double span = std::max({row_max - row_min, col_max - col_min, 1.0});
  const double cell = std::max({best, span / 1024.0, 1e-9});
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins;
  bins.reserve(b.size() * 2);
  for (std::size_t i = 0; i < b.size(); ++i) {
    bins[cell_key(cell_coord(b[i].row, row_min, cell),
                  cell_coord(b[i].col, col_min, cell))]
        .push_back(i);
  }
  for (auto p : a) {
    const auto r0 = cell_coord(p.row - best, row_min, cell);
    const auto r1 = cell_coord(p.row + best, row_min, cell);
    const auto c0 = cell_coord(p.col - best, col_min, cell);
    const auto c1 = cell_coord(p.col + best, col_min, cell);
    for (auto r = r0; r <= r1; ++r) {
      for (auto c = c0; c <= c1; ++c) {
        const auto it = bins.find(cell_key(r, c));
        if (it == bins.end()) continue;
        for (auto i : it->second) {
          best = std::min(best, point_distance(p, b[i]));
          if (best == 0) return 0;
        }
      }
    }
  }
  return best;
}
struct SegmentBox {
  Point2d a, b;
  double r0, r1, c0, c1;
};
inline SegmentBox make_segment(Point2d a, Point2d b) {
  return {a,
          b,
          std::min(a.row, b.row),
          std::max(a.row, b.row),
          std::min(a.col, b.col),
          std::max(a.col, b.col)};
}
inline double aabb_min(const SegmentBox& a, const SegmentBox& b) {
  double dr = 0;
  if (a.r1 < b.r0)
    dr = b.r0 - a.r1;
  else if (b.r1 < a.r0)
    dr = a.r0 - b.r1;
  double dc = 0;
  if (a.c1 < b.c0)
    dc = b.c0 - a.c1;
  else if (b.c1 < a.c0)
    dc = a.c0 - b.c1;
  return std::hypot(dr, dc);
}
inline std::pmr::vector<SegmentBox> contour_segments(
    const XldContour& contour, std::pmr::memory_resource* memory) {
  std::pmr::vector<SegmentBox> out{memory};
  const auto n = segments(contour);
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    auto [p, q] = segment(contour, i);
    out.push_back(make_segment(p, q));
  }
  return out;
}
inline double min_segment_segment(std::span<const SegmentBox> x,
                                  std::span<const SegmentBox> y, double start,
                                  std::pmr::memory_resource* memory) {
  double best = start;
  if (x.empty() || y.empty()) return best;
  const auto box=[](std::span<const SegmentBox> segs){std::array<double,4> b{segs[0].r0,segs[0].r1,segs[0].c0,segs[0].c1};for(const auto& s:segs){b[0]=std::min(b[0],s.r0);b[1]=std::max(b[1],s.r1);b[2]=std::min(b[2],s.c0);b[3]=std::max(b[3],s.c1);}return b;};
  if(bounds_distance(box(x),box(y))>=best)return best;

  auto consider = [&](const SegmentBox& a, const SegmentBox& b) {
    if (aabb_min(a, b) >= best) return;
    const auto hit = crossings(a.a, a.b, b.a, b.b);
    if (hit.count) {
      best = 0;
      return;
    }
    best = std::min({best, point_segment(a.a, b.a, b.b),
                     point_segment(a.b, b.a, b.b), point_segment(b.a, a.a, a.b),
                     point_segment(b.b, a.a, a.b)});
  };
  if (x.size() * y.size() <= 4096) {
    for (const auto& a : x) {
      for (const auto& b : y) {
        consider(a, b);
        if (best == 0) return 0;
      }
    }
    return best;
  }
  double row_min = x.front().r0, row_max = x.front().r1;
  double col_min = x.front().c0, col_max = x.front().c1;
  auto stretch = [&](std::span<const SegmentBox> segs) {
    for (const auto& s : segs) {
      row_min = std::min(row_min, s.r0);
      row_max = std::max(row_max, s.r1);
      col_min = std::min(col_min, s.c0);
      col_max = std::max(col_max, s.c1);
    }
  };
  stretch(x);
  stretch(y);
  const double span = std::max({row_max - row_min, col_max - col_min, 1.0});
  const double cell = std::max({best, span / 1024.0, 1e-9});
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins;
  auto insert = [&](std::size_t i, const SegmentBox& s) {
    const auto r0 = cell_coord(s.r0, row_min, cell);
    const auto r1 = cell_coord(s.r1, row_min, cell);
    const auto c0 = cell_coord(s.c0, col_min, cell);
    const auto c1 = cell_coord(s.c1, col_min, cell);
    for (auto r = r0; r <= r1; ++r)
      for (auto c = c0; c <= c1; ++c) bins[cell_key(r, c)].push_back(i);
  };
  for (std::size_t i = 0; i < y.size(); ++i) insert(i, y[i]);
  std::pmr::vector<std::size_t> cand{memory};
  for (const auto& a : x) {
    cand.clear();
    const auto r0 = cell_coord(a.r0 - best, row_min, cell);
    const auto r1 = cell_coord(a.r1 + best, row_min, cell);
    const auto c0 = cell_coord(a.c0 - best, col_min, cell);
    const auto c1 = cell_coord(a.c1 + best, col_min, cell);
    for (auto r = r0; r <= r1; ++r) {
      for (auto c = c0; c <= c1; ++c) {
        const auto it = bins.find(cell_key(r, c));
        if (it == bins.end()) continue;
        cand.insert(cand.end(), it->second.begin(), it->second.end());
      }
    }
    std::sort(cand.begin(), cand.end());
    cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
    for (auto j : cand) {
      consider(a, y[j]);
      if (best == 0) return 0;
    }
  }
  return best;
}
template <class Fn>
void for_each_indexed_pair(std::span<const SegmentBox> x,
                           std::span<const SegmentBox> y, Fn&& fn,
                           std::pmr::memory_resource* memory) {
  if (x.empty() || y.empty()) return;
  if (x.size() * y.size() <= 4096) {
    for (std::size_t i = 0; i < x.size(); ++i)
      for (std::size_t j = 0; j < y.size(); ++j) fn(i, j);
    return;
  }
  double row_min = x.front().r0, row_max = x.front().r1;
  double col_min = x.front().c0, col_max = x.front().c1;
  auto stretch = [&](std::span<const SegmentBox> segs) {
    for (const auto& s : segs) {
      row_min = std::min(row_min, s.r0);
      row_max = std::max(row_max, s.r1);
      col_min = std::min(col_min, s.c0);
      col_max = std::max(col_max, s.c1);
    }
  };
  stretch(x);
  stretch(y);
  const double span = std::max({row_max - row_min, col_max - col_min, 1.0});
  const double cell = std::max(span / 64.0, 1e-9);
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins;
  for (std::size_t j = 0; j < y.size(); ++j) {
    const auto r0 = cell_coord(y[j].r0, row_min, cell);
    const auto r1 = cell_coord(y[j].r1, row_min, cell);
    const auto c0 = cell_coord(y[j].c0, col_min, cell);
    const auto c1 = cell_coord(y[j].c1, col_min, cell);
    for (auto r = r0; r <= r1; ++r)
      for (auto c = c0; c <= c1; ++c) bins[cell_key(r, c)].push_back(j);
  }
  std::pmr::vector<std::size_t> cand{memory};
  for (std::size_t i = 0; i < x.size(); ++i) {
    cand.clear();
    const auto r0 = cell_coord(x[i].r0, row_min, cell);
    const auto r1 = cell_coord(x[i].r1, row_min, cell);
    const auto c0 = cell_coord(x[i].c0, col_min, cell);
    const auto c1 = cell_coord(x[i].c1, col_min, cell);
    for (auto r = r0; r <= r1; ++r) {
      for (auto c = c0; c <= c1; ++c) {
        const auto it = bins.find(cell_key(r, c));
        if (it == bins.end()) continue;
        cand.insert(cand.end(), it->second.begin(), it->second.end());
      }
    }
    std::sort(cand.begin(), cand.end());
    cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
    for (auto j : cand) fn(i, j);
  }
}
}  // namespace mv::xld::detail
