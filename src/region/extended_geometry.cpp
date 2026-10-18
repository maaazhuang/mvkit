#include <array>
#include <cstdint>
#include <region/extended_common.hpp>

namespace mv::regionproc {
namespace {
using namespace extended;
struct Point {
  double x, y;
};
double cross(Point a, Point b, Point c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
Region convex(const Region& region, bool halcon_boundary=false) {
  if (region.empty()) return region;
  const auto bounds = *region.bounding_box();
  // 规范游程元数据已含 geometry_bounds 检查的极值。
  check(bounds.row_min >= -0x40000000LL && bounds.row_max <= 0x40000000LL &&
            bounds.col_min >= -0x40000000LL && bounds.col_max <= 0x40000000LL,
        "shape_trans", "Region", "floating geometry requires coordinates within +/-2^30");
  const auto width = std::uint64_t(bounds.col_max) - std::uint64_t(bounds.col_min) + 1;
  const auto height = std::uint64_t(bounds.row_max) - std::uint64_t(bounds.row_min) + 1;
  // 四个极端像素证明凸包为整个框；仅在原 Double 方向谓词精确的范围内使用捷径。
  if (width <= (1U << 24) + 1 && height <= (1U << 24) + 1 &&
      region.contains(bounds.row_min, bounds.col_min) &&
      region.contains(bounds.row_min, bounds.col_max) &&
      region.contains(bounds.row_max, bounds.col_min) &&
      region.contains(bounds.row_max, bounds.col_max)) {
    if (region.area() == width * height) return region;
    return Region::rectangle(bounds);
  }
  ExecutionContext context;
  std::pmr::vector<Point> points{context.scratch_resource()}, hull{context.scratch_resource()};
  const auto runs = region.runs();
  const auto box = *region.bounding_box();
  // 此范围内原方向谓词的乘积/差精确，共线内部行点不影响凸包。
  const bool compact_rows = std::uint64_t(box.col_max) - std::uint64_t(box.col_min) <= (1U << 24) &&
                            std::uint64_t(box.row_max) - std::uint64_t(box.row_min) <= (1U << 24);
  if (compact_rows) {
    const auto row_bound = std::min(
        runs.size(),
        static_cast<std::size_t>(std::uint64_t(box.row_max) - std::uint64_t(box.row_min) + 1));
    points.reserve(row_bound * 2);
    for (std::size_t begin = 0; begin < runs.size();) {
      std::size_t end = begin + 1;
      while (end < runs.size() && runs[end].row == runs[begin].row) ++end;
      points.push_back({double(runs[begin].col_begin), double(runs[begin].row)});
      points.push_back({double(runs[end - 1].col_end), double(runs[begin].row)});
      begin = end;
    }
  } else {
    for (const auto& r : runs) {
      points.push_back({double(r.col_begin), double(r.row)});
      points.push_back({double(r.col_end), double(r.row)});
    }
  }
  std::sort(points.begin(), points.end(),
            [](Point a, Point b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  points.erase(std::unique(points.begin(), points.end(),
                           [](Point a, Point b) { return a.x == b.x && a.y == b.y; }),
               points.end());
  if (points.size() == 1) return region;
  for (auto p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
    hull.push_back(p);
  }
  const auto lower = hull.size();
  for (std::size_t i = points.size() - 1; i > 0; --i) {
    auto p = points[i - 1];
    while (hull.size() > lower && cross(hull[hull.size() - 2], hull.back(), p) <= 0)
      hull.pop_back();
    hull.push_back(p);
  }
  hull.pop_back();
  Tuple rows, columns;
  for (auto p : hull) {
    rows.append(p.y);
    columns.append(p.x);
  }
  if (hull.size() == 2) {
    rows.append(hull[0].y);
    columns.append(hull[0].x);
  }
  // 即使浮点边界舍入，并集仍保留全部原像素。
  auto result=gen_region_polygon_filled(rows, columns).united(region);
  if(halcon_boundary) {
    geometry_bounds(region,"shape_trans");
    std::vector<RegionRun> boundary;
    for(std::size_t i=0;i<hull.size();++i) {
      auto x0=static_cast<std::int64_t>(hull[i].x),y0=static_cast<std::int64_t>(hull[i].y);
      auto x1=static_cast<std::int64_t>(hull[(i+1)%hull.size()].x),y1=static_cast<std::int64_t>(hull[(i+1)%hull.size()].y);
      if(y0==y1){boundary.push_back({y0,std::min(x0,x1),std::max(x0,x1)});continue;}
      if(y0>y1){std::swap(x0,x1);std::swap(y0,y1);}
      const auto dx=std::abs(x1-x0),dy=-std::abs(y1-y0),sx=x0<x1?std::int64_t{1}:std::int64_t{-1};
      auto error=dx+dy;
      for(;;) {
        boundary.push_back({y0,x0,x0});if(x0==x1&&y0==y1)break;
        const auto twice=2*error;
        if(twice>=dy){error+=dy;x0+=sx;}if(twice<=dx){error+=dx;++y0;}
      }
    }
    result=result.united(Region::from_runs(boundary));
  }
  return result;
}
}  // namespace
mv::core::Regions shape_trans(const mv::core::Regions& regions, ShapeTransform mode, bool halcon_boundary) {
  check(mode <= ShapeTransform::Rectangle1, "shape_trans", "Mode", "invalid transform");
  return map(regions, [&](const Region& r) {
    if (mode == ShapeTransform::Convex) return convex(r,halcon_boundary);
    const auto b = r.bounding_box();
    return b ? Region::rectangle(*b) : r;
  });
}
mv::core::Regions affine_trans_region(const mv::core::Regions& regions,
                                      const mv::core::Tuple& matrix) {
  constexpr auto op = "affine_trans_region";
  if (matrix.size() != 6)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, op, "Matrix", -1,
                    "six forward affine coefficients required");
  std::array<double, 6> m;
  for (std::size_t i = 0; i < 6; ++i) {
    m[i] = matrix.as_real64(i);
    check(std::isfinite(m[i]), op, "Matrix", "finite coefficients required");
  }
  const double det = m[0] * m[4] - m[1] * m[3];
  check(std::isfinite(det) && det != 0, op, "Matrix", "nonsingular finite determinant required");
  std::array<double, 4> inv{m[4] / det, -m[1] / det, -m[3] / det, m[0] / det};
  for (double v : inv) check(std::isfinite(v), op, "Matrix", "inverse overflow");
  return map(regions, [&](const Region& r) {
    if (r.empty()) return r;
    geometry_bounds(r, op);
    const auto b = *r.bounding_box();
    double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
    for (double x : {double(b.col_min) - .5, double(b.col_max) + .5})
      for (double y : {double(b.row_min) - .5, double(b.row_max) + .5}) {
        const double xx = m[0] * x + m[1] * y + m[2], yy = m[3] * x + m[4] * y + m[5];
        check(std::isfinite(xx) && std::isfinite(yy) && std::abs(xx) <= 0x1p30 &&
                  std::abs(yy) <= 0x1p30,
              op, "Matrix", "transformed support exceeds +/-2^30");
        xmin = std::min(xmin, xx);
        xmax = std::max(xmax, xx);
        ymin = std::min(ymin, yy);
        ymax = std::max(ymax, yy);
      }
    // 保留原矩阵、输入坐标及变换支撑检查；整数平移的 inverse nearest
    // 与规范输入游程一一对应，无需占用位图或逐像素遍历。
    if (m[0] == 1. && m[1] == 0. && m[3] == 0. && m[4] == 1. && std::trunc(m[2]) == m[2] &&
        std::trunc(m[5]) == m[5] && std::abs(m[2]) <= 0x1p31 && std::abs(m[5]) <= 0x1p31) {
      return r.shifted(static_cast<std::int64_t>(m[5]), static_cast<std::int64_t>(m[2]));
    }
    const auto x0 = static_cast<std::int64_t>(std::ceil(xmin)),
               x1 = static_cast<std::int64_t>(std::floor(xmax));
    const auto y0 = static_cast<std::int64_t>(std::ceil(ymin)),
               y1 = static_cast<std::int64_t>(std::floor(ymax));
    ExecutionContext context;
    std::pmr::vector<RegionRun> runs{context.scratch_resource()};
    const auto src_w = static_cast<std::uint64_t>(b.col_max - b.col_min) + 1;
    const auto src_h = static_cast<std::uint64_t>(b.row_max - b.row_min) + 1;
    const bool use_mask = src_h > 0 && src_w <= kOccupancyLimit / src_h;
    std::pmr::vector<std::uint8_t> mask{context.scratch_resource()};
    if (use_mask) {
      mask.assign(static_cast<std::size_t>(src_w * src_h), 0);
      for (const auto& run : r.runs()) {
        const auto y = static_cast<std::size_t>(run.row - b.row_min);
        const auto x0s = static_cast<std::size_t>(std::int64_t(run.col_begin) - b.col_min);
        const auto x1s = static_cast<std::size_t>(std::int64_t(run.col_end) - b.col_min);
        auto* row = mask.data() + y * static_cast<std::size_t>(src_w);
        std::fill(row + x0s, row + x1s + 1, std::uint8_t{1});
      }
    }
    auto source_hit = [&](std::int64_t row, std::int64_t col) {
      if (row < b.row_min || row > b.row_max || col < b.col_min || col > b.col_max) return false;
      if (!use_mask) return r.contains(row, col);
      const auto y = static_cast<std::size_t>(row - b.row_min);
      const auto x = static_cast<std::size_t>(col - b.col_min);
      return mask[y * static_cast<std::size_t>(src_w) + x] != 0;
    };
    for (auto y = y0; y <= y1; ++y) {
      bool active = false;
      std::int64_t start = 0;
      for (auto x = x0; x <= x1; ++x) {
        const double dx = x - m[2], dy = y - m[5];
        const double sx = inv[0] * dx + inv[1] * dy, sy = inv[2] * dx + inv[3] * dy;
        const bool inside = std::isfinite(sx) && std::isfinite(sy) &&
                            sx >= double(b.col_min) - .5 && sx < double(b.col_max) + .5 &&
                            sy >= double(b.row_min) - .5 && sy < double(b.row_max) + .5 &&
                            source_hit(static_cast<std::int64_t>(std::floor(sy + .5)),
                                       static_cast<std::int64_t>(std::floor(sx + .5)));
        if (inside && !active) {
          active = true;
          start = x;
        }
        if (active && (!inside || x == x1)) {
          runs.push_back({y, start, inside ? x : x - 1});
          active = false;
        }
      }
    }
    return Region::from_runs(runs);
  });
}
}  // namespace mv::regionproc
