#include <region/operators.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numeric>
#include <memory_resource>
#include <core/exception.hpp>
#include <runtime/memory/scratch_arena.hpp>

namespace mv::regionproc {
namespace {
using namespace mv::core;
constexpr double coordinate_limit = 0x1p30;
void bounded(double v, const char* op, const char* parameter) {
  if (!std::isfinite(v) || std::abs(v) > coordinate_limit)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    parameter, -1, "finite value in [-2^30,2^30] required");
}
double numeric(const Tuple& t, std::size_t i, const char* parameter) {
  const auto value = t.at(i);
  constexpr auto op = "gen_region_polygon_filled";
  if (value.kind() != ValueKind::Int64 && value.kind() != ValueKind::Real64)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    parameter, static_cast<std::int64_t>(i),
                    "numeric coordinate required");
  const auto v = value.kind() == ValueKind::Int64
                     ? static_cast<double>(value.as_int64())
                     : value.as_real64();
  bounded(v, op, parameter);
  return v;
}
struct Vertex {
  double row, column;
};
void interval(std::pmr::vector<RegionRun>& runs, std::int64_t row, double left,
              double right) {
  const auto first = region_coordinate_rounded(std::ceil(left), "region generation");
  const auto last = region_coordinate_rounded(std::floor(right), "region generation");
  if (first <= last) runs.push_back({row, first, last});
}
}  // namespace

mv::core::Region gen_region_polygon_filled(const mv::core::Tuple& rows,
                                           const mv::core::Tuple& columns, bool halcon_raster) {
  using namespace mv::core;
  constexpr auto op = "gen_region_polygon_filled";
  if(halcon_raster) {
    if(rows.size()!=columns.size())throw Exception(Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"Rows/Columns",-1,"equal coordinate counts required");
    Tuple rr,cc;std::vector<RegionRun> boundary;
    for(std::size_t i=0;i<rows.size();++i) {rr.append(std::round(numeric(rows,i,"Rows")));cc.append(std::round(numeric(columns,i,"Columns")));}
    if(rows.empty())return Region{};
    // 闭合共线多边形沿线双向行走，保留两种中点同值决定，不归一化消除。
    bool collinear=true;std::int64_t direction_row=0,direction_column=0;
    for(std::size_t i=1;i<rows.size();++i) {
      auto dr=static_cast<std::int64_t>(rr.at(i).as_real64()-rr.at(0).as_real64()),dc=static_cast<std::int64_t>(cc.at(i).as_real64()-cc.at(0).as_real64());
      const auto divisor=std::gcd(std::abs(dr),std::abs(dc));if(!divisor)continue;dr/=divisor;dc/=divisor;
      if(dr<0||(dr==0&&dc<0)){dr=-dr;dc=-dc;}
      if(direction_row==0&&direction_column==0){direction_row=dr;direction_column=dc;}
      else if(dr!=direction_row||dc!=direction_column){collinear=false;break;}
    }
    for(std::size_t i=0;i<rows.size();++i) {
      auto x0=static_cast<std::int64_t>(cc.at(i).as_real64()),y0=static_cast<std::int64_t>(rr.at(i).as_real64());
      const auto j=(i+1)%rows.size();auto x1=static_cast<std::int64_t>(cc.at(j).as_real64()),y1=static_cast<std::int64_t>(rr.at(j).as_real64());
      if(y0==y1){boundary.push_back({y0,std::min(x0,x1),std::max(x0,x1)});continue;}
      if(y0>y1){std::swap(x0,x1);std::swap(y0,y1);}
      for(int variant=0;variant<(collinear?2:1);++variant) {
        auto x=x0,y=y0;const auto dx=std::abs(x1-x0),dy=-std::abs(y1-y0),sx=x0<x1?std::int64_t{1}:std::int64_t{-1};auto error=dx+dy;
        for(;;) {
          boundary.push_back({y,x,x});if(x==x1&&y==y1)break;const auto twice=2*error;
          const bool xtie=2*x+sx>0?sx>0:sx<0,ytie=2*y+1>0;
          if(twice>dy||(twice==dy&&(variant?!xtie:xtie))){error+=dy;x+=sx;}
          if(twice<dx||(twice==dx&&(variant?!ytie:ytie))){error+=dx;++y;}
        }
      }
    }
    auto edge=Region::from_runs(boundary);
    return rows.size()<3?edge:gen_region_polygon_filled(rr,cc,false).united(edge);
  }
  if (rows.size() != columns.size() || rows.size() < 3)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, op, "Row/Column", -1,
                    "equal coordinate counts of at least three required");
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<Vertex> vertices{scratch.resource()};
  std::pmr::vector<double> intersections{scratch.resource()};
  std::pmr::vector<RegionRun> runs{scratch.resource()};
  vertices.reserve(rows.size());
  intersections.reserve(rows.size());
  double minrow = coordinate_limit, maxrow = -coordinate_limit;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    Vertex v{numeric(rows, i, "Row"), numeric(columns, i, "Column")};
    minrow = std::min(minrow, v.row);
    maxrow = std::max(maxrow, v.row);
    vertices.push_back(v);
  }
  const auto first = static_cast<std::int64_t>(std::ceil(minrow));
  const auto last = static_cast<std::int64_t>(std::floor(maxrow));
  if (vertices.size() >= 16 && last - first >= 16) {
    struct Edge {
      Vertex a, b;
      std::int64_t first, last;
    };
    std::pmr::vector<Edge> edges{scratch.resource()};
    std::pmr::vector<std::size_t> active{scratch.resource()};
    edges.reserve(vertices.size());
    active.reserve(vertices.size());
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      auto a = vertices[i], b = vertices[(i + 1) % vertices.size()];
      if (a.row > b.row) std::swap(a, b);
      const auto begin = static_cast<std::int64_t>(std::ceil(a.row));
      const auto end = static_cast<std::int64_t>(std::floor(b.row));
      // 整数扫描行匹配原包含端点的垂直守卫，小数水平边在任何行都无边界像素。
      if (begin <= end) edges.push_back({a, b, begin, end});
    }
    std::sort(edges.begin(), edges.end(), [](const auto& a, const auto& b) {
      if (a.first != b.first) return a.first < b.first;
      return a.last < b.last;
    });
    std::size_t next = 0;
    auto next_expiration = last + 1;
    for (auto row = first; row <= last; ++row) {
      // 归属仅在整数边起止事件变化；压缩时重算下次失效位置，不逐行扫描。
      if (row >= next_expiration) {
        next_expiration = last + 1;
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&](auto i) {
                                      if (edges[i].last < row) return true;
                                      next_expiration =
                                          std::min(next_expiration, edges[i].last + 1);
                                      return false;
                                    }),
                     active.end());
      }
      while (next < edges.size() && edges[next].first <= row) {
        next_expiration = std::min(next_expiration, edges[next].last + 1);
        active.push_back(next++);
      }
      intersections.clear();
      for (auto i : active) {
        const auto a = edges[i].a, b = edges[i].b;
        if (row == a.row) interval(runs, row, a.column, a.column);
        if (row == b.row) interval(runs, row, b.column, b.column);
        if (a.row == b.row) {
          if (row == a.row)
            interval(runs, row, std::min(a.column, b.column), std::max(a.column, b.column));
          continue;
        }
        if (row < a.row || row > b.row) continue;
        const double x =
            a.column + (static_cast<double>(row) - a.row) / (b.row - a.row) * (b.column - a.column);
        // 退化多边形或上顶点也包含边界。
        interval(runs, row, x, x);
        // 半开垂直范围使共享顶点只计一次。
        if (row < b.row) intersections.push_back(x);
      }
      std::sort(intersections.begin(), intersections.end());
      for (std::size_t i = 1; i < intersections.size(); i += 2)
        interval(runs, row, intersections[i - 1], intersections[i]);
    }
    // 边输出序不可观测：每边用同一浮点表达式，交点排序后输出游程规范化。
    return Region::from_runs(runs);
  }
  for (auto row = static_cast<std::int64_t>(std::ceil(minrow)); row <= last; ++row) {
    intersections.clear();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      auto a = vertices[i], b = vertices[(i + 1) % vertices.size()];
      if (a.row > b.row) std::swap(a, b);
      if (row == a.row) interval(runs, row, a.column, a.column);
      if (row == b.row) interval(runs, row, b.column, b.column);
      if (a.row == b.row) {
        if (row == a.row)
          interval(runs, row, std::min(a.column, b.column), std::max(a.column, b.column));
        continue;
      }
      if (row < a.row || row > b.row) continue;
      const double x =
          a.column + (static_cast<double>(row) - a.row) / (b.row - a.row) * (b.column - a.column);
      // 退化多边形或上顶点也包含边界。
      interval(runs, row, x, x);
      // 半开垂直范围使共享顶点只计一次。
      if (row < b.row) intersections.push_back(x);
    }
    std::sort(intersections.begin(), intersections.end());
    for (std::size_t i = 1; i < intersections.size(); i += 2)
      interval(runs, row, intersections[i - 1], intersections[i]);
  }
  return Region::from_runs(runs);
}
namespace {
// 普通有限 IEEE-754 值精确步进一个可表示值；
// 零、subnormal、非有限边界仍用 libc nextafter，保留浮点异常行为。
double ellipse_outward(double value, bool toward_positive) {
  if constexpr (std::numeric_limits<double>::is_iec559) {
    auto bits = std::bit_cast<std::uint64_t>(value);
    const auto magnitude = bits & UINT64_C(0x7fffffffffffffff);
    if (magnitude >= UINT64_C(0x0020000000000000) && magnitude < UINT64_C(0x7fe0000000000000)) {
      if ((bits >> 63) != static_cast<std::uint64_t>(toward_positive))
        ++bits;
      else
        --bits;
      return std::bit_cast<double>(bits);
    }
  }
  return std::nextafter(value, toward_positive ? std::numeric_limits<double>::infinity()
                                               : -std::numeric_limits<double>::infinity());
}
}  // namespace
mv::core::Region gen_ellipse(double row, double column, double phi, double radius1,
                             double radius2) {
  using namespace mv::core;
  constexpr auto op = "gen_ellipse";
  bounded(row, op, "Row");
  bounded(column, op, "Column");
  bounded(radius1, op, "Radius1");
  bounded(radius2, op, "Radius2");
  if (!std::isfinite(phi) || radius1 <= 0 || radius2 <= 0)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    !std::isfinite(phi) ? "Phi"
                    : radius1 <= 0      ? "Radius1"
                                        : "Radius2",
                    -1, "finite angle and strictly positive radii required");
  double s = std::sin(phi), c = std::cos(phi);
  // 可精确表示的四分之一圈保持轴对齐端点精确。
  if (std::remainder(phi, 1.57079632679489661923) == 0) {
    if (std::abs(s) < .5) s = 0;
    if (std::abs(c) < .5) c = 0;
  }
  // HALCON 水平轴栅格在扫描转换前对行中心取整，调用方列坐标保留亚像素精度。
  if (s == 0 && radius1 != radius2) row = std::floor(row + 0.5);
  const double extent = radius1 == radius2 ? radius1 : std::hypot(radius1 * s, radius2 * c);
  const double col_extent = radius1 == radius2 ? radius1 : std::hypot(radius1 * c, radius2 * s);
  // 在扫描/分配前检查栅格包围界，不等待逐行生成才发现超范围。
  (void)region_coordinate_rounded(std::ceil(column-col_extent-.5),op);
  (void)region_coordinate_rounded(std::floor(column+col_extent+.5),op);
  const double u = radius1 * s / extent, v = radius2 * c / extent;
  const double offset = radius1 == radius2 ? 0 : radius1 * c * u - radius2 * s * v;
  const double half =
      radius1 >= radius2 ? radius1 * (radius2 / extent) : radius2 * (radius1 / extent);
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<RegionRun> runs{scratch.resource()};
  // 解析扫描线近似，旋转 HALCON 栅格仍可能不同。
  const auto last = region_coordinate_rounded(std::floor(row + extent - 0.5), op);
  const auto first = region_coordinate_rounded(std::ceil(row - extent - 0.5), op);
  // 小栅格保留原端点路径；至少 32 行时省下的库调用才足以摊销位步进守卫。
  if (last - first < 31) {
    for (auto r = first; r <= last; ++r) {
      const double t = (static_cast<double>(r) + 0.5 - row) / extent;
      if (std::abs(t) > 1) continue;
      const double center = column - t * offset;
      const double dx = half * std::sqrt(std::max(0.0, (1 - t) * (1 + t)));
      interval(runs, r, std::nextafter(center - dx - 0.5, -std::numeric_limits<double>::infinity()),
               std::nextafter(center + dx + 0.5, std::numeric_limits<double>::infinity()));
    }
    return Region::from_runs(runs);
  }
  for (auto r = first; r <= last; ++r) {
    const double t = (static_cast<double>(r) + 0.5 - row) / extent;
    if (std::abs(t) > 1) continue;
    const double center = column - t * offset;
    const double dx = half * std::sqrt(std::max(0.0, (1 - t) * (1 + t)));
    interval(runs, r, ellipse_outward(center - dx - 0.5, false),
             ellipse_outward(center + dx + 0.5, true));
  }
  return Region::from_runs(runs);
}
}  // namespace mv::regionproc
