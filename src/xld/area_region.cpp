#include <xld/operators.hpp>
#include <xld/fast_paths_avx2.hpp>
#include <array>
#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <region/operators.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <cmath>
#include <memory_resource>

namespace mv::xld {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
void validate(const XldContour& contour, const char* op) {
  for (const auto& p : contour.points()) {
    if (!(std::abs(p.row) <= 0x1p30 && std::abs(p.col) <= 0x1p30))
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                      "Contour", -1,
                      "finite coordinates in [-2^30,2^30] required");
  }
}
std::size_t work_size(const XldContours& contours) {
  std::size_t work = 0;
  for (std::size_t i = 0; i < contours.contour_count(); ++i)
    work += std::min(contours.contour(i).size(), SIZE_MAX - work);
  return work;
}
// 平移到首顶点后使用 Neumaier 累加，限制相消。
struct Sum {
  double value = 0, correction = 0;
  void add(double x) {
    const auto next = value + x;
    correction += std::abs(value) >= std::abs(x) ? (value - next) + x
                                                 : (x - next) + value;
    value = next;
  }
  double get() const { return value + correction; }
};
using Measurement = ContourAreaMeasurement;
[[noreturn]] void fail_coordinate() {
  throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"area_center_xld","Contour",-1,
                  "finite coordinates in [-2^30,2^30] required");
}
template<bool Centers,bool Vectorized=false>
Measurement measure(const XldContour& contour) {
  auto points = contour.points();
  if (points.empty()) return {};
  const auto validate_point=[](const Point2d& p) {
    if(!(std::abs(p.row)<=0x1p30 && std::abs(p.col)<=0x1p30))
      fail_coordinate();
  };
  validate_point(points.front());
  if (points.size() > 1 && points.front() == points.back())
    points = points.first(points.size() - 1);
  const auto origin = points.front();
  Sum cross, row, col;
  std::size_t first=0;
#if MVKIT_XLD_HAS_AVX2
  if constexpr (Vectorized) {
    const auto& cpu=mv::runtime::cpu_features();
    if(points.size()>=8 && mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
        cpu.selection(),cpu.hardware()),{false,false,true,false})) {
      std::array<double,256> terms,row_terms,col_terms;
      while(points.size()-first>4) {
        const auto count=std::min<std::size_t>(terms.size(),(points.size()-first-1)/4*4);
        if(!contour_cross_terms_avx2(points.subspan(first,count+1),origin,
                                   std::span<double>{terms.data(),count},
                                   Centers ? row_terms.data() : nullptr,
                                   Centers ? col_terms.data() : nullptr))fail_coordinate();
        // 保留原边序及三组补偿和。
        for(std::size_t i=0;i<count;++i) {
          cross.add(terms[i]);
          if constexpr(Centers){row.add(row_terms[i]);col.add(col_terms[i]);}
        }
        first+=count;
      }
    }
  }
#endif
  for (std::size_t i = first; i < points.size(); ++i) {
    const auto a = points[i], b = points[i + 1 < points.size() ? i + 1 : 0];
    // 各后续顶点算术前只检查一次，原点已在上方检查；省独立扫描但保留补偿。
    if(i+1<points.size())validate_point(b);
    const double ar = a.row - origin.row, ac = a.col - origin.col;
    const double br = b.row - origin.row, bc = b.col - origin.col;
    const double w = ac * br - bc * ar;
    cross.add(w);
    if constexpr (Centers) {
      row.add((ar + br) * w);
      col.add((ac + bc) * w);
    }
  }
  const auto twice = cross.get();
  Measurement result{std::abs(twice) * 0.5, origin.row, origin.col, twice <= 0};
  if constexpr (!Centers) return result;
  if (twice == 0) {
    Sum mean_row,mean_col;
    for(const auto& p:points){mean_row.add(p.row-origin.row);mean_col.add(p.col-origin.col);}
    result.row += mean_row.get() / points.size();
    result.col += mean_col.get() / points.size();
  } else {
    result.row += row.get() / (3 * twice);
    result.col += col.get() / (3 * twice);
  }
  if (!mv::core::is_finite(result.row) || !mv::core::is_finite(result.col))
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "area_center_xld", "Contour", -1,
                    "centroid is not representable");
  return result;
}
Region raster(const XldContour& contour, ContourRegionMode mode) {
  validate(contour, "gen_region_contour_xld");
  if (contour.empty()) return {};
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  Tuple rows, columns;
  const auto points = contour.points();
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto y = static_cast<std::int64_t>(std::round(points[i].row));
    auto x = static_cast<std::int64_t>(std::round(points[i].col));
    auto yy = static_cast<std::int64_t>(
        std::round(points[(i + 1) % points.size()].row));
    auto xx = static_cast<std::int64_t>(
        std::round(points[(i + 1) % points.size()].col));
    if (mode == ContourRegionMode::Filled) {
      rows.append(y);
      columns.append(x);
    }
    // 规范端点次序使 Bresenham 同值选择不依赖遍历方向。
    if (y > yy || (y == yy && x > xx)) {
      std::swap(y, yy);
      std::swap(x, xx);
    }
    if (y == yy) {
      runs.push_back({y, std::min(x, xx), std::max(x, xx)});
      continue;
    }
    const auto dx = std::abs(xx - x), dy = -std::abs(yy - y);
    const std::int64_t sx = x < xx ? 1 : -1, sy = y < yy ? 1 : -1;
    auto error = dx + dy;
    while (true) {
      if (!runs.empty() && runs.back().row == y &&
          x >= std::int64_t(runs.back().col_begin) - 1 && x <= std::int64_t(runs.back().col_end) + 1) {
        runs.back().col_begin = std::min<std::int64_t>(runs.back().col_begin, x);
        runs.back().col_end = std::max<std::int64_t>(runs.back().col_end, x);
      } else
        runs.push_back({y, x, x});
      if (x == xx && y == yy) break;
      const auto twice = 2 * error;
      if (twice >= dy) {
        error += dy;
        x += sx;
      }
      if (twice <= dx) {
        error += dx;
        y += sy;
      }
    }
  }
  auto margin = Region::from_runs(runs);
  if (mode == ContourRegionMode::Filled && points.size() >= 3)
    return margin.united(
        mv::regionproc::gen_region_polygon_filled(rows, columns));
  return margin;
}
}  // namespace
ContourAreaMeasurement measure_contour_area(const XldContour& contour,bool centers) {
  return centers ? measure<true,true>(contour) : measure<false,true>(contour);
}
bool contour_area_in_range(const core::XldContour& contour,double lower,double upper) {
  auto points=contour.points();
  if(points.empty())return lower<=0 && upper>=0;
  if(points.size()>(1u<<24)) {
    const auto area=measure_contour_area(contour,false).area;return area>=lower && area<=upper;
  }
  if(!(std::abs(points.front().row)<=0x1p30 && std::abs(points.front().col)<=0x1p30))fail_coordinate();
  if(points.size()>1 && points.front()==points.back())points=points.first(points.size()-1);
  const auto origin=points.front();double sum=0,magnitude=0;std::size_t first=0;
#if MVKIT_XLD_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  if(points.size()>=8 && mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false})) {
    first=(points.size()-1)/4*4;
    if(!contour_cross_sum_avx2(points.first(first+1),origin,sum,magnitude))fail_coordinate();
  }
#endif
  for(std::size_t i=first;i<points.size();++i) {
    const auto a=points[i],b=points[i+1<points.size()?i+1:0];
    if(!(std::abs(b.row)<=0x1p30 && std::abs(b.col)<=0x1p30))fail_coordinate();
    const double term=(a.col-origin.col)*(b.row-origin.row)-(b.col-origin.col)*(a.row-origin.row);
    sum+=term;magnitude+=std::abs(term);
  }
  const auto area=std::abs(sum)*.5;
  // 32*n*epsilon 覆盖普通/补偿求和、绝对项累加及区间端点舍入误差。
  // 项保持原乘积，subnormal 加法加入绝对 padding。四个 SIMD 部分和及最终归约
  // 每项加法少于 n 次，因此同一保守界覆盖该分组。
  const auto error=(32*std::numeric_limits<double>::epsilon()*double(points.size()))*magnitude+
                   32*double(points.size())*std::numeric_limits<double>::denorm_min();
  if(area-error>=lower && area+error<=upper)return true;
  if(area+error<lower || area-error>upper)return false;
  const auto exact=measure_contour_area(contour,false).area;return exact>=lower && exact<=upper;
}
AreaCenterXldResult area_center_xld(const mv::core::XldContours& contours, bool centers) {
  if (contours.empty()) return {};
  if (contours.contour_count() == 1) {
    const auto value = centers ? measure<true,true>(contours.contour(0)) : measure<false,true>(contours.contour(0));
    if (!centers) return {Tuple{value.area}, {}, {}, {}};
    return {Tuple{value.area}, Tuple{value.row}, Tuple{value.col},
            Tuple{value.positive ? "positive" : "negative"}};
  }
  mv::runtime::ScratchArena context;
  std::pmr::vector<Measurement> values(contours.contour_count(),
                                       context.resource());
  ExecutionContext::for_each_partition(
      values.size(), work_size(contours), 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          values[i] = centers ? measure<true,true>(contours.contour(i)) : measure<false,true>(contours.contour(i));
      });
  std::vector<double> area(values.size()), rows(centers ? values.size() : 0),
                      columns(centers ? values.size() : 0);
  std::vector<String> orders; if (centers) orders.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& v = values[i];
    area[i] = v.area;
    if (centers) {
      rows[i] = v.row; columns[i] = v.col;
      orders.emplace_back(v.positive ? "positive" : "negative");
    }
  }
  AreaCenterXldResult result{Tuple{std::move(area)}, Tuple{std::move(rows)},
      Tuple{std::move(columns)}, Tuple{std::move(orders)}};
  return result;
}
mv::core::Regions gen_region_contour_xld(const mv::core::XldContours& contours,
                                         ContourRegionMode mode) {
  if (mode != ContourRegionMode::Filled && mode != ContourRegionMode::Margin)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "gen_region_contour_xld", "Mode", -1,
                    "expected Filled or Margin");
  ExecutionContext context;
  std::pmr::vector<Region> regions(contours.contour_count(),
                                   context.scratch_resource());
  ExecutionContext::for_each_partition(
      regions.size(), work_size(contours), 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          regions[i] = raster(contours.contour(i), mode);
      });
  std::vector<IconicValue> objects;
  objects.reserve(regions.size());
  for (auto& r : regions)
    objects.push_back(IconicValue::from_region(std::move(r)));
  return Regions{IconicTuple{std::move(objects)}};
}
}  // namespace mv::xld
