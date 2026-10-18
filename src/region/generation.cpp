#include <region/operators.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <utility>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory_resource>
#include <vector>

#include <core/exception.hpp>

namespace mv::regionproc {
namespace {
void require_finite(double value, const char* where, const char* name) {
  if (!std::isfinite(value)) throw mv::core::Exception(mv::core::Status::InvalidArgument, where, name);
}
}

mv::core::Region gen_rectangle1(std::int64_t row1, std::int64_t column1,
                                std::int64_t row2, std::int64_t column2) {
  return mv::core::Region::rectangle({row1, column1, row2, column2});
}

namespace {
// 生成器按行写出规范行程；转移 persist 缓冲区，仍执行规范性及面积验证。
mv::core::Region finish_generated(std::pmr::vector<mv::core::RegionRun>&& runs) {
  try { return mv::core::Region::from_canonical_runs(std::move(runs)); }
  catch (const mv::core::Exception& error) {
    throw mv::core::Exception(error.status(), error.error_code(), "Region::from_runs",
        error.parameter(), error.parameter_index(), error.message());
  }
}
}
mv::core::Region gen_circle(double row, double column, double radius) {
  require_finite(row, "gen_circle", "row must be finite");
  require_finite(column, "gen_circle", "column must be finite");
  require_finite(radius, "gen_circle", "radius must be finite");
  if (radius < 0.0) throw mv::core::Exception(mv::core::Status::InvalidArgument, "gen_circle", "radius must be non-negative");
  (void)mv::core::region_coordinate_rounded(std::ceil(column - radius), "gen_circle");
  (void)mv::core::region_coordinate_rounded(std::floor(column + radius), "gen_circle");
  const auto rmin = mv::core::region_coordinate_rounded(std::ceil(row - radius), "region generation");
  const auto rmax = mv::core::region_coordinate_rounded(std::floor(row + radius), "region generation");
  std::pmr::vector<mv::core::RegionRun> runs{mv::runtime::persist_resource()};
  const auto first=static_cast<std::int64_t>(rmin),last=static_cast<std::int64_t>(rmax);
  const auto height=std::max(std::int64_t{0},last-first+1);
  if (height) runs.reserve(static_cast<std::size_t>(std::min<std::int64_t>(height,4096)));
  const double twice_center=row*2;
  // 整数/半整数中心的行差精确，镜像行的平方及端点计算完全相同。
  // 只对较高的圆复用上半部，避免小圆增加展开开销；非对称外包框沿用原路径。
  const bool symmetric=height>=32 && twice_center==std::trunc(twice_center) &&
                       double(first+last)==twice_center;
  const auto stop=symmetric?first+(last-first)/2:last;
  const double r2 = radius * radius;
  for (std::int64_t rr = first; rr <= stop; ++rr) {
    const double dy = static_cast<double>(rr) - row;
    const double remain = r2 - dy * dy;
    if (remain < 0.0) continue;
    const double dx = std::sqrt(std::max(0.0, remain));
    // 外包框已检查 int32；各行端点不会越过外包框，截断加一次校正精确实现 ceil/floor。
    const auto left=column-dx,right=column+dx;
    auto c0=static_cast<std::int64_t>(left),c1=static_cast<std::int64_t>(right);
    c0+=double(c0)<left;c1-=double(c1)>right;
    if (c0 <= c1) {
      auto& run=runs.emplace_back();run.row=static_cast<std::int32_t>(rr);
      run.col_begin=static_cast<std::int32_t>(c0);run.col_end=static_cast<std::int32_t>(c1);
    }
    if (rr == std::numeric_limits<std::int64_t>::max()) break;
  }
  if(symmetric && !runs.empty()) {
    const auto upper=runs.size();
    const auto mirrored=upper-(double(runs.back().row)*2==twice_center?1:0);
    runs.resize(upper+mirrored);
    for(std::size_t i=0;i<mirrored;++i) {
      auto run=runs[mirrored-1-i];
      run.row=static_cast<std::int32_t>(static_cast<std::int64_t>(twice_center)-run.row);
      runs[upper+i]=run;
    }
  }
  return finish_generated(std::move(runs));
}

mv::core::Region gen_rectangle2(double row, double column, double phi, double length1, double length2) {
  require_finite(row, "gen_rectangle2", "row must be finite");
  require_finite(column, "gen_rectangle2", "column must be finite");
  require_finite(phi, "gen_rectangle2", "phi must be finite");
  require_finite(length1, "gen_rectangle2", "length1 must be finite");
  require_finite(length2, "gen_rectangle2", "length2 must be finite");
  if (length1 < 0.0 || length2 < 0.0) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "gen_rectangle2", "lengths must be non-negative");
  }
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  const double row_extent = std::abs(s) * length1 + std::abs(c) * length2;
  const double col_extent = std::abs(c) * length1 + std::abs(s) * length2;
  const auto r0 = mv::core::region_coordinate_rounded(std::ceil(row - row_extent), "region generation");
  const auto r1 = mv::core::region_coordinate_rounded(std::floor(row + row_extent), "region generation");
  const auto c0 = mv::core::region_coordinate_rounded(std::ceil(column - col_extent), "region generation");
  const auto c1 = mv::core::region_coordinate_rounded(std::floor(column + col_extent), "region generation");
  std::pmr::vector<mv::core::RegionRun> runs{mv::runtime::persist_resource()};
  // 有界输入保证原乘加有限、整数区间/中点安全；取整半平面沿列单调，
  // 原谓词、容差及 bbox 全部保留。
  if (std::abs(row) <= 0x1p30 && std::abs(column) <= 0x1p30 &&
      length1 <= 0x1p20 && length2 <= 0x1p20 && c1 >= c0) {
    const double bound_u = length1 + 1e-12, bound_v = length2 + 1e-12;
    // 同一取整线性表达式在各有界坐标单调，四角接受即证明全部 bbox 像素接受。
    bool filled = r0 <= r1;
    for (const auto rr : {r0, r1}) for (const auto cc : {c0, c1}) {
      const double dr = static_cast<double>(rr) - row;
      const double dc = static_cast<double>(cc) - column;
      const double u = dc * c + dr * s;
      const double v = -dc * s + dr * c;
      filled = filled && std::abs(u) <= bound_u && std::abs(v) <= bound_v;
    }
    if (filled) return mv::core::Region::rectangle({r0,c0,r1,c1});
    if (c1 - c0 >= 32) {
    for (std::int64_t rr = r0; rr <= r1; ++rr) {
      const double dr = static_cast<double>(rr) - row;
      std::int64_t first = c0, last = c1;
      const auto clip = [&](const auto& value, bool increasing, double bound) {
        const auto above_lower = [&](std::int64_t column_index) {
          const double x = value(column_index);
          return increasing ? x >= -bound : x <= bound;
        };
        const auto below_upper = [&](std::int64_t column_index) {
          const double x = value(column_index);
          return increasing ? x <= bound : x >= -bound;
        };
        if (!above_lower(last) || !below_upper(first)) return false;
        if (!above_lower(first)) {
          auto low = first, high = last;
          while (low < high) {
            const auto middle = low + (high - low) / 2;
            if (above_lower(middle)) high = middle;
            else low = middle + 1;
          }
          first = low;
        }
        if (!below_upper(last)) {
          auto low = first, high = last;
          while (low < high) {
            const auto middle = low + (high - low + 1) / 2;
            if (below_upper(middle)) low = middle;
            else high = middle - 1;
          }
          last = low;
        }
        return first <= last && above_lower(first) && below_upper(last);
      };
      const auto u = [&](std::int64_t cc) {
        const double dc = static_cast<double>(cc) - column;
        return dc * c + dr * s;
      };
      const auto v = [&](std::int64_t cc) {
        const double dc = static_cast<double>(cc) - column;
        return -dc * s + dr * c;
      };
      if (clip(u, c >= 0, bound_u) && clip(v, s <= 0, bound_v))
        runs.push_back({rr, first, last});
    }
    return finish_generated(std::move(runs));
    }
  }
  for (std::int64_t rr = r0; rr <= r1; ++rr) {
    std::int64_t begin = 0;
    bool active = false;
    for (std::int64_t cc = c0; cc <= c1; ++cc) {
      const double dr = static_cast<double>(rr) - row;
      const double dc = static_cast<double>(cc) - column;
      const double u = dc * c + dr * s;
      const double v = -dc * s + dr * c;
      const bool inside = std::abs(u) <= length1 + 1e-12 && std::abs(v) <= length2 + 1e-12;
      if (inside && !active) { begin = cc; active = true; }
      if ((!inside || cc == c1) && active) {
        const auto end = inside && cc == c1 ? cc : cc - 1;
        runs.push_back({rr, begin, end});
        active = false;
      }
      if (cc == std::numeric_limits<std::int64_t>::max()) break;
    }
    if (rr == std::numeric_limits<std::int64_t>::max()) break;
  }
  return finish_generated(std::move(runs));
}

}  // namespace mv::regionproc
