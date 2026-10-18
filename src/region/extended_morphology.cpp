#include <region/extended_common.hpp>
#include <region/banded_morphology.hpp>
#include <utility>

namespace mv::regionproc {
namespace {
using namespace extended;
Region morph(const Region& region, const Region& element, bool dilation) {
  if (region.empty()) return {};
  ExecutionContext context;
  std::pmr::vector<RegionRun> result{context.scratch_resource()};
  std::pmr::vector<RegionRun> next{context.scratch_resource()};
  const auto input = region.runs();
  const auto bounds = *region.bounding_box();
  const auto element_bounds = *element.bounding_box();
  // 每轮候选区域也必须可表示为 32 位；压缩行带不能绕过普通行程路径的范围检查。
  if (dilation) {
    (void)region_coordinate(bounds.row_min+element_bounds.row_min,"region morphology");
    (void)region_coordinate(bounds.row_max+element_bounds.row_max,"region morphology");
    (void)region_coordinate(bounds.col_min+element_bounds.col_min,"region morphology");
    (void)region_coordinate(bounds.col_max+element_bounds.col_max,"region morphology");
  } else {
    (void)region_coordinate(bounds.row_min-element_bounds.row_max,"region morphology");
    (void)region_coordinate(bounds.row_max-element_bounds.row_min,"region morphology");
    (void)region_coordinate(bounds.col_min-element_bounds.col_max,"region morphology");
    (void)region_coordinate(bounds.col_max-element_bounds.col_min,"region morphology");
  }
  const auto width = static_cast<std::uint64_t>(bounds.col_max)-static_cast<std::uint64_t>(bounds.col_min)+1;
  const auto height = static_cast<std::uint64_t>(bounds.row_max)-static_cast<std::uint64_t>(bounds.row_min)+1;
  const auto limit = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  // Minkowski 加法可交换；满矩形可将既有对数矩形核用于结构元素，
  // 避免输入/元素游程双重遍历；腐蚀仅取决于极值。
  if (width && width<=limit && height && height<=limit &&
      input.size()==height && region.area()/height==width && region.area()%height==0) {
    if (dilation) {
      (void)add(bounds.row_min,element_bounds.row_min);
      (void)add(bounds.row_max,element_bounds.row_max);
      (void)add(bounds.col_min,element_bounds.col_min);
      (void)add(bounds.col_max,element_bounds.col_max);
      // 矩形为单行 band，一次展开平移 band 并集，不在高度倍增时物化中间 RLE。
      if(auto compressed=detail::banded_morphology(region,element,true))return std::move(*compressed);
      // 先平移结构元素，使中间矩形核处于最终坐标界内，接近 INT64_MIN/MAX 时也如此。
      const auto center_row=add(bounds.row_min,static_cast<std::int64_t>((height-1)/2));
      const auto center_col=add(bounds.col_min,static_cast<std::int64_t>((width-1)/2));
      return dilation_rectangle1(element.shifted(center_row,center_col),
                                 static_cast<std::size_t>(width),static_cast<std::size_t>(height));
    }
    const auto safe_sub = [](std::int64_t value,std::int64_t delta) {
      return !((delta>0 && value<std::numeric_limits<std::int64_t>::min()+delta) ||
               (delta<0 && value>std::numeric_limits<std::int64_t>::max()+delta));
    };
    // 仅原全部中间相减安全时使用极值捷径，否则保留带检查回退。
    if (safe_sub(bounds.row_min,element_bounds.row_max) &&
        safe_sub(bounds.row_max,element_bounds.row_min) &&
        safe_sub(bounds.col_min,element_bounds.col_max) &&
        safe_sub(bounds.col_max,element_bounds.col_min)) {
      const auto row_min = bounds.row_min-element_bounds.row_min;
      const auto col_min = bounds.col_min-element_bounds.col_min;
      const auto row_max = bounds.row_max-element_bounds.row_max;
      const auto col_max = bounds.col_max-element_bounds.col_max;
      if (row_min > row_max || col_min > col_max) return {};
      return Region::rectangle({row_min,col_min,row_max,col_max});
    }
  }
  if(auto compressed=detail::banded_morphology(region,element,dilation))return std::move(*compressed);
  result.reserve(input.size());
  next.reserve(input.size());
  // 每条平移输入流已有序，直接合并/求交并复用缓冲，
  // 不逐元素行规范化 Region，也不集中排序全部膨胀候选。
  bool first = true;
  for (const auto& se : element.runs()) {
    next.clear();
    if (dilation) {
      (void)add(bounds.row_min,se.row);
      (void)add(bounds.row_max,se.row);
      (void)add(bounds.col_min,se.col_begin);
      (void)add(bounds.col_max,se.col_end);
    }
    const auto subtraction_safe = [](std::int64_t value,std::int64_t delta) {
      return !((delta>0 && value<std::numeric_limits<std::int64_t>::min()+delta) ||
               (delta<0 && value>std::numeric_limits<std::int64_t>::max()+delta));
    };
    const bool safe_erosion = !dilation &&
        subtraction_safe(bounds.row_min,se.row) && subtraction_safe(bounds.row_max,se.row) &&
        subtraction_safe(bounds.col_min,se.col_begin) && subtraction_safe(bounds.col_max,se.col_begin) &&
        subtraction_safe(bounds.col_min,se.col_end) && subtraction_safe(bounds.col_max,se.col_end);
    const auto emit = [&](RegionRun run) {
      if (!next.empty() && next.back().row == run.row &&
          (run.col_begin <= next.back().col_end ||
           (next.back().col_end != std::numeric_limits<std::int64_t>::max() &&
            run.col_begin == std::int64_t(next.back().col_end) + 1))) {
        next.back().col_end = std::max<std::int64_t>(next.back().col_end, run.col_end);
      } else {
        next.push_back(run);
      }
    };
    std::size_t i = 0;
    for (const auto& r : input) {
      RegionRun shifted;
      if (dilation) {
        shifted = {std::int64_t(r.row)+se.row,std::int64_t(r.col_begin)+se.col_begin,std::int64_t(r.col_end)+se.col_end};
      } else if (safe_erosion) {
        const auto begin = std::int64_t(r.col_begin)-se.col_begin;
        const auto end = std::int64_t(r.col_end)-se.col_end;
        if (begin>end) continue;
        shifted = {std::int64_t(r.row)-se.row,begin,end};
      } else {
        const auto begin = sub(r.col_begin, se.col_begin);
        const auto end = sub(r.col_end, se.col_end);
        if (begin > end) continue;
        shifted = {sub(r.row, se.row), begin, end};
      }
      if (first) {
        emit(shifted);
      } else if (dilation) {
        while (i < result.size() &&
               (result[i].row < shifted.row ||
                (result[i].row == shifted.row &&
                 result[i].col_begin <= shifted.col_begin)))
          emit(result[i++]);
        emit(shifted);
      } else {
        while (i < result.size() &&
               (result[i].row < shifted.row ||
                (result[i].row == shifted.row && result[i].col_end < shifted.col_begin)))
          ++i;
        auto j = i;
        while (j < result.size() && result[j].row == shifted.row &&
               result[j].col_begin <= shifted.col_end) {
          next.push_back({shifted.row, std::max<std::int64_t>(result[j].col_begin, shifted.col_begin),
                          std::min<std::int64_t>(result[j].col_end, shifted.col_end)});
          if (result[j].col_end > shifted.col_end) break;
          ++j;
        }
        i = j;
      }
    }
    if (dilation) while (i < result.size()) emit(result[i++]);
    result.swap(next);
    first = false;
    if (result.empty()) break;
  }
  return Region::from_canonical_runs(std::move(result));
}
Regions custom(const Regions& regions, const Regions& elements, std::int64_t ar,
               std::int64_t ac, std::size_t iterations, int mode) {
  constexpr auto op = "custom region morphology";
  if (elements.size() != 1)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, op,
                    "Element", -1, "exactly one structuring element required");
  check(!elements[0].empty(), op, "Element",
        "structuring element must not be empty");
  check(iterations > 0, op, "Iterations", "iterations must be positive");
  ExecutionContext context;
  std::pmr::vector<RegionRun> offsets{context.scratch_resource()};
  for (const auto& r : elements[0].runs())
    offsets.push_back(
        {sub(r.row, ar), sub(r.col_begin, ac), sub(r.col_end, ac)});
  const auto element = Region::from_runs(offsets);
  return map(regions, [&](const auto& region) {
    auto out = region;
    const bool first_dilate = mode == 0 || mode == 3;
    for (std::size_t i = 0; i < iterations; ++i)
      out = morph(out, element, first_dilate);
    if (mode >= 2)
      for (std::size_t i = 0; i < iterations; ++i)
        out = morph(out, element, !first_dilate);
    return out;
  });
}
Region neighborhood_element(Connectivity connectivity) {
  if (connectivity == Connectivity::Four)
    return Region::from_runs({{-1, 0, 0}, {0, -1, 1}, {1, 0, 0}});
  return Region::rectangle({-1, -1, 1, 1});
}
}  // namespace
mv::core::Regions dilation1(const mv::core::Regions& r,
                            const mv::core::Regions& e, std::int64_t ar,
                            std::int64_t ac, std::size_t n) {
  return custom(r, e, ar, ac, n, 0);
}
mv::core::Regions erosion1(const mv::core::Regions& r,
                           const mv::core::Regions& e, std::int64_t ar,
                           std::int64_t ac, std::size_t n) {
  return custom(r, e, ar, ac, n, 1);
}
mv::core::Regions opening(const mv::core::Regions& r,
                          const mv::core::Regions& e, std::int64_t ar,
                          std::int64_t ac, std::size_t n) {
  return custom(r, e, ar, ac, n, 2);
}
mv::core::Regions closing(const mv::core::Regions& r,
                          const mv::core::Regions& e, std::int64_t ar,
                          std::int64_t ac, std::size_t n) {
  return custom(r, e, ar, ac, n, 3);
}

mv::core::Regions boundary(const mv::core::Regions& regions,
                           RegionBoundary mode, Connectivity connectivity) {
  check(mode <= RegionBoundary::Outer, "boundary", "Mode", "invalid mode");
  check(
      connectivity == Connectivity::Four || connectivity == Connectivity::Eight,
      "boundary", "Neighborhood", "expected 4 or 8");
  const auto element = neighborhood_element(connectivity);
  return map(regions, [&](const Region& r) {
    if (mode == RegionBoundary::Outer)
      return morph(r, element, true).difference(r);
    const auto base = mode == RegionBoundary::InnerFilled
                          ? fill_up(r, connectivity == Connectivity::Four
                                           ? Connectivity::Eight
                                           : Connectivity::Four)
                          : r;
    return base.difference(morph(base, element, false));
  });
}
mv::core::Regions fill_up_shape(const mv::core::Regions& regions,
                                ShapeFeature feature, double minimum,
                                double maximum, Connectivity connectivity, bool halcon_features) {
  validate_feature(feature, "fill_up_shape");
  check(std::isfinite(minimum) && std::isfinite(maximum) && minimum <= maximum,
        "fill_up_shape", "Min/Max", "finite ordered bounds required");
  check(
      connectivity == Connectivity::Four || connectivity == Connectivity::Eight,
      "fill_up_shape", "Neighborhood", "expected 4 or 8");
  return map(regions, [&](const Region& region) {
    const auto box = region.bounding_box();
    if (!box) return region;
    const auto holes =
        connection(Region::rectangle(*box).difference(region), connectivity);
    auto result = region;
    for (std::size_t i = 0; i < holes.size(); ++i) {
      const auto b = *holes[i].bounding_box();
      if (b.row_min == box->row_min || b.row_max == box->row_max ||
          b.col_min == box->col_min || b.col_max == box->col_max)
        continue;
      const double value = (halcon_features?halcon_shape_features:shape_features)(
          holes[i],
          1U << static_cast<unsigned>(feature))[static_cast<unsigned>(feature)];
      if (value >= minimum && value <= maximum)
        result = result.united(holes[i]);
    }
    return result;
  });
}
}  // namespace mv::regionproc
