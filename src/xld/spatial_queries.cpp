#include <xld/contour_utils.hpp>
#include <xld/spatial_index.hpp>
#include <xld/contour_distance_index.hpp>
#include "distance_avx2.hpp"
#include <runtime/cpu/cpu_features.hpp>
#include <span>
#include <tuple>
namespace mv::xld {
using namespace detail;
DistanceResult distance_pc(const XldContours& c, const Tuple& rows,
                           const Tuple& columns) {
  constexpr auto op = "distance_pc";
  cardinality(c.contour_count() == 1, op, "Contour");
  const auto& contour = c.contour(0);
#if MVKIT_XLD_HAS_AVX2
  const bool vectorized=runtime::cpu_features().selection().avx2;
  if(vectorized)validate_distance_points_avx2(contour.points());else validate(contour,op);
#else
  validate(contour,op);
#endif
  require(!contour.empty(), op, "Contour", "empty contour has no distance");
  auto count = std::max(rows.size(), columns.size());
  cardinality((rows.size() == count || rows.size() == 1) &&
                  (columns.size() == count || columns.size() == 1),
              op, "Row/Column");
  if (rows.empty() && columns.empty()) count = 0;
  ExecutionContext ctx;
  std::pmr::vector<std::array<double, 2>> values(count, ctx.scratch_resource());
  const bool indexed=contour.size()>=256 && count>=16 &&
      ExecutionContext::saturated_work(count,contour.size())>=32768;
  std::optional<ContourDistanceIndex> index;
  if(indexed)index.emplace(contour,ctx.scratch_resource());
  if(index && !index->localized())index.reset();
  ExecutionContext::for_each_partition(
      count, ExecutionContext::saturated_work(count, contour.size()), 32768,
      [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) {
          Point2d p{rows.as_real64(rows.size() == 1 ? 0 : i),
                    columns.as_real64(columns.size() == 1 ? 0 : i)};
          coordinate(p.row, op);
          coordinate(p.col, op);
          if(index){values[i]=index->query(p);continue;}
#if MVKIT_XLD_HAS_AVX2
          if(vectorized){values[i]=distance_point_contour_avx2(contour.points(),contour.closed(),p);continue;}
#endif
          // 坐标已限制在 2^30 内，平方距离不溢出；两次 hypot 前先选极端残差。
          double low2 = INFINITY, high2 = -1;
            Point2d nearest_delta{}, farthest_delta{};
            double radius=INFINITY;
          const auto points=contour.points();
          const auto segment_count=points.size()<2?points.size():points.size()-1+(contour.closed() && points.front()!=points.back());
          for (auto q : points) {
            const Point2d d{p.row-q.row,p.col-q.col};
            const double d2=d.row*d.row+d.col*d.col;
            if(d2>high2 || (d2==high2 && std::hypot(d.row,d.col)>std::hypot(farthest_delta.row,farthest_delta.col))){high2=d2;farthest_delta=d;}
          }
          for (std::size_t j = 0; j < segment_count; ++j) {
              const auto a=points[j],b=points[j+1<points.size()?j+1:0];
              if((a.col>p.col+radius && b.col>p.col+radius) ||
                 (a.col<p.col-radius && b.col<p.col-radius) ||
                 (a.row>p.row+radius && b.row>p.row+radius) ||
                 (a.row<p.row-radius && b.row<p.row-radius))continue;
            const double dr=b.row-a.row, dc=b.col-a.col, den=dr*dr+dc*dc;
            const double t=den==0?0:std::clamp(((p.row-a.row)*dr+(p.col-a.col)*dc)/den,0.,1.);
            const Point2d closest=t==0?a:t==1?b:Point2d{std::lerp(a.row,b.row,t),std::lerp(a.col,b.col,t)};
            const Point2d d{p.row-closest.row,p.col-closest.col};
            const double d2=d.row*d.row+d.col*d.col;
              if(d2<low2 || (d2==low2 && std::hypot(d.row,d.col)<std::hypot(nearest_delta.row,nearest_delta.col))){low2=d2;nearest_delta=d;radius=std::nextafter(std::hypot(d.row,d.col),INFINITY);}
          }
          values[i] = {std::hypot(nearest_delta.row,nearest_delta.col),
                       std::hypot(farthest_delta.row,farthest_delta.col)};
        }
      });
  std::vector<double> minimum(count),maximum(count);
  for(std::size_t i=0;i<count;++i){minimum[i]=values[i][0];maximum[i]=values[i][1];}
  return {Tuple(std::move(minimum)),Tuple(std::move(maximum))};
}
DistanceResult distance_cc(const XldContours& a, const XldContours& b,
                           ContourDistanceMode mode) {
  constexpr auto op = "distance_cc";
  require(mode == ContourDistanceMode::PointToPoint ||
              mode == ContourDistanceMode::PointToSegment,
          op, "Mode", "invalid distance mode");
  cardinality(a.contour_count() == b.contour_count(), op, "Contour1/Contour2");
  ExecutionContext ctx;
  std::pmr::vector<std::array<double, 2>> values(a.contour_count(),
                                                 ctx.scratch_resource());
  ExecutionContext::for_each_partition(
      values.size(), ExecutionContext::saturated_work(work(a), work(b)), 32768,
      [&](auto range) {
        for (auto k = range.begin; k < range.end; ++k) {
          const auto& x = a.contour(k);
          const auto& y = b.contour(k);
          validate(x, op);
          validate(y, op);
          require(!x.empty() && !y.empty(), op, "Contours",
                  "empty contour has no distance");
          const auto xp = x.points();
          const auto yp = y.points();
          ExecutionContext local;
          auto* memory = local.scratch_resource();
          double high = max_point_point(xp, yp, memory);
          double low = min_point_point(xp, yp);
          if (mode == ContourDistanceMode::PointToSegment)
            low = min_segment_segment(contour_segments(x, memory),
                                      contour_segments(y, memory), low,
                                      memory);
          values[k] = {low, high};
        }
      });
  DistanceResult result;
  for (auto v : values) {
    result.minimum.append(v[0]);
    result.maximum.append(v[1]);
  }
  return result;
}
IntersectionResult intersection_contours_xld(const XldContours& a,
                                             const XldContours& b,
                                             IntersectionMode mode) {
  constexpr auto op = "intersection_contours_xld";
  require(mode >= IntersectionMode::All && mode <= IntersectionMode::Self, op,
          "IntersectionType", "invalid mode");
  cardinality(a.contour_count() == 1 && b.contour_count() == 1, op,
              "Contour1/Contour2");
  const auto& x = a.contour(0);
  const auto& y = b.contour(0);
  validate(x, op);
  validate(y, op);
  ExecutionContext ctx;
  std::pmr::vector<Point2d> points{ctx.scratch_resource()};
  bool overlap = false;
  auto add = [&](Crossings c, bool include) {
    overlap |= c.overlap;
    if (include)
      for (int i = 0; i < c.count; ++i) points.push_back(c.points[i]);
  };
  auto* memory = ctx.scratch_resource();
  const auto xs = contour_segments(x, memory);
  const auto ys = contour_segments(y, memory);
  for_each_indexed_pair(xs, ys, [&](std::size_t i, std::size_t j) {
    add(crossings(xs[i].a, xs[i].b, ys[j].a, ys[j].b),
        mode != IntersectionMode::Self);
  }, memory);
  auto self = [&](std::span<const SegmentBox> segs) {
    for_each_indexed_pair(segs, segs, [&](std::size_t i, std::size_t j) {
      if (j <= i) return;
      auto c = crossings(segs[i].a, segs[i].b, segs[j].a, segs[j].b);
      bool adjacent = j == i + 1 ||
                      (i == 0 && j + 1 == segs.size() && segs[j].b == segs[i].a);
      if (adjacent && !c.overlap) return;
      add(c, mode != IntersectionMode::Mutual);
    }, memory);
  };
  self(xs);
  if (&x != &y) self(ys);
  std::sort(points.begin(), points.end(), [](auto p, auto q) {
    return std::tie(p.row, p.col) < std::tie(q.row, q.col);
  });
  points.erase(std::unique(points.begin(), points.end()), points.end());
  IntersectionResult result;
  for (auto p : points) {
    result.row.append(p.row);
    result.column.append(p.col);
  }
  result.overlapping.append(overlap ? 1 : 0);
  return result;
}
}  // namespace mv::xld
