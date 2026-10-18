#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <memory_resource>
#include <region/operators.hpp>
#include <runtime/execution/execution_context.hpp>

namespace mv::regionproc {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
constexpr double pi = 3.14159265358979323846;
constexpr std::size_t index(ShapeFeature feature) { return static_cast<std::size_t>(feature); }
double delta(std::int64_t x, std::int64_t origin) {
  return x >= origin ? double(std::uint64_t(x) - std::uint64_t(origin))
                     : -double(std::uint64_t(origin) - std::uint64_t(x));
}
struct Point {
  double x, y;
};
double cross(Point o, Point a, Point b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
double convex_area(const Region& region, const RectI64& box) {
  ExecutionContext context;
  std::pmr::vector<Point> points{context.scratch_resource()}, hull{context.scratch_resource()};
  if (region.runs().size() > points.max_size() / 4)
    throw Exception(Status::OutOfRange, "region_features", "convex hull size overflow");
  const auto runs = region.runs();
  // 此范围内半格角点方向乘积精确；更宽坐标保留原 Double 谓词序。
  const bool compact_rows = std::uint64_t(box.col_max) - std::uint64_t(box.col_min) <= (1U << 24) &&
                            std::uint64_t(box.row_max) - std::uint64_t(box.row_min) <= (1U << 24);
  if (compact_rows && region.contains(box.row_min, box.col_min) &&
      region.contains(box.row_min, box.col_max) && region.contains(box.row_max, box.col_min) &&
      region.contains(box.row_max, box.col_max)) {
    // 旧半格凸包具有四个框角及精确整数面积。
    return (delta(box.col_max, box.col_min) + 1) * (delta(box.row_max, box.row_min) + 1);
  }
  if (compact_rows) {
    const auto row_bound = std::min(
        runs.size(),
        static_cast<std::size_t>(std::uint64_t(box.row_max) - std::uint64_t(box.row_min) + 1));
    points.reserve(row_bound * 4);
    for (std::size_t begin = 0; begin < runs.size();) {
      std::size_t end = begin + 1;
      while (end < runs.size() && runs[end].row == runs[begin].row) ++end;
      const auto x0 = delta(runs[begin].col_begin, box.col_min) - .5,
                 x1 = delta(runs[end - 1].col_end, box.col_min) + .5,
                 y = delta(runs[begin].row, box.row_min);
      points.push_back({x0, y - .5});
      points.push_back({x1, y - .5});
      points.push_back({x0, y + .5});
      points.push_back({x1, y + .5});
      begin = end;
    }
  } else {
    points.reserve(runs.size() * 4);
    for (const auto& run : runs) {
      const auto x0 = delta(run.col_begin, box.col_min) - .5,
                 x1 = delta(run.col_end, box.col_min) + .5, y = delta(run.row, box.row_min);
      points.push_back({x0, y - .5});
      points.push_back({x1, y - .5});
      points.push_back({x0, y + .5});
      points.push_back({x1, y + .5});
    }
  }
  std::sort(points.begin(), points.end(),
            [](auto a, auto b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  points.erase(std::unique(points.begin(), points.end(),
                           [](auto a, auto b) { return a.x == b.x && a.y == b.y; }),
               points.end());
  if (points.size() < 3) return double(region.area());
  for (auto p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
    hull.push_back(p);
  }
  const auto lower = hull.size();
  for (std::size_t i = points.size() - 1; i > 0; --i) {
    const auto p = points[i - 1];
    while (hull.size() > lower && cross(hull[hull.size() - 2], hull.back(), p) <= 0)
      hull.pop_back();
    hull.push_back(p);
  }
  hull.pop_back();
  long double area = 0;
  for (std::size_t i = 1; i + 1 < hull.size(); ++i) area += cross(hull[0], hull[i], hull[i + 1]);
  return std::abs(double(area)) * .5;
}
}  // namespace

void validate_feature(ShapeFeature feature, const char* op) {
  if (static_cast<std::size_t>(feature) >= kShapeFeatureCount)
    throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue,
                              op, "Feature", -1, "invalid feature");
}
namespace {
template<bool Complete>
ShapeFeatures calculate_features(const mv::core::Region& region, std::uint32_t mask) {
  ShapeFeatures out{};
  if (region.empty()) return out;
  const auto box = *region.bounding_box();
  constexpr auto cheap = (1U << static_cast<unsigned>(ShapeFeature::Area)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Width)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Height)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Ratio)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Row1)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Column1)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Row2)) |
                         (1U << static_cast<unsigned>(ShapeFeature::Column2));
  if (mask && !(mask & ~cheap)) {
    const double width = delta(box.col_max, box.col_min) + 1;
    const double height = delta(box.row_max, box.row_min) + 1;
    out[index(ShapeFeature::Area)] = double(region.area());
    out[index(ShapeFeature::Width)] = width;
    out[index(ShapeFeature::Height)] = height;
    out[index(ShapeFeature::Ratio)] = height / width;
    out[index(ShapeFeature::Row1)] = double(box.row_min);
    out[index(ShapeFeature::Column1)] = double(box.col_min);
    out[index(ShapeFeature::Row2)] = double(box.row_max);
    out[index(ShapeFeature::Column2)] = double(box.col_max);
    return out;
  }
  // 内部完整数组保持旧行为；公开字段查询只计算实际依赖的矩和角度。
  constexpr auto moments_mask = (1U << index(ShapeFeature::Orientation)) |
      (1U << index(ShapeFeature::Anisometry)) | (1U << index(ShapeFeature::Bulkiness)) |
      (1U << index(ShapeFeature::StructureFactor)) | (1U << index(ShapeFeature::Ra)) |
      (1U << index(ShapeFeature::Rb)) | (1U << index(ShapeFeature::MomentRow)) |
      (1U << index(ShapeFeature::MomentColumn)) | (1U << index(ShapeFeature::MomentCross));
  const bool moments = Complete || (mask & moments_mask);
  // 合并水平游程解析矩；转浮点前平移坐标，保留 int64 极值附近小形状精度。
  double n = 0, mx = 0, my = 0, xx = 0, yy = 0, xy = 0;
  for (const auto& run : region.runs()) {
    const double length = double(std::uint64_t(run.col_end) - std::uint64_t(run.col_begin)) + 1;
    const double x = (delta(run.col_begin, box.col_min) + delta(run.col_end, box.col_min)) * .5,
                 y = delta(run.row, box.row_min);
    const double total = n + length, dx = x - mx, dy = y - my, weight = n * (length / total);
    if (moments) {
      xx += length * (length * length - 1) / 12 + dx * dx * weight;
      yy += dy * dy * weight;
      xy += dx * dy * weight;
    }
    mx += dx * (length / total);
    my += dy * (length / total);
    n = total;
  }
  const auto put = [&](ShapeFeature f, double value) { out[index(f)] = value; };
  put(ShapeFeature::Area, double(region.area()));
  put(ShapeFeature::Row, double(box.row_min) + my);
  put(ShapeFeature::Column, double(box.col_min) + mx);
  const double width = delta(box.col_max, box.col_min) + 1,
               height = delta(box.row_max, box.row_min) + 1;
  put(ShapeFeature::Width, width);
  put(ShapeFeature::Height, height);
  put(ShapeFeature::Ratio, height / width);
  put(ShapeFeature::Row1, double(box.row_min));
  put(ShapeFeature::Column1, double(box.col_min));
  put(ShapeFeature::Row2, double(box.row_max));
  put(ShapeFeature::Column2, double(box.col_max));
  if (moments) {
    xx /= n;
    yy /= n;
    xy /= n;
    put(ShapeFeature::MomentRow, yy);
    put(ShapeFeature::MomentColumn, xx);
    put(ShapeFeature::MomentCross, xy);
    const double spread = std::hypot(xx - yy, 2 * xy), major = std::max(0., (xx + yy + spread) * .5),
                 minor = std::max(0., (xx + yy - spread) * .5);
    const double ra = 2 * std::sqrt(major), rb = 2 * std::sqrt(minor);
    put(ShapeFeature::Ra, ra);
    put(ShapeFeature::Rb, rb);
    if (Complete || (mask & (1U << index(ShapeFeature::Orientation)))) {
      double angle = spread <= 1e-14 * std::max(1., xx + yy) ? 0 : .5 * std::atan2(2 * xy, xx - yy);
      if (angle >= pi / 2) angle -= pi;
      put(ShapeFeature::Orientation, angle);
    }
    if (rb > 0) {
      const auto anisometry = ra / rb, bulkiness = pi * ra * rb / n;
      put(ShapeFeature::Anisometry, anisometry);
      put(ShapeFeature::Bulkiness, bulkiness);
      put(ShapeFeature::StructureFactor, anisometry * bulkiness - 1);
    }
  }
  if (mask & (1U << index(ShapeFeature::Circularity))) {
    double radius2 = 0;
    for (const auto& run : region.runs()) {
      const auto dy = delta(run.row, box.row_min) - my;
      for (auto col : {run.col_begin, run.col_end}) {
        const auto dx = delta(col, box.col_min) - mx;
        radius2 = std::max(radius2, dx * dx + dy * dy);
      }
    }
    put(ShapeFeature::Circularity, radius2 == 0 ? 1 : std::min(1., n / (pi * radius2)));
  }
  if (mask & (1U << index(ShapeFeature::Convexity))) {
    const auto area = convex_area(region, box);
    put(ShapeFeature::Convexity, area > 0 ? std::min(1., n / area) : 0);
  }
  return out;
}

} // namespace
ShapeFeatures shape_features(const mv::core::Region& region, std::uint32_t mask) {
  return calculate_features<true>(region, mask);
}

ShapeFeatures halcon_shape_features(const mv::core::Region& region, std::uint32_t mask) {
  constexpr auto convexity=1U << index(ShapeFeature::Convexity);
  auto result=(mask & ~convexity)?shape_features(region,mask & ~convexity):ShapeFeatures{};
  if(region.empty())return result;
  const double area=double(region.area());
  result[index(ShapeFeature::Orientation)]=-result[index(ShapeFeature::Orientation)];
  if(result[index(ShapeFeature::Orientation)]>=pi/2-1e-14)result[index(ShapeFeature::Orientation)]=-pi/2;
  for(auto feature:{ShapeFeature::MomentRow,ShapeFeature::MomentColumn,ShapeFeature::MomentCross})
    result[index(feature)]*=area;
  // 整数像素产生精确对称矩，避免微小协方差残差改变零值或 anisometry=1 的闭区间选择。
  if(mask & ((1U<<index(ShapeFeature::Anisometry))|(1U<<index(ShapeFeature::MomentRow))|
             (1U<<index(ShapeFeature::MomentColumn))|(1U<<index(ShapeFeature::MomentCross)))) {
  const auto box=*region.bounding_box();
  long double sr=0,sc=0,srr=0,scc=0,src=0;
  for(const auto& run:region.runs()) {
    const long double n=double(std::uint64_t(run.col_end)-std::uint64_t(run.col_begin))+1;
    const long double row=delta(run.row,box.row_min),col=(delta(run.col_begin,box.col_min)+delta(run.col_end,box.col_min))*.5;
    sr+=n*row;sc+=n*col;srr+=n*row*row;scc+=n*(col*col+(n*n-1)/12);src+=n*row*col;
  }
  const double rr=static_cast<double>(srr-sr*sr/area),cc=static_cast<double>(scc-sc*sc/area),rc=static_cast<double>(src-sr*sc/area);
  result[index(ShapeFeature::MomentRow)]=rr;result[index(ShapeFeature::MomentColumn)]=cc;result[index(ShapeFeature::MomentCross)]=rc;
  if(rr==cc&&rc==0&&rr>0)result[index(ShapeFeature::Anisometry)]=1;
  }
  if(region.area()==1)result[index(ShapeFeature::Circularity)]=0;
  if(result[index(ShapeFeature::Rb)]==0)result[index(ShapeFeature::StructureFactor)]=-1;
  if(mask & convexity) {
    const auto hull=shape_trans(mv::core::Regions{region},ShapeTransform::Convex,true);
    result[index(ShapeFeature::Convexity)]=area/double(hull[0].area());
  }
  return result;
}

mv::core::Tuple region_features(const mv::core::Regions& regions,
                                std::span<const ShapeFeature> features) {
  if (features.empty())
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::InvalidCardinality, "region_features",
                              "Features", -1, "at least one feature is required");
  std::uint32_t mask = 0;
  for (auto feature : features) {
    validate_feature(feature, "region_features");
    mask |= 1U << index(feature);
  }
  if (regions.size() > std::numeric_limits<std::size_t>::max() / features.size())
    throw mv::core::Exception(mv::core::Status::OutOfRange, "region_features",
                              "output size overflow");
  if (mask == (1U << index(ShapeFeature::Area))) {
    std::vector<double> values(regions.size() * features.size());
    const auto objects = regions.iconic_tuple().values();
    for (std::size_t i = 0; i < objects.size(); ++i)
      std::fill_n(values.data() + i * features.size(), features.size(),
                  double(objects[i].as_region().area()));
    return mv::core::Tuple{std::move(values)};
  }
  constexpr auto metadata =
      (1U << index(ShapeFeature::Area)) | (1U << index(ShapeFeature::Width)) |
      (1U << index(ShapeFeature::Height)) | (1U << index(ShapeFeature::Ratio)) |
      (1U << index(ShapeFeature::Row1)) | (1U << index(ShapeFeature::Column1)) |
      (1U << index(ShapeFeature::Row2)) | (1U << index(ShapeFeature::Column2));
  constexpr auto centers = (1U << index(ShapeFeature::Row)) | (1U << index(ShapeFeature::Column));
  const bool centers_only = (mask & centers) && !(mask & ~(metadata | centers));
  constexpr auto convexity = 1U << index(ShapeFeature::Convexity);
  if (regions.size() == 1 && (mask & convexity) && !(mask & ~(metadata | convexity))) {
    const auto objects = regions.iconic_tuple().values();
    const auto compact = [](const auto& object) {
      const auto box = object.as_region().bounding_box();
      return !box || (std::uint64_t(box->col_max) - std::uint64_t(box->col_min) <= (1U << 24) &&
                      std::uint64_t(box->row_max) - std::uint64_t(box->row_min) <= (1U << 24));
    };
    if (std::all_of(objects.begin(), objects.end(), compact)) {
      // 此范围内原序整数游程长度和在所有舍入模式均精确；
      // 内部 shape_features 返回完整数组，不能跳过矩。
      std::vector<double> values(regions.size() * features.size());
      for (std::size_t i = 0; i < objects.size(); ++i) {
        const auto& region = objects[i].as_region();
        auto row = shape_features(region, metadata);
        if (!region.empty()) {
          const auto area = convex_area(region, *region.bounding_box());
          row[index(ShapeFeature::Convexity)] =
              area > 0 ? std::min(1., double(region.area()) / area) : 0;
        }
        for (std::size_t j = 0; j < features.size(); ++j)
          values[i * features.size() + j] = row[index(features[j])];
      }
      return mv::core::Tuple{std::move(values)};
    }
  }
  ExecutionContext context;
  // 输出缓冲直接转交给同质 Tuple，避免二次复制和逐元素追加。
  std::vector<double> values(regions.size() * features.size());
  std::size_t work = 0;
  for (std::size_t i = 0; i < regions.size(); ++i)
    work += std::min(regions[i].runs().size(), std::numeric_limits<std::size_t>::max() - work);
  ExecutionContext::for_each_partition(regions.size(), work, 32768, [&, metadata](auto range) {
    // 仅在本分区调用中复用已证明相同的不可变视图。
    const mv::core::RegionRun* previous_runs = nullptr;
    std::size_t previous_count = 0;
    bool previous_valid = false;
    ShapeFeatures previous_centers{};
    for (auto i = range.begin; i < range.end; ++i) {
      // 仅公开请求字段使用该路径，内部 shape_features 仍返回完整数组。
      const auto row = [&] {
        if (!centers_only) return calculate_features<false>(regions[i], mask);
        const auto& region = regions[i];
        const auto runs = region.runs();
        if (previous_valid && previous_runs == runs.data() && previous_count == runs.size())
          return previous_centers;
        auto result = shape_features(region, metadata);
        if (region.empty()) return result;
        const auto box = *region.bounding_box();
        double n = 0, mx = 0, my = 0;
        for (const auto& run : runs) {
          const double length =
              double(std::uint64_t(run.col_end) - std::uint64_t(run.col_begin)) + 1;
          const double x = (delta(run.col_begin, box.col_min) + delta(run.col_end, box.col_min)) *
                           .5,
                       y = delta(run.row, box.row_min);
          const double total = n + length, dx = x - mx, dy = y - my;
          mx += dx * (length / total);
          my += dy * (length / total);
          n = total;
        }
        result[index(ShapeFeature::Row)] = double(box.row_min) + my;
        result[index(ShapeFeature::Column)] = double(box.col_min) + mx;
        previous_runs = runs.data();
        previous_count = runs.size();
        previous_centers = result;
        previous_valid = true;
        return result;
      }();
      for (std::size_t j = 0; j < features.size(); ++j)
        values[i * features.size() + j] = row[index(features[j])];
    }
  });
  return mv::core::Tuple{std::move(values)};
}
mv::core::Tuple circularity(const mv::core::Regions& regions) {
  const std::array<ShapeFeature, 1> f{ShapeFeature::Circularity};
  return region_features(regions, f);
}
mv::core::Tuple orientation_region(const mv::core::Regions& regions) {
  const std::array<ShapeFeature, 1> f{ShapeFeature::Orientation};
  return region_features(regions, f);
}
EccentricityTuples eccentricity(const mv::core::Regions& regions) {
  const std::array<ShapeFeature, 3> f{ShapeFeature::Anisometry, ShapeFeature::Bulkiness,
                                      ShapeFeature::StructureFactor};
  const auto values = region_features(regions, f);
  const auto source = values.empty() ? std::span<const double>{} : values.real64_span();
  std::vector<double> anisometry(regions.size()), bulkiness(regions.size()), structure(regions.size());
  for (std::size_t i = 0; i < regions.size(); ++i) {
    anisometry[i]=source[i*3]; bulkiness[i]=source[i*3+1]; structure[i]=source[i*3+2];
  }
  return {mv::core::Tuple{std::move(anisometry)}, mv::core::Tuple{std::move(bulkiness)},
          mv::core::Tuple{std::move(structure)}};
}

mv::core::Regions sort_region(const mv::core::Regions& regions, RegionSortMode mode,
                              SortOrder order, SortAxis axis) {
  if (mode > RegionSortMode::Character || order > SortOrder::Descending || axis > SortAxis::Column)
    throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue,
                              "sort_region", "Mode", -1, "invalid sorting control");
  if (mode == RegionSortMode::Character) return sort_characters(regions, order, axis);
  struct Key {
    std::size_t index;
    bool empty;
    std::int64_t row = 0, col = 0;
    double center_row = 0, center_col = 0;
  };
  ExecutionContext context;
  std::pmr::vector<Key> keys{context.scratch_resource()};
  keys.reserve(regions.size());
  for (std::size_t i = 0; i < regions.size(); ++i) {
    Key key{i, regions[i].empty()};
    if (!key.empty) {
      const auto& region = regions[i];
      const auto box = *region.bounding_box();
      key.row = box.row_min;
      key.col = box.col_min;
      switch (mode) {
        case RegionSortMode::FirstPoint:
          key.row = region.runs().front().row;
          key.col = region.runs().front().col_begin;
          break;
        case RegionSortMode::LastPoint:
          key.row = region.runs().back().row;
          key.col = region.runs().back().col_end;
          break;
        case RegionSortMode::UpperRight:
          key.col = box.col_max;
          break;
        case RegionSortMode::LowerLeft:
          key.row = box.row_max;
          break;
        case RegionSortMode::LowerRight:
          key.row = box.row_max;
          key.col = box.col_max;
          break;
        case RegionSortMode::Center: {
          const auto f = shape_features(region, 0);
          key.center_row = f[index(ShapeFeature::Row)];
          key.center_col = f[index(ShapeFeature::Column)];
          break;
        }
        default:
          break;
      }
    }
    keys.push_back(key);
  }
  std::sort(keys.begin(), keys.end(), [&](const Key& a, const Key& b) {
    if (a.empty != b.empty) return !a.empty;
    if (a.empty) return a.index < b.index;
    const auto compare = [](auto ar, auto ac, auto br, auto bc) {
      return ar < br ? -1 : ar > br ? 1 : ac < bc ? -1 : ac > bc ? 1 : 0;
    };
    int result;
    if (mode == RegionSortMode::Center)
      result = axis == SortAxis::Row
                   ? compare(a.center_row, a.center_col, b.center_row, b.center_col)
                   : compare(a.center_col, a.center_row, b.center_col, b.center_row);
    else
      result = axis == SortAxis::Row ? compare(a.row, a.col, b.row, b.col)
                                     : compare(a.col, a.row, b.col, b.row);
    return result == 0                     ? a.index < b.index
           : order == SortOrder::Ascending ? result < 0
                                           : result > 0;
  });
  std::vector<mv::core::Region> output;
  output.reserve(keys.size());
  for (const auto& key : keys) output.push_back(regions[key.index]);
  return mv::core::Regions{std::move(output)};
}
mv::core::Regions select_shape_std(const mv::core::Regions& regions, StandardShape shape,
                                   double percent) {
  if (shape > StandardShape::Rectangle2 || !std::isfinite(percent) || percent < 0 || percent > 100)
    throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue,
                              "select_shape_std", "Shape/Percent", -1,
                              "valid shape and percent in [0,100] required");
  if (regions.empty()) return {};
  if (shape != StandardShape::MaxArea) return select_rectangles(regions, shape, percent);
  std::size_t best = 0;
  for (std::size_t i = 1; i < regions.size(); ++i)
    if (regions[i].area() > regions[best].area()) best = i;
  return mv::core::Regions{regions[best]};
}
}  // namespace mv::regionproc
