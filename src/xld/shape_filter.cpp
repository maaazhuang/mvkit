#include <xld/contour_utils.hpp>
#include <xld/operators.hpp>
#include <string>
#include <array>
#include <span>
namespace mv::xld {
mv::core::XldContours select_shape_xld(const mv::core::XldContours& contours,
                                       const mv::core::Tuple& features,
                                       bool all, const mv::core::Tuple& minimum,
                                       const mv::core::Tuple& maximum) {
  using namespace detail;
  constexpr auto op = "select_shape_xld";
  const auto n = features.size();
  cardinality(n > 0, op, "Features");
  cardinality((minimum.size() == 1 || minimum.size() == n) &&
                  (maximum.size() == 1 || maximum.size() == n),
              op, "Min/Max");
  enum Feature {
    Area,
    Length,
    Row,
    Column,
    Width,
    Height,
    Ratio,
    Circularity,
    Closed,
    PointCount
  };
  mv::runtime::ScratchArena ctx;
  auto* memory = ctx.resource();
  // 常用谓词放栈上，异常长特征列表仍动态存储，不限制公开基数。
  constexpr std::size_t inline_features=8;
  std::array<Feature,inline_features> feature_buffer;
  std::array<double,inline_features> low_buffer,high_buffer;
  std::pmr::vector<Feature> feature_storage{memory};
  std::pmr::vector<double> low_storage{memory},high_storage{memory};
  if(n>inline_features) {
    feature_storage.resize(n);low_storage.resize(n);high_storage.resize(n);
  }
  const auto selected=n<=inline_features ? std::span<Feature>{feature_buffer}.first(n)
                                        : std::span<Feature>{feature_storage};
  const auto lo=n<=inline_features ? std::span<double>{low_buffer}.first(n)
                                  : std::span<double>{low_storage};
  const auto hi=n<=inline_features ? std::span<double>{high_buffer}.first(n)
                                  : std::span<double>{high_storage};
  bool need_area = false, need_bounds = false, need_length = false, need_centers = false;
  for (std::size_t i = 0; i < n; ++i) {
    auto name = features.as_string(i);
    Feature f;
    if (name == "area")
      f = Area;
    else if (name == "contlength")
      f = Length;
    else if (name == "row")
      f = Row;
    else if (name == "column")
      f = Column;
    else if (name == "width")
      f = Width;
    else if (name == "height")
      f = Height;
    else if (name == "ratio")
      f = Ratio;
    else if (name == "circularity")
      f = Circularity;
    else if (name == "closed")
      f = Closed;
    else if (name == "point_count")
      f = PointCount;
    else {
      require(false, op, "Features", "unsupported XLD shape feature");
      f = Area;
    }
    selected[i]=f;
    need_area |= f == Area || f == Row || f == Column || f == Circularity;
    need_bounds |= f == Width || f == Height || f == Ratio;
    need_length |= f == Length;
    need_centers |= f == Row || f == Column || f == Circularity;
    double a = minimum.as_real64(minimum.size() == 1 ? 0 : i),
           b = maximum.as_real64(maximum.size() == 1 ? 0 : i);
    require(std::isfinite(a) && std::isfinite(b), op, "Min/Max",
            "finite ranges required");
    lo[i]=a;
    hi[i]=b;
  }
  // 大批次保留既有补偿批量测量，直接逐对象原型使正式 P95 退化。
  const bool only_area=n==1 && selected[0]==Area;
  const bool batch_area=need_area && !only_area && work(contours)>=32768;
  AreaCenterXldResult areas;
  if(batch_area)areas=area_center_xld(contours,need_centers);
  std::array<unsigned char,1> single_keep{};
  std::pmr::vector<unsigned char> keep_storage{memory};
  if(contours.contour_count()!=1)keep_storage.resize(contours.contour_count());
  const auto keep=contours.contour_count()==1 ? std::span<unsigned char>{single_keep}
                                           : std::span<unsigned char>{keep_storage};
  const auto process=[&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) {
          const auto& c = contours.contour(i);
          if(only_area){keep[i]=contour_area_in_range(c,lo[0],hi[0]);continue;}
          const auto measured = batch_area
              ? ContourAreaMeasurement{areas.area.as_real64(i),
                    need_centers ? areas.row.as_real64(i) : 0.,
                    need_centers ? areas.column.as_real64(i) : 0.,true}
              : need_area ? measure_contour_area(c,need_centers) : ContourAreaMeasurement{};
          if (!need_area) validate(c, op); // 面积测量已检查全部坐标。
          Point2d low{}, high{};
          if (!c.empty()) low = high = c.points().front();
          if (need_bounds) for (auto p : c.points()) {
            low.row = std::min(low.row, p.row);
            low.col = std::min(low.col, p.col);
            high.row = std::max(high.row, p.row);
            high.col = std::max(high.col, p.col);
          }
          double width = high.col - low.col, height = high.row - low.row,
                 length = need_length ? length_xld(c) : 0,
                 area = measured.area;
          bool pass = all;
          for (std::size_t k = 0; k < n; ++k) {
            double value = 0;
            switch (selected[k]) {
              case Area:
                value = area;
                break;
              case Length:
                value = length;
                break;
              case Row:
                value = area==0 ? 0 : measured.row;
                break;
              case Column:
                value = area==0 ? 0 : measured.col;
                break;
              case Width:
                value = width;
                break;
              case Height:
                value = height;
                break;
              case Ratio:
                value = width == 0 ? 0 : height / width;
                break;
              case Circularity: {
                double max_squared_radius=0;
                for(const auto p:c.points()) {
                  const double dr=p.row-measured.row,dc=p.col-measured.col;
                  max_squared_radius=std::max(max_squared_radius,dr*dr+dc*dc);
                }
                value=max_squared_radius==0 ? 0 : std::min(1.,area/(pi*max_squared_radius));
                break;
              }
              case Closed:
                value = c.closed() || (c.size() > 1 &&
                                       c.points().front() == c.points().back());
                break;
              case PointCount:
                value = double(c.size());
                break;
            }
            bool match = value >= lo[k] && value <= hi[k];
            pass = all ? pass && match : pass || match;
          }
          keep[i] = pass;
        }
      };
  if(keep.size()==1) {
    struct Range {std::size_t begin,end;};
    process(Range{0,1});
  } else ExecutionContext::for_each_partition(keep.size(),work(contours),32768,process);
  if(std::all_of(keep.begin(),keep.end(),[](auto value){return value!=0;}))return contours;
  std::pmr::vector<XldContour> output{memory};
  for (std::size_t i = 0; i < keep.size(); ++i)
    if (keep[i]) output.push_back(contours.contour(i));
  return objects(output);
}
}  // namespace mv::xld
