#include <region/operators.hpp>
#include <region/structuring_element.hpp>
#include <region/banded_morphology.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <core/exception.hpp>

// 形态学逐 Region 对象独立执行；重叠结果不会隐式合并。

namespace mv::regionproc {
namespace {
StructuringElement circle_se(double radius) {
  if (!std::isfinite(radius) || radius < 0.0) throw mv::core::Exception(mv::core::Status::InvalidArgument,"circle morphology","radius must be finite and non-negative");
  const auto limit=mv::core::region_coordinate_rounded(std::floor(radius), "circle morphology");
  std::vector<StructuringOffset> offsets;
  const double rr=radius*radius;
  for(std::int64_t dr=-limit;dr<=limit;++dr){
    for(std::int64_t dc=-limit;dc<=limit;++dc){
      if(static_cast<double>(dr)*dr+static_cast<double>(dc)*dc <= rr+1e-12) offsets.push_back({dr,dc});
      if(dc==limit) break;
    }
    if(dr==limit) break;
  }
  if(offsets.empty()) offsets.push_back({0,0});
  return StructuringElement{std::move(offsets)};
}

// 各区间通过跨度倍增组合，RLE 合并次数为对数级，不逐结构元素像素平移。
mv::core::Region rectangle_one(const mv::core::Region& input, std::size_t width,
                               std::size_t height, bool dilation) {
  if (!width || !height)
    throw mv::core::Exception(mv::core::Status::InvalidArgument,"rectangle morphology","width and height must be positive");
  if (width > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
      height > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()))
    throw mv::core::Exception(mv::core::Status::OutOfRange,"rectangle morphology","structuring element size exceeds int64");
  if (input.empty()) return {};
  if(!dilation && width>input.max_run_length())return {};
  const auto w=static_cast<std::int64_t>(width),h=static_cast<std::int64_t>(height);
  const auto add=[](std::int64_t v,std::int64_t d) {
    if((d>0 && v>std::numeric_limits<std::int64_t>::max()-d) ||
       (d<0 && v<std::numeric_limits<std::int64_t>::min()-d))
      throw mv::core::Exception(mv::core::Status::OutOfRange,"rectangle morphology","coordinate overflows int64");
    return v+d;
  };
  // 面积和外包框已证明完整矩形；直接调整四边，省去逐行识别重复 profile。
  if(input.is_rectangle()) {
    auto box=*input.bounding_box();
    box.col_min=add(box.col_min,dilation?-(w-1)/2:(w-1)/2);
    box.col_max=add(box.col_max,dilation?w/2:-w/2);
    (void)mv::core::region_coordinate(box.col_min);
    (void)mv::core::region_coordinate(box.col_max);
    box.row_min=add(box.row_min,dilation?-(h-1)/2:(h-1)/2);
    box.row_max=add(box.row_max,dilation?h/2:-h/2);
    return mv::core::Region::rectangle(box);
  }
  const auto runs=input.runs();
  std::size_t profile=1;
  while(profile<runs.size() && runs[profile].row==runs[0].row)++profile;
  bool repeated=runs.size()%profile==0;
  for(std::size_t i=profile;repeated && i<runs.size();++i) {
    repeated=static_cast<std::uint64_t>(runs[i].row)-static_cast<std::uint64_t>(runs[0].row)==i/profile &&
             runs[i].col_begin==runs[i%profile].col_begin && runs[i].col_end==runs[i%profile].col_end;
  }
  if(!repeated)if(auto compressed=detail::banded_rectangle(input,w,h,dilation))return std::move(*compressed);
  std::vector<mv::core::RegionRun> horizontal;
  horizontal.reserve(repeated?profile:runs.size());
  for(std::size_t i=0;i<(repeated?profile:runs.size());++i) {
    const auto& r=runs[i];
    if(!dilation && static_cast<std::uint64_t>(r.col_end)-static_cast<std::uint64_t>(r.col_begin)<width-1)continue;
    horizontal.push_back({r.row,add(r.col_begin,dilation?-(w-1)/2:(w-1)/2),add(r.col_end,dilation?w/2:-w/2)});
  }
  auto out=mv::core::Region::from_runs(horizontal);
  if(out.empty())return {};
  if(repeated) {
    const auto first=add(runs.front().row,dilation?-(h-1)/2:(h-1)/2);
    const auto last=add(runs.back().row,dilation?h/2:-h/2);
    if(first>last)return {};
    const auto row_count=static_cast<std::uint64_t>(last)-static_cast<std::uint64_t>(first)+1;
    if(row_count==0 || row_count>std::numeric_limits<std::size_t>::max()/out.runs().size())
      throw mv::core::Exception(mv::core::Status::OutOfRange,"rectangle morphology","result size overflow");
    const auto count=static_cast<std::size_t>(row_count)*out.runs().size();
    const bool safe=row_count<=std::numeric_limits<std::uint64_t>::max()/out.area() &&
        count<=std::vector<mv::core::RegionRun>{}.max_size();
    // 紧凑验证保留精确元数据；数量/面积不确定时保留原错误。
    if(safe) {
      std::vector<mv::core::RegionInterval> profile;
      profile.reserve(out.runs().size());
      for(const auto& run:out.runs())profile.push_back({run.col_begin,run.col_end});
      const mv::core::RegionBandView band{first,last,profile};
      return mv::core::Region::from_row_bands({&band,1});
    }
    std::vector<mv::core::RegionRun> result;
    result.reserve(static_cast<std::size_t>(row_count)*out.runs().size());
    for(auto row=first;;++row) {
      for(const auto& run:out.runs())result.push_back({row,run.col_begin,run.col_end});
      if(row==last)break;
    }
    return mv::core::Region::from_runs(result);
  }
  out=out.shifted(dilation?-(h-1)/2:-h/2,0);
  for(std::size_t covered=1;covered<height && !out.empty();) {
    const auto step=std::min(covered,height-covered);
    const auto shifted=out.shifted(static_cast<std::int64_t>(step),0);
    out=dilation?out.united(shifted):out.intersect(shifted);
    covered+=step;
  }
  return out;
}

mv::core::Region element_region(const StructuringElement& se) {
  std::vector<mv::core::RegionRun> runs;
  runs.reserve(se.offsets().size());
  for (const auto& offset : se.offsets()) runs.push_back({offset.row, offset.column, offset.column});
  return mv::core::Region::from_runs(runs);
}
mv::core::Region dilate_one(const mv::core::Region& r, const StructuringElement& se) {
  if (r.empty()) return {};
  const auto result=dilation1(mv::core::Regions{{r}},mv::core::Regions{{element_region(se)}},0,0,1);
  return result[0];
}
mv::core::Region erode_one(const mv::core::Region& r, const StructuringElement& se) {
  if (r.empty()) return {};
  const auto result=erosion1(mv::core::Regions{{r}},mv::core::Regions{{element_region(se)}},0,0,1);
  return result[0];
}
template<class F> mv::core::Regions map_regions(const mv::core::Regions&rs,F&&f){std::vector<mv::core::Region>out;out.reserve(rs.size());for(std::size_t index=0;index<rs.size();++index)out.push_back(f(rs[index]));return mv::core::Regions{std::move(out)};}
}

namespace {
// 保留原圆盘谓词；此整数界内行列乘积与和精确，仅半径算术舍入。
mv::core::Region prepared_circle(double radius) {
  if (!std::isfinite(radius) || radius < 0.0 || radius > 4096.0)
    return element_region(circle_se(radius));
  const auto limit = static_cast<std::int64_t>(std::floor(radius));
  const double rr = radius * radius;
  std::vector<mv::core::RegionRun> runs;
  runs.reserve(static_cast<std::size_t>(limit * 2 + 1));
  for (std::int64_t row = -limit; row <= limit; ++row) {
    const auto inside = [&](std::int64_t column) {
      return static_cast<double>(row) * row + static_cast<double>(column) * column <= rr + 1e-12;
    };
    if (!inside(0)) continue;
    std::int64_t low = 0, high = limit;
    while (low < high) {
      const auto middle = low + (high - low + 1) / 2;
      if (inside(middle)) low = middle;
      else high = middle - 1;
    }
    runs.push_back({row, -low, low});
  }
  if (runs.empty()) runs.push_back({0,0,0});
  return mv::core::Region::from_runs(runs);
}
mv::core::Regions circle_morph(const mv::core::Regions& regions,double radius,int mode) {
  const mv::core::Regions element{{prepared_circle(radius)}};
  if (mode == 0) return dilation1(regions,element,0,0,1);
  if (mode == 1) return erosion1(regions,element,0,0,1);
  if (mode == 2) return opening(regions,element,0,0,1);
  return closing(regions,element,0,0,1);
}
}
mv::core::Region dilation_circle(const mv::core::Region&r,double radius){return circle_morph(mv::core::Regions{{r}},radius,0)[0];}
mv::core::Regions dilation_circle(const mv::core::Regions&rs,double radius){return circle_morph(rs,radius,0);}
mv::core::Region erosion_circle(const mv::core::Region&r,double radius){return circle_morph(mv::core::Regions{{r}},radius,1)[0];}
mv::core::Regions erosion_circle(const mv::core::Regions&rs,double radius){return circle_morph(rs,radius,1);}
mv::core::Region opening_circle(const mv::core::Region&r,double radius){return circle_morph(mv::core::Regions{{r}},radius,2)[0];}
mv::core::Regions opening_circle(const mv::core::Regions&rs,double radius){return circle_morph(rs,radius,2);}
mv::core::Region closing_circle(const mv::core::Region&r,double radius){return circle_morph(mv::core::Regions{{r}},radius,3)[0];}
mv::core::Regions closing_circle(const mv::core::Regions&rs,double radius){return circle_morph(rs,radius,3);}

mv::core::Region dilation_rectangle1(const mv::core::Region&r,std::size_t w,std::size_t h){return rectangle_one(r,w,h,true);}
mv::core::Regions dilation_rectangle1(const mv::core::Regions&rs,std::size_t w,std::size_t h){(void)rectangle_one({},w,h,true);return map_regions(rs,[&](const auto&r){return rectangle_one(r,w,h,true);});}
mv::core::Region erosion_rectangle1(const mv::core::Region&r,std::size_t w,std::size_t h){return rectangle_one(r,w,h,false);}
mv::core::Regions erosion_rectangle1(const mv::core::Regions&rs,std::size_t w,std::size_t h){(void)rectangle_one({},w,h,false);return map_regions(rs,[&](const auto&r){return rectangle_one(r,w,h,false);});}
mv::core::Region opening_rectangle1(const mv::core::Region&r,std::size_t w,std::size_t h){return rectangle_one(rectangle_one(r,w,h,false),w,h,true);}
mv::core::Regions opening_rectangle1(const mv::core::Regions&rs,std::size_t w,std::size_t h){(void)rectangle_one({},w,h,false);return map_regions(rs,[&](const auto&r){return opening_rectangle1(r,w,h);});}
mv::core::Region closing_rectangle1(const mv::core::Region&r,std::size_t w,std::size_t h){return rectangle_one(rectangle_one(r,w,h,true),w,h,false);}
mv::core::Regions closing_rectangle1(const mv::core::Regions&rs,std::size_t w,std::size_t h){(void)rectangle_one({},w,h,true);return map_regions(rs,[&](const auto&r){return closing_rectangle1(r,w,h);});}

}  // namespace mv::regionproc
