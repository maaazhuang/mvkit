#include <mvkit/c/mv_region.h>
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "regions_cache.hpp"
#include "region_generation.hpp"
#include "tuple_input.hpp"
#include <core/runtime_context.hpp>
#include <region/operators.hpp>
#include <imageproc/operators.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>
#include <vector>
namespace {
using mv::c_api::TupleInput;using mv::core::Region;using mv::core::Regions;
using mv::core::ErrorCode;
std::string_view text(const TupleInput& input) {
  if(input.size()!=1)input.fail(ErrorCode::InvalidCardinality,-1,"exactly one string required");return input.string(0);
}
std::size_t positive(const TupleInput& input) {
  const auto value=input.scalar_integer(true);if(value<1)input.fail(ErrorCode::InvalidValue,0,"positive integer required");return static_cast<std::size_t>(value);
}
mv::regionproc::ShapeFeature selected_feature(const TupleInput& input) {
  constexpr const char* names[]={"area","row","column","width","height","ratio","row1","column1","row2","column2","circularity","phi","anisometry","bulkiness","struct_factor","convexity","ra","rb","moments_m20","moments_m02","moments_m11"};
  const auto name=text(input);
  if(name!="area" && name!="anisometry" && name!="convexity" && name!="phi" && name!="ra" && name!="rb")
    input.fail(ErrorCode::UnsupportedValue,0,"fill_up_shape feature is not implemented");
  for(unsigned i=0;i<21;++i)if(name==names[i])return static_cast<mv::regionproc::ShapeFeature>(i);
  input.fail(ErrorCode::UnsupportedValue,0,"feature is not implemented");
}
Regions morphology(const Regions& regions,const Regions& elements,std::size_t iterations,int mode,const char* op) {
  if(elements.size()!=1)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"StructElement",-1,"exactly one structuring element required");
  if(elements[0].empty()) {
    if(mode==1)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::UnsupportedValue,op,"StructElement",0,"unbounded erosion with empty structuring element is not implemented");
    std::vector<Region> empty(regions.size());return Regions{std::move(empty)};
  }
  const auto bounds=*elements[0].bounding_box();
  if(bounds.row_min< -0x40000000LL || bounds.row_max>0x40000000LL || bounds.col_min< -0x40000000LL || bounds.col_max>0x40000000LL)throw mv::core::Exception(mv::core::Status::OutOfRange,ErrorCode::IndexOutOfRange,op,"StructElement",0,"structuring element centroid requires coordinates within +/-2^30");
  const auto center=mv::regionproc::area_center(elements[0]);
  const auto row=static_cast<std::int64_t>(std::floor(center.row+.5)),column=static_cast<std::int64_t>(std::floor(center.column+.5));
  std::vector<mv::core::RegionRun> reflected;reflected.reserve(elements[0].runs().size());
  for(const auto& run:elements[0].runs()) {
    // 公开参考点为质心，不再使用任意私有锚点。
    if(run.row==INT64_MIN||run.col_begin==INT64_MIN||run.col_end==INT64_MIN)throw mv::core::Exception(mv::core::Status::OutOfRange,ErrorCode::IndexOutOfRange,op,"StructElement",0,"structuring element reflection overflows");
    reflected.push_back({-run.row,-run.col_end,-run.col_begin});
  }
  const Regions mirror{Region::from_runs(reflected)};
  if(mode==0)return mv::regionproc::dilation1(regions,mirror,-row,-column,iterations);
  if(mode==1)return mv::regionproc::erosion1(regions,elements,row,column,iterations);
  if(mode==2)return mv::regionproc::opening(regions,elements,row,column,1);
  return mv::regionproc::closing(regions,mirror,-row,-column,1);
}
}
extern "C" ::MvStatus mv_gen_ellipse(MvObject* Ellipse,::MvTuple Row,::MvTuple Column,::MvTuple Phi,::MvTuple Radius1,::MvTuple Radius2) {
  return mv::c_api::translate([&]{constexpr auto op="mv_gen_ellipse";mv::c_api::require_output(Ellipse,op,"Ellipse");
    const std::array<TupleInput,5> inputs={TupleInput(Row,op,"Row"),TupleInput(Column,op,"Column"),TupleInput(Phi,op,"Phi"),TupleInput(Radius1,op,"Radius1"),TupleInput(Radius2,op,"Radius2")};
    std::size_t count=0;bool empty=false;for(const auto& input:inputs){count=std::max(count,input.size());empty=empty||input.size()==0;}if(empty)count=0;
    for(const auto& input:inputs)if(input.size()!=1&&input.size()!=count)input.fail(ErrorCode::InvalidCardinality,-1,"generation controls must be scalar or have matching lengths");
    std::vector<Region> regions;regions.reserve(count);
    for(std::size_t i=0;i<count;++i){std::array<double,5> v;for(std::size_t k=0;k<5;++k){v[k]=inputs[k].number(inputs[k].size()==1?0:i);if(!std::isfinite(v[k]))inputs[k].fail(ErrorCode::InvalidValue,i,"finite value required");}for(std::size_t k:{0u,1u,3u,4u})if(std::abs(v[k])>0x1p30)inputs[k].fail(ErrorCode::InvalidValue,i,"value exceeds +/-2^30");
      for(std::size_t k:{3u,4u})if(v[k]<0)inputs[k].fail(ErrorCode::InvalidValue,i,"nonnegative radius required");
      if(v[3]==0||v[4]==0)regions.emplace_back();
      else if(v[3]==v[4])regions.push_back(mv::c_api::generated_circle(v[0],v[1],v[3]));
      else regions.push_back(mv::regionproc::gen_ellipse(v[0],v[1],v[2],v[3],v[4]));}
    *Ellipse=mv::c_api::make_regions(Regions{std::move(regions)});
  });
}
extern "C" ::MvStatus mv_affine_trans_region(MvObject Region,MvObject* RegionAffineTrans,::MvTuple HomMat2D,::MvTuple Interpolate) {
  return mv::c_api::translate([&]{constexpr auto op="mv_affine_trans_region";mv::c_api::require_output(RegionAffineTrans,op,"RegionAffineTrans");const auto regions=mv::c_api::regions_value(Region,op,"Region");
    const TupleInput matrix(HomMat2D,op,"HomMat2D"),interpolate(Interpolate,op,"Interpolate");if(text(interpolate)!="nearest_neighbor")interpolate.fail(ErrorCode::UnsupportedValue,0,"only nearest_neighbor is implemented");
    if(matrix.size()!=6)matrix.fail(ErrorCode::InvalidCardinality,-1,"six affine coefficients required");std::array<double,6> m;for(unsigned i=0;i<6;++i){m[i]=matrix.number(i);if(!std::isfinite(m[i]))matrix.fail(ErrorCode::InvalidValue,i,"finite coefficient required");}
    // HALCON 使用 (row,column)，原点位于左上像素角点。
    mv::core::Tuple swapped;for(double v:{m[4],m[3],m[5]+.5*(m[3]+m[4]-1),m[1],m[0],m[2]+.5*(m[0]+m[1]-1)})swapped.append(v);
    *RegionAffineTrans=mv::c_api::make_regions(mv::regionproc::affine_trans_region(regions,swapped));
  });
}
extern "C" ::MvStatus mv_boundary(MvObject Region,MvObject* RegionBorder,::MvTuple BoundaryType) {
  return mv::c_api::translate([&]{constexpr auto op="mv_boundary";mv::c_api::require_output(RegionBorder,op,"RegionBorder");const auto regions=mv::c_api::regions_value(Region,op,"Region");const TupleInput input(BoundaryType,op,"BoundaryType");const auto name=text(input);
    if(name!="inner"&&name!="inner_filled"&&name!="outer")input.fail(ErrorCode::UnsupportedValue,0,"invalid boundary type");
    const auto mode=name=="inner"?mv::regionproc::RegionBoundary::Inner:name=="inner_filled"?mv::regionproc::RegionBoundary::InnerFilled:mv::regionproc::RegionBoundary::Outer;
    // inner/outer 使用八邻域元素；HALCON 填充边界使用互补腐蚀连通性及八连通孔洞。
    if(mode==mv::regionproc::RegionBoundary::InnerFilled) {
      const auto filled=mv::regionproc::fill_up(regions,mv::regionproc::Connectivity::Eight);
      const auto connectivity=static_cast<mv::regionproc::Connectivity>(12-static_cast<int>(mv::core::current_runtime_context().neighborhood));
      *RegionBorder=mv::c_api::make_regions(mv::regionproc::boundary(filled,mv::regionproc::RegionBoundary::Inner,connectivity));
    } else *RegionBorder=mv::c_api::make_regions(mv::regionproc::boundary(regions,mode,mv::regionproc::Connectivity::Eight));
  });
}
extern "C" ::MvStatus mv_fill_up_shape(MvObject Region,MvObject* RegionFillUp,::MvTuple Feature,::MvTuple Min,::MvTuple Max) {
  return mv::c_api::translate([&]{constexpr auto op="mv_fill_up_shape";mv::c_api::require_output(RegionFillUp,op,"RegionFillUp");const auto regions=mv::c_api::regions_value(Region,op,"Region");const auto feature=selected_feature(TupleInput(Feature,op,"Feature"));
    const double minimum=TupleInput(Min,op,"Min").scalar_number(),maximum=TupleInput(Max,op,"Max").scalar_number();
    *RegionFillUp=mv::c_api::make_regions(mv::regionproc::fill_up_shape(regions,feature,minimum,maximum,static_cast<mv::regionproc::Connectivity>(12-static_cast<int>(mv::core::current_runtime_context().neighborhood)),true));
  });
}
extern "C" ::MvStatus mv_distance_transform(MvObject Region,MvObject* DistanceImage,::MvTuple Metric,::MvTuple Foreground,::MvTuple Width,::MvTuple Height) {
  return mv::c_api::translate([&]{constexpr auto op="mv_distance_transform";mv::c_api::require_output(DistanceImage,op,"DistanceImage");const auto regions=mv::c_api::regions_value(Region,op,"Region");const TupleInput metric(Metric,op,"Metric"),foreground(Foreground,op,"Foreground");const auto name=text(metric),flag=text(foreground);
    if(name!="euclidean"&&name!="city-block"&&name!="chessboard")metric.fail(ErrorCode::UnsupportedValue,0,"metric is not implemented");if(flag!="true"&&flag!="false")foreground.fail(ErrorCode::UnsupportedValue,0,"expected true or false");
    const auto width=positive(TupleInput(Width,op,"Width")),height=positive(TupleInput(Height,op,"Height"));
    const auto mode=name=="euclidean"?mv::regionproc::DistanceMetric::Euclidean:name=="city-block"?mv::regionproc::DistanceMetric::CityBlock:mv::regionproc::DistanceMetric::Chessboard;
    auto images=mv::regionproc::distance_transform(regions,mode,flag=="true",width,height);
    images=mv::imageproc::convert_image_type(images,mv::core::PixelType::Int32);
    *DistanceImage=mv::c_api::make_image_handle(std::move(images),op).release();
  });
}
extern "C" ::MvStatus mv_dilation1(MvObject Region, MvObject StructElement, MvObject* RegionDilation, ::MvTuple Iterations) {
  return mv::c_api::translate([&]{constexpr auto op="mv_dilation1";mv::c_api::require_output(RegionDilation,op,"RegionDilation");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),element=mv::c_api::regions_value(StructElement,op,"StructElement");
    const auto iterations=positive(TupleInput(Iterations,op,"Iterations"));
    *RegionDilation=mv::c_api::make_regions(morphology(regions,element,iterations,0,op));
  });
}
extern "C" ::MvStatus mv_erosion1(MvObject Region, MvObject StructElement, MvObject* RegionErosion, ::MvTuple Iterations) {
  return mv::c_api::translate([&]{constexpr auto op="mv_erosion1";mv::c_api::require_output(RegionErosion,op,"RegionErosion");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),element=mv::c_api::regions_value(StructElement,op,"StructElement");
    const auto iterations=positive(TupleInput(Iterations,op,"Iterations"));
    *RegionErosion=mv::c_api::make_regions(morphology(regions,element,iterations,1,op));
  });
}
extern "C" ::MvStatus mv_opening(MvObject Region, MvObject StructElement, MvObject* RegionOpening) {
  return mv::c_api::translate([&]{constexpr auto op="mv_opening";mv::c_api::require_output(RegionOpening,op,"RegionOpening");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),element=mv::c_api::regions_value(StructElement,op,"StructElement");
    const auto iterations=std::size_t{1};
    *RegionOpening=mv::c_api::make_regions(morphology(regions,element,iterations,2,op));
  });
}
extern "C" ::MvStatus mv_closing(MvObject Region, MvObject StructElement, MvObject* RegionClosing) {
  return mv::c_api::translate([&]{constexpr auto op="mv_closing";mv::c_api::require_output(RegionClosing,op,"RegionClosing");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),element=mv::c_api::regions_value(StructElement,op,"StructElement");
    const auto iterations=std::size_t{1};
    *RegionClosing=mv::c_api::make_regions(morphology(regions,element,iterations,3,op));
  });
}

extern "C" ::MvStatus mv_top_hat(MvObject Region,MvObject StructElement,MvObject* RegionTopHat) {
  return mv::c_api::translate([&] {
    constexpr auto op="top_hat";mv::c_api::require_output(RegionTopHat,op,"RegionTopHat");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),elements=mv::c_api::regions_value(StructElement,op,"StructElement");
    if(elements.empty())throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"StructElement",-1,"at least one structuring element required");
    const Regions first{elements[0]};
    const auto transformed=morphology(regions,first,1,2,op);
    std::vector<mv::core::Region> result;result.reserve(regions.size());
    for(std::size_t i=0;i<regions.size();++i)result.push_back(regions[i].difference(transformed[i]));
    *RegionTopHat=mv::c_api::make_regions(mv::core::Regions{std::move(result)});
  });
}

extern "C" ::MvStatus mv_bottom_hat(MvObject Region,MvObject StructElement,MvObject* RegionBottomHat) {
  return mv::c_api::translate([&] {
    constexpr auto op="bottom_hat";mv::c_api::require_output(RegionBottomHat,op,"RegionBottomHat");
    const auto regions=mv::c_api::regions_value(Region,op,"Region"),elements=mv::c_api::regions_value(StructElement,op,"StructElement");
    if(elements.empty())throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"StructElement",-1,"at least one structuring element required");
    const Regions first{elements[0]};
    const auto transformed=morphology(regions,first,1,3,op);
    std::vector<mv::core::Region> result;result.reserve(regions.size());
    for(std::size_t i=0;i<regions.size();++i)result.push_back(transformed[i].difference(regions[i]));
    *RegionBottomHat=mv::c_api::make_regions(mv::core::Regions{std::move(result)});
  });
}
