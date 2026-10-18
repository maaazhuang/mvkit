#include <mvkit/c/mv_region.h>
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "regions_cache.hpp"
#include "region_generation.hpp"
#include "tuple_input.hpp"
#include <region/operators.hpp>
#include <region/raster_edge.hpp>
#include <region/rectangle_raster.hpp>
#include <array>
#include <cmath>
#include <vector>

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
using mv::core::Region;
using mv::core::Regions;

template<std::size_t N> std::size_t generation_count(const std::array<TupleInput,N>& inputs) {
  std::size_t count=0;bool empty=false;
  for(const auto& input:inputs){count=std::max(count,input.size());empty=empty||input.size()==0;}
  if(empty)count=0;
  for(const auto& input:inputs)if(input.size()!=1&&input.size()!=count)
    input.fail(ErrorCode::InvalidCardinality,-1,"generation controls must have matching lengths or be scalar");
  return count;
}
double number(const TupleInput& input,std::size_t index) {
  const auto i=input.size()==1?0:index;const double value=input.number(i);
  if(!std::isfinite(value))input.fail(ErrorCode::InvalidValue,i,"finite control value required");
  return value;
}
double coordinate(const TupleInput& input,std::size_t index) {
  const auto value=number(input,index);
  if(value<mv::core::region_coordinate_min||value>mv::core::region_coordinate_max)
    input.fail_range(input.size()==1?0:index,"coordinate exceeds int32 range");
  return value;
}
double nonnegative(const TupleInput& input,std::size_t index) {
  const auto value=coordinate(input,index);
  if(value<0)input.fail(ErrorCode::InvalidValue,index,"nonnegative length required");
  return value;
}
void require_region(::MvObject input,const char* op) {
  mv::c_api::require_iconic(input,op,"Region");input->value.require_kind(mv::core::IconicKind::Region,op);
}
std::size_t dimension(::MvTuple value,const char* op,const char* parameter) {
  const TupleInput input(value,op,parameter);const auto size=input.scalar_number();
  if(size<0||size>=0x1p31)input.fail(ErrorCode::InvalidValue,0,"nonnegative mask dimension in supported range required");
  return static_cast<std::size_t>(std::floor(size+.5));
}
using mv::c_api::generated_circle;
Region generated_rectangle2(double row,double column,double phi,double length1,double length2) {
  std::array<double,4> rows,columns;
  const double c=std::cos(phi),s=std::sin(phi);
  std::size_t corner=0;
  for(auto uv:std::array<std::array<int,2>,4>{{{1,1},{-1,1},{-1,-1},{1,-1}}}) {
    rows[corner]=std::round(row-uv[0]*length1*s+uv[1]*length2*c);
    columns[corner++]=std::round(column+uv[0]*length1*c+uv[1]*length2*s);
  }
  std::array<std::int64_t,4> rr,cc;bool bounded=true;
  for(std::size_t i=0;i<4;++i) {
    const auto y=rows[i],x=columns[i];
    if(!(std::abs(y)<=0x1p30 && std::abs(x)<=0x1p30)){bounded=false;break;}
    rr[i]=static_cast<std::int64_t>(y);cc[i]=static_cast<std::int64_t>(x);
  }
  if(bounded)return mv::regionproc::rectangle_corner_runs(rr,cc);
  // 不满足快路径前置条件时保留原参数及错误优先级。
  mv::core::Tuple row_tuple,column_tuple;
  for(std::size_t i=0;i<4;++i){row_tuple.append(rows[i]);column_tuple.append(columns[i]);}
  auto result=mv::regionproc::gen_region_polygon_filled(row_tuple,column_tuple);
  std::vector<mv::core::RegionRun> boundary;
  for(std::size_t i=0;i<4;++i) {
    auto y=static_cast<std::int64_t>(rows[i]),x=static_cast<std::int64_t>(columns[i]);
    auto yy=static_cast<std::int64_t>(rows[(i+1)%4]),xx=static_cast<std::int64_t>(columns[(i+1)%4]);
    mv::regionproc::raster_edge_runs(y,x,yy,xx,[&](const auto& run){boundary.push_back(run);});
  }
  return result.united(Region::from_runs(boundary));
}
template<int Mode> Regions circle_morphology(const Regions& regions,double radius) {
  const Regions element{generated_circle(0,0,radius)};
  if constexpr(Mode==0)return mv::regionproc::dilation1(regions,element,0,0,1);
  if constexpr(Mode==1) {
    auto result=mv::regionproc::erosion1(regions,element,0,0,1);
    const auto rounded=std::floor(radius*2)*.5;
    return std::floor(rounded)==rounded?mv::regionproc::move_region(result,-1,-1):result;
  }
  if constexpr(Mode==2)return mv::regionproc::opening(regions,element,0,0,1);
  return mv::regionproc::closing(regions,element,0,0,1);
}
std::int64_t rounded_offset(::MvTuple value,const char* op,const char* parameter) {
  const TupleInput input(value,op,parameter);
  if(input.size()!=1)input.fail(ErrorCode::InvalidCardinality,-1,"one offset required");
  const auto rounded=input.at(0).kind()==mv::core::ValueKind::Int64
      ? static_cast<double>(input.integer(0)) : std::round(input.scalar_number());
  if(rounded<mv::core::region_coordinate_min||rounded>mv::core::region_coordinate_max)
    input.fail_range(0,"offset exceeds int32 range");
  return static_cast<std::int64_t>(rounded);
}
}

extern "C" ::MvStatus mv_gen_rectangle1(::MvObject* Rectangle,::MvTuple Row1,::MvTuple Column1,::MvTuple Row2,::MvTuple Column2) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_gen_rectangle1";mv::c_api::require_output(Rectangle,op,"Rectangle");
    const std::array inputs{TupleInput(Row1,op,"Row1"),TupleInput(Column1,op,"Column1"),TupleInput(Row2,op,"Row2"),TupleInput(Column2,op,"Column2")};
    const auto count=generation_count(inputs);std::vector<Region> values;values.reserve(count);
    for(std::size_t i=0;i<count;++i) {
      std::array<std::int64_t,4> v;for(std::size_t k=0;k<4;++k)v[k]=static_cast<std::int64_t>(std::round(coordinate(inputs[k],i)));
      values.push_back(mv::regionproc::gen_rectangle1(v[0],v[1],v[2],v[3]));
    }
    *Rectangle=mv::c_api::make_regions(Regions(std::move(values)));
  });
}
extern "C" ::MvStatus mv_gen_rectangle2(::MvObject* Rectangle,::MvTuple Row,::MvTuple Column,::MvTuple Phi,::MvTuple Length1,::MvTuple Length2) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_gen_rectangle2";mv::c_api::require_output(Rectangle,op,"Rectangle");
    const std::array inputs{TupleInput(Row,op,"Row"),TupleInput(Column,op,"Column"),TupleInput(Phi,op,"Phi"),TupleInput(Length1,op,"Length1"),TupleInput(Length2,op,"Length2")};
    const auto count=generation_count(inputs);std::vector<Region> values;values.reserve(count);
    for(std::size_t i=0;i<count;++i)values.push_back(generated_rectangle2(coordinate(inputs[0],i),coordinate(inputs[1],i),number(inputs[2],i),nonnegative(inputs[3],i),nonnegative(inputs[4],i)));
    *Rectangle=mv::c_api::make_regions(Regions(std::move(values)));
  });
}
extern "C" ::MvStatus mv_gen_circle(::MvObject* Circle,::MvTuple Row,::MvTuple Column,::MvTuple Radius) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_gen_circle";mv::c_api::require_output(Circle,op,"Circle");
    const std::array inputs{TupleInput(Row,op,"Row"),TupleInput(Column,op,"Column"),TupleInput(Radius,op,"Radius")};
    const auto count=generation_count(inputs);std::vector<Region> values;values.reserve(count);
    for(std::size_t i=0;i<count;++i) {
      values.push_back(generated_circle(coordinate(inputs[0],i),coordinate(inputs[1],i),nonnegative(inputs[2],i)));
    }
    *Circle=mv::c_api::make_regions(Regions(std::move(values)));
  });
}
#define CIRCLE_OPERATOR(NAME,MODE) \
extern "C" ::MvStatus NAME(::MvObject Region,::MvObject* Output,::MvTuple Radius) { \
  return mv::c_api::translate([&] {constexpr auto op=#NAME;require_region(Region,op);mv::c_api::require_output(Output,op,"Output"); \
    const TupleInput input(Radius,op,"Radius");const double radius=input.scalar_number(); \
    if(radius<.5||radius>511.5)input.fail(ErrorCode::InvalidValue,0,"radius must be in [0.5,511.5]"); \
    *Output=mv::c_api::make_regions(circle_morphology<MODE>(mv::c_api::regions_value(Region,op),radius));}); \
}
CIRCLE_OPERATOR(mv_dilation_circle,0)
CIRCLE_OPERATOR(mv_erosion_circle,1)
CIRCLE_OPERATOR(mv_opening_circle,2)
CIRCLE_OPERATOR(mv_closing_circle,3)
#undef CIRCLE_OPERATOR
#define RECTANGLE_OPERATOR(NAME,FN,MODE) \
extern "C" ::MvStatus NAME(::MvObject Region,::MvObject* Output,::MvTuple Width,::MvTuple Height) { \
  return mv::c_api::translate([&] {constexpr auto op=#NAME;require_region(Region,op);mv::c_api::require_output(Output,op,"Output"); \
    auto width=dimension(Width,op,"Width"),height=dimension(Height,op,"Height"); \
    if constexpr(MODE>=2) {if(width==0||height==0)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidValue,op,width==0?"Width":"Height",0,"positive mask dimensions required");} \
    if(Region->value.empty()){*Output=mv::c_api::make_regions(Regions{});return;} \
    if(width==0||height==0){std::vector<mv::core::Region> empty(Region->value.size());*Output=mv::c_api::make_regions(Regions(std::move(empty)));return;} \
    width=std::max(std::size_t(1),width);height=std::max(std::size_t(1),height); \
    auto result=mv::regionproc::FN(mv::c_api::regions_value(Region,op),width,height); \
    if constexpr(MODE==1)result=mv::regionproc::move_region(result,height%2==0?1:0,width%2==0?1:0); \
    *Output=mv::c_api::make_regions(std::move(result));}); \
}
RECTANGLE_OPERATOR(mv_dilation_rectangle1,dilation_rectangle1,0)
RECTANGLE_OPERATOR(mv_erosion_rectangle1,erosion_rectangle1,1)
RECTANGLE_OPERATOR(mv_opening_rectangle1,opening_rectangle1,2)
RECTANGLE_OPERATOR(mv_closing_rectangle1,closing_rectangle1,3)
#undef RECTANGLE_OPERATOR
extern "C" ::MvStatus mv_move_region(::MvObject Region,::MvObject* RegionMoved,::MvTuple Row,::MvTuple Column) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_move_region";require_region(Region,op);mv::c_api::require_output(RegionMoved,op,"RegionMoved");
    const auto row=rounded_offset(Row,op,"Row"),column=rounded_offset(Column,op,"Column");
    *RegionMoved=mv::c_api::make_regions(mv::regionproc::move_region(mv::c_api::regions_value(Region,op),row,column));
  });
}
