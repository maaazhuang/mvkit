#include <mvkit/c/mv_region.h>
#include "error_state.hpp"
#include "regions_cache.hpp"
#include "tuple_input.hpp"
#include <region/operators.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string_view>
#include <vector>
namespace {
using mv::c_api::TupleInput;
using mv::regionproc::ShapeFeature;
constexpr const char* feature_names[]={"area","row","column","width","height","ratio","row1","column1","row2","column2","circularity","phi","anisometry","bulkiness","struct_factor","convexity","ra","rb","moments_m20","moments_m02","moments_m11"};
void output(const void* pointer,const char* op,const char* parameter) {
  if(!pointer)throw mv::core::Exception(mv::core::Status::InvalidArgument,mv::core::ErrorCode::NullArgument,op,parameter,-1,"output must not be null");
}
std::string_view scalar_string(const TupleInput& input) {
  if(input.size()!=1)input.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one string required");
  return input.string(0);
}
std::vector<ShapeFeature> features(const TupleInput& input) {
  if(input.size()==0)input.fail(mv::core::ErrorCode::InvalidCardinality,-1,"at least one feature required");
  std::vector<ShapeFeature> result;result.reserve(input.size());
  for(std::size_t i=0;i<input.size();++i) {
    const auto name=input.string(i);std::size_t k=0;
    while(k<mv::regionproc::kShapeFeatureCount&&name!=feature_names[k])++k;
    if(k==mv::regionproc::kShapeFeatureCount)input.fail(mv::core::ErrorCode::UnsupportedValue,i,"feature is not implemented");
    result.push_back(static_cast<ShapeFeature>(k));
  }
  return result;
}
double bound(const TupleInput& input,std::size_t i,bool minimum) {
  const auto k=input.size()==1?0:i;
  const auto value=input.at(k);
  if(value.kind()==mv::core::ValueKind::String) {
    if(input.string(k)==(minimum?"min":"max"))return minimum?-std::numeric_limits<double>::max():std::numeric_limits<double>::max();
    input.fail(mv::core::ErrorCode::UnsupportedValue,k,"expected numeric bound or min/max sentinel");
  }
  const double number=input.number(k);
  if(!std::isfinite(number))input.fail(mv::core::ErrorCode::InvalidValue,k,"finite bound required");
  return number;
}
}
extern "C" ::MvStatus mv_region_features(MvObject Regions,::MvTuple Features,MvTuple* Value) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_region_features";output(Value,op,"Value");
    const auto regions=mv::c_api::regions_value(Regions,op,"Regions");
    const TupleInput input(Features,op,"Features");
    if(regions.empty()&&input.size()==0){*Value=new MvTupleImpl();return;}
    const auto selected=features(input);
    std::uint32_t mask=0;for(auto f:selected)mask|=1U<<static_cast<unsigned>(f);
    auto result=std::make_unique<MvTupleImpl>();
    if(selected.size()==1&&selected[0]==ShapeFeature::Area) {mv::c_api::ensure_centroid_cache(*Regions);result->value=Regions->ExportCache().cached_real_area;*Value=result.release();return;}
    if(regions.size()>std::numeric_limits<std::size_t>::max()/selected.size())throw mv::core::Exception(mv::core::Status::OutOfRange,mv::core::ErrorCode::InvalidCardinality,op,"Value",-1,"output size overflow");
    std::vector<double> flattened;flattened.reserve(regions.size()*selected.size());
    for(std::size_t i=0;i<regions.size();++i) {
      const auto values=mv::regionproc::halcon_shape_features(regions[i],mask);
      for(auto f:selected)flattened.push_back(values[static_cast<std::size_t>(f)]);
    }
    result->value=mv::core::Tuple{std::move(flattened)};
    *Value=result.release();
  });
}
extern "C" ::MvStatus mv_select_shape(MvObject Regions,MvObject* SelectedRegions,::MvTuple Features,::MvTuple Operation,::MvTuple Min,::MvTuple Max) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_select_shape";output(SelectedRegions,op,"SelectedRegions");
    const auto regions=mv::c_api::regions_value(Regions,op,"Regions");
    const TupleInput feature_input(Features,op,"Features"),operation(Operation,op,"Operation"),min(Min,op,"Min"),max(Max,op,"Max");
    if(regions.empty()){*SelectedRegions=mv::c_api::make_regions(regions);return;}
    const auto selected=features(feature_input);
    const auto name=scalar_string(operation);
    if(name!="and"&&name!="or")operation.fail(mv::core::ErrorCode::UnsupportedValue,0,"expected and or or");
    for(const auto* limits:{&min,&max})if(limits->size()!=selected.size())limits->fail(mv::core::ErrorCode::InvalidCardinality,-1,"bound count must equal feature count");
    std::vector<mv::regionproc::ShapeCriterion> criteria;criteria.reserve(selected.size());
    for(std::size_t i=0;i<selected.size();++i)criteria.push_back({selected[i],bound(min,i,true),bound(max,i,false)});
    *SelectedRegions=mv::c_api::make_regions(mv::regionproc::select_shape(regions,criteria,name=="and"?mv::regionproc::SelectionOperation::And:mv::regionproc::SelectionOperation::Or,true));
  });
}
extern "C" ::MvStatus mv_sort_region(MvObject Regions,MvObject* SortedRegions,::MvTuple SortMode,::MvTuple Order,::MvTuple RowOrCol) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_sort_region";output(SortedRegions,op,"SortedRegions");
    const auto regions=mv::c_api::regions_value(Regions,op,"Regions");
    const TupleInput mode(SortMode,op,"SortMode"),order(Order,op,"Order"),axis(RowOrCol,op,"RowOrCol");
    if(mode.size()!=1&&mode.size()!=2)mode.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one sort name or character/overlap pair required");
    const auto name=mode.string(0),direction=scalar_string(order),primary=scalar_string(axis);
    if(direction!="true"&&direction!="false")order.fail(mv::core::ErrorCode::UnsupportedValue,0,"expected true or false");
    if(primary!="row"&&primary!="column")axis.fail(mv::core::ErrorCode::UnsupportedValue,0,"expected row or column");
    constexpr const char* modes[]={"first_point","last_point","upper_left","upper_right","lower_left","lower_right","","character"};
    std::size_t k=0;while(k<8&&(name!=modes[k]||k==6))++k;
    if(k==8)mode.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported sort mode");
    if(k!=7&&mode.size()!=1)mode.fail(mv::core::ErrorCode::InvalidCardinality,-1,"second control is only valid for character mode");
    double overlap=15;
    if(mode.size()==2) {overlap=mode.number(1);if(!std::isfinite(overlap)||overlap<0||overlap>100)mode.fail(mv::core::ErrorCode::InvalidValue,1,"overlap must be in [0,100]");}
    const auto ordering=direction=="true"?mv::regionproc::SortOrder::Ascending:mv::regionproc::SortOrder::Descending;
    const auto sort_axis=primary=="row"?mv::regionproc::SortAxis::Row:mv::regionproc::SortAxis::Column;
    *SortedRegions=mv::c_api::make_regions(k==7?mv::regionproc::sort_characters(regions,ordering,sort_axis,overlap/100,true):mv::regionproc::sort_region(regions,static_cast<mv::regionproc::RegionSortMode>(k),ordering,sort_axis));
  });
}
extern "C" ::MvStatus mv_select_shape_std(MvObject Regions,MvObject* SelectedRegions,::MvTuple Shape,::MvTuple Percent) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_select_shape_std";output(SelectedRegions,op,"SelectedRegions");
    const auto regions=mv::c_api::regions_value(Regions,op,"Regions");
    const TupleInput shape(Shape,op,"Shape");const auto name=scalar_string(shape);
    const double percent=TupleInput(Percent,op,"Percent").scalar_number();
    if(name!="max_area"&&name!="rectangle1"&&name!="rectangle2")shape.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported standard shape");
    *SelectedRegions=mv::c_api::make_regions(name=="max_area"?mv::regionproc::select_shape_std(regions,mv::regionproc::StandardShape::MaxArea,0):mv::regionproc::select_rectangles(regions,name=="rectangle1"?mv::regionproc::StandardShape::Rectangle1:mv::regionproc::StandardShape::Rectangle2,percent));
  });
}
extern "C" ::MvStatus mv_shape_trans(MvObject Region,MvObject* RegionTrans,::MvTuple Type) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_shape_trans";output(RegionTrans,op,"RegionTrans");
    const auto regions=mv::c_api::regions_value(Region,op,"Region");
    const TupleInput type(Type,op,"Type");const auto name=scalar_string(type);
    if(name!="convex"&&name!="rectangle1")type.fail(mv::core::ErrorCode::UnsupportedValue,0,"only convex and rectangle1 are implemented");
    *RegionTrans=mv::c_api::make_regions(mv::regionproc::shape_trans(regions,name=="convex"?mv::regionproc::ShapeTransform::Convex:mv::regionproc::ShapeTransform::Rectangle1,true));
  });
}
extern "C" ::MvStatus mv_test_region_point(MvObject Regions,::MvTuple Row,::MvTuple Column,MvTuple* IsInside) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_test_region_point";output(IsInside,op,"IsInside");
    const auto regions=mv::c_api::regions_value(Regions,op,"Regions");
    const TupleInput rows(Row,op,"Row"),columns(Column,op,"Column");
    if(!regions.empty()&&rows.size()==0)rows.fail(mv::core::ErrorCode::InvalidCardinality,-1,"at least one point required");
    const mv::c_api::ControlTupleView row_view(Row,op,"Row"),column_view(Column,op,"Column");
    const auto& owned_rows=static_cast<const mv::core::Tuple&>(row_view);
    const auto& owned_columns=static_cast<const mv::core::Tuple&>(column_view);
    if(owned_rows.kind()==mv::core::TupleKind::Int64&&owned_columns.kind()==mv::core::TupleKind::Int64) {
      auto result=std::make_unique<MvTupleImpl>();result->value=mv::regionproc::test_region_point(regions,owned_rows,owned_columns);*IsInside=result.release();return;
    }
    auto coordinates=[](const TupleInput& input) {
      mv::core::Tuple result;
      for(std::size_t i=0;i<input.size();++i) {
        const auto value=input.at(i);
        result.append(value.kind()==mv::core::ValueKind::Real64?mv::core::Value{std::floor(value.as_real64()+.5)}:value);
      }
      return result;
    };
    const auto r=coordinates(rows),c=coordinates(columns);
    auto result=std::make_unique<MvTupleImpl>();result->value=mv::regionproc::test_region_point(regions,r,c);*IsInside=result.release();
  });
}
