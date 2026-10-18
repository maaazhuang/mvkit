#include <mvkit/c/mv_shape_matching.h>
#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
#include <shape_matching/operators.hpp>
#include <charconv>
#include <cmath>
#include <limits>
#include <string_view>
#include <vector>
namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
using mv::shape_matching::SearchParameters;
template<class F> decltype(auto) algorithm(const char* op,F&& fn){try{return std::forward<F>(fn)();}catch(const mv::core::Exception& ex){throw mv::core::Exception(ex.status(),ex.error_code(),op,ex.parameter(),ex.parameter_index(),ex.message());}}
std::vector<const mv::shape_matching::ShapeModel*> models(::MvTuple input,const char* op,bool multi){
 const TupleInput ids(input,op,multi?"ModelIDs":"ModelID");
 if(ids.size()==0||(!multi&&ids.size()!=1)||ids.size()>4096)ids.fail(ErrorCode::InvalidCardinality,-1,multi?"one to 4096 models required":"exactly one model required");
 std::vector<const mv::shape_matching::ShapeModel*> result;result.reserve(ids.size());
 for(std::size_t i=0;i<ids.size();++i){const auto v=ids.at(i);if(v.kind()!=mv::core::ValueKind::Handle)ids.fail(ErrorCode::InvalidType,i,"Shape control handle required");const auto h=v.as_handle();if(h.type_id()!=mv::c_api::ShapeModelObject::kTypeId)ids.fail(ErrorCode::InvalidType,i,"Shape control handle required");const auto* model=&h.as<mv::c_api::ShapeModelObject>()->handle()->value;if(model->empty())ids.fail(ErrorCode::InvalidValue,i,"model must be initialized");result.push_back(model);}return result;
}
void cardinality(const TupleInput& v,std::size_t n){if(v.size()!=1&&v.size()!=n)v.fail(ErrorCode::InvalidCardinality,-1,"one value or one per model required");}
std::size_t index(const TupleInput& v,std::size_t i){return v.size()==1?0:i;}
double number(const TupleInput& v,std::size_t i,double low,double high){const auto at=index(v,i);const auto x=v.number(at);if(!std::isfinite(x)||x<low||x>high)v.fail(ErrorCode::InvalidValue,at,"numeric control outside allowed range");return x;}
int mode(const TupleInput& v,std::size_t i){const auto text=v.string(i);if(text=="none"||text=="false")return 0;if(text=="interpolation"||text=="true")return 1;if(text=="least_squares")return 2;if(text=="least_squares_high")return 3;if(text=="least_squares_very_high")return 4;v.fail(ErrorCode::UnsupportedValue,i,"unsupported subpixel mode");}
int deformation(const TupleInput& v,std::size_t i){const auto text=v.string(i);constexpr std::string_view prefix="max_deformation ";if(text.substr(0,prefix.size())!=prefix)return -1;int n=0;const auto first=text.data()+prefix.size(),end=text.data()+text.size();const auto parsed=std::from_chars(first,end,n);if(parsed.ec!=std::errc{}||parsed.ptr!=end||n<0||n>32)v.fail(ErrorCode::UnsupportedValue,i,"implemented max_deformation range is 0..32");return n;}
struct SubPixel{int mode{},deformation{};};
std::vector<SubPixel> subpixels(const TupleInput& v,std::size_t n,bool multi){
 if(v.size()==0||v.size()>(multi?2*n:2))v.fail(ErrorCode::InvalidCardinality,-1,"invalid subpixel cardinality");std::vector<SubPixel> result;
 if(!multi){SubPixel q;bool have_mode=false;for(std::size_t i=0;i<v.size();++i){const auto d=deformation(v,i);if(d>=0)q.deformation=d;else {q.mode=mode(v,i);have_mode=true;}}if(!have_mode)v.fail(ErrorCode::UnsupportedValue,-1,"deformation requires a subpixel mode");result.push_back(q);return result;}
 for(std::size_t i=0;i<v.size();++i){SubPixel q;q.mode=mode(v,i);if(i+1<v.size()){const auto d=deformation(v,i+1);if(d>=0){q.deformation=d;++i;}}result.push_back(q);}
 if(result.size()!=1&&result.size()!=n)v.fail(ErrorCode::InvalidCardinality,-1,"one subpixel specification or one per model required");return result;
}
struct Controls{
 std::vector<SearchParameters> options;mv::matching::Selection selection;
 Controls(const std::vector<const mv::shape_matching::ShapeModel*>& model,const char* op,bool scaled,bool multi,
  ::MvTuple start_in,::MvTuple extent_in,::MvTuple lo_in,::MvTuple hi_in,::MvTuple score_in,::MvTuple matches_in,::MvTuple overlap_in,::MvTuple sub_in,::MvTuple levels_in,::MvTuple greed_in){
  const auto n=model.size();const TupleInput start(start_in,op,"AngleStart"),extent(extent_in,op,"AngleExtent"),score(score_in,op,"MinScore"),matches(matches_in,op,"NumMatches"),overlap(overlap_in,op,"MaxOverlap"),sub(sub_in,op,"SubPixel"),levels(levels_in,op,"NumLevels"),greed(greed_in,op,"Greediness");
  for(const auto* p:{&start,&extent,&score,&matches,&overlap,&greed})cardinality(*p,n);
  const auto specs=subpixels(sub,n,multi);
  if(levels.size()!=1&&levels.size()!=2&&levels.size()!=n&&levels.size()!=2*n)levels.fail(ErrorCode::InvalidCardinality,-1,"one or two levels globally or per model required");
  TupleInput lo(lo_in,op,"ScaleMin"),hi(hi_in,op,"ScaleMax");if(scaled){cardinality(lo,n);cardinality(hi,n);}options.reserve(n);
  for(std::size_t i=0;i<n;++i){SearchParameters q;q.scale_search=scaled?1:0;
   q.angle_start=number(start,i,-std::numeric_limits<double>::max(),std::numeric_limits<double>::max());q.angle_extent=number(extent,i,0,6.28318530717958647692);q.min_score=number(score,i,0,1);q.max_overlap=number(overlap,i,0,1);q.greediness=number(greed,i,0,1);
   const auto at=index(matches,i);const auto quota=matches.integer(at);if(quota<0)matches.fail(ErrorCode::InvalidValue,at,"nonnegative match count required");q.num_matches=quota;
   if(scaled){q.scale_min=number(lo,i,0,std::numeric_limits<double>::max());q.scale_max=number(hi,i,0,std::numeric_limits<double>::max());if(q.scale_min<=0)lo.fail(ErrorCode::UnsupportedValue,index(lo,i),"positive ScaleMin required by the current kernel");if(q.scale_max<q.scale_min)hi.fail(ErrorCode::InvalidValue,index(hi,i),"ScaleMax must be at least ScaleMin");}
   const auto spec=specs[specs.size()==1?0:i];q.subpixel=spec.mode;q.max_deformation=spec.deformation;
   std::size_t high_index=0,low_index=0;bool pair=false;
   if(levels.size()==n&&n>1)high_index=i;else if(levels.size()==2*n){high_index=2*i;low_index=high_index+1;pair=true;}else if(levels.size()==2){low_index=1;pair=true;}
   const auto highest=levels.integer(high_index),lowest=pair?levels.integer(low_index):1;
   if(highest<0)levels.fail(ErrorCode::InvalidValue,high_index,"highest level must be nonnegative");if(lowest<0)levels.fail(ErrorCode::UnsupportedValue,low_index,"increased tolerance for a negative lowest level is not implemented");
   const auto available=model[i]->levels().size();const auto high=highest==0?available:std::min(available,static_cast<std::size_t>(highest));const auto low=std::min(available,static_cast<std::size_t>(std::max<std::int64_t>(1,lowest)));if(low>high)levels.fail(ErrorCode::InvalidValue,low_index,"lowest level must not exceed highest");q.coarsest=high-1;q.finest=low-1;options.push_back(q);
  }
  selection.num_matches=matches.size()==1?options.front().num_matches:-1;selection.max_overlap=overlap.size()==1?options.front().max_overlap:-1;
 }
};
template<std::size_t N> void search(bool scaled,bool multi,::MvObject input,::MvTuple ids,::MvTuple start,::MvTuple extent,::MvTuple lo,::MvTuple hi,::MvTuple score,::MvTuple matches,::MvTuple overlap,::MvTuple sub,::MvTuple levels,::MvTuple greed,mv::c_api::ControlOutputs<N>& output,const char* op){
 mv::c_api::require_iconic(input,op,"Image");input->value.require_kind(mv::core::IconicKind::Image,op);if(input->value.empty()){output.Commit();return;}
 const auto values=models(ids,op,multi);const auto count=input->value.size();if(count!=1&&(!multi||count!=values.size()))throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"Image",-1,"one image or one per model required");
 std::vector<const mv::core::Image*> images;for(std::size_t i=0;i<count;++i){const auto& image=input->value.at(i).as_image();if(image.channel_count()!=1)throw mv::core::Exception(mv::core::Status::Unsupported,ErrorCode::UnsupportedValue,op,"Image",i,"multichannel Shape search is not implemented");if(!images.empty()){const auto& first=*images.front();if(image.data()!=first.data()||image.width()!=first.width()||image.height()!=first.height()||image.stride()!=first.stride()||image.pixel_type()!=first.pixel_type())throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidValue,op,"Image",i,"model-specific images must share the same pixel matrix and may differ only by domain");}images.push_back(&image);}
 const Controls controls(values,op,scaled,multi,start,extent,lo,hi,score,matches,overlap,sub,levels,greed);
 const auto append=[&](const auto& pose){output[0].append(pose.row);output[1].append(pose.column);output[2].append(pose.angle);if(scaled)output[3].append(pose.scale);output[3+std::size_t(scaled)].append(pose.score);};
 if(multi){const auto found=algorithm(op,[&]{return mv::shape_matching::find_models(*images[0],values,controls.options,controls.selection,count==1?std::span<const mv::core::Image* const>{}:std::span<const mv::core::Image* const>{images});});for(const auto& match:found){append(match.pose);output[4+std::size_t(scaled)].append(static_cast<std::int64_t>(match.model_index));}}
 else {const auto found=algorithm(op,[&]{return mv::shape_matching::find_model(*images[0],*values[0],controls.options[0]);});for(const auto& pose:found)append(pose);}output.Commit();
}
}
extern "C" ::MvStatus mv_find_shape_model(::MvObject Image,::MvTuple ModelID,::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple MinScore,::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,::MvTuple NumLevels,::MvTuple Greediness,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Score){return mv::c_api::translate([&]{constexpr auto op="find_shape_model";mv::c_api::ControlOutputs<4> output(op,{Row,Column,Angle,Score});search(false,false,Image,ModelID,AngleStart,AngleExtent,{},{},MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels,Greediness,output,op);});}
extern "C" ::MvStatus mv_find_scaled_shape_model(::MvObject Image,::MvTuple ModelID,::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple ScaleMin,::MvTuple ScaleMax,::MvTuple MinScore,::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,::MvTuple NumLevels,::MvTuple Greediness,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Scale,::MvTuple* Score){return mv::c_api::translate([&]{constexpr auto op="find_scaled_shape_model";mv::c_api::ControlOutputs<5> output(op,{Row,Column,Angle,Scale,Score});search(true,false,Image,ModelID,AngleStart,AngleExtent,ScaleMin,ScaleMax,MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels,Greediness,output,op);});}
extern "C" ::MvStatus mv_find_shape_models(::MvObject Image,::MvTuple ModelIDs,::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple MinScore,::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,::MvTuple NumLevels,::MvTuple Greediness,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Score,::MvTuple* Model){return mv::c_api::translate([&]{constexpr auto op="find_shape_models";mv::c_api::ControlOutputs<5> output(op,{Row,Column,Angle,Score,Model});search(false,true,Image,ModelIDs,AngleStart,AngleExtent,{},{},MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels,Greediness,output,op);});}
extern "C" ::MvStatus mv_find_scaled_shape_models(::MvObject Image,::MvTuple ModelIDs,::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple ScaleMin,::MvTuple ScaleMax,::MvTuple MinScore,::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,::MvTuple NumLevels,::MvTuple Greediness,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Scale,::MvTuple* Score,::MvTuple* Model){return mv::c_api::translate([&]{constexpr auto op="find_scaled_shape_models";mv::c_api::ControlOutputs<6> output(op,{Row,Column,Angle,Scale,Score,Model});search(true,true,Image,ModelIDs,AngleStart,AngleExtent,ScaleMin,ScaleMax,MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels,Greediness,output,op);});}
