#include <mvkit/c/mv_ncc_matching.h>
#include "control_object.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
#include "control_operator_outputs.hpp"
#include <ncc_matching/model.hpp>
#include "model_snapshot.hpp"
#include <core/model_io.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {
void required(const void* pointer,const char* op,const char* parameter) {
  if(!pointer)throw mv::core::Exception(mv::core::Status::InvalidArgument,mv::core::ErrorCode::NullArgument,op,parameter,-1,"must not be null");
}

template<class Fn> decltype(auto) algorithm(const char* op,Fn&& fn) {
  try{return std::forward<Fn>(fn)();}
  catch(const mv::core::Exception& ex) {
    throw mv::core::Exception(ex.status(),ex.error_code(),op,ex.parameter(),ex.parameter_index(),ex.message());
  }
}

}
extern "C" ::MvStatus mv_create_ncc_model(::MvObject Template,::MvTuple NumLevels,
    ::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple AngleStep,
    ::MvTuple Metric,::MvTuple* ModelID) {
  return mv::c_api::translate([&] {
    constexpr auto op="create_ncc_model";
    mv::c_api::ControlOutputs<1> output(op,{ModelID});
    const auto& source=mv::c_api::require_single_image(Template,op,"Template");
    if(source.channel_count()!=1 || (source.pixel_type()!=mv::core::PixelType::UInt8 && source.pixel_type()!=mv::core::PixelType::UInt16))
      throw mv::core::Exception(mv::core::Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Template",-1,"one byte or uint2 channel required");
    const mv::c_api::TupleInput levels(NumLevels,op,"NumLevels"),start(AngleStart,op,"AngleStart"),
        extent(AngleExtent,op,"AngleExtent"),step(AngleStep,op,"AngleStep"),metric(Metric,op,"Metric");
    const auto automatic=[](const mv::c_api::TupleInput& input) {
      if(input.size()!=1)input.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one value required");
      if(input.at(0).kind()!=mv::core::ValueKind::String)return false;
      if(input.string(0)!="auto")input.fail(mv::core::ErrorCode::UnsupportedValue,0,"expected auto");
      return true;
    };
    mv::ncc_matching::CreateOptions options;
    if(!automatic(levels)) {
      const auto count=levels.scalar_integer();
      if(count<0||count>16)levels.fail(mv::core::ErrorCode::InvalidValue,0,"levels must be 0..16 or auto");
      options.levels=static_cast<int>(count);
    }
    options.angle_start=start.scalar_number();options.angle_extent=extent.scalar_number();
    if(!automatic(step)) {
      options.angle_step=step.scalar_number();
      if(options.angle_step<0 || options.angle_step>3.14159265358979323846/16)step.fail(mv::core::ErrorCode::InvalidValue,0,"angle step must be in [0,pi/16] or auto");
    }
    if(metric.size()!=1)metric.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one metric required");
    const auto name=metric.string(0);
    if(name=="ignore_global_polarity")options.ignore_global_polarity=true;
    else if(name!="use_polarity")metric.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported NCC metric");
    auto model=std::make_unique<MvNCCModelImpl>();
    model->value=algorithm(op,[&]{return mv::ncc_matching::create(source,options);});
    output[0].append(mv::core::Handle::make<mv::c_api::NccModelObject>(model.get()));
    mv::c_api::release_opaque(model.release());output.Commit();
  });
}

namespace {
::MvNCCModel control_model(::MvTuple input,const char* op) {
  const mv::c_api::TupleInput view(input,op,"ModelID");
  if(view.size()!=1)view.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one NCC model required");
  if(view.kind_at(0)!=mv::core::ValueKind::Handle)view.fail(mv::core::ErrorCode::InvalidType,0,"NCC model handle required");
  const auto handle=view.handle(0);
  if(handle.empty()||handle.type_id()!=mv::c_api::NccModelObject::kTypeId)view.fail(mv::core::ErrorCode::InvalidType,0,"control handle is not an NCC model");
  return handle.object<mv::c_api::NccModelObject>().handle();
}
std::string filename(::MvTuple input,const char* op) {
  const mv::c_api::TupleInput view(input,op,"FileName");
  if(view.size()!=1)view.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one file name required");
  const auto name=view.string(0);
  if(name.empty()||name.find('\0')!=std::string_view::npos)view.fail(mv::core::ErrorCode::InvalidValue,0,"nonempty file name without NUL required");
  return std::string(name);
}
}
extern "C" ::MvStatus mv_get_ncc_model_params(::MvTuple ModelID,::MvTuple* NumLevels,
    ::MvTuple* AngleStart,::MvTuple* AngleExtent,::MvTuple* AngleStep,::MvTuple* Metric) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_ncc_model_params";
    mv::c_api::ControlOutputs<5> output(op,{NumLevels,AngleStart,AngleExtent,AngleStep,Metric});
    const auto model=control_model(ModelID,op);const auto& options=model->value.options;
    output[0].append(static_cast<std::int64_t>(model->value.levels.size()));
    output[1].append(options.angle_start);output[2].append(options.angle_extent);output[3].append(options.angle_extent==0 ? 0. : options.angle_step);
    output[4].append(options.ignore_global_polarity?"ignore_global_polarity":"use_polarity");
    output.Commit();
  });
}
extern "C" ::MvStatus mv_write_ncc_model(::MvTuple ModelID,::MvTuple FileName) {
  return mv::c_api::translate([&]{constexpr auto op="write_ncc_model";const auto model=control_model(ModelID,op);const auto name=filename(FileName,op);algorithm(op,[&]{mv::ncc_matching::write(model->value,name);});});
}

extern "C" ::MvStatus mv_read_ncc_model(::MvTuple FileName,::MvTuple* ModelID) {
  return mv::c_api::translate([&]{constexpr auto op="read_ncc_model";mv::c_api::ControlOutputs<1> output(op,{ModelID});
    const auto name=filename(FileName,op);auto model=std::make_unique<MvNCCModelImpl>();
    model->value=algorithm(op,[&]{return mv::c_api::read_snapshot<mv::ncc_matching::Model>(
      mv::core::model_io::read_file(name,64u*1024u*1024u+128,"read_ncc_model"),
      mv::ncc_matching::deserialize,model->snapshot);});
    output[0].append(mv::core::Handle::make<mv::c_api::NccModelObject>(model.get()));
    mv::c_api::release_opaque(model.release());output.Commit();
  });
}

namespace {
std::vector<const mv::ncc_matching::Model*> ncc_models(::MvTuple input,const char* op,bool multiple) {
  const mv::c_api::TupleInput ids(input,op,multiple?"ModelIDs":"ModelID");
  if(!ids.size() || ids.size()>4096 || (!multiple && ids.size()!=1))ids.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one model or a nonempty model array (maximum 4096) required");
  std::vector<const mv::ncc_matching::Model*> result;result.reserve(ids.size());
  for(std::size_t i=0;i<ids.size();++i) {
    const auto value=ids.at(i);
    if(value.kind()!=mv::core::ValueKind::Handle)ids.fail(mv::core::ErrorCode::InvalidType,i,"NCC model handle required");
    const auto handle=value.as_handle();
    if(handle.empty() || handle.type_id()!=mv::c_api::NccModelObject::kTypeId)ids.fail(mv::core::ErrorCode::InvalidType,i,"control handle is not an NCC model");
    result.push_back(&handle.as<mv::c_api::NccModelObject>()->handle()->value);
  }
  return result;
}
struct NccControls {
  std::vector<mv::ncc_matching::FindOptions> options;
  mv::matching::Selection selection;
  NccControls(std::size_t count,const char* op,::MvTuple a,::MvTuple e,::MvTuple s,
      ::MvTuple n,::MvTuple o,::MvTuple p,::MvTuple l) {
    const mv::c_api::TupleInput start(a,op,"AngleStart"),extent(e,op,"AngleExtent"),score(s,op,"MinScore"),
        matches(n,op,"NumMatches"),overlap(o,op,"MaxOverlap"),subpixel(p,op,"SubPixel"),levels(l,op,"NumLevels");
    for(const auto* control:{&start,&extent,&score,&matches,&overlap,&subpixel})
      if(control->size()!=1 && control->size()!=count)control->fail(mv::core::ErrorCode::InvalidCardinality,-1,"one value or one value per model required");
    if(levels.size()!=1 && levels.size()!=2 && levels.size()!=count && levels.size()!=2*count)
      levels.fail(mv::core::ErrorCode::InvalidCardinality,-1,"highest level or shared/per-model level pairs required");
    options.reserve(count);
    const auto number=[](const mv::c_api::TupleInput& v,std::size_t i,double low,double high) {
      const auto index=v.size()==1?0:i;const auto x=v.number(index);
      if(!std::isfinite(x)||x<low||x>high)v.fail(mv::core::ErrorCode::InvalidValue,index,"numeric control outside allowed range");return x;
    };
    for(std::size_t i=0;i<count;++i) {
      mv::ncc_matching::FindOptions q;
      q.angle_start=number(start,i,-std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
      q.angle_extent=number(extent,i,0,6.28318530717958647692);q.min_score=number(score,i,0,1);q.max_overlap=number(overlap,i,0,1);
      const auto index=matches.size()==1?0:i;const auto quota=matches.integer(index);
      if(quota<0)matches.fail(mv::core::ErrorCode::InvalidValue,index,"match count must be nonnegative");q.num_matches=std::size_t(quota);
      const auto mode=subpixel.string(subpixel.size()==1?0:i);
      if(mode!="true" && mode!="false")subpixel.fail(mv::core::ErrorCode::UnsupportedValue,subpixel.size()==1?0:i,"expected true or false");q.subpixel=mode=="true";
      std::size_t high_index=0,low_index=0;bool pair=false;
      if(levels.size()==count && count>1)high_index=i;
      else if(levels.size()==2*count){high_index=2*i;low_index=high_index+1;pair=true;}
      else if(levels.size()==2){low_index=1;pair=true;}
      const auto highest=levels.integer(high_index),lowest=pair?levels.integer(low_index):1;
      if(highest<0 || highest>std::numeric_limits<int>::max() || lowest<0 || lowest>std::numeric_limits<int>::max())
        levels.fail(mv::core::ErrorCode::InvalidValue,high_index,"invalid highest/lowest level pair");
      q.highest_level=int(highest);q.lowest_level=int(std::max<std::int64_t>(1,lowest));options.push_back(q);
    }
    selection.num_matches=matches.size()==1?std::int64_t(options.front().num_matches):-1;
    selection.max_overlap=overlap.size()==1?options.front().max_overlap:-1;
  }
};
const mv::core::Image* ncc_image(::MvObject input,const char* op) {
  mv::c_api::require_iconic(input,op,"Image");input->value.require_kind(mv::core::IconicKind::Image,op);
  if(input->value.empty())return nullptr;
  const auto& image=mv::c_api::require_single_image(input,op,"Image");
  if(image.channel_count()!=1 || (image.pixel_type()!=mv::core::PixelType::UInt8 && image.pixel_type()!=mv::core::PixelType::UInt16))
    throw mv::core::Exception(mv::core::Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"one byte or uint2 channel required");
  return &image;
}
template<std::size_t N> void ncc_columns(mv::c_api::ControlOutputs<N>& output,const mv::ncc_matching::Match& match) {
  // HALCON 返回边缘中心刚体变换的平移，内核使用模型原点的实际像素中心坐标。
  const auto c=std::cos(match.angle),s=std::sin(match.angle);
  output[0].append(match.row+.5*(1-c+s));output[1].append(match.column+.5*(1-c-s));
  output[2].append(match.angle);output[3].append(match.score);
}
}
extern "C" ::MvStatus mv_find_ncc_model(::MvObject Image,::MvTuple ModelID,
    ::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple MinScore,
    ::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,
    ::MvTuple NumLevels,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Score) {
  return mv::c_api::translate([&] {
    constexpr auto op="find_ncc_model";mv::c_api::ControlOutputs<4> output(op,{Row,Column,Angle,Score});
    const auto image=ncc_image(Image,op);const auto models=ncc_models(ModelID,op,false);
    const NccControls controls(1,op,AngleStart,AngleExtent,MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels);
    if(!image){output.Commit();return;}
    const auto found=algorithm(op,[&]{return mv::ncc_matching::find(*image,*models[0],controls.options[0]);});
    for(const auto& match:found)ncc_columns(output,match);output.Commit();
  });
}
extern "C" ::MvStatus mv_find_ncc_models(::MvObject Image,::MvTuple ModelIDs,
    ::MvTuple AngleStart,::MvTuple AngleExtent,::MvTuple MinScore,
    ::MvTuple NumMatches,::MvTuple MaxOverlap,::MvTuple SubPixel,
    ::MvTuple NumLevels,::MvTuple* Row,::MvTuple* Column,::MvTuple* Angle,::MvTuple* Score,::MvTuple* Model) {
  return mv::c_api::translate([&] {
    constexpr auto op="find_ncc_models";mv::c_api::ControlOutputs<5> output(op,{Row,Column,Angle,Score,Model});
    const auto image=ncc_image(Image,op);const auto models=ncc_models(ModelIDs,op,true);
    const NccControls controls(models.size(),op,AngleStart,AngleExtent,MinScore,NumMatches,MaxOverlap,SubPixel,NumLevels);
    if(!image){output.Commit();return;}
    const auto found=algorithm(op,[&]{return mv::ncc_matching::find_models(*image,models,controls.options,controls.selection,true);});
    for(const auto& match:found){ncc_columns(output,match.pose);output[4].append(std::int64_t(match.model_index));}output.Commit();
  });
}
