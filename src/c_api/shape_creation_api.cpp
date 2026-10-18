#include <mvkit/c/mv_shape_matching.h>
#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
#include <shape_matching/operators.hpp>
#include <numbers>
namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
using mv::core::Tuple;
template<class Fn> decltype(auto) algorithm(const char* op,Fn&& fn){
  try{return std::forward<Fn>(fn)();}catch(const mv::core::Exception& ex){throw mv::core::Exception(ex.status(),ex.error_code(),op,ex.parameter(),ex.parameter_index(),ex.message());}
}
void iconic(::MvObject object,const char* op,bool xld){
  const char* parameter=xld?"Contours":"Template";
  if(!object)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::NullArgument,op,parameter,-1,"iconic input must not be null");
  object->value.require_kind(xld?mv::core::IconicKind::XldContour:mv::core::IconicKind::Image,op);
  if(!xld&&object->value.size()>1)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,parameter,-1,"at most one image object required");
}
Tuple auto_number(::MvTuple input,const char* op,const char* parameter,bool integer,bool allow_auto=true){
  const TupleInput v(input,op,parameter);
  if(v.size()!=1)v.fail(ErrorCode::InvalidCardinality,-1,"exactly one control value required");
  if(v.at(0).kind()==mv::core::ValueKind::String){
    if(!allow_auto||v.string(0)!="auto")v.fail(ErrorCode::UnsupportedValue,0,"only auto is implemented");
    return Tuple("auto");
  }
  const auto n=integer?double(v.scalar_integer()):v.scalar_number();
  if(n<0 || (integer&&n>30))v.fail(ErrorCode::InvalidValue,0,"nonnegative control within the supported range required");
  return integer?Tuple(static_cast<std::int64_t>(n)):Tuple(n);
}
Tuple optimization(::MvTuple input,const char* op){
  const TupleInput v(input,op,"Optimization");if(v.size()<1||v.size()>2)v.fail(ErrorCode::InvalidCardinality,-1,"one or two optimization values required");
  Tuple output;std::string_view reduction;bool no_pregeneration=false;
  for(std::size_t i=0;i<v.size();++i){const auto name=v.string(i);
    if(name=="pregeneration")v.fail(ErrorCode::UnsupportedValue,i,"eager model pregeneration is not implemented");
    if(name=="no_pregeneration"){no_pregeneration=true;continue;}
    if(name!="none"&&name!="auto"&&name!="point_reduction_low"&&name!="point_reduction_medium"&&name!="point_reduction_high")v.fail(ErrorCode::UnsupportedValue,i,"unsupported optimization");
    reduction=name;
  }
  // 私有 XLD 解析器要求一个 reduction 值；重复正式控制按输入顺序处理，与图像解析器一致。
  if(!reduction.empty())output.append(reduction);
  if(no_pregeneration)output.append("no_pregeneration");
  if(output.empty())output.append("none");return output;
}
std::string metric(::MvTuple input,const char* op,bool xld){
  const TupleInput v(input,op,"Metric");if(v.size()!=1)v.fail(ErrorCode::InvalidCardinality,-1,"exactly one metric required");
  const auto name=v.string(0);
  if(name!="use_polarity"&&name!="ignore_global_polarity"&&name!="ignore_local_polarity"&&(!(!xld&&name=="ignore_color_polarity")))v.fail(ErrorCode::UnsupportedValue,0,"unsupported metric");
  return std::string(name);
}
Tuple contrast(::MvTuple input,const char* op){
  const TupleInput v(input,op,"Contrast");
  if(v.size()<1||v.size()>3)v.fail(ErrorCode::InvalidCardinality,-1,"one to three contrast values required");
  Tuple output;
  for(std::size_t i=0;i<v.size();++i){
    if(v.at(i).kind()==mv::core::ValueKind::String){const auto s=v.string(i);
      if(s!="auto"&&s!="auto_contrast"&&s!="auto_contrast_hyst"&&s!="auto_min_size")v.fail(ErrorCode::UnsupportedValue,i,"unsupported contrast selector");
      if(s=="auto_min_size"&&v.size()==1)v.fail(ErrorCode::InvalidCardinality,-1,"auto_min_size requires contrast thresholds");output.append(s);
    }else {const auto n=v.number(i);if(!std::isfinite(n)||n<0||n>2147483647.)v.fail(ErrorCode::InvalidValue,i,"finite nonnegative contrast within the supported range required");output.append(std::round(n));}
  }
  return output;
}
void create(::MvObject source,bool xld,bool scaled,::MvTuple levels,::MvTuple start,::MvTuple extent,
    ::MvTuple step,::MvTuple lo,::MvTuple hi,::MvTuple scale_step,::MvTuple opt,::MvTuple met,
    ::MvTuple con,::MvTuple minc,::MvTuple* out,const char* op){
  mv::c_api::ControlOutputs<1> output(op,{out});iconic(source,op,xld);
  if(source->value.empty()){output.Commit();return;}
  auto n=auto_number(levels,op,"NumLevels",true);
  const auto a=TupleInput(start,op,"AngleStart").scalar_number();
  const TupleInput ec(extent,op,"AngleExtent");const auto e=ec.scalar_number();
  if(e<0||e>2*std::numbers::pi)ec.fail(ErrorCode::InvalidValue,0,"angle extent must be within 0..2*pi");
  auto as=auto_number(step,op,"AngleStep",false);
  double lower=1,upper=1;Tuple ss;
  if(scaled){const TupleInput lc(lo,op,"ScaleMin"),hc(hi,op,"ScaleMax");lower=lc.scalar_number();upper=hc.scalar_number();
    if(lower<=0)lc.fail(ErrorCode::InvalidValue,0,"positive scale minimum required");
    if(upper<lower)hc.fail(ErrorCode::InvalidValue,0,"scale maximum must be at least minimum");
    ss=auto_number(scale_step,op,"ScaleStep",false);}
  auto os=optimization(opt,op);const auto m=metric(met,op,xld);
  auto mc=auto_number(minc,op,"MinContrast",false,!xld);
  if(mc.at(0).kind()!=mv::core::ValueKind::String)mc=Tuple(std::round(mc.as_real64()));
  auto model=std::make_unique<MvShapeModelImpl>();
  if(xld){const auto value=mv::core::XldContours{source->value};
    model->value=algorithm(op,[&]{return scaled?
      mv::shape_matching::create_scaled_shape_model_xld(value,n,a,e,as,lower,upper,ss,os,m,mc.as_real64()):
      mv::shape_matching::create_shape_model_xld(value,n,a,e,as,os,m,mc.as_real64());});
  }else {auto c=contrast(con,op);const auto& image=source->value.at(0).as_image();
    if(image.channel_count()!=1)throw mv::core::Exception(mv::core::Status::Unsupported,ErrorCode::UnsupportedValue,op,"Template",0,"multichannel Shape creation is not implemented");
    model->value=algorithm(op,[&]{return scaled?
      mv::shape_matching::create_scaled_shape_model(image,n,a,e,as,lower,upper,ss,os,m,c,mc):
      mv::shape_matching::create_shape_model(image,n,a,e,as,os,m,c,mc);});
  }
  output[0].append(mv::core::Handle::make<mv::c_api::ShapeModelObject>(model.get()));
  mv::c_api::release_opaque(model.release());output.Commit();
}
}
extern "C" ::MvStatus mv_create_shape_model(::MvObject Template,::MvTuple NumLevels,::MvTuple AngleStart,
    ::MvTuple AngleExtent,::MvTuple AngleStep,::MvTuple Optimization,::MvTuple Metric,
    ::MvTuple Contrast,::MvTuple MinContrast,::MvTuple* ModelID){
  return mv::c_api::translate([&]{create(Template,false,false,NumLevels,AngleStart,AngleExtent,AngleStep,{}, {}, {},Optimization,Metric,Contrast,MinContrast,ModelID,"create_shape_model");});
}
extern "C" ::MvStatus mv_create_scaled_shape_model(::MvObject Template,::MvTuple NumLevels,::MvTuple AngleStart,
    ::MvTuple AngleExtent,::MvTuple AngleStep,::MvTuple ScaleMin,::MvTuple ScaleMax,::MvTuple ScaleStep,
    ::MvTuple Optimization,::MvTuple Metric,::MvTuple Contrast,::MvTuple MinContrast,::MvTuple* ModelID){
  return mv::c_api::translate([&]{create(Template,false,true,NumLevels,AngleStart,AngleExtent,AngleStep,ScaleMin,ScaleMax,ScaleStep,Optimization,Metric,Contrast,MinContrast,ModelID,"create_scaled_shape_model");});
}
extern "C" ::MvStatus mv_create_shape_model_xld(::MvObject Contours,::MvTuple NumLevels,::MvTuple AngleStart,
    ::MvTuple AngleExtent,::MvTuple AngleStep,::MvTuple Optimization,::MvTuple Metric,::MvTuple MinContrast,::MvTuple* ModelID){
  return mv::c_api::translate([&]{create(Contours,true,false,NumLevels,AngleStart,AngleExtent,AngleStep,{}, {}, {},Optimization,Metric,{},MinContrast,ModelID,"create_shape_model_xld");});
}
extern "C" ::MvStatus mv_create_scaled_shape_model_xld(::MvObject Contours,::MvTuple NumLevels,::MvTuple AngleStart,
    ::MvTuple AngleExtent,::MvTuple AngleStep,::MvTuple ScaleMin,::MvTuple ScaleMax,::MvTuple ScaleStep,
    ::MvTuple Optimization,::MvTuple Metric,::MvTuple MinContrast,::MvTuple* ModelID){
  return mv::c_api::translate([&]{create(Contours,true,true,NumLevels,AngleStart,AngleExtent,AngleStep,ScaleMin,ScaleMax,ScaleStep,Optimization,Metric,{},MinContrast,ModelID,"create_scaled_shape_model_xld");});
}
