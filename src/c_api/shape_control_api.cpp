#include <mvkit/c/mv_shape_matching.h>
#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "xld_cache.hpp"
#include "tuple_input.hpp"
#include <shape_matching/operators.hpp>
#include <xld/operators.hpp>
#include "model_snapshot.hpp"
#include <core/model_io.hpp>

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
::MvShapeModel model(::MvTuple input,const char* op) {
  const TupleInput ids(input,op,"ModelID");
  if(ids.size()!=1)ids.fail(ErrorCode::InvalidCardinality,-1,"exactly one Shape model required");
  const auto checked=[&](const mv::core::Handle& h) {
    const auto* wrapper=h.object_if<mv::c_api::ShapeModelObject>();
    if(!wrapper)
      ids.fail(ErrorCode::InvalidType,0,"control handle is not a Shape model");
    return wrapper->handle();
  };
  // 同步调用期间输入元组拥有句柄；同质拥有型输入可直接借用，Mixed/外部值保持原检查。
  if(const auto* tuple=ids.owned_tuple();tuple && tuple->kind()==mv::core::TupleKind::Handle)
    return checked(tuple->handle_span().front());
  if(ids.kind_at(0)!=mv::core::ValueKind::Handle)ids.fail(ErrorCode::InvalidType,0,"Shape model handle required");
  return checked(ids.handle(0));
}
template<class Fn> decltype(auto) algorithm(const char* op,Fn&& fn) {
  try{return std::forward<Fn>(fn)();}
  catch(const mv::core::Exception& ex){throw mv::core::Exception(ex.status(),ex.error_code(),op,
      ex.parameter(),ex.parameter_index(),ex.message());}
}
std::string filename(::MvTuple input,const char* op) {
  const TupleInput view(input,op,"FileName");
  if(view.size()!=1)view.fail(ErrorCode::InvalidCardinality,-1,"exactly one file name required");
  const auto name=view.string(0);
  if(name.empty()||name.find('\0')!=std::string_view::npos)
    view.fail(ErrorCode::InvalidValue,0,"nonempty file name without NUL required");
  return std::string(name);
}
}
extern "C" ::MvStatus mv_get_shape_model_origin(::MvTuple ModelID,::MvTuple* Row,::MvTuple* Column) {
  return mv::c_api::translate([&]{constexpr auto op="get_shape_model_origin";
    mv::c_api::ControlOutputs<2> output(op,{Row,Column});const auto& m=model(ModelID,op)->value;
    output[0].append(m.origin_row()-m.default_origin_row());
    output[1].append(m.origin_column()-m.default_origin_column());output.Commit();});
}
extern "C" ::MvStatus mv_set_shape_model_origin(::MvTuple ModelID,::MvTuple Row,::MvTuple Column) {
  return mv::c_api::translate([&]{constexpr auto op="set_shape_model_origin";auto* target=model(ModelID,op);
    const auto row=TupleInput(Row,op,"Row").scalar_number(),column=TupleInput(Column,op,"Column").scalar_number();
    const auto previous_row=target->value.origin_row(),previous_column=target->value.origin_column();
    algorithm(op,[&]{target->value.set_origin_offset(row,column);});
    if(target->value.is_xld_model() || previous_row!=target->value.origin_row() || previous_column!=target->value.origin_column() ||
       std::signbit(previous_row)!=std::signbit(target->value.origin_row()) ||
       std::signbit(previous_column)!=std::signbit(target->value.origin_column()))target->snapshot.reset();});
}
extern "C" ::MvStatus mv_get_shape_model_contours(::MvObject* ModelContours,::MvTuple ModelID,::MvTuple Level) {
  return mv::c_api::translate([&]{constexpr auto op="get_shape_model_contours";
    if(!ModelContours)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::NullArgument,op,"ModelContours",-1,"output must not be null");
    const auto* m=model(ModelID,op);const TupleInput control(Level,op,"Level");
    const auto rounded=std::round(control.scalar_number());
    if(rounded<1||rounded>double(m->value.levels().size()))control.fail(ErrorCode::InvalidValue,0,"pyramid level must be in 1..NumLevels");
    const auto level=static_cast<std::int64_t>(rounded);
    auto contours=algorithm(op,[&]{return mv::shape_matching::get_shape_model_contours(m->value,level-1);});
    // HALCON 在原模型坐标系返回每一层轮廓。
    const auto scale=m->value.levels()[std::size_t(level-1)].pyramid_scale;
    if(scale!=1.)contours=algorithm(op,[&]{return mv::xld::affine_trans_contour_xld(contours,mv::core::Tuple{scale,0.,0.,0.,scale,0.});});
    *ModelContours=mv::c_api::make_xld_contours(contours);});
}
extern "C" ::MvStatus mv_get_shape_model_params(::MvTuple ModelID,::MvTuple* NumLevels,
    ::MvTuple* AngleStart,::MvTuple* AngleExtent,::MvTuple* AngleStep,::MvTuple* ScaleMin,
    ::MvTuple* ScaleMax,::MvTuple* ScaleStep,::MvTuple* Metric,::MvTuple* MinContrast) {
  return mv::c_api::translate([&]{constexpr auto op="get_shape_model_params";
    mv::c_api::ControlOutputs<9> output(op,{NumLevels,AngleStart,AngleExtent,AngleStep,ScaleMin,ScaleMax,ScaleStep,Metric,MinContrast});
    const auto& m=model(ModelID,op)->value;
    output[0].append(static_cast<std::int64_t>(m.levels().size()));
    output[1].append(m.angle_start());output[2].append(m.angle_extent());output[3].append(m.angle_extent()==0?0.:m.angle_step());
    output[4].append(m.scale_min());output[5].append(m.scale_max());output[6].append(m.scale_min()==m.scale_max()?0.:m.scale_step());
    output[7].append(std::string_view(m.metric_name()));if(!std::isfinite(m.min_contrast())||m.min_contrast()<0||m.min_contrast()>=9223372036854775808.0)
      throw mv::core::Exception(mv::core::Status::OutOfRange,ErrorCode::InvalidValue,op,"MinContrast",0,"stored contrast exceeds integer output range");
    output[8].append(static_cast<std::int64_t>(std::round(m.min_contrast())));output.Commit();});
}
extern "C" ::MvStatus mv_set_shape_model_param(::MvTuple ModelID,::MvTuple GenParamName,::MvTuple GenParamValue) {
  return mv::c_api::translate([&]{constexpr auto op="set_shape_model_param";auto* target=model(ModelID,op);
    const TupleInput names(GenParamName,op,"GenParamName"),values(GenParamValue,op,"GenParamValue");
    if(names.size()!=values.size())values.fail(ErrorCode::InvalidCardinality,-1,"parameter names and values must have equal lengths");
    for(std::size_t i=0;i<names.size();++i) {
      const auto name=names.string(i);
      if(name!="min_contrast"&&name!="timeout"&&name!="border_shape_models"&&name!="model_cache")
        names.fail(ErrorCode::UnsupportedValue,i,"Shape parameter is not implemented");
      if(name=="min_contrast"||name=="timeout") {
        const auto n=values.number(i);if(!std::isfinite(n)||(name=="min_contrast"&&n<0))values.fail(ErrorCode::InvalidValue,i,"finite value and nonnegative contrast required");
      } else {const auto text=values.string(i);
        if(text!="true"&&text!="false")
          values.fail(ErrorCode::UnsupportedValue,i,"unsupported parameter value");
      }
    }
    auto next=target->value;
    const mv::c_api::ControlTupleView ns(GenParamName,op,"GenParamName");
    mv::core::Tuple vs;for(std::size_t i=0;i<names.size();++i) {
      if(names.string(i)=="min_contrast")vs.append(std::round(values.number(i)));
      else vs.append(values.at(i));
    }
    algorithm(op,[&]{mv::shape_matching::set_shape_model_param(next,ns,vs);});target->value=std::move(next);target->snapshot.reset();});
}
extern "C" ::MvStatus mv_write_shape_model(::MvTuple ModelID,::MvTuple FileName) {
  return mv::c_api::translate([&]{constexpr auto op="write_shape_model";const auto* m=model(ModelID,op);
    const auto name=filename(FileName,op);algorithm(op,[&]{mv::shape_matching::write_model(m->value,name);});});
}
extern "C" ::MvStatus mv_read_shape_model(::MvTuple FileName,::MvTuple* ModelID) {
  return mv::c_api::translate([&]{constexpr auto op="read_shape_model";mv::c_api::ControlOutputs<1> output(op,{ModelID});
    const auto name=filename(FileName,op);auto m=std::make_unique<MvShapeModelImpl>();
    m->value=algorithm(op,[&]{return mv::c_api::read_snapshot<mv::shape_matching::ShapeModel>(
      mv::core::model_io::read_file(name,256u*1024u*1024u+128,"read_shape_model"),
      mv::shape_matching::deserialize_model,m->snapshot);});
    output[0].append(mv::core::Handle::make<mv::c_api::ShapeModelObject>(m.get()));
    mv::c_api::release_opaque(m.release());output.Commit();});
}
