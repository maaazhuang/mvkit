#include <mvkit/c/mv_stereo.h>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "stereo_model_control.hpp"
#include "tuple_input.hpp"
namespace {
std::shared_ptr<mv::c_api::StereoModelObject> model(const mv::c_api::TupleInput& input) {
  if (input.size() != 1)
    input.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one stereo model required");
  if (input.at(0).kind() != mv::core::ValueKind::Handle)
    input.fail(mv::core::ErrorCode::InvalidType, 0, "stereo model handle required");
  const auto handle = input.at(0).as_handle();
  if (handle.type_id() != mv::c_api::StereoModelObject::kTypeId)
    input.fail(mv::core::ErrorCode::InvalidType, 0, "handle must refer to an actual stereo model");
  return handle.as<mv::c_api::StereoModelObject>();
}
}  // namespace
extern "C" ::MvStatus mv_create_stereo_model(::MvTuple CameraSetupModelID, ::MvTuple Method,
                                           ::MvTuple GenParamName, ::MvTuple GenParamValue,
                                           ::MvTuple* StereoModelID) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_create_stereo_model";
    mv::c_api::ControlOutputs<1> out(op, {StereoModelID});
    const mv::c_api::TupleInput setup(CameraSetupModelID, op, "CameraSetupModelID"),
        method(Method, op, "Method"), names(GenParamName, op, "GenParamName"),
        values(GenParamValue, op, "GenParamValue");
    if (setup.size() != 1)
      setup.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one camera setup required");
    if (method.size() != 1)
      method.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one method required");
    if (setup.at(0).kind() != mv::core::ValueKind::Handle)
      setup.fail(mv::core::ErrorCode::InvalidType, 0, "camera setup handle required");
    const auto handle = setup.at(0).as_handle();
    if (handle.type_id() != mv::c_api::CameraSetupModelObject::kTypeId)
      setup.fail(mv::core::ErrorCode::InvalidType, 0, "camera setup model required");
    const auto camera = handle.as<mv::c_api::CameraSetupModelObject>();
    if (!camera->handle()->value)
      setup.fail(mv::core::ErrorCode::InvalidObjectState, 0, "camera setup is not initialized");
    out[0].append(mv::core::Handle::make<mv::c_api::StereoModelObject>(
        *camera->handle()->value, method.string(0), names.materialize(), values.materialize()));
    out.Commit();
  });
}
extern "C" ::MvStatus mv_set_stereo_model_param(::MvTuple StereoModelID, ::MvTuple GenParamName,
                                              ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_stereo_model_param";
    const mv::c_api::TupleInput id(StereoModelID, op, "StereoModelID"),
        names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    model(id)->value.set_parameters(names.materialize(), values.materialize());
  });
}
extern "C" ::MvStatus mv_clear_stereo_model(::MvTuple StereoModelID) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput id(StereoModelID, "mv_clear_stereo_model", "StereoModelID");
    model(id)->clear();
  });
}
extern "C" ::MvStatus mv_reconstruct_points_stereo(::MvTuple StereoModelID, ::MvTuple Row,
                                                 ::MvTuple Column, ::MvTuple CovIP,
                                                 ::MvTuple CameraIdx, ::MvTuple PointIdx, ::MvTuple* X,
                                                 ::MvTuple* Y, ::MvTuple* Z, ::MvTuple* CovWP,
                                                 ::MvTuple* PointIdxOut) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_reconstruct_points_stereo";
    mv::c_api::ControlOutputs<5> out(op, {X, Y, Z, CovWP, PointIdxOut});
    const mv::c_api::TupleInput id(StereoModelID, op, "StereoModelID"), rows(Row, op, "Row"),
        columns(Column, op, "Column"), cov(CovIP, op, "CovIP"), cameras(CameraIdx, op, "CameraIdx"),
        points(PointIdx, op, "PointIdx");
    auto result = model(id)->value.reconstruct(rows.materialize(), columns.materialize(), cov.materialize(),
                                               cameras.materialize(), points.materialize());
    out[0] = std::move(result.x);
    out[1] = std::move(result.y);
    out[2] = std::move(result.z);
    out[3] = std::move(result.covariance);
    out[4] = std::move(result.indices);
    out.Commit();
  });
}
