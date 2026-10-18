#include "api_bridge.hpp"
#include <mvkit/c/mv_structured_light.h>

#include <mvkit/cpp/mv_reconstruction3d.hpp>
#include <mvkit/cpp/mv_structured_light.hpp>

namespace mv {

MvStructuredLightModel::MvStructuredLightModel() noexcept = default;
MvStructuredLightModel::MvStructuredLightModel(const MvStructuredLightModel&) = default;
MvStructuredLightModel::MvStructuredLightModel(MvStructuredLightModel&&) noexcept = default;
MvStructuredLightModel& MvStructuredLightModel::operator=(const MvStructuredLightModel&) = default;
MvStructuredLightModel& MvStructuredLightModel::operator=(MvStructuredLightModel&&) noexcept = default;
MvStructuredLightModel::~MvStructuredLightModel() noexcept = default;
MvStructuredLightModel::MvStructuredLightModel(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::StructuredLightModel)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvStructuredLightModel", "Handle", -1,
                      "control handle has wrong model type");
}

MvObjectModel3D::MvObjectModel3D() noexcept = default;
MvObjectModel3D::MvObjectModel3D(const MvObjectModel3D&) = default;
MvObjectModel3D::MvObjectModel3D(MvObjectModel3D&&) noexcept = default;
MvObjectModel3D& MvObjectModel3D::operator=(const MvObjectModel3D&) = default;
MvObjectModel3D& MvObjectModel3D::operator=(MvObjectModel3D&&) noexcept = default;
MvObjectModel3D::~MvObjectModel3D() noexcept = default;
MvObjectModel3D::MvObjectModel3D(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::ObjectModel3D)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvObjectModel3D", "Handle", -1,
                      "control handle has wrong model type");
}

void CreateStructuredLightModel(const MvTuple& ModelType, MvTuple* StructuredLightModel) {
  cpp_api::TupleOutputs<1> out("CreateStructuredLightModel", {StructuredLightModel});
  cpp_api::check_status(mv_create_structured_light_model(ModelType.ControlInput(), out.Out(0)),
                           "CreateStructuredLightModel");
  out.Commit();
}

void DecodeStructuredLightPattern(const MvObject& CameraImages, const MvTuple& StructuredLightModel) {
  cpp_api::check_status(
      mv_decode_structured_light_pattern(CameraImages.NativeHandle(), StructuredLightModel.ControlInput()),
      "DecodeStructuredLightPattern");
}

void GenStructuredLightPattern(MvObject* PatternImages, const MvTuple& StructuredLightModel) {
  cpp_api::require_output(PatternImages, "GenStructuredLightPattern");
  ::MvObject raw = nullptr;
  cpp_api::check_status(mv_gen_structured_light_pattern(&raw, StructuredLightModel.ControlInput()),
                           "GenStructuredLightPattern");
  auto staged = detail::MvObjectAccess::Adopt(raw);
  detail::MvObjectAccess::CheckOutput(PatternImages, staged);
  detail::MvObjectAccess::Commit(PatternImages, std::move(staged));
}

void GetStructuredLightModelParam(const MvTuple& StructuredLightModel, const MvTuple& GenParamName,
                                  MvTuple* GenParamValue) {
  cpp_api::TupleOutputs<1> out("GetStructuredLightModelParam", {GenParamValue});
  cpp_api::check_status(mv_get_structured_light_model_param(StructuredLightModel.ControlInput(),
                                                               GenParamName.ControlInput(), out.Out(0)),
                           "GetStructuredLightModelParam");
  out.Commit();
}

void GetStructuredLightObject(MvObject* Object, const MvTuple& StructuredLightModel,
                              const MvTuple& ObjectName) {
  cpp_api::require_output(Object, "GetStructuredLightObject");
  ::MvObject raw = nullptr;
  cpp_api::check_status(
      mv_get_structured_light_object(&raw, StructuredLightModel.ControlInput(), ObjectName.ControlInput()),
      "GetStructuredLightObject");
  auto staged = detail::MvObjectAccess::Adopt(raw);
  detail::MvObjectAccess::CheckOutput(Object, staged);
  detail::MvObjectAccess::Commit(Object, std::move(staged));
}

void ReadStructuredLightModel(const MvTuple& FileName, MvTuple* StructuredLightModel) {
  cpp_api::TupleOutputs<1> out("ReadStructuredLightModel", {StructuredLightModel});
  cpp_api::check_status(mv_read_structured_light_model(FileName.ControlInput(), out.Out(0)),
                           "ReadStructuredLightModel");
  out.Commit();
}

void ReconstructSurfaceStructuredLight(const MvTuple& StructuredLightModel, MvTuple* ObjectModel3D) {
  cpp_api::TupleOutputs<1> out("ReconstructSurfaceStructuredLight", {ObjectModel3D});
  cpp_api::check_status(
      mv_reconstruct_surface_structured_light(StructuredLightModel.ControlInput(), out.Out(0)),
      "ReconstructSurfaceStructuredLight");
  out.Commit();
}

void SetStructuredLightModelParam(const MvTuple& StructuredLightModel, const MvTuple& GenParamName,
                                  const MvTuple& GenParamValue) {
  cpp_api::check_status(
      mv_set_structured_light_model_param(StructuredLightModel.ControlInput(), GenParamName.ControlInput(),
                                          GenParamValue.ControlInput()),
      "SetStructuredLightModelParam");
}

void WriteStructuredLightModel(const MvTuple& StructuredLightModel, const MvTuple& FileName) {
  cpp_api::check_status(
      mv_write_structured_light_model(StructuredLightModel.ControlInput(), FileName.ControlInput()),
      "WriteStructuredLightModel");
}

void MvStructuredLightModel::CreateStructuredLightModel(const MvString& ModelType) {
  MvTuple result;
  mv::CreateStructuredLightModel(MvTuple(MvString(ModelType)), &result);
  MvHandle::operator=(result.H());
}

void MvStructuredLightModel::CreateStructuredLightModel(const char* ModelType) {
  MvTuple result;
  mv::CreateStructuredLightModel(MvTuple(MvString(ModelType)), &result);
  MvHandle::operator=(result.H());
}

void MvStructuredLightModel::CreateStructuredLightModel(const wchar_t* ModelType) {
  MvTuple result;
  mv::CreateStructuredLightModel(MvTuple(MvString(ModelType)), &result);
  MvHandle::operator=(result.H());
}

void MvStructuredLightModel::DecodeStructuredLightPattern(const MvImage& CameraImages) const {
  mv::DecodeStructuredLightPattern(CameraImages, MvTuple(*this));
}

MvImage MvStructuredLightModel::GenStructuredLightPattern() const {
  MvImage result;
  mv::GenStructuredLightPattern(&result, MvTuple(*this));
  return result;
}

MvTuple MvStructuredLightModel::GetStructuredLightModelParam(const MvTuple& GenParamName) const {
  MvTuple result;
  mv::GetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), &result);
  return result;
}

MvTuple MvStructuredLightModel::GetStructuredLightModelParam(const MvString& GenParamName) const {
  MvTuple result;
  mv::GetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), &result);
  return result;
}

MvTuple MvStructuredLightModel::GetStructuredLightModelParam(const char* GenParamName) const {
  MvTuple result;
  mv::GetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), &result);
  return result;
}

MvTuple MvStructuredLightModel::GetStructuredLightModelParam(const wchar_t* GenParamName) const {
  MvTuple result;
  mv::GetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), &result);
  return result;
}

MvObject MvStructuredLightModel::GetStructuredLightObject(const MvTuple& ObjectName) const {
  MvObject result;
  mv::GetStructuredLightObject(&result, MvTuple(*this), MvTuple(ObjectName));
  return result;
}

MvObject MvStructuredLightModel::GetStructuredLightObject(const MvString& ObjectName) const {
  MvObject result;
  mv::GetStructuredLightObject(&result, MvTuple(*this), MvTuple(ObjectName));
  return result;
}

MvObject MvStructuredLightModel::GetStructuredLightObject(const char* ObjectName) const {
  MvObject result;
  mv::GetStructuredLightObject(&result, MvTuple(*this), MvTuple(ObjectName));
  return result;
}

MvObject MvStructuredLightModel::GetStructuredLightObject(const wchar_t* ObjectName) const {
  MvObject result;
  mv::GetStructuredLightObject(&result, MvTuple(*this), MvTuple(ObjectName));
  return result;
}

void MvStructuredLightModel::ReadStructuredLightModel(const MvString& FileName) {
  MvTuple result;
  mv::ReadStructuredLightModel(MvTuple(MvString(FileName)), &result);
  MvHandle::operator=(result.H());
}

void MvStructuredLightModel::ReadStructuredLightModel(const char* FileName) {
  MvTuple result;
  mv::ReadStructuredLightModel(MvTuple(MvString(FileName)), &result);
  MvHandle::operator=(result.H());
}

void MvStructuredLightModel::ReadStructuredLightModel(const wchar_t* FileName) {
  MvTuple result;
  mv::ReadStructuredLightModel(MvTuple(MvString(FileName)), &result);
  MvHandle::operator=(result.H());
}

MvObjectModel3D MvStructuredLightModel::ReconstructSurfaceStructuredLight() const {
  MvTuple result;
  mv::ReconstructSurfaceStructuredLight(MvTuple(*this), &result);
  return MvObjectModel3D(result.H());
}

void MvStructuredLightModel::SetStructuredLightModelParam(const MvTuple& GenParamName,
                                                          const MvTuple& GenParamValue) const {
  mv::SetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), MvTuple(GenParamValue));
}

void MvStructuredLightModel::SetStructuredLightModelParam(const MvString& GenParamName,
                                                          std::int64_t GenParamValue) const {
  mv::SetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), MvTuple(GenParamValue));
}

void MvStructuredLightModel::SetStructuredLightModelParam(const char* GenParamName,
                                                          std::int64_t GenParamValue) const {
  mv::SetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), MvTuple(GenParamValue));
}

void MvStructuredLightModel::SetStructuredLightModelParam(const wchar_t* GenParamName,
                                                          std::int64_t GenParamValue) const {
  mv::SetStructuredLightModelParam(MvTuple(*this), MvTuple(GenParamName), MvTuple(GenParamValue));
}

void MvStructuredLightModel::WriteStructuredLightModel(const MvString& FileName) const {
  mv::WriteStructuredLightModel(MvTuple(*this), MvTuple(MvString(FileName)));
}

void MvStructuredLightModel::WriteStructuredLightModel(const char* FileName) const {
  mv::WriteStructuredLightModel(MvTuple(*this), MvTuple(MvString(FileName)));
}

void MvStructuredLightModel::WriteStructuredLightModel(const wchar_t* FileName) const {
  mv::WriteStructuredLightModel(MvTuple(*this), MvTuple(MvString(FileName)));
}

void MvObjectModel3D::ReconstructSurfaceStructuredLight(const MvStructuredLightModel& StructuredLightModel) {
  MvTuple result;
  mv::ReconstructSurfaceStructuredLight(MvTuple(StructuredLightModel), &result);
  MvHandle::operator=(result.H());
}

}  // namespace mv
