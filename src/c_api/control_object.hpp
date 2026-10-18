#pragma once

#include <atomic>
#include <memory>
#include <utility>

#include "opaque.hpp"
#include "tuple_input.hpp"

#include <core/exception.hpp>
#include <core/handle.hpp>

namespace mv::c_api {

template <class Opaque>
void retain_opaque(Opaque* handle) noexcept {
  handle->refs.fetch_add(1, std::memory_order_relaxed);
}

template <class Opaque>
void release_opaque(Opaque* handle) noexcept {
  if (handle != nullptr && handle->refs.fetch_sub(1, std::memory_order_acq_rel) == 1U) {
    delete handle;
  }
}

template <class Opaque, core::TypeId Type>
class ControlObject final : public core::Object {
 public:
  static constexpr core::TypeId kTypeId = Type;

  explicit ControlObject(Opaque* handle) noexcept : handle_(handle) { retain_opaque(handle_); }
  ~ControlObject() override { release_opaque(handle_); }

  [[nodiscard]] core::TypeId type_id() const noexcept override { return kTypeId; }
  [[nodiscard]] bool is_valid() const noexcept override { return handle_->valid.load(); }
  [[nodiscard]] Opaque* handle() const {
    if (!is_valid()) throw core::Exception(core::Status::InvalidState, core::ErrorCode::InvalidObjectState,
                                          "control handle", "ModelID", -1, "handle has been cleared");
    return handle_;
  }
  void clear() override {
    if (!handle_->valid.exchange(false))
      throw core::Exception(core::Status::InvalidState, "clear_handle", "handle has already been cleared");
    handle_->value = decltype(handle_->value){};
    if constexpr (requires { handle_->coordinate_columns; }) handle_->coordinate_columns.reset();
    if constexpr (requires { handle_->nearest_index; }) handle_->nearest_index.reset();
    if constexpr (requires { handle_->snapshot; }) handle_->snapshot.reset();
  }

 private:
  Opaque* handle_;
};

using CalibrationDataObject =
    ControlObject<MvCalibDataImpl, core::make_type_id("mvkit.control.CalibrationData")>;
using ShapeModelObject =
    ControlObject<MvShapeModelImpl, core::make_type_id("mvkit.control.ShapeModel")>;
using NccModelObject =
    ControlObject<MvNCCModelImpl, core::make_type_id("mvkit.control.NccModel")>;
using DictObject = ControlObject<MvDictImpl, core::make_type_id("mvkit.control.Dict")>;
using ObjectModel3DObject =
    ControlObject<MvObjectModel3DImpl, core::make_type_id("mvkit.control.ObjectModel3D")>;
using CameraSetupModelObject =
    ControlObject<MvCameraSetupModelImpl, core::make_type_id("mvkit.control.CameraSetupModel")>;
using StereoRectificationObject = ControlObject<
    MvStereoRectificationImpl, core::make_type_id("mvkit.control.StereoRectification")>;
using StructuredLightModelObject = ControlObject<
    MvStructuredLightModelImpl, core::make_type_id("mvkit.control.StructuredLightModel")>;

template <class Wrapper, class Opaque>
[[nodiscard]] ::MvHandle to_generic_object(Opaque* handle) {
  auto value = core::Handle::make<Wrapper>(handle);
  auto object = std::make_unique<MvHandleImpl>();
  object->value = std::move(value);
  return object.release();
}

template <class Wrapper>
[[nodiscard]] auto from_generic_object(::MvHandle object, const char* operation) {
  if (object == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, "object", -1, "object handle must not be null");
  }
  if (object->value.empty()) {
    throw core::Exception(core::Status::InvalidState, core::ErrorCode::InvalidObjectState,
                          operation, "object", -1, "object handle is empty");
  }
  if (object->value.type_id() != Wrapper::kTypeId) {
    throw core::Exception(core::Status::TypeMismatch, core::ErrorCode::InvalidType,
                          operation, "object", -1,
                          "object type does not match requested model type");
  }
  const auto wrapper = object->value.as<Wrapper>();
  auto* handle = wrapper->handle();
  retain_opaque(handle);
  return handle;
}

// 正式算子与扩展统一使用单元素控制元组，类型检查留在适配器内部。
template <class Wrapper>
[[nodiscard]] auto control_handle_from_tuple(::MvTuple tuple, const char* operation,
                                             const char* parameter = "ModelID") {
  const TupleInput input(tuple, operation, parameter);
  if (input.size() != 1)
    input.fail(core::ErrorCode::InvalidCardinality, -1, "exactly one control handle required");
  const auto value = input.at(0);
  if (value.kind() != core::ValueKind::Handle)
    input.fail(core::ErrorCode::InvalidType, 0, "control handle required");
  const auto& handle = value.as_handle();
  if (handle.type_id() != Wrapper::kTypeId)
    input.fail(core::ErrorCode::InvalidType, 0, "control handle type does not match");
  return handle.object<Wrapper>().handle();
}

}  // namespace mv::c_api
