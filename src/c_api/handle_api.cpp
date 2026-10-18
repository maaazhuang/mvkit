#include <mvkit/c/mv_handle.h>

#include <atomic>
#include "error_state.hpp"
#include "opaque.hpp"
#include "control_object.hpp"
#include "handle_bridge.hpp"
#include "tuple_input.hpp"
#include "stereo_model_control.hpp"
#include "serialized_item.hpp"
#include <core/exception.hpp>

namespace {
void require_object(::MvHandle object, const char* where) {
  if (object == nullptr) throw mv::core::Exception(
      mv::core::Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
      where, "object", -1, "object handle must not be null");
}
void require_output(const void* output, const char* where) {
  if (output == nullptr) throw mv::core::Exception(
      mv::core::Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
      where, "output", -1, "output pointer must not be null");
}
::MvObjectType public_type(mv::core::TypeId type) noexcept {
  using namespace mv::c_api;
  if (type == CalibrationDataObject::kTypeId) return MV_OBJECT_TYPE_CALIBRATION_DATA;
  if (type == ShapeModelObject::kTypeId) return MV_OBJECT_TYPE_SHAPE_MODEL;
  if (type == NccModelObject::kTypeId) return MV_OBJECT_TYPE_NCC_MODEL;
  if (type == DictObject::kTypeId) return MV_OBJECT_TYPE_DICT;
  if (type == StereoModelObject::kTypeId) return MV_OBJECT_TYPE_STEREO_MODEL;
  if (type == SerializedItemObject::kTypeId) return MV_OBJECT_TYPE_SERIALIZED_ITEM;
  if (type == ObjectModel3DObject::kTypeId) return MV_OBJECT_TYPE_OBJECT_MODEL_3D;
  if (type == CameraSetupModelObject::kTypeId) return MV_OBJECT_TYPE_CAMERA_SETUP_MODEL;
  if (type == StereoRectificationObject::kTypeId) return MV_OBJECT_TYPE_STEREO_RECTIFICATION;
  if (type == StructuredLightModelObject::kTypeId) return MV_OBJECT_TYPE_STRUCTURED_LIGHT_MODEL;
  return MV_OBJECT_TYPE_UNKNOWN;
}
}

extern "C" ::MvStatus mv_private_object_retain(::MvHandle object) {
  return mv::c_api::translate([&] { require_object(object, "mv_private_object_retain"); object->refs.fetch_add(1, std::memory_order_relaxed); });
}
extern "C" ::MvStatus mv_private_object_release(::MvHandle object) {
  return mv::c_api::translate([&] { require_object(object, "mv_private_object_release"); if (object->refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete object; });
}
extern "C" ::MvStatus mv_private_object_get_type(::MvHandle object, ::MvObjectType* type) {
  return mv::c_api::translate([&] {
    require_object(object, "mv_private_object_get_type");
    require_output(type, "mv_private_object_get_type");
    *type = object->value.is_valid()?public_type(object->value.type_id()):MV_OBJECT_TYPE_UNKNOWN;
  });
}

extern "C" bool mv_private_handle_is_valid(::MvHandle object) noexcept {
  return object && object->value.is_valid();
}
extern "C" ::MvStatus mv_clear_handle(::MvTuple Handle) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput input(Handle,"clear_handle","Handle");
    for(std::size_t i=0;i<input.size();++i) {
      const auto value=input.at(i);
      if(value.kind()!=mv::core::ValueKind::Handle) input.fail(mv::core::ErrorCode::InvalidType,i,"control handle required");
      value.as_handle().clear();
    }
  });
}
extern "C" ::MvStatus mv_tuple_sem_type(::MvTuple T,::MvTuple* SemType) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput input(T,"tuple_sem_type","T");
    require_output(SemType,"tuple_sem_type");
    const char* name="any";
    const auto storage_kind=input.homogeneous_kind();
    if(input.size() && storage_kind==mv::core::TupleKind::Int64)name="integer";
    else if(input.size() && storage_kind==mv::core::TupleKind::Real64)name="real";
    else if(input.size() && storage_kind==mv::core::TupleKind::String)name="string";
    else if(input.size()) {
      const auto kind=input.at(0).kind();bool same=true,valid=true;mv::core::TypeId type=0;
      for(std::size_t i=0;i<input.size();++i) {
        const auto v=input.at(i);
        if(v.kind()!=kind) {same=false;break;}
        if(kind==mv::core::ValueKind::Handle) {
          const auto handle=v.as_handle();valid=valid && handle.is_valid();
          if(i==0)type=handle.type_id();else if(type!=handle.type_id())valid=false;
        }
      }
      if(same) {
        if(kind==mv::core::ValueKind::Int64)name="integer";
        else if(kind==mv::core::ValueKind::Real64)name="real";
        else if(kind==mv::core::ValueKind::String)name="string";
        else if(kind==mv::core::ValueKind::Handle) {
          name="handle";
          if(valid) switch(public_type(type)) {
            case MV_OBJECT_TYPE_DICT:name="dict";break;
            case MV_OBJECT_TYPE_CALIBRATION_DATA:name="calib_data";break;
            case MV_OBJECT_TYPE_SERIALIZED_ITEM:name="serialized_item";break;
            case MV_OBJECT_TYPE_SHAPE_MODEL:name="shape_model";break;
            case MV_OBJECT_TYPE_NCC_MODEL:name="ncc_model";break;
            case MV_OBJECT_TYPE_CAMERA_SETUP_MODEL:name="camera_setup_model";break;
            case MV_OBJECT_TYPE_OBJECT_MODEL_3D:name="object_model_3d";break;
            case MV_OBJECT_TYPE_STRUCTURED_LIGHT_MODEL:name="structured_light_model";break;
            case MV_OBJECT_TYPE_STEREO_MODEL:name="stereo_model";break;
          }
        }
      }
    }
    auto result=std::make_unique<MvTupleImpl>();result->value.append(name);*SemType=result.release();
  });
}
