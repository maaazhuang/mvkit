#include <mvkit/c/mv_handle.h>
#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "serialized_item.hpp"
#include <shape_matching/operators.hpp>
#include <limits>

namespace {
constexpr std::size_t max_bytes = 256U * 1024U * 1024U + 128;
using mv::c_api::TupleInput;
using mv::core::ErrorCode;

// 私有编解码错误转换为公开算子及参数，保持结构化错误边界。
template<class Fn> decltype(auto) algorithm(const char* op, const char* parameter, Fn&& fn) {
  try { return std::forward<Fn>(fn)(); }
  catch (const mv::core::Exception& error) {
    throw mv::core::Exception(error.status(), error.error_code(), op, parameter, 0, error.message());
  }
}

mv::core::Handle single_handle(MvTuple tuple, const char* op, const char* parameter) {
  const TupleInput input(tuple, op, parameter);
  if (input.size() != 1) input.fail(ErrorCode::InvalidCardinality, -1, "exactly one handle required");
  if (input.kind_at(0) != mv::core::ValueKind::Handle) input.fail(ErrorCode::InvalidType, 0, "handle required");
  const auto handle = input.handle(0);
  if (!handle.is_valid())
    throw mv::core::Exception(mv::core::Status::InvalidState, ErrorCode::InvalidObjectState,
                             op, parameter, 0, "handle is empty or cleared");
  return handle;
}
template<class Wrapper, class Opaque, class Value>
mv::core::Handle model_handle(Value value,std::shared_ptr<const mv::c_api::ModelSnapshot> snapshot={}) {
  auto model = std::make_unique<Opaque>();
  model->value = std::move(value);
  if constexpr(requires {model->snapshot;})model->snapshot=std::move(snapshot);
  auto handle = mv::core::Handle::make<Wrapper>(model.get());
  mv::c_api::release_opaque(model.release());
  return handle;
}
template<class Model,class Wrapper,class Opaque,class Decode>
mv::core::Handle restore_prepared(mv::c_api::SerializedItemObject& item,const std::string& bytes,Decode decode) {
  std::lock_guard lock(item.prepare_mutex);
  if(item.prepared)
    return model_handle<Wrapper,Opaque>(mv::c_api::clone_model(std::get<Model>(item.prepared->model)),item.prepared);
  auto value=decode(bytes);
  if(bytes.size()<=4u*1024u*1024u && mv::c_api::snapshot_budget(value))
    item.prepared=std::make_shared<mv::c_api::ModelSnapshot>(bytes,mv::c_api::clone_model(value));
  return model_handle<Wrapper,Opaque>(std::move(value),item.prepared);
}
}  // namespace

extern "C" MvStatus mv_serialize_handle(MvTuple Handle, MvTuple* SerializedItem) {
  return mv::c_api::translate([&] {
    constexpr auto op = "serialize_handle";
    mv::c_api::ControlOutputs<1> output(op, {SerializedItem});
    const auto handle = single_handle(Handle, op, "Handle");
    std::string bytes;
    std::shared_ptr<const mv::c_api::ModelSnapshot> snapshot;
    if (handle.type_id() == mv::c_api::ShapeModelObject::kTypeId)
      snapshot = algorithm(op, "Handle", [&] { return mv::c_api::encoded_snapshot(*handle.object<mv::c_api::ShapeModelObject>().handle(),bytes); });
    else if (handle.type_id() == mv::c_api::NccModelObject::kTypeId)
      snapshot = algorithm(op, "Handle", [&] { return mv::c_api::encoded_snapshot(*handle.object<mv::c_api::NccModelObject>().handle(),bytes); });
    else throw mv::core::Exception(mv::core::Status::Unsupported, ErrorCode::UnsupportedValue,
                                  op, "Handle", 0, "handle type does not support serialization");
    if(snapshot)output[0].append(mv::core::Handle::make<mv::c_api::SerializedItemObject>(std::move(snapshot)));
    else output[0].append(mv::core::Handle::make<mv::c_api::SerializedItemObject>(std::move(bytes)));
    output.Commit();
  });
}

extern "C" MvStatus mv_deserialize_handle(MvTuple SerializedItem, MvTuple* Handle) {
  return mv::c_api::translate([&] {
    constexpr auto op = "deserialize_handle";
    mv::c_api::ControlOutputs<1> output(op, {Handle});
    const auto handle = single_handle(SerializedItem, op, "SerializedItem");
    if (handle.type_id() != mv::c_api::SerializedItemObject::kTypeId)
      throw mv::core::Exception(mv::core::Status::TypeMismatch, ErrorCode::InvalidType,
                               op, "SerializedItem", 0, "serialized item required");
    auto& item=handle.object<mv::c_api::SerializedItemObject>();
    const auto& bytes = item.bytes();
    mv::core::Handle restored;
    if (bytes.starts_with("MVKIT_SHAPE "))
      restored = algorithm(op,"SerializedItem",[&]{return restore_prepared<mv::shape_matching::ShapeModel,mv::c_api::ShapeModelObject,MvShapeModelImpl>(item,bytes,mv::shape_matching::deserialize_model);});
    else if (bytes.starts_with("MVKIT_NCC "))
      restored = algorithm(op,"SerializedItem",[&]{return restore_prepared<mv::ncc_matching::Model,mv::c_api::NccModelObject,MvNCCModelImpl>(item,bytes,mv::ncc_matching::deserialize);});
    else throw mv::core::Exception(mv::core::Status::DecodeError, ErrorCode::IoFailure,
                                  op, "SerializedItem", 0, "unknown serialized model signature");
    output[0].append(std::move(restored)); output.Commit();
  });
}

extern "C" MvStatus mv_create_serialized_item_ptr(MvTuple Pointer, MvTuple Size, MvTuple Copy,
                                                   MvTuple* SerializedItemHandle) {
  return mv::c_api::translate([&] {
    constexpr auto op = "create_serialized_item_ptr";
    mv::c_api::ControlOutputs<1> output(op, {SerializedItemHandle});
    const TupleInput pointer(Pointer, op, "Pointer"), size(Size, op, "Size"), copy(Copy, op, "Copy");
    const auto address = static_cast<std::uintptr_t>(pointer.scalar_integer());
    const auto length = size.scalar_integer();
    if (copy.size() != 1) copy.fail(ErrorCode::InvalidCardinality, -1, "one copy mode required");
    if (copy.string(0) != "true")
      throw mv::core::Exception(mv::core::Status::Unsupported, ErrorCode::UnsupportedValue,
                               op, "Copy", 0, "only owned copy mode true is supported");
    if (length <= 0) size.fail(ErrorCode::InvalidValue, 0, "positive byte count required");
    if (static_cast<std::uint64_t>(length) > max_bytes)
      throw mv::core::Exception(mv::core::Status::LimitExceeded, ErrorCode::InvalidValue,
                               op, "Size", 0, "serialized item exceeds size limit");
    if (!address || address > std::numeric_limits<std::uintptr_t>::max() - static_cast<std::size_t>(length))
      pointer.fail(ErrorCode::InvalidValue, 0, "non-null address without range overflow required");
    std::string bytes(reinterpret_cast<const char*>(address), static_cast<std::size_t>(length));
    output[0].append(mv::core::Handle::make<mv::c_api::SerializedItemObject>(std::move(bytes)));
    output.Commit();
  });
}

extern "C" MvStatus mv_get_serialized_item_ptr(MvTuple SerializedItemHandle, MvTuple* Pointer,
                                                MvTuple* Size) {
  return mv::c_api::translate([&] {
    constexpr auto op = "get_serialized_item_ptr";
    mv::c_api::ControlOutputs<2> output(op, {Pointer, Size});
    const auto handle = single_handle(SerializedItemHandle, op, "SerializedItemHandle");
    if (handle.type_id() != mv::c_api::SerializedItemObject::kTypeId)
      throw mv::core::Exception(mv::core::Status::TypeMismatch, ErrorCode::InvalidType,
                               op, "SerializedItemHandle", 0, "serialized item required");
    const auto& bytes = handle.object<mv::c_api::SerializedItemObject>().bytes();
    output[0].append(static_cast<std::int64_t>(reinterpret_cast<std::intptr_t>(bytes.data())));
    output[1].append(static_cast<std::int64_t>(bytes.size())); output.Commit();
  });
}

