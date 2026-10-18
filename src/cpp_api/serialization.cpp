#include "api_bridge.hpp"

namespace mv {
void SerializeHandle(const MvTuple& Handle, MvTuple* SerializedItem) {
  cpp_api::TupleOutputs<1> output("SerializeHandle", {SerializedItem});
  cpp_api::check_status(mv_serialize_handle(Handle.ControlInput(), output.Out(0)), "SerializeHandle");
  output.Commit();
}
void DeserializeHandle(const MvTuple& SerializedItem, MvTuple* Handle) {
  cpp_api::TupleOutputs<1> output("DeserializeHandle", {Handle});
  cpp_api::check_status(mv_deserialize_handle(SerializedItem.ControlInput(), output.Out(0)), "DeserializeHandle");
  output.Commit();
}
void CreateSerializedItemPtr(const MvTuple& Pointer, const MvTuple& Size, const MvTuple& Copy,
                             MvTuple* SerializedItemHandle) {
  cpp_api::TupleOutputs<1> output("CreateSerializedItemPtr", {SerializedItemHandle});
  cpp_api::check_status(mv_create_serialized_item_ptr(Pointer.ControlInput(), Size.ControlInput(),
      Copy.ControlInput(), output.Out(0)), "CreateSerializedItemPtr");
  output.Commit();
}
void GetSerializedItemPtr(const MvTuple& SerializedItemHandle, MvTuple* Pointer, MvTuple* Size) {
  cpp_api::TupleOutputs<2> output("GetSerializedItemPtr", {Pointer, Size});
  cpp_api::check_status(mv_get_serialized_item_ptr(SerializedItemHandle.ControlInput(),
      output.Out(0), output.Out(1)), "GetSerializedItemPtr");
  output.Commit();
}
MvSerializedItem MvHandle::SerializeHandle() const {
  MvTuple output;
  mv::SerializeHandle(MvTuple(*this), &output);
  return MvSerializedItem(output.H(0));
}
void MvHandle::DeserializeHandle(const MvSerializedItem& SerializedItem) {
  MvTuple output;
  mv::DeserializeHandle(MvTuple(SerializedItem), &output);
  *this = output.H(0);
}
MvSerializedItem::MvSerializedItem() noexcept = default;
MvSerializedItem::MvSerializedItem(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::SerializedItem)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvSerializedItem",
                      "SerializedItemHandle", -1, "control handle is not a serialized item");
}
MvSerializedItem::MvSerializedItem(void* Pointer, std::int64_t Size, const MvString& Copy) {
  CreateSerializedItemPtr(Pointer, Size, Copy);
}
MvSerializedItem::MvSerializedItem(void* Pointer, std::int64_t Size, const char* Copy)
    : MvSerializedItem(Pointer, Size, MvString(Copy)) {}
MvSerializedItem::MvSerializedItem(void* Pointer, std::int64_t Size, const wchar_t* Copy)
    : MvSerializedItem(Pointer, Size, MvString(Copy)) {}
void MvSerializedItem::CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const MvString& Copy) {
  MvTuple output;
  mv::CreateSerializedItemPtr(MvTuple(static_cast<std::int64_t>(reinterpret_cast<std::intptr_t>(Pointer))),
      MvTuple(Size), MvTuple(Copy), &output);
  *this = MvSerializedItem(output.H(0));
}
void MvSerializedItem::CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const char* Copy) {
  CreateSerializedItemPtr(Pointer, Size, MvString(Copy));
}
void MvSerializedItem::CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const wchar_t* Copy) {
  CreateSerializedItemPtr(Pointer, Size, MvString(Copy));
}
void* MvSerializedItem::GetSerializedItemPtr(std::int64_t* Size) const {
  cpp_api::require_output(Size, "GetSerializedItemPtr");
  MvTuple pointer, size;
  mv::GetSerializedItemPtr(MvTuple(*this), &pointer, &size);
  const auto address = static_cast<std::intptr_t>(pointer.I(0));
  *Size = size.I(0);
  return reinterpret_cast<void*>(address);
}
}  // namespace mv
