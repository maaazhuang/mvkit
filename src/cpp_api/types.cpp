#include "api_bridge.hpp"
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_image_processing.h>
#include <mvkit/c/mv_region.h>

#include <cstring>
#include <mvkit/cpp/mvkit.hpp>
#include "../c_api/tuple_bridge.hpp"
#include "../c_api/handle_bridge.hpp"
#include <new>
#include <string>
#include <utility>

namespace mv {
namespace {
[[noreturn]] void invalid(const char* op, const char* param, MvErrorCode code, const char* message,
                          std::int64_t index = -1) {
  throw MvException(code == MvErrorCode::InvalidType          ? MvStatus::TypeMismatch
                    : code == MvErrorCode::IndexOutOfRange    ? MvStatus::OutOfRange
                    : code == MvErrorCode::InvalidObjectState ? MvStatus::InvalidState
                                                            : MvStatus::InvalidArgument,
                    code, {op, std::strlen(op)}, {param, std::strlen(param)}, index,
                    {message, std::strlen(message)});
}
using cpp_api::check_status;
void check(::MvStatus status, const char* op) { check_status(status, op); }
template <class Call>
void output(MvObject* out, const char* op, Call&& call) {
  if (!out) invalid(op, "output", MvErrorCode::NullArgument, "output must not be null");
  ::MvObject value = nullptr;
  check(call(&value), op);
  detail::MvObjectAccess::Commit(out, detail::MvObjectAccess::Adopt(value));
}
std::string checked_string(MvStringView value) {
  if (!value.data && value.size)
    invalid("MvString", "value", MvErrorCode::NullArgument, "nonempty string requires data");
  return value.size ? std::string(value.data, value.size) : std::string{};
}
}  // namespace

struct MvString::Impl {
  explicit Impl(MvStringView value) : text(checked_string(value)) {}
  std::string text;
};
MvString::MvString() noexcept = default;
MvString::MvString(const char* value) : MvString(MvStringView{value, value ? std::strlen(value) : 0}) {}
MvString::MvString(MvStringView value) : impl_(new Impl(value)) {}
MvString::MvString(const wchar_t* value) {
  std::string utf8;
  if (value)
    for (std::size_t i = 0; value[i]; ++i) {
      std::uint32_t c = static_cast<std::uint32_t>(value[i]);
      if (sizeof(wchar_t) == 2 && c >= 0xd800 && c <= 0xdbff) {
        const auto low = static_cast<std::uint32_t>(value[++i]);
        if (low < 0xdc00 || low > 0xdfff)
          invalid("MvString", "value", MvErrorCode::InvalidValue, "invalid UTF-16 surrogate", i - 1);
        c = 0x10000 + ((c - 0xd800) << 10) + (low - 0xdc00);
      }
      if ((c >= 0xd800 && c <= 0xdfff) || c > 0x10ffff)
        invalid("MvString", "value", MvErrorCode::InvalidValue, "invalid Unicode scalar", i);
      if (c < 0x80)
        utf8.push_back(static_cast<char>(c));
      else if (c < 0x800) {
        utf8.push_back(static_cast<char>(0xc0 | (c >> 6)));
        utf8.push_back(static_cast<char>(0x80 | (c & 63)));
      } else if (c < 0x10000) {
        utf8.push_back(static_cast<char>(0xe0 | (c >> 12)));
        utf8.push_back(static_cast<char>(0x80 | ((c >> 6) & 63)));
        utf8.push_back(static_cast<char>(0x80 | (c & 63)));
      } else {
        utf8.push_back(static_cast<char>(0xf0 | (c >> 18)));
        utf8.push_back(static_cast<char>(0x80 | ((c >> 12) & 63)));
        utf8.push_back(static_cast<char>(0x80 | ((c >> 6) & 63)));
        utf8.push_back(static_cast<char>(0x80 | (c & 63)));
      }
    }
  impl_ = new Impl({utf8.data(), utf8.size()});
}
MvString::MvString(const MvString& value) : impl_(value.impl_ ? new Impl(value.View()) : nullptr) {}
MvString::MvString(MvString&& value) noexcept : impl_(std::exchange(value.impl_, nullptr)) {}
MvString& MvString::operator=(const MvString& value) {
  if (this != &value) {
    MvString copy(value);
    std::swap(impl_, copy.impl_);
  }
  return *this;
}
MvString& MvString::operator=(MvString&& value) noexcept {
  if (this != &value) {
    delete impl_;
    impl_ = std::exchange(value.impl_, nullptr);
  }
  return *this;
}
MvString::~MvString() noexcept { delete impl_; }
const char* MvString::Text() const noexcept { return impl_ ? impl_->text.c_str() : ""; }
std::size_t MvString::Length() const noexcept { return impl_ ? impl_->text.size() : 0; }
MvStringView MvString::View() const noexcept { return {Text(), Length()}; }

struct MvException::Impl {
  Impl(MvStatus s, MvErrorCode c, MvStringView op, MvStringView param, std::int64_t i, MvStringView message)
      : status(s),
        code(c),
        operation(checked_string(op)),
        parameter(checked_string(param)),
        index(i),
        text(checked_string(message)) {
    if (!operation.empty()) {
      text = operation + (text.empty() ? std::string{} : ": " + text);
    }
  }
  MvStatus status;
  MvErrorCode code;
  std::string operation, parameter;
  std::int64_t index;
  std::string text;
};
MvException::MvException(MvStatus s, MvErrorCode c, MvStringView op, MvStringView param, std::int64_t i,
                         MvStringView message)
    : impl_(new Impl(s, c, op, param, i, message)) {}
MvException::MvException(const MvException& value) : impl_(value.impl_ ? new Impl(*value.impl_) : nullptr) {}
MvException::MvException(MvException&& value) noexcept : impl_(std::exchange(value.impl_, nullptr)) {}
MvException& MvException::operator=(const MvException& value) {
  if (this != &value) {
    MvException copy(value);
    std::swap(impl_, copy.impl_);
  }
  return *this;
}
MvException& MvException::operator=(MvException&& value) noexcept {
  if (this != &value) {
    delete impl_;
    impl_ = std::exchange(value.impl_, nullptr);
  }
  return *this;
}
MvException::~MvException() noexcept { delete impl_; }
MvStatus MvException::GetStatus() const noexcept { return impl_ ? impl_->status : MvStatus::InternalError; }
MvErrorCode MvException::GetErrorCode() const noexcept {
  return impl_ ? impl_->code : MvErrorCode::InternalFailure;
}
MvStringView MvException::GetOperatorName() const noexcept {
  return impl_ ? MvStringView{impl_->operation.data(), impl_->operation.size()} : MvStringView{"", 0};
}
MvStringView MvException::GetParameterName() const noexcept {
  return impl_ ? MvStringView{impl_->parameter.data(), impl_->parameter.size()} : MvStringView{"", 0};
}
std::int64_t MvException::GetParameterIndex() const noexcept { return impl_ ? impl_->index : -1; }
const char* MvException::what() const noexcept { return impl_ ? impl_->text.c_str() : ""; }

MvHandle::MvHandle() noexcept = default;
MvHandle::MvHandle(::MvHandle value, AdoptTag) noexcept : handle_(value) {}
MvHandle::MvHandle(const MvHandle& value) : handle_(value.handle_) {
  if (handle_) check(mv_private_object_retain(handle_), "MvHandle");
}
MvHandle::MvHandle(MvHandle&& value) noexcept : handle_(std::exchange(value.handle_, nullptr)) {}
MvHandle& MvHandle::operator=(const MvHandle& value) {
  if (this != &value) {
    MvHandle copy(value);
    std::swap(handle_, copy.handle_);
  }
  return *this;
}
MvHandle& MvHandle::operator=(MvHandle&& value) noexcept {
  if (this != &value) {
    Release();
    handle_ = std::exchange(value.handle_, nullptr);
  }
  return *this;
}
MvHandle::~MvHandle() noexcept { Release(); }
void MvHandle::Release() noexcept {
  if (handle_) {
    (void)mv_private_object_release(handle_);
    handle_ = nullptr;
  }
}
MvHandle MvHandle::FromNative(::MvHandle value) {
  check(mv_private_object_retain(value), "MvHandle::FromNative");
  return MvHandle(value, AdoptTag{});
}
bool MvHandle::IsInitialized() const noexcept { return mv_private_handle_is_valid(handle_); }
MvObjectType MvHandle::Type() const {
  if (!handle_) return MvObjectType::Unknown;
  ::MvObjectType type = MV_OBJECT_TYPE_UNKNOWN;
  check(mv_private_object_get_type(handle_, &type), "MvHandle::Type");
  return static_cast<MvObjectType>(type);
}
::MvHandle MvHandle::NativeHandle() const noexcept { return handle_; }

void MvTuple::InitInput() noexcept {
  // 公共类的 ABI 预留必须容纳 C DLL 中的实际借用包装器。
  static_assert(sizeof(input_storage_) >= kMvPrivateTupleInputSize, "C++ tuple conversion storage capacity");
  mv_private_tuple_input_init(input_storage_, nullptr, &scalar_, 1);
}
void MvTuple::DestroyInput() noexcept { mv_private_tuple_input_destroy(input_storage_); }
MvTuple::MvTuple() noexcept { InitInput(); }
MvTuple::MvTuple(int value) noexcept : MvTuple(static_cast<std::int64_t>(value)) {}
MvTuple::MvTuple(std::int64_t value) noexcept : MvTuple() {
  scalar_.kind = MV_VALUE_INT64;
  scalar_.data.int64_value = value;
}
MvTuple::MvTuple(double value) noexcept : MvTuple() {
  scalar_.kind = MV_VALUE_REAL64;
  scalar_.data.real64_value = value;
}
MvTuple::MvTuple(const char* value) : MvTuple() {
  // 直接建立拥有型字符串，避免先构造临时 MvString 再深复制一次。
  text_=new MvString(value);
  scalar_.kind=MV_VALUE_STRING;
  scalar_.data.string_value={text_->Text(),text_->Length()};
}
MvTuple::MvTuple(const MvString& value) : MvTuple() {
  text_=new MvString(value);
  scalar_.kind = MV_VALUE_STRING;
  scalar_.data.string_value = {text_->Text(), text_->Length()};
}
MvTuple::MvTuple(const MvHandle& value) : MvTuple() {
  ::MvHandle handle = value.NativeHandle();
  if (!handle) invalid("MvTuple", "handle", MvErrorCode::NullArgument, "handle must not be null");
  check(mv_private_object_retain(handle), "MvTuple");
  scalar_.kind = MV_VALUE_HANDLE;
  scalar_.data.handle_value = handle;
}
MvTuple::MvTuple(const MvTuple& value) : MvTuple() {
  scalar_=value.scalar_;
  if (value.tuple_)
    check(mv_copy_tuple(value.tuple_, &tuple_), "MvTuple");
  else if (value.text_) {
    text_ = new MvString(*value.text_);
    scalar_.data.string_value = {text_->Text(), text_->Length()};
  } else if (scalar_.kind == MV_VALUE_HANDLE)
    check(mv_private_object_retain(scalar_.data.handle_value), "MvTuple");
}
MvTuple::MvTuple(MvTuple&& value) noexcept : MvTuple() { Swap(value); }
MvTuple& MvTuple::operator=(const MvTuple& value) {
  if (this != &value) {
    MvTuple copy(value);
    Swap(copy);
  }
  return *this;
}
MvTuple& MvTuple::operator=(MvTuple&& value) noexcept {
  if (this != &value) {
    MvTuple copy(std::move(value));
    Swap(copy);
  }
  return *this;
}
MvTuple::~MvTuple() noexcept {
  DestroyInput();
  if (tuple_)
    (void)mv_destroy_tuple(tuple_);
  else if (scalar_.kind == MV_VALUE_HANDLE)
    (void)mv_private_object_release(scalar_.data.handle_value);
  delete text_;
}
void MvTuple::Swap(MvTuple& value) noexcept {
  std::swap(tuple_, value.tuple_);
  std::swap(text_, value.text_);
  std::swap(scalar_, value.scalar_);
}
::MvTuple MvTuple::ControlInput() const noexcept {
  if (tuple_) return tuple_;
  if (scalar_.kind==MV_VALUE_EMPTY) return nullptr;
  return const_cast<MvTupleImpl*>(reinterpret_cast<const MvTupleImpl*>(input_storage_));
}
std::size_t MvTuple::Length() const {
  if (!tuple_) return scalar_.kind == MV_VALUE_EMPTY ? 0 : 1;
  std::size_t size = 0;
  check(mv_length_tuple(tuple_, &size), "MvTuple::Length");
  return size;
}
double MvTuple::D(std::size_t index) const {
  if (tuple_) {
    double value = 0;
    check(mv_get_d(tuple_, index, &value), "MvTuple::D");
    return value;
  }
  if (index >= Length())
    invalid("MvTuple::D", "index", MvErrorCode::IndexOutOfRange, "index out of range", index);
  if (scalar_.kind == MV_VALUE_INT64) return static_cast<double>(scalar_.data.int64_value);
  if (scalar_.kind == MV_VALUE_REAL64) return scalar_.data.real64_value;
  invalid("MvTuple::D", "value", MvErrorCode::InvalidType, "numeric value required", index);
}
std::int64_t MvTuple::I(std::size_t index) const {
  if (tuple_) {
    std::int64_t value = 0;
    check(mv_get_i(tuple_, index, &value), "MvTuple::I");
    return value;
  }
  if (index >= Length())
    invalid("MvTuple::I", "index", MvErrorCode::IndexOutOfRange, "index out of range", index);
  if (scalar_.kind != MV_VALUE_INT64)
    invalid("MvTuple::I", "value", MvErrorCode::InvalidType, "integer value required", index);
  return scalar_.data.int64_value;
}
MvString MvTuple::S(std::size_t index) const {
  if (tuple_) {
    ::MvStringView value{};
    check(mv_get_s(tuple_, index, &value), "MvTuple::S");
    return MvString({value.data, value.size});
  }
  if (index >= Length())
    invalid("MvTuple::S", "index", MvErrorCode::IndexOutOfRange, "index out of range", index);
  if (scalar_.kind != MV_VALUE_STRING)
    invalid("MvTuple::S", "value", MvErrorCode::InvalidType, "string value required", index);
  return *text_;
}
MvHandle MvTuple::H(std::size_t index) const {
  if (tuple_) {
    ::MvHandle value = nullptr;
    check(mv_get_h(tuple_, index, &value), "MvTuple::H");
    return MvHandle::FromNative(value);
  }
  if (index >= Length())
    invalid("MvTuple::H", "index", MvErrorCode::IndexOutOfRange, "index out of range", index);
  if (scalar_.kind != MV_VALUE_HANDLE)
    invalid("MvTuple::H", "value", MvErrorCode::InvalidType, "handle value required", index);
  return MvHandle::FromNative(scalar_.data.handle_value);
}
::MvTuple MvTuple::Materialize() {
  if (tuple_) return tuple_;
  struct Owner {
    ::MvTuple value{nullptr};
    ~Owner() {
      if (value) (void)mv_destroy_tuple(value);
    }
  } candidate;
  check(mv_create_tuple(&candidate.value, 0), "MvTuple");
  switch (scalar_.kind) {
    case MV_VALUE_INT64:
      check(mv_private_tuple_append_i(candidate.value, scalar_.data.int64_value), "MvTuple");
      break;
    case MV_VALUE_REAL64:
      check(mv_private_tuple_append_d(candidate.value, scalar_.data.real64_value), "MvTuple");
      break;
    case MV_VALUE_STRING:
      check(mv_private_tuple_append_s(candidate.value, scalar_.data.string_value), "MvTuple");
      break;
    case MV_VALUE_HANDLE:
      check(mv_private_tuple_append_h(candidate.value, scalar_.data.handle_value), "MvTuple");
      break;
    default:
      break;
  }
  if (scalar_.kind == MV_VALUE_HANDLE) (void)mv_private_object_release(scalar_.data.handle_value);
  delete text_;
  text_ = nullptr;
  scalar_ = {};
  tuple_ = candidate.value;
  candidate.value = nullptr;
  return tuple_;
}
MvTuple& MvTuple::Append(int value) { return Append(static_cast<std::int64_t>(value)); }
MvTuple& MvTuple::Append(std::int64_t value) {
  check(mv_private_tuple_append_i(Materialize(), value), "MvTuple::Append");
  return *this;
}
MvTuple& MvTuple::Append(double value) {
  check(mv_private_tuple_append_d(Materialize(), value), "MvTuple::Append");
  return *this;
}
MvTuple& MvTuple::Append(const char* value) {
  // 输入可能指向元组内联字符串，Materialize 会释放该字符串。
  const MvString copy(value);
  check(mv_private_tuple_append_s(Materialize(), {copy.Text(), copy.Length()}), "MvTuple::Append");
  return *this;
}
MvTuple& MvTuple::Append(const MvHandle& value) {
  if (!value.NativeHandle())
    invalid("MvTuple::Append", "handle", MvErrorCode::NullArgument, "handle must not be null");
  check(mv_private_tuple_append_h(Materialize(), value.NativeHandle()), "MvTuple::Append");
  return *this;
}
MvTuple MvTuple::TupleConcat(const MvTuple& value) const {
  MvTuple result;
  mv::TupleConcat(*this, value, &result);
  return result;
}
MvTuple MvTuple::TupleSelect(const MvTuple& indices) const {
  MvTuple result;
  mv::TupleSelect(*this, indices, &result);
  return result;
}
MvTuple MvTuple::TupleSelectRange(const MvTuple& left, const MvTuple& right) const {
  MvTuple result;
  mv::TupleSelectRange(*this, left, right, &result);
  return result;
}

MvObject::MvObject() { check(mv_gen_empty_obj(&handle_), "GenEmptyObj"); }
MvObject::MvObject(::MvIconicKind kind) : MvObject() { kind_ = kind; }
MvObject::MvObject(const MvObject& v) {
  if(v.handle_) check(mv_copy_obj(v.handle_,&handle_,MvTuple(1).ControlInput(),MvTuple(-1).ControlInput()),"MvObject");
}
MvObject::MvObject(MvObject&& v) noexcept : handle_(std::exchange(v.handle_, nullptr)) {}
MvObject::MvObject(MvObject&& v, ::MvIconicKind kind) noexcept
    : handle_(std::exchange(v.handle_, nullptr)), kind_(kind) {}
MvObject::MvObject(const MvObject& v, ::MvIconicKind kind) : kind_(kind) {
  CheckKind(v.handle_);
  check(mv_copy_obj(v.handle_,&handle_,MvTuple(1).ControlInput(),MvTuple(-1).ControlInput()),"MvObject");
}
MvObject& MvObject::operator=(const MvObject& v) {
  if (this != &v) {
    CheckKind(v.handle_);
    ::MvObject copied=nullptr;
    check(mv_copy_obj(v.handle_,&copied,MvTuple(1).ControlInput(),MvTuple(-1).ControlInput()),"MvObject");
    Release();
    handle_ = copied;
  }
  return *this;
}
MvObject& MvObject::operator=(MvObject&& v) {
  if (this != &v) {
    CheckKind(v.handle_);
    Release();
    handle_ = std::exchange(v.handle_, nullptr);
  }
  return *this;
}
MvObject::~MvObject() noexcept { Release(); }
void MvObject::Release() noexcept {
  if (handle_) {
    (void)mv_clear_obj(handle_);
    handle_ = nullptr;
  }
}
void MvObject::CheckKind(::MvObject h) const {
  if (!h) invalid("MvObject", "objects", MvErrorCode::InvalidObjectState, "object is moved-from");
  if (kind_ < 0) return;
  ::MvTuple classes=nullptr;
  check(mv_get_obj_class(h,&classes),"MvObject");
  MvTuple values=detail::MvTupleAccess::Adopt(classes);
  const char* expected=kind_==MV_ICONIC_IMAGE?"image":kind_==MV_ICONIC_REGION?"region":"xld_cont";
  for(std::size_t i=0;i<values.Length();++i) {
    if(std::strcmp(values.S(i).Text(),expected)!=0)
      invalid("MvObject","objects",MvErrorCode::InvalidType,"iconic object kind mismatch",i);
  }
}
MvObject::MvObject(::MvObject h, AdoptTag) noexcept : handle_(h) {}
::MvObject MvObject::NativeHandle() const noexcept { return handle_; }
MvObject MvObject::FromNative(::MvObject h) {
  ::MvObject copied=nullptr;
  check(mv_copy_obj(h,&copied,MvTuple(1).ControlInput(),MvTuple(-1).ControlInput()),"MvObject::FromNative");
  return MvObject(copied, AdoptTag{});
}
std::int64_t MvObject::CountObj() const {
  MvTuple count;
  mv::CountObj(*this,&count);
  return count.I();
}
void MvObject::GenEmptyObj() { mv::GenEmptyObj(this); }
MvObject MvObject::SelectObj(const MvTuple& index) const {
  ::MvObject h = nullptr;
  check(mv_select_obj(handle_, &h, index.ControlInput()), "SelectObj");
  return MvObject(h, AdoptTag{});
}
MvObject MvObject::SelectObj(std::int64_t index) const { return SelectObj(MvTuple(index)); }
MvObject MvObject::ConcatObj(const MvObject& other) const {
  ::MvObject h = nullptr;
  check(mv_concat_obj(handle_, other.handle_, &h), "ConcatObj");
  return MvObject(h, AdoptTag{});
}
MvObject MvObject::CopyObj(std::int64_t i, std::int64_t n) const {
  MvTuple index(i), count(n);
  ::MvObject h = nullptr;
  check(mv_copy_obj(handle_, &h, index.ControlInput(), count.ControlInput()), "CopyObj");
  return MvObject(h, AdoptTag{});
}
MvImage::MvImage() : MvObject(MV_ICONIC_IMAGE) {}
MvImage::MvImage(const MvImage& v) : MvObject(v, MV_ICONIC_IMAGE) {}
MvImage::MvImage(MvImage&& v) noexcept : MvObject(std::move(v), MV_ICONIC_IMAGE) {}
MvImage& MvImage::operator=(const MvImage& v) {
  MvObject::operator=(v);
  return *this;
}
MvImage& MvImage::operator=(MvImage&& v) {
  MvObject::operator=(std::move(v));
  return *this;
}
MvImage::MvImage(const MvObject& v) : MvObject(v, MV_ICONIC_IMAGE) {}
MvRegion::MvRegion() : MvObject(MV_ICONIC_REGION) {}
MvRegion::MvRegion(const MvRegion& v) : MvObject(v, MV_ICONIC_REGION) {}
MvRegion::MvRegion(MvRegion&& v) noexcept : MvObject(std::move(v), MV_ICONIC_REGION) {}
MvRegion& MvRegion::operator=(const MvRegion& v) {
  MvObject::operator=(v);
  return *this;
}
MvRegion& MvRegion::operator=(MvRegion&& v) {
  MvObject::operator=(std::move(v));
  return *this;
}
MvRegion::MvRegion(const MvObject& v) : MvObject(v, MV_ICONIC_REGION) {}
MvXLD::MvXLD() : MvObject(MV_ICONIC_XLD_CONTOUR) {}
MvXLD::MvXLD(const MvXLD& v) : MvObject(v, MV_ICONIC_XLD_CONTOUR) {}
MvXLD::MvXLD(MvXLD&& v) noexcept : MvObject(std::move(v), MV_ICONIC_XLD_CONTOUR) {}
MvXLD& MvXLD::operator=(const MvXLD& v) {
  MvObject::operator=(v);
  return *this;
}
MvXLD& MvXLD::operator=(MvXLD&& v) {
  MvObject::operator=(std::move(v));
  return *this;
}
MvXLD::MvXLD(const MvObject& v) : MvObject(v, MV_ICONIC_XLD_CONTOUR) {}
MvXLDCont::MvXLDCont() : MvXLD() {}
MvXLDCont::MvXLDCont(const MvXLDCont& v) : MvXLD(v) {}
MvXLDCont::MvXLDCont(MvXLDCont&& v) noexcept : MvXLD(std::move(v)) {}
MvXLDCont& MvXLDCont::operator=(const MvXLDCont& v) {
  MvObject::operator=(v);
  return *this;
}
MvXLDCont& MvXLDCont::operator=(MvXLDCont&& v) {
  MvObject::operator=(std::move(v));
  return *this;
}
MvXLDCont::MvXLDCont(const MvObject& v) : MvXLD(v) {}

void AddImage(const MvObject& a, const MvObject& b, MvObject* out, const MvTuple& mult, const MvTuple& add) {
  output(out, "AddImage", [&](::MvObject* h) {
    return mv_add_image(a.NativeHandle(), b.NativeHandle(), h, mult.ControlInput(), add.ControlInput());
  });
}
void Threshold(const MvObject& a, MvObject* out, const MvTuple& low, const MvTuple& high) {
  output(out, "Threshold", [&](::MvObject* h) {
    return mv_threshold(a.NativeHandle(), h, low.ControlInput(), high.ControlInput());
  });
}
void ReadImage(MvObject* out, const MvTuple& filenames) {
  output(out, "ReadImage", [&](::MvObject* h) { return mv_read_image(h, filenames.ControlInput()); });
}
void GenEmptyObj(MvObject* out) {
  output(out, "GenEmptyObj", [](::MvObject* h) { return mv_gen_empty_obj(h); });
}
MvImage MvImage::AddImage(const MvImage& b, const MvTuple& mult, const MvTuple& add) const {
  MvImage out;
  mv::AddImage(*this, b, &out, mult, add);
  return out;
}
MvImage MvImage::AddImage(const MvImage& b, double mult, double add) const {
  return AddImage(b, MvTuple(mult), MvTuple(add));
}
MvRegion MvImage::Threshold(const MvTuple& low, const MvTuple& high) const {
  MvRegion out;
  mv::Threshold(*this, &out, low, high);
  return out;
}
MvRegion MvImage::Threshold(double low, double high) const { return Threshold(MvTuple(low), MvTuple(high)); }
void MvImage::ReadImage(const MvTuple& files) { mv::ReadImage(this, files); }
void MvImage::ReadImage(const char* file) { ReadImage(MvTuple(file)); }
std::size_t MvImage::Width() const {
  if (CountObj() == 0) return 0;
  MvTuple width, height; mv::GetImageSize(*this, &width, &height);
  if (width.Length() != 1) invalid("Width", "Image", MvErrorCode::InvalidCardinality, "one image required");
  return static_cast<std::size_t>(width.I());
}
std::size_t MvImage::Height() const {
  if (CountObj() == 0) return 0;
  MvTuple width, height; mv::GetImageSize(*this, &width, &height);
  if (height.Length() != 1) invalid("Height", "Image", MvErrorCode::InvalidCardinality, "one image required");
  return static_cast<std::size_t>(height.I());
}
MvPixelType MvImage::Type(std::size_t channel) const {
  const auto selected = AccessChannel(static_cast<std::int64_t>(channel + 1));
  MvTuple type; mv::GetImageType(selected, &type);
  if (type.Length() != 1) invalid("Type", "Image", MvErrorCode::InvalidCardinality, "one image required");
  const auto value = type.S();
  const std::string_view name(value.Text(), value.Length());
  if (name == "byte") return MvPixelType::UInt8;
  if (name == "uint2") return MvPixelType::UInt16;
  if (name == "int2") return MvPixelType::Int16;
  if (name == "int4") return MvPixelType::Int32;
  return MvPixelType::Float32;
}
const void* MvImage::Data(std::size_t channel) const {
  const auto selected = AccessChannel(static_cast<std::int64_t>(channel + 1));
  MvTuple pointer, type, width, height;
  mv::GetImagePointer1(selected, &pointer, &type, &width, &height);
  if (pointer.Length() != 1) invalid("Data", "Image", MvErrorCode::InvalidCardinality, "one image required");
  return reinterpret_cast<const void*>(static_cast<std::uintptr_t>(pointer.I()));
}
void* MvImage::MutableData(std::size_t channel) {
  // 可写便捷访问由标准 CopyImage/GetImagePointer1 组成，保留现有副本快照。
  MvImage copied; mv::CopyImage(*this, &copied);
  auto* data = const_cast<void*>(copied.Data(channel));
  detail::MvObjectAccess::Commit(this, std::move(copied));
  return data;
}

void CountObj(const MvObject& objects, MvTuple* count) {
  cpp_api::require_output(count, "CountObj");
  ::MvTuple handle = nullptr;
  check(mv_count_obj(objects.NativeHandle(), &handle), "CountObj");
  *count = detail::MvTupleAccess::Adopt(handle);
}
void SelectObj(const MvObject& objects, MvObject* selected, const MvTuple& index) {
  output(selected, "SelectObj",
         [&](::MvObject* h) { return mv_select_obj(objects.NativeHandle(), h, index.ControlInput()); });
}
void CopyObj(const MvObject& objects, MvObject* copied, const MvTuple& index, const MvTuple& count) {
  output(copied, "CopyObj", [&](::MvObject* h) {
    return mv_copy_obj(objects.NativeHandle(), h, index.ControlInput(), count.ControlInput());
  });
}
void ConcatObj(const MvObject& left, const MvObject& right, MvObject* concatenated) {
  output(concatenated, "ConcatObj",
         [&](::MvObject* h) { return mv_concat_obj(left.NativeHandle(), right.NativeHandle(), h); });
}
void TupleConcat(const MvTuple& left, const MvTuple& right, MvTuple* concatenated) {
  cpp_api::require_output(concatenated, "TupleConcat");
  ::MvTuple handle = nullptr;
  check(mv_tuple_concat(left.ControlInput(), right.ControlInput(), &handle), "TupleConcat");
  *concatenated = detail::MvTupleAccess::Adopt(handle);
}
void TupleSelect(const MvTuple& tuple, const MvTuple& indices, MvTuple* selected) {
  cpp_api::require_output(selected, "TupleSelect");
  ::MvTuple handle = nullptr;
  check(mv_tuple_select(tuple.ControlInput(), indices.ControlInput(), &handle), "TupleSelect");
  *selected = detail::MvTupleAccess::Adopt(handle);
}
void TupleSelectRange(const MvTuple& tuple, const MvTuple& left, const MvTuple& right, MvTuple* selected) {
  cpp_api::require_output(selected, "TupleSelectRange");
  ::MvTuple handle = nullptr;
  check(mv_tuple_select_range(tuple.ControlInput(), left.ControlInput(), right.ControlInput(), &handle),
        "TupleSelectRange");
  *selected = detail::MvTupleAccess::Adopt(handle);
}

MvImage MvImage::SelectObj(const MvTuple& index) const { return MvImage(MvObject::SelectObj(index)); }
MvImage MvImage::SelectObj(std::int64_t index) const { return SelectObj(MvTuple(index)); }
MvImage MvImage::CopyObj(std::int64_t index, std::int64_t count) const {
  return MvImage(MvObject::CopyObj(index, count));
}
MvImage MvImage::ConcatObj(const MvImage& other) const { return MvImage(MvObject::ConcatObj(other)); }

MvRegion MvRegion::SelectObj(const MvTuple& index) const { return MvRegion(MvObject::SelectObj(index)); }
MvRegion MvRegion::SelectObj(std::int64_t index) const { return SelectObj(MvTuple(index)); }
MvRegion MvRegion::CopyObj(std::int64_t index, std::int64_t count) const {
  return MvRegion(MvObject::CopyObj(index, count));
}
MvRegion MvRegion::ConcatObj(const MvRegion& other) const { return MvRegion(MvObject::ConcatObj(other)); }

MvXLDCont MvXLDCont::SelectObj(const MvTuple& index) const { return MvXLDCont(MvObject::SelectObj(index)); }
MvXLDCont MvXLDCont::SelectObj(std::int64_t index) const { return SelectObj(MvTuple(index)); }
MvXLDCont MvXLDCont::CopyObj(std::int64_t index, std::int64_t count) const {
  return MvXLDCont(MvObject::CopyObj(index, count));
}
MvXLDCont MvXLDCont::ConcatObj(const MvXLDCont& other) const { return MvXLDCont(MvObject::ConcatObj(other)); }

}  // namespace mv

namespace mv {
void GetObjClass(const MvObject& objects,MvTuple* classes) {
  cpp_api::TupleOutputs<1> out("GetObjClass",{classes});
  check(mv_get_obj_class(objects.NativeHandle(),out.Out(0)),"GetObjClass");out.Commit();
}
void CopyImage(const MvObject& image,MvObject* copied) {
  output(copied,"CopyImage",[&](::MvObject* result){return mv_copy_image(image.NativeHandle(),result);});
}
}

namespace mv {
void TupleType(const MvTuple& t,MvTuple* type) {
  cpp_api::TupleOutputs<1> out("TupleType",{type});check(mv_tuple_type(t.ControlInput(),out.Out(0)),"TupleType");out.Commit();
}
void ClearHandle(const MvTuple& handle) {check(mv_clear_handle(handle.ControlInput()),"ClearHandle");}
void TupleSemType(const MvTuple& t,MvTuple* type) {
  cpp_api::TupleOutputs<1> out("TupleSemType",{type});check(mv_tuple_sem_type(t.ControlInput(),out.Out(0)),"TupleSemType");out.Commit();
}
}

namespace mv {
void ClearObj(MvObject* objects) {
  if (!objects) invalid("ClearObj", "Objects", MvErrorCode::NullArgument, "object output must not be null");
  MvObject empty;
  *objects = std::move(empty);
}
}
