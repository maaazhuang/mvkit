#pragma once

#include <cmath>
#include <string_view>
#include <mvkit/c/mv_tuple.h>
#include "opaque.hpp"
#include <core/exception.hpp>

namespace mv::c_api {

// 仅为视图，不物化 core::Tuple；标量适配不分配，只有需要拥有型元组的内核才物化。
class TupleInput {
 public:
  TupleInput(::MvTuple input, std::string_view operation,
             std::string_view parameter)
      : operation_(operation), parameter_(parameter) {
    if (input && input->refs.load(std::memory_order_relaxed)==0)
      input_={input->borrowed_handle,input->borrowed_values,input->borrowed_count};
    else input_={input,nullptr,0};
    const auto& descriptor=input_;
    if ((descriptor.handle && (descriptor.values || descriptor.count)) ||
        (!descriptor.handle && descriptor.count && !descriptor.values))
      fail(core::ErrorCode::InvalidValue, -1, "invalid control tuple representation");
  }
  std::size_t size() const noexcept {
    return input_.handle ? input_.handle->value.size() : input_.count;
  }
  // 拥有型同质数值无需逐元素构造通用 Value；Mixed 仍交给原检查路径。
  core::TupleKind homogeneous_kind() const noexcept {
    return input_.handle ? input_.handle->value.kind() : core::TupleKind::Mixed;
  }
  const core::Tuple* owned_tuple() const noexcept {return input_.handle?&input_.handle->value:nullptr;}
  core::ValueKind kind_at(std::size_t index) const {
    if(index>=size())fail(core::ErrorCode::IndexOutOfRange,index,"control tuple index out of range");
    if(input_.handle) {
      switch(input_.handle->value.kind()) {
        case core::TupleKind::Int64:return core::ValueKind::Int64;
        case core::TupleKind::Real64:return core::ValueKind::Real64;
        case core::TupleKind::String:return core::ValueKind::String;
        case core::TupleKind::Handle:return core::ValueKind::Handle;
        default:return input_.handle->value.at(index).kind();
      }
    }
    switch(input_.values[index].kind) {
      case MV_VALUE_INT64:return core::ValueKind::Int64;
      case MV_VALUE_REAL64:return core::ValueKind::Real64;
      case MV_VALUE_STRING:return core::ValueKind::String;
      case MV_VALUE_HANDLE:return core::ValueKind::Handle;
      default:fail(core::ErrorCode::InvalidType,index,"invalid control value type");
    }
  }
  core::Value at(std::size_t index) const {
    if (index >= size()) fail(core::ErrorCode::IndexOutOfRange, index, "control tuple index out of range");
    if (input_.handle) return input_.handle->value.at(index);
    const auto& v = input_.values[index];
    switch (v.kind) {
      case MV_VALUE_INT64: return core::Value{v.data.int64_value};
      case MV_VALUE_REAL64: return core::Value{v.data.real64_value};
      case MV_VALUE_STRING:
        if (!v.data.string_value.data && v.data.string_value.size)
          fail(core::ErrorCode::NullArgument, index, "string data must not be null");
        return core::Value{core::String{std::string_view{
            v.data.string_value.data ? v.data.string_value.data : "", v.data.string_value.size}}};
      case MV_VALUE_HANDLE:
        if (!v.data.handle_value) fail(core::ErrorCode::NullArgument, index, "control handle must not be null");
        return core::Value{v.data.handle_value->value};
      default: fail(core::ErrorCode::InvalidType, index, "invalid control value type");
    }
  }
  core::Handle handle(std::size_t index) const {
    if (index >= size()) fail(core::ErrorCode::IndexOutOfRange, index, "control tuple index out of range");
    // 直接读取拥有型/借用型句柄，避免先复制 Value 再复制其中的 Handle。
    if (input_.handle) return input_.handle->value.as_handle(index);
    const auto& value = input_.values[index];
    if (value.kind != MV_VALUE_HANDLE) return at(index).as_handle();
    if (!value.data.handle_value) fail(core::ErrorCode::NullArgument, index, "control handle must not be null");
    return value.data.handle_value->value;
  }
  double number(std::size_t index) const {
    // 数值快路径避免构造 Value/String。
    if (index >= size()) fail(core::ErrorCode::IndexOutOfRange, index, "control tuple index out of range");
    if (input_.handle) {
      const auto kind=input_.handle->value.kind();
      if(kind==core::TupleKind::Int64 || kind==core::TupleKind::Real64)
        return input_.handle->value.as_real64(index);
      try { return input_.handle->value.as_real64(index); }
      catch (const core::Exception&) {
        fail(core::ErrorCode::InvalidType, index, "numeric control value required");
      }
    }
    const auto& v = input_.values[index];
    if (v.kind == MV_VALUE_INT64) return static_cast<double>(v.data.int64_value);
    if (v.kind == MV_VALUE_REAL64) return v.data.real64_value;
    fail(core::ErrorCode::InvalidType, index, "numeric control value required");
  }
  double scalar_number() const {
    if (size() != 1) fail(core::ErrorCode::InvalidCardinality, -1, "exactly one control value required");
    const auto value = number(0);
    if (!std::isfinite(value)) fail(core::ErrorCode::InvalidValue, 0, "finite control value required");
    return value;
  }
  std::int64_t integer(std::size_t index, bool accept_integral_real = false) const {
    if(index>=size())fail(core::ErrorCode::IndexOutOfRange,index,"control tuple index out of range");
    if(input_.handle && input_.handle->value.kind()==core::TupleKind::Int64)
      return input_.handle->value.as_int64(index);
    auto value = at(index);
    if (value.kind() == core::ValueKind::Int64) return value.as_int64();
    if (accept_integral_real && value.kind() == core::ValueKind::Real64) {
      const double number=value.as_real64();
      if (std::isfinite(number) && std::trunc(number)==number &&
          number>=-9223372036854775808.0 && number<9223372036854775808.0)
        return static_cast<std::int64_t>(number);
      fail(core::ErrorCode::InvalidValue, index, "integer-valued control required");
    }
    fail(core::ErrorCode::InvalidType, index, "integer control value required");
  }
  std::int64_t scalar_integer(bool accept_integral_real = false) const {
    if (size() != 1) fail(core::ErrorCode::InvalidCardinality, -1, "exactly one control value required");
    return integer(0, accept_integral_real);
  }
  std::string_view string(std::size_t index) const {
    if (index >= size()) fail(core::ErrorCode::IndexOutOfRange, index, "control tuple index out of range");
    if (input_.handle) {
      if (input_.handle->value.kind()!=core::TupleKind::String &&
          input_.handle->value.at(index).kind() != core::ValueKind::String)
        fail(core::ErrorCode::InvalidType, index, "string control value required");
      return input_.handle->value.string_view(index);
    }
    const auto& v = input_.values[index];
    if (v.kind != MV_VALUE_STRING) fail(core::ErrorCode::InvalidType, index, "string control value required");
    if (!v.data.string_value.data && v.data.string_value.size)
      fail(core::ErrorCode::NullArgument, index, "string data must not be null");
    return {v.data.string_value.data ? v.data.string_value.data : "", v.data.string_value.size};
  }
  core::Tuple materialize() const {
    if (input_.handle) return input_.handle->value;
    if (input_.count == 1) {
      if (input_.values[0].kind == MV_VALUE_INT64) return core::Tuple{input_.values[0].data.int64_value};
      if (input_.values[0].kind == MV_VALUE_REAL64) return core::Tuple{input_.values[0].data.real64_value};
    }
    if(input_.count && (input_.values[0].kind==MV_VALUE_INT64 || input_.values[0].kind==MV_VALUE_REAL64)) {
      const auto kind=input_.values[0].kind;
      bool same=true;
      for(std::size_t i=1;i<input_.count;++i)if(input_.values[i].kind!=kind){same=false;break;}
      if(same) {
        if(kind==MV_VALUE_INT64) {
          std::vector<std::int64_t> values;values.reserve(input_.count);
          for(std::size_t i=0;i<input_.count;++i)values.push_back(input_.values[i].data.int64_value);
          return core::Tuple{std::move(values)};
        }
        std::vector<double> values;values.reserve(input_.count);
        for(std::size_t i=0;i<input_.count;++i)values.push_back(input_.values[i].data.real64_value);
        return core::Tuple{std::move(values)};
      }
    }
    core::Tuple output;
    for (std::size_t i = 0; i < size(); ++i) output.append(at(i));
    return output;
  }
  [[noreturn]] void fail(core::ErrorCode code, std::int64_t index,
                         std::string_view message) const {
    throw core::Exception(code == core::ErrorCode::InvalidType ? core::Status::TypeMismatch :
        core::Status::InvalidArgument, code, operation_, parameter_, index, message);
  }
  [[noreturn]] void fail_range(std::int64_t index, std::string_view message) const {
    throw core::Exception(core::Status::OutOfRange, core::ErrorCode::IndexOutOfRange,
                          operation_, parameter_, index, message);
  }
 private:
  struct Descriptor { ::MvTuple handle; const ::MvValue* values; std::size_t count; } input_{};
  std::string_view operation_, parameter_;
};

// 既有拥有型输入按引用传递；仅对接收核心 Tuple 的内核物化借用数组，转换临时值覆盖完整调用。
class ControlTupleView {
 public:
  ControlTupleView(::MvTuple input, std::string_view operation, std::string_view parameter) {
    const TupleInput checked(input,operation,parameter);
    if (input && input->refs.load(std::memory_order_relaxed)!=0) value_=&input->value;
    else { storage_=checked.materialize();value_=&storage_; }
  }
  ControlTupleView(const ControlTupleView&)=delete;
  operator const core::Tuple&() const noexcept { return *value_; }
 private:
  core::Tuple storage_;
  const core::Tuple* value_;
};

// 为接收原生拥有型 Tuple 的旧 SDK/数据辅助接口提供桥接；正式算子直接读取 TupleInput，保留标量无分配路径。
class NativeTupleInput {
 public:
  NativeTupleInput(::MvTuple input, std::string_view operation, std::string_view parameter)
      : handle_(input) {
    if (input && input->refs.load(std::memory_order_relaxed)==0) {
      storage_.value=TupleInput(input,operation,parameter).materialize();
      handle_=&storage_;
    }
  }
  ::MvTuple get() const noexcept { return handle_; }
 private:
  MvTupleImpl storage_;
  ::MvTuple handle_;
};
}
