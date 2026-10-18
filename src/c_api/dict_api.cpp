#include <mvkit/c/mv_dict.h>

#include <atomic>
#include <string_view>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include "control_object.hpp"
#include "tuple_input.hpp"

#include <core/exception.hpp>

namespace {
void require_dict(::MvDict dict, const char* where) {
  if (dict == nullptr) throw mv::core::Exception(
      mv::core::Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
      where, "dict", -1, "dict handle must not be null");
}
void require_output(const void* output, const char* where) {
  if (output == nullptr) throw mv::core::Exception(
      mv::core::Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
      where, "output", -1, "output pointer must not be null");
}
std::string_view checked_key(::MvStringView key, const char* where) {
  if (key.data == nullptr && key.size) throw mv::core::Exception(
      mv::core::Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
      where, "key", -1, "dictionary key data must not be null");
  return {key.data ? key.data : "", key.size};
}
::MvDictValueKind to_c_kind(mv::core::DictValueKind kind) {
  return kind == mv::core::DictValueKind::Tuple ? MV_DICT_VALUE_TUPLE : MV_DICT_VALUE_ICONIC_TUPLE;
}

::MvDict control_dict(::MvTuple input, const char* op) {
  const mv::c_api::TupleInput view(input, op, "DictHandle");
  if (view.size() != 1) view.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one dictionary handle required");
  const auto value = view.at(0);
  if (value.kind() != mv::core::ValueKind::Handle)
    view.fail(mv::core::ErrorCode::InvalidType, 0, "dictionary handle required");
  const auto& handle = value.as_handle();
  if (handle.type_id() != mv::c_api::DictObject::kTypeId)
    view.fail(mv::core::ErrorCode::InvalidType, 0, "control handle is not a dictionary");
  return handle.object<mv::c_api::DictObject>().handle();
}
mv::core::Value dict_key(const mv::c_api::TupleInput& view, std::size_t i) {
  auto key = view.at(i);
  if (key.kind() != mv::core::ValueKind::String && key.kind() != mv::core::ValueKind::Int64)
    view.fail(mv::core::ErrorCode::InvalidType, i, "key must be a string or integer");
  return key;
}
template<class Call> ::MvStatus dict_call(const char* op, Call call) {
  return mv::c_api::translate([&] {
    try { call(); }
    catch (const mv::core::Exception& e) {
      auto parameter=e.parameter();
      if(parameter=="key")parameter="Key";
      else if(parameter=="value")parameter="Tuple";
      throw mv::core::Exception(e.status(),e.error_code(),op,parameter,e.parameter_index(),e.message());
    }
  });
}
}

extern "C" ::MvStatus mv_create_dict(::MvTuple* dict) {
  return dict_call("mv_create_dict",[&] {
    require_output(dict, "mv_create_dict");
    auto created = std::make_unique<MvDictImpl>();
    auto result = std::make_unique<MvTupleImpl>();
    result->value.append(mv::core::Handle::make<mv::c_api::DictObject>(created.get()));
    // 控制对象已拥有强类型状态，释放构造时的初始引用。
    mv::c_api::release_opaque(created.release());
    *dict = result.release();
  });
}

extern "C" ::MvStatus mv_set_dict_tuple(::MvTuple dict, ::MvTuple keys, ::MvTuple value) {
  return dict_call("mv_set_dict_tuple",[&] {
    const char* op = "mv_set_dict_tuple";
    auto state = control_dict(dict, op);
    const mv::c_api::TupleInput key_view(keys, op, "Key"), values(value, op, "Tuple");
    if (key_view.size() == 1) {
      // 标量键借用已有数值/文本；只复制一次载荷，提交时转移所有权。
      const auto kind = key_view.kind_at(0);
      if (kind == mv::core::ValueKind::Int64 || kind == mv::core::ValueKind::String) {
        std::int64_t integer_key = 0;
        std::string_view string_key;
        if (kind == mv::core::ValueKind::Int64) integer_key = key_view.integer(0);
        else string_key = key_view.string(0);
        auto payload = values.materialize();
        std::unique_lock lock(state->mutex);
        if (kind == mv::core::ValueKind::Int64) state->value.set(integer_key, std::move(payload));
        else state->value.set(string_key, std::move(payload));
        return;
      }
    }
    // 其他类型仍使用原验证路径，保留非法借用值的错误优先级。
    for (std::size_t i = 0; i < key_view.size(); ++i) (void)dict_key(key_view, i);
    // HALCON 24.11.1 仅允许空键配零或一个值；实际基数限制比“忽略值”的一般说明更严格。
    if (!key_view.size()) {
      if (values.size()>1) values.fail(mv::core::ErrorCode::InvalidCardinality,-1,"empty keys require zero or one value");
      return;
    }
    const auto payload = values.materialize();
    std::unique_lock lock(state->mutex);
    if (key_view.size() == 1) {
      auto key = dict_key(key_view, 0);
      if (key.kind() == mv::core::ValueKind::Int64) state->value.set(key.as_int64(), payload);
      else state->value.set(key.as_string().view(), payload);
    } else state->value.set_tuple_keys(key_view.materialize(), payload);
  });
}

extern "C" ::MvStatus mv_get_dict_tuple(::MvTuple dict, ::MvTuple keys, ::MvTuple* value) {
  return dict_call("mv_get_dict_tuple",[&] {
    const char* op = "mv_get_dict_tuple"; require_output(value, op);
    auto state = control_dict(dict, op);
    const mv::c_api::TupleInput key_view(keys, op, "Key");
    if(key_view.size()==1) {
      // 单键直接借用文本，不先复制成 Value、键元组及解析后的键数组。
      const auto kind=key_view.kind_at(0);
      if(kind==mv::core::ValueKind::String || kind==mv::core::ValueKind::Int64) {
        const auto text=kind==mv::core::ValueKind::String ? key_view.string(0) : std::string_view{};
        const auto integer=kind==mv::core::ValueKind::Int64 ? key_view.integer(0) : 0;
        auto result=std::make_unique<MvTupleImpl>();
        std::shared_lock lock(state->mutex);
        const auto& stored=kind==mv::core::ValueKind::String ? state->value.get(text) : state->value.get(integer);
        result->value=stored;*value=result.release();return;
      }
    }
    for (std::size_t i = 0; i < key_view.size(); ++i) (void)dict_key(key_view, i);
    auto result = std::make_unique<MvTupleImpl>();
    std::shared_lock lock(state->mutex);
    result->value = state->value.get_tuple_keys(key_view.materialize());
    *value = result.release();
  });
}

extern "C" ::MvStatus mv_set_dict_object(::MvObject value, ::MvTuple dict, ::MvTuple key) {
  return dict_call("mv_set_dict_object",[&] {
    const char* op = "mv_set_dict_object";
    mv::c_api::require_iconic(value, op, "Object"); auto state = control_dict(dict, op);
    const mv::c_api::TupleInput view(key, op, "Key");
    if (view.size() != 1) view.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one key required");
    const auto parsed = dict_key(view, 0);
    std::unique_lock lock(state->mutex);
    if (parsed.kind() == mv::core::ValueKind::Int64) state->value.set(parsed.as_int64(), value->value);
    else state->value.set(parsed.as_string().view(), value->value);
  });
}

extern "C" ::MvStatus mv_get_dict_object(::MvObject* value, ::MvTuple dict, ::MvTuple key) {
  return dict_call("mv_get_dict_object",[&] {
    const char* op = "mv_get_dict_object"; require_output(value, op); auto state = control_dict(dict, op);
    const mv::c_api::TupleInput view(key, op, "Key");
    if (view.size() != 1) view.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one key required");
    const auto parsed = dict_key(view, 0);
    std::shared_lock lock(state->mutex);
    auto result = mv::c_api::make_iconic(parsed.kind() == mv::core::ValueKind::Int64 ?
        state->value.get_object(parsed.as_int64()) : state->value.get_object(parsed.as_string().view()));
    *value = result.release();
  });
}


extern "C" ::MvStatus mv_get_dict_param(::MvTuple DictHandle,::MvTuple GenParamName,::MvTuple Key,::MvTuple* GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_dict_param";
    auto* dict=control_dict(DictHandle,op);
    const mv::c_api::TupleInput names(GenParamName,op,"GenParamName"),keys(Key,op,"Key");
    require_output(GenParamValue,op);
    if(names.size()!=1) names.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one query name required");
    const auto name=names.string(0);
    auto result=std::make_unique<MvTupleImpl>();
    std::shared_lock lock(dict->mutex);
    if(name=="keys") {
      if(keys.size()) keys.fail(mv::core::ErrorCode::InvalidCardinality,-1,"keys query requires empty Key");
      result->value=dict->value.keys();
    } else if(name=="key_exists" || name=="key_data_type") {
      for(std::size_t i=0;i<keys.size();++i) {
        const auto key=dict_key(keys,i);
        const bool exists=key.kind()==mv::core::ValueKind::String?dict->value.contains(key.as_string().view()):dict->value.contains(key.as_int64());
        if(name=="key_exists") result->value.append(exists?1:0);
        else {
          if(!exists) throw mv::core::Exception(mv::core::Status::NotFound,mv::core::ErrorCode::MissingItem,op,"Key",i,"dictionary key not found");
          const auto kind=key.kind()==mv::core::ValueKind::String?dict->value.value_kind(key.as_string().view()):dict->value.value_kind(key.as_int64());
          result->value.append(kind==mv::core::DictValueKind::Tuple?"tuple":"object");
        }
      }
    } else names.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported dictionary query");
    *GenParamValue=result.release();
  });
}
extern "C" ::MvStatus mv_remove_dict_key(::MvTuple DictHandle,::MvTuple Key) {
  return mv::c_api::translate([&] {
    constexpr auto op="remove_dict_key";
    auto* dict=control_dict(DictHandle,op);
    const mv::c_api::TupleInput keys(Key,op,"Key");
    if(!keys.size()) keys.fail(mv::core::ErrorCode::InvalidCardinality,-1,"at least one dictionary key required");
    std::unique_lock lock(dict->mutex);
    std::exception_ptr first;
    for(std::size_t i=0;i<keys.size();++i) {
      try {
        const auto key=dict_key(keys,i);
        const bool erased=key.kind()==mv::core::ValueKind::String?dict->value.erase(key.as_string().view()):dict->value.erase(key.as_int64());
        if(!erased) throw mv::core::Exception(mv::core::Status::NotFound,mv::core::ErrorCode::MissingItem,op,"Key",i,"dictionary key not found");
      } catch(...) {if(!first)first=std::current_exception();}
    }
    if(first) std::rethrow_exception(first);
  });
}
