#include <mvkit/c/mv_tuple.h>

#include <atomic>
#include <string_view>
#include <memory>

#include "error_state.hpp"
#include "opaque.hpp"
#include "tuple_input.hpp"
#include "tuple_bridge.hpp"

#include <core/exception.hpp>

namespace {

void require_tuple(::MvTuple tuple, const char* where, const char* parameter = "tuple") {
  if (tuple == nullptr) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::NullArgument,
                              where, parameter, -1,
                              "tuple handle must not be null");
  }
  if (tuple->refs.load(std::memory_order_relaxed)==0) {
    throw mv::core::Exception(mv::core::Status::InvalidState,
      mv::core::ErrorCode::InvalidObjectState,where,parameter,-1,
      "borrowed tuple is read-only; clone it for an independent owner");
  }
}

void require_output(const void* output, const char* where, const char* parameter = "output") {
  if (output == nullptr) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::NullArgument,
                              where, parameter, -1,
                              "output pointer must not be null");
  }
}

void require_readable_tuple(::MvTuple tuple,const char* operation) {
  if (!tuple) require_tuple(tuple,operation);
}

void require_index(const mv::c_api::TupleInput& input,std::size_t index,const char* operation) {
  if (index>=input.size())
    throw mv::core::Exception(mv::core::Status::OutOfRange,
      mv::core::ErrorCode::IndexOutOfRange,operation,"index",static_cast<std::int64_t>(index),
      "tuple index out of range");
}

std::string_view checked_view(::MvStringView value, const char* where) {
  if (value.size == 0) return {};
  if (value.data == nullptr) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::NullArgument,
                              where, "value", -1,
                              "string data must not be null when size is non-zero");
  }
  return {value.data, value.size};
}

::MvTupleKind to_c_kind(mv::core::TupleKind kind) {
  switch (kind) {
    case mv::core::TupleKind::Empty: return MV_TUPLE_EMPTY;
    case mv::core::TupleKind::Int64: return MV_TUPLE_INT64;
    case mv::core::TupleKind::Real64: return MV_TUPLE_REAL64;
    case mv::core::TupleKind::String: return MV_TUPLE_STRING;
    case mv::core::TupleKind::Handle: return MV_TUPLE_HANDLE;
    case mv::core::TupleKind::Mixed: return MV_TUPLE_MIXED;
  }
  return MV_TUPLE_MIXED;
}

::MvValueKind to_c_value_kind(mv::core::ValueKind kind) {
  switch (kind) {
    case mv::core::ValueKind::Empty: return MV_VALUE_EMPTY;
    case mv::core::ValueKind::Int64: return MV_VALUE_INT64;
    case mv::core::ValueKind::Real64: return MV_VALUE_REAL64;
    case mv::core::ValueKind::String: return MV_VALUE_STRING;
    case mv::core::ValueKind::Handle: return MV_VALUE_HANDLE;
  }
  return MV_VALUE_EMPTY;
}

}  // namespace

extern "C" void mv_private_tuple_input_init(void* storage,::MvTuple handle,
                                          const ::MvValue* values,std::size_t count) noexcept {
  static_assert(sizeof(MvTupleImpl)<=kMvPrivateTupleInputSize,"C++ tuple conversion storage ABI");
  static_assert(alignof(MvTupleImpl)<=alignof(std::max_align_t),"tuple conversion alignment");
  auto* view=new (storage) MvTupleImpl(0);
  view->borrowed_handle=handle;
  view->borrowed_values=values;
  view->borrowed_count=count;
}

extern "C" void mv_private_tuple_input_destroy(void* storage) noexcept {
  static_cast<MvTupleImpl*>(storage)->~MvTupleImpl();
}

extern "C" ::MvStatus mv_create_tuple(::MvTuple* tuple, size_t length) {
  return mv::c_api::translate([&] {
    require_output(tuple, "mv_create_tuple");
    auto created = std::make_unique<MvTupleImpl>();
    created->value.resize(length);
    *tuple = created.release();
  });
}

extern "C" ::MvStatus mv_copy_tuple(::MvTuple tuple, ::MvTuple* clone) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_copy_tuple");
    const mv::c_api::TupleInput input(tuple,"mv_copy_tuple","tuple");
    require_output(clone, "mv_copy_tuple");
    auto created = std::make_unique<MvTupleImpl>();
    created->value = input.materialize();
    *clone = created.release();
  });
}

extern "C" ::MvStatus mv_destroy_tuple(::MvTuple tuple) {
  return mv::c_api::translate([&] {
    require_tuple(tuple, "mv_destroy_tuple");
    if (tuple->refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete tuple;
  });
}

extern "C" ::MvStatus mv_length_tuple(::MvTuple tuple, size_t* size) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_length_tuple");
    const mv::c_api::TupleInput input(tuple,"mv_length_tuple","tuple");
    require_output(size, "mv_length_tuple");
    *size = input.size();
  });
}

extern "C" ::MvStatus mv_get_type(::MvTuple tuple, size_t index, ::MvValueKind* kind) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_get_type");
    const mv::c_api::TupleInput input(tuple,"mv_get_type","tuple");
    require_output(kind, "mv_get_type");
    require_index(input,index,"mv_get_type");
    *kind = to_c_value_kind(input.at(index).kind());
  });
}

extern "C" ::MvStatus mv_tuple_concat(
    ::MvTuple left,
    ::MvTuple right,
    ::MvTuple* result) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput a(left, "tuple_concat", "T1"), b(right, "tuple_concat", "T2");
    require_output(result, "tuple_concat", "Concat");
    auto created = std::make_unique<MvTupleImpl>();
    if (left && right && left->refs.load()!=0 && right->refs.load()!=0) created->value = left->value.concat(right->value);
    else {
      for (std::size_t i=0; i<a.size(); ++i) created->value.append(a.at(i));
      for (std::size_t i=0; i<b.size(); ++i) created->value.append(b.at(i));
    }
    *result = created.release();
  });
}

extern "C" ::MvStatus mv_tuple_select(
    ::MvTuple tuple,
    ::MvTuple indices,
    ::MvTuple* selected) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput source(tuple, "tuple_select", "Tuple"), positions(indices, "tuple_select", "Index");
    require_output(selected, "tuple_select", "Selected");
    if (!source.size() && positions.size()) source.fail(mv::core::ErrorCode::InvalidCardinality, -1, "input tuple must not be empty");
    auto created = std::make_unique<MvTupleImpl>();
    // 同质数值按原类型收集，避免 Value 包装及结果逐次扩容。
    if(const auto* numeric=source.owned_tuple();numeric &&
        (numeric->kind()==mv::core::TupleKind::Int64||numeric->kind()==mv::core::TupleKind::Real64)) {
      const auto gather=[&](auto values) {
        using T=std::remove_cv_t<typename decltype(values)::element_type>;
          if(const auto* indices=positions.owned_tuple();indices && indices->kind()==mv::core::TupleKind::Int64) {
            const auto offsets=indices->int64_span();
            std::vector<T> result(offsets.size());
            // 一次准备整数索引，逐项仍按原顺序验证并保留首个错误位置。
            for(std::size_t i=0;i<offsets.size();++i) {
              const auto index=offsets[i];
              if(index<0 || static_cast<std::uint64_t>(index)>=values.size())
                throw mv::core::Exception(mv::core::Status::OutOfRange,mv::core::ErrorCode::IndexOutOfRange,
                    "tuple_select","Index",i,"tuple selection index out of range");
              result[i]=values[static_cast<std::size_t>(index)];
            }
            created->value=mv::core::Tuple{std::move(result)};
            return;
          }
        std::vector<T> result;result.reserve(positions.size());
        for(std::size_t i=0;i<positions.size();++i) {
          const auto index=positions.integer(i,true);
          if(index<0||static_cast<std::uint64_t>(index)>=values.size())
            throw mv::core::Exception(mv::core::Status::OutOfRange,mv::core::ErrorCode::IndexOutOfRange,
                "tuple_select","Index",i,"tuple selection index out of range");
          result.push_back(values[static_cast<std::size_t>(index)]);
        }
        created->value=mv::core::Tuple{std::move(result)};
      };
      if(numeric->kind()==mv::core::TupleKind::Int64)gather(numeric->int64_span());
      else gather(numeric->real64_span());
      *selected=created.release();return;
    }
    for (std::size_t i=0; i<positions.size(); ++i) {
      auto index=positions.integer(i,true);
      if (index<0 || static_cast<std::uint64_t>(index)>=source.size())
        throw mv::core::Exception(mv::core::Status::OutOfRange, mv::core::ErrorCode::IndexOutOfRange,
            "tuple_select", "Index", i, "tuple selection index out of range");
      created->value.append(source.at(static_cast<std::size_t>(index)));
    }
    *selected = created.release();
  });
}

extern "C" ::MvStatus mv_tuple_select_range(
    ::MvTuple tuple,
    ::MvTuple left_index,
    ::MvTuple right_index,
    ::MvTuple* selected) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput source(tuple, "tuple_select_range", "Tuple"),
        left(left_index, "tuple_select_range", "Leftindex"), right(right_index, "tuple_select_range", "Rightindex");
    require_output(selected, "tuple_select_range", "Selected");
    const auto l=left.scalar_integer(true), r=right.scalar_integer(true);
    if (l<0) left.fail(mv::core::ErrorCode::InvalidValue, 0, "left index must be nonnegative");
    if (r<l-1) right.fail(mv::core::ErrorCode::InvalidValue, 0, "range must satisfy right >= left - 1");
    if ((r==l-1 && static_cast<std::uint64_t>(l)>source.size()) ||
        (r!=l-1 && (static_cast<std::uint64_t>(l)>=source.size() || static_cast<std::uint64_t>(r)>=source.size())))
      throw mv::core::Exception(mv::core::Status::OutOfRange, mv::core::ErrorCode::IndexOutOfRange,
          "tuple_select_range", static_cast<std::uint64_t>(l)>=source.size() ? "Leftindex" : "Rightindex", 0,
          "tuple selection index out of range");
    auto created = std::make_unique<MvTupleImpl>();
    if (r!=l-1) {
      if (tuple && tuple->refs.load()!=0) created->value=tuple->value.select_range(l,r);
      else for (auto i=l; i<=r; ++i) created->value.append(source.at(static_cast<std::size_t>(i)));
    }
    *selected = created.release();
  });
}

extern "C" ::MvStatus mv_private_tuple_append_i(::MvTuple tuple, int64_t value) {
  return mv::c_api::translate([&] {
    require_tuple(tuple, "mv_private_tuple_append_i");
    tuple->value.append(static_cast<std::int64_t>(value));
    tuple->discard_handle_cache();
  });
}

extern "C" ::MvStatus mv_private_tuple_append_d(::MvTuple tuple, double value) {
  return mv::c_api::translate([&] {
    require_tuple(tuple, "mv_private_tuple_append_d");
    tuple->value.append(value);
    tuple->discard_handle_cache();
  });
}

extern "C" ::MvStatus mv_private_tuple_append_s(::MvTuple tuple, ::MvStringView value) {
  return mv::c_api::translate([&] {
    require_tuple(tuple, "mv_private_tuple_append_s");
    tuple->value.append(checked_view(value, "mv_private_tuple_append_s"));
    tuple->discard_handle_cache();
  });
}

extern "C" ::MvStatus mv_private_tuple_append_h(::MvTuple tuple, ::MvHandle handle) {
  return mv::c_api::translate([&] {
    require_tuple(tuple, "mv_private_tuple_append_h");
    if (handle == nullptr) {
      throw mv::core::Exception(mv::core::Status::InvalidArgument,
                                mv::core::ErrorCode::NullArgument,
                                "mv_private_tuple_append_h", "handle", -1,
                                "object handle must not be null");
    }
    const auto value=handle->value;
    tuple->value.append(value);
    tuple->discard_handle_cache();
  });
}

extern "C" ::MvStatus mv_get_i(::MvTuple tuple, size_t index, int64_t* value) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_get_i");
    const mv::c_api::TupleInput input(tuple,"mv_get_i","tuple");
    require_output(value, "mv_get_i");
    require_index(input,index,"mv_get_i");
    *value = input.at(index).as_int64();
  });
}

extern "C" ::MvStatus mv_get_d(::MvTuple tuple, size_t index, double* value) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_get_d");
    const mv::c_api::TupleInput input(tuple,"mv_get_d","tuple");
    require_output(value, "mv_get_d");
    require_index(input,index,"mv_get_d");
    *value = input.number(index);
  });
}

extern "C" ::MvStatus mv_get_s(::MvTuple tuple, size_t index, ::MvStringView* value) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_get_s");
    const mv::c_api::TupleInput input(tuple,"mv_get_s","tuple");
    require_output(value, "mv_get_s");
    require_index(input,index,"mv_get_s");
    const auto view = input.string(index);
    value->data = view.data();
    value->size = view.size();
  });
}

extern "C" ::MvStatus mv_get_h(::MvTuple tuple, size_t index, ::MvHandle* handle) {
  return mv::c_api::translate([&] {
    require_readable_tuple(tuple,"mv_get_h");
    const mv::c_api::TupleInput input(tuple,"mv_get_h","tuple");
    require_output(handle, "mv_get_h");
    require_index(input,index,"mv_get_h");
    const auto value=input.at(index).as_handle();
    auto* owner=tuple->refs.load()!=0?tuple:tuple->borrowed_handle;
    if(!owner) {
      *handle=tuple->borrowed_values[index].data.handle_value;
      return;
    }
    auto* cache=owner->handle_cache.load();
    if(!cache) {
      auto candidate=std::make_unique<MvTupleHandleCache>();
      if(owner->handle_cache.compare_exchange_strong(cache,candidate.get()))cache=candidate.release();
    }
    std::lock_guard lock(cache->mutex);
    for(const auto& entry:cache->handles) {
      if(entry.first==index) {*handle=entry.second;return;}
    }
    auto created=std::make_unique<MvHandleImpl>();created->value=value;
    cache->handles.emplace_back(index,created.get());*handle=created.release();
  });
}

extern "C" ::MvStatus mv_resize_tuple(::MvTuple tuple,size_t length) {
  return mv::c_api::translate([&] { require_tuple(tuple,"mv_resize_tuple");tuple->value.resize(length);tuple->discard_handle_cache(); });
}

extern "C" ::MvStatus mv_set_i(::MvTuple tuple,::int64_t value,size_t index) {
  return mv::c_api::translate([&] {require_tuple(tuple,"mv_set_i");tuple->value.set(index,mv::core::Value{value});tuple->discard_handle_cache();});
}

extern "C" ::MvStatus mv_set_d(::MvTuple tuple,double value,size_t index) {
  return mv::c_api::translate([&] {require_tuple(tuple,"mv_set_d");tuple->value.set(index,mv::core::Value{value});tuple->discard_handle_cache();});
}

extern "C" ::MvStatus mv_set_s(::MvTuple tuple,::MvStringView value,size_t index) {
  return mv::c_api::translate([&] {require_tuple(tuple,"mv_set_s");tuple->value.set(index,mv::core::Value{checked_view(value,"mv_set_s")});tuple->discard_handle_cache();});
}

extern "C" ::MvStatus mv_set_h(::MvTuple tuple,::MvHandle value,size_t index) {
  return mv::c_api::translate([&] {require_tuple(tuple,"mv_set_h");require_output(value,"mv_set_h","value");tuple->value.set(index,mv::core::Value{value->value});tuple->discard_handle_cache();});
}

extern "C" ::MvStatus mv_tuple_type(::MvTuple T,::MvTuple* Type) {
  return mv::c_api::translate([&] {
    const mv::c_api::TupleInput input(T,"tuple_type","T");
    require_output(Type,"tuple_type","Type");
    auto result=std::make_unique<MvTupleImpl>();
    const auto* owner = T && T->refs.load(std::memory_order_relaxed) == 0 ? T->borrowed_handle : T;
    const auto kind = owner ? owner->value.kind() : input.materialize().kind();
    result->value.append(static_cast<std::int64_t>(input.size() ? to_c_kind(kind) : MV_TUPLE_ANY));
    *Type=result.release();
  });
}
