#include <algorithm>
#include <core/exception.hpp>
#include <core/tuple.hpp>
#include <utility>

namespace mv::core {

Tuple Tuple::into_shared() && {
  if(std::holds_alternative<SharedStorage>(storage_) || size()<64)return std::move(*this);
  Tuple result;result.storage_=std::make_shared<const Tuple>(std::move(*this));return result;
}
void Tuple::detach() {
  if(const auto* shared=std::get_if<SharedStorage>(&storage_)) {
    Storage copy=*shared?(*shared)->storage_:Storage{};
    storage_=std::move(copy);
  }
}

Tuple::Tuple(std::int64_t value) : storage_(std::in_place_type<IntStorage>, value) {}
Tuple::Tuple(double value) : storage_(std::in_place_type<RealStorage>, value) {}
Tuple::Tuple(String value) : storage_(std::in_place_type<StringStorage>, std::move(value)) {}
Tuple::Tuple(Handle value) : storage_(std::in_place_type<HandleStorage>, std::move(value)) {}

Tuple::Tuple(std::initializer_list<std::int64_t> values) : storage_(IntStorage(values)) {
  if (values.size() == 0) storage_ = std::monostate{};
}

Tuple::Tuple(std::initializer_list<double> values) : storage_(RealStorage(values)) {
  if (values.size() == 0) storage_ = std::monostate{};
}

Tuple::Tuple(std::vector<double> values) : storage_(RealStorage(std::move(values))) {
  if (std::get<RealStorage>(storage_).empty()) storage_ = std::monostate{};
}

Tuple::Tuple(std::vector<std::int64_t> values) : storage_(IntStorage(std::move(values))) {
  if (std::get<IntStorage>(storage_).empty()) storage_ = std::monostate{};
}

Tuple::Tuple(std::initializer_list<String> values) : storage_(StringStorage(values)) {
  if (values.size() == 0) storage_ = std::monostate{};
}

Tuple::Tuple(std::vector<String> values) : storage_(StringStorage(std::move(values))) {
  if (std::get<StringStorage>(storage_).empty()) storage_ = std::monostate{};
}

Tuple::Tuple(std::initializer_list<const char*> values) {
  if (values.size() == 0) return;
  StringStorage result;
  result.reserve(values.size());
  for (const char* value : values) result.emplace_back(value);
  storage_ = std::move(result);
}

Tuple::Tuple(std::initializer_list<Handle> values) : storage_(HandleStorage(values)) {
  if (values.size() == 0) storage_ = std::monostate{};
}

Tuple::Tuple(std::initializer_list<Value> values) {
  if (values.size() == 0) return;
  const auto first = values.begin()->kind();
  const bool homogeneous = std::all_of(
      values.begin(), values.end(), [first](const Value& value) { return value.kind() == first; });
  if (!homogeneous || first == ValueKind::Empty) {
    storage_ = MixedStorage(values);
    return;
  }
  switch (first) {
    case ValueKind::Int64: {
      IntStorage result;
      result.reserve(values.size());
      for (const auto& value : values) result.push_back(value.as_int64());
      storage_ = std::move(result);
      break;
    }
    case ValueKind::Real64: {
      RealStorage result;
      result.reserve(values.size());
      for (const auto& value : values) result.push_back(value.as_real64());
      storage_ = std::move(result);
      break;
    }
    case ValueKind::String: {
      StringStorage result;
      result.reserve(values.size());
      for (const auto& value : values) result.push_back(value.as_string());
      storage_ = std::move(result);
      break;
    }
    case ValueKind::Handle: {
      HandleStorage result;
      result.reserve(values.size());
      for (const auto& value : values) result.push_back(value.as_handle());
      storage_ = std::move(result);
      break;
    }
    default:
      storage_ = MixedStorage(values);
      break;
  }
}


void Tuple::resize(std::size_t length) {
  if (!length) {
    storage_ = std::monostate{};
    return;
  }
  detach();
  if (std::holds_alternative<std::monostate>(storage_)) {
    storage_ = IntStorage(length);
    return;
  }
  std::visit([length](auto& values) {
    using T = std::decay_t<decltype(values)>;
    if constexpr (std::is_same_v<T, MixedStorage>) values.resize(length, Value{std::int64_t{0}});
    else if constexpr (!std::is_same_v<T, std::monostate> && !std::is_same_v<T,SharedStorage>) values.resize(length);
  }, storage_);
}

void Tuple::set(std::size_t index, Value value) {
  check_index(index, size(), "Tuple::set");
  detach();
  if (auto* values = std::get_if<IntStorage>(&storage_); values && value.kind() == ValueKind::Int64) {
    (*values)[index] = value.as_int64();
  } else if (auto* values = std::get_if<RealStorage>(&storage_); values && value.kind() == ValueKind::Real64) {
    (*values)[index] = value.as_real64();
  } else if (auto* values = std::get_if<StringStorage>(&storage_); values && value.kind() == ValueKind::String) {
    (*values)[index] = value.as_string();
  } else if (auto* values = std::get_if<HandleStorage>(&storage_); values && value.kind() == ValueKind::Handle) {
    (*values)[index] = value.as_handle();
  } else {
    if (!std::holds_alternative<MixedStorage>(storage_)) promote_to_mixed();
    std::get<MixedStorage>(storage_)[index] = std::move(value);
  }
}


void Tuple::check_index(std::size_t index, std::size_t count, std::string_view where) {
  if (index >= count) {
    throw Exception(Status::OutOfRange, where, "tuple index out of range");
  }
}

Value Tuple::at(std::size_t index) const {
  check_index(index, size(), "Tuple::at");
  if (const auto* values = std::get_if<IntStorage>(&read_storage())) return Value{(*values)[index]};
  if (const auto* values = std::get_if<RealStorage>(&read_storage())) return Value{(*values)[index]};
  if (const auto* values = std::get_if<StringStorage>(&read_storage())) return Value{(*values)[index]};
  if (const auto* values = std::get_if<HandleStorage>(&read_storage())) return Value{(*values)[index]};
  if (const auto* values = std::get_if<MixedStorage>(&read_storage())) return (*values)[index];
  throw Exception(Status::OutOfRange, "Tuple::at", "tuple is empty");
}

String Tuple::as_string(std::size_t index) const { return at(index).as_string(); }
std::string_view Tuple::string_view(std::size_t index) const {
  check_index(index, size(), "Tuple::string_view");
  if (const auto* values = std::get_if<StringStorage>(&read_storage())) return (*values)[index].view();
  if (const auto* values = std::get_if<MixedStorage>(&read_storage()))
    return (*values)[index].as_string().view();
  throw Exception(Status::TypeMismatch, "Tuple::string_view", "tuple element is not String");
}
Handle Tuple::as_handle(std::size_t index) const {
  if (const auto* values = std::get_if<HandleStorage>(&read_storage())) {
    check_index(index, values->size(), "Tuple::at");
    return (*values)[index];
  }
  if (const auto* values = std::get_if<MixedStorage>(&read_storage())) {
    check_index(index, values->size(), "Tuple::at");
    return (*values)[index].as_handle();
  }
  return at(index).as_handle();
}

Tuple Tuple::concat(const Tuple& other) const {
  if (empty() && other.kind() != TupleKind::Mixed) return other;
  if (other.empty() && kind() != TupleKind::Mixed) return *this;
  Tuple fast_result;
  bool copied = false;
  std::visit(
      [&](const auto& left) {
        using T = std::decay_t<decltype(left)>;
        if constexpr (!std::is_same_v<T, std::monostate> && !std::is_same_v<T, MixedStorage> && !std::is_same_v<T,SharedStorage>) {
          if (const auto* right = std::get_if<T>(&other.read_storage())) {
            T values;
            if (right->size() <= values.max_size() - left.size()) {
              values.reserve(left.size() + right->size());
              values.append_range(left.begin(), left.end());
              values.append_range(right->begin(), right->end());
              fast_result.storage_ = std::move(values);
              copied = true;
            }
          }
        }
      },
      read_storage());
  if (copied) return fast_result;
  Tuple result;
  for (std::size_t index = 0; index < size(); ++index) result.append(at(index));
  for (std::size_t index = 0; index < other.size(); ++index) result.append(other.at(index));
  return result;
}

Tuple Tuple::select(const Tuple& indices) const {
  if (empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, "Tuple::select",
                    "tuple", -1, "input tuple must not be empty");
  }
  if (indices.empty()) return {};
  if (const auto* positions = std::get_if<IntStorage>(&indices.read_storage())) {
    Tuple fast_result;
    bool copied = false;
    std::visit(
        [&](const auto& source) {
          using T = std::decay_t<decltype(source)>;
          if constexpr (!std::is_same_v<T, std::monostate> && !std::is_same_v<T, MixedStorage> && !std::is_same_v<T,SharedStorage>) {
            T values;
            values.reserve(positions->size());
            for (std::size_t index = 0; index < positions->size(); ++index) {
              const auto selected = (*positions)[index];
              if (selected < 0 || static_cast<std::uint64_t>(selected) >= source.size()) {
                throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, "Tuple::select",
                                "indices", static_cast<std::int64_t>(index),
                                "tuple selection index out of range");
              }
              values.push_back(source[static_cast<std::size_t>(selected)]);
            }
            fast_result.storage_ = std::move(values);
            copied = true;
          }
        },
        read_storage());
    if (copied) return fast_result;
  }
  Tuple result;
  for (std::size_t index = 0; index < indices.size(); ++index) {
    const std::int64_t selected = indices.as_int64(index);
    if (selected < 0 || static_cast<std::uint64_t>(selected) >= size()) {
      throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, "Tuple::select", "indices",
                      static_cast<std::int64_t>(index), "tuple selection index out of range");
    }
    result.append(at(static_cast<std::size_t>(selected)));
  }
  return result;
}

Tuple Tuple::select_range(std::int64_t left_index, std::int64_t right_index) const {
  if (empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, "Tuple::select_range",
                    "tuple", -1, "input tuple must not be empty");
  }
  if (left_index < 0 || right_index < left_index - 1) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, "Tuple::select_range",
                    "right_index", -1, "range must satisfy left >= 0 and right >= left - 1");
  }
  if (right_index == left_index - 1) return {};
  if (static_cast<std::uint64_t>(left_index) >= size()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, "Tuple::select_range",
                    "left_index", -1, "left index is out of bounds");
  }
  if (static_cast<std::uint64_t>(right_index) >= size()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, "Tuple::select_range",
                    "right_index", -1, "right index is out of bounds");
  }

  Tuple fast_result;
  bool copied = false;
  std::visit(
      [&](const auto& values) {
        using T = std::decay_t<decltype(values)>;
        if constexpr (!std::is_same_v<T, std::monostate> && !std::is_same_v<T, MixedStorage> && !std::is_same_v<T,SharedStorage>) {
          fast_result.storage_ = T(values.begin() + left_index, values.begin() + right_index + 1);
          copied = true;
        }
      },
      read_storage());
  if (copied) return fast_result;
  Tuple result;
  for (std::int64_t index = left_index; index <= right_index; ++index) {
    result.append(at(static_cast<std::size_t>(index)));
  }
  return result;
}

std::span<const std::int64_t> Tuple::int64_span() const {
  if (const auto* values = std::get_if<IntStorage>(&read_storage())) return *values;
  throw Exception(Status::TypeMismatch, "Tuple::int64_span", "tuple is not homogeneous Int64");
}

std::span<const double> Tuple::real64_span() const {
  if (const auto* values = std::get_if<RealStorage>(&read_storage())) return *values;
  throw Exception(Status::TypeMismatch, "Tuple::real64_span", "tuple is not homogeneous Real64");
}

std::span<const Handle> Tuple::handle_span() const {
  if (const auto* values = std::get_if<HandleStorage>(&read_storage())) return *values;
  throw Exception(Status::TypeMismatch, "Tuple::handle_span", "tuple is not homogeneous Handle");
}

void Tuple::promote_to_mixed() {
  detach();
  MixedStorage result;
  result.reserve(size());
  if (const auto* ints = std::get_if<IntStorage>(&storage_)) {
    for (auto value : *ints) result.emplace_back(value);
  } else if (const auto* reals = std::get_if<RealStorage>(&storage_)) {
    for (auto value : *reals) result.emplace_back(value);
  } else if (const auto* strings = std::get_if<StringStorage>(&storage_)) {
    for (const auto& value : *strings) result.emplace_back(value);
  } else if (const auto* handles = std::get_if<HandleStorage>(&storage_)) {
    for (const auto& value : *handles) result.emplace_back(value);
  } else if (std::holds_alternative<MixedStorage>(storage_)) {
    return;
  }
  storage_ = std::move(result);
}



Tuple& Tuple::append(String value) {
  if (storage_.index() == 0) {
    storage_.emplace<StringStorage>(std::move(value));
    return *this;
  }
  return append(Value{std::move(value)});
}
Tuple& Tuple::append(Handle value) {
  if (storage_.index() == 0) {
    storage_.emplace<HandleStorage>(std::move(value));
    return *this;
  }
  return append(Value{std::move(value)});
}

Tuple& Tuple::append(Value value) {
  detach();
  if (empty()) {
    switch (value.kind()) {
      case ValueKind::Int64:
        storage_ = IntStorage{value.as_int64()};
        return *this;
      case ValueKind::Real64:
        storage_ = RealStorage{value.as_real64()};
        return *this;
      case ValueKind::String:
        storage_ = StringStorage{value.as_string()};
        return *this;
      case ValueKind::Handle:
        storage_ = HandleStorage{value.as_handle()};
        return *this;
      default:
        storage_ = MixedStorage{std::move(value)};
        return *this;
    }
  }

  const auto tuple_kind = kind();
  if (tuple_kind == TupleKind::Int64 && value.kind() == ValueKind::Int64) {
    std::get<IntStorage>(storage_).push_back(value.as_int64());
    return *this;
  }
  if (tuple_kind == TupleKind::Real64 && value.kind() == ValueKind::Real64) {
    std::get<RealStorage>(storage_).push_back(value.as_real64());
    return *this;
  }
  if (tuple_kind == TupleKind::String && value.kind() == ValueKind::String) {
    std::get<StringStorage>(storage_).push_back(value.as_string());
    return *this;
  }
  if (tuple_kind == TupleKind::Handle && value.kind() == ValueKind::Handle) {
    std::get<HandleStorage>(storage_).push_back(value.as_handle());
    return *this;
  }
  if (tuple_kind != TupleKind::Mixed) promote_to_mixed();
  std::get<MixedStorage>(storage_).push_back(std::move(value));
  return *this;
}

}  // namespace mv::core
