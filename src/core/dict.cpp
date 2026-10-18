#include <core/dict.hpp>

#include <utility>
#include <vector>

#include <core/broadcast.hpp>
#include <core/exception.hpp>

namespace mv::core {

Tuple Dict::keys() const {
  Tuple result;
  for (const auto& [key, value] : values_) {
    std::visit([&result](const auto& item) {
      if constexpr (std::is_same_v<std::decay_t<decltype(item)>, std::int64_t>) result.append(item);
      else result.append(std::string_view{item});
    }, key);
  }
  return result;
}

Dict::Key Dict::string_key(std::string_view key, std::string_view /** 算子标识。 */) {
  // 空字符串为合法 HALCON 键，与空键元组不同。
  return std::string{key};
}

Dict::Key Dict::tuple_key(const Tuple& keys, std::size_t index, std::string_view operation) {
  const mv::core::Value element = keys.at(index);
  switch (element.kind()) {
    case ValueKind::Int64:
      return Key{element.as_int64()};
    case ValueKind::String:
      return string_key(element.as_string().view(), operation);
    default:
      throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                      "key", static_cast<std::int64_t>(index),
                      "dictionary key tuple elements must be string or int64");
  }
}

const Dict::Value& Dict::value(const Key& key, std::string_view operation) const {
  if (const auto it = values_.find(key); it != values_.end()) return it->second;
  throw Exception(Status::NotFound, ErrorCode::MissingItem, operation,
                  "key", -1, "dictionary key was not found");
}

Dict::Value& Dict::value(const Key& key, std::string_view operation) {
  if (auto it = values_.find(key); it != values_.end()) return it->second;
  throw Exception(Status::NotFound, ErrorCode::MissingItem, operation,
                  "key", -1, "dictionary key was not found");
}

const Tuple& Dict::as_tuple(const Value& value, std::string_view operation) {
  if (const auto* tuple = std::get_if<Tuple>(&value)) return *tuple;
  throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                  "value", -1, "dictionary value is not a Tuple");
}

Tuple& Dict::as_tuple(Value& value, std::string_view operation) {
  if (auto* tuple = std::get_if<Tuple>(&value)) return *tuple;
  throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                  "value", -1, "dictionary value is not a Tuple");
}

const IconicTuple& Dict::as_iconic(const Value& value, std::string_view operation) {
  if (const auto* iconic = std::get_if<IconicTuple>(&value)) return *iconic;
  throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                  "value", -1, "dictionary value is not an IconicTuple");
}

bool Dict::contains(std::string_view key) const {
  return values_.contains(string_key(key, "Dict::contains"));
}

bool Dict::contains(std::int64_t key) const {
  return values_.contains(Key{key});
}

void Dict::set(std::string_view key, Tuple value) {
  values_.insert_or_assign(string_key(key, "Dict::set"), std::move(value).into_shared());
}

void Dict::set(std::int64_t key, Tuple value) {
  values_.insert_or_assign(Key{key}, std::move(value).into_shared());
}

void Dict::set(std::string_view key, IconicTuple value) {
  values_.insert_or_assign(string_key(key, "Dict::set"), std::move(value));
}

void Dict::set(std::int64_t key, IconicTuple value) {
  values_.insert_or_assign(Key{key}, std::move(value));
}

void Dict::set_tuple_keys(const Tuple& keys, const Tuple& values) {
  constexpr std::string_view operation = "Dict::set_tuple_keys";
  if (keys.empty()) return;

  std::vector<Key> parsed_keys;
  parsed_keys.reserve(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index) {
    parsed_keys.push_back(tuple_key(keys, index, operation));
  }

  Dict updated = *this;
  if (keys.size() == 1U) {
    auto copy=values;
    updated.values_.insert_or_assign(parsed_keys.front(), std::move(copy).into_shared());
  } else {
    if (values.size() != 1U && values.size() != keys.size()) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, "value", -1,
                      "value tuple length must be 1 or equal to key tuple length");
    }
    const BroadcastPlan plan = BroadcastPlan::make(
        {{keys.size(), "key", true}, {values.size(), "value", true}}, operation);
    for (std::size_t index = 0; index < plan.size(); ++index) {
      Tuple scalar;
      scalar.append(values.at(plan.source_index(1, index)));
      updated.values_.insert_or_assign(parsed_keys[index], std::move(scalar));
    }
  }
  values_.swap(updated.values_);
}

void Dict::set_object_tuple_keys(const Tuple& keys, const IconicTuple& values) {
  constexpr std::string_view operation = "Dict::set_object_tuple_keys";
  if (keys.empty()) return;
  std::vector<Key> parsed_keys;
  parsed_keys.reserve(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index) {
    parsed_keys.push_back(tuple_key(keys, index, operation));
  }

  Dict updated = *this;
  if (keys.size() == 1U) {
    updated.values_.insert_or_assign(parsed_keys.front(), values);
  } else {
    if (values.size() != 1U && values.size() != keys.size()) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, "value", -1,
                      "iconic tuple length must be 1 or equal to key tuple length");
    }
    for (std::size_t index = 0; index < keys.size(); ++index) {
      const std::size_t source = values.size() == 1U ? 0U : index;
      updated.values_.insert_or_assign(
          parsed_keys[index],
          IconicTuple{std::vector<IconicValue>{values.at(source)}});
    }
  }
  values_.swap(updated.values_);
}

DictValueKind Dict::value_kind(std::string_view key) const {
  return std::holds_alternative<Tuple>(value(string_key(key, "Dict::value_kind"), "Dict::value_kind"))
             ? DictValueKind::Tuple
             : DictValueKind::IconicTuple;
}

DictValueKind Dict::value_kind(std::int64_t key) const {
  return std::holds_alternative<Tuple>(value(Key{key}, "Dict::value_kind"))
             ? DictValueKind::Tuple
             : DictValueKind::IconicTuple;
}

const Tuple& Dict::get(std::string_view key) const {
  return as_tuple(value(string_key(key, "Dict::get"), "Dict::get"), "Dict::get");
}

Tuple& Dict::get(std::string_view key) {
  return as_tuple(value(string_key(key, "Dict::get"), "Dict::get"), "Dict::get");
}

const Tuple& Dict::get(std::int64_t key) const {
  return as_tuple(value(Key{key}, "Dict::get"), "Dict::get");
}

Tuple& Dict::get(std::int64_t key) {
  return as_tuple(value(Key{key}, "Dict::get"), "Dict::get");
}

const IconicTuple& Dict::get_object(std::string_view key) const {
  return as_iconic(value(string_key(key, "Dict::get_object"), "Dict::get_object"),
                   "Dict::get_object");
}

const IconicTuple& Dict::get_object(std::int64_t key) const {
  return as_iconic(value(Key{key}, "Dict::get_object"), "Dict::get_object");
}

Tuple Dict::get_tuple_keys(const Tuple& keys) const {
  constexpr std::string_view operation = "Dict::get_tuple_keys";
  if (keys.empty()) return {};
  if (keys.size() == 1U) {
    const Key key = tuple_key(keys, 0, operation);
    return as_tuple(value(key, operation), operation);
  }

  std::vector<Key> parsed_keys;
  parsed_keys.reserve(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index) {
    parsed_keys.push_back(tuple_key(keys, index, operation));
  }

  Tuple result;
  for (const Key& key : parsed_keys) {
    const Tuple& stored = as_tuple(value(key, operation), operation);
    if (stored.size() != 1U) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, "value", -1,
                      "multiple keys require a single-element Tuple for every key");
    }
    result.append(stored.at(0));
  }
  return result;
}

IconicTuple Dict::get_object_tuple_keys(const Tuple& keys) const {
  constexpr std::string_view operation = "Dict::get_object_tuple_keys";
  if (keys.empty()) return {};
  if (keys.size() == 1U) {
    return as_iconic(value(tuple_key(keys, 0, operation), operation), operation);
  }
  IconicTuple result;
  for (std::size_t index = 0; index < keys.size(); ++index) {
    const auto& stored = as_iconic(
        value(tuple_key(keys, index, operation), operation), operation);
    if (stored.size() != 1U) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, "value", static_cast<std::int64_t>(index),
                      "multiple keys require one iconic object for every key");
    }
    result = result.concat(stored);
  }
  return result;
}

const Tuple* Dict::find(std::string_view key) const noexcept {
  const auto it = values_.find(Key{std::string{key}});
  return it == values_.end() ? nullptr : std::get_if<Tuple>(&it->second);
}

Tuple* Dict::find(std::string_view key) noexcept {
  const auto it = values_.find(Key{std::string{key}});
  return it == values_.end() ? nullptr : std::get_if<Tuple>(&it->second);
}

const Tuple* Dict::find(std::int64_t key) const noexcept {
  const auto it = values_.find(Key{key});
  return it == values_.end() ? nullptr : std::get_if<Tuple>(&it->second);
}

Tuple* Dict::find(std::int64_t key) noexcept {
  const auto it = values_.find(Key{key});
  return it == values_.end() ? nullptr : std::get_if<Tuple>(&it->second);
}

const IconicTuple* Dict::find_object(std::string_view key) const noexcept {
  const auto it = values_.find(Key{std::string{key}});
  return it == values_.end() ? nullptr : std::get_if<IconicTuple>(&it->second);
}

const IconicTuple* Dict::find_object(std::int64_t key) const noexcept {
  const auto it = values_.find(Key{key});
  return it == values_.end() ? nullptr : std::get_if<IconicTuple>(&it->second);
}

bool Dict::erase(std::string_view key) {
  return values_.erase(string_key(key, "Dict::erase")) != 0;
}

bool Dict::erase(std::int64_t key) {
  return values_.erase(Key{key}) != 0;
}

}  // namespace mv::core
