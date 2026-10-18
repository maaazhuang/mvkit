#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <core/iconic_tuple.hpp>
#include <core/tuple.hpp>

namespace mv::core {

enum class DictValueKind : std::uint8_t {
  Tuple,
  IconicTuple,
};

class Dict {
 public:
  [[nodiscard]] std::size_t size() const noexcept { return values_.size(); }
  [[nodiscard]] bool empty() const noexcept { return values_.empty(); }

  [[nodiscard]] bool contains(std::string_view key) const;
  [[nodiscard]] bool contains(std::int64_t key) const;

  void set(std::string_view key, Tuple value);
  void set(std::int64_t key, Tuple value);
  void set(std::string_view key, IconicTuple value);
  void set(std::int64_t key, IconicTuple value);
  void set_tuple_keys(const Tuple& keys, const Tuple& values);
  void set_object_tuple_keys(const Tuple& keys, const IconicTuple& values);

  [[nodiscard]] DictValueKind value_kind(std::string_view key) const;
  [[nodiscard]] DictValueKind value_kind(std::int64_t key) const;
  [[nodiscard]] const Tuple& get(std::string_view key) const;
  [[nodiscard]] Tuple& get(std::string_view key);
  [[nodiscard]] const Tuple& get(std::int64_t key) const;
  [[nodiscard]] Tuple& get(std::int64_t key);
  [[nodiscard]] const IconicTuple& get_object(std::string_view key) const;
  [[nodiscard]] const IconicTuple& get_object(std::int64_t key) const;
  [[nodiscard]] Tuple get_tuple_keys(const Tuple& keys) const;
  [[nodiscard]] IconicTuple get_object_tuple_keys(const Tuple& keys) const;

  [[nodiscard]] const Tuple* find(std::string_view key) const noexcept;
  [[nodiscard]] Tuple* find(std::string_view key) noexcept;
  [[nodiscard]] const Tuple* find(std::int64_t key) const noexcept;
  [[nodiscard]] Tuple* find(std::int64_t key) noexcept;
  [[nodiscard]] const IconicTuple* find_object(std::string_view key) const noexcept;
  [[nodiscard]] const IconicTuple* find_object(std::int64_t key) const noexcept;

  bool erase(std::string_view key);
  bool erase(std::int64_t key);
  void clear() noexcept { values_.clear(); }
  [[nodiscard]] Tuple keys() const;

 private:
  using Key = std::variant<std::int64_t, std::string>;
  using Value = std::variant<Tuple, IconicTuple>;

  [[nodiscard]] static Key string_key(std::string_view key, std::string_view operation);
  [[nodiscard]] static Key tuple_key(const Tuple& keys, std::size_t index,
                                     std::string_view operation);
  [[nodiscard]] const Value& value(const Key& key, std::string_view operation) const;
  [[nodiscard]] Value& value(const Key& key, std::string_view operation);
  [[nodiscard]] static const Tuple& as_tuple(const Value& value, std::string_view operation);
  [[nodiscard]] static Tuple& as_tuple(Value& value, std::string_view operation);
  [[nodiscard]] static const IconicTuple& as_iconic(const Value& value, std::string_view operation);

  std::map<Key, Value> values_;
};

}  // namespace mv::core
