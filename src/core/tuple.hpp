#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <core/string.hpp>
#include <core/tuple_storage.hpp>
#include <core/value.hpp>

namespace mv::core {

enum class TupleKind : std::uint8_t {
  Empty,
  Int64,
  Real64,
  String,
  Handle,
  Mixed,
};

class Tuple {
public:
  Tuple() = default;
  Tuple(std::int64_t value);
  Tuple(int value) : Tuple(static_cast<std::int64_t>(value)) {}
  Tuple(double value);
  Tuple(String value);
  Tuple(Handle value);
  Tuple(const char *value) : Tuple(String{value}) {}
  Tuple(std::string value) : Tuple(String{std::move(value)}) {}
  Tuple(std::string_view value) : Tuple(String{value}) {}

  Tuple(std::initializer_list<std::int64_t> values);
  Tuple(std::initializer_list<double> values);
  explicit Tuple(std::vector<double> values);
  explicit Tuple(std::vector<std::int64_t> values);
  Tuple(std::initializer_list<String> values);
  explicit Tuple(std::vector<String> values);
  Tuple(std::initializer_list<const char *> values);
  Tuple(std::initializer_list<Handle> values);
  Tuple(std::initializer_list<Value> values);

  [[nodiscard]] std::size_t size() const noexcept {
    return std::visit(
        [](const auto &value) -> std::size_t {
          using T = std::decay_t<decltype(value)>;
          if constexpr (std::is_same_v<T, std::monostate>)
            return 0;
          else if constexpr(std::is_same_v<T,SharedStorage>)
            return value?value->size():0;
          else
            return value.size();
        },
        read_storage());
  }
  [[nodiscard]] bool empty() const noexcept { return size() == 0; }
  [[nodiscard]] TupleKind kind() const noexcept {
    switch (read_storage().index()) {
    case 0:
      return TupleKind::Empty;
    case 1:
      return TupleKind::Int64;
    case 2:
      return TupleKind::Real64;
    case 3:
      return TupleKind::String;
    case 4:
      return TupleKind::Handle;
    case 5:
      return TupleKind::Mixed;
    default:
      return TupleKind::Empty;
    }
  }

  [[nodiscard]] Value at(std::size_t index) const;
  [[nodiscard]] std::int64_t as_int64(std::size_t index = 0) const {
    if (const auto *values = std::get_if<IntStorage>(&read_storage())) {
      if (index >= values->size())
        check_index(index, values->size(), "Tuple::at");
      return (*values)[index];
    }
    if (const auto* values = std::get_if<MixedStorage>(&read_storage())) {
      if (index >= values->size()) check_index(index, values->size(), "Tuple::at");
      return (*values)[index].as_int64();
    }
    return at(index).as_int64();
  }
  [[nodiscard]] double as_real64(std::size_t index = 0) const {
    // 借用实际元素，避免为混合相机/位姿元组复制完整 Value；转换仍通过原 Value 规则。
    if (const auto *values = std::get_if<RealStorage>(&read_storage())) {
      if (index >= values->size())
        check_index(index, values->size(), "Tuple::at");
      return (*values)[index];
    }
    if (const auto *values = std::get_if<IntStorage>(&read_storage())) {
      if (index >= values->size())
        check_index(index, values->size(), "Tuple::at");
      return static_cast<double>((*values)[index]);
    }
    if (const auto* values = std::get_if<MixedStorage>(&read_storage())) {
      if (index >= values->size()) check_index(index, values->size(), "Tuple::at");
      return (*values)[index].as_real64();
    }
    return at(index).as_real64();
  }
  [[nodiscard]] String as_string(std::size_t index = 0) const;
  [[nodiscard]] std::string_view string_view(std::size_t index = 0) const;
  [[nodiscard]] Handle as_handle(std::size_t index = 0) const;

  [[nodiscard]] Tuple concat(const Tuple &other) const;
  [[nodiscard]] Tuple select(const Tuple &indices) const;
  [[nodiscard]] Tuple select_range(std::int64_t left_index,
                                   std::int64_t right_index) const;

  [[nodiscard]] std::span<const std::int64_t> int64_span() const;
  [[nodiscard]] std::span<const double> real64_span() const;
  [[nodiscard]] std::span<const Handle> handle_span() const;

  Tuple &append(std::int64_t value) {
    if (storage_.index() == 0) {
      storage_.emplace<IntStorage>(value);
      return *this;
    }
    if (std::holds_alternative<SharedStorage>(storage_)) detach();
    if (auto *values = std::get_if<IntStorage>(&storage_)) {
      values->push_back(value);
      return *this;
    }
    return append(Value{value});
  }
  Tuple &append(int value) { return append(static_cast<std::int64_t>(value)); }
  Tuple &append(double value) {
    if (storage_.index() == 0) {
      storage_.emplace<RealStorage>(value);
      return *this;
    }
    if (std::holds_alternative<SharedStorage>(storage_)) detach();
    if (auto *values = std::get_if<RealStorage>(&storage_)) {
      values->push_back(value);
      return *this;
    }
    return append(Value{value});
  }
  Tuple &append(String value);
  Tuple &append(Handle value);
  Tuple &append(const char *value) { return append(String{value}); }
  Tuple &append(std::string_view value) { return append(String{value}); }
  Tuple &append(Value value);

  // C SDK 的定长构造、显式调整长度及按索引设置；新增元素尚未设置。
  void resize(std::size_t size);
  void set(std::size_t index, Value value);

  // 字典写入时冻结较大载荷；查询可共享只读数据，后续写操作先独立复制。
  [[nodiscard]] Tuple into_shared() &&;
  friend bool operator==(const Tuple& a,const Tuple& b) {return a.read_storage()==b.read_storage();}

private:
  using IntStorage = TupleStorage<std::int64_t>;
  using RealStorage = TupleStorage<double>;
  using StringStorage = TupleStorage<String>;
  using HandleStorage = TupleStorage<Handle>;
  using MixedStorage = std::vector<Value>;
  using SharedStorage = std::shared_ptr<const Tuple>;
  using Storage = std::variant<std::monostate, IntStorage, RealStorage,
                               StringStorage, HandleStorage, MixedStorage,SharedStorage>;

  [[nodiscard]] const Storage& read_storage() const noexcept {
    if(const auto* shared=std::get_if<SharedStorage>(&storage_);shared && *shared)return (*shared)->storage_;
    return storage_;
  }
  void detach();

  void promote_to_mixed();
  static void check_index(std::size_t index, std::size_t size,
                          std::string_view where);

  Storage storage_;
};

} // namespace mv::core
