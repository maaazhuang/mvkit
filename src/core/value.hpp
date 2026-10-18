#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <variant>

#include <core/handle.hpp>
#include <core/string.hpp>

namespace mv::core {

enum class ValueKind : std::uint8_t {
  Empty,
  Int64,
  Real64,
  String,
  Handle,
};

class Value {
 public:
  Value() = default;
  Value(std::int64_t value) : storage_(value) {}
  Value(int value) : storage_(static_cast<std::int64_t>(value)) {}
  Value(double value) : storage_(value) {}
  Value(String value) : storage_(std::move(value)) {}
  Value(const char* value) : storage_(String{value}) {}
  Value(std::string value) : storage_(String{std::move(value)}) {}
  Value(std::string_view value) : storage_(String{value}) {}
  Value(Handle value) : storage_(std::move(value)) {}

  [[nodiscard]] ValueKind kind() const noexcept;
  [[nodiscard]] bool empty() const noexcept { return kind() == ValueKind::Empty; }

  [[nodiscard]] std::int64_t as_int64() const;
  [[nodiscard]] double as_real64() const;
  [[nodiscard]] const String& as_string() const;
  [[nodiscard]] const Handle& as_handle() const;

  friend bool operator==(const Value&, const Value&) = default;

 private:
  using Storage = std::variant<std::monostate, std::int64_t, double, String, Handle>;
  Storage storage_;
};

}  // namespace mv::core
