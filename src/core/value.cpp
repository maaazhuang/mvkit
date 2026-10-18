#include <core/value.hpp>

#include <cmath>
#include <limits>

#include <core/exception.hpp>

namespace mv::core {

ValueKind Value::kind() const noexcept {
  switch (storage_.index()) {
    case 0: return ValueKind::Empty;
    case 1: return ValueKind::Int64;
    case 2: return ValueKind::Real64;
    case 3: return ValueKind::String;
    case 4: return ValueKind::Handle;
    default: return ValueKind::Empty;
  }
}

std::int64_t Value::as_int64() const {
  if (const auto* value = std::get_if<std::int64_t>(&storage_)) {
    return *value;
  }
  if (const auto* value = std::get_if<double>(&storage_)) {
    constexpr double kMin = static_cast<double>(std::numeric_limits<std::int64_t>::min());
    constexpr double kMaxExclusive = 9223372036854775808.0;  // 2^63
    if (std::isfinite(*value) && std::trunc(*value) == *value && *value >= kMin && *value < kMaxExclusive) {
      return static_cast<std::int64_t>(*value);
    }
  }
  throw Exception(Status::TypeMismatch, "Value::as_int64", "value cannot be represented exactly as Int64");
}

double Value::as_real64() const {
  if (const auto* value = std::get_if<double>(&storage_)) {
    return *value;
  }
  if (const auto* value = std::get_if<std::int64_t>(&storage_)) {
    return static_cast<double>(*value);
  }
  throw Exception(Status::TypeMismatch, "Value::as_real64", "value is not numeric");
}

const String& Value::as_string() const {
  if (const auto* value = std::get_if<String>(&storage_)) {
    return *value;
  }
  throw Exception(Status::TypeMismatch, "Value::as_string", "value is not String");
}

const Handle& Value::as_handle() const {
  if (const auto* value = std::get_if<Handle>(&storage_)) {
    return *value;
  }
  throw Exception(Status::TypeMismatch, "Value::as_handle", "value is not Handle");
}

}  // namespace mv::core
