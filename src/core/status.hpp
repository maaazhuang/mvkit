#pragma once

#include <cstdint>

namespace mv::core {

enum class Status : std::int32_t {
  Ok = 0,
  InvalidArgument = -1,
  OutOfRange = -2,
  TypeMismatch = -3,
  NotFound = -4,
  OutOfMemory = -5,
  Unsupported = -6,
  InvalidState = -7,
  IoError = -100,
  DecodeError = -101,
  EncodeError = -102,
  LimitExceeded = -103,
  InternalError = -1000,
};

[[nodiscard]] constexpr bool succeeded(Status status) noexcept {
  return static_cast<std::int32_t>(status) >= 0;
}

[[nodiscard]] constexpr bool failed(Status status) noexcept {
  return static_cast<std::int32_t>(status) < 0;
}

}  // namespace mv::core
