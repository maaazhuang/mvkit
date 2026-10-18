#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#include <core/status.hpp>

namespace mv::core {

enum class ErrorCode : std::int32_t {
  Unspecified = 0,
  NullArgument = 1,
  InvalidCardinality = 2,
  InvalidType = 3,
  InvalidValue = 4,
  IndexOutOfRange = 5,
  MissingItem = 6,
  UnsupportedValue = 7,
  InvalidObjectState = 8,
  IoFailure = 9,
  InternalFailure = 10,
};

[[nodiscard]] constexpr ErrorCode default_error_code(Status status) noexcept {
  switch (status) {
    case Status::Ok: return ErrorCode::Unspecified;
    case Status::InvalidArgument: return ErrorCode::InvalidValue;
    case Status::OutOfRange: return ErrorCode::IndexOutOfRange;
    case Status::TypeMismatch: return ErrorCode::InvalidType;
    case Status::NotFound: return ErrorCode::MissingItem;
    case Status::Unsupported: return ErrorCode::UnsupportedValue;
    case Status::InvalidState: return ErrorCode::InvalidObjectState;
    case Status::IoError:
    case Status::DecodeError:
    case Status::EncodeError: return ErrorCode::IoFailure;
    case Status::OutOfMemory:
    case Status::LimitExceeded:
    case Status::InternalError: return ErrorCode::InternalFailure;
  }
  return ErrorCode::InternalFailure;
}

class Exception final : public std::runtime_error {
 public:
  Exception(Status status, std::string_view where, std::string_view message);
  Exception(Status status, ErrorCode code, std::string_view where,
            std::string_view parameter, std::int64_t parameter_index,
            std::string_view message);

  [[nodiscard]] Status status() const noexcept { return status_; }
  [[nodiscard]] std::string_view where() const noexcept { return where_; }
  [[nodiscard]] std::string_view message() const noexcept { return message_; }
  [[nodiscard]] ErrorCode error_code() const noexcept { return error_code_; }
  [[nodiscard]] std::string_view parameter() const noexcept { return parameter_; }
  [[nodiscard]] std::int64_t parameter_index() const noexcept { return parameter_index_; }

 private:
  Status status_;
  ErrorCode error_code_{ErrorCode::Unspecified};
  std::string where_;
  std::string parameter_;
  std::int64_t parameter_index_{-1};
  std::string message_;
};

}  // namespace mv::core
