#pragma once

#include <string_view>

#include <core/exception.hpp>

namespace mv::c_api {
namespace detail {

inline bool contains(std::string_view text, std::string_view token) noexcept {
  return text.find(token) != std::string_view::npos;
}

inline std::string_view infer_parameter(std::string_view message) noexcept {
  if (contains(message, "index")) return "index";
  if (contains(message, "capacity")) return "capacity";
  if (contains(message, "output")) return "output";
  if (contains(message, "image")) return "image";
  if (contains(message, "region")) return "regions";
  if (contains(message, "contour") || contains(message, "XLD")) return "contours";
  if (contains(message, "tuple") || contains(message, "Tuple")) return "tuple";
  if (contains(message, "model")) return "model";
  if (contains(message, "measure")) return "measure";
  if (contains(message, "calibration")) return "calibration_data";
  if (contains(message, "string")) return "value";
  if (contains(message, "interpolation")) return "interpolation";
  if (contains(message, "filter")) return "filter";
  if (contains(message, "mask")) return "mask";
  if (contains(message, "mode")) return "mode";
  if (contains(message, "attribute")) return "attribute";
  if (contains(message, "parameter")) return "parameter";
  return "argument";
}

inline core::ErrorCode infer_code(core::Status status,
                                  std::string_view message) noexcept {
  if (contains(message, "must not be null") || contains(message, "null")) {
    return core::ErrorCode::NullArgument;
  }
  if (status == core::Status::OutOfRange || contains(message, "out of range") ||
      contains(message, "capacity is too small")) {
    return core::ErrorCode::IndexOutOfRange;
  }
  if (status == core::Status::Unsupported || contains(message, "unsupported") ||
      contains(message, "invalid mode") || contains(message, "invalid filter") ||
      contains(message, "invalid mask") || contains(message, "invalid attribute") ||
      contains(message, "unknown parameter")) {
    return core::ErrorCode::UnsupportedValue;
  }
  return core::default_error_code(status);
}

}  // namespace detail

inline core::Exception Exception(core::Status status, std::string_view operation,
                                 std::string_view message) {
  return core::Exception(status, detail::infer_code(status, message), operation,
                         detail::infer_parameter(message), -1, message);
}

inline core::Exception Exception(core::Status status, core::ErrorCode code,
                                 std::string_view operation,
                                 std::string_view parameter,
                                 std::int64_t parameter_index,
                                 std::string_view message) {
  return core::Exception(status, code, operation, parameter, parameter_index,
                         message);
}

}  // namespace mv::c_api
