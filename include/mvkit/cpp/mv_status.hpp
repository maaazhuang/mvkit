#pragma once

#include <cstdint>

#include <mvkit/c/mv_status.h>

namespace mv {

/** @brief 与 C ABI 对应的操作状态码。 */
enum class MvStatus : std::int32_t {
  Ok = MV_STATUS_OK,
  InvalidArgument = MV_STATUS_INVALID_ARGUMENT,
  OutOfRange = MV_STATUS_OUT_OF_RANGE,
  TypeMismatch = MV_STATUS_TYPE_MISMATCH,
  NotFound = MV_STATUS_NOT_FOUND,
  OutOfMemory = MV_STATUS_OUT_OF_MEMORY,
  Unsupported = MV_STATUS_UNSUPPORTED,
  InvalidState = MV_STATUS_INVALID_STATE,
  IoError = MV_STATUS_IO_ERROR,
  DecodeError = MV_STATUS_DECODE_ERROR,
  EncodeError = MV_STATUS_ENCODE_ERROR,
  LimitExceeded = MV_STATUS_LIMIT_EXCEEDED,
  InternalError = MV_STATUS_INTERNAL_ERROR,
};

/**
 * @brief 判断状态码是否表示成功。
 *
 * @param[in] status 逐项操作状态。
 *
 * @return 成功时为 true，否则为 false。
 */
constexpr bool succeeded(MvStatus status) noexcept {
  return static_cast<std::int32_t>(status) >= 0;
}

/**
 * @brief 判断状态码是否表示失败。
 *
 * @param[in] status 逐项操作状态。
 *
 * @return 失败时为 true，否则为 false。
 */
constexpr bool failed(MvStatus status) noexcept {
  return !succeeded(status);
}

/**
 * @brief 将 C++ 状态码转换为对应的 C 状态码。
 *
 * @param[in] status 逐项操作状态。
 *
 * @return 对应的 C 状态码。
 */
constexpr ::MvStatus to_native(MvStatus status) noexcept {
  return static_cast<::MvStatus>(status);
}

/**
 * @brief 将 C 状态码转换为对应的 C++ 状态码。
 *
 * @param[in] status 逐项操作状态。
 *
 * @return 对应的 C++ 状态码。
 */
constexpr MvStatus from_native(::MvStatus status) noexcept {
  return static_cast<MvStatus>(status);
}

}  // namespace mv
