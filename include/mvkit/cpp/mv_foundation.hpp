#pragma once

/** HALCON 风格接口共享的轻量 C++ 基础类型。
 * 对象和元组所有权由实际 Mv* 类实现。 */
#include <mvkit/c/mv_api.h>

#include <cstddef>
#include <cstdint>
#include <mvkit/cpp/mv_export.hpp>
#include <mvkit/cpp/mv_status.hpp>

namespace mv {


/**
 * @brief 不拥有字符存储的 UTF-8 字符串视图。
 *
 * @details 视图不延长字符存储的生命周期；使用期间必须保持来源存储有效。
 */
struct MvStringView {
  const char* data;  ///< 借用的 UTF-8 字符存储。
  std::size_t size;  ///< 视图长度，单位为字节。

  /**
   * @brief 构造空字符串视图。
   */
  constexpr MvStringView() noexcept : data(nullptr), size(0) {}

  /**
   * @brief 借用字符存储构造字符串视图。
   *
   * @param[in] value 借用的字符存储，不转移所有权。
   * @param[in] length 视图的字节长度。
   */
  constexpr MvStringView(const char* value, std::size_t length) noexcept : data(value), size(length) {}

  /**
   * @brief 借用字符存储构造字符串视图。
   *
   * @param[in] literal 借用的字符数组，长度截止到首个空字符或数组末尾。
   */
  template <std::size_t N>
  constexpr MvStringView(const char (&literal)[N]) noexcept : data(literal), size(bounded_length(literal)) {}

 private:
  /**
   * @brief 计算字符数组中首个空字符之前的长度。
   *
   * @param[in] value 借用的字符存储，不转移所有权。
   *
   * @return 首个空字符之前的字节数；无空字符时为数组长度。
   */
  template <std::size_t N>
  static constexpr std::size_t bounded_length(const char (&value)[N]) noexcept {
    std::size_t length = 0;
    while (length < N && value[length] != '\0') {
      ++length;
    }
    return length;
  }

};

/** @brief 对参数、对象状态及底层失败原因的具体错误分类。 */
enum class MvErrorCode : std::int32_t {
  Unspecified = MV_ERROR_UNSPECIFIED,
  NullArgument = MV_ERROR_NULL_ARGUMENT,
  InvalidCardinality = MV_ERROR_INVALID_CARDINALITY,
  InvalidType = MV_ERROR_INVALID_TYPE,
  InvalidValue = MV_ERROR_INVALID_VALUE,
  IndexOutOfRange = MV_ERROR_INDEX_OUT_OF_RANGE,
  MissingItem = MV_ERROR_MISSING_ITEM,
  UnsupportedValue = MV_ERROR_UNSUPPORTED_VALUE,
  InvalidObjectState = MV_ERROR_INVALID_OBJECT_STATE,
  IoFailure = MV_ERROR_IO_FAILURE,
  InternalFailure = MV_ERROR_INTERNAL_FAILURE,
};

/**
 * @brief 根据状态码取得默认的具体错误码。
 *
 * @param[in] status 逐项操作状态。
 *
 * @return 状态码对应的默认错误码。
 */
constexpr MvErrorCode default_error_code(MvStatus status) noexcept {
  switch (status) {
    case MvStatus::Ok:
      return MvErrorCode::Unspecified;
    case MvStatus::InvalidArgument:
      return MvErrorCode::InvalidValue;
    case MvStatus::OutOfRange:
      return MvErrorCode::IndexOutOfRange;
    case MvStatus::TypeMismatch:
      return MvErrorCode::InvalidType;
    case MvStatus::NotFound:
      return MvErrorCode::MissingItem;
    case MvStatus::Unsupported:
      return MvErrorCode::UnsupportedValue;
    case MvStatus::InvalidState:
      return MvErrorCode::InvalidObjectState;
    case MvStatus::IoError:
    case MvStatus::DecodeError:
    case MvStatus::EncodeError:
      return MvErrorCode::IoFailure;
    case MvStatus::OutOfMemory:
    case MvStatus::LimitExceeded:
    case MvStatus::InternalError:
      return MvErrorCode::InternalFailure;
  }
  return MvErrorCode::InternalFailure;
}

}  // namespace mv
