#include "error_state.hpp"

#include <mvkit/c/mv_api.h>

namespace mv::c_api {
namespace {

struct ErrorState {
  bool dirty{false};
  ::MvStatus status{MV_STATUS_OK};
  ::MvErrorCode code{MV_ERROR_UNSPECIFIED};
  std::string operation;
  std::string parameter;
  std::int64_t parameter_index{MV_ERROR_PARAMETER_INDEX_NONE};
  std::string message;
  std::string composed;
};

thread_local ErrorState g_last_error;

::MvStringView view(const std::string& value) noexcept {
  return {value.data(), value.size()};
}

}  // namespace

void clear_last_error() noexcept {
  // 错误状态由本线程独占；连续成功调用无需反复清空空字符串。
  if (!g_last_error.dirty) return;
  g_last_error.dirty = false;
  g_last_error.status = MV_STATUS_OK;
  g_last_error.code = MV_ERROR_UNSPECIFIED;
  g_last_error.operation.clear();
  g_last_error.parameter.clear();
  g_last_error.parameter_index = MV_ERROR_PARAMETER_INDEX_NONE;
  g_last_error.message.clear();
  g_last_error.composed.clear();
}

void set_last_error(::MvStatus status,
                    std::string_view operation,
                    std::string_view message,
                    ::MvErrorCode code,
                    std::string_view parameter,
                    std::int64_t parameter_index) noexcept {
  try {
    ErrorState next;
    next.dirty = true;
    next.status = status;
    next.code = code;
    next.operation.assign(operation.data(), operation.size());
    next.parameter.assign(parameter.data(), parameter.size());
    next.parameter_index = parameter_index;
    next.message.assign(message.data(), message.size());
    if (next.operation.empty()) {
      next.composed = next.message;
    } else if (next.message.empty()) {
      next.composed = next.operation;
    } else {
      next.composed.reserve(next.operation.size() + next.message.size() + 2);
      next.composed.append(next.operation);
      next.composed.append(": ");
      next.composed.append(next.message);
    }
    g_last_error = std::move(next);
  } catch (...) {
    g_last_error.dirty = true;
    try {
      g_last_error.status = status;
      g_last_error.code = code;
      g_last_error.operation.clear();
      g_last_error.parameter.clear();
      g_last_error.parameter_index = MV_ERROR_PARAMETER_INDEX_NONE;
      g_last_error.message.assign("failed to store error details");
      g_last_error.composed = g_last_error.message;
    } catch (...) {
      g_last_error.status = status;
      g_last_error.code = code;
      g_last_error.operation.clear();
      g_last_error.parameter.clear();
      g_last_error.parameter_index = MV_ERROR_PARAMETER_INDEX_NONE;
      g_last_error.message.clear();
      g_last_error.composed.clear();
    }
  }
}

std::string_view last_error() noexcept { return g_last_error.composed; }

::MvErrorInfo last_error_info() noexcept {
  return {g_last_error.status, g_last_error.code, view(g_last_error.operation),
          view(g_last_error.parameter), g_last_error.parameter_index,
          view(g_last_error.message)};
}

::MvStatus to_c_status(core::Status status) noexcept {
  return static_cast<::MvStatus>(status);
}

::MvErrorCode to_c_error_code(core::ErrorCode code) noexcept {
  switch (code) {
    case core::ErrorCode::Unspecified: return MV_ERROR_UNSPECIFIED;
    case core::ErrorCode::NullArgument: return MV_ERROR_NULL_ARGUMENT;
    case core::ErrorCode::InvalidCardinality: return MV_ERROR_INVALID_CARDINALITY;
    case core::ErrorCode::InvalidType: return MV_ERROR_INVALID_TYPE;
    case core::ErrorCode::InvalidValue: return MV_ERROR_INVALID_VALUE;
    case core::ErrorCode::IndexOutOfRange: return MV_ERROR_INDEX_OUT_OF_RANGE;
    case core::ErrorCode::MissingItem: return MV_ERROR_MISSING_ITEM;
    case core::ErrorCode::UnsupportedValue: return MV_ERROR_UNSUPPORTED_VALUE;
    case core::ErrorCode::InvalidObjectState: return MV_ERROR_INVALID_OBJECT_STATE;
    case core::ErrorCode::IoFailure: return MV_ERROR_IO_FAILURE;
    case core::ErrorCode::InternalFailure: return MV_ERROR_INTERNAL_FAILURE;
  }
  return MV_ERROR_INTERNAL_FAILURE;
}

}  // namespace mv::c_api

extern "C" ::MvStatus mv_get_last_error(::MvStringView* error) {
  if (error == nullptr) return MV_STATUS_INVALID_ARGUMENT;
  const auto text = mv::c_api::last_error();
  error->data = text.data();
  error->size = text.size();
  return MV_STATUS_OK;
}

extern "C" ::MvStatus mv_get_last_error_info(::MvErrorInfo* error) {
  if (error == nullptr) return MV_STATUS_INVALID_ARGUMENT;
  *error = mv::c_api::last_error_info();
  return MV_STATUS_OK;
}
