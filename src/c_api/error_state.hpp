#pragma once

#include <exception>
#include <new>
#include <string>
#include <string_view>
#include <utility>

#include <mvkit/c/mv_api.h>
#include <core/exception.hpp>
#include <core/runtime_context.hpp>
#include "structured_error.hpp"

namespace mv::c_api {

void clear_last_error() noexcept;
void set_last_error(::MvStatus status,
                    std::string_view operation,
                    std::string_view message,
                    ::MvErrorCode code = MV_ERROR_UNSPECIFIED,
                    std::string_view parameter = {},
                    std::int64_t parameter_index = MV_ERROR_PARAMETER_INDEX_NONE) noexcept;
[[nodiscard]] std::string_view last_error() noexcept;
[[nodiscard]] ::MvErrorInfo last_error_info() noexcept;
[[nodiscard]] ::MvStatus to_c_status(core::Status status) noexcept;
[[nodiscard]] ::MvErrorCode to_c_error_code(core::ErrorCode code) noexcept;

template <class Fn>
::MvStatus translate(Fn&& fn) noexcept {
  try {
    core::initialize_runtime_context();
    std::forward<Fn>(fn)();
    clear_last_error();
    return MV_STATUS_OK;
  } catch (const core::Exception& ex) {
    const ::MvStatus status = to_c_status(ex.status());
    const auto parameter = ex.parameter().empty()
                               ? detail::infer_parameter(ex.message())
                               : ex.parameter();
    const auto code = ex.error_code() == core::ErrorCode::Unspecified
                          ? detail::infer_code(ex.status(), ex.message())
                          : ex.error_code();
    set_last_error(status, ex.where(), ex.message(),
                   to_c_error_code(code), parameter,
                   ex.parameter_index());
    return status;
  } catch (const std::bad_alloc&) {
    set_last_error(MV_STATUS_OUT_OF_MEMORY, {}, "out of memory", MV_ERROR_INTERNAL_FAILURE);
    return MV_STATUS_OUT_OF_MEMORY;
  } catch (const std::exception& ex) {
    set_last_error(MV_STATUS_INTERNAL_ERROR, {}, ex.what(), MV_ERROR_INTERNAL_FAILURE);
    return MV_STATUS_INTERNAL_ERROR;
  } catch (...) {
    set_last_error(MV_STATUS_INTERNAL_ERROR, {}, "unknown internal error", MV_ERROR_INTERNAL_FAILURE);
    return MV_STATUS_INTERNAL_ERROR;
  }
}

}  // namespace mv::c_api
