#ifndef MVKIT_C_MV_API_H
#define MVKIT_C_MV_API_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stdint.h>
#include <mvkit/c/mv_status.h>
#include <mvkit/c/mv_string.h>

#if defined(_WIN32)
  #if defined(MVKIT_BUILD_SHARED)
    #define MV_API __declspec(dllexport)
  #elif defined(MVKIT_USE_SHARED)
    #define MV_API __declspec(dllimport)
  #else
    #define MV_API
  #endif
#else
  #if defined(__GNUC__) || defined(__clang__)
    #define MV_API __attribute__((visibility("default")))
  #else
    #define MV_API
  #endif
#endif

#if defined(__cplusplus)
  #define MV_DEPRECATED(message) [[deprecated(message)]]
#elif defined(_MSC_VER)
  #define MV_DEPRECATED(message) __declspec(deprecated(message))
#elif defined(__GNUC__) || defined(__clang__)
  #define MV_DEPRECATED(message) __attribute__((deprecated(message)))
#else
  #define MV_DEPRECATED(message)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t MvErrorCode;
#define MV_ERROR_UNSPECIFIED ((int32_t)0)
#define MV_ERROR_NULL_ARGUMENT ((int32_t)1)
#define MV_ERROR_INVALID_CARDINALITY ((int32_t)2)
#define MV_ERROR_INVALID_TYPE ((int32_t)3)
#define MV_ERROR_INVALID_VALUE ((int32_t)4)
#define MV_ERROR_INDEX_OUT_OF_RANGE ((int32_t)5)
#define MV_ERROR_MISSING_ITEM ((int32_t)6)
#define MV_ERROR_UNSUPPORTED_VALUE ((int32_t)7)
#define MV_ERROR_INVALID_OBJECT_STATE ((int32_t)8)
#define MV_ERROR_IO_FAILURE ((int32_t)9)
#define MV_ERROR_INTERNAL_FAILURE ((int32_t)10)
#define MV_ERROR_PARAMETER_INDEX_NONE ((int64_t)-1)

/** 结构化错误详情；所有字符串视图均为当前线程持有的借用视图。 */
typedef struct MvErrorInfo {
  MvStatus status;
  MvErrorCode code;
  MvStringView operation;
  MvStringView parameter;
  int64_t parameter_index;
  MvStringView message;
} MvErrorInfo;

/**
 * @brief 获取当前线程最近一次 C ABI 错误的文本视图。
 *
 * @details 获取当前线程最近一次 C ABI 错误信息。
 *
 * @param[out] error 当前线程最近一次错误的借用字符串视图。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_last_error(MvStringView* error);

/**
 * @brief 获取当前线程最近一次 C ABI 错误的结构化详情。
 *
 * @details 获取当前线程最近一次 C ABI 错误的结构化信息。
 * 返回的 字符串视图 由运行库持有，在当前线程下一次 MVKit C ABI 调用前有效。
 *
 * @param[out] error 当前线程最近一次错误的结构化详情。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_last_error_info(MvErrorInfo* error);

#ifdef __cplusplus
}
#endif

#endif
