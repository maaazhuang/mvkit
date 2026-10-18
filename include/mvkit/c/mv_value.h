#ifndef MVKIT_C_MV_VALUE_H
#define MVKIT_C_MV_VALUE_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stdint.h>
#include <mvkit/c/mv_string.h>

/** 控制对象（模型、字典等），区别于 iconic MvObject。 */
typedef struct MvHandleImpl* MvHandle;

typedef int32_t MvValueKind;
#define MV_VALUE_EMPTY ((int32_t)0)
#define MV_VALUE_INT64 ((int32_t)1)
#define MV_VALUE_REAL64 ((int32_t)2)
#define MV_VALUE_STRING ((int32_t)4)
#define MV_VALUE_HANDLE ((int32_t)16)

typedef struct MvValue {
  MvValueKind kind;
  union {
    int64_t int64_value;
    double real64_value;
    MvStringView string_value;
    MvHandle handle_value;
  } data;
} MvValue;

#endif
