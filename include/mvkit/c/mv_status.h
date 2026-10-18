#ifndef MVKIT_C_MV_STATUS_H
#define MVKIT_C_MV_STATUS_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stdint.h>

typedef int32_t MvStatus;

#define MV_STATUS_OK ((int32_t)0)
#define MV_STATUS_INVALID_ARGUMENT ((int32_t)-1)
#define MV_STATUS_OUT_OF_RANGE ((int32_t)-2)
#define MV_STATUS_TYPE_MISMATCH ((int32_t)-3)
#define MV_STATUS_NOT_FOUND ((int32_t)-4)
#define MV_STATUS_OUT_OF_MEMORY ((int32_t)-5)
#define MV_STATUS_UNSUPPORTED ((int32_t)-6)
#define MV_STATUS_INVALID_STATE ((int32_t)-7)
#define MV_STATUS_IO_ERROR ((int32_t)-100)
#define MV_STATUS_DECODE_ERROR ((int32_t)-101)
#define MV_STATUS_ENCODE_ERROR ((int32_t)-102)
#define MV_STATUS_LIMIT_EXCEEDED ((int32_t)-103)
#define MV_STATUS_INTERNAL_ERROR ((int32_t)-1000)

#endif
