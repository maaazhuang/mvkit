#ifndef MVKIT_C_MV_STRING_H
#define MVKIT_C_MV_STRING_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stddef.h>

typedef struct MvStringView {
  const char* data;
  size_t size;
} MvStringView;

#endif
