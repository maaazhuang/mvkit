#pragma once
#include <cstddef>
#include <cstdint>

// 历史回归使用的原生存储描述，不安装到 SDK。
typedef int32_t MvPixelType;
#define MV_PIXEL_UINT8 ((int32_t)0)
#define MV_PIXEL_UINT16 ((int32_t)1)
#define MV_PIXEL_INT16 ((int32_t)2)
#define MV_PIXEL_INT32 ((int32_t)3)
#define MV_PIXEL_FLOAT32 ((int32_t)4)

typedef int32_t MvChannelOrder;
#define MV_CHANNEL_ORDER_RGB ((int32_t)0)
#define MV_CHANNEL_ORDER_BGR ((int32_t)1)

/**
 * @brief 释放成功包装的外部图像缓冲区。
 *
 * @details 在对应平面缓冲区的最后一次引用释放时调用；回调不得抛出异常。
 *
 * @param[in] context 包装时提供的 release_context。
 *
 * @return 无返回值。
 */
typedef void (*MvReleaseCallback)(void* context);

typedef struct MvImageInfo {
  size_t width;
  size_t height;
  size_t channel_count;
} MvImageInfo;

typedef struct MvImagePlaneInfo {
  size_t stride;
  MvPixelType pixel_type;
} MvImagePlaneInfo;

typedef struct MvImagePlane {
  MvPixelType pixel_type;
  void* data;
  size_t stride;
  MvReleaseCallback release_callback;
  void* release_context;
} MvImagePlane;
