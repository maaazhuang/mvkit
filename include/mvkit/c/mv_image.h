#ifndef MVKIT_C_MV_IMAGE_H
#define MVKIT_C_MV_IMAGE_H

/** @file
 * @brief HALCON 风格的图像创建、内存访问及文件接口。 */

#include <stddef.h>
#include <stdint.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_iconic.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 对象统一使用 MvObject。空数组、copy_obj/clear_obj、count/select/concat 和 copy_image 见 mv_iconic.h。
 * 像素指针以不截断地址的整数控制元组传入；图像通道使用统一类型。 */
/**
 * @brief 创建像素值为零的单通道图像。
 *
 * @details 支持 byte/uint2/int2/int4/real，Width 和 Height 为正整数。
 *
 * @param[out] Image 图像对象；输出成功后使用 mv_clear_obj 释放。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image_const(MvObject* Image, MvTuple Type, MvTuple Width, MvTuple Height);

/**
 * @brief 从连续行存储的像素指针复制创建单通道图像。
 *
 * @details 调用期间复制像素，返回后源缓冲区可以释放；支持 byte/uint2/int2/int4/real。
 *
 * @param[out] Image 图像对象；输出成功后使用 mv_clear_obj 释放。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image1(MvObject* Image, MvTuple Type, MvTuple Width, MvTuple Height, MvTuple PixelPointer);

/**
 * @brief 从三个连续行存储的同类型平面复制创建三通道图像。
 *
 * @details 三个平面采用同一 Type；不提供每通道独立类型或释放上下文。
 *
 * @param[out] ImageRGB 三通道图像对象；输出成功后使用 mv_clear_obj 释放。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointerRed 像素平面实际地址的整数控制元组。
 * @param[in] PixelPointerGreen 像素平面实际地址的整数控制元组。
 * @param[in] PixelPointerBlue 像素平面实际地址的整数控制元组。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image3(MvObject* ImageRGB, MvTuple Type, MvTuple Width, MvTuple Height, MvTuple PixelPointerRed, MvTuple PixelPointerGreen, MvTuple PixelPointerBlue);

/**
 * @brief 引用连续行存储的外部单通道像素，最后一次引用退出时执行清理。
 *
 * @details ClearProc 为 void (*)(void*) 的地址，接收原像素指针；0 表示不清理。只有创建成功才接管回调，回调不得抛异常。
 *
 * @param[out] Image 图像对象；输出成功后使用 mv_clear_obj 释放。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 * @param[in] ClearProc 清理函数地址整数元组，0 表示不清理。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image1_extern(MvObject* Image, MvTuple Type, MvTuple Width, MvTuple Height, MvTuple PixelPointer, MvTuple ClearProc);

/**
 * @brief 从连续行存储的交错 RGB/BGR 数据复制创建三通道图像。
 *
 * @details 当前支持 byte 输出及 rgb/bgr/rgbx/bgrx、BitsPerChannel=-1/8、BitShift=0；Alignment 为保留参数。宽高为 0 时使用源尺寸，起点从 0 开始；其他编码明确返回不支持。
 *
 * @param[out] ImageRGB 三通道图像对象；输出成功后使用 mv_clear_obj 释放。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 * @param[in] ColorFormat 交错颜色排列名称。
 * @param[in] OriginalWidth 源图像宽度。
 * @param[in] OriginalHeight 源图像高度。
 * @param[in] Alignment 保留的整数控制参数。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] ImageWidth 输出裁剪宽度，0 使用源宽度。
 * @param[in] ImageHeight 输出裁剪高度，0 使用源高度。
 * @param[in] StartRow 从 0 开始的裁剪起始行。
 * @param[in] StartColumn 从 0 开始的裁剪起始列。
 * @param[in] BitsPerChannel 输出通道有效位数。
 * @param[in] BitShift 输入数据右移位数。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image_interleaved(MvObject* ImageRGB, MvTuple PixelPointer, MvTuple ColorFormat, MvTuple OriginalWidth, MvTuple OriginalHeight, MvTuple Alignment, MvTuple Type, MvTuple ImageWidth, MvTuple ImageHeight, MvTuple StartRow, MvTuple StartColumn, MvTuple BitsPerChannel, MvTuple BitShift);

/**
 * @brief 获取各图像第一通道的像素借用指针、类型和尺寸。
 *
 * @details 指针以整数元组返回，仅在图像仍存在时有效；写入会影响共享像素，不执行隐式分离。需要独立可写数据先调用 copy_image。
 *
 * @param[in] Image 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] Pointer 像素平面实际地址的整数控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Type 像素类型名称控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Width 图像宽度控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Height 图像高度控制元组。 成功后使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_image_pointer1(MvObject Image, MvTuple* Pointer, MvTuple* Type, MvTuple* Width, MvTuple* Height);

/**
 * @brief 获取各三通道图像的像素借用指针、统一类型和尺寸。
 *
 * @details 要求三个通道具有相同类型且连续存储；指针借用及写入规则同 get_image_pointer1。
 *
 * @param[in] ImageRGB 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] PointerRed 像素平面实际地址的整数控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] PointerGreen 像素平面实际地址的整数控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] PointerBlue 像素平面实际地址的整数控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Type 像素类型名称控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Width 图像宽度控制元组。 成功后使用 mv_destroy_tuple 释放。
 * @param[out] Height 图像高度控制元组。 成功后使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_image_pointer3(MvObject ImageRGB, MvTuple* PointerRed, MvTuple* PointerGreen, MvTuple* PointerBlue, MvTuple* Type, MvTuple* Width, MvTuple* Height);

/**
 * @brief 按对象顺序获取图像第一通道的像素类型。
 *
 * @param[in] Image 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] Type 像素类型名称控制元组。 成功后使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_image_type(MvObject Image, MvTuple* Type);

/**
 * @brief 按对象顺序查询图像尺寸。
 *
 * @param[in] Image 输入图像对象数组。
 * @param[out] Width 结果拥有型元组，成功后使用 mv_destroy_tuple 释放。
 * @param[out] Height 结果拥有型元组，成功后使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_image_size(MvObject Image, MvTuple* Width, MvTuple* Height);

/**
 * @brief 按对象顺序查询图像通道数量。
 *
 * @param[in] MultiChannelImage 输入图像对象数组。
 * @param[out] Channels 结果拥有型元组，成功后使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_count_channels(MvObject MultiChannelImage, MvTuple* Channels);

/**
 * @brief 从指定文件读取图像对象数组。
 *
 * @details HALCON 正式算子： 从文件读取图像。
 *
 * @param[out] image 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] filenames 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_image(MvObject* image, MvTuple filenames);

/**
 * @brief 将图像写入文件。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Format 图像或模型的文件格式元组；调用期间只读借用。
 * @param[in] FillColor 图像定义域外的填充灰度元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_image(MvObject Image, MvTuple Format, MvTuple FillColor, MvTuple FileName);

#ifdef __cplusplus
}
#endif

#endif
