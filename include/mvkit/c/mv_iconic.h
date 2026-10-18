#ifndef MVKIT_C_MV_ICONIC_H
#define MVKIT_C_MV_ICONIC_H

/** @file
 * @brief HALCON 风格 iconic 对象元组 C ABI。 */

#include <stddef.h>
#include <stdint.h>

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 不透明 iconic 数组，可包含 0/1/N 张图像、区域或 XLD 轮廓。多通道图像仍是一个对象。
 * 空句柄非法，mv_gen_empty_obj 创建的是有效零对象数组。
 * C++ 使用 mv::MvObject 及强类型视图；::MvObject 为 C 句柄。 */
typedef struct MvObjectImpl* MvObject;

typedef int32_t MvIconicKind;
#define MV_ICONIC_IMAGE ((int32_t)0)
#define MV_ICONIC_REGION ((int32_t)1)
#define MV_ICONIC_XLD_CONTOUR ((int32_t)2)

/**
 * @brief 清理图标对象引用，最后一个使用者退出后回收共享载荷。
 *
 * @param[in] objects 待释放的一次拥有型图标对象引用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_clear_obj(MvObject objects);

/**
 * @brief 创建不含任何图标对象的空对象数组。
 *
 * @details HALCON 风格对象元组算子。
 *
 * @param[out] objects 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_empty_obj(MvObject* objects);

/**
 * @brief 查询图标对象数组的对象数量。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] count 图标对象数量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_count_obj(MvObject objects, MvTuple* count);

/**
 * @brief 按 从 1 开始的对象索引选择图标对象。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] selected 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] index 数据、元素或对象索引元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_obj(MvObject objects, MvObject* selected,
                              MvTuple index);

/**
 * @brief 按顺序拼接两个图标对象数组。
 *
 * @param[in] left 左侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] right 右侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] concatenated 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_concat_obj(MvObject left, MvObject right,
                              MvObject* concatenated);

/**
 * @brief 复制指定范围的图标对象引用并共享其载荷。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] copied 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] index 数据、元素或对象索引元组；调用期间只读借用。
 * @param[in] num_obj 对象数量控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_copy_obj(MvObject objects, MvObject* copied,
                            MvTuple index, MvTuple num_obj);

/**
 * @brief 查询各图标对象的种类，依次返回 image、region 或 xld_cont。
 *
 * @param[in] Objects 只读借用的图标对象数组。
 * @param[out] Classes 种类字符串元组，使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_obj_class(MvObject Objects, MvTuple* Classes);

/**
 * @brief 深复制图像像素及定义域，不共享像素存储。
 *
 * @param[in] Image 只读借用的图像对象数组。
 * @param[out] DupImage 独立图像对象数组，使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_copy_image(MvObject Image, MvObject* DupImage);

#ifdef __cplusplus
}
#endif

#endif
