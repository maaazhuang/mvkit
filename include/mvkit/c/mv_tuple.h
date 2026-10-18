#ifndef MVKIT_C_MV_TUPLE_H
#define MVKIT_C_MV_TUPLE_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stddef.h>
#include <stdint.h>

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_status.h>
#include <mvkit/c/mv_string.h>
#include <mvkit/c/mv_value.h>
/** C 算子输入/输出统一使用一种控制元组。输入逻辑只读，输出为拥有型句柄。
 * 拥有型元组使用 mv_create_tuple/mv_copy_tuple/mv_destroy_tuple 创建、复制和销毁。C++ ControlInput() 提供只读借用，源修改或销毁后失效；
 * 需要独立拥有时使用 mv_copy_tuple。算子不保留输入视图；空算子输入表示空元组。 */
typedef struct MvTupleImpl* MvTuple;

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t MvTupleKind;

#define MV_TUPLE_EMPTY ((int32_t)0)
#define MV_TUPLE_INT64 ((int32_t)1)
#define MV_TUPLE_REAL64 ((int32_t)2)
#define MV_TUPLE_STRING ((int32_t)4)
#define MV_TUPLE_HANDLE ((int32_t)16)
#define MV_TUPLE_MIXED ((int32_t)8)
#define MV_TUPLE_ANY ((int32_t)31)

/**
 * @brief 创建指定长度的拥有型控制元组，读取元素前须先设置其值。
 *
 * @param[out] tuple 控制元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[in] length 初始元素数量，0 创建空元组。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_tuple(MvTuple* tuple, size_t length);

/**
 * @brief 克隆控制元组，生成可独立修改和释放的拥有型结果。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[out] clone 克隆结果元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_copy_tuple(MvTuple tuple, MvTuple* clone);

/**
 * @brief 释放拥有型控制元组的一次引用。
 *
 * @param[in] tuple 待释放的一次拥有型控制元组引用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_destroy_tuple(MvTuple tuple);

/**
 * @brief 查询控制元组的元素数量。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[out] size 控制元组的元素数量。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_length_tuple(MvTuple tuple, size_t* size);

/**
 * @brief 获取指定元素的实际类型；混合类型控制元组 必须通过此函数逐元素查询。
 *
 * @details 返回 MV_STATUS_OK 表示成功；索引越界返回 MV_STATUS_OUT_OF_RANGE。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] index 从 0 开始的控制元组元素索引。
 * @param[out] kind 指定元素的实际类型。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_type(MvTuple tuple, size_t index, MvValueKind* kind);

/**
 * @brief 连接两个控制元组，结果保持元素顺序。
 *
 * @param[in] left 待连接或组合的控制元组；调用期间只读借用。
 * @param[in] right 待连接或组合的控制元组；调用期间只读借用。
 * @param[out] result 操作结果元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_tuple_concat(MvTuple left, MvTuple right, MvTuple* result);

/**
 * @brief 按 从 0 开始的整数值索引选择元素；接受无小数部分的 real，保留顺序与重复项。
 *
 * @details 空索引元组 返回空结果；空输入和非空索引返回 MV_STATUS_INVALID_ARGUMENT。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] indices 数据、元素或对象索引元组；调用期间只读借用。
 * @param[out] selected 操作结果元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_tuple_select(MvTuple tuple, MvTuple indices, MvTuple* selected);

/**
 * @brief 选择闭区间 [left_index, right_index]。
 *
 * @details `right_index == left_index - 1` 且 left_index <= length 返回空控制元组；其它反向范围无效。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] left_index 选择区间的起止索引元组；调用期间只读借用。
 * @param[in] right_index 选择区间的起止索引元组；调用期间只读借用。
 * @param[out] selected 操作结果元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_tuple_select_range(
    MvTuple tuple,
    MvTuple left_index,
    MvTuple right_index,
    MvTuple* selected);

/**
 * @brief 读取指定索引的整数元素。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] index 从 0 开始的控制元组元素索引。
 * @param[out] value 保存、追加或读取的数据。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_i(MvTuple tuple, size_t index, int64_t* value);

/**
 * @brief 读取指定索引的实数元素。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] index 从 0 开始的控制元组元素索引。
 * @param[out] value 保存、追加或读取的数据。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_d(MvTuple tuple, size_t index, double* value);

/**
 * @brief 读取指定索引的字符串借用视图。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] index 从 0 开始的控制元组元素索引。
 * @param[out] value 指定元素的借用字符串视图。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_s(MvTuple tuple, size_t index, MvStringView* value);

/**
 * @brief 读取指定索引的通用控制句柄。
 *
 * @details 成功时返回源元组拥有的借用包装器；不得单独释放，源元组修改或销毁后失效。
 * 需要独立拥有时使用 mv_copy_tuple 或 mv_tuple_select 复制控制元组。
 * 失败时不修改 handle。
 *
 * @param[in] tuple 控制元组；调用期间只读借用。
 * @param[in] index 数据、元素或对象索引。
 * @param[out] handle 通用控制句柄的借用值；源元组修改或销毁后失效，不得单独释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_h(MvTuple tuple, size_t index, MvHandle* handle);

/**
 * @brief 调整控制元组长度，保留既有前缀；新增元素读取前须先设置。
 *
 * @param[in,out] tuple 拥有型控制元组，借用输入不能修改。
 * @param[in] length 新的元素数量。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_resize_tuple(MvTuple tuple, size_t length);

/**
 * @brief 设置控制元组中指定索引的64 位整数元素。
 *
 * @param[in,out] tuple 拥有型控制元组，借用输入不能修改。
 * @param[in] value 64 位整数。
 * @param[in] index 从 0 开始的元素索引，必须小于当前长度。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_i(MvTuple tuple, int64_t value, size_t index);

/**
 * @brief 设置控制元组中指定索引的双精度实数元素。
 *
 * @param[in,out] tuple 拥有型控制元组，借用输入不能修改。
 * @param[in] value 双精度实数。
 * @param[in] index 从 0 开始的元素索引，必须小于当前长度。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_d(MvTuple tuple, double value, size_t index);

/**
 * @brief 设置控制元组中指定索引的UTF-8 字符串视图元素。
 *
 * @param[in,out] tuple 拥有型控制元组，借用输入不能修改。
 * @param[in] value UTF-8 字符串视图，复制其内容。
 * @param[in] index 从 0 开始的元素索引，必须小于当前长度。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_s(MvTuple tuple, MvStringView value, size_t index);

/**
 * @brief 设置控制元组中指定索引的通用控制句柄元素。
 *
 * @param[in,out] tuple 拥有型控制元组，借用输入不能修改。
 * @param[in] value 通用控制句柄，保留底层对象的共享引用。
 * @param[in] index 从 0 开始的元素索引，必须小于当前长度。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_h(MvTuple tuple, MvHandle value, size_t index);

/**
 * @brief 查询控制元组的整体类型，返回 HALCON 类型码。
 *
 * @param[in] T 只读借用的控制元组，空输入返回 31（MV_TUPLE_ANY）。
 * @param[out] Type 单元素整数结果元组，由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_tuple_type(MvTuple T, MvTuple* Type);

#ifdef __cplusplus
}
#endif

#endif
