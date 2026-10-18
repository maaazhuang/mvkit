#ifndef MVKIT_C_MV_DICT_H
#define MVKIT_C_MV_DICT_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stddef.h>
#include <stdint.h>

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_iconic.h>
#include <mvkit/c/mv_string.h>
#include <mvkit/c/mv_tuple.h>
#include <mvkit/c/mv_handle.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建字典，返回包含一个受检控制句柄的 拥有型控制元组。
 *
 * @param[out] dict 字典元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_dict(MvTuple* dict);

/**
 * @brief 单键保存完整控制元组（可为空），多键使用 scalar-or-N 值。
 *
 * @details 空键配合 0/1 个值不执行操作（HALCON 24.11.1）；空字符串是合法键。输入仅在调用中同步借用。
 *
 * @param[in] dict 字典元组；调用期间只读借用。
 * @param[in] keys 字典键元组；调用期间只读借用。
 * @param[in] value 待保存的数据元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_dict_tuple(MvTuple dict, MvTuple keys, MvTuple value);

/**
 * @brief 多键要求每个键存储一个控制值；失败保持原输出。
 *
 * @param[in] dict 字典元组；调用期间只读借用。
 * @param[in] keys 字典键元组；调用期间只读借用。
 * @param[out] value 读取的数据元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_dict_tuple(MvTuple dict, MvTuple keys, MvTuple* value);

/**
 * @brief HALCON set/get_dict_object 要求一个键，保存完整图标对象数组。
 *
 * @details 对象载荷共享并保留引用；失败保持原状态和输出。
 *
 * @param[in] value 待保存的图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] dict 字典元组；调用期间只读借用。
 * @param[in] key 字典键元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_dict_object(MvObject value, MvTuple dict, MvTuple key);

/**
 * @brief 读取指定字典键保存的完整图标对象数组。
 *
 * @param[out] value 读取的图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] dict 字典元组；调用期间只读借用。
 * @param[in] key 字典键元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_dict_object(MvObject* value, MvTuple dict, MvTuple key);

/**
 * @brief 查询字典的全部键、键存在性或键值种类。
 *
 * @details GenParamName 为单个 keys、key_exists 或 key_data_type；keys 查询要求 Key 为空。
 * key_data_type 按键顺序返回 tuple 或 object，缺失键返回错误且保持原输出。
 *
 * @param[in] DictHandle 包含一个字典句柄的控制元组。
 * @param[in] GenParamName 查询名称元组。
 * @param[in] Key 字符串或整数键元组。
 * @param[out] GenParamValue 结果元组，使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_dict_param(MvTuple DictHandle, MvTuple GenParamName, MvTuple Key, MvTuple* GenParamValue);

/**
 * @brief 删除字典中的指定键及关联数据。
 *
 * @details 某个键无效或不存在时，继续尝试删除其余键，最后报告首个错误；不是原子批量提交。
 *
 * @param[in,out] DictHandle 包含一个字典句柄的控制元组。
 * @param[in] Key 非空的字符串或整数键元组。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_remove_dict_key(MvTuple DictHandle, MvTuple Key);

#ifdef __cplusplus
}
#endif

#endif
