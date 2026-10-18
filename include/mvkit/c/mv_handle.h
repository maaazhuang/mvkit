#ifndef MVKIT_C_MV_HANDLE_H
#define MVKIT_C_MV_HANDLE_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

#include <stdint.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_value.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 类型擦除控制对象的稳定公开类型标识。 */
typedef uint64_t MvObjectType;
#define MV_OBJECT_TYPE_UNKNOWN ((uint64_t)0)
#define MV_OBJECT_TYPE_CALIBRATION_DATA ((uint64_t)2)
#define MV_OBJECT_TYPE_SHAPE_MODEL ((uint64_t)4)
#define MV_OBJECT_TYPE_OBJECT_MODEL_3D ((uint64_t)5)
#define MV_OBJECT_TYPE_CAMERA_SETUP_MODEL ((uint64_t)6)
#define MV_OBJECT_TYPE_STEREO_RECTIFICATION ((uint64_t)7)
#define MV_OBJECT_TYPE_STRUCTURED_LIGHT_MODEL ((uint64_t)8)
#define MV_OBJECT_TYPE_NCC_MODEL ((uint64_t)9)
#define MV_OBJECT_TYPE_DICT ((uint64_t)10)
#define MV_OBJECT_TYPE_STEREO_MODEL ((uint64_t)11)
#define MV_OBJECT_TYPE_SERIALIZED_ITEM ((uint64_t)12)

/**
 * @brief 清除控制句柄的共享内容，使全部别名失效。
 *
 * @details 清理资源与销毁 Tuple 分开；销毁一个元组仅释放其引用，其他拥有者可以继续使用。
 *
 * @param[in,out] Handle 控制句柄元组，清理后不得继续用于模型业务操作。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_clear_handle(MvTuple Handle);

/**
 * @brief 查询控制元组或同类控制句柄的语义类型。
 *
 * @details 清理后的句柄及不同种类的句柄返回 handle；空或混合基础类型返回 any。
 *
 * @param[in] T 只读借用的控制元组。
 * @param[out] SemType 单元素类型字符串元组，使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_tuple_sem_type(MvTuple T, MvTuple* SemType);

/**
 * @brief 将一个控制模型保存为拥有独立字节存储的序列化项。
 * @details 支持 Shape 和 NCC 模型；使用 MVKit 自有版本化格式，不兼容 HALCON
 * 二进制格式。可变模型的序列化与修改须由调用方同步，不保存结果和可重建缓存。
 * @param[in] Handle 单元素有效模型句柄元组，调用期间借用。
 * @param[out] SerializedItem 拥有型序列化句柄元组，使用 mv_destroy_tuple 释放引用；
 * mv_clear_handle 可清除共享内容。失败时输出保持原值。
 * @return 成功返回 MV_STATUS_OK；不支持的模型类型返回 MV_STATUS_UNSUPPORTED。
 */
MV_API MvStatus mv_serialize_handle(MvTuple Handle, MvTuple* SerializedItem);

/**
 * @brief 从序列化项恢复独立的控制模型。
 * @details 自动识别 Shape 或 NCC 模型，验证格式版本、大小和校验和；每次恢复
 * 创建独立模型身份。
 * @param[in] SerializedItem 单元素有效序列化项句柄元组，调用期间借用。
 * @param[out] Handle 拥有型模型句柄元组，使用 mv_destroy_tuple 释放引用；失败时保持原值。
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_deserialize_handle(MvTuple SerializedItem, MvTuple* Handle);

/**
 * @brief 将外部字节缓冲区复制为序列化项。
 * @details 仅支持 Copy="true"；不接管外部内存。这里只复制数据，模型格式在反序列化时
 * 验证。调用方须保证指针覆盖 Size 个可读字节，大小上限为 256 MiB 加 128 字节。
 * @param[in] Pointer 缓冲区地址的单元素整数元组，不允许空地址。
 * @param[in] Size 正字节数的单元素整数元组。
 * @param[in] Copy 单元素复制模式字符串元组，当前仅接受 true。
 * @param[out] SerializedItemHandle 拥有型序列化句柄元组，使用 mv_destroy_tuple 释放引用；失败时保持原值。
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_serialized_item_ptr(MvTuple Pointer, MvTuple Size, MvTuple Copy,
                                             MvTuple* SerializedItemHandle);

/**
 * @brief 借用序列化项的字节地址并查询字节数。
 * @details 地址在序列化项仍有拥有者且未被 clear_handle 清除期间有效；不得修改或释放
 * 借用存储，不得与清除操作并发。需要独立存储时由调用方复制字节。
 * @param[in] SerializedItemHandle 单元素有效序列化项句柄元组。
 * @param[out] Pointer 拥有型地址整数元组，使用 mv_destroy_tuple 释放；字节存储仍为借用。
 * @param[out] Size 拥有型字节数整数元组，使用 mv_destroy_tuple 释放。
 * @return 成功返回 MV_STATUS_OK；失败时两个输出保持原值，输出指针不得互相别名。
 */
MV_API MvStatus mv_get_serialized_item_ptr(MvTuple SerializedItemHandle, MvTuple* Pointer,
                                          MvTuple* Size);

#ifdef __cplusplus
}
#endif

#endif
