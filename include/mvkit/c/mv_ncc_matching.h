#ifndef MVKIT_C_MV_NCC_MATCHING_H
#define MVKIT_C_MV_NCC_MATCHING_H

#include <stddef.h>
#include <stdint.h>
#include <mvkit/c/mv_tuple.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_handle.h>
#include <mvkit/c/mv_image.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
 *
 * @details ModelID 为拥有型 Tuple，包含一个强类型控制句柄。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移引用所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] AngleStep 角度采样步长元组；调用期间只读借用。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[out] ModelID NCC模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_ncc_model(MvObject Template,MvTuple NumLevels,
    MvTuple AngleStart,MvTuple AngleExtent,MvTuple AngleStep,
    MvTuple Metric,MvTuple* ModelID);

/**
 * @brief 查询 NCC 模型的训练和搜索参数。
 *
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间只读借用。
 * @param[out] NumLevels 金字塔层级控制元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleStart 起始角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleExtent 角度区间长度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleStep 角度采样步长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Metric 距离或匹配度量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_ncc_model_params(MvTuple ModelID,MvTuple* NumLevels,
    MvTuple* AngleStart,MvTuple* AngleExtent,MvTuple* AngleStep,MvTuple* Metric);

/**
 * @brief 在图像中搜索 NCC 匹配模型。
 *
 * @details 标准搜索控制；NumLevels 为最高层或 [最高层,最低层]，层数从 1 开始。
 * 全部结果列均为拥有型 Tuple，同时提交。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_ncc_model(MvObject Image,MvTuple ModelID,
    MvTuple AngleStart,MvTuple AngleExtent,MvTuple MinScore,
    MvTuple NumMatches,MvTuple MaxOverlap,MvTuple SubPixel,
    MvTuple NumLevels,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score);

/**
 * @brief 将 NCC 模型写入 MVKit 原生格式文件。
 *
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_ncc_model(MvTuple ModelID,MvTuple FileName);

/**
 * @brief 从 MVKit 原生格式文件读取 NCC 模型。
 *
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[out] ModelID NCC模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_ncc_model(MvTuple FileName,MvTuple* ModelID);

/**
 * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
 *
 * @details NumLevels 接受 1/N 个最高层、共享层对（N=2 时除外）或 N 个交错层对。
 * Model 为 ModelIDs 中的 从 0 开始 索引。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelIDs 包含NCC模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Model 模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_ncc_models(MvObject Image,MvTuple ModelIDs,
    MvTuple AngleStart,MvTuple AngleExtent,MvTuple MinScore,
    MvTuple NumMatches,MvTuple MaxOverlap,MvTuple SubPixel,
    MvTuple NumLevels,MvTuple* Row,MvTuple* Column,MvTuple* Angle,
    MvTuple* Score,MvTuple* Model);

#ifdef __cplusplus
}
#endif
#endif
