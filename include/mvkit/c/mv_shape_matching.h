#ifndef MVKIT_C_MV_SHAPE_MATCHING_H
#define MVKIT_C_MV_SHAPE_MATCHING_H

/** @file
 * @brief 基于方向梯度的 Shape Matching C ABI。
 */

#include <stdint.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_handle.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_tuple.h>
#include <mvkit/c/mv_xld.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 根据模板图像创建形状匹配模型。
 *
 * @details HALCON C2 创建控制；空 iconic 输入返回空 ModelID。
 * 图像创建要求一张灰度图，全部 XLD 对象组成一个模型；带符号 XLD 度量要求 edge_direction。失败保持 ModelID。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移引用所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] AngleStep 角度采样步长元组；调用期间只读借用。
 * @param[in] Optimization 模型优化方式元组；调用期间只读借用。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[in] Contrast 模板对比度阈值元组；调用期间只读借用。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间只读借用。
 * @param[out] ModelID 形状匹配模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_shape_model(MvObject Template, MvTuple NumLevels,
    MvTuple AngleStart, MvTuple AngleExtent, MvTuple AngleStep,
    MvTuple Optimization, MvTuple Metric, MvTuple Contrast,
    MvTuple MinContrast, MvTuple* ModelID);

/**
 * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移引用所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] AngleStep 角度采样步长元组；调用期间只读借用。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间只读借用。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间只读借用。
 * @param[in] ScaleStep 模型尺度采样步长元组；调用期间只读借用。
 * @param[in] Optimization 模型优化方式元组；调用期间只读借用。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[in] Contrast 模板对比度阈值元组；调用期间只读借用。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间只读借用。
 * @param[out] ModelID 形状匹配模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_scaled_shape_model(MvObject Template, MvTuple NumLevels,
    MvTuple AngleStart, MvTuple AngleExtent, MvTuple AngleStep,
    MvTuple ScaleMin, MvTuple ScaleMax, MvTuple ScaleStep,
    MvTuple Optimization, MvTuple Metric, MvTuple Contrast,
    MvTuple MinContrast, MvTuple* ModelID);

/**
 * @brief 根据 XLD 模板轮廓创建形状匹配模型。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] AngleStep 角度采样步长元组；调用期间只读借用。
 * @param[in] Optimization 模型优化方式元组；调用期间只读借用。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间只读借用。
 * @param[out] ModelID 形状匹配模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_shape_model_xld(MvObject Contours, MvTuple NumLevels,
    MvTuple AngleStart, MvTuple AngleExtent, MvTuple AngleStep,
    MvTuple Optimization, MvTuple Metric, MvTuple MinContrast, MvTuple* ModelID);

/**
 * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] AngleStep 角度采样步长元组；调用期间只读借用。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间只读借用。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间只读借用。
 * @param[in] ScaleStep 模型尺度采样步长元组；调用期间只读借用。
 * @param[in] Optimization 模型优化方式元组；调用期间只读借用。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间只读借用。
 * @param[out] ModelID 形状匹配模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_scaled_shape_model_xld(MvObject Contours, MvTuple NumLevels,
    MvTuple AngleStart, MvTuple AngleExtent, MvTuple AngleStep,
    MvTuple ScaleMin, MvTuple ScaleMax, MvTuple ScaleStep,
    MvTuple Optimization, MvTuple Metric, MvTuple MinContrast, MvTuple* ModelID);

/**
 * @brief 设置形状匹配模型的原点偏移。
 *
 * @details HALCON C2 控制；属性算子要求恰好一个 ModelID。
 * Origin 为相对默认参考点的偏移，不是模板坐标；ModelContours 的金字塔层数为 1-based。失败保持全部输出。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_shape_model_origin(MvTuple ModelID, MvTuple Row, MvTuple Column);

/**
 * @brief 查询形状匹配模型的原点偏移。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_shape_model_origin(MvTuple ModelID, MvTuple* Row, MvTuple* Column);

/**
 * @brief 获取指定金字塔层级的形状模型轮廓。
 *
 * @param[out] ModelContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] Level 模型轮廓的金字塔层级元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_shape_model_contours(MvObject* ModelContours, MvTuple ModelID, MvTuple Level);

/**
 * @brief 设置形状匹配模型的指定参数。
 *
 * @details 支持 min_contrast、timeout、border_shape_models、model_cache。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_shape_model_param(MvTuple ModelID, MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief 查询形状匹配模型的训练和搜索参数。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[out] NumLevels 金字塔层级控制元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleStart 起始角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleExtent 角度区间长度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] AngleStep 角度采样步长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ScaleMin 模型尺度下限元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ScaleMax 模型尺度上限元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ScaleStep 模型尺度采样步长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Metric 距离或匹配度量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] MinContrast 模板对比度阈值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_shape_model_params(MvTuple ModelID, MvTuple* NumLevels,
    MvTuple* AngleStart, MvTuple* AngleExtent, MvTuple* AngleStep,
    MvTuple* ScaleMin, MvTuple* ScaleMax, MvTuple* ScaleStep, MvTuple* Metric, MvTuple* MinContrast);

/**
 * @brief 在图像中搜索形状匹配模型。
 *
 * @details HALCON C2 搜索控制，返回拥有型平行结果列。空 iconic 输入在验证控制前返回空结果。
 * 非空 Image 为一个对象，或在多模型搜索中每模型一个对象；失败保持全部输出。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] Greediness 搜索贪婪度元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_shape_model(MvObject Image,
    MvTuple ModelID, MvTuple AngleStart, MvTuple AngleExtent, MvTuple MinScore, MvTuple NumMatches, MvTuple MaxOverlap, MvTuple SubPixel, MvTuple NumLevels, MvTuple Greediness,
    MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score);

/**
 * @brief 在图像中搜索支持尺度变化的形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间只读借用。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] Greediness 搜索贪婪度元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Scale 坐标、灰度或模型的缩放控制元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_scaled_shape_model(MvObject Image,
    MvTuple ModelID, MvTuple AngleStart, MvTuple AngleExtent, MvTuple ScaleMin, MvTuple ScaleMax, MvTuple MinScore, MvTuple NumMatches, MvTuple MaxOverlap, MvTuple SubPixel, MvTuple NumLevels, MvTuple Greediness,
    MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score);

/**
 * @brief 在图像中搜索多个形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelIDs 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] Greediness 搜索贪婪度元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Model 模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_shape_models(MvObject Image,
    MvTuple ModelIDs, MvTuple AngleStart, MvTuple AngleExtent, MvTuple MinScore, MvTuple NumMatches, MvTuple MaxOverlap, MvTuple SubPixel, MvTuple NumLevels, MvTuple Greediness,
    MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model);

/**
 * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] ModelIDs 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleExtent 角度区间长度元组；调用期间只读借用。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间只读借用。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间只读借用。
 * @param[in] MinScore 匹配得分下限元组；调用期间只读借用。
 * @param[in] NumMatches 返回匹配数量的控制元组；调用期间只读借用。
 * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间只读借用。
 * @param[in] SubPixel 亚像素细化方式元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] Greediness 搜索贪婪度元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Scale 坐标、灰度或模型的缩放控制元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Model 模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_scaled_shape_models(MvObject Image,
    MvTuple ModelIDs, MvTuple AngleStart, MvTuple AngleExtent, MvTuple ScaleMin, MvTuple ScaleMax, MvTuple MinScore, MvTuple NumMatches, MvTuple MaxOverlap, MvTuple SubPixel, MvTuple NumLevels, MvTuple Greediness,
    MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score, MvTuple* Model);

/**
 * @brief MVKit 原生模型持久化，不能与 HALCON 二进制文件互换。
 *
 * @details FileName 为单个 UTF-8 字符串，表示完整文件路径。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_shape_model(MvTuple ModelID, MvTuple FileName);

/**
 * @brief 从 MVKit 原生格式文件读取形状匹配模型。
 *
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[out] ModelID 形状匹配模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_shape_model(MvTuple FileName, MvTuple* ModelID);

#ifdef __cplusplus
}
#endif
#endif
