#ifndef MVKIT_C_MV_STRUCTURED_LIGHT_H
#define MVKIT_C_MV_STRUCTURED_LIGHT_H

#include <mvkit/c/mv_calibration.h>
#include <mvkit/c/mv_reconstruction3d.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建指定类型的结构光模型。
 *
 * @details F24 模型参数：quality_output（默认 false）、min_phase_amplitude（灰度，默认 0）、
 * saturation_margin [0,127]（0 禁用相位帧裁剪拒绝）、intensity_stddev（独立帧灰度噪声）、
 * projector_coordinate_stddev（像素）、hole_max_width
 * [0,4096]、hole_max_depth_jump（模型单位，默认 .01）。
 * 噪声默认 0 表示未知。启用质量后 get_object 支持
 * confidence_image（Float32）、reason_image（UInt16 标志）
 * 和 coordinate_stddev_image（逐方向 Float32）。重建点云通过 get_object_model_3d_params 提供
 * observed_valid、filled、quality_reason、depth_uncertainty。修改参数使采集/解码状态失效，应在生成
 * pattern 前设置。
 *
 * @param[in] ModelType 结构光模型类型元组；调用期间只读借用。
 * @param[out] StructuredLightModel 结构光模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_structured_light_model(MvTuple ModelType, MvTuple* StructuredLightModel);

/**
 * @brief 按 MVKit .mvslm 格式保存配置及可选完整相机/投影仪配置。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_structured_light_model(MvTuple StructuredLightModel, MvTuple FileName);

/**
 * @brief 恢复 MVKit .mvslm 模型；不存储采集图像或解码中间状态。
 *
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[out] StructuredLightModel 结构光模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_structured_light_model(MvTuple FileName, MvTuple* StructuredLightModel);

/**
 * @brief 设置结构光模型的指定参数。
 *
 * @details camera_setup_model 保存标定系统副本，设备 0 为相机、设备 1 为投影仪；绑定或清除会使旧解码结果失效。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_structured_light_model_param(MvTuple StructuredLightModel,
                                                    MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief 查询结构光模型的指定参数。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[out] GenParamValue 参数或数据项的值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_structured_light_model_param(MvTuple StructuredLightModel,
                                                    MvTuple GenParamName, MvTuple* GenParamValue);

/**
 * @brief 生成结构光模型所需的投影图案。
 *
 * @param[out] PatternImages 结构光投影图案；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_structured_light_pattern(MvObject* PatternImages, MvTuple StructuredLightModel);

/**
 * @brief 根据采集图像解码结构光图案。
 *
 * @param[in] CameraImages 结构光采集图像；调用期间借用，不转移引用所有权。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_decode_structured_light_pattern(MvObject CameraImages, MvTuple StructuredLightModel);

/**
 * @brief 获取结构光模型中的图案、解码或质量图像对象。
 *
 * @param[out] Object 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 * @param[in] ObjectName 对象或轮廓查询名称元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_structured_light_object(MvObject* Object, MvTuple StructuredLightModel,
                                               MvTuple ObjectName);

/**
 * @brief 根据解码结果重建结构光三维表面。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间只读借用。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_reconstruct_surface_structured_light(MvTuple StructuredLightModel,
                                                        MvTuple* ObjectModel3D);

#ifdef __cplusplus
}
#endif
#endif
