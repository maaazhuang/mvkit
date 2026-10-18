#ifndef MVKIT_C_MV_STEREO_H
#define MVKIT_C_MV_STEREO_H

#include <stdint.h>
#include <mvkit/c/mv_reconstruction3d.h>
#include <mvkit/c/mv_calibration.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
 *
 * @param[out] Map1 映射图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Map2 映射图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CamParam1 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] CamParam2 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] RelPose 设备间相对位姿元组；调用期间只读借用。
 * @param[in] SubSampling 亚采样控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间只读借用。
 * @param[out] CamParamRect1 校正后的相机参数元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] CamParamRect2 校正后的相机参数元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] CamPoseRect1 校正后的相机位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] CamPoseRect2 校正后的相机位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] RelPoseRect 设备间相对位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_binocular_rectification_map(
    MvObject* Map1, MvObject* Map2, MvTuple CamParam1, MvTuple CamParam2,
    MvTuple RelPose, MvTuple SubSampling, MvTuple Method, MvTuple MapType,
    MvTuple* CamParamRect1, MvTuple* CamParamRect2, MvTuple* CamPoseRect1,
    MvTuple* CamPoseRect2, MvTuple* RelPoseRect);

/**
 * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
 *
 * @param[in] CamParamRect1 校正后的相机参数元组；调用期间只读借用。
 * @param[in] CamParamRect2 校正后的相机参数元组；调用期间只读借用。
 * @param[in] RelPoseRect 设备间相对位姿元组；调用期间只读借用。
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Col1 列坐标元组；调用期间只读借用。
 * @param[in] Disparity 视差结果元组；调用期间只读借用。
 * @param[out] X X 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Y Y 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Z Z 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_disparity_to_point_3d(MvTuple CamParamRect1, MvTuple CamParamRect2,
                                          MvTuple RelPoseRect, MvTuple Row1, MvTuple Col1,
                                          MvTuple Disparity, MvTuple* X, MvTuple* Y, MvTuple* Z);

/**
 * @brief real 单通道图像，保持对象数量及定义域，三个输出整体提交。
 *
 * @param[in] Disparity 视差结果；调用期间借用，不转移引用所有权。
 * @param[out] X X 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Y Y 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Z Z 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CamParamRect1 校正后的相机参数元组；调用期间只读借用。
 * @param[in] CamParamRect2 校正后的相机参数元组；调用期间只读借用。
 * @param[in] RelPoseRect 设备间相对位姿元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_disparity_image_to_xyz(MvObject Disparity, MvObject* X, MvObject* Y, MvObject* Z,
                                          MvTuple CamParamRect1, MvTuple CamParamRect2,
                                          MvTuple RelPoseRect);

/**
 * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
 *
 * @param[in] ImageRect1 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] ImageRect2 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] Disparity 视差结果；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Score 匹配或视差得分；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] TextureThresh 纹理阈值元组；调用期间只读借用。
 * @param[in] MinDisparity 视差搜索范围元组；调用期间只读借用。
 * @param[in] MaxDisparity 视差搜索范围元组；调用期间只读借用。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间只读借用。
 * @param[in] ScoreThresh 得分阈值元组；调用期间只读借用。
 * @param[in] Filter 边缘滤波器类型元组；调用期间只读借用。
 * @param[in] SubDisparity 亚像素细化方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_binocular_disparity(
    MvObject ImageRect1, MvObject ImageRect2, MvObject* Disparity, MvObject* Score,
    MvTuple Method, MvTuple MaskWidth, MvTuple MaskHeight,
    MvTuple TextureThresh, MvTuple MinDisparity, MvTuple MaxDisparity,
    MvTuple NumLevels, MvTuple ScoreThresh, MvTuple Filter, MvTuple SubDisparity);

/**
 * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
 *
 * @details 平滑惩罚接受 [0,INT32_MAX] 整数，输出同时提交。
 *
 * @param[in] ImageRect1 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] ImageRect2 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] Disparity 视差结果；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Score 匹配或视差得分；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MinDisparity 视差搜索范围元组；调用期间只读借用。
 * @param[in] MaxDisparity 视差搜索范围元组；调用期间只读借用。
 * @param[in] SurfaceSmoothing 表面或边缘平滑控制元组；调用期间只读借用。
 * @param[in] EdgeSmoothing 表面或边缘平滑控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_binocular_disparity_ms(
    MvObject ImageRect1, MvObject ImageRect2, MvObject* Disparity, MvObject* Score,
    MvTuple MinDisparity, MvTuple MaxDisparity, MvTuple SurfaceSmoothing,
    MvTuple EdgeSmoothing, MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] StereoModelID 稀疏重建模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_stereo_model(MvTuple CameraSetupModelID, MvTuple Method,
                                       MvTuple GenParamName, MvTuple GenParamValue,
                                       MvTuple* StereoModelID);

/**
 * @brief 设置稀疏多相机重建模型的指定参数。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_stereo_model_param(MvTuple StereoModelID, MvTuple GenParamName,
                                          MvTuple GenParamValue);

/**
 * @brief 清理稀疏多相机重建模型及其状态。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_clear_stereo_model(MvTuple StereoModelID);

/**
 * @brief 观测元组等长；CovIP 为空或每个观测四个值。
 *
 * @details 不足两次观测的点省略，PointIdxOut 有序；五个拥有型输出元组同时提交。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间只读借用。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] CovIP 图像点或世界点的测量协方差元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] PointIdx 输入点索引元组；调用期间只读借用。
 * @param[out] X X 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Y Y 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Z Z 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] CovWP 图像点或世界点的测量协方差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] PointIdxOut 输出点索引元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_reconstruct_points_stereo(MvTuple StereoModelID, MvTuple Row,
                                             MvTuple Column, MvTuple CovIP, MvTuple CameraIdx,
                                             MvTuple PointIdx, MvTuple* X, MvTuple* Y, MvTuple* Z,
                                             MvTuple* CovWP, MvTuple* PointIdxOut);

#ifdef __cplusplus
}
#endif
#endif
