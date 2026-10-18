#ifndef MVKIT_C_MV_CALIBRATION_H
#define MVKIT_C_MV_CALIBRATION_H
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_handle.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_image_processing.h>
#include <mvkit/c/mv_tuple.h>
#include <mvkit/c/mv_xld.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建指定相机和标定对象数量的标定数据模型。
 *
 * @param[in] CalibSetup 标定配置类型元组；调用期间只读借用。
 * @param[in] NumCameras 相机数量元组；调用期间只读借用。
 * @param[in] NumCalibObjects 标定对象数量元组；调用期间只读借用。
 * @param[out] CalibDataID 标定数据模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_calib_data(MvTuple CalibSetup, MvTuple NumCameras,
                                     MvTuple NumCalibObjects, MvTuple* CalibDataID);

/**
 * @brief 设置标定数据模型中指定相机的类型和参数。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CameraType 相机模型类型元组；调用期间只读借用。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_calib_data_cam_param(MvTuple CalibDataID, MvTuple CameraIdx,
                                            MvTuple CameraType, MvTuple CameraParam);

/**
 * @brief 设置指定标定对象的描述文件或描述内容。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjDescr 标定对象描述元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_calib_data_calib_object(MvTuple CalibDataID, MvTuple CalibObjIdx,
                                               MvTuple CalibObjDescr);

/**
 * @brief 保存指定相机、标定对象和位姿的观测点及初始位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间只读借用。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Index 数据、元素或对象索引元组；调用期间只读借用。
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_calib_data_observ_points(MvTuple CalibDataID, MvTuple CameraIdx,
                                                MvTuple CalibObjIdx, MvTuple CalibObjPoseIdx,
                                                MvTuple Row, MvTuple Column, MvTuple Index,
                                                MvTuple Pose);

/**
 * @brief 检测图像中的标定对象，并将检测结果写入标定数据模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_find_calib_object(MvObject Image, MvTuple CalibDataID, MvTuple CameraIdx,
                                     MvTuple CalibObjIdx, MvTuple CalibObjPoseIdx,
                                     MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief 读取指定标定观测的图像坐标、点索引和位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Index 数据、元素或对象索引元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Pose 三维位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_calib_data_observ_points(MvTuple CalibDataID, MvTuple CameraIdx,
                                                MvTuple CalibObjIdx, MvTuple CalibObjPoseIdx,
                                                MvTuple* Row, MvTuple* Column, MvTuple* Index, MvTuple* Pose);

/**
 * @brief 读取指定标定观测的轮廓对象。
 *
 * @param[out] Contours 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] ContourName 对象或轮廓查询名称元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_calib_data_observ_contours(MvObject* Contours, MvTuple CalibDataID,
                                                  MvTuple ContourName, MvTuple CameraIdx,
                                                  MvTuple CalibObjIdx, MvTuple CalibObjPoseIdx);

/**
 * @brief 按项目类型、索引和数据名称设置标定模型数据。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] ItemType 数据项目的类别或索引元组；调用期间只读借用。
 * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间只读借用。
 * @param[in] DataName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] DataValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_calib_data(MvTuple CalibDataID, MvTuple ItemType, MvTuple ItemIdx,
                                  MvTuple DataName, MvTuple DataValue);

/**
 * @brief 按项目类型、索引和数据名称查询标定模型数据。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] ItemType 数据项目的类别或索引元组；调用期间只读借用。
 * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间只读借用。
 * @param[in] DataName 参数或数据项名称元组；调用期间只读借用。
 * @param[out] DataValue 参数或数据项的值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_calib_data(MvTuple CalibDataID, MvTuple ItemType, MvTuple ItemIdx,
                                  MvTuple DataName, MvTuple* DataValue);

/**
 * @brief 根据已保存的标定观测优化相机参数及位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[out] Error 错误详情或标定误差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_calibrate_cameras(MvTuple CalibDataID, MvTuple* Error);

/**
 * @brief 根据已保存的标定观测求解手眼关系。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[out] Errors 错误详情或标定误差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_calibrate_hand_eye(MvTuple CalibDataID, MvTuple* Errors);

/**
 * @brief 将三维相机坐标投影到图像平面。
 *
 * @details 相机坐标；三个坐标元组必须等长。
 *
 * @param[in] X X 坐标元组；调用期间只读借用。
 * @param[in] Y Y 坐标元组；调用期间只读借用。
 * @param[in] Z Z 坐标元组；调用期间只读借用。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_project_3d_point(MvTuple X, MvTuple Y, MvTuple Z, MvTuple CameraParam,
                                    MvTuple* Row, MvTuple* Column);

/**
 * @brief 按输入和输出相机参数转换点的径向畸变。
 *
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Col 列坐标元组；调用期间只读借用。
 * @param[in] CamParamIn 输入相机参数元组；调用期间只读借用。
 * @param[in] CamParamOut 输出相机参数元组；调用期间只读借用。
 * @param[out] RowChanged 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ColChanged 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_change_radial_distortion_points(MvTuple Row, MvTuple Col,
                                                   MvTuple CamParamIn, MvTuple CamParamOut,
                                                   MvTuple* RowChanged, MvTuple* ColChanged);

/**
 * @brief 将图像坐标映射到指定世界平面。
 *
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] WorldPose 世界平面位姿元组；调用期间只读借用。
 * @param[in] Rows 行坐标元组；调用期间只读借用。
 * @param[in] Cols 列坐标元组；调用期间只读借用。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间只读借用。
 * @param[out] X X 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Y Y 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_image_points_to_world_plane(MvTuple CameraParam, MvTuple WorldPose,
                                               MvTuple Rows, MvTuple Cols, MvTuple Scale,
                                               MvTuple* X, MvTuple* Y);

/**
 * @brief 生成径向畸变校正使用的编码映射图像。
 *
 * @details HALCON 编码 Map 与区域约束的图像校正。nearest_neighbor 返回一个 int4 索引通道；
 * bilinear 返回一个 int4 索引通道和四个 uint2 权重通道。未实现 vector_field 存储，因此不支持
 * coord_map_sub_pix。
 * 图像输入支持 0/1/N 个对象；径向校正只使用首个 Region，零区域对象时使用完整输出范围。
 *
 * @param[out] Map 映射图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CamParamIn 输入相机参数元组；调用期间只读借用。
 * @param[in] CamParamOut 输出相机参数元组；调用期间只读借用。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_radial_distortion_map(MvObject* Map, MvTuple CamParamIn, MvTuple CamParamOut,
                                             MvTuple MapType);

/**
 * @brief 按相机参数校正指定区域内的图像径向畸变。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] ImageRectified 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CamParamIn 输入相机参数元组；调用期间只读借用。
 * @param[in] CamParamOut 输出相机参数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_change_radial_distortion_image(MvObject Image, MvObject Region, MvObject* ImageRectified,
                                                  MvTuple CamParamIn, MvTuple CamParamOut);

/**
 * @brief 生成图像到世界平面的编码逆采样映射。
 *
 * @param[out] Map 映射图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] WorldPose 世界平面位姿元组；调用期间只读借用。
 * @param[in] WidthIn 输入图像宽度元组；调用期间只读借用。
 * @param[in] HeightIn 输入图像高度元组；调用期间只读借用。
 * @param[in] WidthMapped 映射图像宽度元组；调用期间只读借用。
 * @param[in] HeightMapped 映射图像高度元组；调用期间只读借用。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间只读借用。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_image_to_world_plane_map(MvObject* Map, MvTuple CameraParam,
                                                MvTuple WorldPose, MvTuple WidthIn,
                                                MvTuple HeightIn, MvTuple WidthMapped,
                                                MvTuple HeightMapped, MvTuple Scale,
                                                MvTuple MapType);

/**
 * @brief 将图像重采样到指定世界平面。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageWorld 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] WorldPose 世界平面位姿元组；调用期间只读借用。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_image_to_world_plane(MvObject Image, MvObject* ImageWorld, MvTuple CameraParam,
                                        MvTuple WorldPose, MvTuple Width, MvTuple Height,
                                        MvTuple Scale, MvTuple Interpolation);

/**
 * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
 *
 * @details 其他合法 Mode 仅支持系数不变；改变视场的优化尚未实现。
 *
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 * @param[in] CamParamIn 输入相机参数元组；调用期间只读借用。
 * @param[in] DistortionCoeffs 径向畸变系数元组；调用期间只读借用。
 * @param[out] CamParamOut 输出相机参数元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_change_radial_distortion_cam_par(MvTuple Mode, MvTuple CamParamIn,
                                                    MvTuple DistortionCoeffs, MvTuple* CamParamOut);

/**
 * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ContoursTrans 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] WorldPose 世界平面位姿元组；调用期间只读借用。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_contour_to_world_plane_xld(MvObject Contours, MvObject* ContoursTrans,
                                              MvTuple CameraParam, MvTuple WorldPose,
                                              MvTuple Scale);

/**
 * @brief 按相机参数改变轮廓点径向畸变；点属性不保留。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ContoursRectified 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] CamParamIn 输入相机参数元组；调用期间只读借用。
 * @param[in] CamParamOut 输出相机参数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_change_radial_distortion_contours_xld(MvObject Contours, MvObject* ContoursRectified,
                                                         MvTuple CamParamIn, MvTuple CamParamOut);

/**
 * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
 *
 * @details 使用 HALCON 正式方法和 QualityType="error"，质量为像素 RMS。
 * WorldZ 可为空（平面 Z=0），其他坐标必须等长。三点迭代与远心平面方法存在位姿歧义。
 * 支持四种 area_scan 相机模型，尚未实现 line_scan 相机。
 *
 * @param[in] WorldX X 坐标元组；调用期间只读借用。
 * @param[in] WorldY Y 坐标元组；调用期间只读借用。
 * @param[in] WorldZ Z 坐标元组；调用期间只读借用。
 * @param[in] ImageRow 图像结果元组；调用期间只读借用。
 * @param[in] ImageColumn 图像结果元组；调用期间只读借用。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] QualityType 质量评价方式元组；调用期间只读借用。
 * @param[out] Pose 三维位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Quality 质量评价结果元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_vector_to_pose(MvTuple WorldX, MvTuple WorldY, MvTuple WorldZ,
                                  MvTuple ImageRow, MvTuple ImageColumn, MvTuple CameraParam,
                                  MvTuple Method, MvTuple QualityType, MvTuple* Pose,
                                  MvTuple* Quality);

/**
 * @brief 根据对应二维点估计仿射变换矩阵。
 *
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[in] Qx X 坐标元组；调用期间只读借用。
 * @param[in] Qy Y 坐标元组；调用期间只读借用。
 * @param[out] HomMat2D 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_vector_to_hom_mat2d(MvTuple Px, MvTuple Py, MvTuple Qx, MvTuple Qy,
                                       MvTuple* HomMat2D);

/**
 * @brief 根据对应二维点及测量协方差估计投影变换矩阵。
 *
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[in] Qx X 坐标元组；调用期间只读借用。
 * @param[in] Qy Y 坐标元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] CovXX1 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[in] CovYY1 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[in] CovXY1 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[in] CovXX2 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[in] CovYY2 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[in] CovXY2 对应点的测量协方差分量元组；调用期间只读借用。
 * @param[out] HomMat2D 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Covariance 估计参数的协方差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_vector_to_proj_hom_mat2d(MvTuple Px, MvTuple Py, MvTuple Qx,
                                            MvTuple Qy, MvTuple Method, MvTuple CovXX1,
                                            MvTuple CovYY1, MvTuple CovXY1, MvTuple CovXX2,
                                            MvTuple CovYY2, MvTuple CovXY2, MvTuple* HomMat2D,
                                            MvTuple* Covariance);

/**
 * @brief 写入 HALCON 24.11 HCPR v3；当前支持四种 area_scan division/polynomial 模型。
 *
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] CamParFile 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_cam_par(MvTuple CameraParam, MvTuple CamParFile);

/**
 * @brief 读取上述 HCPR v3 及历史 MVKIT_CAM_PAR 文本；其他相机模型/版本显式拒绝。
 *
 * @param[in] CamParFile 文件路径元组；调用期间只读借用。
 * @param[out] CameraParam 相机内参与畸变参数元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_cam_par(MvTuple CamParFile, MvTuple* CameraParam);

/**
 * @brief 写入 HALCON f/r/t 位姿文本；保留输入表示类型。
 *
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 * @param[in] PoseFile 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_pose(MvTuple Pose, MvTuple PoseFile);

/**
 * @brief 读取 HALCON f/r/t 位姿文本及历史 MVKIT_POSE 文本。
 *
 * @param[in] PoseFile 文件路径元组；调用期间只读借用。
 * @param[out] Pose 三维位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_pose(MvTuple PoseFile, MvTuple* Pose);

/**
 * @brief 删除单个观测；一个模型及三个整数索引，失败保持模型。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间只读借用。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_remove_calib_data_observ(MvTuple CalibDataID, MvTuple CameraIdx,
                                            MvTuple CalibObjIdx, MvTuple CalibObjPoseIdx);

/**
 * @brief 写入 `MVKIT_CALIB_DATA` 状态；不含求解器缓存/指针。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_calib_data(MvTuple CalibDataID, MvTuple FileName);

/**
 * @brief 读取 `MVKIT_CALIB_DATA` 并返回 CalibDataID 句柄控制元组。
 *
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[out] CalibDataID 标定数据模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_calib_data(MvTuple FileName, MvTuple* CalibDataID);

/** 唯一正式 create_calib_data 入口返回拥有型 句柄控制元组。 */

#ifdef __cplusplus
}
#endif
#endif
