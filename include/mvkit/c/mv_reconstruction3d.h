#ifndef MVKIT_C_MV_RECONSTRUCTION3D_H
#define MVKIT_C_MV_RECONSTRUCTION3D_H

/** @file
 * @brief 三维对象模型与相机系统公共 C 接口。
 */

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_geometry.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_tuple.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
 *
 * @details 笛卡尔点/面渲染暂不支持。
 *
 * @param[out] X X 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Y Y 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Z Z 坐标；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Type XYZ 图像的生成方式元组；调用期间只读借用。
 * @param[in] CamParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_object_model_3d_to_xyz(MvObject* X, MvObject* Y, MvObject* Z, MvTuple ObjectModel3D,
                                          MvTuple Type, MvTuple CamParam, MvTuple Pose);

/**
 * @brief 按参数名称读取三维对象模型的属性。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[out] GenParamValue 参数或数据项的值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_object_model_3d_params(MvTuple ObjectModel3D, MvTuple GenParamName,
                                              MvTuple* GenParamValue);

/**
 * @brief 由等长 X/Y/Z 坐标元组 生成无组织点云；按 HALCON 保留非有限坐标。
 *
 * @param[in] X X 坐标元组；调用期间只读借用。
 * @param[in] Y Y 坐标元组；调用期间只读借用。
 * @param[in] Z Z 坐标元组；调用期间只读借用。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_object_model_3d_from_points(MvTuple X, MvTuple Y, MvTuple Z,
                                                   MvTuple* ObjectModel3D);

/**
 * @brief 使用 X、Y、Z 坐标图像构造一个三维对象模型。
 *
 * @param[in] x X 坐标；调用期间借用，不转移引用所有权。
 * @param[in] y Y 坐标；调用期间借用，不转移引用所有权。
 * @param[in] z Z 坐标；调用期间借用，不转移引用所有权。
 * @param[out] model 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_xyz_to_object_model_3d(MvObject x, MvObject y, MvObject z, MvTuple* model);

/**
 * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
 *
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Status 逐项操作状态元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_read_object_model_3d(MvTuple FileName, MvTuple Scale, MvTuple GenParamName,
                                        MvTuple GenParamValue, MvTuple* ObjectModel3D, MvTuple* Status);

/**
 * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
 *
 * @details 和平面状态，不保存索引/scratch。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] FileType 图像或模型的文件格式元组；调用期间只读借用。
 * @param[in] FileName 文件路径元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_write_object_model_3d(MvTuple ObjectModel3D, MvTuple FileType,
                                         MvTuple FileName, MvTuple GenParamName,
                                         MvTuple GenParamValue);

/**
 * @brief 用位姿刚体变换点云，输出新不可变模型；保留 有组织点云的映射与置信度。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 * @param[out] ObjectModel3DRigidTrans 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_rigid_trans_object_model_3d(MvTuple ObjectModel3D, MvTuple Pose,
                                               MvTuple* ObjectModel3DRigidTrans);

/**
 * @brief 用 12 元 3x4 仿射变换点云；无逆变换属性的点云接受奇异矩阵。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间只读借用。
 * @param[out] ObjectModel3DAffineTrans 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_object_model_3d(MvTuple ObjectModel3D, MvTuple HomMat3D,
                                                MvTuple* ObjectModel3DAffineTrans);

/**
 * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Attrib 属性或特征名称元组；调用期间只读借用。
 * @param[in] MinValue 属性筛选范围界限元组；调用期间只读借用。
 * @param[in] MaxValue 属性筛选范围界限元组；调用期间只读借用。
 * @param[out] ObjectModel3DThresholded 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_points_object_model_3d(MvTuple ObjectModel3D, MvTuple Attrib,
                                                 MvTuple MinValue, MvTuple MaxValue,
                                                 MvTuple* ObjectModel3DThresholded);

/**
 * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] SamplingParam 采样间距或采样控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] SampledObjectModel3D 采样后的三维对象模型元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_sample_object_model_3d(MvTuple ObjectModel3D, MvTuple Method,
                                          MvTuple SamplingParam, MvTuple GenParamName,
                                          MvTuple GenParamValue, MvTuple* SampledObjectModel3D);

/**
 * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] ObjectModel3DNormals 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_surface_normals_object_model_3d(MvTuple ObjectModel3D, MvTuple Method,
                                                   MvTuple GenParamName, MvTuple GenParamValue,
                                                   MvTuple* ObjectModel3DNormals);

/**
 * @brief HALCON C2：每模型一个拟合输出；不可观测的拟合不产生原语。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] ObjectModel3DOut 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fit_primitives_object_model_3d(MvTuple ObjectModel3D, MvTuple GenParamName,
                                                  MvTuple GenParamValue, MvTuple* ObjectModel3DOut);

/**
 * @brief HALCON C2：距离及可选最近索引存储到源模型。
 *
 * @param[in] ObjectModel3DFrom 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] ObjectModel3DTo 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 * @param[in] MaxDistance 距离阈值元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_object_model_3d(MvTuple ObjectModel3DFrom, MvTuple ObjectModel3DTo,
                                            MvTuple Pose, MvTuple MaxDistance,
                                            MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
 *
 * @param[in] ObjectModel3D1 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] ObjectModel3D2 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] Pose 三维位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Score 匹配或视差得分元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_register_object_model_3d_pair(MvTuple ObjectModel3D1, MvTuple ObjectModel3D2,
                                                 MvTuple Method, MvTuple GenParamName,
                                                 MvTuple GenParamValue, MvTuple* Pose, MvTuple* Score);

/**
 * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
 *
 * @details greedy/implicit 和一般 polygon 暂不支持；Information 按输出模型返回三角形数量。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 * @param[out] TriangulatedObjectModel3D 三角化后的三维对象模型元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Information 求解或拟合的信息量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_triangulate_object_model_3d(MvTuple ObjectModel3D, MvTuple Method,
                                               MvTuple GenParamName, MvTuple GenParamValue,
                                               MvTuple* TriangulatedObjectModel3D, MvTuple* Information);

/**
 * @brief HALCON C2：固定正相机数量，每台相机具有独立世界位姿。
 *
 * @param[in] NumCameras 相机数量元组；调用期间只读借用。
 * @param[out] CameraSetupModelID 相机配置模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_create_camera_setup_model(MvTuple NumCameras, MvTuple* CameraSetupModelID);

/**
 * @brief 设置相机配置模型中指定相机的参数及位姿。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] CameraType 相机模型类型元组；调用期间只读借用。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间只读借用。
 * @param[in] CameraPose 相机位姿元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_camera_setup_cam_param(MvTuple CameraSetupModelID, MvTuple CameraIdx,
                                              MvTuple CameraType, MvTuple CameraParam,
                                              MvTuple CameraPose);

/**
 * @brief 查询相机配置模型的指定参数。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间只读借用。
 * @param[in] CameraIdx 相机索引元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[out] GenParamValue 参数或数据项的值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_camera_setup_param(MvTuple CameraSetupModelID, MvTuple CameraIdx,
                                          MvTuple GenParamName, MvTuple* GenParamValue);

/**
 * @brief 按指定特征及阈值分解三维对象模型的连通分量。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Feature 属性或特征名称元组；调用期间只读借用。
 * @param[in] Value 保存、追加或读取的数据元组；调用期间只读借用。
 * @param[out] ObjectModel3DConnected 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_connection_object_model_3d(MvTuple ObjectModel3D, MvTuple Feature,
                                              MvTuple Value, MvTuple* ObjectModel3DConnected);

/**
 * @brief 计算三维对象模型与指定平面的交线。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间只读借用。
 * @param[in] Plane 截交平面元组；调用期间只读借用。
 * @param[out] ObjectModel3DIntersection 三维对象模型句柄元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_intersect_plane_object_model_3d(MvTuple ObjectModel3D, MvTuple Plane,
                                                   MvTuple* ObjectModel3DIntersection);

#ifdef __cplusplus
}
#endif

#endif
