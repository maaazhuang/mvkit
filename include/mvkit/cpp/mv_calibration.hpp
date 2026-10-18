#pragma once

#include <mvkit/c/mv_calibration.h>

#include <mvkit/cpp/mv_api.hpp>
#include <mvkit/cpp/mv_iconic.hpp>

namespace mv {
/**
 * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
 *
 * @param[out] Map1 映射图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Map2 映射图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CamParam1 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParam2 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] RelPose 设备间相对位姿元组；调用期间借用，不转移所有权。
 * @param[in] SubSampling 亚采样控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间借用，不转移所有权。
 * @param[out] CamParamRect1 校正后的相机参数元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] CamParamRect2 校正后的相机参数元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] CamPoseRect1 校正后的相机位姿元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] CamPoseRect2 校正后的相机位姿元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] RelPoseRect 设备间相对位姿元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_binocular_rectification_map
 */
MVKIT_CPP_API void GenBinocularRectificationMap(MvObject* Map1, MvObject* Map2,
    const MvTuple& CamParam1, const MvTuple& CamParam2, const MvTuple& RelPose,
    const MvTuple& SubSampling, const MvTuple& Method, const MvTuple& MapType,
    MvTuple* CamParamRect1, MvTuple* CamParamRect2, MvTuple* CamPoseRect1,
    MvTuple* CamPoseRect2, MvTuple* RelPoseRect);

/**
 * @brief real 单通道图像，保持对象数量及定义域，三个输出整体提交。
 *
 * @param[in] Disparity 视差结果；调用期间借用，不转移所有权。
 * @param[out] X X 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CamParamRect1 校正后的相机参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamRect2 校正后的相机参数元组；调用期间借用，不转移所有权。
 * @param[in] RelPoseRect 设备间相对位姿元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_disparity_image_to_xyz
 */
MVKIT_CPP_API void DisparityImageToXyz(const MvObject& Disparity, MvObject* X, MvObject* Y, MvObject* Z,
                                      const MvTuple& CamParamRect1, const MvTuple& CamParamRect2,
                                      const MvTuple& RelPoseRect);

/**
 * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
 *
 * @param[in] CamParamRect1 校正后的相机参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamRect2 校正后的相机参数元组；调用期间借用，不转移所有权。
 * @param[in] RelPoseRect 设备间相对位姿元组；调用期间借用，不转移所有权。
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Col1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Disparity 视差结果元组；调用期间借用，不转移所有权。
 * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Z Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_disparity_to_point_3d
 */
MVKIT_CPP_API void DisparityToPoint3d(const MvTuple& CamParamRect1, const MvTuple& CamParamRect2,
                                     const MvTuple& RelPoseRect, const MvTuple& Row1, const MvTuple& Col1,
                                     const MvTuple& Disparity, MvTuple* X, MvTuple* Y, MvTuple* Z);

/**
 * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
 *
 * @param[in] WorldX X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] WorldY Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] WorldZ Z 坐标元组；调用期间借用，不转移所有权。
 * @param[in] ImageRow 图像中的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] ImageColumn 图像中的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] QualityType 质量评价方式元组；调用期间借用，不转移所有权。
 * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Quality 质量评价结果元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_vector_to_pose
 */
MVKIT_CPP_API void VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                                const MvTuple& ImageRow, const MvTuple& ImageColumn,
                                const MvTuple& CameraParam, const MvTuple& Method, const MvTuple& QualityType,
                                MvTuple* Pose, MvTuple* Quality);

/**
 * @brief 根据对应二维点及测量协方差估计投影变换矩阵。
 *
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Qx X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Qy Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] CovXX1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[in] CovYY1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[in] CovXY1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[in] CovXX2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[in] CovYY2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[in] CovXY2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2D 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Covariance 估计参数的协方差元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_vector_to_proj_hom_mat2d
 */
MVKIT_CPP_API void VectorToProjHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx,
                                        const MvTuple& Qy, const MvTuple& Method, const MvTuple& CovXX1,
                                        const MvTuple& CovYY1, const MvTuple& CovXY1, const MvTuple& CovXX2,
                                        const MvTuple& CovYY2, const MvTuple& CovXY2, MvTuple* HomMat2D,
                                        MvTuple* Covariance);

/**
 * @brief 生成径向畸变校正使用的编码映射图像。
 *
 * @param[out] Map 映射图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CamParamIn 源相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamOut 目标相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_radial_distortion_map
 */
MVKIT_CPP_API void GenRadialDistortionMap(MvObject* Map, const MvTuple& CamParamIn,
                                          const MvTuple& CamParamOut, const MvTuple& MapType);

/**
 * @brief 按相机参数校正指定区域内的图像径向畸变。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] ImageRectified 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CamParamIn 源相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamOut 目标相机内参与畸变参数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_change_radial_distortion_image
 */
MVKIT_CPP_API void ChangeRadialDistortionImage(const MvObject& Image, const MvObject& Region,
                                               MvObject* ImageRectified, const MvTuple& CamParamIn,
                                               const MvTuple& CamParamOut);

/**
 * @brief 生成图像到世界平面的编码逆采样映射。
 *
 * @param[out] Map 映射图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] WorldPose 世界平面位姿元组；调用期间借用，不转移所有权。
 * @param[in] WidthIn 输入图像宽度元组；调用期间借用，不转移所有权。
 * @param[in] HeightIn 输入图像高度元组；调用期间借用，不转移所有权。
 * @param[in] WidthMapped 映射图像宽度元组；调用期间借用，不转移所有权。
 * @param[in] HeightMapped 映射图像高度元组；调用期间借用，不转移所有权。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
 * @param[in] MapType 编码映射图像的存储类型元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image_to_world_plane_map
 */
MVKIT_CPP_API void GenImageToWorldPlaneMap(MvObject* Map, const MvTuple& CameraParam,
                                           const MvTuple& WorldPose, const MvTuple& WidthIn,
                                           const MvTuple& HeightIn, const MvTuple& WidthMapped,
                                           const MvTuple& HeightMapped, const MvTuple& Scale,
                                           const MvTuple& MapType);

/**
 * @brief 将图像重采样到指定世界平面。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageWorld 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] WorldPose 世界平面位姿元组；调用期间借用，不转移所有权。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_image_to_world_plane
 */
MVKIT_CPP_API void ImageToWorldPlane(const MvObject& Image, MvObject* ImageWorld, const MvTuple& CameraParam,
                                     const MvTuple& WorldPose, const MvTuple& Width, const MvTuple& Height,
                                     const MvTuple& Scale, const MvTuple& Interpolation);

class MvCameraSetupModel;
class MvObjectModel3D;
class MvObjectModel3DArray;
class MvCamPar;

/**
 * @brief 自动管理共享引用的可变标定数据对象。
 *
 * @details 标定数据使用共享控制身份，与图标对象数组独立。
 */
class MVKIT_CPP_API MvCalibData : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvCalibData() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvCalibData(const MvHandle& handle);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCalibData(const MvCalibData&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvCalibData(MvCalibData&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCalibData& operator=(const MvCalibData&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvCalibData& operator=(MvCalibData&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvCalibData() noexcept;

  /**
   * @brief 根据已保存的标定观测优化相机参数及位姿。
   *
   * @return 错误详情或标定误差。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_calibrate_cameras
   */
  double CalibrateCameras() const;

  /**
   * @brief 根据已保存的标定观测求解手眼关系。
   *
   * @return 错误详情或标定误差元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_calibrate_hand_eye
   */
  MvTuple CalibrateHandEye() const;

  /**
   * @brief 删除单个观测；一个模型及三个整数索引，失败保持模型。
   *
   * @param[in] CameraIdx 相机索引。
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjPoseIdx 标定观测位姿索引。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_remove_calib_data_observ
   */
  void RemoveCalibDataObserv(std::int64_t CameraIdx, std::int64_t CalibObjIdx,
                             std::int64_t CalibObjPoseIdx) const;

  /**
   * @brief 读取 `MVKIT_CALIB_DATA` 并返回 CalibDataID 句柄控制元组。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_calib_data
   */
  void ReadCalibData(const MvString& FileName);

  /**
   * @brief 读取 `MVKIT_CALIB_DATA` 并返回 CalibDataID 句柄控制元组。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadCalibData(const char* FileName);

  /**
   * @brief 读取 `MVKIT_CALIB_DATA` 并返回 CalibDataID 句柄控制元组。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ReadCalibData(const wchar_t* FileName);

  /**
   * @brief 写入 `MVKIT_CALIB_DATA` 状态；不含求解器缓存/指针。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_calib_data
   */
  void WriteCalibData(const MvString& FileName) const;

  /**
   * @brief 写入 `MVKIT_CALIB_DATA` 状态；不含求解器缓存/指针。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteCalibData(const char* FileName) const;

  /**
   * @brief 写入 `MVKIT_CALIB_DATA` 状态；不含求解器缓存/指针。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WriteCalibData(const wchar_t* FileName) const;

  /**
   * @brief 创建指定相机和标定对象数量的标定数据模型。
   *
   * @param[in] CalibSetup 标定配置类型；调用期间借用，不转移所有权。
   * @param[in] NumCameras 相机数量。
   * @param[in] NumCalibObjects 标定对象数量。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_calib_data
   */
  void CreateCalibData(const MvString& CalibSetup, std::int64_t NumCameras, std::int64_t NumCalibObjects);

  /**
   * @brief 创建指定相机和标定对象数量的标定数据模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateCalibData(const char* CalibSetup, std::int64_t NumCameras, std::int64_t NumCalibObjects);

  /**
   * @brief 创建指定相机和标定对象数量的标定数据模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateCalibData(const wchar_t* CalibSetup, std::int64_t NumCameras, std::int64_t NumCalibObjects);

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   *
   * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
   * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_calib_data_cam_param
   */
  void SetCalibDataCamParam(const MvTuple& CameraIdx, const MvTuple& CameraType,
                            const MvCamPar& CameraParam) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvTuple& CameraIdx, const MvString& CameraType,
                            const MvCamPar& CameraParam) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvTuple& CameraIdx, const char* CameraType,
                            const MvCamPar& CameraParam) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvTuple& CameraIdx, const wchar_t* CameraType,
                            const MvCamPar& CameraParam) const;

  /**
   * @brief 设置指定标定对象的描述文件或描述内容。
   *
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjDescr 标定对象描述元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_calib_data_calib_object
   */
  void SetCalibDataCalibObject(std::int64_t CalibObjIdx, const MvTuple& CalibObjDescr) const;

  /**
   * @brief 设置指定标定对象的描述文件或描述内容。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCalibObject(std::int64_t CalibObjIdx, double CalibObjDescr) const;

  /**
   * @brief 按项目类型、索引和数据名称设置标定模型数据。
   *
   * @param[in] ItemType 数据项目的类别或索引；调用期间借用，不转移所有权。
   * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间借用，不转移所有权。
   * @param[in] DataName 参数或数据项名称；调用期间借用，不转移所有权。
   * @param[in] DataValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_calib_data
   */
  void SetCalibData(const MvString& ItemType, const MvTuple& ItemIdx, const MvString& DataName,
                    const MvTuple& DataValue) const;

  /**
   * @brief 按项目类型、索引和数据名称设置标定模型数据。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void SetCalibData(const MvString& ItemType, std::int64_t ItemIdx, const MvString& DataName,
                    const MvString& DataValue) const;

  /**
   * @brief 按项目类型、索引和数据名称设置标定模型数据。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void SetCalibData(const char* ItemType, std::int64_t ItemIdx, const char* DataName,
                    const char* DataValue) const;

  /**
   * @brief 按项目类型、索引和数据名称设置标定模型数据。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void SetCalibData(const wchar_t* ItemType, std::int64_t ItemIdx, const wchar_t* DataName,
                    const wchar_t* DataValue) const;

  /**
   * @brief 按项目类型、索引和数据名称查询标定模型数据。
   *
   * @param[in] ItemType 数据项目的类别或索引；调用期间借用，不转移所有权。
   * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间借用，不转移所有权。
   * @param[in] DataName 参数或数据项名称元组；调用期间借用，不转移所有权。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_calib_data
   */
  MvTuple GetCalibData(const MvString& ItemType, const MvTuple& ItemIdx, const MvTuple& DataName) const;

  /**
   * @brief 按项目类型、索引和数据名称查询标定模型数据。
   * @overload 采用标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCalibData(const MvString& ItemType, std::int64_t ItemIdx, const MvString& DataName) const;

  /**
   * @brief 按项目类型、索引和数据名称查询标定模型数据。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCalibData(const char* ItemType, std::int64_t ItemIdx, const char* DataName) const;

  /**
   * @brief 按项目类型、索引和数据名称查询标定模型数据。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCalibData(const wchar_t* ItemType, std::int64_t ItemIdx, const wchar_t* DataName) const;

  /**
   * @brief 保存指定相机、标定对象和位姿的观测点及初始位姿。
   *
   * @param[in] CameraIdx 相机索引。
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjPoseIdx 标定观测位姿索引。
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Index 数据、元素或对象索引元组；调用期间借用，不转移所有权。
   * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_calib_data_observ_points
   */
  void SetCalibDataObservPoints(std::int64_t CameraIdx, std::int64_t CalibObjIdx,
                                std::int64_t CalibObjPoseIdx, const MvTuple& Row, const MvTuple& Column,
                                const MvTuple& Index, const MvTuple& Pose) const;

  /**
   * @brief 读取指定标定观测的图像坐标、点索引和位姿。
   *
   * @param[in] CameraIdx 相机索引。
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjPoseIdx 标定观测位姿索引。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Index 数据、元素或对象索引元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_calib_data_observ_points
   */
  void GetCalibDataObservPoints(std::int64_t CameraIdx, std::int64_t CalibObjIdx,
                                std::int64_t CalibObjPoseIdx, MvTuple* Row, MvTuple* Column, MvTuple* Index,
                                MvTuple* Pose) const;

  /**
   * @brief 检测图像中的标定对象，并将检测结果写入标定数据模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] CameraIdx 相机索引。
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjPoseIdx 标定观测位姿索引。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_calib_object
   */
  void FindCalibObject(const MvImage& Image, std::int64_t CameraIdx, std::int64_t CalibObjIdx,
                       std::int64_t CalibObjPoseIdx, const MvTuple& GenParamName,
                       const MvTuple& GenParamValue) const;

  /**
   * @brief 读取指定标定观测的轮廓对象。
   *
   * @param[in] ContourName 对象或轮廓查询名称；调用期间借用，不转移所有权。
   * @param[in] CameraIdx 相机索引。
   * @param[in] CalibObjIdx 标定对象索引。
   * @param[in] CalibObjPoseIdx 标定观测位姿索引。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_calib_data_observ_contours
   */
  MvXLDCont GetCalibDataObservContours(const MvString& ContourName, std::int64_t CameraIdx,
                                       std::int64_t CalibObjIdx, std::int64_t CalibObjPoseIdx) const;

  /**
   * @brief 读取指定标定观测的轮廓对象。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont GetCalibDataObservContours(const char* ContourName, std::int64_t CameraIdx,
                                       std::int64_t CalibObjIdx, std::int64_t CalibObjPoseIdx) const;

  /**
   * @brief 读取指定标定观测的轮廓对象。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont GetCalibDataObservContours(const wchar_t* ContourName, std::int64_t CameraIdx,
                                       std::int64_t CalibObjIdx, std::int64_t CalibObjPoseIdx) const;

};

/**
 * @brief 保存相机内参的控制元组。
 *
 * @details 相机模型及参数由调用的算子验证。
 */
class MVKIT_CPP_API MvCamPar : public MvTuple {
 public:
  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   *
   * @param[out] Map2 映射图像；接收结果的 C++ 对象，自动管理资源。
   * @param[in] CamParam2 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] RelPose 设备间相对位姿；调用期间借用，不转移所有权。
   * @param[in] SubSampling 亚采样值。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] MapType 编码映射图像的存储类型；调用期间借用，不转移所有权。
   * @param[out] CamParamRect1 校正后的相机参数；接收结果的 C++ 对象，自动管理资源。
   * @param[out] CamParamRect2 校正后的相机参数；接收结果的 C++ 对象，自动管理资源。
   * @param[out] CamPoseRect1 校正后的相机位姿；接收结果的 C++ 对象，自动管理资源。
   * @param[out] CamPoseRect2 校正后的相机位姿；接收结果的 C++ 对象，自动管理资源。
   * @param[out] RelPoseRect 设备间相对位姿；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 映射图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_binocular_rectification_map
   */
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam2, const MvPose& RelPose, double SubSampling,
      const MvString& Method, const MvString& MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam2, const MvPose& RelPose, double SubSampling,
      const char* Method, const char* MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam2, const MvPose& RelPose, double SubSampling,
      const wchar_t* Method, const wchar_t* MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief real 单通道图像，保持对象数量及定义域，三个输出整体提交。
   *
   * @param[in] Disparity 视差结果；调用期间借用，不转移所有权。
   * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[in] CamParamRect2 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] RelPoseRect 设备间相对位姿；调用期间借用，不转移所有权。
   *
   * @return X 坐标，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_disparity_image_to_xyz
   */
  MvImage DisparityImageToXyz(const MvImage& Disparity, MvImage* Y, MvImage* Z,
                             const MvCamPar& CamParamRect2, const MvPose& RelPoseRect) const;

  /**
   * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
   *
   * @param[in] CamParamRect2 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] RelPoseRect 设备间相对位姿；调用期间借用，不转移所有权。
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Col1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Disparity 视差结果元组；调用期间借用，不转移所有权。
   * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_disparity_to_point_3d
   */
  void DisparityToPoint3d(const MvCamPar& CamParamRect2, const MvPose& RelPoseRect,
                          const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Disparity,
                          MvTuple* X, MvTuple* Y, MvTuple* Z) const;

  /**
   * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void DisparityToPoint3d(const MvCamPar& CamParamRect2, const MvPose& RelPoseRect,
                          double Row1, double Col1, double Disparity,
                          double* X, double* Y, double* Z) const;

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   *
   * @param[in] WorldX X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] WorldY Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] WorldZ Z 坐标元组；调用期间借用，不转移所有权。
   * @param[in] ImageRow 图像中的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ImageColumn 图像中的列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] QualityType 质量评价方式元组；调用期间借用，不转移所有权。
   * @param[out] Quality 质量评价结果元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_vector_to_pose
   */
  MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvString& Method,
                      const MvTuple& QualityType, MvTuple* Quality) const;

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用字符串对象及标量输出参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvString& Method,
                      const MvString& QualityType, double* Quality) const;

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const char* Method,
                      const char* QualityType, double* Quality) const;

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const wchar_t* Method,
                      const wchar_t* QualityType, double* Quality) const;

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   *
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] MapType 编码映射图像的存储类型；调用期间借用，不转移所有权。
   *
   * @return 映射图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_radial_distortion_map
   */
  MvImage GenRadialDistortionMap(const MvCamPar& CamParamOut, const MvString& MapType) const;

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenRadialDistortionMap(const MvCamPar& CamParamOut, const char* MapType) const;

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   * @overload 采用宽字符字符串参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenRadialDistortionMap(const MvCamPar& CamParamOut, const wchar_t* MapType) const;

  /**
   * @brief 按相机参数校正指定区域内的图像径向畸变。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] Region 输入区域；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_image
   */
  MvImage ChangeRadialDistortionImage(const MvImage& Image, const MvRegion& Region,
                                      const MvCamPar& CamParamOut) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   *
   * @param[in] WorldPose 世界平面位姿；调用期间借用，不转移所有权。
   * @param[in] WidthIn 输入图像宽度。
   * @param[in] HeightIn 输入图像高度。
   * @param[in] WidthMapped 映射图像宽度。
   * @param[in] HeightMapped 映射图像高度。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   * @param[in] MapType 编码映射图像的存储类型；调用期间借用，不转移所有权。
   *
   * @return 映射图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_image_to_world_plane_map
   */
  MvImage GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const MvTuple& Scale,
                                  const MvString& MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const MvString& Scale,
                                  const MvString& MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const char* Scale,
                                  const char* MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const wchar_t* Scale,
                                  const wchar_t* MapType) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] WorldPose 世界平面位姿；调用期间借用，不转移所有权。
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_image_to_world_plane
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const MvTuple& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const MvString& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const char* Scale, const char* Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const wchar_t* Scale, const wchar_t* Interpolation) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   *
   * @param[in] CameraSetupModelID 包含相机配置模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
   * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
   * @param[in] CameraPose 相机位姿元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_camera_setup_cam_param
   */
  void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                              const MvTuple& CameraType, const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                              const MvString& CameraType, const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                              const char* CameraType, const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                              const wchar_t* CameraType, const MvTuple& CameraPose) const;

  /**
   * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
   *
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   * @param[in] DistortionCoeffs 径向畸变系数元组；调用期间借用，不转移所有权。
   *
   * @return 输出相机参数，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_cam_par
   */
  MvCamPar ChangeRadialDistortionCamPar(const MvString& Mode, const MvTuple& DistortionCoeffs) const;

  /**
   * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
   * @overload 采用标量数值参数。
   *
   * @return 输出相机参数，由返回对象自动管理资源。
   */
  MvCamPar ChangeRadialDistortionCamPar(const MvString& Mode, double DistortionCoeffs) const;

  /**
   * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 输出相机参数，由返回对象自动管理资源。
   */
  MvCamPar ChangeRadialDistortionCamPar(const char* Mode, double DistortionCoeffs) const;

  /**
   * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 输出相机参数，由返回对象自动管理资源。
   */
  MvCamPar ChangeRadialDistortionCamPar(const wchar_t* Mode, double DistortionCoeffs) const;

  /**
   * @brief 将三维相机坐标投影到图像平面。
   *
   * @param[in] X X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Y Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Z Z 坐标元组；调用期间借用，不转移所有权。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_project_3d_point
   */
  void Project3dPoint(const MvTuple& X, const MvTuple& Y, const MvTuple& Z, MvTuple* Row,
                      MvTuple* Column) const;

  /**
   * @brief 按输入和输出相机参数转换点的径向畸变。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Col 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[out] RowChanged 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ColChanged 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_points
   */
  void ChangeRadialDistortionPoints(const MvTuple& Row, const MvTuple& Col, const MvCamPar& CamParamOut,
                                    MvTuple* RowChanged, MvTuple* ColChanged) const;

  /**
   * @brief 将图像坐标映射到指定世界平面。
   *
   * @param[in] WorldPose 世界平面位姿；调用期间借用，不转移所有权。
   * @param[in] Rows 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Cols 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_image_points_to_world_plane
   */
  void ImagePointsToWorldPlane(const MvPose& WorldPose, const MvTuple& Rows, const MvTuple& Cols,
                               const MvTuple& Scale, MvTuple* X, MvTuple* Y) const;

  /**
   * @brief 将图像坐标映射到指定世界平面。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void ImagePointsToWorldPlane(const MvPose& WorldPose, const MvTuple& Rows, const MvTuple& Cols,
                               const MvString& Scale, MvTuple* X, MvTuple* Y) const;

  /**
   * @brief 将图像坐标映射到指定世界平面。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ImagePointsToWorldPlane(const MvPose& WorldPose, const MvTuple& Rows, const MvTuple& Cols,
                               const char* Scale, MvTuple* X, MvTuple* Y) const;

  /**
   * @brief 将图像坐标映射到指定世界平面。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ImagePointsToWorldPlane(const MvPose& WorldPose, const MvTuple& Rows, const MvTuple& Cols,
                               const wchar_t* Scale, MvTuple* X, MvTuple* Y) const;

  /**
   * @brief 按相机参数改变轮廓点径向畸变；点属性不保留。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_contours_xld
   */
  MvXLDCont ChangeRadialDistortionContoursXld(const MvXLDCont& Contours, const MvCamPar& CamParamOut) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   *
   * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Type XYZ 图像的生成方式；调用期间借用，不转移所有权。
   * @param[in] Pose 三维位姿；调用期间借用，不转移所有权。
   *
   * @return X 坐标，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_object_model_3d_to_xyz
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3DArray& ObjectModel3D,
                             const MvString& Type, const MvPose& Pose) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用字符串对象参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& ObjectModel3D,
                             const MvString& Type, const MvPose& Pose) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& ObjectModel3D, const char* Type,
                             const MvPose& Pose) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用宽字符字符串参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& ObjectModel3D,
                             const wchar_t* Type, const MvPose& Pose) const;

  /**
   * @brief 构造空的控制值对象。
   */
  MvCamPar() noexcept;

  /**
   * @brief 复制控制元组内容，构造当前值类型。
   *
   * @param[in] value 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvCamPar(const MvTuple& value);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCamPar(const MvCamPar&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvCamPar(MvCamPar&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCamPar& operator=(const MvCamPar&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvCamPar& operator=(MvCamPar&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvCamPar() noexcept;

  /**
   * @brief 读取上述 HCPR v3 及历史 MVKIT_CAM_PAR 文本；其他相机模型/版本显式拒绝。
   *
   * @param[in] CamParFile 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_cam_par
   */
  void ReadCamPar(const MvString& CamParFile);

  /**
   * @brief 读取上述 HCPR v3 及历史 MVKIT_CAM_PAR 文本；其他相机模型/版本显式拒绝。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadCamPar(const char* CamParFile);

  /**
   * @brief 读取上述 HCPR v3 及历史 MVKIT_CAM_PAR 文本；其他相机模型/版本显式拒绝。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ReadCamPar(const wchar_t* CamParFile);

  /**
   * @brief 写入 HALCON 24.11 HCPR v3；当前支持四种 area_scan division/polynomial 模型。
   *
   * @param[in] CamParFile 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_cam_par
   */
  void WriteCamPar(const MvString& CamParFile) const;

  /**
   * @brief 写入 HALCON 24.11 HCPR v3；当前支持四种 area_scan division/polynomial 模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteCamPar(const char* CamParFile) const;

  /**
   * @brief 写入 HALCON 24.11 HCPR v3；当前支持四种 area_scan division/polynomial 模型。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WriteCamPar(const wchar_t* CamParFile) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   *
   * @param[in] CalibDataID 包含标定数据模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
   * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_calib_data_cam_param
   */
  void SetCalibDataCamParam(const MvCalibData& CalibDataID, const MvTuple& CameraIdx,
                            const MvTuple& CameraType) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvCalibData& CalibDataID, const MvTuple& CameraIdx,
                            const MvString& CameraType) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvCalibData& CalibDataID, const MvTuple& CameraIdx,
                            const char* CameraType) const;

  /**
   * @brief 设置标定数据模型中指定相机的类型和参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void SetCalibDataCamParam(const MvCalibData& CalibDataID, const MvTuple& CameraIdx,
                            const wchar_t* CameraType) const;

};

/**
 * @brief 创建指定相机和标定对象数量的标定数据模型。
 *
 * @param[in] CalibSetup 标定配置类型元组；调用期间借用，不转移所有权。
 * @param[in] NumCameras 相机数量元组；调用期间借用，不转移所有权。
 * @param[in] NumCalibObjects 标定对象数量元组；调用期间借用，不转移所有权。
 * @param[out] CalibDataID 标定数据模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_calib_data
 */
MVKIT_CPP_API void CreateCalibData(const MvTuple& CalibSetup, const MvTuple& NumCameras,
                                   const MvTuple& NumCalibObjects, MvTuple* CalibDataID);

/**
 * @brief 设置标定数据模型中指定相机的类型和参数。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_calib_data_cam_param
 */
MVKIT_CPP_API void SetCalibDataCamParam(const MvTuple& CalibDataID, const MvTuple& CameraIdx,
                                        const MvTuple& CameraType, const MvTuple& CameraParam);

/**
 * @brief 设置指定标定对象的描述文件或描述内容。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjDescr 标定对象描述元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_calib_data_calib_object
 */
MVKIT_CPP_API void SetCalibDataCalibObject(const MvTuple& CalibDataID, const MvTuple& CalibObjIdx,
                                           const MvTuple& CalibObjDescr);

/**
 * @brief 按项目类型、索引和数据名称设置标定模型数据。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ItemType 数据项目的类别或索引元组；调用期间借用，不转移所有权。
 * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间借用，不转移所有权。
 * @param[in] DataName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] DataValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_calib_data
 */
MVKIT_CPP_API void SetCalibData(const MvTuple& CalibDataID, const MvTuple& ItemType, const MvTuple& ItemIdx,
                                const MvTuple& DataName, const MvTuple& DataValue);

/**
 * @brief 按项目类型、索引和数据名称查询标定模型数据。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ItemType 数据项目的类别或索引元组；调用期间借用，不转移所有权。
 * @param[in] ItemIdx 数据项目的类别或索引元组；调用期间借用，不转移所有权。
 * @param[in] DataName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] DataValue 参数或数据项的值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_calib_data
 */
MVKIT_CPP_API void GetCalibData(const MvTuple& CalibDataID, const MvTuple& ItemType, const MvTuple& ItemIdx,
                                const MvTuple& DataName, MvTuple* DataValue);

/**
 * @brief 保存指定相机、标定对象和位姿的观测点及初始位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间借用，不转移所有权。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Index 数据、元素或对象索引元组；调用期间借用，不转移所有权。
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_calib_data_observ_points
 */
MVKIT_CPP_API void SetCalibDataObservPoints(const MvTuple& CalibDataID, const MvTuple& CameraIdx,
                                            const MvTuple& CalibObjIdx, const MvTuple& CalibObjPoseIdx,
                                            const MvTuple& Row, const MvTuple& Column, const MvTuple& Index,
                                            const MvTuple& Pose);

/**
 * @brief 读取指定标定观测的图像坐标、点索引和位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Index 数据、元素或对象索引元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_calib_data_observ_points
 */
MVKIT_CPP_API void GetCalibDataObservPoints(const MvTuple& CalibDataID, const MvTuple& CameraIdx,
                                            const MvTuple& CalibObjIdx, const MvTuple& CalibObjPoseIdx,
                                            MvTuple* Row, MvTuple* Column, MvTuple* Index, MvTuple* Pose);

/**
 * @brief 检测图像中的标定对象，并将检测结果写入标定数据模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_find_calib_object
 */
MVKIT_CPP_API void FindCalibObject(const MvObject& Image, const MvTuple& CalibDataID,
                                   const MvTuple& CameraIdx, const MvTuple& CalibObjIdx,
                                   const MvTuple& CalibObjPoseIdx, const MvTuple& GenParamName,
                                   const MvTuple& GenParamValue);

/**
 * @brief 读取指定标定观测的轮廓对象。
 *
 * @param[out] Contours 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ContourName 对象或轮廓查询名称元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_calib_data_observ_contours
 */
MVKIT_CPP_API void GetCalibDataObservContours(MvObject* Contours, const MvTuple& CalibDataID,
                                              const MvTuple& ContourName, const MvTuple& CameraIdx,
                                              const MvTuple& CalibObjIdx, const MvTuple& CalibObjPoseIdx);

/**
 * @brief 根据已保存的标定观测优化相机参数及位姿。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] Error 错误详情或标定误差元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_calibrate_cameras
 */
MVKIT_CPP_API void CalibrateCameras(const MvTuple& CalibDataID, MvTuple* Error);

/**
 * @brief 根据已保存的标定观测求解手眼关系。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] Errors 错误详情或标定误差元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_calibrate_hand_eye
 */
MVKIT_CPP_API void CalibrateHandEye(const MvTuple& CalibDataID, MvTuple* Errors);

/**
 * @brief 删除单个观测；一个模型及三个整数索引，失败保持模型。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjIdx 标定对象索引元组；调用期间借用，不转移所有权。
 * @param[in] CalibObjPoseIdx 标定观测位姿索引元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_remove_calib_data_observ
 */
MVKIT_CPP_API void RemoveCalibDataObserv(const MvTuple& CalibDataID, const MvTuple& CameraIdx,
                                         const MvTuple& CalibObjIdx, const MvTuple& CalibObjPoseIdx);

/**
 * @brief 读取 `MVKIT_CALIB_DATA` 并返回 CalibDataID 句柄控制元组。
 *
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] CalibDataID 标定数据模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_calib_data
 */
MVKIT_CPP_API void ReadCalibData(const MvTuple& FileName, MvTuple* CalibDataID);

/**
 * @brief 写入 `MVKIT_CALIB_DATA` 状态；不含求解器缓存/指针。
 *
 * @param[in] CalibDataID 包含标定数据模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_calib_data
 */
MVKIT_CPP_API void WriteCalibData(const MvTuple& CalibDataID, const MvTuple& FileName);

/**
 * @brief 读取上述 HCPR v3 及历史 MVKIT_CAM_PAR 文本；其他相机模型/版本显式拒绝。
 *
 * @param[in] CamParFile 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] CameraParam 相机内参与畸变参数元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_cam_par
 */
MVKIT_CPP_API void ReadCamPar(const MvTuple& CamParFile, MvTuple* CameraParam);

/**
 * @brief 写入 HALCON 24.11 HCPR v3；当前支持四种 area_scan division/polynomial 模型。
 *
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParFile 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_cam_par
 */
MVKIT_CPP_API void WriteCamPar(const MvTuple& CameraParam, const MvTuple& CamParFile);

/**
 * @brief 读取 HALCON f/r/t 位姿文本及历史 MVKIT_POSE 文本。
 *
 * @param[in] PoseFile 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_pose
 */
MVKIT_CPP_API void ReadPose(const MvTuple& PoseFile, MvTuple* Pose);

/**
 * @brief 写入 HALCON f/r/t 位姿文本；保留输入表示类型。
 *
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 * @param[in] PoseFile 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_pose
 */
MVKIT_CPP_API void WritePose(const MvTuple& Pose, const MvTuple& PoseFile);

/**
 * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ContoursTrans 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] WorldPose 世界平面位姿元组；调用期间借用，不转移所有权。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_contour_to_world_plane_xld
 */
MVKIT_CPP_API void ContourToWorldPlaneXld(const MvObject& Contours, MvObject* ContoursTrans,
                                          const MvTuple& CameraParam, const MvTuple& WorldPose,
                                          const MvTuple& Scale);

/**
 * @brief 按相机参数改变轮廓点径向畸变；点属性不保留。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ContoursRectified 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] CamParamIn 源相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamOut 目标相机内参与畸变参数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_change_radial_distortion_contours_xld
 */
MVKIT_CPP_API void ChangeRadialDistortionContoursXld(const MvObject& Contours, MvObject* ContoursRectified,
                                                     const MvTuple& CamParamIn, const MvTuple& CamParamOut);

/**
 * @brief 将三维相机坐标投影到图像平面。
 *
 * @param[in] X X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Y Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Z Z 坐标元组；调用期间借用，不转移所有权。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_project_3d_point
 */
MVKIT_CPP_API void Project3dPoint(const MvTuple& X, const MvTuple& Y, const MvTuple& Z,
                                  const MvTuple& CameraParam, MvTuple* Row, MvTuple* Column);

/**
 * @brief 按输入和输出相机参数转换点的径向畸变。
 *
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Col 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] CamParamIn 源相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CamParamOut 目标相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[out] RowChanged 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ColChanged 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_change_radial_distortion_points
 */
MVKIT_CPP_API void ChangeRadialDistortionPoints(const MvTuple& Row, const MvTuple& Col,
                                                const MvTuple& CamParamIn, const MvTuple& CamParamOut,
                                                MvTuple* RowChanged, MvTuple* ColChanged);

/**
 * @brief 将图像坐标映射到指定世界平面。
 *
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] WorldPose 世界平面位姿元组；调用期间借用，不转移所有权。
 * @param[in] Rows 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Cols 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
 * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_image_points_to_world_plane
 */
MVKIT_CPP_API void ImagePointsToWorldPlane(const MvTuple& CameraParam, const MvTuple& WorldPose,
                                           const MvTuple& Rows, const MvTuple& Cols, const MvTuple& Scale,
                                           MvTuple* X, MvTuple* Y);

}  // namespace mv

namespace mv {
/**
 * @brief 改写相机畸变系数；fixed 要求 1 个 kappa 或 5 个多项式系数。
 *
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 * @param[in] CamParamIn 源相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] DistortionCoeffs 径向畸变系数元组；调用期间借用，不转移所有权。
 * @param[out] CamParamOut 目标相机内参与畸变参数元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_change_radial_distortion_cam_par
 */
MVKIT_CPP_API void ChangeRadialDistortionCamPar(const MvTuple& Mode, const MvTuple& CamParamIn,
                                                const MvTuple& DistortionCoeffs, MvTuple* CamParamOut);

}
