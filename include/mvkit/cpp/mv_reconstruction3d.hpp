#pragma once
#include <mvkit/cpp/mv_geometry.hpp>
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {
class MvCamPar;
/**
 * @brief 自动管理共享引用的多相机配置模型。
 *
 * @details 共享相机配置句柄，固定相机数量可大于二。
 */
class MVKIT_CPP_API MvCameraSetupModel : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvCameraSetupModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvCameraSetupModel(const MvHandle& handle);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCameraSetupModel(const MvCameraSetupModel&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvCameraSetupModel(MvCameraSetupModel&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvCameraSetupModel& operator=(const MvCameraSetupModel&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvCameraSetupModel& operator=(MvCameraSetupModel&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvCameraSetupModel() noexcept;

  /**
   * @brief HALCON C2：固定正相机数量，每台相机具有独立世界位姿。
   *
   * @param[in] NumCameras 相机数量。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_camera_setup_model
   */
  void CreateCameraSetupModel(std::int64_t NumCameras);

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   *
   * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
   * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] CameraPose 相机位姿元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_camera_setup_cam_param
   */
  void SetCameraSetupCamParam(const MvTuple& CameraIdx, const MvTuple& CameraType,
                              const MvCamPar& CameraParam, const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvTuple& CameraIdx, const MvString& CameraType,
                              const MvCamPar& CameraParam, const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvTuple& CameraIdx, const char* CameraType, const MvCamPar& CameraParam,
                              const MvTuple& CameraPose) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void SetCameraSetupCamParam(const MvTuple& CameraIdx, const wchar_t* CameraType,
                              const MvCamPar& CameraParam, const MvTuple& CameraPose) const;

  /**
   * @brief 查询相机配置模型的指定参数。
   *
   * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称；调用期间借用，不转移所有权。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_camera_setup_param
   */
  MvTuple GetCameraSetupParam(const MvTuple& CameraIdx, const MvString& GenParamName) const;

  /**
   * @brief 查询相机配置模型的指定参数。
   * @overload 采用标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCameraSetupParam(std::int64_t CameraIdx, const MvString& GenParamName) const;

  /**
   * @brief 查询相机配置模型的指定参数。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCameraSetupParam(std::int64_t CameraIdx, const char* GenParamName) const;

  /**
   * @brief 查询相机配置模型的指定参数。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetCameraSetupParam(std::int64_t CameraIdx, const wchar_t* GenParamName) const;

};
/**
 * @brief HALCON C2：固定正相机数量，每台相机具有独立世界位姿。
 *
 * @param[in] NumCameras 相机数量元组；调用期间借用，不转移所有权。
 * @param[out] CameraSetupModelID 相机配置模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_camera_setup_model
 */
MVKIT_CPP_API void CreateCameraSetupModel(const MvTuple& NumCameras, MvTuple* CameraSetupModelID);

/**
 * @brief 设置相机配置模型中指定相机的参数及位姿。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] CameraType 相机模型类型元组；调用期间借用，不转移所有权。
 * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] CameraPose 相机位姿元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_camera_setup_cam_param
 */
MVKIT_CPP_API void SetCameraSetupCamParam(const MvTuple& CameraSetupModelID, const MvTuple& CameraIdx,
                                          const MvTuple& CameraType, const MvTuple& CameraParam,
                                          const MvTuple& CameraPose);

/**
 * @brief 查询相机配置模型的指定参数。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] GenParamValue 参数或数据项的值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_camera_setup_param
 */
MVKIT_CPP_API void GetCameraSetupParam(const MvTuple& CameraSetupModelID, const MvTuple& CameraIdx,
                                       const MvTuple& GenParamName, MvTuple* GenParamValue);

class MvStructuredLightModel;
class MvObjectModel3DArray;
/**
 * @brief 自动管理共享引用的三维对象模型。
 *
 * @details 共享对象模型身份；显式清理使全部副本失效。
 */
class MVKIT_CPP_API MvObjectModel3D : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvObjectModel3D() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvObjectModel3D(const MvHandle&);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectModel3D(const MvObjectModel3D&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvObjectModel3D(MvObjectModel3D&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectModel3D& operator=(const MvObjectModel3D&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvObjectModel3D& operator=(MvObjectModel3D&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvObjectModel3D() noexcept;

  /**
   * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Attrib 属性或特征名称元组；调用期间借用，不转移所有权。
   * @param[in] MinValue 属性筛选范围界限元组；调用期间借用，不转移所有权。
   * @param[in] MaxValue 属性筛选范围界限元组；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_points_object_model_3d
   */
  static MvObjectModel3DArray SelectPointsObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                        const MvTuple& Attrib, const MvTuple& MinValue,
                                                        const MvTuple& MaxValue);

  /**
   * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
   * @overload 采用标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SelectPointsObjectModel3d(const MvString& Attrib, double MinValue, double MaxValue) const;

  /**
   * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SelectPointsObjectModel3d(const char* Attrib, double MinValue, double MaxValue) const;

  /**
   * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SelectPointsObjectModel3d(const wchar_t* Attrib, double MinValue, double MaxValue) const;

  /**
   * @brief 按指定特征及阈值分解三维对象模型的连通分量。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Feature 属性或特征名称元组；调用期间借用，不转移所有权。
   * @param[in] Value 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_connection_object_model_3d
   */
  static MvObjectModel3DArray ConnectionObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                      const MvTuple& Feature, const MvTuple& Value);

  /**
   * @brief 按指定特征及阈值分解三维对象模型的连通分量。
   * @overload 采用标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3DArray ConnectionObjectModel3d(const MvString& Feature, double Value) const;

  /**
   * @brief 按指定特征及阈值分解三维对象模型的连通分量。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3DArray ConnectionObjectModel3d(const char* Feature, double Value) const;

  /**
   * @brief 按指定特征及阈值分解三维对象模型的连通分量。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3DArray ConnectionObjectModel3d(const wchar_t* Feature, double Value) const;

  /**
   * @brief 计算三维对象模型与指定平面的交线。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Plane 截交平面；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_intersect_plane_object_model_3d
   */
  static MvObjectModel3DArray IntersectPlaneObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                          const MvPoseArray& Plane);

  /**
   * @brief 计算三维对象模型与指定平面的交线。
   * @overload 采用强类型对象参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D IntersectPlaneObjectModel3d(const MvPose& Plane) const;

  /**
   * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 逐项操作状态元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_object_model_3d
   */
  MvTuple ReadObjectModel3d(const MvString& FileName, const MvTuple& Scale, const MvTuple& GenParamName,
                            const MvTuple& GenParamValue);

  /**
   * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
   * @overload 采用字符串对象参数。
   *
   * @return 逐项操作状态元组，由返回对象自动管理资源。
   */
  static MvTuple ReadObjectModel3d(const MvString& FileName, const MvTuple& Scale,
                                   const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                   MvObjectModel3DArray* ObjectModel3D);

  /**
   * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
   * @overload 采用字符串对象参数。
   *
   * @return 逐项操作状态，由返回对象自动管理资源。
   */
  MvString ReadObjectModel3d(const MvString& FileName, const MvString& Scale, const MvString& GenParamName,
                             const MvString& GenParamValue);

  /**
   * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 逐项操作状态，由返回对象自动管理资源。
   */
  MvString ReadObjectModel3d(const char* FileName, const char* Scale, const char* GenParamName,
                             const char* GenParamValue);

  /**
   * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
   * @overload 采用宽字符字符串参数。
   *
   * @return 逐项操作状态，由返回对象自动管理资源。
   */
  MvString ReadObjectModel3d(const wchar_t* FileName, const wchar_t* Scale, const wchar_t* GenParamName,
                             const wchar_t* GenParamValue);

  /**
   * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
   *
   * @param[in] FileType 图像或模型的文件格式；调用期间借用，不转移所有权。
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_object_model_3d
   */
  void WriteObjectModel3d(const MvString& FileType, const MvString& FileName, const MvTuple& GenParamName,
                          const MvTuple& GenParamValue) const;

  /**
   * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void WriteObjectModel3d(const MvString& FileType, const MvString& FileName, const MvString& GenParamName,
                          const MvString& GenParamValue) const;

  /**
   * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteObjectModel3d(const char* FileType, const char* FileName, const char* GenParamName,
                          const char* GenParamValue) const;

  /**
   * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WriteObjectModel3d(const wchar_t* FileType, const wchar_t* FileName, const wchar_t* GenParamName,
                          const wchar_t* GenParamValue) const;

  /**
   * @brief 根据解码结果重建结构光三维表面。
   *
   * @param[in] StructuredLightModel 包含结构光模型句柄的值；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_reconstruct_surface_structured_light
   */
  void ReconstructSurfaceStructuredLight(const MvStructuredLightModel& StructuredLightModel);

  /**
   * @brief 由等长 X/Y/Z 坐标元组 生成无组织点云；按 HALCON 保留非有限坐标。
   *
   * @param[in] X X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Y Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Z Z 坐标元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_object_model_3d_from_points
   */
  void GenObjectModel3dFromPoints(const MvTuple& X, const MvTuple& Y, const MvTuple& Z);

  /**
   * @brief 由等长 X/Y/Z 坐标元组 生成无组织点云；按 HALCON 保留非有限坐标。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenObjectModel3dFromPoints(double X, double Y, double Z);

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   *
   * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Type XYZ 图像的生成方式；调用期间借用，不转移所有权。
   * @param[in] CamParam 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] Pose 三维位姿；调用期间借用，不转移所有权。
   *
   * @return X 坐标，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_object_model_3d_to_xyz
   */
  static MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3DArray& ObjectModel3D,
                                    const MvString& Type, const MvCamPar& CamParam, const MvPose& Pose);

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用字符串对象参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvString& Type, const MvCamPar& CamParam,
                             const MvPose& Pose) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const char* Type, const MvCamPar& CamParam,
                             const MvPose& Pose) const;

  /**
   * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
   * @overload 采用宽字符字符串参数。
   *
   * @return X 坐标，由返回对象自动管理资源。
   */
  MvImage ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const wchar_t* Type, const MvCamPar& CamParam,
                             const MvPose& Pose) const;

  /**
   * @brief HALCON C2：距离及可选最近索引存储到源模型。
   *
   * @param[in] ObjectModel3DTo 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Pose 三维位姿；调用期间借用，不转移所有权。
   * @param[in] MaxDistance 距离阈值元组；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_object_model_3d
   */
  void DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose,
                             const MvTuple& MaxDistance, const MvTuple& GenParamName,
                             const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2：距离及可选最近索引存储到源模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose, double MaxDistance,
                             const MvString& GenParamName, const MvString& GenParamValue) const;

  /**
   * @brief HALCON C2：距离及可选最近索引存储到源模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose, double MaxDistance,
                             const char* GenParamName, const char* GenParamValue) const;

  /**
   * @brief HALCON C2：距离及可选最近索引存储到源模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose, double MaxDistance,
                             const wchar_t* GenParamName, const wchar_t* GenParamValue) const;

  /**
   * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
   *
   * @param[in] ObjectModel3D2 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_register_object_model_3d_pair
   */
  MvPose RegisterObjectModel3dPair(const MvObjectModel3D& ObjectModel3D2, const MvString& Method,
                                   const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                   MvTuple* Score) const;

  /**
   * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
   * @overload 采用标量数值参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose RegisterObjectModel3dPair(const MvObjectModel3D& ObjectModel3D2, const MvString& Method,
                                   const MvString& GenParamName, double GenParamValue, MvTuple* Score) const;

  /**
   * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose RegisterObjectModel3dPair(const MvObjectModel3D& ObjectModel3D2, const char* Method,
                                   const char* GenParamName, double GenParamValue, MvTuple* Score) const;

  /**
   * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  MvPose RegisterObjectModel3dPair(const MvObjectModel3D& ObjectModel3D2, const wchar_t* Method,
                                   const wchar_t* GenParamName, double GenParamValue, MvTuple* Score) const;

  /**
   * @brief HALCON C2：每模型一个拟合输出；不可观测的拟合不产生原语。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fit_primitives_object_model_3d
   */
  static MvObjectModel3DArray FitPrimitivesObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                         const MvTuple& GenParamName,
                                                         const MvTuple& GenParamValue);

  /**
   * @brief HALCON C2：每模型一个拟合输出；不可观测的拟合不产生原语。
   * @overload 采用控制元组参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D FitPrimitivesObjectModel3d(const MvTuple& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   * @param[out] Information 求解或拟合的信息量元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 三角化后的三维对象模型，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_triangulate_object_model_3d
   */
  static MvObjectModel3DArray TriangulateObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                       const MvString& Method, const MvTuple& GenParamName,
                                                       const MvTuple& GenParamValue, MvTuple* Information);

  /**
   * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
   * @overload 采用字符串对象及标量输出参数。
   *
   * @return 三角化后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D TriangulateObjectModel3d(const MvString& Method, const MvTuple& GenParamName,
                                           const MvTuple& GenParamValue, std::int64_t* Information) const;

  /**
   * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 三角化后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D TriangulateObjectModel3d(const char* Method, const MvTuple& GenParamName,
                                           const MvTuple& GenParamValue, std::int64_t* Information) const;

  /**
   * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
   * @overload 采用宽字符字符串参数。
   *
   * @return 三角化后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D TriangulateObjectModel3d(const wchar_t* Method, const MvTuple& GenParamName,
                                           const MvTuple& GenParamValue, std::int64_t* Information) const;

  /**
   * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] SamplingParam 采样间距或采样控制元组；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 采样后的三维对象模型，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_sample_object_model_3d
   */
  static MvObjectModel3DArray SampleObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                  const MvString& Method, const MvTuple& SamplingParam,
                                                  const MvTuple& GenParamName, const MvTuple& GenParamValue);

  /**
   * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
   * @overload 采用标量数值参数。
   *
   * @return 采样后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D SampleObjectModel3d(const MvString& Method, double SamplingParam,
                                      const MvTuple& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 采样后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D SampleObjectModel3d(const char* Method, double SamplingParam, const MvTuple& GenParamName,
                                      const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 采样后的三维对象模型，由返回对象自动管理资源。
   */
  MvObjectModel3D SampleObjectModel3d(const wchar_t* Method, double SamplingParam,
                                      const MvTuple& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_surface_normals_object_model_3d
   */
  static MvObjectModel3DArray SurfaceNormalsObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                          const MvString& Method, const MvTuple& GenParamName,
                                                          const MvTuple& GenParamValue);

  /**
   * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
   * @overload 采用字符串对象参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SurfaceNormalsObjectModel3d(const MvString& Method, const MvTuple& GenParamName,
                                              const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SurfaceNormalsObjectModel3d(const char* Method, const MvTuple& GenParamName,
                                              const MvTuple& GenParamValue) const;

  /**
   * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
   * @overload 采用宽字符字符串参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D SurfaceNormalsObjectModel3d(const wchar_t* Method, const MvTuple& GenParamName,
                                              const MvTuple& GenParamValue) const;

  /**
   * @brief 使用 X、Y、Z 坐标图像构造一个三维对象模型。
   *
   * @param[in] X X 坐标；调用期间借用，不转移所有权。
   * @param[in] Y Y 坐标；调用期间借用，不转移所有权。
   * @param[in] Z Z 坐标；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_xyz_to_object_model_3d
   */
  void XyzToObjectModel3d(const MvImage& X, const MvImage& Y, const MvImage& Z);

  /**
   * @brief 按参数名称读取三维对象模型的属性。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_object_model_3d_params
   */
  static MvTuple GetObjectModel3dParams(const MvObjectModel3DArray& ObjectModel3D,
                                        const MvTuple& GenParamName);

  /**
   * @brief 按参数名称读取三维对象模型的属性。
   * @overload 采用控制元组参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetObjectModel3dParams(const MvTuple& GenParamName) const;

  /**
   * @brief 用位姿刚体变换点云，输出新不可变模型；保留 有组织点云的映射与置信度。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] Pose 三维位姿；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_rigid_trans_object_model_3d
   */
  static MvObjectModel3DArray RigidTransObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                      const MvPoseArray& Pose);

  /**
   * @brief 用位姿刚体变换点云，输出新不可变模型；保留 有组织点云的映射与置信度。
   * @overload 采用强类型对象参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D RigidTransObjectModel3d(const MvPose& Pose) const;

  /**
   * @brief 用 12 元 3x4 仿射变换点云；无逆变换属性的点云接受奇异矩阵。
   *
   * @param[in] ObjectModel3D 包含三维对象模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] HomMat3D 三维齐次变换矩阵；调用期间借用，不转移所有权。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_object_model_3d
   */
  static MvObjectModel3DArray AffineTransObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                       const MvHomMat3DArray& HomMat3D);

  /**
   * @brief 用 12 元 3x4 仿射变换点云；无逆变换属性的点云接受奇异矩阵。
   * @overload 采用强类型对象参数。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   */
  MvObjectModel3D AffineTransObjectModel3d(const MvHomMat3D& HomMat3D) const;

};
struct MvObjectModel3DArrayData;
/**
 * @brief 拥有型三维对象模型数组，自动管理各元素的共享引用。
 */
class MVKIT_CPP_API MvObjectModel3DArray {
 public:
  /**
   * @brief 构造空的拥有型数组。
   */
  MvObjectModel3DArray() noexcept;

  /**
   * @brief 复制数组元素，构造拥有型数组。
   *
   * @param[in] models 需要复制的模型数组。
   * @param[in] length 几何长度或半长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectModel3DArray(const MvObjectModel3D* models, std::int64_t length);

  /**
   * @brief 复制构造，复制数组元素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectModel3DArray(const MvObjectModel3DArray&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvObjectModel3DArray(MvObjectModel3DArray&&) noexcept;

  /**
   * @brief 复制赋值，复制数组元素。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectModel3DArray& operator=(const MvObjectModel3DArray&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvObjectModel3DArray& operator=(MvObjectModel3DArray&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvObjectModel3DArray() noexcept;

  /**
   * @brief 清空当前对象内容并释放其拥有的资源。
   *
   * @return 无返回值。
   */
  virtual void Clear() noexcept;

  /**
   * @brief 借用数组的连续元素存储。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return 连续元素存储的只读借用指针。
   */
  const MvObjectModel3D* Tools() const noexcept;

  /**
   * @brief 查询当前数组的元素数量。
   *
   * @return 当前容器的元素数量。
   */
  virtual std::int64_t Length() const noexcept;

  /**
   * @brief 从控制元组更新当前对象内容。
   *
   * @param[in] handles 包含受检控制句柄的控制元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual void SetFromTuple(const MvTuple& handles);

  /**
   * @brief 将当前对象内容转换为拥有型控制元组。
   *
   * @return 当前内容的拥有型控制元组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual MvTuple ConvertToTuple() const;

 private:
  MvObjectModel3DArrayData* data_{nullptr};
};
/**
 * @brief 由等长 X/Y/Z 坐标元组 生成无组织点云；按 HALCON 保留非有限坐标。
 *
 * @param[in] X X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Y Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Z Z 坐标元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_object_model_3d_from_points
 */
MVKIT_CPP_API void GenObjectModel3dFromPoints(const MvTuple& X, const MvTuple& Y, const MvTuple& Z,
                                              MvTuple* ObjectModel3D);

/**
 * @brief HALCON C2；from_xyz_map 要求恰好一个具有存储映射的模型。
 *
 * @param[out] X X 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Type XYZ 图像的生成方式元组；调用期间借用，不转移所有权。
 * @param[in] CamParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_object_model_3d_to_xyz
 */
MVKIT_CPP_API void ObjectModel3dToXyz(MvObject* X, MvObject* Y, MvObject* Z, const MvTuple& ObjectModel3D,
                                      const MvTuple& Type, const MvTuple& CamParam, const MvTuple& Pose);

/**
 * @brief 使用 X、Y、Z 坐标图像构造一个三维对象模型。
 *
 * @param[in] X X 坐标；调用期间借用，不转移所有权。
 * @param[in] Y Y 坐标；调用期间借用，不转移所有权。
 * @param[in] Z Z 坐标；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3D 包含三维对象模型句柄的控制元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_xyz_to_object_model_3d
 */
MVKIT_CPP_API void XyzToObjectModel3d(const MvObject& X, const MvObject& Y, const MvObject& Z,
                                      MvTuple* ObjectModel3D);

/**
 * @brief 按参数名称读取三维对象模型的属性。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] GenParamValue 参数或数据项的值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_object_model_3d_params
 */
MVKIT_CPP_API void GetObjectModel3dParams(const MvTuple& ObjectModel3D, const MvTuple& GenParamName,
                                          MvTuple* GenParamValue);

/**
 * @brief 用位姿刚体变换点云，输出新不可变模型；保留 有组织点云的映射与置信度。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DRigidTrans 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_rigid_trans_object_model_3d
 */
MVKIT_CPP_API void RigidTransObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Pose,
                                           MvTuple* ObjectModel3DRigidTrans);

/**
 * @brief 用 12 元 3x4 仿射变换点云；无逆变换属性的点云接受奇异矩阵。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DAffineTrans 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_object_model_3d
 */
MVKIT_CPP_API void AffineTransObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& HomMat3D,
                                            MvTuple* ObjectModel3DAffineTrans);

/**
 * @brief 按属性名称列表或逐点数值闭区间筛选，维护点关联的 mapping、mesh 和 lines。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Attrib 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] MinValue 属性筛选范围界限元组；调用期间借用，不转移所有权。
 * @param[in] MaxValue 属性筛选范围界限元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DThresholded 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_points_object_model_3d
 */
MVKIT_CPP_API void SelectPointsObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Attrib,
                                             const MvTuple& MinValue, const MvTuple& MaxValue,
                                             MvTuple* ObjectModel3DThresholded);

/**
 * @brief 按指定特征及阈值分解三维对象模型的连通分量。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Feature 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] Value 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DConnected 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_connection_object_model_3d
 */
MVKIT_CPP_API void ConnectionObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Feature,
                                           const MvTuple& Value, MvTuple* ObjectModel3DConnected);

/**
 * @brief 计算三维对象模型与指定平面的交线。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Plane 截交平面元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DIntersection 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_intersect_plane_object_model_3d
 */
MVKIT_CPP_API void IntersectPlaneObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Plane,
                                               MvTuple* ObjectModel3DIntersection);

/**
 * @brief 读取 PLY 或 MVKit 原生格式；Scale 接收单位字符串或数值，Status 为独立控制输出。
 *
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Status 逐项操作状态元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_object_model_3d
 */
MVKIT_CPP_API void ReadObjectModel3d(const MvTuple& FileName, const MvTuple& Scale,
                                     const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                     MvTuple* ObjectModel3D, MvTuple* Status);

/**
 * @brief FileType=`ply` 为交换格式，只写有效点；`mvkit` 保存组织映射、有效掩码、confidence、法向、mesh
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] FileType 图像或模型的文件格式元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_object_model_3d
 */
MVKIT_CPP_API void WriteObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& FileType,
                                      const MvTuple& FileName, const MvTuple& GenParamName,
                                      const MvTuple& GenParamValue);

/**
 * @brief HALCON C2：Method 为 icp/matching；Pose 将模型 1 变换到模型 2，Score 为标量重叠评分。
 *
 * @param[in] ObjectModel3D1 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ObjectModel3D2 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_register_object_model_3d_pair
 */
MVKIT_CPP_API void RegisterObjectModel3dPair(const MvTuple& ObjectModel3D1, const MvTuple& ObjectModel3D2,
                                             const MvTuple& Method, const MvTuple& GenParamName,
                                             const MvTuple& GenParamValue, MvTuple* Pose, MvTuple* Score);

/**
 * @brief HALCON C2：每模型一个拟合输出；不可观测的拟合不产生原语。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DOut 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fit_primitives_object_model_3d
 */
MVKIT_CPP_API void FitPrimitivesObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& GenParamName,
                                              const MvTuple& GenParamValue, MvTuple* ObjectModel3DOut);

/**
 * @brief HALCON C2：xyz_mapping 及孔洞/视点控制，支持三角 polygon 转换。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] TriangulatedObjectModel3D 三角化后的三维对象模型元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Information 求解或拟合的信息量元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_triangulate_object_model_3d
 */
MVKIT_CPP_API void TriangulateObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method,
                                            const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                            MvTuple* TriangulatedObjectModel3D, MvTuple* Information);

/**
 * @brief HALCON C2 采样控制；支持的点采样方法及数值差异见模块文档。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] SamplingParam 采样间距或采样控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] SampledObjectModel3D 采样后的三维对象模型元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_sample_object_model_3d
 */
MVKIT_CPP_API void SampleObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method,
                                       const MvTuple& SamplingParam, const MvTuple& GenParamName,
                                       const MvTuple& GenParamValue, MvTuple* SampledObjectModel3D);

/**
 * @brief HALCON C2 MLS、映射及三角顶点法向；数值/属性差异继续按模块文档保留。
 *
 * @param[in] ObjectModel3D 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3DNormals 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_surface_normals_object_model_3d
 */
MVKIT_CPP_API void SurfaceNormalsObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method,
                                               const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                               MvTuple* ObjectModel3DNormals);

}  // namespace mv

namespace mv {
/**
 * @brief HALCON C2：距离及可选最近索引存储到源模型。
 *
 * @param[in] ObjectModel3DFrom 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ObjectModel3DTo 包含三维对象模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 * @param[in] MaxDistance 距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_object_model_3d
 */
MVKIT_CPP_API void DistanceObjectModel3d(const MvTuple& ObjectModel3DFrom, const MvTuple& ObjectModel3DTo,
                                         const MvTuple& Pose, const MvTuple& MaxDistance,
                                         const MvTuple& GenParamName, const MvTuple& GenParamValue);

}  // namespace mv
