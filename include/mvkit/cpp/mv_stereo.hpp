#pragma once

#include <mvkit/cpp/mv_iconic.hpp>
#include <mvkit/cpp/mv_calibration.hpp>

namespace mv {

/**
 * @brief 自动管理共享引用的可变立体重建模型。
 *
 * @details 稀疏多相机模型；外部副本共享身份和清理状态。
 */
class MVKIT_CPP_API MvStereoModel : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvStereoModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStereoModel(const MvHandle& handle);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvStereoModel(const MvStereoModel&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvStereoModel(MvStereoModel&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvStereoModel& operator=(const MvStereoModel&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvStereoModel& operator=(MvStereoModel&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvStereoModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStereoModel(const MvCameraSetupModel&, const MvString& Method, const MvTuple& GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStereoModel(const MvCameraSetupModel&, const MvString& Method, const MvString& GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] Method 算法方法。
   * @param[in] GenParamName 参数或数据项名称。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStereoModel(const MvCameraSetupModel&, const char* Method, const char* GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] Method 算法方法。
   * @param[in] GenParamName 参数或数据项名称。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStereoModel(const MvCameraSetupModel&, const wchar_t* Method, const wchar_t* GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
   *
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_stereo_model
   */
  void CreateStereoModel(const MvCameraSetupModel&, const MvString& Method, const MvTuple& GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void CreateStereoModel(const MvCameraSetupModel&, const MvString& Method, const MvString& GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void CreateStereoModel(const MvCameraSetupModel&, const char* Method, const char* GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void CreateStereoModel(const MvCameraSetupModel&, const wchar_t* Method, const wchar_t* GenParamName,
                         const MvTuple& GenParamValue);

  /**
   * @brief 设置稀疏多相机重建模型的指定参数。
   *
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_stereo_model_param
   */
  void SetStereoModelParam(const MvTuple& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief 设置稀疏多相机重建模型的指定参数。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetStereoModelParam(const MvString& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief 设置稀疏多相机重建模型的指定参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetStereoModelParam(const char* GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief 设置稀疏多相机重建模型的指定参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void SetStereoModelParam(const wchar_t* GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief 清理稀疏多相机重建模型及其状态。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_clear_stereo_model
   */
  void ClearStereoModel() const;

  /**
   * @brief 观测元组等长；CovIP 为空或每个观测四个值。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] CovIP 图像点或世界点的测量协方差元组；调用期间借用，不转移所有权。
   * @param[in] CameraIdx 相机索引。
   * @param[in] PointIdx 输入点索引。
   * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] CovWP 图像点或世界点的测量协方差元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] PointIdxOut 输出点索引元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_reconstruct_points_stereo
   */
  void ReconstructPointsStereo(const MvTuple& Row, const MvTuple& Column, const MvTuple& CovIP,
                               std::int64_t CameraIdx, std::int64_t PointIdx, MvTuple* X, MvTuple* Y,
                               MvTuple* Z, MvTuple* CovWP, MvTuple* PointIdxOut) const;

  /**
   * @brief 观测元组等长；CovIP 为空或每个观测四个值。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void ReconstructPointsStereo(double Row, double Column, const MvTuple& CovIP, std::int64_t CameraIdx,
                               std::int64_t PointIdx, double* X, double* Y, double* Z, double* CovWP,
                               std::int64_t* PointIdxOut) const;

};
/**
 * @brief 稀疏双目模型，创建时复制相机配置；当前实现 points_3d，surface 模型返回 Unsupported。
 *
 * @param[in] CameraSetupModelID 包含相机配置模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 * @param[out] StereoModelID 稀疏重建模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_stereo_model
 */
MVKIT_CPP_API void CreateStereoModel(const MvTuple& CameraSetupModelID, const MvTuple& Method,
                                     const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                     MvTuple* StereoModelID);

/**
 * @brief 设置稀疏多相机重建模型的指定参数。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_stereo_model_param
 */
MVKIT_CPP_API void SetStereoModelParam(const MvTuple& StereoModelID, const MvTuple& GenParamName,
                                       const MvTuple& GenParamValue);

/**
 * @brief 清理稀疏多相机重建模型及其状态。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_clear_stereo_model
 */
MVKIT_CPP_API void ClearStereoModel(const MvTuple& StereoModelID);

/**
 * @brief 观测元组等长；CovIP 为空或每个观测四个值。
 *
 * @param[in] StereoModelID 包含稀疏重建模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] CovIP 图像点或世界点的测量协方差元组；调用期间借用，不转移所有权。
 * @param[in] CameraIdx 相机索引元组；调用期间借用，不转移所有权。
 * @param[in] PointIdx 输入点索引元组；调用期间借用，不转移所有权。
 * @param[out] X X 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Y Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Z Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] CovWP 图像点或世界点的测量协方差元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] PointIdxOut 输出点索引元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_reconstruct_points_stereo
 */
MVKIT_CPP_API void ReconstructPointsStereo(const MvTuple& StereoModelID, const MvTuple& Row,
                                           const MvTuple& Column, const MvTuple& CovIP,
                                           const MvTuple& CameraIdx, const MvTuple& PointIdx, MvTuple* X,
                                           MvTuple* Y, MvTuple* Z, MvTuple* CovWP, MvTuple* PointIdxOut);

/**
 * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
 *
 * @param[in] ImageRect1 第一张校正图像；调用期间借用，不转移所有权。
 * @param[in] ImageRect2 第二张校正图像；调用期间借用，不转移所有权。
 * @param[out] Disparity 视差结果；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Score 匹配或视差得分；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] TextureThresh 纹理阈值元组；调用期间借用，不转移所有权。
 * @param[in] MinDisparity 视差搜索下限元组；调用期间借用，不转移所有权。
 * @param[in] MaxDisparity 视差搜索上限元组；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] ScoreThresh 得分阈值元组；调用期间借用，不转移所有权。
 * @param[in] Filter 视差过滤方式元组；调用期间借用，不转移所有权。
 * @param[in] SubDisparity 亚像素细化方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_binocular_disparity
 */
MVKIT_CPP_API void BinocularDisparity(
    const MvObject& ImageRect1, const MvObject& ImageRect2, MvObject* Disparity, MvObject* Score,
    const MvTuple& Method, const MvTuple& MaskWidth, const MvTuple& MaskHeight,
    const MvTuple& TextureThresh, const MvTuple& MinDisparity, const MvTuple& MaxDisparity,
    const MvTuple& NumLevels, const MvTuple& ScoreThresh, const MvTuple& Filter,
    const MvTuple& SubDisparity);

/**
 * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
 *
 * @param[in] ImageRect1 第一张校正图像；调用期间借用，不转移所有权。
 * @param[in] ImageRect2 第二张校正图像；调用期间借用，不转移所有权。
 * @param[out] Disparity 视差结果；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Score 匹配或视差得分；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MinDisparity 视差搜索下限元组；调用期间借用，不转移所有权。
 * @param[in] MaxDisparity 视差搜索上限元组；调用期间借用，不转移所有权。
 * @param[in] SurfaceSmoothing 表面平滑控制值元组；调用期间借用，不转移所有权。
 * @param[in] EdgeSmoothing 边缘平滑控制值元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_binocular_disparity_ms
 */
MVKIT_CPP_API void BinocularDisparityMs(
    const MvObject& ImageRect1, const MvObject& ImageRect2, MvObject* Disparity, MvObject* Score,
    const MvTuple& MinDisparity, const MvTuple& MaxDisparity, const MvTuple& SurfaceSmoothing,
    const MvTuple& EdgeSmoothing, const MvTuple& GenParamName, const MvTuple& GenParamValue);

}  // namespace mv
