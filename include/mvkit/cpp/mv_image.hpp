#pragma once
#include <mvkit/c/mv_image.h>

#include <mvkit/cpp/mv_iconic.hpp>

namespace mv {

/** @brief 图像平面的像素类型。 */
enum class MvPixelType : std::int32_t {
  UInt8 = 0,
  UInt16 = 1,
  Int16 = 2,
  Int32 = 3,
  Float32 = 4,
};

/**
 * @brief 仅包含图像的图标对象数组；对象索引与图像通道索引相互独立。
 *
 * @details SelectObj 选择图像对象，AccessChannel 选择图像通道。
 */
class MVKIT_CPP_API MvImage : public MvObject {
 public:
  /**
   * @brief 创建像素值为零的单通道图像。
   *
   * @param[in] Type 像素类型名称，支持 byte/uint2/int2/int4/real。
   * @param[in] Width 图像宽度，创建时为正整数。
   * @param[in] Height 图像高度，创建时为正整数。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImageConst(const MvTuple& Type, const MvTuple& Width, const MvTuple& Height);

  /**
   * @brief 创建像素值为零的单通道图像。
   *
   * @overload 使用像素类型名称和整数尺寸。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImageConst(const char* Type, std::int64_t Width, std::int64_t Height);

  /**
   * @brief 从连续像素复制创建单通道图像。
   *
   * @param[in] Type 像素类型名称，支持 byte/uint2/int2/int4/real。
   * @param[in] Width 图像宽度，创建时为正整数。
   * @param[in] Height 图像高度，创建时为正整数。
   * @param[in] PixelPointer 连续像素的实际地址；输入只在调用期间读取。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImage1(const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer);

  /**
   * @brief 从连续像素复制创建单通道图像。
   *
   * @overload 使用原生指针和整数尺寸。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImage1(const char* Type, std::int64_t Width, std::int64_t Height, const void* PixelPointer);

  /**
   * @brief 从同类型连续 RGB 三平面复制创建三通道图像。
   *
   * @param[in] Type 像素类型名称，支持 byte/uint2/int2/int4/real。
   * @param[in] Width 图像宽度，创建时为正整数。
   * @param[in] Height 图像高度，创建时为正整数。
   * @param[in] Red 连续红色像素平面的地址。
   * @param[in] Green 连续绿色像素平面的地址。
   * @param[in] Blue 连续蓝色像素平面的地址。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImage3(const char* Type, std::int64_t Width, std::int64_t Height, const void* Red, const void* Green, const void* Blue);

  /**
   * @brief 引用外部连续单通道像素，并按标准回调协议清理。
   *
   * @param[in] Type 像素类型名称，支持 byte/uint2/int2/int4/real。
   * @param[in] Width 图像宽度，创建时为正整数。
   * @param[in] Height 图像高度，创建时为正整数。
   * @param[in] PixelPointer 外部连续像素地址；有效图像存活期间必须可访问。
   * @param[in] ClearProc void(*)(void*) 清理函数地址，参数为原像素指针；零表示不清理。
   *
   * @details 构造成功后才接管回调，最后一个共享像素引用退出时调用一次；失败不调用，回调不得抛异常。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  void GenImage1Extern(const char* Type, std::int64_t Width, std::int64_t Height, void* PixelPointer, void* ClearProc);

  /**
   * @brief 逐图像查询第一通道的像素类型名称。
   *
   * @return 查询结果控制元组。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  MvTuple GetImageType() const;

  /**
   * @brief 逐图像读取第一通道的借用地址及类型和尺寸。
   *
   * @param[out] Type 接收像素类型名称元组。
   * @param[out] Width 接收图像宽度元组。
   * @param[out] Height 接收图像高度元组。
   *
   * @return 借用指针地址整数元组；像素生命周期由图像决定。
   *
   * @throws MvException 参数非法或底层操作失败时抛出。
   */
  MvTuple GetImagePointer1(MvTuple* Type, MvTuple* Width, MvTuple* Height) const;

  /**
   * @brief 查询单张图像的通道数量。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。
   *
   * @return 图像通道数量。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::size_t Channels() const;

  /**
   * @brief 查询单张图像指定通道的行字节步长。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。
   *
   * @param[in] channel 从 0 开始的图像通道索引。
   *
   * @return 指定通道每行的字节步长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::size_t Stride(std::size_t channel = 0) const;

  /**
   * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
   *
   * @param[in] ImageRect2 第二张校正图像；调用期间借用，不转移所有权。
   * @param[out] Score 匹配或视差得分；接收结果的 C++ 对象，自动管理资源。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] MaskWidth 矩形窗口宽度。
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] TextureThresh 纹理阈值元组；调用期间借用，不转移所有权。
   * @param[in] MinDisparity 视差搜索下限。
   * @param[in] MaxDisparity 视差搜索上限。
   * @param[in] NumLevels 金字塔层级值。
   * @param[in] ScoreThresh 得分阈值元组；调用期间借用，不转移所有权。
   * @param[in] Filter 视差过滤方式元组；调用期间借用，不转移所有权。
   * @param[in] SubDisparity 亚像素细化方式；调用期间借用，不转移所有权。
   *
   * @return 视差结果，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_binocular_disparity
   */
  MvImage BinocularDisparity(const MvImage& ImageRect2, MvImage* Score, const MvString& Method,
                             std::int64_t MaskWidth, std::int64_t MaskHeight, const MvTuple& TextureThresh,
                             std::int64_t MinDisparity, std::int64_t MaxDisparity, std::int64_t NumLevels,
                             const MvTuple& ScoreThresh, const MvTuple& Filter,
                             const MvString& SubDisparity) const;

  /**
   * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
   * @overload 采用标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparity(const MvImage& ImageRect2, MvImage* Score, const MvString& Method,
                             std::int64_t MaskWidth, std::int64_t MaskHeight, double TextureThresh,
                             std::int64_t MinDisparity, std::int64_t MaxDisparity, std::int64_t NumLevels,
                             double ScoreThresh, const MvString& Filter, const MvString& SubDisparity) const;

  /**
   * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparity(const MvImage& ImageRect2, MvImage* Score, const char* Method,
                             std::int64_t MaskWidth, std::int64_t MaskHeight, double TextureThresh,
                             std::int64_t MinDisparity, std::int64_t MaxDisparity, std::int64_t NumLevels,
                             double ScoreThresh, const char* Filter, const char* SubDisparity) const;

  /**
   * @brief 采用 HALCON C 接口 2 顺序，每个接受的图像对产生一张 Disparity 和一张 Score。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparity(const MvImage& ImageRect2, MvImage* Score, const wchar_t* Method,
                             std::int64_t MaskWidth, std::int64_t MaskHeight, double TextureThresh,
                             std::int64_t MinDisparity, std::int64_t MaxDisparity, std::int64_t NumLevels,
                             double ScoreThresh, const wchar_t* Filter, const wchar_t* SubDisparity) const;

  /**
   * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
   *
   * @param[in] ImageRect2 第二张校正图像；调用期间借用，不转移所有权。
   * @param[out] Score 匹配或视差得分；接收结果的 C++ 对象，自动管理资源。
   * @param[in] MinDisparity 视差搜索下限。
   * @param[in] MaxDisparity 视差搜索上限。
   * @param[in] SurfaceSmoothing 表面平滑控制值。
   * @param[in] EdgeSmoothing 边缘平滑控制值。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 视差结果，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_binocular_disparity_ms
   */
  MvImage BinocularDisparityMs(const MvImage& ImageRect2, MvImage* Score, std::int64_t MinDisparity,
                               std::int64_t MaxDisparity, std::int64_t SurfaceSmoothing,
                               std::int64_t EdgeSmoothing, const MvTuple& GenParamName,
                               const MvTuple& GenParamValue) const;

  /**
   * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
   * @overload 采用标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparityMs(const MvImage& ImageRect2, MvImage* Score, std::int64_t MinDisparity,
                               std::int64_t MaxDisparity, std::int64_t SurfaceSmoothing,
                               std::int64_t EdgeSmoothing, const MvString& GenParamName,
                               const MvString& GenParamValue) const;

  /**
   * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparityMs(const MvImage& ImageRect2, MvImage* Score, std::int64_t MinDisparity,
                               std::int64_t MaxDisparity, std::int64_t SurfaceSmoothing,
                               std::int64_t EdgeSmoothing, const char* GenParamName,
                               const char* GenParamValue) const;

  /**
   * @brief SGM；控制元组参数仅在调用中借用，空视差界限表示自动选择。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 视差结果，由返回对象自动管理资源。
   */
  MvImage BinocularDisparityMs(const MvImage& ImageRect2, MvImage* Score, std::int64_t MinDisparity,
                               std::int64_t MaxDisparity, std::int64_t SurfaceSmoothing,
                               std::int64_t EdgeSmoothing, const wchar_t* GenParamName,
                               const wchar_t* GenParamValue) const;

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   *
   * @param[in] CamParam1 相机内参与畸变参数；调用期间借用，不转移所有权。
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
   * @return 第二台相机的校正映射图像；第一台相机的映射写入当前实例。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_binocular_rectification_map
   */
  MvImage GenBinocularRectificationMap(const MvCamPar& CamParam1, const MvCamPar& CamParam2,
                                       const MvPose& RelPose, double SubSampling, const MvString& Method,
                                       const MvString& MapType, MvCamPar* CamParamRect1,
                                       MvCamPar* CamParamRect2, MvPose* CamPoseRect1, MvPose* CamPoseRect2,
                                       MvPose* RelPoseRect);

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 第二台相机的校正映射图像；第一台相机的映射写入当前实例。
   */
  MvImage GenBinocularRectificationMap(const MvCamPar& CamParam1, const MvCamPar& CamParam2,
                                       const MvPose& RelPose, double SubSampling, const char* Method,
                                       const char* MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
                                       MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect);

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 第二台相机的校正映射图像；第一台相机的映射写入当前实例。
   */
  MvImage GenBinocularRectificationMap(const MvCamPar& CamParam1, const MvCamPar& CamParam2,
                                       const MvPose& RelPose, double SubSampling, const wchar_t* Method,
                                       const wchar_t* MapType, MvCamPar* CamParamRect1,
                                       MvCamPar* CamParamRect2, MvPose* CamPoseRect1, MvPose* CamPoseRect2,
                                       MvPose* RelPoseRect);

  /**
   * @brief real 单通道图像，保持对象数量及定义域，三个输出整体提交。
   *
   * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[in] CamParamRect1 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] CamParamRect2 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] RelPoseRect 设备间相对位姿；调用期间借用，不转移所有权。
   *
   * @return X 坐标，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_disparity_image_to_xyz
   */
  MvImage DisparityImageToXyz(MvImage* Y, MvImage* Z, const MvCamPar& CamParamRect1,
                              const MvCamPar& CamParamRect2, const MvPose& RelPoseRect) const;

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   *
   * @param[in] WorldX X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] WorldY Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] WorldZ Z 坐标元组；调用期间借用，不转移所有权。
   * @param[in] ImageRow 图像中的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ImageColumn 图像中的列坐标元组；调用期间借用，不转移所有权。
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
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
  static MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const MvString& Method, const MvTuple& QualityType, MvTuple* Quality);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用字符串对象及标量输出参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  static MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const MvString& Method, const MvString& QualityType, double* Quality);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  static MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const char* Method, const char* QualityType, double* Quality);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   */
  static MvPose VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const wchar_t* Method, const wchar_t* QualityType, double* Quality);

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   *
   * @param[in] CamParamIn 源相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] MapType 编码映射图像的存储类型；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_radial_distortion_map
   */
  void GenRadialDistortionMap(const MvCamPar& CamParamIn, const MvCamPar& CamParamOut,
                              const MvString& MapType);

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void GenRadialDistortionMap(const MvCamPar& CamParamIn, const MvCamPar& CamParamOut, const char* MapType);

  /**
   * @brief 生成径向畸变校正使用的编码映射图像。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void GenRadialDistortionMap(const MvCamPar& CamParamIn, const MvCamPar& CamParamOut,
                              const wchar_t* MapType);

  /**
   * @brief 按相机参数校正指定区域内的图像径向畸变。
   *
   * @param[in] Region 输入区域；调用期间借用，不转移所有权。
   * @param[in] CamParamIn 源相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_image
   */
  MvImage ChangeRadialDistortionImage(const MvRegion& Region, const MvCamPar& CamParamIn,
                                      const MvCamPar& CamParamOut) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   *
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] WorldPose 世界平面位姿；调用期间借用，不转移所有权。
   * @param[in] WidthIn 输入图像宽度。
   * @param[in] HeightIn 输入图像高度。
   * @param[in] WidthMapped 映射图像宽度。
   * @param[in] HeightMapped 映射图像高度。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   * @param[in] MapType 编码映射图像的存储类型；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_image_to_world_plane_map
   */
  void GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t WidthIn,
                               std::int64_t HeightIn, std::int64_t WidthMapped, std::int64_t HeightMapped,
                               const MvTuple& Scale, const MvString& MapType);

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t WidthIn,
                               std::int64_t HeightIn, std::int64_t WidthMapped, std::int64_t HeightMapped,
                               const MvString& Scale, const MvString& MapType);

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t WidthIn,
                               std::int64_t HeightIn, std::int64_t WidthMapped, std::int64_t HeightMapped,
                               const char* Scale, const char* MapType);

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t WidthIn,
                               std::int64_t HeightIn, std::int64_t WidthMapped, std::int64_t HeightMapped,
                               const wchar_t* Scale, const wchar_t* MapType);

  /**
   * @brief 将图像重采样到指定世界平面。
   *
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
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
  MvImage ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const MvTuple& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const MvString& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const char* Scale, const char* Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                            std::int64_t Height, const wchar_t* Scale, const wchar_t* Interpolation) const;




















    /**
   * @brief 将图像写入文件。
   *
   * @param[in] Format 图像或模型的文件格式；调用期间借用，不转移所有权。
   * @param[in] FillColor 图像定义域外的填充灰度元组；调用期间借用，不转移所有权。
   * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_image
   */
  void WriteImage(const MvString& Format, const MvTuple& FillColor, const MvTuple& FileName) const;

  /**
   * @brief 将图像写入文件。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void WriteImage(const MvString& Format, std::int64_t FillColor, const MvString& FileName) const;

  /**
   * @brief 将图像写入文件。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void WriteImage(const char* Format, std::int64_t FillColor, const char* FileName) const;

  /**
   * @brief 将图像写入文件。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void WriteImage(const wchar_t* Format, std::int64_t FillColor, const wchar_t* FileName) const;

  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   *
   * @param[in] HomMat2D 二维变换矩阵；调用期间借用，不转移所有权。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   * @param[in] AdaptImageSize 是否调整输出图像尺寸；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_image
   */
  MvImage AffineTransImage(const MvHomMat2D& HomMat2D, const MvString& Interpolation,
                           const MvString& AdaptImageSize) const;

  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AffineTransImage(const MvHomMat2D& HomMat2D, const char* Interpolation,
                           const char* AdaptImageSize) const;

  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AffineTransImage(const MvHomMat2D& HomMat2D, const wchar_t* Interpolation,
                           const wchar_t* AdaptImageSize) const;

  /**
   * @brief 一个 int4 线性索引 Map 或五通道 int4/uint2 权重 Map 决定输出尺寸和定义域。
   *
   * @param[in] Map 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_map_image
   */
  MvImage MapImage(const MvImage& Map) const;

  /**
   * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleEnd 终止角度。
   * @param[in] RadiusStart 极坐标采样起始半径元组；调用期间借用，不转移所有权。
   * @param[in] RadiusEnd 极坐标采样终止半径元组；调用期间借用，不转移所有权。
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_polar_trans_image_ext
   */
  MvImage PolarTransImageExt(const MvTuple& Row, const MvTuple& Column, double AngleStart, double AngleEnd,
                             const MvTuple& RadiusStart, const MvTuple& RadiusEnd, std::int64_t Width,
                             std::int64_t Height, const MvString& Interpolation) const;

  /**
   * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd,
                             double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height,
                             const MvString& Interpolation) const;

  /**
   * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd,
                             double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height,
                             const char* Interpolation) const;

  /**
   * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd,
                             double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height,
                             const wchar_t* Interpolation) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   *
   * @param[in] HomMat2D 二维变换矩阵；调用期间借用，不转移所有权。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   * @param[in] AdaptImageSize 是否调整输出图像尺寸；调用期间借用，不转移所有权。
   * @param[in] TransformDomain 是否变换图像定义域；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_projective_trans_image
   */
  MvImage ProjectiveTransImage(const MvHomMat2D& HomMat2D, const MvString& Interpolation,
                               const MvString& AdaptImageSize, const MvString& TransformDomain) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage ProjectiveTransImage(const MvHomMat2D& HomMat2D, const char* Interpolation,
                               const char* AdaptImageSize, const char* TransformDomain) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage ProjectiveTransImage(const MvHomMat2D& HomMat2D, const wchar_t* Interpolation,
                               const wchar_t* AdaptImageSize, const wchar_t* TransformDomain) const;

  /**
   * @brief HALCON HSV 约定：三个独立平面，保持类型；real hue 使用弧度。
   *
   * @param[in] ImageGreen 图标对象数组；调用期间借用，不转移所有权。
   * @param[in] ImageBlue 图标对象数组；调用期间借用，不转移所有权。
   * @param[out] ImageResult2 图像；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ImageResult3 图像；接收结果的 C++ 对象，自动管理资源。
   * @param[in] ColorSpace 颜色空间名称；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_trans_from_rgb
   */
  MvImage TransFromRgb(const MvImage& ImageGreen, const MvImage& ImageBlue, MvImage* ImageResult2,
                       MvImage* ImageResult3, const MvString& ColorSpace) const;

  /**
   * @brief HALCON HSV 约定：三个独立平面，保持类型；real hue 使用弧度。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage TransFromRgb(const MvImage& ImageGreen, const MvImage& ImageBlue, MvImage* ImageResult2,
                       MvImage* ImageResult3, const char* ColorSpace) const;

  /**
   * @brief HALCON HSV 约定：三个独立平面，保持类型；real hue 使用弧度。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage TransFromRgb(const MvImage& ImageGreen, const MvImage& ImageBlue, MvImage* ImageResult2,
                       MvImage* ImageResult3, const wchar_t* ColorSpace) const;

  /**
   * @brief 将指定颜色空间的三个图像通道转换为 RGB 通道。
   *
   * @param[in] ImageInput2 输入颜色通道图像；调用期间借用，不转移所有权。
   * @param[in] ImageInput3 输入颜色通道图像；调用期间借用，不转移所有权。
   * @param[out] ImageGreen 图像；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ImageBlue 图像；接收结果的 C++ 对象，自动管理资源。
   * @param[in] ColorSpace 颜色空间名称；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_trans_to_rgb
   */
  MvImage TransToRgb(const MvImage& ImageInput2, const MvImage& ImageInput3, MvImage* ImageGreen,
                     MvImage* ImageBlue, const MvString& ColorSpace) const;

  /**
   * @brief 将指定颜色空间的三个图像通道转换为 RGB 通道。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage TransToRgb(const MvImage& ImageInput2, const MvImage& ImageInput3, MvImage* ImageGreen,
                     MvImage* ImageBlue, const char* ColorSpace) const;

  /**
   * @brief 将指定颜色空间的三个图像通道转换为 RGB 通道。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage TransToRgb(const MvImage& ImageInput2, const MvImage& ImageInput3, MvImage* ImageGreen,
                     MvImage* ImageBlue, const wchar_t* ColorSpace) const;

  /**
   * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
   *
   * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
   * @param[in] Component 高斯导数分量；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_derivate_gauss
   */
  MvImage DerivateGauss(const MvTuple& Sigma, const MvString& Component) const;

  /**
   * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage DerivateGauss(double Sigma, const MvString& Component) const;

  /**
   * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage DerivateGauss(double Sigma, const char* Component) const;

  /**
   * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage DerivateGauss(double Sigma, const wchar_t* Component) const;

  /**
   * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
   *
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] LightDark 前景亮暗或线条极性；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_local_threshold
   */
  MvRegion LocalThreshold(const MvString& Method, const MvString& LightDark, const MvTuple& GenParamName,
                          const MvTuple& GenParamValue) const;

  /**
   * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion LocalThreshold(const MvString& Method, const MvString& LightDark, const MvString& GenParamName,
                          std::int64_t GenParamValue) const;

  /**
   * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion LocalThreshold(const char* Method, const char* LightDark, const char* GenParamName,
                          std::int64_t GenParamValue) const;

  /**
   * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion LocalThreshold(const wchar_t* Method, const wchar_t* LightDark, const wchar_t* GenParamName,
                          std::int64_t GenParamValue) const;

  /**
   * @brief 单通道图像定义域的 marker minimax 洪泛，每个 marker 一个盆地，按图像顺序输出。
   *
   * @param[in] Markers 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 分水岭区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_watersheds_marker
   */
  MvRegion WatershedsMarker(const MvRegion& Markers) const;

  /**
   * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
   *
   * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
   * @param[in] Percent 百分比控制元组；调用期间借用，不转移所有权。
   * @param[out] Min 筛选或测量范围界限元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Max 筛选或测量范围界限元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Range 灰度动态范围元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_min_max_gray
   */
  void MinMaxGray(const MvRegion& Regions, const MvTuple& Percent, MvTuple* Min, MvTuple* Max,
                  MvTuple* Range) const;

  /**
   * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void MinMaxGray(const MvRegion& Regions, double Percent, double* Min, double* Max, double* Range) const;

  /**
   * @brief 按轴对齐矩形裁剪图像。
   *
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_crop_rectangle1
   */
  MvImage CropRectangle1(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2,
                         const MvTuple& Column2) const;

  /**
   * @brief 按轴对齐矩形裁剪图像。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage CropRectangle1(std::int64_t Row1, std::int64_t Column1, std::int64_t Row2,
                         std::int64_t Column2) const;

  /**
   * @brief 镜像图像。
   *
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_mirror_image
   */
  MvImage MirrorImage(const MvString& Mode) const;

  /**
   * @brief 镜像图像。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage MirrorImage(const char* Mode) const;

  /**
   * @brief 镜像图像。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage MirrorImage(const wchar_t* Mode) const;

  /**
   * @brief 缩放图像到指定尺寸。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_zoom_image_size
   */
  MvImage ZoomImageSize(std::int64_t Width, std::int64_t Height, const MvString& Interpolation) const;

  /**
   * @brief 缩放图像到指定尺寸。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ZoomImageSize(std::int64_t Width, std::int64_t Height, const char* Interpolation) const;

  /**
   * @brief 缩放图像到指定尺寸。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ZoomImageSize(std::int64_t Width, std::int64_t Height, const wchar_t* Interpolation) const;

  /**
   * @brief 计算 Sobel 边缘幅值。
   *
   * @param[in] FilterType 边缘滤波器类型；调用期间借用，不转移所有权。
   * @param[in] Size 元素或键值项数量元组；调用期间借用，不转移所有权。
   *
   * @return 线条或边缘的响应幅值，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_sobel_amp
   */
  MvImage SobelAmp(const MvString& FilterType, const MvTuple& Size) const;

  /**
   * @brief 计算 Sobel 边缘幅值。
   * @overload 采用标量数值参数。
   *
   * @return 线条或边缘的响应幅值，由返回对象自动管理资源。
   */
  MvImage SobelAmp(const MvString& FilterType, std::int64_t Size) const;

  /**
   * @brief 计算 Sobel 边缘幅值。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 线条或边缘的响应幅值，由返回对象自动管理资源。
   */
  MvImage SobelAmp(const char* FilterType, std::int64_t Size) const;

  /**
   * @brief 计算 Sobel 边缘幅值。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 线条或边缘的响应幅值，由返回对象自动管理资源。
   */
  MvImage SobelAmp(const wchar_t* FilterType, std::int64_t Size) const;

  /**
   * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
   *
   * @param[in] ThresholdImage 图标对象数组；调用期间借用，不转移所有权。
   * @param[in] Offset 灰度或坐标偏移元组；调用期间借用，不转移所有权。
   * @param[in] LightDark 前景亮暗或线条极性；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_dyn_threshold
   */
  MvRegion DynThreshold(const MvImage& ThresholdImage, const MvTuple& Offset,
                        const MvString& LightDark) const;

  /**
   * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion DynThreshold(const MvImage& ThresholdImage, double Offset, const MvString& LightDark) const;

  /**
   * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion DynThreshold(const MvImage& ThresholdImage, double Offset, const char* LightDark) const;

  /**
   * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion DynThreshold(const MvImage& ThresholdImage, double Offset, const wchar_t* LightDark) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   *
   * @param[in] Region 输入区域；调用期间借用，不转移所有权。
   * @param[in] Grayval 灰度值元组；调用期间借用，不转移所有权。
   * @param[in] Type 数据、模型或原语类型；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_paint_region
   */
  MvImage PaintRegion(const MvRegion& Region, const MvTuple& Grayval, const MvString& Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvRegion& Region, double Grayval, const MvString& Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvRegion& Region, double Grayval, const char* Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvRegion& Region, double Grayval, const wchar_t* Type) const;

  /**
   * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
   *
   * @param[in] Filter 边缘滤波器类型；调用期间借用，不转移所有权。
   * @param[in] Alpha 边缘滤波参数。
   * @param[in] Low 滞后阈值的低阈值元组；调用期间借用，不转移所有权。
   * @param[in] High 滞后阈值的高阈值元组；调用期间借用，不转移所有权。
   *
   * @return 边缘轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_edges_sub_pix
   */
  MvXLDCont EdgesSubPix(const MvString& Filter, double Alpha, const MvTuple& Low, const MvTuple& High) const;

  /**
   * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
   * @overload 采用标量数值参数。
   *
   * @return 边缘轮廓，由返回对象自动管理资源。
   */
  MvXLDCont EdgesSubPix(const MvString& Filter, double Alpha, std::int64_t Low, std::int64_t High) const;

  /**
   * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 边缘轮廓，由返回对象自动管理资源。
   */
  MvXLDCont EdgesSubPix(const char* Filter, double Alpha, std::int64_t Low, std::int64_t High) const;

  /**
   * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 边缘轮廓，由返回对象自动管理资源。
   */
  MvXLDCont EdgesSubPix(const wchar_t* Filter, double Alpha, std::int64_t Low, std::int64_t High) const;

  /**
   * @brief 单通道图像数组灰度等值线，Threshold 为单个有限数值。
   *
   * @param[in] Threshold 边缘或灰度阈值元组；调用期间借用，不转移所有权。
   *
   * @return 边界处理方式，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_threshold_sub_pix
   */
  MvXLDCont ThresholdSubPix(const MvTuple& Threshold) const;

  /**
   * @brief 单通道图像数组灰度等值线，Threshold 为单个有限数值。
   * @overload 采用标量数值参数。
   *
   * @return 边界处理方式，由返回对象自动管理资源。
   */
  MvXLDCont ThresholdSubPix(double Threshold) const;

  /**
   * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
   *
   * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
   * @param[in] Low 滞后阈值的低阈值元组；调用期间借用，不转移所有权。
   * @param[in] High 滞后阈值的高阈值元组；调用期间借用，不转移所有权。
   * @param[in] LightDark 前景亮暗或线条极性；调用期间借用，不转移所有权。
   * @param[in] ExtractWidth 是否计算线条宽度；调用期间借用，不转移所有权。
   * @param[in] LineModel 线条类型；调用期间借用，不转移所有权。
   * @param[in] CompleteJunctions 是否补全线条交汇点；调用期间借用，不转移所有权。
   *
   * @return 中心线轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_lines_gauss
   */
  MvXLDCont LinesGauss(const MvTuple& Sigma, const MvTuple& Low, const MvTuple& High,
                       const MvString& LightDark, const MvString& ExtractWidth, const MvString& LineModel,
                       const MvString& CompleteJunctions) const;

  /**
   * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
   * @overload 采用标量数值参数。
   *
   * @return 中心线轮廓，由返回对象自动管理资源。
   */
  MvXLDCont LinesGauss(double Sigma, double Low, double High, const MvString& LightDark,
                       const MvString& ExtractWidth, const MvString& LineModel,
                       const MvString& CompleteJunctions) const;

  /**
   * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 中心线轮廓，由返回对象自动管理资源。
   */
  MvXLDCont LinesGauss(double Sigma, double Low, double High, const char* LightDark, const char* ExtractWidth,
                       const char* LineModel, const char* CompleteJunctions) const;

  /**
   * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 中心线轮廓，由返回对象自动管理资源。
   */
  MvXLDCont LinesGauss(double Sigma, double Low, double High, const wchar_t* LightDark,
                       const wchar_t* ExtractWidth, const wchar_t* LineModel,
                       const wchar_t* CompleteJunctions) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   *
   * @param[in] ModelIDs 包含形状匹配模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
   * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
   * @param[in] ScaleMin 模型尺度下限元组；调用期间借用，不转移所有权。
   * @param[in] ScaleMax 模型尺度上限元组；调用期间借用，不转移所有权。
   * @param[in] MinScore 匹配得分下限元组；调用期间借用，不转移所有权。
   * @param[in] NumMatches 返回匹配数量的控制元组；调用期间借用，不转移所有权。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间借用，不转移所有权。
   * @param[in] SubPixel 亚像素细化方式元组；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] Greediness 搜索贪婪度元组；调用期间借用，不转移所有权。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Scale 坐标、灰度或模型的缩放控制元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Model 模型句柄元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_scaled_shape_models
   */
  void FindScaledShapeModels(const MvShapeModelArray& ModelIDs, const MvTuple& AngleStart,
                             const MvTuple& AngleExtent, const MvTuple& ScaleMin, const MvTuple& ScaleMax,
                             const MvTuple& MinScore, const MvTuple& NumMatches, const MvTuple& MaxOverlap,
                             const MvTuple& SubPixel, const MvTuple& NumLevels, const MvTuple& Greediness,
                             MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score,
                             MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent,
                             double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                             double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels,
                             double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                             MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent,
                             double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                             double MaxOverlap, const char* SubPixel, std::int64_t NumLevels,
                             double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                             MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent,
                             double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                             double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels,
                             double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                             MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   *
   * @param[in] ModelIDs 包含形状匹配模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
   * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
   * @param[in] MinScore 匹配得分下限元组；调用期间借用，不转移所有权。
   * @param[in] NumMatches 返回匹配数量的控制元组；调用期间借用，不转移所有权。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间借用，不转移所有权。
   * @param[in] SubPixel 亚像素细化方式元组；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] Greediness 搜索贪婪度元组；调用期间借用，不转移所有权。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Model 模型句柄元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_shape_models
   */
  void FindShapeModels(const MvShapeModelArray& ModelIDs, const MvTuple& AngleStart,
                       const MvTuple& AngleExtent, const MvTuple& MinScore, const MvTuple& NumMatches,
                       const MvTuple& MaxOverlap, const MvTuple& SubPixel, const MvTuple& NumLevels,
                       const MvTuple& Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle,
                       MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent, double MinScore,
                       std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel,
                       std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                       MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent, double MinScore,
                       std::int64_t NumMatches, double MaxOverlap, const char* SubPixel,
                       std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                       MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvShapeModel& ModelIDs, double AngleStart, double AngleExtent, double MinScore,
                       std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel,
                       std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                       MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   *
   * @param[in] ModelID 包含形状匹配模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] ScaleMin 模型尺度下限。
   * @param[in] ScaleMax 模型尺度上限。
   * @param[in] MinScore 匹配得分下限元组；调用期间借用，不转移所有权。
   * @param[in] NumMatches 返回匹配数量的值。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例。
   * @param[in] SubPixel 亚像素细化方式元组；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] Greediness 搜索贪婪度。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Scale 坐标、灰度或模型的缩放控制元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_scaled_shape_model
   */
  void FindScaledShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent,
                            double ScaleMin, double ScaleMax, const MvTuple& MinScore,
                            std::int64_t NumMatches, double MaxOverlap, const MvTuple& SubPixel,
                            const MvTuple& NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                            MvTuple* Angle, MvTuple* Scale, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent,
                            double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                            double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels,
                            double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                            MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent,
                            double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                            double MaxOverlap, const char* SubPixel, std::int64_t NumLevels,
                            double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                            MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent,
                            double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches,
                            double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels,
                            double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale,
                            MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   *
   * @param[in] ModelID 包含形状匹配模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] MinScore 匹配得分下限元组；调用期间借用，不转移所有权。
   * @param[in] NumMatches 返回匹配数量的值。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例。
   * @param[in] SubPixel 亚像素细化方式元组；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] Greediness 搜索贪婪度。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_shape_model
   */
  void FindShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent,
                      const MvTuple& MinScore, std::int64_t NumMatches, double MaxOverlap,
                      const MvTuple& SubPixel, const MvTuple& NumLevels, double Greediness, MvTuple* Row,
                      MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                      std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel,
                      std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                      MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                      std::int64_t NumMatches, double MaxOverlap, const char* SubPixel,
                      std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                      MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvShapeModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                      std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel,
                      std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column,
                      MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   *
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] ScaleMin 模型尺度下限。
   * @param[in] ScaleMax 模型尺度上限。
   * @param[in] ScaleStep 模型尺度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] Contrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_scaled_shape_model
   */
  MvShapeModel CreateScaledShapeModel(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                                      const MvTuple& AngleStep, double ScaleMin, double ScaleMax,
                                      const MvTuple& ScaleStep, const MvTuple& Optimization,
                                      const MvString& Metric, const MvTuple& Contrast,
                                      const MvTuple& MinContrast) const;

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                      double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                      const MvString& Optimization, const MvString& Metric,
                                      std::int64_t Contrast, std::int64_t MinContrast) const;

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                      double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                      const char* Optimization, const char* Metric, std::int64_t Contrast,
                                      std::int64_t MinContrast) const;

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                      double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                      const wchar_t* Optimization, const wchar_t* Metric,
                                      std::int64_t Contrast, std::int64_t MinContrast) const;

  /**
   * @brief 根据模板图像创建形状匹配模型。
   *
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] Contrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_shape_model
   */
  MvShapeModel CreateShapeModel(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                                const MvTuple& AngleStep, const MvTuple& Optimization, const MvString& Metric,
                                const MvTuple& Contrast, const MvTuple& MinContrast) const;

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                double AngleStep, const MvString& Optimization, const MvString& Metric,
                                std::int64_t Contrast, std::int64_t MinContrast) const;

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                double AngleStep, const char* Optimization, const char* Metric,
                                std::int64_t Contrast, std::int64_t MinContrast) const;

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModel(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                double AngleStep, const wchar_t* Optimization, const wchar_t* Metric,
                                std::int64_t Contrast, std::int64_t MinContrast) const;

  /**
   * @brief 设置形状匹配模型的指定参数。
   *
   * @param[in] ModelID 包含形状匹配模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_shape_model_param
   */
  static void SetShapeModelParam(const MvShapeModel& ModelID, const MvTuple& GenParamName,
                                 const MvTuple& GenParamValue);

  /**
   * @brief 构造有效的空图标对象数组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvImage();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvImage(const MvImage&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvImage(MvImage&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvImage& operator=(const MvImage&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvImage& operator=(MvImage&&);

  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvImage(const MvObject& objects);

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   *
   * @param[in] index 从 1 开始的图标对象索引；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_obj
   */
  MvImage SelectObj(const MvTuple& index) const;

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage SelectObj(std::int64_t index) const;

  /**
   * @brief 复制指定范围的图标对象引用并共享其载荷。
   *
   * @param[in] index 从 1 开始的图标对象索引。
   * @param[in] count 需要复制的图标对象数量。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_copy_obj
   */
  MvImage CopyObj(std::int64_t index, std::int64_t count) const;

  /**
   * @brief 按顺序拼接两个图标对象数组。
   *
   * @param[in] other 参与操作的另一图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_concat_obj
   */
  MvImage ConcatObj(const MvImage& other) const;

  /**
   * @brief 逐对象获取 0..N 图像对象数组 的当前定义域。
   *
   * @return 图像定义域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_domain
   */
  MvRegion GetDomain() const;

  /**
   * @brief 使用第一个 Region 收缩各 Image 的定义域，与 HALCON 的 Region 输入语义一致。
   *
   * @param[in] region 输入区域；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_reduce_domain
   */
  MvImage ReduceDomain(const MvRegion& region) const;

  /**
   * @brief 逐对象恢复 0..N 图像对象数组 的完整定义域，并保持顺序。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_full_domain
   */
  MvImage FullDomain() const;

  /**
   * @brief 无符号为 max-value，有符号/浮点为 -value（有符号最小值饱和）。保持定义域，域外置零。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_invert_image
   */
  MvImage InvertImage() const;

  /**
   * @brief 将 RGB/BGR 图像转换为灰度图。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_rgb1_to_gray
   */
  MvImage Rgb1ToGray() const;

  /**
   * @brief 裁剪到定义域包围盒并平移到新原点；空定义域报错，零对象保持为空。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_crop_domain
   */
  MvImage CropDomain() const;

  /**
   * @brief 逐对象逐通道转换到指定像素类型，保持定义域及对象顺序。
   *
   * @param[in] type 输出像素类型；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_convert_image_type
   */
  MvImage ConvertImageType(const MvString& type) const;

  /**
   * @brief 逐对象逐通道转换到指定像素类型，保持定义域及对象顺序。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ConvertImageType(const char* type) const;

  /**
   * @brief 按 从 1 开始的通道索引 获取单个图像的单通道图像，保持定义域。
   *
   * @param[in] channel 通道索引。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_access_channel
   */
  MvImage AccessChannel(std::int64_t channel) const;

  /**
   * @brief 三个等数量单通道 图像对象数组 合成三通道对象。
   *
   * @param[in] second 输入图像；调用期间借用，不转移所有权。
   * @param[in] third 输入图像；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_compose3
   */
  MvImage Compose3(const MvImage& second, const MvImage& third) const;

  /**
   * @brief 逐对象拆分恰好三个通道的 图像对象数组，各结果共享像素并保留定义域。
   *
   * @param[out] second 图像；接收结果的 C++ 对象，自动管理资源。
   * @param[out] third 图像；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_decompose3
   */
  MvImage Decompose3(MvImage* second, MvImage* third) const;

  /**
   * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
   *
   * @param[in] region 输入区域；调用期间借用，不转移所有权。
   * @param[out] deviation 灰度均值或标准差元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 灰度均值或标准差元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_intensity
   */
  MvTuple Intensity(const MvRegion& region, MvTuple* deviation) const;

  /**
   * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
   * @overload 采用标量输出参数。
   *
   * @return 灰度均值或标准差。
   */
  double Intensity(const MvRegion& region, double* deviation) const;

  /**
   * @brief 单个 Region、单个单通道图像 的 256-bin 直方图，支持五种像素类型。
   *
   * @param[in] region 输入区域；调用期间借用，不转移所有权。
   * @param[out] relative 归一化灰度直方图元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 各灰度桶的像素计数元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_histo
   */
  MvTuple GrayHisto(const MvRegion& region, MvTuple* relative) const;

  /**
   * @brief 逐对象执行均值滤波并保持 图像对象数组 顺序。
   *
   * @param[in] MaskWidth 矩形窗口宽度。
   * @param[in] MaskHeight 矩形窗口高度。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_mean_image
   */
  MvImage MeanImage(std::int64_t MaskWidth, std::int64_t MaskHeight) const;

  /**
   * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
   *
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] LightDark 前景亮暗或线条极性；调用期间借用，不转移所有权。
   * @param[out] UsedThreshold 自动选择的灰度阈值元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_binary_threshold
   */
  MvRegion BinaryThreshold(const MvString& Method, const MvString& LightDark, MvTuple* UsedThreshold) const;

  /**
   * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
   * @overload 采用字符串对象及标量输出参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion BinaryThreshold(const MvString& Method, const MvString& LightDark,
                           std::int64_t* UsedThreshold) const;

  /**
   * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion BinaryThreshold(const char* Method, const char* LightDark, std::int64_t* UsedThreshold) const;

  /**
   * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion BinaryThreshold(const wchar_t* Method, const wchar_t* LightDark,
                           std::int64_t* UsedThreshold) const;

  /**
   * @brief 单通道 UInt8/UInt16 直方图谷值分割；Gaussian 直方图 sigma 在 [0,32] 内，0 禁用平滑。
   *
   * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_auto_threshold
   */
  MvRegion AutoThreshold(const MvTuple& Sigma) const;

  /**
   * @brief 单通道 UInt8/UInt16 直方图谷值分割；Gaussian 直方图 sigma 在 [0,32] 内，0 禁用平滑。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion AutoThreshold(double Sigma) const;

  /**
   * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
   *
   * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_rotate_image
   */
  MvImage RotateImage(const MvTuple& Phi, const MvString& Interpolation) const;

  /**
   * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage RotateImage(double Phi, const MvString& Interpolation) const;

  /**
   * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage RotateImage(double Phi, const char* Interpolation) const;

  /**
   * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage RotateImage(double Phi, const wchar_t* Interpolation) const;

  /**
   * @brief 逐对象执行高斯滤波并保持 图像对象数组 顺序。
   *
   * @param[in] Size 元素或键值项数量。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gauss_filter
   */
  MvImage GaussFilter(std::int64_t Size) const;

  /**
   * @brief 逐对象执行中值滤波并保持 图像对象数组 顺序。
   *
   * @param[in] MaskType 中值滤波窗口形状；调用期间借用，不转移所有权。
   * @param[in] Radius 半径。
   * @param[in] Margin 边缘或边界处理模式元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_median_image
   */
  MvImage MedianImage(const MvString& MaskType, std::int64_t Radius, const MvTuple& Margin) const;

  /**
   * @brief 逐对象执行中值滤波并保持 图像对象数组 顺序。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage MedianImage(const MvString& MaskType, std::int64_t Radius, const MvString& Margin) const;

  /**
   * @brief 逐对象执行中值滤波并保持 图像对象数组 顺序。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage MedianImage(const char* MaskType, std::int64_t Radius, const char* Margin) const;

  /**
   * @brief 奇数宽高 [1,4095] 窗口内的灰度最小值，continued 边界；支持五种类型及混合平面，输入必须有限。
   *
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] MaskWidth 矩形窗口宽度。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_erosion_rect
   */
  MvImage GrayErosionRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const;

  /**
   * @brief 奇数宽高 [1,4095] 窗口内的灰度最大值，continued 边界；支持五种类型及混合平面，输入必须有限。
   *
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] MaskWidth 矩形窗口宽度。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_dilation_rect
   */
  MvImage GrayDilationRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const;

  /**
   * @brief 奇数矩形窗口 [1,4095] 的灰度腐蚀后膨胀；continued 边界，两阶段之间保留完整栅格 halo。
   *
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] MaskWidth 矩形窗口宽度。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_opening_rect
   */
  MvImage GrayOpeningRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const;

  /**
   * @brief 奇数矩形窗口 [1,4095] 的灰度膨胀后腐蚀；continued 边界，两阶段之间保留完整栅格 halo。
   *
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] MaskWidth 矩形窗口宽度。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_closing_rect
   */
  MvImage GrayClosingRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const;

  /**
   * @brief Image + factor*(Image-local_mean)；有限 factor >=0，奇数矩形宽高 [1,4095]，mirrored 边界，保持类型/定义域。
   *
   * @param[in] MaskWidth 矩形窗口宽度。
   * @param[in] MaskHeight 矩形窗口高度。
   * @param[in] Factor 强调或缩放因子。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_emphasize
   */
  MvImage Emphasize(std::int64_t MaskWidth, std::int64_t MaskHeight, double Factor) const;

  /**
   * @brief 量化 Laplacian-of-Gaussian，sigma 在 [0.1,32] 内，mirrored 边界，逐通道输出 Int16。
   *
   * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_laplace_of_gauss
   */
  MvImage LaplaceOfGauss(const MvTuple& Sigma) const;

  /**
   * @brief 量化 Laplacian-of-Gaussian，sigma 在 [0.1,32] 内，mirrored 边界，逐通道输出 Int16。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage LaplaceOfGauss(double Sigma) const;

          /**
   * @brief 单通道 UInt8/UInt16 定义域内 CDF-min 均衡，分别使用 256/65536 个桶。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_equ_histo_image
   */
  MvImage EquHistoImage() const;

  /**
   * @brief 执行线性灰度缩放。
   *
   * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_scale_image
   */
  MvImage ScaleImage(const MvTuple& Mult, const MvTuple& Add) const;

  /**
   * @brief 执行线性灰度缩放。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ScaleImage(double Mult, double Add) const;

  /**
   * @brief 对等数量的 Image 对象逐对求和；控制参数各一个有限数值。
   *
   * @param[in] image2 输入图像；调用期间借用，不转移所有权。
   * @param[in] mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   * @param[in] add 灰度加法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_add_image
   */
  MvImage AddImage(const MvImage& image2, const MvTuple& mult, const MvTuple& add) const;

  /**
   * @brief 对等数量的 Image 对象逐对求和；控制参数各一个有限数值。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AddImage(const MvImage& image2, double mult, double add) const;

  /**
   * @brief abs(a-b)*mult；平面类型一致，定义域取交集，对象按 scalar-or-N 配对。
   *
   * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
   * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_abs_diff_image
   */
  MvImage AbsDiffImage(const MvImage& Image2, const MvTuple& Mult) const;

  /**
   * @brief abs(a-b)*mult；平面类型一致，定义域取交集，对象按 scalar-or-N 配对。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AbsDiffImage(const MvImage& Image2, double Mult) const;

  /**
   * @brief 按 scalar-or-N 规则执行双 图像对象数组 减法。
   *
   * @param[in] ImageSubtrahend 减数图像；调用期间借用，不转移所有权。
   * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_sub_image
   */
  MvImage SubImage(const MvImage& ImageSubtrahend, const MvTuple& Mult, const MvTuple& Add) const;

  /**
   * @brief 按 scalar-or-N 规则执行双 图像对象数组 减法。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage SubImage(const MvImage& ImageSubtrahend, double Mult, double Add) const;

  /**
   * @brief 按 scalar-or-N 规则执行双 图像对象数组 乘法。
   *
   * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
   * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_mult_image
   */
  MvImage MultImage(const MvImage& Image2, const MvTuple& Mult, const MvTuple& Add) const;

  /**
   * @brief 按 scalar-or-N 规则执行双 图像对象数组 乘法。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage MultImage(const MvImage& Image2, double Mult, double Add) const;

  /**
   * @brief (a/b)*mult+add；除数为零时结果为零，不受 add 影响。类型/定义域规则同 abs_diff_image。
   *
   * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
   * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
   * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_div_image
   */
  MvImage DivImage(const MvImage& Image2, const MvTuple& Mult, const MvTuple& Add) const;

  /**
   * @brief (a/b)*mult+add；除数为零时结果为零，不受 add 影响。类型/定义域规则同 abs_diff_image。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage DivImage(const MvImage& Image2, double Mult, double Add) const;

  /**
   * @brief 逐 Image object 按灰度范围生成 Region object。
   *
   * @param[in] minimum 灰度下限元组；调用期间借用，不转移所有权。
   * @param[in] maximum 灰度上限元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_threshold
   */
  MvRegion Threshold(const MvTuple& minimum, const MvTuple& maximum) const;

  /**
   * @brief 逐 Image object 按灰度范围生成 Region object。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion Threshold(double minimum, double maximum) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   *
   * @param[in] ModelID 包含NCC模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] MinScore 匹配得分下限。
   * @param[in] NumMatches 返回匹配数量的值。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例。
   * @param[in] SubPixel 亚像素细化方式；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_ncc_model
   */
  void FindNccModel(const MvNCCModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                    std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel,
                    const MvTuple& NumLevels, MvTuple* Row, MvTuple* Column, MvTuple* Angle,
                    MvTuple* Score) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModel(const MvNCCModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                    std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel,
                    std::int64_t NumLevels, MvTuple* Row, MvTuple* Column, MvTuple* Angle,
                    MvTuple* Score) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModel(const MvNCCModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                    std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels,
                    MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   *
   * @param[in] ModelIDs 包含NCC模型句柄的值；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
   * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
   * @param[in] MinScore 匹配得分下限元组；调用期间借用，不转移所有权。
   * @param[in] NumMatches 返回匹配数量的控制元组；调用期间借用，不转移所有权。
   * @param[in] MaxOverlap 允许的最大匹配重叠比例元组；调用期间借用，不转移所有权。
   * @param[in] SubPixel 亚像素细化方式元组；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Score 匹配或视差得分元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Model 模型句柄元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_find_ncc_models
   */
  void FindNccModels(const MvNCCModelArray& ModelIDs, const MvTuple& AngleStart, const MvTuple& AngleExtent,
                     const MvTuple& MinScore, const MvTuple& NumMatches, const MvTuple& MaxOverlap,
                     const MvTuple& SubPixel, const MvTuple& NumLevels, MvTuple* Row, MvTuple* Column,
                     MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModels(const MvNCCModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                     std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel,
                     std::int64_t NumLevels, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score,
                     MvTuple* Model) const;

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModels(const MvNCCModel& ModelID, double AngleStart, double AngleExtent, double MinScore,
                     std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels,
                     MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   *
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   *
   * @return NCC模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_ncc_model
   */
  MvNCCModel CreateNccModel(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                            const MvTuple& AngleStep, const MvString& Metric) const;

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return NCC模型句柄，由返回对象自动管理资源。
   */
  MvNCCModel CreateNccModel(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                            const MvTuple& AngleStep, const char* Metric) const;

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用标量数值参数。
   *
   * @return NCC模型句柄，由返回对象自动管理资源。
   */
  MvNCCModel CreateNccModel(std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep,
                            const MvString& Metric) const;

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return NCC模型句柄，由返回对象自动管理资源。
   */
  MvNCCModel CreateNccModel(std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep,
                            const char* Metric) const;

  /**
   * @brief 从指定文件读取图像对象数组。
   *
   * @param[in] filenames 文件路径元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_image
   */
  void ReadImage(const MvTuple& filenames);

  /**
   * @brief 从指定文件读取图像对象数组。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadImage(const char* filename);

  /**
   * @brief 查询单张图像的像素宽度。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。
   *
   * @return 图像宽度，单位为像素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::size_t Width() const;

  /**
   * @brief 查询单张图像的像素高度。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。
   *
   * @return 图像高度，单位为像素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::size_t Height() const;

  /**
   * @brief 查询指定图像通道的像素类型。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。
   *
   * @param[in] channel 从 0 开始的图像通道索引。
   *
   * @return 指定通道的像素类型。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPixelType Type(std::size_t channel = 0) const;

  /**
   * @brief 借用单张图像指定通道的只读像素数据。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。使用期间须保留图像引用；可写访问引发的存储分离可能使先前的借用指针失效。
   *
   * @param[in] channel 从 0 开始的图像通道索引。
   *
   * @return 只读像素借用指针。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  const void* Data(std::size_t channel = 0) const;

  /**
   * @brief 借用单张图像指定通道的可写像素数据。
   *
   * @details 要求当前对象恰好包含一张图像；通道索引从 0 开始。使用期间须保留图像引用；可写访问引发的存储分离可能使先前的借用指针失效。
   *
   * @param[in] channel 从 0 开始的图像通道索引。
   *
   * @return 可写像素借用指针。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  void* MutableData(std::size_t channel = 0);

};

/**
 * @brief 对等数量的 Image 对象逐对求和；控制参数各一个有限数值。
 *
 * @param[in] image1 输入图像；调用期间借用，不转移所有权。
 * @param[in] image2 输入图像；调用期间借用，不转移所有权。
 * @param[out] result 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 * @param[in] add 灰度加法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_add_image
 */
MVKIT_CPP_API void AddImage(const MvObject& image1, const MvObject& image2, MvObject* result,
                            const MvTuple& mult, const MvTuple& add);

/**
 * @brief abs(a-b)*mult；平面类型一致，定义域取交集，对象按 scalar-or-N 配对。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageAbsDiff 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_abs_diff_image
 */
MVKIT_CPP_API void AbsDiffImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageAbsDiff,
                                const MvTuple& Mult);

/**
 * @brief 按 scalar-or-N 规则执行双 图像对象数组 减法。
 *
 * @param[in] ImageMinuend 被减图像；调用期间借用，不转移所有权。
 * @param[in] ImageSubtrahend 减数图像；调用期间借用，不转移所有权。
 * @param[out] ImageSub 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_sub_image
 */
MVKIT_CPP_API void SubImage(const MvObject& ImageMinuend, const MvObject& ImageSubtrahend, MvObject* ImageSub,
                            const MvTuple& Mult, const MvTuple& Add);

/**
 * @brief 按 scalar-or-N 规则执行双 图像对象数组 乘法。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageResult 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_mult_image
 */
MVKIT_CPP_API void MultImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageResult,
                             const MvTuple& Mult, const MvTuple& Add);

/**
 * @brief (a/b)*mult+add；除数为零时结果为零，不受 add 影响。类型/定义域规则同 abs_diff_image。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageResult 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_div_image
 */
MVKIT_CPP_API void DivImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageResult,
                            const MvTuple& Mult, const MvTuple& Add);

/**
 * @brief 逐 Image object 按灰度范围生成 Region object。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] minimum 灰度下限元组；调用期间借用，不转移所有权。
 * @param[in] maximum 灰度上限元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_threshold
 */
MVKIT_CPP_API void Threshold(const MvObject& image, MvObject* region, const MvTuple& minimum,
                             const MvTuple& maximum);

/**
 * @brief 从指定文件读取图像对象数组。
 *
 * @param[out] image 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] filenames 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_image
 */
MVKIT_CPP_API void ReadImage(MvObject* image, const MvTuple& filenames);

/**
 * @brief 按轴对齐矩形裁剪图像。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImagePart 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_crop_rectangle1
 */
MVKIT_CPP_API void CropRectangle1(const MvObject& Image, MvObject* ImagePart, const MvTuple& Row1,
                                  const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2);

/**
 * @brief 镜像图像。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageMirror 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_mirror_image
 */
MVKIT_CPP_API void MirrorImage(const MvObject& Image, MvObject* ImageMirror, const MvTuple& Mode);

/**
 * @brief 缩放图像到指定尺寸。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageZoom 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_zoom_image_size
 */
MVKIT_CPP_API void ZoomImageSize(const MvObject& Image, MvObject* ImageZoom, const MvTuple& Width,
                                 const MvTuple& Height, const MvTuple& Interpolation);

/**
 * @brief 计算 Sobel 边缘幅值。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] EdgeAmplitude 线条或边缘的响应幅值；接收结果的 C++ 对象，自动管理资源。
 * @param[in] FilterType 边缘滤波器类型元组；调用期间借用，不转移所有权。
 * @param[in] Size 元素或键值项数量元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_sobel_amp
 */
MVKIT_CPP_API void SobelAmp(const MvObject& Image, MvObject* EdgeAmplitude, const MvTuple& FilterType,
                            const MvTuple& Size);

/**
 * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
 *
 * @param[in] OrigImage 原始图像；调用期间借用，不转移所有权。
 * @param[in] ThresholdImage 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionDynThresh 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Offset 灰度或坐标偏移元组；调用期间借用，不转移所有权。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_dyn_threshold
 */
MVKIT_CPP_API void DynThreshold(const MvObject& OrigImage, const MvObject& ThresholdImage,
                                MvObject* RegionDynThresh, const MvTuple& Offset, const MvTuple& LightDark);

/**
 * @brief 逐对象获取 0..N 图像对象数组 的当前定义域。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] domain 图像定义域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_domain
 */
MVKIT_CPP_API void GetDomain(const MvObject& image, MvObject* domain);

/**
 * @brief 使用第一个 Region 收缩各 Image 的定义域，与 HALCON 的 Region 输入语义一致。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[in] region 输入区域；调用期间借用，不转移所有权。
 * @param[out] reduced 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_reduce_domain
 */
MVKIT_CPP_API void ReduceDomain(const MvObject& image, const MvObject& region, MvObject* reduced);

/**
 * @brief 逐对象恢复 0..N 图像对象数组 的完整定义域，并保持顺序。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] full 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_full_domain
 */
MVKIT_CPP_API void FullDomain(const MvObject& image, MvObject* full);

/**
 * @brief 无符号为 max-value，有符号/浮点为 -value（有符号最小值饱和）。保持定义域，域外置零。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] inverted 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_invert_image
 */
MVKIT_CPP_API void InvertImage(const MvObject& image, MvObject* inverted);

/**
 * @brief 将 RGB/BGR 图像转换为灰度图。
 *
 * @param[in] image 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] gray 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_rgb1_to_gray
 */
MVKIT_CPP_API void Rgb1ToGray(const MvObject& image, MvObject* gray);

/**
 * @brief 裁剪到定义域包围盒并平移到新原点；空定义域报错，零对象保持为空。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] cropped 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_crop_domain
 */
MVKIT_CPP_API void CropDomain(const MvObject& image, MvObject* cropped);

/**
 * @brief 逐对象逐通道转换到指定像素类型，保持定义域及对象顺序。
 *
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] converted 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] type 输出像素类型元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_convert_image_type
 */
MVKIT_CPP_API void ConvertImageType(const MvObject& image, MvObject* converted, const MvTuple& type);

/**
 * @brief 按 从 1 开始的通道索引 获取单个图像的单通道图像，保持定义域。
 *
 * @param[in] image 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] channelImage 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] channel 通道索引元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_access_channel
 */
MVKIT_CPP_API void AccessChannel(const MvObject& image, MvObject* channelImage, const MvTuple& channel);

/**
 * @brief 三个等数量单通道 图像对象数组 合成三通道对象。
 *
 * @param[in] first 输入图像；调用期间借用，不转移所有权。
 * @param[in] second 输入图像；调用期间借用，不转移所有权。
 * @param[in] third 输入图像；调用期间借用，不转移所有权。
 * @param[out] composed 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_compose3
 */
MVKIT_CPP_API void Compose3(const MvObject& first, const MvObject& second, const MvObject& third,
                            MvObject* composed);

/**
 * @brief 逐对象拆分恰好三个通道的 图像对象数组，各结果共享像素并保留定义域。
 *
 * @param[in] image 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] first 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] second 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] third 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_decompose3
 */
MVKIT_CPP_API void Decompose3(const MvObject& image, MvObject* first, MvObject* second, MvObject* third);

/**
 * @brief 逐对象执行均值滤波并保持 图像对象数组 顺序。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageMean 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_mean_image
 */
MVKIT_CPP_API void MeanImage(const MvObject& Image, MvObject* ImageMean, const MvTuple& MaskWidth,
                             const MvTuple& MaskHeight);

/**
 * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间借用，不转移所有权。
 * @param[out] UsedThreshold 自动选择的灰度阈值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_binary_threshold
 */
MVKIT_CPP_API void BinaryThreshold(const MvObject& Image, MvObject* Region, const MvTuple& Method,
                                   const MvTuple& LightDark, MvTuple* UsedThreshold);

/**
 * @brief 单通道 UInt8/UInt16 直方图谷值分割；Gaussian 直方图 sigma 在 [0,32] 内，0 禁用平滑。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Regions 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_auto_threshold
 */
MVKIT_CPP_API void AutoThreshold(const MvObject& Image, MvObject* Regions, const MvTuple& Sigma);

/**
 * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageRotate 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_rotate_image
 */
MVKIT_CPP_API void RotateImage(const MvObject& Image, MvObject* ImageRotate, const MvTuple& Phi,
                               const MvTuple& Interpolation);

/**
 * @brief 逐对象执行高斯滤波并保持 图像对象数组 顺序。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageGauss 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Size 元素或键值项数量元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gauss_filter
 */
MVKIT_CPP_API void GaussFilter(const MvObject& Image, MvObject* ImageGauss, const MvTuple& Size);

/**
 * @brief 逐对象执行中值滤波并保持 图像对象数组 顺序。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageMedian 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskType 中值滤波窗口形状元组；调用期间借用，不转移所有权。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 * @param[in] Margin 边缘或边界处理模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_median_image
 */
MVKIT_CPP_API void MedianImage(const MvObject& Image, MvObject* ImageMedian, const MvTuple& MaskType,
                               const MvTuple& Radius, const MvTuple& Margin);

/**
 * @brief 奇数宽高 [1,4095] 窗口内的灰度最小值，continued 边界；支持五种类型及混合平面，输入必须有限。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageMin 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gray_erosion_rect
 */
MVKIT_CPP_API void GrayErosionRect(const MvObject& Image, MvObject* ImageMin, const MvTuple& MaskHeight,
                                   const MvTuple& MaskWidth);

/**
 * @brief 奇数宽高 [1,4095] 窗口内的灰度最大值，continued 边界；支持五种类型及混合平面，输入必须有限。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageMax 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gray_dilation_rect
 */
MVKIT_CPP_API void GrayDilationRect(const MvObject& Image, MvObject* ImageMax, const MvTuple& MaskHeight,
                                    const MvTuple& MaskWidth);

/**
 * @brief 奇数矩形窗口 [1,4095] 的灰度腐蚀后膨胀；continued 边界，两阶段之间保留完整栅格 halo。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageOpening 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gray_opening_rect
 */
MVKIT_CPP_API void GrayOpeningRect(const MvObject& Image, MvObject* ImageOpening, const MvTuple& MaskHeight,
                                   const MvTuple& MaskWidth);

/**
 * @brief 奇数矩形窗口 [1,4095] 的灰度膨胀后腐蚀；continued 边界，两阶段之间保留完整栅格 halo。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageClosing 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gray_closing_rect
 */
MVKIT_CPP_API void GrayClosingRect(const MvObject& Image, MvObject* ImageClosing, const MvTuple& MaskHeight,
                                   const MvTuple& MaskWidth);

/**
 * @brief Image + factor*(Image-local_mean)；有限 factor >=0，奇数矩形宽高 [1,4095]，mirrored 边界，保持类型/定义域。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageEmphasize 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间借用，不转移所有权。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间借用，不转移所有权。
 * @param[in] Factor 强调或缩放因子元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_emphasize
 */
MVKIT_CPP_API void Emphasize(const MvObject& Image, MvObject* ImageEmphasize, const MvTuple& MaskWidth,
                             const MvTuple& MaskHeight, const MvTuple& Factor);

/**
 * @brief 量化 Laplacian-of-Gaussian，sigma 在 [0.1,32] 内，mirrored 边界，逐通道输出 Int16。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageLaplace 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_laplace_of_gauss
 */
MVKIT_CPP_API void LaplaceOfGauss(const MvObject& Image, MvObject* ImageLaplace, const MvTuple& Sigma);

/**
 * @brief 单通道 UInt8/UInt16 定义域内 CDF-min 均衡，分别使用 256/65536 个桶。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageEquHisto 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_equ_histo_image
 */
MVKIT_CPP_API void EquHistoImage(const MvObject& Image, MvObject* ImageEquHisto);

/**
 * @brief 执行线性灰度缩放。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageScaled 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mult 灰度乘法系数元组；调用期间借用，不转移所有权。
 * @param[in] Add 灰度加法系数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_scale_image
 */
MVKIT_CPP_API void ScaleImage(const MvObject& Image, MvObject* ImageScaled, const MvTuple& Mult,
                              const MvTuple& Add);

/**
 * @brief HALCON HSV 约定：三个独立平面，保持类型；real hue 使用弧度。
 *
 * @param[in] ImageRed 图标对象数组；调用期间借用，不转移所有权。
 * @param[in] ImageGreen 图标对象数组；调用期间借用，不转移所有权。
 * @param[in] ImageBlue 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] ImageResult1 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ImageResult2 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ImageResult3 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] ColorSpace 颜色空间名称元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_trans_from_rgb
 */
MVKIT_CPP_API void TransFromRgb(const MvObject& ImageRed, const MvObject& ImageGreen,
                                const MvObject& ImageBlue, MvObject* ImageResult1, MvObject* ImageResult2,
                                MvObject* ImageResult3, const MvTuple& ColorSpace);

/**
 * @brief 将指定颜色空间的三个图像通道转换为 RGB 通道。
 *
 * @param[in] ImageInput1 输入颜色通道图像；调用期间借用，不转移所有权。
 * @param[in] ImageInput2 输入颜色通道图像；调用期间借用，不转移所有权。
 * @param[in] ImageInput3 输入颜色通道图像；调用期间借用，不转移所有权。
 * @param[out] ImageRed 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ImageGreen 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ImageBlue 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] ColorSpace 颜色空间名称元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_trans_to_rgb
 */
MVKIT_CPP_API void TransToRgb(const MvObject& ImageInput1, const MvObject& ImageInput2,
                              const MvObject& ImageInput3, MvObject* ImageRed, MvObject* ImageGreen,
                              MvObject* ImageBlue, const MvTuple& ColorSpace);

/**
 * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] DerivGauss 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
 * @param[in] Component 高斯导数分量元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_derivate_gauss
 */
MVKIT_CPP_API void DerivateGauss(const MvObject& Image, MvObject* DerivGauss, const MvTuple& Sigma,
                                 const MvTuple& Component);

/**
 * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Method 算法方法元组；调用期间借用，不转移所有权。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_local_threshold
 */
MVKIT_CPP_API void LocalThreshold(const MvObject& Image, MvObject* Region, const MvTuple& Method,
                                  const MvTuple& LightDark, const MvTuple& GenParamName,
                                  const MvTuple& GenParamValue);

/**
 * @brief 单通道图像定义域的 marker minimax 洪泛，每个 marker 一个盆地，按图像顺序输出。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] Markers 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] Basins 分水岭区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_watersheds_marker
 */
MVKIT_CPP_API void WatershedsMarker(const MvObject& Image, const MvObject& Markers, MvObject* Basins);

/**
 * @brief 将图像写入文件。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] Format 图像或模型的文件格式元组；调用期间借用，不转移所有权。
 * @param[in] FillColor 图像定义域外的填充灰度元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_image
 */
MVKIT_CPP_API void WriteImage(const MvObject& Image, const MvTuple& Format, const MvTuple& FillColor,
                              const MvTuple& FileName);

/**
 * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageAffineTrans 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 * @param[in] AdaptImageSize 是否调整输出图像尺寸元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_image
 */
MVKIT_CPP_API void AffineTransImage(const MvObject& Image, MvObject* ImageAffineTrans,
                                    const MvTuple& HomMat2D, const MvTuple& Interpolation,
                                    const MvTuple& AdaptImageSize);

/**
 * @brief 一个 int4 线性索引 Map 或五通道 int4/uint2 权重 Map 决定输出尺寸和定义域。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] Map 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] ImageMapped 图像；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_map_image
 */
MVKIT_CPP_API void MapImage(const MvObject& Image, const MvObject& Map, MvObject* ImageMapped);

/**
 * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] PolarTransImage 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleEnd 终止角度元组；调用期间借用，不转移所有权。
 * @param[in] RadiusStart 极坐标采样起始半径元组；调用期间借用，不转移所有权。
 * @param[in] RadiusEnd 极坐标采样终止半径元组；调用期间借用，不转移所有权。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_polar_trans_image_ext
 */
MVKIT_CPP_API void PolarTransImageExt(const MvObject& Image, MvObject* PolarTransImage, const MvTuple& Row,
                                      const MvTuple& Column, const MvTuple& AngleStart,
                                      const MvTuple& AngleEnd, const MvTuple& RadiusStart,
                                      const MvTuple& RadiusEnd, const MvTuple& Width, const MvTuple& Height,
                                      const MvTuple& Interpolation);

/**
 * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] TransImage 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Interpolation 插值方式元组；调用期间借用，不转移所有权。
 * @param[in] AdaptImageSize 是否调整输出图像尺寸元组；调用期间借用，不转移所有权。
 * @param[in] TransformDomain 是否变换图像定义域元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_projective_trans_image
 */
MVKIT_CPP_API void ProjectiveTransImage(const MvObject& Image, MvObject* TransImage, const MvTuple& HomMat2D,
                                        const MvTuple& Interpolation, const MvTuple& AdaptImageSize,
                                        const MvTuple& TransformDomain);

/**
 * @brief 创建像素值为零的单通道图像。
 *
 * @details 支持 byte/uint2/int2/int4/real，Width 和 Height 为正整数。
 *
 * @param[out] Image 图像对象；由 C++ 对象自动管理资源。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image_const
 */
MVKIT_CPP_API void GenImageConst(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 从连续行存储的像素指针复制创建单通道图像。
 *
 * @details 调用期间复制像素，返回后源缓冲区可以释放；支持 byte/uint2/int2/int4/real。
 *
 * @param[out] Image 图像对象；由 C++ 对象自动管理资源。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image1
 */
MVKIT_CPP_API void GenImage1(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer);

/**
 * @brief 从三个连续行存储的同类型平面复制创建三通道图像。
 *
 * @details 三个平面采用同一 Type；不提供每通道独立类型或释放上下文。
 *
 * @param[out] ImageRGB 三通道图像对象；由 C++ 对象自动管理资源。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointerRed 像素平面实际地址的整数控制元组。
 * @param[in] PixelPointerGreen 像素平面实际地址的整数控制元组。
 * @param[in] PixelPointerBlue 像素平面实际地址的整数控制元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image3
 */
MVKIT_CPP_API void GenImage3(MvObject* ImageRGB, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointerRed, const MvTuple& PixelPointerGreen, const MvTuple& PixelPointerBlue);

/**
 * @brief 引用连续行存储的外部单通道像素，最后一次引用退出时执行清理。
 *
 * @details ClearProc 为 void (*)(void*) 的地址，接收原像素指针；0 表示不清理。只有创建成功才接管回调，回调不得抛异常。
 *
 * @param[out] Image 图像对象；由 C++ 对象自动管理资源。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] Width 图像宽度控制元组。
 * @param[in] Height 图像高度控制元组。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 * @param[in] ClearProc 清理函数地址整数元组，0 表示不清理。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image1_extern
 */
MVKIT_CPP_API void GenImage1Extern(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer, const MvTuple& ClearProc);

/**
 * @brief 从连续行存储的交错 RGB/BGR 数据复制创建三通道图像。
 *
 * @details 当前支持 byte 输出及 rgb/bgr/rgbx/bgrx、BitsPerChannel=-1/8、BitShift=0；Alignment 为保留参数。宽高为 0 时使用源尺寸，起点从 0 开始；其他编码明确返回不支持。
 *
 * @param[out] ImageRGB 三通道图像对象；由 C++ 对象自动管理资源。
 * @param[in] PixelPointer 像素缓冲区实际地址的整数元组。
 * @param[in] ColorFormat 交错颜色排列名称。
 * @param[in] OriginalWidth 源图像宽度。
 * @param[in] OriginalHeight 源图像高度。
 * @param[in] Alignment 保留的整数控制参数。
 * @param[in] Type 像素类型名称控制元组。
 * @param[in] ImageWidth 输出裁剪宽度，0 使用源宽度。
 * @param[in] ImageHeight 输出裁剪高度，0 使用源高度。
 * @param[in] StartRow 从 0 开始的裁剪起始行。
 * @param[in] StartColumn 从 0 开始的裁剪起始列。
 * @param[in] BitsPerChannel 输出通道有效位数。
 * @param[in] BitShift 输入数据右移位数。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_image_interleaved
 */
MVKIT_CPP_API void GenImageInterleaved(MvObject* ImageRGB, const MvTuple& PixelPointer, const MvTuple& ColorFormat, const MvTuple& OriginalWidth, const MvTuple& OriginalHeight, const MvTuple& Alignment, const MvTuple& Type, const MvTuple& ImageWidth, const MvTuple& ImageHeight, const MvTuple& StartRow, const MvTuple& StartColumn, const MvTuple& BitsPerChannel, const MvTuple& BitShift);

/**
 * @brief 获取各图像第一通道的像素借用指针、类型和尺寸。
 *
 * @details 指针以整数元组返回，仅在图像仍存在时有效；写入会影响共享像素，不执行隐式分离。需要独立可写数据先调用 copy_image。
 *
 * @param[in] Image 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] Pointer 像素平面实际地址的整数控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Type 像素类型名称控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Width 图像宽度控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Height 图像高度控制元组。 结果由 C++ 元组自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_image_pointer1
 */
MVKIT_CPP_API void GetImagePointer1(const MvObject& Image, MvTuple* Pointer, MvTuple* Type, MvTuple* Width, MvTuple* Height);

/**
 * @brief 获取各三通道图像的像素借用指针、统一类型和尺寸。
 *
 * @details 要求三个通道具有相同类型且连续存储；指针借用及写入规则同 get_image_pointer1。
 *
 * @param[in] ImageRGB 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] PointerRed 像素平面实际地址的整数控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] PointerGreen 像素平面实际地址的整数控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] PointerBlue 像素平面实际地址的整数控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Type 像素类型名称控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Width 图像宽度控制元组。 结果由 C++ 元组自动管理资源。
 * @param[out] Height 图像高度控制元组。 结果由 C++ 元组自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_image_pointer3
 */
MVKIT_CPP_API void GetImagePointer3(const MvObject& ImageRGB, MvTuple* PointerRed, MvTuple* PointerGreen, MvTuple* PointerBlue, MvTuple* Type, MvTuple* Width, MvTuple* Height);

/**
 * @brief 按对象顺序获取图像第一通道的像素类型。
 *
 * @param[in] Image 输入图像对象数组；调用期间借用，不转移所有权。
 * @param[out] Type 像素类型名称控制元组。 结果由 C++ 元组自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_image_type
 */
MVKIT_CPP_API void GetImageType(const MvObject& Image, MvTuple* Type);

/**
 * @brief 按对象顺序查询图像尺寸。
 *
 * @param[in] Image 输入图像对象数组。
 * @param[out] Width 结果拥有型元组，结果由 C++ 元组自动管理资源。
 * @param[out] Height 结果拥有型元组，结果由 C++ 元组自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_image_size
 */
MVKIT_CPP_API void GetImageSize(const MvObject& Image, MvTuple* Width, MvTuple* Height);

/**
 * @brief 按对象顺序查询图像通道数量。
 *
 * @param[in] MultiChannelImage 输入图像对象数组。
 * @param[out] Channels 结果拥有型元组，结果由 C++ 元组自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_count_channels
 */
MVKIT_CPP_API void CountChannels(const MvObject& MultiChannelImage, MvTuple* Channels);

}  // namespace mv
