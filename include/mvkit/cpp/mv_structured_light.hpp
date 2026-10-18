#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {
class MvObjectModel3D;
/**
 * @brief 自动管理共享引用的可变结构光模型。
 *
 * @details 共享控制身份；副本共享修改，调用方须同步访问。
 */
class MVKIT_CPP_API MvStructuredLightModel : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvStructuredLightModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvStructuredLightModel(const MvHandle&);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvStructuredLightModel(const MvStructuredLightModel&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvStructuredLightModel(MvStructuredLightModel&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvStructuredLightModel& operator=(const MvStructuredLightModel&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvStructuredLightModel& operator=(MvStructuredLightModel&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvStructuredLightModel() noexcept;

  /**
   * @brief 创建指定类型的结构光模型。
   *
   * @param[in] ModelType 结构光模型类型；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_structured_light_model
   */
  void CreateStructuredLightModel(const MvString& ModelType);

  /**
   * @brief 创建指定类型的结构光模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void CreateStructuredLightModel(const char* ModelType);

  /**
   * @brief 创建指定类型的结构光模型。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void CreateStructuredLightModel(const wchar_t* ModelType);

  /**
   * @brief 根据采集图像解码结构光图案。
   *
   * @param[in] CameraImages 结构光采集图像；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_decode_structured_light_pattern
   */
  void DecodeStructuredLightPattern(const MvImage& CameraImages) const;

  /**
   * @brief 生成结构光模型所需的投影图案。
   *
   * @return 结构光投影图案，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_structured_light_pattern
   */
  MvImage GenStructuredLightPattern() const;

  /**
   * @brief 查询结构光模型的指定参数。
   *
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_structured_light_model_param
   */
  MvTuple GetStructuredLightModelParam(const MvTuple& GenParamName) const;

  /**
   * @brief 查询结构光模型的指定参数。
   * @overload 采用字符串对象参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetStructuredLightModelParam(const MvString& GenParamName) const;

  /**
   * @brief 查询结构光模型的指定参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetStructuredLightModelParam(const char* GenParamName) const;

  /**
   * @brief 查询结构光模型的指定参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 参数或数据项的值元组，由返回对象自动管理资源。
   */
  MvTuple GetStructuredLightModelParam(const wchar_t* GenParamName) const;

  /**
   * @brief 获取结构光模型中的图案、解码或质量图像对象。
   *
   * @param[in] ObjectName 对象或轮廓查询名称元组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_structured_light_object
   */
  MvObject GetStructuredLightObject(const MvTuple& ObjectName) const;

  /**
   * @brief 获取结构光模型中的图案、解码或质量图像对象。
   * @overload 采用字符串对象参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvObject GetStructuredLightObject(const MvString& ObjectName) const;

  /**
   * @brief 获取结构光模型中的图案、解码或质量图像对象。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvObject GetStructuredLightObject(const char* ObjectName) const;

  /**
   * @brief 获取结构光模型中的图案、解码或质量图像对象。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvObject GetStructuredLightObject(const wchar_t* ObjectName) const;

  /**
   * @brief 恢复 MVKit .mvslm 模型；不存储采集图像或解码中间状态。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_structured_light_model
   */
  void ReadStructuredLightModel(const MvString& FileName);

  /**
   * @brief 恢复 MVKit .mvslm 模型；不存储采集图像或解码中间状态。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadStructuredLightModel(const char* FileName);

  /**
   * @brief 恢复 MVKit .mvslm 模型；不存储采集图像或解码中间状态。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ReadStructuredLightModel(const wchar_t* FileName);

  /**
   * @brief 根据解码结果重建结构光三维表面。
   *
   * @return 三维对象模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_reconstruct_surface_structured_light
   */
  MvObjectModel3D ReconstructSurfaceStructuredLight() const;

  /**
   * @brief 设置结构光模型的指定参数。
   *
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_structured_light_model_param
   */
  void SetStructuredLightModelParam(const MvTuple& GenParamName, const MvTuple& GenParamValue) const;

  /**
   * @brief 设置结构光模型的指定参数。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void SetStructuredLightModelParam(const MvString& GenParamName, std::int64_t GenParamValue) const;

  /**
   * @brief 设置结构光模型的指定参数。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void SetStructuredLightModelParam(const char* GenParamName, std::int64_t GenParamValue) const;

  /**
   * @brief 设置结构光模型的指定参数。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void SetStructuredLightModelParam(const wchar_t* GenParamName, std::int64_t GenParamValue) const;

  /**
   * @brief 按 MVKit .mvslm 格式保存配置及可选完整相机/投影仪配置。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_structured_light_model
   */
  void WriteStructuredLightModel(const MvString& FileName) const;

  /**
   * @brief 按 MVKit .mvslm 格式保存配置及可选完整相机/投影仪配置。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteStructuredLightModel(const char* FileName) const;

  /**
   * @brief 按 MVKit .mvslm 格式保存配置及可选完整相机/投影仪配置。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WriteStructuredLightModel(const wchar_t* FileName) const;

};

/**
 * @brief 创建指定类型的结构光模型。
 *
 * @param[in] ModelType 结构光模型类型元组；调用期间借用，不转移所有权。
 * @param[out] StructuredLightModel 结构光模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_structured_light_model
 */
MVKIT_CPP_API void CreateStructuredLightModel(const MvTuple& ModelType, MvTuple* StructuredLightModel);

/**
 * @brief 根据采集图像解码结构光图案。
 *
 * @param[in] CameraImages 结构光采集图像；调用期间借用，不转移所有权。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_decode_structured_light_pattern
 */
MVKIT_CPP_API void DecodeStructuredLightPattern(const MvObject& CameraImages,
                                                const MvTuple& StructuredLightModel);

/**
 * @brief 生成结构光模型所需的投影图案。
 *
 * @param[out] PatternImages 结构光投影图案；接收结果的 C++ 对象，自动管理资源。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_structured_light_pattern
 */
MVKIT_CPP_API void GenStructuredLightPattern(MvObject* PatternImages, const MvTuple& StructuredLightModel);

/**
 * @brief 查询结构光模型的指定参数。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] GenParamValue 参数或数据项的值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_structured_light_model_param
 */
MVKIT_CPP_API void GetStructuredLightModelParam(const MvTuple& StructuredLightModel,
                                                const MvTuple& GenParamName, MvTuple* GenParamValue);

/**
 * @brief 获取结构光模型中的图案、解码或质量图像对象。
 *
 * @param[out] Object 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] ObjectName 对象或轮廓查询名称元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_structured_light_object
 */
MVKIT_CPP_API void GetStructuredLightObject(MvObject* Object, const MvTuple& StructuredLightModel,
                                            const MvTuple& ObjectName);

/**
 * @brief 恢复 MVKit .mvslm 模型；不存储采集图像或解码中间状态。
 *
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] StructuredLightModel 结构光模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_structured_light_model
 */
MVKIT_CPP_API void ReadStructuredLightModel(const MvTuple& FileName, MvTuple* StructuredLightModel);

/**
 * @brief 根据解码结果重建结构光三维表面。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] ObjectModel3D 三维对象模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_reconstruct_surface_structured_light
 */
MVKIT_CPP_API void ReconstructSurfaceStructuredLight(const MvTuple& StructuredLightModel,
                                                     MvTuple* ObjectModel3D);

/**
 * @brief 设置结构光模型的指定参数。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_structured_light_model_param
 */
MVKIT_CPP_API void SetStructuredLightModelParam(const MvTuple& StructuredLightModel,
                                                const MvTuple& GenParamName, const MvTuple& GenParamValue);

/**
 * @brief 按 MVKit .mvslm 格式保存配置及可选完整相机/投影仪配置。
 *
 * @param[in] StructuredLightModel 包含结构光模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_structured_light_model
 */
MVKIT_CPP_API void WriteStructuredLightModel(const MvTuple& StructuredLightModel, const MvTuple& FileName);

}  // namespace mv
