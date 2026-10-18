#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {
class MvNCCModelArray;
/**
 * @brief 自动管理共享引用的 NCC 匹配模型。
 *
 * @details 复制共享的不可变 NCC 控制模型，默认构造未初始化。
 */
class MVKIT_CPP_API MvNCCModel : public MvHandle {
 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvNCCModel() noexcept;

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvNCCModel(const MvNCCModel&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvNCCModel(MvNCCModel&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvNCCModel& operator=(const MvNCCModel&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvNCCModel& operator=(MvNCCModel&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvNCCModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvNCCModel(const MvHandle& handle);

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   *
   * @param[in] Template 模板图像；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_ncc_model
   */
  void CreateNccModel(const MvImage& Template,const MvTuple& NumLevels,double AngleStart,
      double AngleExtent,const MvTuple& AngleStep,const MvString& Metric);

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateNccModel(const MvImage& Template,const MvTuple& NumLevels,double AngleStart,
      double AngleExtent,const MvTuple& AngleStep,const char* Metric);

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateNccModel(const MvImage& Template,std::int64_t NumLevels,double AngleStart,
      double AngleExtent,double AngleStep,const MvString& Metric);

  /**
   * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateNccModel(const MvImage& Template,std::int64_t NumLevels,double AngleStart,
      double AngleExtent,double AngleStep,const char* Metric);

  /**
   * @brief 查询 NCC 模型的训练和搜索参数。
   *
   * @param[out] AngleStart 起始角度。
   * @param[out] AngleExtent 角度区间长度。
   * @param[out] AngleStep 角度采样步长。
   * @param[out] Metric 距离或匹配度量；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 金字塔层级值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_ncc_model_params
   */
  std::int64_t GetNccModelParams(double* AngleStart,double* AngleExtent,double* AngleStep,MvString* Metric) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  void FindNccModel(const MvImage& Image,double AngleStart,double AngleExtent,double MinScore,
      std::int64_t NumMatches,double MaxOverlap,const MvString& SubPixel,const MvTuple& NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModel(const MvImage& Image,double AngleStart,double AngleExtent,double MinScore,
      std::int64_t NumMatches,double MaxOverlap,const MvString& SubPixel,std::int64_t NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score) const;

  /**
   * @brief 在图像中搜索 NCC 匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModel(const MvImage& Image,double AngleStart,double AngleExtent,double MinScore,
      std::int64_t NumMatches,double MaxOverlap,const char* SubPixel,std::int64_t NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score) const;

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  static void FindNccModels(const MvImage& Image,const MvNCCModelArray& ModelIDs,
      const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& MinScore,
      const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,const MvTuple& NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score,MvTuple* Model);

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModels(const MvImage& Image,double AngleStart,double AngleExtent,double MinScore,
      std::int64_t NumMatches,double MaxOverlap,const MvString& SubPixel,std::int64_t NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score,MvTuple* Model) const;

  /**
   * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindNccModels(const MvImage& Image,double AngleStart,double AngleExtent,double MinScore,
      std::int64_t NumMatches,double MaxOverlap,const char* SubPixel,std::int64_t NumLevels,
      MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score,MvTuple* Model) const;

  /**
   * @brief 从 MVKit 原生格式文件读取 NCC 模型。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_ncc_model
   */
  void ReadNccModel(const MvString& FileName);

  /**
   * @brief 从 MVKit 原生格式文件读取 NCC 模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadNccModel(const char* FileName);

  /**
   * @brief 将 NCC 模型写入 MVKit 原生格式文件。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_ncc_model
   */
  void WriteNccModel(const MvString& FileName) const;

  /**
   * @brief 将 NCC 模型写入 MVKit 原生格式文件。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteNccModel(const char* FileName) const;

};
struct MvNCCModelArrayData;
/**
 * @brief 拥有型 NCC 模型数组，自动管理各元素的共享引用。
 */
class MVKIT_CPP_API MvNCCModelArray {
 public:
  /**
   * @brief 构造空的拥有型数组。
   */
  MvNCCModelArray() noexcept;

  /**
   * @brief 复制数组元素，构造拥有型数组。
   *
   * @param[in] models 需要复制的模型数组。
   * @param[in] length 几何长度或半长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvNCCModelArray(const MvNCCModel* models,std::int64_t length);

  /**
   * @brief 复制构造，复制数组元素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvNCCModelArray(const MvNCCModelArray&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvNCCModelArray(MvNCCModelArray&&) noexcept;

  /**
   * @brief 复制赋值，复制数组元素。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvNCCModelArray& operator=(const MvNCCModelArray&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvNCCModelArray& operator=(MvNCCModelArray&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvNCCModelArray() noexcept;

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
  const MvNCCModel* Tools() const noexcept;

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
  MvNCCModelArrayData* data_{nullptr};
};
/**
 * @brief 在图像中搜索 NCC 匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间借用，不转移所有权。
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
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_find_ncc_model
 */
MVKIT_CPP_API void FindNccModel(const MvObject& Image,const MvTuple& ModelID,
    const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& MinScore,
    const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,
    const MvTuple& NumLevels,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score);

/**
 * @brief 多模型控制接受单值或每模型一个值。标量 NumMatches 是全局配额，标量 MaxOverlap 跨模型抑制。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelIDs 包含NCC模型句柄的控制元组；调用期间借用，不转移所有权。
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
MVKIT_CPP_API void FindNccModels(const MvObject& Image,const MvTuple& ModelIDs,
    const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& MinScore,
    const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,
    const MvTuple& NumLevels,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score,MvTuple* Model);

/**
 * @brief 使用一张 byte/uint2 模板图像；NumLevels/AngleStep 接受 "auto"、零或标量。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
 * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[out] ModelID NCC模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_ncc_model
 */
MVKIT_CPP_API void CreateNccModel(const MvObject& Template,const MvTuple& NumLevels,
    const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& AngleStep,
    const MvTuple& Metric,MvTuple* ModelID);

/**
 * @brief 查询 NCC 模型的训练和搜索参数。
 *
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] NumLevels 金字塔层级控制元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleStart 起始角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleExtent 角度区间长度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleStep 角度采样步长元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Metric 距离或匹配度量元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_ncc_model_params
 */
MVKIT_CPP_API void GetNccModelParams(const MvTuple& ModelID,MvTuple* NumLevels,
    MvTuple* AngleStart,MvTuple* AngleExtent,MvTuple* AngleStep,MvTuple* Metric);

/**
 * @brief 从 MVKit 原生格式文件读取 NCC 模型。
 *
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] ModelID NCC模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_ncc_model
 */
MVKIT_CPP_API void ReadNccModel(const MvTuple& FileName,MvTuple* ModelID);

/**
 * @brief 将 NCC 模型写入 MVKit 原生格式文件。
 *
 * @param[in] ModelID 包含NCC模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_ncc_model
 */
MVKIT_CPP_API void WriteNccModel(const MvTuple& ModelID,const MvTuple& FileName);

}
