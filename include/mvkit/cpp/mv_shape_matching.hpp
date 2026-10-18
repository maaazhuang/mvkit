#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {
class MvShapeModelArray;
/**
 * @brief 自动管理共享引用的形状匹配模型。
 *
 * @details 复制共享的 形状控制模型，默认构造未初始化。
 */
class MVKIT_CPP_API MvShapeModel : public MvHandle {
 public:
  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  static void FindScaledShapeModels(const MvImage& Image, const MvShapeModelArray& ModelIDs, const MvTuple& AngleStart, const MvTuple& AngleExtent, const MvTuple& ScaleMin, const MvTuple& ScaleMax, const MvTuple& MinScore, const MvTuple& NumMatches, const MvTuple& MaxOverlap, const MvTuple& SubPixel, const MvTuple& NumLevels, const MvTuple& Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score, MvTuple* Model);

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  static void FindShapeModels(const MvImage& Image, const MvShapeModelArray& ModelIDs, const MvTuple& AngleStart, const MvTuple& AngleExtent, const MvTuple& MinScore, const MvTuple& NumMatches, const MvTuple& MaxOverlap, const MvTuple& SubPixel, const MvTuple& NumLevels, const MvTuple& Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model);

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索多个形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModels(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score, MvTuple* Model) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  void FindScaledShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, const MvTuple& MinScore, std::int64_t NumMatches, double MaxOverlap, const MvTuple& SubPixel, const MvTuple& NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索支持尺度变化的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindScaledShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double ScaleMin, double ScaleMax, double MinScore, std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Scale, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  void FindShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, const MvTuple& MinScore, std::int64_t NumMatches, double MaxOverlap, const MvTuple& SubPixel, const MvTuple& NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const MvString& SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const char* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 在图像中搜索形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FindShapeModel(const MvImage& Image, double AngleStart, double AngleExtent, double MinScore, std::int64_t NumMatches, double MaxOverlap, const wchar_t* SubPixel, std::int64_t NumLevels, double Greediness, MvTuple* Row, MvTuple* Column, MvTuple* Angle, MvTuple* Score) const;

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] ScaleMin 模型尺度下限。
   * @param[in] ScaleMax 模型尺度上限。
   * @param[in] ScaleStep 模型尺度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_scaled_shape_model_xld
   */
  void CreateScaledShapeModelXld(const MvXLDCont& Contours, const MvTuple& NumLevels, double AngleStart, double AngleExtent, const MvTuple& AngleStep, double ScaleMin, double ScaleMax, const MvTuple& ScaleStep, const MvTuple& Optimization, const MvString& Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const MvString& Optimization, const MvString& Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const char* Optimization, const char* Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const wchar_t* Optimization, const wchar_t* Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_shape_model_xld
   */
  void CreateShapeModelXld(const MvXLDCont& Contours, const MvTuple& NumLevels, double AngleStart, double AngleExtent, const MvTuple& AngleStep, const MvTuple& Optimization, const MvString& Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const MvString& Optimization, const MvString& Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const char* Optimization, const char* Metric, std::int64_t MinContrast);

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModelXld(const MvXLDCont& Contours, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const wchar_t* Optimization, const wchar_t* Metric, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   *
   * @param[in] Template 模板图像；调用期间借用，不转移所有权。
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
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_scaled_shape_model
   */
  void CreateScaledShapeModel(const MvImage& Template, const MvTuple& NumLevels, double AngleStart, double AngleExtent, const MvTuple& AngleStep, double ScaleMin, double ScaleMax, const MvTuple& ScaleStep, const MvTuple& Optimization, const MvString& Metric, const MvTuple& Contrast, const MvTuple& MinContrast);

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const MvString& Optimization, const MvString& Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const char* Optimization, const char* Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateScaledShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep, const wchar_t* Optimization, const wchar_t* Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建形状匹配模型。
   *
   * @param[in] Template 模板图像；调用期间借用，不转移所有权。
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] Contrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_shape_model
   */
  void CreateShapeModel(const MvImage& Template, const MvTuple& NumLevels, double AngleStart, double AngleExtent, const MvTuple& AngleStep, const MvTuple& Optimization, const MvString& Metric, const MvTuple& Contrast, const MvTuple& MinContrast);

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const MvString& Optimization, const MvString& Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const char* Optimization, const char* Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 根据模板图像创建形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void CreateShapeModel(const MvImage& Template, std::int64_t NumLevels, double AngleStart, double AngleExtent, double AngleStep, const wchar_t* Optimization, const wchar_t* Metric, std::int64_t Contrast, std::int64_t MinContrast);

  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvShapeModel() noexcept;

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvShapeModel(const MvShapeModel&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvShapeModel(MvShapeModel&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvShapeModel& operator=(const MvShapeModel&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvShapeModel& operator=(MvShapeModel&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvShapeModel() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvShapeModel(const MvHandle& handle);

  /**
   * @brief 获取指定金字塔层级的形状模型轮廓。
   *
   * @param[in] Level 模型轮廓的金字塔层级。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_shape_model_contours
   */
  MvXLDCont GetShapeModelContours(std::int64_t Level) const;

  /**
   * @brief 查询形状匹配模型的原点偏移。
   *
   * @param[out] Row 行坐标。
   * @param[out] Column 列坐标。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_shape_model_origin
   */
  void GetShapeModelOrigin(double* Row,double* Column) const;

  /**
   * @brief 设置形状匹配模型的原点偏移。
   *
   * @param[in] Row 行坐标。
   * @param[in] Column 列坐标。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_shape_model_origin
   */
  void SetShapeModelOrigin(double Row,double Column) const;

  /**
   * @brief 设置形状匹配模型的指定参数。
   *
   * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_shape_model_param
   */
  void SetShapeModelParam(const MvTuple& GenParamName,const MvTuple& GenParamValue) const;

  /**
   * @brief 查询形状匹配模型的训练和搜索参数。
   *
   * @param[out] AngleStart 起始角度。
   * @param[out] AngleExtent 角度区间长度。
   * @param[out] AngleStep 角度采样步长。
   * @param[out] ScaleMin 模型尺度下限元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ScaleMax 模型尺度上限元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ScaleStep 模型尺度采样步长元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Metric 距离或匹配度量；接收结果的 C++ 对象，自动管理资源。
   * @param[out] MinContrast 模板对比度阈值。
   *
   * @return 金字塔层级控制元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_shape_model_params
   */
  MvTuple GetShapeModelParams(double* AngleStart,double* AngleExtent,double* AngleStep,
      MvTuple* ScaleMin,MvTuple* ScaleMax,MvTuple* ScaleStep,MvString* Metric,std::int64_t* MinContrast) const;

  /**
   * @brief 查询形状匹配模型的训练和搜索参数。
   * @overload 采用标量输出参数。
   *
   * @return 金字塔层级值。
   */
  std::int64_t GetShapeModelParams(double* AngleStart,double* AngleExtent,double* AngleStep,
      double* ScaleMin,double* ScaleMax,double* ScaleStep,MvString* Metric,std::int64_t* MinContrast) const;

  /**
   * @brief 从 MVKit 原生格式文件读取形状匹配模型。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_shape_model
   */
  void ReadShapeModel(const MvString& FileName);

  /**
   * @brief 从 MVKit 原生格式文件读取形状匹配模型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadShapeModel(const char* FileName);

  /**
   * @brief 从 MVKit 原生格式文件读取形状匹配模型。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ReadShapeModel(const wchar_t* FileName);

  /**
   * @brief MVKit 原生模型持久化，不能与 HALCON 二进制文件互换。
   *
   * @param[in] FileName 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_shape_model
   */
  void WriteShapeModel(const MvString& FileName) const;

  /**
   * @brief MVKit 原生模型持久化，不能与 HALCON 二进制文件互换。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WriteShapeModel(const char* FileName) const;

  /**
   * @brief MVKit 原生模型持久化，不能与 HALCON 二进制文件互换。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WriteShapeModel(const wchar_t* FileName) const;

};
/**
 * @brief 获取指定金字塔层级的形状模型轮廓。
 *
 * @param[out] ModelContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Level 模型轮廓的金字塔层级元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_shape_model_contours
 */
MVKIT_CPP_API void GetShapeModelContours(MvObject* ModelContours,const MvTuple& ModelID,const MvTuple& Level);

/**
 * @brief 查询形状匹配模型的原点偏移。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_shape_model_origin
 */
MVKIT_CPP_API void GetShapeModelOrigin(const MvTuple& ModelID,MvTuple* Row,MvTuple* Column);

/**
 * @brief 设置形状匹配模型的原点偏移。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_shape_model_origin
 */
MVKIT_CPP_API void SetShapeModelOrigin(const MvTuple& ModelID,const MvTuple& Row,const MvTuple& Column);

/**
 * @brief 设置形状匹配模型的指定参数。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_shape_model_param
 */
MVKIT_CPP_API void SetShapeModelParam(const MvTuple& ModelID,const MvTuple& GenParamName,const MvTuple& GenParamValue);

/**
 * @brief 查询形状匹配模型的训练和搜索参数。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[out] NumLevels 金字塔层级控制元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleStart 起始角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleExtent 角度区间长度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] AngleStep 角度采样步长元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ScaleMin 模型尺度下限元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ScaleMax 模型尺度上限元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ScaleStep 模型尺度采样步长元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Metric 距离或匹配度量元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] MinContrast 模板对比度阈值元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_shape_model_params
 */
MVKIT_CPP_API void GetShapeModelParams(const MvTuple& ModelID,MvTuple* NumLevels,
    MvTuple* AngleStart,MvTuple* AngleExtent,MvTuple* AngleStep,MvTuple* ScaleMin,
    MvTuple* ScaleMax,MvTuple* ScaleStep,MvTuple* Metric,MvTuple* MinContrast);

/**
 * @brief 从 MVKit 原生格式文件读取形状匹配模型。
 *
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 * @param[out] ModelID 形状匹配模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_read_shape_model
 */
MVKIT_CPP_API void ReadShapeModel(const MvTuple& FileName,MvTuple* ModelID);

/**
 * @brief MVKit 原生模型持久化，不能与 HALCON 二进制文件互换。
 *
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
 * @param[in] FileName 文件路径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_write_shape_model
 */
MVKIT_CPP_API void WriteShapeModel(const MvTuple& ModelID,const MvTuple& FileName);

/**
 * @brief 根据模板图像创建形状匹配模型。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
 * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[in] Contrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[out] ModelID 形状匹配模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_shape_model
 */
MVKIT_CPP_API void CreateShapeModel(const MvObject& Template,const MvTuple& NumLevels,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& AngleStep,const MvTuple& Optimization,const MvTuple& Metric,const MvTuple& Contrast,const MvTuple& MinContrast,MvTuple* ModelID);

/**
 * @brief 根据模板图像创建支持尺度搜索的形状匹配模型。
 *
 * @param[in] Template 模板图像；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
 * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间借用，不转移所有权。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间借用，不转移所有权。
 * @param[in] ScaleStep 模型尺度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[in] Contrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[out] ModelID 形状匹配模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_scaled_shape_model
 */
MVKIT_CPP_API void CreateScaledShapeModel(const MvObject& Template,const MvTuple& NumLevels,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& AngleStep,const MvTuple& ScaleMin,const MvTuple& ScaleMax,const MvTuple& ScaleStep,const MvTuple& Optimization,const MvTuple& Metric,const MvTuple& Contrast,const MvTuple& MinContrast,MvTuple* ModelID);

/**
 * @brief 根据 XLD 模板轮廓创建形状匹配模型。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
 * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[out] ModelID 形状匹配模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_shape_model_xld
 */
MVKIT_CPP_API void CreateShapeModelXld(const MvObject& Contours,const MvTuple& NumLevels,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& AngleStep,const MvTuple& Optimization,const MvTuple& Metric,const MvTuple& MinContrast,MvTuple* ModelID);

/**
 * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
 * @param[in] AngleStart 起始角度元组；调用期间借用，不转移所有权。
 * @param[in] AngleExtent 角度区间长度元组；调用期间借用，不转移所有权。
 * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] ScaleMin 模型尺度下限元组；调用期间借用，不转移所有权。
 * @param[in] ScaleMax 模型尺度上限元组；调用期间借用，不转移所有权。
 * @param[in] ScaleStep 模型尺度采样步长元组；调用期间借用，不转移所有权。
 * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[in] MinContrast 模板对比度阈值元组；调用期间借用，不转移所有权。
 * @param[out] ModelID 形状匹配模型句柄元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_scaled_shape_model_xld
 */
MVKIT_CPP_API void CreateScaledShapeModelXld(const MvObject& Contours,const MvTuple& NumLevels,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& AngleStep,const MvTuple& ScaleMin,const MvTuple& ScaleMax,const MvTuple& ScaleStep,const MvTuple& Optimization,const MvTuple& Metric,const MvTuple& MinContrast,MvTuple* ModelID);

struct MvShapeModelArrayData;
/**
 * @brief 拥有型形状模型数组，自动管理各元素的共享引用。
 */
class MVKIT_CPP_API MvShapeModelArray {
 public:
  /**
   * @brief 构造空的拥有型数组。
   */
  MvShapeModelArray() noexcept;

  /**
   * @brief 复制数组元素，构造拥有型数组。
   *
   * @param[in] models 需要复制的模型数组。
   * @param[in] length 几何长度或半长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvShapeModelArray(const MvShapeModel* models,std::int64_t length);

  /**
   * @brief 复制构造，复制数组元素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvShapeModelArray(const MvShapeModelArray&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvShapeModelArray(MvShapeModelArray&&) noexcept;

  /**
   * @brief 复制赋值，复制数组元素。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvShapeModelArray& operator=(const MvShapeModelArray&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvShapeModelArray& operator=(MvShapeModelArray&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvShapeModelArray() noexcept;

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
  const MvShapeModel* Tools() const noexcept;

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
  MvShapeModelArrayData* data_{nullptr};
};
/**
 * @brief 在图像中搜索形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
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
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_find_shape_model
 */
MVKIT_CPP_API void FindShapeModel(const MvObject& Image,const MvTuple& ModelID,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& MinScore,const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,const MvTuple& NumLevels,const MvTuple& Greediness,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score);

/**
 * @brief 在图像中搜索支持尺度变化的形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelID 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
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
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_find_scaled_shape_model
 */
MVKIT_CPP_API void FindScaledShapeModel(const MvObject& Image,const MvTuple& ModelID,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& ScaleMin,const MvTuple& ScaleMax,const MvTuple& MinScore,const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,const MvTuple& NumLevels,const MvTuple& Greediness,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Scale,MvTuple* Score);

/**
 * @brief 在图像中搜索多个形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelIDs 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
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
MVKIT_CPP_API void FindShapeModels(const MvObject& Image,const MvTuple& ModelIDs,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& MinScore,const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,const MvTuple& NumLevels,const MvTuple& Greediness,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Score,MvTuple* Model);

/**
 * @brief 在图像中搜索多个支持尺度变化的形状匹配模型。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[in] ModelIDs 包含形状匹配模型句柄的控制元组；调用期间借用，不转移所有权。
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
MVKIT_CPP_API void FindScaledShapeModels(const MvObject& Image,const MvTuple& ModelIDs,const MvTuple& AngleStart,const MvTuple& AngleExtent,const MvTuple& ScaleMin,const MvTuple& ScaleMax,const MvTuple& MinScore,const MvTuple& NumMatches,const MvTuple& MaxOverlap,const MvTuple& SubPixel,const MvTuple& NumLevels,const MvTuple& Greediness,MvTuple* Row,MvTuple* Column,MvTuple* Angle,MvTuple* Scale,MvTuple* Score,MvTuple* Model);

}
