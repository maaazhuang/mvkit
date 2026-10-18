#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {

/**
 * @brief 仅包含区域的图标对象数组；一个空区域仍计为一个对象。
 */
class MVKIT_CPP_API MvRegion : public MvObject {
 public:
  /**
   * @brief 生成一个空区域，与零对象数组不同。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_empty_region
   */
  void GenEmptyRegion();

  /**
   * @brief 由等长 Row/ColumnBegin/ColumnEnd 元组生成一个区域，栅格坐标须在有符号 32 位范围内。
   *
   * @param[in] Row 各游程的行坐标；调用期间借用，不转移所有权。
   * @param[in] ColumnBegin 各游程的起始列坐标；调用期间借用，不转移所有权。
   * @param[in] ColumnEnd 各游程的结束列坐标；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_region_runs
   */
  void GenRegionRuns(const MvTuple& Row, const MvTuple& ColumnBegin, const MvTuple& ColumnEnd);

  /**
   * @brief 由等长 Row/ColumnBegin/ColumnEnd 元组生成一个区域，栅格坐标须在有符号 32 位范围内。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenRegionRuns(std::int64_t Row, std::int64_t ColumnBegin, std::int64_t ColumnEnd);

  /**
   * @brief 将单区域读入三个独立拥有型整数元组，按行及起始列排序。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ColumnBegin 起点列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ColumnEnd 终点列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_region_runs
   */
  void GetRegionRuns(MvTuple* Row, MvTuple* ColumnBegin, MvTuple* ColumnEnd) const;

  /**
   * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  void MinMaxGray(const MvImage& Image, const MvTuple& Percent, MvTuple* Min, MvTuple* Max,
                  MvTuple* Range) const;

  /**
   * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void MinMaxGray(const MvImage& Image, double Percent, double* Min, double* Max, double* Range) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] Grayval 灰度值元组；调用期间借用，不转移所有权。
   * @param[in] Type 数据、模型或原语类型；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_paint_region
   */
  MvImage PaintRegion(const MvImage& Image, const MvTuple& Grayval, const MvString& Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvImage& Image, double Grayval, const MvString& Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvImage& Image, double Grayval, const char* Type) const;

  /**
   * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage PaintRegion(const MvImage& Image, double Grayval, const wchar_t* Type) const;

  /**
   * @brief 从区域对象数组生成轮廓；Mode 为 center、border 或 border_holes。
   *
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_contour_region_xld
   */
  MvXLDCont GenContourRegionXld(const MvString& Mode) const;

  /**
   * @brief 从区域对象数组生成轮廓；Mode 为 center、border 或 border_holes。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont GenContourRegionXld(const char* Mode) const;

  /**
   * @brief 从区域对象数组生成轮廓；Mode 为 center、border 或 border_holes。
   * @overload 采用宽字符字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont GenContourRegionXld(const wchar_t* Mode) const;

  /**
   * @brief 使用 scalar-or-N 数值控制批量生成椭圆。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Radius1 椭圆长半轴元组；调用期间借用，不转移所有权。
   * @param[in] Radius2 椭圆短半轴元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_ellipse
   */
  void GenEllipse(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1,
                  const MvTuple& Radius2);

  /**
   * @brief 使用 scalar-or-N 数值控制批量生成椭圆。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenEllipse(double Row, double Column, double Phi, double Radius1, double Radius2);

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   *
   * @param[in] HomMat2D 二维变换矩阵；调用期间借用，不转移所有权。
   * @param[in] Interpolate 插值方式；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_region
   */
  MvRegion AffineTransRegion(const MvHomMat2D& HomMat2D, const MvString& Interpolate) const;

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion AffineTransRegion(const MvHomMat2D& HomMat2D, const char* Interpolate) const;

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion AffineTransRegion(const MvHomMat2D& HomMat2D, const wchar_t* Interpolate) const;

  /**
   * @brief BoundaryType 为 inner/inner_filled/outer。inner/outer 使用八邻域形态学；
   *
   * @param[in] BoundaryType 区域边界提取方式；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_boundary
   */
  MvRegion Boundary(const MvString& BoundaryType) const;

  /**
   * @brief BoundaryType 为 inner/inner_filled/outer。inner/outer 使用八邻域形态学；
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion Boundary(const char* BoundaryType) const;

  /**
   * @brief BoundaryType 为 inner/inner_filled/outer。inner/outer 使用八邻域形态学；
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion Boundary(const wchar_t* BoundaryType) const;

  /**
   * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
   *
   * @param[in] Feature 属性或特征名称；调用期间借用，不转移所有权。
   * @param[in] Min 筛选或测量范围界限元组；调用期间借用，不转移所有权。
   * @param[in] Max 筛选或测量范围界限元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fill_up_shape
   */
  MvRegion FillUpShape(const MvString& Feature, const MvTuple& Min, const MvTuple& Max) const;

  /**
   * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion FillUpShape(const MvString& Feature, double Min, double Max) const;

  /**
   * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion FillUpShape(const char* Feature, double Min, double Max) const;

  /**
   * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion FillUpShape(const wchar_t* Feature, double Min, double Max) const;

  /**
   * @brief 每个区域返回一张 Int32 图像。Metric 为 euclidean/city-block/chessboard，
   *
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] Foreground 距离变换的前景选择；调用期间借用，不转移所有权。
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_transform
   */
  MvImage DistanceTransform(const MvString& Metric, const MvString& Foreground, std::int64_t Width,
                            std::int64_t Height) const;

  /**
   * @brief 每个区域返回一张 Int32 图像。Metric 为 euclidean/city-block/chessboard，
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage DistanceTransform(const char* Metric, const char* Foreground, std::int64_t Width,
                            std::int64_t Height) const;

  /**
   * @brief 每个区域返回一张 Int32 图像。Metric 为 euclidean/city-block/chessboard，
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage DistanceTransform(const wchar_t* Metric, const wchar_t* Foreground, std::int64_t Width,
                            std::int64_t Height) const;

  /**
   * @brief 位置无关的自定义形态学，使用一个结构元素对象，参考点为取整质心。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   * @param[in] Iterations 拟合或优化迭代次数。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_dilation1
   */
  MvRegion Dilation1(const MvRegion& StructElement, std::int64_t Iterations) const;

  /**
   * @brief 使用位置无关的结构区域腐蚀输入区域。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   * @param[in] Iterations 拟合或优化迭代次数。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_erosion1
   */
  MvRegion Erosion1(const MvRegion& StructElement, std::int64_t Iterations) const;

  /**
   * @brief 使用位置无关的结构区域执行区域开运算。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_opening
   */
  MvRegion Opening(const MvRegion& StructElement) const;

  /**
   * @brief 使用位置无关的结构区域执行区域闭运算。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_closing
   */
  MvRegion Closing(const MvRegion& StructElement) const;

  /**
   * @brief 填充闭合多边形，使用 even-odd 内部规则并包含像素中心边界。
   *
   * @param[in] Rows 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Columns 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_region_polygon_filled
   */
  void GenRegionPolygonFilled(const MvTuple& Rows, const MvTuple& Columns);

  /**
   * @brief Zhang-Suen 候选细化并逐步检查拓扑，前景八连通、背景四连通。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_skeleton
   */
  MvRegion Skeleton() const;

  /**
   * @brief 按对象均衡网格分块，Width/Height 为有限正标量。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 分块区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_partition_rectangle
   */
  MvRegion PartitionRectangle(double Width, double Height) const;

  /**
   * @brief 均衡列切分向最小列面积移动，Distance>=1，Percent 在 [0,100] 内。
   *
   * @param[in] Distance 距离。
   * @param[in] Percent 百分比值。
   *
   * @return 分块区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_partition_dynamic
   */
  MvRegion PartitionDynamic(double Distance, double Percent) const;

  /**
   * @brief 构造有效的空图标对象数组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvRegion();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvRegion(const MvRegion&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvRegion(MvRegion&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvRegion& operator=(const MvRegion&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvRegion& operator=(MvRegion&&);

  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvRegion(const MvObject& objects);

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
  MvRegion SelectObj(const MvTuple& index) const;

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectObj(std::int64_t index) const;

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
  MvRegion CopyObj(std::int64_t index, std::int64_t count) const;

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
  MvRegion ConcatObj(const MvRegion& other) const;

  /**
   * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
   *
   * @param[in] image 输入图像；调用期间借用，不转移所有权。
   * @param[out] deviation 灰度均值或标准差元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 灰度均值或标准差元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_intensity
   */
  MvTuple Intensity(const MvImage& image, MvTuple* deviation) const;

  /**
   * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
   * @overload 采用标量输出参数。
   *
   * @return 灰度均值或标准差。
   */
  double Intensity(const MvImage& image, double* deviation) const;

  /**
   * @brief 单个 Region、单个单通道图像 的 256-bin 直方图，支持五种像素类型。
   *
   * @param[in] image 输入图像；调用期间借用，不转移所有权。
   * @param[out] relative 归一化灰度直方图元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 各灰度桶的像素计数元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gray_histo
   */
  MvTuple GrayHisto(const MvImage& image, MvTuple* relative) const;

  /**
   * @brief 生成轴对齐矩形区域。
   *
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_rectangle1
   */
  void GenRectangle1(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2,
                     const MvTuple& Column2);

  /**
   * @brief 生成轴对齐矩形区域。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenRectangle1(double Row1, double Column1, double Row2, double Column2);

  /**
   * @brief 生成旋转矩形区域。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Length1 第一方向的几何半长元组；调用期间借用，不转移所有权。
   * @param[in] Length2 第二方向的几何半长元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_rectangle2
   */
  void GenRectangle2(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Length1,
                     const MvTuple& Length2);

  /**
   * @brief 生成旋转矩形区域。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenRectangle2(double Row, double Column, double Phi, double Length1, double Length2);

  /**
   * @brief 生成圆形区域。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_circle
   */
  void GenCircle(const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius);

  /**
   * @brief 生成圆形区域。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenCircle(double Row, double Column, double Radius);

  /**
   * @brief 使用圆形结构元素膨胀区域，Radius 为 [0.5,511.5] 中的一个数值。
   *
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_dilation_circle
   */
  MvRegion DilationCircle(const MvTuple& Radius) const;

  /**
   * @brief 使用圆形结构元素膨胀区域，Radius 为 [0.5,511.5] 中的一个数值。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion DilationCircle(double Radius) const;

  /**
   * @brief 使用圆形结构元素腐蚀区域。
   *
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_erosion_circle
   */
  MvRegion ErosionCircle(const MvTuple& Radius) const;

  /**
   * @brief 使用圆形结构元素腐蚀区域。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion ErosionCircle(double Radius) const;

  /**
   * @brief 使用圆形结构元素执行开运算。
   *
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_opening_circle
   */
  MvRegion OpeningCircle(const MvTuple& Radius) const;

  /**
   * @brief 使用圆形结构元素执行开运算。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion OpeningCircle(double Radius) const;

  /**
   * @brief 使用位置无关结构区域的区域顶帽/底帽。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_top_hat
   */
  MvRegion TopHat(const MvRegion& StructElement) const;

  /**
   * @brief 计算区域闭运算结果与原区域之差。
   *
   * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_bottom_hat
   */
  MvRegion BottomHat(const MvRegion& StructElement) const;

  /**
   * @brief 使用圆形结构元素执行闭运算。
   *
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_closing_circle
   */
  MvRegion ClosingCircle(const MvTuple& Radius) const;

  /**
   * @brief 使用圆形结构元素执行闭运算。
   * @overload 采用标量数值参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion ClosingCircle(double Radius) const;

  /**
   * @brief 使用矩形结构元素膨胀区域，Width/Height 各为一个数值并取最近整数。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_dilation_rectangle1
   */
  MvRegion DilationRectangle1(std::int64_t Width, std::int64_t Height) const;

  /**
   * @brief 使用矩形结构元素腐蚀区域。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_erosion_rectangle1
   */
  MvRegion ErosionRectangle1(std::int64_t Width, std::int64_t Height) const;

  /**
   * @brief 使用矩形结构元素执行开运算。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_opening_rectangle1
   */
  MvRegion OpeningRectangle1(std::int64_t Width, std::int64_t Height) const;

  /**
   * @brief 使用矩形结构元素执行闭运算。
   *
   * @param[in] Width 图像或分块宽度。
   * @param[in] Height 图像或分块高度。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_closing_rectangle1
   */
  MvRegion ClosingRectangle1(std::int64_t Width, std::int64_t Height) const;

  /**
   * @brief 按整数偏移平移每个区域；偏移和结果坐标须在有符号 32 位范围内，越界保留原输出。
   *
   * @param[in] Row 行坐标。
   * @param[in] Column 列坐标。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_move_region
   */
  MvRegion MoveRegion(std::int64_t Row, std::int64_t Column) const;

  /**
   * @brief 每个左侧对象与全部右侧对象的并集异或，保持左侧对象数量。
   *
   * @param[in] Region2 输入区域；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_symm_difference
   */
  MvRegion SymmDifference(const MvRegion& Region2) const;

  /**
   * @brief 返回一个 Int64 布尔值，判断对象数相同且逐个像素集合相等。
   *
   * @param[in] Regions2 输入区域；调用期间借用，不转移所有权。
   *
   * @return 几何关系判定标记。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_test_equal_region
   */
  std::int64_t TestEqualRegion(const MvRegion& Regions2) const;

  /**
   * @brief 任一点命中判断：一个点配 N 个区域，或 M 个点配一个区域。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 几何关系判定标记。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_test_region_point
   */
  std::int64_t TestRegionPoint(const MvTuple& Row, const MvTuple& Column) const;

  /**
   * @brief 任一点命中判断：一个点配 N 个区域，或 M 个点配一个区域。
   * @overload 采用标量数值参数。
   *
   * @return 几何关系判定标记。
   */
  std::int64_t TestRegionPoint(std::int64_t Row, std::int64_t Column) const;

  /**
   * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
   *
   * @param[in] Features 属性或特征名称元组；调用期间借用，不转移所有权。
   *
   * @return 保存、追加或读取的数据元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_region_features
   */
  MvTuple RegionFeatures(const MvTuple& Features) const;

  /**
   * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
   * @overload 采用字符串对象参数。
   *
   * @return 保存、追加或读取的数据。
   */
  double RegionFeatures(const MvString& Features) const;

  /**
   * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 保存、追加或读取的数据。
   */
  double RegionFeatures(const char* Features) const;

  /**
   * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
   * @overload 采用宽字符字符串参数。
   *
   * @return 保存、追加或读取的数据。
   */
  double RegionFeatures(const wchar_t* Features) const;

  /**
   * @brief 按形状特征筛选 Region 对象。
   *
   * @param[in] Features 属性或特征名称元组；调用期间借用，不转移所有权。
   * @param[in] Operation 多个特征条件的组合方式；调用期间借用，不转移所有权。
   * @param[in] Min 筛选或测量范围界限元组；调用期间借用，不转移所有权。
   * @param[in] Max 筛选或测量范围界限元组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_shape
   */
  MvRegion SelectShape(const MvTuple& Features, const MvString& Operation, const MvTuple& Min,
                       const MvTuple& Max) const;

  /**
   * @brief 按形状特征筛选 Region 对象。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectShape(const MvString& Features, const MvString& Operation, double Min, double Max) const;

  /**
   * @brief 按形状特征筛选 Region 对象。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectShape(const char* Features, const char* Operation, double Min, double Max) const;

  /**
   * @brief 按形状特征筛选 Region 对象。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectShape(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const;

  /**
   * @brief HALCON 字符串控制：SortMode、Order=true/false、RowOrCol=row/column。
   *
   * @param[in] SortMode 排序依据元组；调用期间借用，不转移所有权。
   * @param[in] Order 排序、通道或点序值；调用期间借用，不转移所有权。
   * @param[in] RowOrCol 分块或排序使用的行列方向；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_sort_region
   */
  MvRegion SortRegion(const MvTuple& SortMode, const MvString& Order, const MvString& RowOrCol) const;

  /**
   * @brief HALCON 字符串控制：SortMode、Order=true/false、RowOrCol=row/column。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SortRegion(const MvTuple& SortMode, const char* Order, const char* RowOrCol) const;

  /**
   * @brief HALCON 字符串控制：SortMode、Order=true/false、RowOrCol=row/column。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SortRegion(const MvTuple& SortMode, const wchar_t* Order, const wchar_t* RowOrCol) const;

  /**
   * @brief max_area 选择首个最大对象；Percent 必须有限，可超出 [0,100]，在 max_area 中忽略。
   *
   * @param[in] Shape 区域变换的目标形状；调用期间借用，不转移所有权。
   * @param[in] Percent 百分比值。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_shape_std
   */
  MvRegion SelectShapeStd(const MvString& Shape, double Percent) const;

  /**
   * @brief max_area 选择首个最大对象；Percent 必须有限，可超出 [0,100]，在 max_area 中忽略。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectShapeStd(const char* Shape, double Percent) const;

  /**
   * @brief max_area 选择首个最大对象；Percent 必须有限，可超出 [0,100]，在 max_area 中忽略。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvRegion SelectShapeStd(const wchar_t* Shape, double Percent) const;

  /**
   * @brief 将区域转换为指定的几何形状。
   *
   * @param[in] Type 数据、模型或原语类型；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_shape_trans
   */
  MvRegion ShapeTrans(const MvString& Type) const;

  /**
   * @brief 将区域转换为指定的几何形状。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion ShapeTrans(const char* Type) const;

  /**
   * @brief 将区域转换为指定的几何形状。
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion ShapeTrans(const wchar_t* Type) const;

  /**
   * @brief 将 Regions 中全部对象合并为一个 Region 对象。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_union1
   */
  MvRegion Union1() const;

  /**
   * @brief 按 Regions 语义执行并集。
   *
   * @param[in] Region2 右侧图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_union2
   */
  MvRegion Union2(const MvRegion& Region2) const;

  /**
   * @brief 按 Regions 语义执行交集。
   *
   * @param[in] Region2 右侧图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_intersection
   */
  MvRegion Intersection(const MvRegion& Region2) const;

  /**
   * @brief 按 Regions 语义执行差集。
   *
   * @param[in] Sub 右侧图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_difference
   */
  MvRegion Difference(const MvRegion& Sub) const;

  /**
   * @brief 将区域按连通性拆分为独立对象。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_connection
   */
  MvRegion Connection() const;

  /**
   * @brief 填充区域内部孔洞。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fill_up
   */
  MvRegion FillUp() const;

  /**
   * @brief 计算各 Region 对象的面积和重心。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 区域或轮廓面积元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_area_center
   */
  MvTuple AreaCenter(MvTuple* Row, MvTuple* Column) const;

  /**
   * @brief 计算各 Region 对象的面积和重心。
   * @overload 采用标量输出参数。
   *
   * @return 区域或轮廓面积。
   */
  std::int64_t AreaCenter(double* Row, double* Column) const;

  /**
   * @brief 计算各 Region 对象的最小轴对齐外接矩形。
   *
   * @param[out] Row1 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column1 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Row2 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column2 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_smallest_rectangle1
   */
  void SmallestRectangle1(MvTuple* Row1, MvTuple* Column1, MvTuple* Row2, MvTuple* Column2) const;

  /**
   * @brief 计算各 Region 对象的最小轴对齐外接矩形。
   * @overload 采用标量输出参数。
   *
   * @return 无返回值。
   */
  void SmallestRectangle1(std::int64_t* Row1, std::int64_t* Column1, std::int64_t* Row2,
                          std::int64_t* Column2) const;

  /**
   * @brief 计算各 Region 对象的最小旋转外接矩形。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Phi 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Length1 第一方向的几何半长元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Length2 第二方向的几何半长元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_smallest_rectangle2
   */
  void SmallestRectangle2(MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Length1,
                          MvTuple* Length2) const;

  /**
   * @brief 计算各 Region 对象的最小旋转外接矩形。
   * @overload 采用标量输出参数。
   *
   * @return 无返回值。
   */
  void SmallestRectangle2(double* Row, double* Column, double* Phi, double* Length1, double* Length2) const;

  /**
   * @brief 计算各 Region 对象的最小外接圆。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Radius 半径元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_smallest_circle
   */
  void SmallestCircle(MvTuple* Row, MvTuple* Column, MvTuple* Radius) const;

  /**
   * @brief 计算各 Region 对象的最小外接圆。
   * @overload 采用标量输出参数。
   *
   * @return 无返回值。
   */
  void SmallestCircle(double* Row, double* Column, double* Radius) const;

  /**
   * @brief min(1,area/(pi*max_center_distance^2))；单像素或空区域为零。
   *
   * @return 保存、追加或读取的数据元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_circularity
   */
  MvTuple Circularity() const;

  /**
   * @brief HALCON 按轮廓定向的主轴，弧度范围 [-pi,pi)；空区域为零。
   *
   * @return 保存、追加或读取的数据元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_orientation_region
   */
  MvTuple OrientationRegion() const;

  /**
   * @brief Anisometry=Ra/Rb，Bulkiness=pi*Ra*Rb/area，StructureFactor=Anisometry*Bulkiness-1。
   *
   * @param[out] Bulkiness 区域形状特征元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] StructureFactor 区域形状特征元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 区域形状特征元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_eccentricity
   */
  MvTuple Eccentricity(MvTuple* Bulkiness, MvTuple* StructureFactor) const;

  /**
   * @brief Anisometry=Ra/Rb，Bulkiness=pi*Ra*Rb/area，StructureFactor=Anisometry*Bulkiness-1。
   * @overload 采用标量输出参数。
   *
   * @return 区域形状特征。
   */
  double Eccentricity(double* Bulkiness, double* StructureFactor) const;

};

/**
 * @brief 将 Regions 中全部对象合并为一个 Region 对象。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionUnion 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_union1
 */
MVKIT_CPP_API void Union1(const MvObject& Region,MvObject* RegionUnion);

/**
 * @brief 按 Regions 语义执行并集。
 *
 * @param[in] Region1 左侧图标对象数组；调用期间借用，不转移所有权。
 * @param[in] Region2 右侧图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionUnion 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_union2
 */
MVKIT_CPP_API void Union2(const MvObject& Region1,const MvObject& Region2,MvObject* RegionUnion);

/**
 * @brief 按 Regions 语义执行交集。
 *
 * @param[in] Region1 左侧图标对象数组；调用期间借用，不转移所有权。
 * @param[in] Region2 右侧图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionIntersection 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_intersection
 */
MVKIT_CPP_API void Intersection(const MvObject& Region1,const MvObject& Region2,MvObject* RegionIntersection);

/**
 * @brief 按 Regions 语义执行差集。
 *
 * @param[in] Region 左侧图标对象数组；调用期间借用，不转移所有权。
 * @param[in] Sub 右侧图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionDifference 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_difference
 */
MVKIT_CPP_API void Difference(const MvObject& Region,const MvObject& Sub,MvObject* RegionDifference);

/**
 * @brief 将区域按连通性拆分为独立对象。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] ConnectedRegions 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_connection
 */
MVKIT_CPP_API void Connection(const MvObject& Region,MvObject* ConnectedRegions);

/**
 * @brief 填充区域内部孔洞。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionFillUp 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fill_up
 */
MVKIT_CPP_API void FillUp(const MvObject& Region,MvObject* RegionFillUp);

/**
 * @brief 计算各 Region 对象的面积和重心。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Area 区域或轮廓面积元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_area_center
 */
MVKIT_CPP_API void AreaCenter(const MvObject& Regions,MvTuple* Area,MvTuple* Row,MvTuple* Column);

/**
 * @brief 计算各 Region 对象的最小轴对齐外接矩形。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Row1 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column1 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Row2 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column2 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_smallest_rectangle1
 */
MVKIT_CPP_API void SmallestRectangle1(const MvObject& Regions,MvTuple* Row1,MvTuple* Column1,MvTuple* Row2,MvTuple* Column2);

/**
 * @brief 计算各 Region 对象的最小旋转外接矩形。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Phi 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Length1 第一方向的几何半长元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Length2 第二方向的几何半长元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_smallest_rectangle2
 */
MVKIT_CPP_API void SmallestRectangle2(const MvObject& Regions,MvTuple* Row,MvTuple* Column,MvTuple* Phi,MvTuple* Length1,MvTuple* Length2);

/**
 * @brief 计算各 Region 对象的最小外接圆。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Radius 半径元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_smallest_circle
 */
MVKIT_CPP_API void SmallestCircle(const MvObject& Regions,MvTuple* Row,MvTuple* Column,MvTuple* Radius);

/**
 * @brief min(1,area/(pi*max_center_distance^2))；单像素或空区域为零。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Circularity 保存、追加或读取的数据元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_circularity
 */
MVKIT_CPP_API void Circularity(const MvObject& Regions,MvTuple* Circularity);

/**
 * @brief HALCON 按轮廓定向的主轴，弧度范围 [-pi,pi)；空区域为零。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Phi 保存、追加或读取的数据元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_orientation_region
 */
MVKIT_CPP_API void OrientationRegion(const MvObject& Regions,MvTuple* Phi);

/**
 * @brief Anisometry=Ra/Rb，Bulkiness=pi*Ra*Rb/area，StructureFactor=Anisometry*Bulkiness-1。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Anisometry 区域形状特征元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Bulkiness 区域形状特征元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] StructureFactor 区域形状特征元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_eccentricity
 */
MVKIT_CPP_API void Eccentricity(const MvObject& Regions,MvTuple* Anisometry,MvTuple* Bulkiness,MvTuple* StructureFactor);

/**
 * @brief 生成轴对齐矩形区域。
 *
 * @param[out] Rectangle 矩形区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_rectangle1
 */
MVKIT_CPP_API void GenRectangle1(MvObject* Rectangle, const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2);

/**
 * @brief 生成旋转矩形区域。
 *
 * @param[out] Rectangle 矩形区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Length1 第一方向的几何半长元组；调用期间借用，不转移所有权。
 * @param[in] Length2 第二方向的几何半长元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_rectangle2
 */
MVKIT_CPP_API void GenRectangle2(MvObject* Rectangle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Length1, const MvTuple& Length2);

/**
 * @brief 生成圆形区域。
 *
 * @param[out] Circle 圆形区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_circle
 */
MVKIT_CPP_API void GenCircle(MvObject* Circle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius);

/**
 * @brief 使用圆形结构元素膨胀区域，Radius 为 [0.5,511.5] 中的一个数值。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionDilation 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_dilation_circle
 */
MVKIT_CPP_API void DilationCircle(const MvObject& Region, MvObject* RegionDilation, const MvTuple& Radius);

/**
 * @brief 使用圆形结构元素腐蚀区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionErosion 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_erosion_circle
 */
MVKIT_CPP_API void ErosionCircle(const MvObject& Region, MvObject* RegionErosion, const MvTuple& Radius);

/**
 * @brief 使用圆形结构元素执行开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionOpening 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_opening_circle
 */
MVKIT_CPP_API void OpeningCircle(const MvObject& Region, MvObject* RegionOpening, const MvTuple& Radius);

/**
 * @brief 使用圆形结构元素执行闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionClosing 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_closing_circle
 */
MVKIT_CPP_API void ClosingCircle(const MvObject& Region, MvObject* RegionClosing, const MvTuple& Radius);

/**
 * @brief 使用矩形结构元素膨胀区域，Width/Height 各为一个数值并取最近整数。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionDilation 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_dilation_rectangle1
 */
MVKIT_CPP_API void DilationRectangle1(const MvObject& Region, MvObject* RegionDilation, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 使用矩形结构元素腐蚀区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionErosion 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_erosion_rectangle1
 */
MVKIT_CPP_API void ErosionRectangle1(const MvObject& Region, MvObject* RegionErosion, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 使用矩形结构元素执行开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionOpening 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_opening_rectangle1
 */
MVKIT_CPP_API void OpeningRectangle1(const MvObject& Region, MvObject* RegionOpening, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 使用矩形结构元素执行闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionClosing 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_closing_rectangle1
 */
MVKIT_CPP_API void ClosingRectangle1(const MvObject& Region, MvObject* RegionClosing, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 按整数偏移平移每个区域；偏移和结果坐标须在有符号 32 位范围内，越界保留原输出。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionMoved 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_move_region
 */
MVKIT_CPP_API void MoveRegion(const MvObject& Region, MvObject* RegionMoved, const MvTuple& Row, const MvTuple& Column);

/**
 * @brief 每个左侧对象与全部右侧对象的并集异或，保持左侧对象数量。
 *
 * @param[in] Region1 输入区域；调用期间借用，不转移所有权。
 * @param[in] Region2 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionDifference 区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_symm_difference
 */
MVKIT_CPP_API void SymmDifference(const MvObject& Region1, const MvObject& Region2, MvObject* RegionDifference);

/**
 * @brief 返回一个 Int64 布尔值，判断对象数相同且逐个像素集合相等。
 *
 * @param[in] Regions1 输入区域；调用期间借用，不转移所有权。
 * @param[in] Regions2 输入区域；调用期间借用，不转移所有权。
 * @param[out] IsEqual 几何关系判定标记元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_test_equal_region
 */
MVKIT_CPP_API void TestEqualRegion(const MvObject& Regions1, const MvObject& Regions2, MvTuple* IsEqual);

/**
 * @brief 任一点命中判断：一个点配 N 个区域，或 M 个点配一个区域。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[out] IsInside 几何关系判定标记元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_test_region_point
 */
MVKIT_CPP_API void TestRegionPoint(const MvObject& Regions, const MvTuple& Row, const MvTuple& Column, MvTuple* IsInside);

/**
 * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[in] Features 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[out] Value 保存、追加或读取的数据元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_region_features
 */
MVKIT_CPP_API void RegionFeatures(const MvObject& Regions, const MvTuple& Features, MvTuple* Value);

/**
 * @brief 按形状特征筛选 Region 对象。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] SelectedRegions 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Features 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] Operation 多个特征条件的组合方式元组；调用期间借用，不转移所有权。
 * @param[in] Min 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Max 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_shape
 */
MVKIT_CPP_API void SelectShape(const MvObject& Regions, MvObject* SelectedRegions, const MvTuple& Features, const MvTuple& Operation, const MvTuple& Min, const MvTuple& Max);

/**
 * @brief HALCON 字符串控制：SortMode、Order=true/false、RowOrCol=row/column。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] SortedRegions 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] SortMode 排序依据元组；调用期间借用，不转移所有权。
 * @param[in] Order 排序、通道或点序控制元组；调用期间借用，不转移所有权。
 * @param[in] RowOrCol 分块或排序使用的行列方向元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_sort_region
 */
MVKIT_CPP_API void SortRegion(const MvObject& Regions, MvObject* SortedRegions, const MvTuple& SortMode, const MvTuple& Order, const MvTuple& RowOrCol);

/**
 * @brief max_area 选择首个最大对象；Percent 必须有限，可超出 [0,100]，在 max_area 中忽略。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] SelectedRegions 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Shape 区域变换的目标形状元组；调用期间借用，不转移所有权。
 * @param[in] Percent 百分比控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_shape_std
 */
MVKIT_CPP_API void SelectShapeStd(const MvObject& Regions, MvObject* SelectedRegions, const MvTuple& Shape, const MvTuple& Percent);

/**
 * @brief 将区域转换为指定的几何形状。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionTrans 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Type 数据、模型或原语类型元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_shape_trans
 */
MVKIT_CPP_API void ShapeTrans(const MvObject& Region, MvObject* RegionTrans, const MvTuple& Type);

/**
 * @brief 填充闭合多边形，使用 even-odd 内部规则并包含像素中心边界。
 *
 * @param[out] Region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Rows 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Columns 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_region_polygon_filled
 */
MVKIT_CPP_API void GenRegionPolygonFilled(MvObject* Region, const MvTuple& Rows, const MvTuple& Columns);

/**
 * @brief Zhang-Suen 候选细化并逐步检查拓扑，前景八连通、背景四连通。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] Skeleton 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_skeleton
 */
MVKIT_CPP_API void Skeleton(const MvObject& Region, MvObject* Skeleton);

/**
 * @brief 按对象均衡网格分块，Width/Height 为有限正标量。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] Partitioned 分块区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_partition_rectangle
 */
MVKIT_CPP_API void PartitionRectangle(const MvObject& Region, MvObject* Partitioned, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 均衡列切分向最小列面积移动，Distance>=1，Percent 在 [0,100] 内。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] Partitioned 分块区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Distance 距离元组；调用期间借用，不转移所有权。
 * @param[in] Percent 百分比控制元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_partition_dynamic
 */
MVKIT_CPP_API void PartitionDynamic(const MvObject& Region, MvObject* Partitioned, const MvTuple& Distance, const MvTuple& Percent);

/**
 * @brief 使用 scalar-or-N 数值控制批量生成椭圆。
 *
 * @param[out] Ellipse 椭圆区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Radius1 椭圆长半轴元组；调用期间借用，不转移所有权。
 * @param[in] Radius2 椭圆短半轴元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_ellipse
 */
MVKIT_CPP_API void GenEllipse(MvObject* Ellipse, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2);

/**
 * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionAffineTrans 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Interpolate 插值方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_region
 */
MVKIT_CPP_API void AffineTransRegion(const MvObject& Region, MvObject* RegionAffineTrans, const MvTuple& HomMat2D, const MvTuple& Interpolate);

/**
 * @brief BoundaryType 为 inner/inner_filled/outer。inner/outer 使用八邻域形态学；
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionBorder 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] BoundaryType 区域边界提取方式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_boundary
 */
MVKIT_CPP_API void Boundary(const MvObject& Region, MvObject* RegionBorder, const MvTuple& BoundaryType);

/**
 * @brief 生成一个空区域，与零对象数组不同。
 *
 * @param[out] EmptyRegion 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_empty_region
 */
MVKIT_CPP_API void GenEmptyRegion(MvObject* EmptyRegion);

/**
 * @brief 由等长 Row/ColumnBegin/ColumnEnd 元组生成一个区域，栅格坐标须在有符号 32 位范围内。
 *
 * @param[out] Region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 各游程的行坐标；调用期间借用，不转移所有权。
 * @param[in] ColumnBegin 各游程的起始列坐标；调用期间借用，不转移所有权。
 * @param[in] ColumnEnd 各游程的结束列坐标；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_region_runs
 */
MVKIT_CPP_API void GenRegionRuns(MvObject* Region, const MvTuple& Row, const MvTuple& ColumnBegin, const MvTuple& ColumnEnd);

/**
 * @brief 将单区域读入三个独立拥有型整数元组，按行及起始列排序。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ColumnBegin 起点列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ColumnEnd 终点列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_region_runs
 */
MVKIT_CPP_API void GetRegionRuns(const MvObject& Region, MvTuple* Row, MvTuple* ColumnBegin, MvTuple* ColumnEnd);

/**
 * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] RegionFillUp 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Feature 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] Min 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Max 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fill_up_shape
 */
MVKIT_CPP_API void FillUpShape(const MvObject& Region, MvObject* RegionFillUp, const MvTuple& Feature, const MvTuple& Min, const MvTuple& Max);

/**
 * @brief 每个区域返回一张 Int32 图像。Metric 为 euclidean/city-block/chessboard，
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[out] DistanceImage 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Metric 距离或匹配度量元组；调用期间借用，不转移所有权。
 * @param[in] Foreground 距离变换的前景选择元组；调用期间借用，不转移所有权。
 * @param[in] Width 图像或分块宽度元组；调用期间借用，不转移所有权。
 * @param[in] Height 图像或分块高度元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_transform
 */
MVKIT_CPP_API void DistanceTransform(const MvObject& Region, MvObject* DistanceImage, const MvTuple& Metric, const MvTuple& Foreground, const MvTuple& Width, const MvTuple& Height);

/**
 * @brief 位置无关的自定义形态学，使用一个结构元素对象，参考点为取整质心。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionDilation 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_dilation1
 */
MVKIT_CPP_API void Dilation1(const MvObject& Region, const MvObject& StructElement, MvObject* RegionDilation, const MvTuple& Iterations);

/**
 * @brief 使用位置无关的结构区域腐蚀输入区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionErosion 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_erosion1
 */
MVKIT_CPP_API void Erosion1(const MvObject& Region, const MvObject& StructElement, MvObject* RegionErosion, const MvTuple& Iterations);

/**
 * @brief 使用位置无关的结构区域执行区域开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionOpening 区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_opening
 */
MVKIT_CPP_API void Opening(const MvObject& Region, const MvObject& StructElement, MvObject* RegionOpening);

/**
 * @brief 使用位置无关的结构区域执行区域闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionClosing 区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_closing
 */
MVKIT_CPP_API void Closing(const MvObject& Region, const MvObject& StructElement, MvObject* RegionClosing);

/**
 * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
MVKIT_CPP_API void MinMaxGray(const MvObject& Regions, const MvObject& Image, const MvTuple& Percent,
                              MvTuple* Min, MvTuple* Max, MvTuple* Range);

/**
 * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] ImageResult 图像；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Grayval 灰度值元组；调用期间借用，不转移所有权。
 * @param[in] Type 数据、模型或原语类型元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_paint_region
 */
MVKIT_CPP_API void PaintRegion(const MvObject& Region, const MvObject& Image, MvObject* ImageResult,
                               const MvTuple& Grayval, const MvTuple& Type);

/**
 * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
 *
 * @param[in] region 输入区域；调用期间借用，不转移所有权。
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] mean 灰度均值或标准差元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] deviation 灰度均值或标准差元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_intensity
 */
MVKIT_CPP_API void Intensity(const MvObject& region, const MvObject& image, MvTuple* mean,
                             MvTuple* deviation);

/**
 * @brief 单个 Region、单个单通道图像 的 256-bin 直方图，支持五种像素类型。
 *
 * @param[in] region 输入区域；调用期间借用，不转移所有权。
 * @param[in] image 输入图像；调用期间借用，不转移所有权。
 * @param[out] absolute 各灰度桶的像素计数元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] relative 归一化灰度直方图元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gray_histo
 */
MVKIT_CPP_API void GrayHisto(const MvObject& region, const MvObject& image, MvTuple* absolute,
                             MvTuple* relative);

/**
 * @brief 使用位置无关结构区域的区域顶帽/底帽。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionTopHat 区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_top_hat
 */
MVKIT_CPP_API void TopHat(const MvObject& Region, const MvObject& StructElement, MvObject* RegionTopHat);

/**
 * @brief 计算区域闭运算结果与原区域之差。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] RegionBottomHat 区域；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_bottom_hat
 */
MVKIT_CPP_API void BottomHat(const MvObject& Region, const MvObject& StructElement,
                             MvObject* RegionBottomHat);

}
