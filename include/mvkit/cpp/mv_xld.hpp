#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {

/**
 * @brief 共享 XLD 图标对象的基类。
 *
 * @details 当前实现支持 XLD 轮廓。
 */
class MVKIT_CPP_API MvXLD : public MvObject {
 public:
  /**
   * @brief 逐轮廓判断首尾坐标是否完全相等，不使用私有闭合元数据。
   *
   * @return 轮廓闭合标记元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_test_closed_xld
   */
  MvTuple TestClosedXld() const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
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
   * @see mv_select_shape_xld
   */
  MvXLD SelectShapeXld(const MvTuple& Features, const MvString& Operation, const MvTuple& Min,
                       const MvTuple& Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLD SelectShapeXld(const MvString& Features, const MvString& Operation, double Min, double Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLD SelectShapeXld(const char* Features, const char* Operation, double Min, double Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLD SelectShapeXld(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const;

  /**
   * @brief 构造有效的空图标对象数组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLD();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLD(const MvXLD&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvXLD(MvXLD&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLD& operator=(const MvXLD&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLD& operator=(MvXLD&&);

  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvXLD(const MvObject& objects);

  /**
   * @brief 每条轮廓一个长度；有效空数组返回空控制元组，结果由 C++ 对象自动管理。
   *
   * @return 几何长度或半长元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_length_xld
   */
  MvTuple LengthXld() const;

  /**
   * @brief 连续面积/质心，自动闭合，四个输出对象必须互异。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 区域或轮廓面积元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_area_center_xld
   */
  MvTuple AreaCenterXld(MvTuple* Row, MvTuple* Column, MvTuple* PointOrder) const;

  /**
   * @brief 连续面积/质心，自动闭合，四个输出对象必须互异。
   * @overload 采用标量输出参数。
   *
   * @return 区域或轮廓面积。
   */
  double AreaCenterXld(double* Row, double* Column, MvString* PointOrder) const;

};

/**
 * @brief 仅包含 XLD 轮廓的图标对象数组。
 */
class MVKIT_CPP_API MvXLDCont : public MvXLD {
 public:
  /**
   * @brief 将单轮廓坐标读入独立拥有型 Row/Col 元组。
   *
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Col 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_contour_xld
   */
  void GetContourXld(MvTuple* Row, MvTuple* Col) const;

  /**
   * @brief 读取一个点属性：response、edge_direction 或 angle。
   *
   * @param[in] Name 参数或数据项名称；调用期间借用，不转移所有权。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_contour_attrib_xld
   */
  MvTuple GetContourAttribXld(const MvString& Name) const;

  /**
   * @brief 读取一个点属性：response、edge_direction 或 angle。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   */
  MvTuple GetContourAttribXld(const char* Name) const;

  /**
   * @brief 读取一个点属性：response、edge_direction 或 angle。
   * @overload 采用宽字符字符串参数。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   */
  MvTuple GetContourAttribXld(const wchar_t* Name) const;

  /**
   * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
   *
   * @param[in] Name 参数或数据项名称元组；调用期间借用，不转移所有权。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_contour_global_attrib_xld
   */
  MvTuple GetContourGlobalAttribXld(const MvTuple& Name) const;

  /**
   * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
   * @overload 采用字符串对象参数。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   */
  MvTuple GetContourGlobalAttribXld(const MvString& Name) const;

  /**
   * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   */
  MvTuple GetContourGlobalAttribXld(const char* Name) const;

  /**
   * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
   * @overload 采用宽字符字符串参数。
   *
   * @return 属性或特征名称元组，由返回对象自动管理资源。
   */
  MvTuple GetContourGlobalAttribXld(const wchar_t* Name) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   *
   * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
   * @param[in] WorldPose 世界平面位姿；调用期间借用，不转移所有权。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_contour_to_world_plane_xld
   */
  MvXLDCont ContourToWorldPlaneXld(const MvTuple& CameraParam, const MvPose& WorldPose,
                                   const MvTuple& Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用字符串对象参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvTuple& CameraParam, const MvPose& WorldPose,
                                   const MvString& Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvTuple& CameraParam, const MvPose& WorldPose,
                                   const char* Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用宽字符字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvTuple& CameraParam, const MvPose& WorldPose,
                                   const wchar_t* Scale) const;

  /**
   * @brief 按相机参数改变轮廓点径向畸变；点属性不保留。
   *
   * @param[in] CamParamIn 源相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] CamParamOut 目标相机内参与畸变参数；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_change_radial_distortion_contours_xld
   */
  MvXLDCont ChangeRadialDistortionContoursXld(const MvCamPar& CamParamIn, const MvCamPar& CamParamOut) const;

  /**
   * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
   *
   * @param[in] Algorithm 拟合算法；调用期间借用，不转移所有权。
   * @param[in] MaxNumPoints 拟合使用的最大点数量。
   * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量。
   * @param[in] Iterations 拟合或优化迭代次数。
   * @param[in] ClippingFactor 鲁棒拟合的裁剪因子。
   * @param[out] RowBegin 起点行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ColBegin 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] RowEnd 终点行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] ColEnd 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Nr 直线单位法向的行、列分量元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Nc 直线单位法向的行、列分量元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Dist 直线的法向距离元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fit_line_contour_xld
   */
  void FitLineContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints,
                         std::int64_t Iterations, double ClippingFactor, MvTuple* RowBegin, MvTuple* ColBegin,
                         MvTuple* RowEnd, MvTuple* ColEnd, MvTuple* Nr, MvTuple* Nc, MvTuple* Dist) const;

  /**
   * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FitLineContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints,
                         std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin,
                         double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const;

  /**
   * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitLineContourXld(const char* Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints,
                         std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin,
                         double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const;

  /**
   * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitLineContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints,
                         std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin,
                         double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const;

  /**
   * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
   *
   * @param[in] Algorithm 拟合算法；调用期间借用，不转移所有权。
   * @param[in] MaxNumPoints 拟合使用的最大点数量。
   * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离。
   * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量。
   * @param[in] Iterations 拟合或优化迭代次数。
   * @param[in] ClippingFactor 鲁棒拟合的裁剪因子。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Radius 半径元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fit_circle_contour_xld
   */
  void FitCircleContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                           std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor,
                           MvTuple* Row, MvTuple* Column, MvTuple* Radius, MvTuple* StartPhi, MvTuple* EndPhi,
                           MvTuple* PointOrder) const;

  /**
   * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FitCircleContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                           std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor,
                           double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi,
                           MvString* PointOrder) const;

  /**
   * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitCircleContourXld(const char* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                           std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor,
                           double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi,
                           MvString* PointOrder) const;

  /**
   * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitCircleContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                           std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor,
                           double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi,
                           MvString* PointOrder) const;

  /**
   * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
   *
   * @param[in] Algorithm 拟合算法；调用期间借用，不转移所有权。
   * @param[in] MaxNumPoints 拟合使用的最大点数量。
   * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离。
   * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量。
   * @param[in] VossTabSize 椭圆拟合控制参数。
   * @param[in] Iterations 拟合或优化迭代次数。
   * @param[in] ClippingFactor 鲁棒拟合的裁剪因子。
   * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Phi 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Radius1 椭圆长半轴元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Radius2 椭圆短半轴元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_fit_ellipse_contour_xld
   */
  void FitEllipseContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                            std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations,
                            double ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Phi,
                            MvTuple* Radius1, MvTuple* Radius2, MvTuple* StartPhi, MvTuple* EndPhi,
                            MvTuple* PointOrder) const;

  /**
   * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void FitEllipseContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                            std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations,
                            double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1,
                            double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const;

  /**
   * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitEllipseContourXld(const char* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                            std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations,
                            double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1,
                            double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const;

  /**
   * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void FitEllipseContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist,
                            std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations,
                            double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1,
                            double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const;

  /**
   * @brief 将 XLD 轮廓分割为几何段。
   *
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   * @param[in] SmoothCont 分段前的轮廓平滑值。
   * @param[in] MaxLineDist1 分段拟合的距离阈值。
   * @param[in] MaxLineDist2 分段拟合的距离阈值。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_segment_contours_xld
   */
  MvXLDCont SegmentContoursXld(const MvString& Mode, std::int64_t SmoothCont, double MaxLineDist1,
                               double MaxLineDist2) const;

  /**
   * @brief 将 XLD 轮廓分割为几何段。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont SegmentContoursXld(const char* Mode, std::int64_t SmoothCont, double MaxLineDist1,
                               double MaxLineDist2) const;

  /**
   * @brief 将 XLD 轮廓分割为几何段。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont SegmentContoursXld(const wchar_t* Mode, std::int64_t SmoothCont, double MaxLineDist1,
                               double MaxLineDist2) const;

  /**
   * @brief 按轮廓特征筛选 XLD。
   *
   * @param[in] Feature 属性或特征名称；调用期间借用，不转移所有权。
   * @param[in] Min1 筛选或测量范围界限。
   * @param[in] Max1 筛选或测量范围界限。
   * @param[in] Min2 筛选或测量范围界限。
   * @param[in] Max2 筛选或测量范围界限。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_contours_xld
   */
  MvXLDCont SelectContoursXld(const MvString& Feature, double Min1, double Max1, double Min2,
                              double Max2) const;

  /**
   * @brief 按轮廓特征筛选 XLD。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectContoursXld(const char* Feature, double Min1, double Max1, double Min2, double Max2) const;

  /**
   * @brief 按轮廓特征筛选 XLD。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectContoursXld(const wchar_t* Feature, double Min1, double Max1, double Min2,
                              double Max2) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
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
   * @see mv_select_shape_xld
   */
  MvXLDCont SelectShapeXld(const MvTuple& Features, const MvString& Operation, const MvTuple& Min,
                           const MvTuple& Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectShapeXld(const MvString& Features, const MvString& Operation, double Min, double Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectShapeXld(const char* Features, const char* Operation, double Min, double Max) const;

  /**
   * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectShapeXld(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const;

  /**
   * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
   *
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Col1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Col2 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] CloseContours 是否连接裁剪后轮廓的端点；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_crop_contours_xld
   */
  MvXLDCont CropContoursXld(const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Row2,
                            const MvTuple& Col2, const MvString& CloseContours) const;

  /**
   * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont CropContoursXld(double Row1, double Col1, double Row2, double Col2,
                            const MvString& CloseContours) const;

  /**
   * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont CropContoursXld(double Row1, double Col1, double Row2, double Col2,
                            const char* CloseContours) const;

  /**
   * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont CropContoursXld(double Row1, double Col1, double Row2, double Col2,
                            const wchar_t* CloseContours) const;

  /**
   * @brief SortMode 为 upper_left/upper_right/lower_left/lower_right/character；
   *
   * @param[in] SortMode 排序依据；调用期间借用，不转移所有权。
   * @param[in] Order 排序、通道或点序值；调用期间借用，不转移所有权。
   * @param[in] RowOrCol 分块或排序使用的行列方向；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_sort_contours_xld
   */
  MvXLDCont SortContoursXld(const MvString& SortMode, const MvString& Order, const MvString& RowOrCol) const;

  /**
   * @brief SortMode 为 upper_left/upper_right/lower_left/lower_right/character；
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SortContoursXld(const char* SortMode, const char* Order, const char* RowOrCol) const;

  /**
   * @brief SortMode 为 upper_left/upper_right/lower_left/lower_right/character；
   * @overload 采用宽字符字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SortContoursXld(const wchar_t* SortMode, const wchar_t* Order, const wchar_t* RowOrCol) const;

  /**
   * @brief 合并开放非退化轮廓。距离使用回归线投影；至少一个参考同时满足 MaxDistAbs/MaxDistRel。
   *
   * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值。
   * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值。
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_union_adjacent_contours_xld
   */
  MvXLDCont UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const MvString& Mode) const;

  /**
   * @brief 合并开放非退化轮廓。距离使用回归线投影；至少一个参考同时满足 MaxDistAbs/MaxDistRel。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const char* Mode) const;

  /**
   * @brief 合并开放非退化轮廓。距离使用回归线投影；至少一个参考同时满足 MaxDistAbs/MaxDistRel。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const wchar_t* Mode) const;

  /**
   * @brief 按距离、偏移和方向阈值合并共线轮廓。
   *
   * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值。
   * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值。
   * @param[in] MaxShift 共线轮廓的最大侧向偏移。
   * @param[in] MaxAngle 轮廓合并的角度阈值。
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_union_collinear_contours_xld
   */
  MvXLDCont UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle,
                                      const MvString& Mode) const;

  /**
   * @brief 按距离、偏移和方向阈值合并共线轮廓。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle,
                                      const char* Mode) const;

  /**
   * @brief 按距离、偏移和方向阈值合并共线轮廓。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle,
                                      const wchar_t* Mode) const;

  /**
   * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
   *
   * @param[in] MaxArcAngleDiff 轮廓合并的角度阈值元组；调用期间借用，不转移所有权。
   * @param[in] MaxArcOverlap 圆弧允许的最大重叠角度元组；调用期间借用，不转移所有权。
   * @param[in] MaxTangentAngle 轮廓合并的角度阈值元组；调用期间借用，不转移所有权。
   * @param[in] MaxDist 距离阈值元组；调用期间借用，不转移所有权。
   * @param[in] MaxRadiusDiff 圆弧中心或半径差的阈值元组；调用期间借用，不转移所有权。
   * @param[in] MaxCenterDist 圆弧中心或半径差的阈值元组；调用期间借用，不转移所有权。
   * @param[in] MergeSmallContours 是否合并短轮廓；调用期间借用，不转移所有权。
   * @param[in] Iterations 拟合或优化迭代次数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_union_cocircular_contours_xld
   */
  MvXLDCont UnionCocircularContoursXld(const MvTuple& MaxArcAngleDiff, const MvTuple& MaxArcOverlap,
                                       const MvTuple& MaxTangentAngle, const MvTuple& MaxDist,
                                       const MvTuple& MaxRadiusDiff, const MvTuple& MaxCenterDist,
                                       const MvString& MergeSmallContours, std::int64_t Iterations) const;

  /**
   * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle,
                                       double MaxDist, double MaxRadiusDiff, double MaxCenterDist,
                                       const MvString& MergeSmallContours, std::int64_t Iterations) const;

  /**
   * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle,
                                       double MaxDist, double MaxRadiusDiff, double MaxCenterDist,
                                       const char* MergeSmallContours, std::int64_t Iterations) const;

  /**
   * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle,
                                       double MaxDist, double MaxRadiusDiff, double MaxCenterDist,
                                       const wchar_t* MergeSmallContours, std::int64_t Iterations) const;

  /**
   * @brief 由等长度、非空的 Row/Col 数值元组 生成一个 XLD 轮廓。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Col 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_contour_polygon_xld
   */
  void GenContourPolygonXld(const MvTuple& Row, const MvTuple& Col);

  /**
   * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
   * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
   * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
   * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间借用，不转移所有权。
   * @param[in] Resolution 轮廓采样分辨率。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_circle_contour_xld
   */
  void GenCircleContourXld(const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius,
                           const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder,
                           double Resolution);

  /**
   * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi,
                           const MvString& PointOrder, double Resolution);

  /**
   * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi,
                           const char* PointOrder, double Resolution);

  /**
   * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi,
                           const wchar_t* PointOrder, double Resolution);

  /**
   * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Radius1 椭圆长半轴元组；调用期间借用，不转移所有权。
   * @param[in] Radius2 椭圆短半轴元组；调用期间借用，不转移所有权。
   * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
   * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
   * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间借用，不转移所有权。
   * @param[in] Resolution 轮廓采样分辨率。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_ellipse_contour_xld
   */
  void GenEllipseContourXld(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi,
                            const MvTuple& Radius1, const MvTuple& Radius2, const MvTuple& StartPhi,
                            const MvTuple& EndPhi, const MvTuple& PointOrder, double Resolution);

  /**
   * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2,
                            double StartPhi, double EndPhi, const MvString& PointOrder, double Resolution);

  /**
   * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2,
                            double StartPhi, double EndPhi, const char* PointOrder, double Resolution);

  /**
   * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 无返回值。
   */
  void GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2,
                            double StartPhi, double EndPhi, const wchar_t* PointOrder, double Resolution);

  /**
   * @brief 每条轮廓一个输出。端点半值远离零取整，自动闭合，固定八连通 Bresenham 边界。
   *
   * @param[in] Mode 处理或查询模式；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_region_contour_xld
   */
  MvRegion GenRegionContourXld(const MvString& Mode) const;

  /**
   * @brief 每条轮廓一个输出。端点半值远离零取整，自动闭合，固定八连通 Bresenham 边界。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion GenRegionContourXld(const char* Mode) const;

  /**
   * @brief 每条轮廓一个输出。端点半值远离零取整，自动闭合，固定八连通 Bresenham 边界。
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion GenRegionContourXld(const wchar_t* Mode) const;

  /**
   * @brief HALCON 像素角点约定：HomMat2D 作用于 (row+.5,column+.5)，变换后再减 .5。
   *
   * @param[in] HomMat2D 二维变换矩阵；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_contour_xld
   */
  MvXLDCont AffineTransContourXld(const MvHomMat2D& HomMat2D) const;

  /**
   * @brief 使用投影变换矩阵变换 XLD 轮廓。
   *
   * @param[in] HomMat2D 二维变换矩阵；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_projective_trans_contour_xld
   */
  MvXLDCont ProjectiveTransContourXld(const MvHomMat2D& HomMat2D) const;

  /**
   * @brief 必要时追加首点实现显式闭合，同时复制首点属性。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_close_contours_xld
   */
  MvXLDCont CloseContoursXld() const;

  /**
   * @brief 查询首个轮廓到点的线段最小距离和最大距离。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[out] DistanceMin 最小距离元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] DistanceMax 最大距离元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_pc
   */
  void DistancePc(const MvTuple& Row, const MvTuple& Column, MvTuple* DistanceMin,
                  MvTuple* DistanceMax) const;

  /**
   * @brief 查询首个轮廓到点的线段最小距离和最大距离。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void DistancePc(double Row, double Column, double* DistanceMin, double* DistanceMax) const;

  /**
   * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
   *
   * @param[in] Contour2 输入轮廓；调用期间借用，不转移所有权。
   * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
   * @param[out] DistanceMin 最小距离元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] DistanceMax 最大距离元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_cc
   */
  void DistanceCc(const MvXLDCont& Contour2, const MvTuple& Mode, MvTuple* DistanceMin,
                  MvTuple* DistanceMax) const;

  /**
   * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
   * @overload 采用字符串对象及标量输出参数。
   *
   * @return 无返回值。
   */
  void DistanceCc(const MvXLDCont& Contour2, const MvString& Mode, double* DistanceMin,
                  double* DistanceMax) const;

  /**
   * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void DistanceCc(const MvXLDCont& Contour2, const char* Mode, double* DistanceMin,
                  double* DistanceMax) const;

  /**
   * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void DistanceCc(const MvXLDCont& Contour2, const wchar_t* Mode, double* DistanceMin,
                  double* DistanceMax) const;

  /**
   * @brief 使用局部回归平滑；NumRegrPoints 为单个不小于 3 的奇数。
   *
   * @param[in] NumRegrPoints 局部回归窗口的点数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_smooth_contours_xld
   */
  MvXLDCont SmoothContoursXld(std::int64_t NumRegrPoints) const;

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
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
   * @param[in] MinContrast 模板对比度阈值。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_scaled_shape_model_xld
   */
  MvShapeModel CreateScaledShapeModelXld(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                                         const MvTuple& AngleStep, double ScaleMin, double ScaleMax,
                                         const MvTuple& ScaleStep, const MvTuple& Optimization,
                                         const MvString& Metric, std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                         double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                         const MvString& Optimization, const MvString& Metric,
                                         std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                         double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                         const char* Optimization, const char* Metric,
                                         std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建支持尺度搜索的形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateScaledShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                         double AngleStep, double ScaleMin, double ScaleMax, double ScaleStep,
                                         const wchar_t* Optimization, const wchar_t* Metric,
                                         std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   *
   * @param[in] NumLevels 金字塔层级控制元组；调用期间借用，不转移所有权。
   * @param[in] AngleStart 起始角度。
   * @param[in] AngleExtent 角度区间长度。
   * @param[in] AngleStep 角度采样步长元组；调用期间借用，不转移所有权。
   * @param[in] Optimization 模型优化方式元组；调用期间借用，不转移所有权。
   * @param[in] Metric 距离或匹配度量；调用期间借用，不转移所有权。
   * @param[in] MinContrast 模板对比度阈值。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_shape_model_xld
   */
  MvShapeModel CreateShapeModelXld(const MvTuple& NumLevels, double AngleStart, double AngleExtent,
                                   const MvTuple& AngleStep, const MvTuple& Optimization,
                                   const MvString& Metric, std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                   double AngleStep, const MvString& Optimization, const MvString& Metric,
                                   std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                   double AngleStep, const char* Optimization, const char* Metric,
                                   std::int64_t MinContrast) const;

  /**
   * @brief 根据 XLD 模板轮廓创建形状匹配模型。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 形状匹配模型句柄，由返回对象自动管理资源。
   */
  MvShapeModel CreateShapeModelXld(std::int64_t NumLevels, double AngleStart, double AngleExtent,
                                   double AngleStep, const wchar_t* Optimization, const wchar_t* Metric,
                                   std::int64_t MinContrast) const;

  /**
   * @brief 构造有效的空图标对象数组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLDCont();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLDCont(const MvXLDCont&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvXLDCont(MvXLDCont&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLDCont& operator=(const MvXLDCont&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvXLDCont& operator=(MvXLDCont&&);

  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvXLDCont(const MvObject& objects);

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
  MvXLDCont SelectObj(const MvTuple& index) const;

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvXLDCont SelectObj(std::int64_t index) const;

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
  MvXLDCont CopyObj(std::int64_t index, std::int64_t count) const;

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
  MvXLDCont ConcatObj(const MvXLDCont& other) const;

};

/**
 * @brief 由等长度、非空的 Row/Col 数值元组 生成一个 XLD 轮廓。
 *
 * @param[out] Contour 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Col 列坐标元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_contour_polygon_xld
 */
MVKIT_CPP_API void GenContourPolygonXld(MvObject* Contour, const MvTuple& Row, const MvTuple& Col);

/**
 * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
 *
 * @param[out] ContCircle 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Radius 半径元组；调用期间借用，不转移所有权。
 * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
 * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
 * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间借用，不转移所有权。
 * @param[in] Resolution 轮廓采样分辨率元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_circle_contour_xld
 */
MVKIT_CPP_API void GenCircleContourXld(MvObject* ContCircle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, const MvTuple& Resolution);

/**
 * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
 *
 * @param[out] ContEllipse 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Radius1 椭圆长半轴元组；调用期间借用，不转移所有权。
 * @param[in] Radius2 椭圆短半轴元组；调用期间借用，不转移所有权。
 * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
 * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间借用，不转移所有权。
 * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间借用，不转移所有权。
 * @param[in] Resolution 轮廓采样分辨率元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_ellipse_contour_xld
 */
MVKIT_CPP_API void GenEllipseContourXld(MvObject* ContEllipse, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, const MvTuple& Resolution);

/**
 * @brief 从区域对象数组生成轮廓；Mode 为 center、border 或 border_holes。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移所有权。
 * @param[out] Contours 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_contour_region_xld
 */
MVKIT_CPP_API void GenContourRegionXld(const MvObject& Regions, MvObject* Contours, const MvTuple& Mode);

/**
 * @brief 每条轮廓一个输出。端点半值远离零取整，自动闭合，固定八连通 Bresenham 边界。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] Region 区域；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_region_contour_xld
 */
MVKIT_CPP_API void GenRegionContourXld(const MvObject& Contour, MvObject* Region, const MvTuple& Mode);

/**
 * @brief 每条轮廓一个长度；有效空数组返回空控制元组，结果由 C++ 对象自动管理。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] Length 几何长度或半长元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_length_xld
 */
MVKIT_CPP_API void LengthXld(const MvObject& XLD, MvTuple* Length);

/**
 * @brief 连续面积/质心，自动闭合，四个输出对象必须互异。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] Area 区域或轮廓面积元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_area_center_xld
 */
MVKIT_CPP_API void AreaCenterXld(const MvObject& XLD, MvTuple* Area, MvTuple* Row, MvTuple* Column, MvTuple* PointOrder);

/**
 * @brief HALCON 像素角点约定：HomMat2D 作用于 (row+.5,column+.5)，变换后再减 .5。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ContoursAffineTrans 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_contour_xld
 */
MVKIT_CPP_API void AffineTransContourXld(const MvObject& Contours, MvObject* ContoursAffineTrans, const MvTuple& HomMat2D);

/**
 * @brief 使用投影变换矩阵变换 XLD 轮廓。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ContoursProjTrans 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_projective_trans_contour_xld
 */
MVKIT_CPP_API void ProjectiveTransContourXld(const MvObject& Contours, MvObject* ContoursProjTrans, const MvTuple& HomMat2D);

/**
 * @brief 必要时追加首点实现显式闭合，同时复制首点属性。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ClosedContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_close_contours_xld
 */
MVKIT_CPP_API void CloseContoursXld(const MvObject& Contours, MvObject* ClosedContours);

/**
 * @brief 查询首个轮廓到点的线段最小距离和最大距离。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[out] DistanceMin 最小距离元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] DistanceMax 最大距离元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_pc
 */
MVKIT_CPP_API void DistancePc(const MvObject& Contour, const MvTuple& Row, const MvTuple& Column, MvTuple* DistanceMin, MvTuple* DistanceMax);

/**
 * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
 *
 * @param[in] Contour1 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Contour2 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 * @param[out] DistanceMin 最小距离元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] DistanceMax 最大距离元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_cc
 */
MVKIT_CPP_API void DistanceCc(const MvObject& Contour1, const MvObject& Contour2, const MvTuple& Mode, MvTuple* DistanceMin, MvTuple* DistanceMax);

/**
 * @brief 两个单轮廓的互交/自交查询；任一对象数组为空时三个结果均为空。
 *
 * @param[in] Contour1 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Contour2 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] IntersectionType 轮廓交点查询模式元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] IsOverlapping 几何关系判定标记元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_intersection_contours_xld
 */
MVKIT_CPP_API void IntersectionContoursXld(const MvObject& Contour1, const MvObject& Contour2, const MvTuple& IntersectionType, MvTuple* Row, MvTuple* Column, MvTuple* IsOverlapping);

/**
 * @brief 使用局部回归平滑；NumRegrPoints 为单个不小于 3 的奇数。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] SmoothedContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] NumRegrPoints 局部回归窗口的点数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_smooth_contours_xld
 */
MVKIT_CPP_API void SmoothContoursXld(const MvObject& Contours, MvObject* SmoothedContours, const MvTuple& NumRegrPoints);

/**
 * @brief 按轮廓特征筛选 XLD。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] SelectedContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Feature 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] Min1 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Max1 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Min2 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Max2 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_contours_xld
 */
MVKIT_CPP_API void SelectContoursXld(const MvObject& Contours, MvObject* SelectedContours, const MvTuple& Feature, const MvTuple& Min1, const MvTuple& Max1, const MvTuple& Min2, const MvTuple& Max2);

/**
 * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] SelectedXLD 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Features 属性或特征名称元组；调用期间借用，不转移所有权。
 * @param[in] Operation 多个特征条件的组合方式元组；调用期间借用，不转移所有权。
 * @param[in] Min 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 * @param[in] Max 筛选或测量范围界限元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_shape_xld
 */
MVKIT_CPP_API void SelectShapeXld(const MvObject& XLD, MvObject* SelectedXLD, const MvTuple& Features, const MvTuple& Operation, const MvTuple& Min, const MvTuple& Max);

/**
 * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] CroppedContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Col1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Col2 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] CloseContours 是否连接裁剪后轮廓的端点元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_crop_contours_xld
 */
MVKIT_CPP_API void CropContoursXld(const MvObject& Contours, MvObject* CroppedContours, const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Row2, const MvTuple& Col2, const MvTuple& CloseContours);

/**
 * @brief SortMode 为 upper_left/upper_right/lower_left/lower_right/character；
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] SortedContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] SortMode 排序依据元组；调用期间借用，不转移所有权。
 * @param[in] Order 排序、通道或点序控制元组；调用期间借用，不转移所有权。
 * @param[in] RowOrCol 分块或排序使用的行列方向元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_sort_contours_xld
 */
MVKIT_CPP_API void SortContoursXld(const MvObject& Contours, MvObject* SortedContours, const MvTuple& SortMode, const MvTuple& Order, const MvTuple& RowOrCol);

/**
 * @brief 合并开放非退化轮廓。距离使用回归线投影；至少一个参考同时满足 MaxDistAbs/MaxDistRel。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] UnionContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_union_adjacent_contours_xld
 */
MVKIT_CPP_API void UnionAdjacentContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxDistAbs, const MvTuple& MaxDistRel, const MvTuple& Mode);

/**
 * @brief 按距离、偏移和方向阈值合并共线轮廓。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] UnionContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxShift 共线轮廓的最大侧向偏移元组；调用期间借用，不转移所有权。
 * @param[in] MaxAngle 轮廓合并的角度阈值元组；调用期间借用，不转移所有权。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_union_collinear_contours_xld
 */
MVKIT_CPP_API void UnionCollinearContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxDistAbs, const MvTuple& MaxDistRel, const MvTuple& MaxShift, const MvTuple& MaxAngle, const MvTuple& Mode);

/**
 * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] UnionContours 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] MaxArcAngleDiff 轮廓合并的角度阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxArcOverlap 圆弧允许的最大重叠角度元组；调用期间借用，不转移所有权。
 * @param[in] MaxTangentAngle 轮廓合并的角度阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxDist 距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxRadiusDiff 圆弧中心或半径差的阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxCenterDist 圆弧中心或半径差的阈值元组；调用期间借用，不转移所有权。
 * @param[in] MergeSmallContours 是否合并短轮廓元组；调用期间借用，不转移所有权。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_union_cocircular_contours_xld
 */
MVKIT_CPP_API void UnionCocircularContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxArcAngleDiff, const MvTuple& MaxArcOverlap, const MvTuple& MaxTangentAngle, const MvTuple& MaxDist, const MvTuple& MaxRadiusDiff, const MvTuple& MaxCenterDist, const MvTuple& MergeSmallContours, const MvTuple& Iterations);

/**
 * @brief 将 XLD 轮廓分割为几何段。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] ContoursSplit 轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Mode 处理或查询模式元组；调用期间借用，不转移所有权。
 * @param[in] SmoothCont 分段前的轮廓平滑控制元组；调用期间借用，不转移所有权。
 * @param[in] MaxLineDist1 分段拟合的距离阈值元组；调用期间借用，不转移所有权。
 * @param[in] MaxLineDist2 分段拟合的距离阈值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_segment_contours_xld
 */
MVKIT_CPP_API void SegmentContoursXld(const MvObject& Contours, MvObject* ContoursSplit, const MvTuple& Mode, const MvTuple& SmoothCont, const MvTuple& MaxLineDist1, const MvTuple& MaxLineDist2);

/**
 * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Edges 边缘轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Filter 边缘滤波器类型元组；调用期间借用，不转移所有权。
 * @param[in] Alpha 边缘滤波参数元组；调用期间借用，不转移所有权。
 * @param[in] Low 滞后阈值的低阈值元组；调用期间借用，不转移所有权。
 * @param[in] High 滞后阈值的高阈值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_edges_sub_pix
 */
MVKIT_CPP_API void EdgesSubPix(const MvObject& Image, MvObject* Edges, const MvTuple& Filter, const MvTuple& Alpha, const MvTuple& Low, const MvTuple& High);

/**
 * @brief 单通道图像数组灰度等值线，Threshold 为单个有限数值。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Border 边界处理方式；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Threshold 边缘或灰度阈值元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_threshold_sub_pix
 */
MVKIT_CPP_API void ThresholdSubPix(const MvObject& Image, MvObject* Border, const MvTuple& Threshold);

/**
 * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移所有权。
 * @param[out] Lines 中心线轮廓；接收结果的 C++ 对象，自动管理资源。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间借用，不转移所有权。
 * @param[in] Low 滞后阈值的低阈值元组；调用期间借用，不转移所有权。
 * @param[in] High 滞后阈值的高阈值元组；调用期间借用，不转移所有权。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间借用，不转移所有权。
 * @param[in] ExtractWidth 是否计算线条宽度元组；调用期间借用，不转移所有权。
 * @param[in] LineModel 线条类型元组；调用期间借用，不转移所有权。
 * @param[in] CompleteJunctions 是否补全线条交汇点元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_lines_gauss
 */
MVKIT_CPP_API void LinesGauss(const MvObject& Image, MvObject* Lines, const MvTuple& Sigma, const MvTuple& Low, const MvTuple& High, const MvTuple& LightDark, const MvTuple& ExtractWidth, const MvTuple& LineModel, const MvTuple& CompleteJunctions);

/**
 * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间借用，不转移所有权。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间借用，不转移所有权。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间借用，不转移所有权。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间借用，不转移所有权。
 * @param[out] RowBegin 起点行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ColBegin 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] RowEnd 终点行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] ColEnd 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Nr 直线单位法向的行、列分量元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Nc 直线单位法向的行、列分量元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Dist 直线的法向距离元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fit_line_contour_xld
 */
MVKIT_CPP_API void FitLineContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& ClippingEndPoints, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* RowBegin, MvTuple* ColBegin, MvTuple* RowEnd, MvTuple* ColEnd, MvTuple* Nr, MvTuple* Nc, MvTuple* Dist);

/**
 * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间借用，不转移所有权。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间借用，不转移所有权。
 * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离元组；调用期间借用，不转移所有权。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间借用，不转移所有权。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Radius 半径元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fit_circle_contour_xld
 */
MVKIT_CPP_API void FitCircleContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& MaxClosureDist, const MvTuple& ClippingEndPoints, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Radius, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder);

/**
 * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间借用，不转移所有权。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间借用，不转移所有权。
 * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离元组；调用期间借用，不转移所有权。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间借用，不转移所有权。
 * @param[in] VossTabSize 椭圆拟合控制参数元组；调用期间借用，不转移所有权。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间借用，不转移所有权。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Phi 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Radius1 椭圆长半轴元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Radius2 椭圆短半轴元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] PointOrder 轮廓点的遍历方向元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_fit_ellipse_contour_xld
 */
MVKIT_CPP_API void FitEllipseContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& MaxClosureDist, const MvTuple& ClippingEndPoints, const MvTuple& VossTabSize, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Radius1, MvTuple* Radius2, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder);

}

namespace mv {
/**
 * @brief 将单轮廓坐标读入独立拥有型 Row/Col 元组。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Col 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_contour_xld
 */
MVKIT_CPP_API void GetContourXld(const MvObject& Contour, MvTuple* Row, MvTuple* Col);

/**
 * @brief 逐轮廓判断首尾坐标是否完全相等，不使用私有闭合元数据。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移所有权。
 * @param[out] IsClosed 轮廓闭合标记元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_test_closed_xld
 */
MVKIT_CPP_API void TestClosedXld(const MvObject& XLD, MvTuple* IsClosed);

/**
 * @brief 读取一个点属性：response、edge_direction 或 angle。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Name 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] Attrib 属性或特征名称元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_contour_attrib_xld
 */
MVKIT_CPP_API void GetContourAttribXld(const MvObject& Contour, const MvTuple& Name, MvTuple* Attrib);

/**
 * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移所有权。
 * @param[in] Name 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[out] Attrib 属性或特征名称元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_contour_global_attrib_xld
 */
MVKIT_CPP_API void GetContourGlobalAttribXld(const MvObject& Contour, const MvTuple& Name, MvTuple* Attrib);

}
