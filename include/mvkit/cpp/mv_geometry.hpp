#pragma once
#include <mvkit/cpp/mv_iconic.hpp>
namespace mv {
class MvCameraSetupModel;
class MvCamPar;
/**
 * @brief 生成二维仿射单位矩阵 `[1,0,0, 0,1,0]`；图像坐标按 HALCON 惯例将 row 传给 x、column 传给 y。
 *
 * @param[out] HomMat2DIdentity 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_identity
 */
MVKIT_CPP_API void HomMat2dIdentity(MvTuple* HomMat2DIdentity);

/**
 * @brief 在已有仿射矩阵左侧复合平移 (Tx,Ty)；Tx/Ty 为 x/y。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Tx 两个坐标轴的平移量元组；调用期间借用，不转移所有权。
 * @param[in] Ty 两个坐标轴的平移量元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2DTranslate 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_translate
 */
MVKIT_CPP_API void HomMat2dTranslate(const MvTuple& HomMat2D, const MvTuple& Tx, const MvTuple& Ty,
                                     MvTuple* HomMat2DTranslate);

/**
 * @brief 绕 (Px,Py) 旋转 Phi 弧度后左侧复合；Phi 为 x-y 平面数学正方向。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2DRotate 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_rotate
 */
MVKIT_CPP_API void HomMat2dRotate(const MvTuple& HomMat2D, const MvTuple& Phi, const MvTuple& Px,
                                  const MvTuple& Py, MvTuple* HomMat2DRotate);

/**
 * @brief 绕 (Px,Py) 以 (Sx,Sy) 缩放后左侧复合；零缩放拒绝。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Sx 两个坐标轴的缩放系数元组；调用期间借用，不转移所有权。
 * @param[in] Sy 两个坐标轴的缩放系数元组；调用期间借用，不转移所有权。
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2DScale 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_scale
 */
MVKIT_CPP_API void HomMat2dScale(const MvTuple& HomMat2D, const MvTuple& Sx, const MvTuple& Sy,
                                 const MvTuple& Px, const MvTuple& Py, MvTuple* HomMat2DScale);

/**
 * @brief 复合 Left*Right，先应用 Right 再应用 Left。奇异输入拒绝。
 *
 * @param[in] HomMat2DLeft 待复合的二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] HomMat2DRight 待复合的二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2DCompose 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_compose
 */
MVKIT_CPP_API void HomMat2dCompose(const MvTuple& HomMat2DLeft, const MvTuple& HomMat2DRight,
                                   MvTuple* HomMat2DCompose);

/**
 * @brief 求二维仿射逆；奇异或非有限矩阵拒绝。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2DInvert 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat2d_invert
 */
MVKIT_CPP_API void HomMat2dInvert(const MvTuple& HomMat2D, MvTuple* HomMat2DInvert);

/**
 * @brief 用仿射矩阵变换点 (Px,Py)=(x,y)；坐标数组必须等长，支持两个空数组。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[out] Qx X 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Qy Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_point_2d
 */
MVKIT_CPP_API void AffineTransPoint2d(const MvTuple& HomMat2D, const MvTuple& Px, const MvTuple& Py,
                                      MvTuple* Qx, MvTuple* Qy);

/**
 * @brief 由两点及方向角生成刚体变换，把 (Row1,Column1,Angle1) 映到第二组。
 *
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Angle1 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Angle2 方向或旋转角度元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2D 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_vector_angle_to_rigid
 */
MVKIT_CPP_API void VectorAngleToRigid(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Angle1,
                                      const MvTuple& Row2, const MvTuple& Column2, const MvTuple& Angle2,
                                      MvTuple* HomMat2D);

/**
 * @brief 点到点欧氏距离；Row/Column 支持 scalar-or-N。
 *
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
 * @param[out] Distance 距离元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_pp
 */
MVKIT_CPP_API void DistancePp(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2,
                              const MvTuple& Column2, MvTuple* Distance);

/**
 * @brief 点到无限直线距离；直线由两点定义，重合端点拒绝。
 *
 * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
 * @param[out] Distance 距离元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_distance_pl
 */
MVKIT_CPP_API void DistancePl(const MvTuple& Row, const MvTuple& Column, const MvTuple& Row1,
                              const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2,
                              MvTuple* Distance);

/**
 * @brief 两条无限直线交点。平行相离报错；重合时 IsOverlapping=1 并返回第一线首点。
 *
 * @param[in] Line1Row1 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line1Column1 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line1Row2 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line1Column2 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line2Row1 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line2Column1 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line2Row2 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] Line2Column2 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[out] Row 行坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Column 列坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] IsOverlapping 几何关系判定标记元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_intersection_lines
 */
MVKIT_CPP_API void IntersectionLines(const MvTuple& Line1Row1, const MvTuple& Line1Column1,
                                     const MvTuple& Line1Row2, const MvTuple& Line1Column2,
                                     const MvTuple& Line2Row1, const MvTuple& Line2Column1,
                                     const MvTuple& Line2Row2, const MvTuple& Line2Column2, MvTuple* Row,
                                     MvTuple* Column, MvTuple* IsOverlapping);

/**
 * @brief 两直线方向夹角，范围 (-pi,pi]，方向从点1指向点2。
 *
 * @param[in] RowA1 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] ColumnA1 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] RowA2 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] ColumnA2 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] RowB1 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] ColumnB1 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[in] RowB2 直线端点的行坐标元组；调用期间借用，不转移所有权。
 * @param[in] ColumnB2 直线端点的列坐标元组；调用期间借用，不转移所有权。
 * @param[out] Angle 方向或旋转角度元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_angle_ll
 */
MVKIT_CPP_API void AngleLl(const MvTuple& RowA1, const MvTuple& ColumnA1, const MvTuple& RowA2,
                           const MvTuple& ColumnA2, const MvTuple& RowB1, const MvTuple& ColumnB1,
                           const MvTuple& RowB2, const MvTuple& ColumnB2, MvTuple* Angle);

/**
 * @brief 将位姿元组 `[Tx,Ty,Tz,RxDeg,RyDeg,RzDeg,0]` 转为 12 元 3x4 齐次矩阵。
 *
 * @param[in] Pose 三维位姿元组；调用期间借用，不转移所有权。
 * @param[out] HomMat3D 三维齐次变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_pose_to_hom_mat3d
 */
MVKIT_CPP_API void PoseToHomMat3d(const MvTuple& Pose, MvTuple* HomMat3D);

/**
 * @brief 将刚体 3x4 矩阵转为位姿；一般仿射拒绝。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] Pose 三维位姿元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat3d_to_pose
 */
MVKIT_CPP_API void HomMat3dToPose(const MvTuple& HomMat3D, MvTuple* Pose);

/**
 * @brief 复合 Left*Right；12 元仿射，奇异输入拒绝。
 *
 * @param[in] HomMat3DLeft 待复合的三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] HomMat3DRight 待复合的三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] HomMat3DCompose 三维齐次变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat3d_compose
 */
MVKIT_CPP_API void HomMat3dCompose(const MvTuple& HomMat3DLeft, const MvTuple& HomMat3DRight,
                                   MvTuple* HomMat3DCompose);

/**
 * @brief 求 3x4 仿射逆；线性部分奇异则拒绝。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[out] HomMat3DInvert 三维齐次变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_hom_mat3d_invert
 */
MVKIT_CPP_API void HomMat3dInvert(const MvTuple& HomMat3D, MvTuple* HomMat3DInvert);

/**
 * @brief 用 3x4 仿射变换三维点；Px/Py/Pz 必须等长，支持全部为空。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间借用，不转移所有权。
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Pz Z 坐标元组；调用期间借用，不转移所有权。
 * @param[out] Qx X 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Qy Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
 * @param[out] Qz 变换后的 Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_affine_trans_point_3d
 */
MVKIT_CPP_API void AffineTransPoint3d(const MvTuple& HomMat3D, const MvTuple& Px, const MvTuple& Py,
                                      const MvTuple& Pz, MvTuple* Qx, MvTuple* Qy, MvTuple* Qz);

class MvHomMat2D;
class MvHomMat3D;
class MvPose;
/**
 * @brief 表示二维齐次变换的控制元组。
 */
class MVKIT_CPP_API MvHomMat2D : public MvTuple {
 public:
  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] Interpolation 插值方式；调用期间借用，不转移所有权。
   * @param[in] AdaptImageSize 是否调整输出图像尺寸；调用期间借用，不转移所有权。
   *
   * @return 图像，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_image
   */
  MvImage AffineTransImage(const MvImage& Image, const MvString& Interpolation,
                           const MvString& AdaptImageSize) const;

  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AffineTransImage(const MvImage& Image, const char* Interpolation, const char* AdaptImageSize) const;

  /**
   * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage AffineTransImage(const MvImage& Image, const wchar_t* Interpolation,
                           const wchar_t* AdaptImageSize) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
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
  MvImage ProjectiveTransImage(const MvImage& Image, const MvString& Interpolation,
                               const MvString& AdaptImageSize, const MvString& TransformDomain) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage ProjectiveTransImage(const MvImage& Image, const char* Interpolation, const char* AdaptImageSize,
                               const char* TransformDomain) const;

  /**
   * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
   * @overload 采用宽字符字符串参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvImage ProjectiveTransImage(const MvImage& Image, const wchar_t* Interpolation,
                               const wchar_t* AdaptImageSize, const wchar_t* TransformDomain) const;

  /**
   * @brief HALCON 像素角点约定：HomMat2D 作用于 (row+.5,column+.5)，变换后再减 .5。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_contour_xld
   */
  MvXLDCont AffineTransContourXld(const MvXLDCont& Contours) const;

  /**
   * @brief 使用投影变换矩阵变换 XLD 轮廓。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_projective_trans_contour_xld
   */
  MvXLDCont ProjectiveTransContourXld(const MvXLDCont& Contours) const;

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   *
   * @param[in] Region 输入区域；调用期间借用，不转移所有权。
   * @param[in] Interpolate 插值方式；调用期间借用，不转移所有权。
   *
   * @return 区域，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_region
   */
  MvRegion AffineTransRegion(const MvRegion& Region, const MvString& Interpolate) const;

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion AffineTransRegion(const MvRegion& Region, const char* Interpolate) const;

  /**
   * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
   * @overload 采用宽字符字符串参数。
   *
   * @return 区域，由返回对象自动管理资源。
   */
  MvRegion AffineTransRegion(const MvRegion& Region, const wchar_t* Interpolate) const;

  /**
   * @brief 构造空的控制值对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat2D();

  /**
   * @brief 复制控制元组内容，构造当前值类型。
   *
   * @param[in] values 用于构造的控制元组内容；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvHomMat2D(const MvTuple& values);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat2D(const MvHomMat2D&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvHomMat2D(MvHomMat2D&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat2D& operator=(const MvHomMat2D&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvHomMat2D& operator=(MvHomMat2D&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvHomMat2D() noexcept;

  /**
   * @brief 根据对应二维点估计仿射变换矩阵。
   *
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Qx X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Qy Y 坐标元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_vector_to_hom_mat2d
   */
  void VectorToHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx, const MvTuple& Qy);

  /**
   * @brief 根据对应二维点及测量协方差估计投影变换矩阵。
   *
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Qx X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Qy Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] CovXX1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   * @param[in] CovYY1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   * @param[in] CovXY1 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   * @param[in] CovXX2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   * @param[in] CovYY2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   * @param[in] CovXY2 对应点的测量协方差分量元组；调用期间借用，不转移所有权。
   *
   * @return 估计参数的协方差元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_vector_to_proj_hom_mat2d
   */
  MvTuple VectorToProjHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx, const MvTuple& Qy,
                               const MvString& Method, const MvTuple& CovXX1, const MvTuple& CovYY1,
                               const MvTuple& CovXY1, const MvTuple& CovXX2, const MvTuple& CovYY2,
                               const MvTuple& CovXY2);

  /**
   * @brief 根据对应二维点及测量协方差估计投影变换矩阵。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 估计参数的协方差元组，由返回对象自动管理资源。
   */
  MvTuple VectorToProjHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx, const MvTuple& Qy,
                               const char* Method, const MvTuple& CovXX1, const MvTuple& CovYY1,
                               const MvTuple& CovXY1, const MvTuple& CovXX2, const MvTuple& CovYY2,
                               const MvTuple& CovXY2);

  /**
   * @brief 根据对应二维点及测量协方差估计投影变换矩阵。
   * @overload 采用宽字符字符串参数。
   *
   * @return 估计参数的协方差元组，由返回对象自动管理资源。
   */
  MvTuple VectorToProjHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx, const MvTuple& Qy,
                               const wchar_t* Method, const MvTuple& CovXX1, const MvTuple& CovYY1,
                               const MvTuple& CovXY1, const MvTuple& CovXX2, const MvTuple& CovYY2,
                               const MvTuple& CovXY2);

  /**
   * @brief 生成二维仿射单位矩阵 `[1,0,0, 0,1,0]`；图像坐标按 HALCON 惯例将 row 传给 x、column 传给 y。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_identity
   */
  void HomMat2dIdentity();

  /**
   * @brief 在已有仿射矩阵左侧复合平移 (Tx,Ty)；Tx/Ty 为 x/y。
   *
   * @param[in] Tx 两个坐标轴的平移量元组；调用期间借用，不转移所有权。
   * @param[in] Ty 两个坐标轴的平移量元组；调用期间借用，不转移所有权。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_translate
   */
  MvHomMat2D HomMat2dTranslate(const MvTuple& Tx, const MvTuple& Ty) const;

  /**
   * @brief 在已有仿射矩阵左侧复合平移 (Tx,Ty)；Tx/Ty 为 x/y。
   * @overload 采用标量数值参数。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   */
  MvHomMat2D HomMat2dTranslate(double Tx, double Ty) const;

  /**
   * @brief 绕 (Px,Py) 旋转 Phi 弧度后左侧复合；Phi 为 x-y 平面数学正方向。
   *
   * @param[in] Phi 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_rotate
   */
  MvHomMat2D HomMat2dRotate(const MvTuple& Phi, const MvTuple& Px, const MvTuple& Py) const;

  /**
   * @brief 绕 (Px,Py) 旋转 Phi 弧度后左侧复合；Phi 为 x-y 平面数学正方向。
   * @overload 采用标量数值参数。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   */
  MvHomMat2D HomMat2dRotate(double Phi, double Px, double Py) const;

  /**
   * @brief 绕 (Px,Py) 以 (Sx,Sy) 缩放后左侧复合；零缩放拒绝。
   *
   * @param[in] Sx 两个坐标轴的缩放系数元组；调用期间借用，不转移所有权。
   * @param[in] Sy 两个坐标轴的缩放系数元组；调用期间借用，不转移所有权。
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_scale
   */
  MvHomMat2D HomMat2dScale(const MvTuple& Sx, const MvTuple& Sy, const MvTuple& Px, const MvTuple& Py) const;

  /**
   * @brief 绕 (Px,Py) 以 (Sx,Sy) 缩放后左侧复合；零缩放拒绝。
   * @overload 采用标量数值参数。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   */
  MvHomMat2D HomMat2dScale(double Sx, double Sy, double Px, double Py) const;

  /**
   * @brief 复合 Left*Right，先应用 Right 再应用 Left。奇异输入拒绝。
   *
   * @param[in] HomMat2DRight 待复合的二维变换矩阵；调用期间借用，不转移所有权。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_compose
   */
  MvHomMat2D HomMat2dCompose(const MvHomMat2D& HomMat2DRight) const;

  /**
   * @brief 求二维仿射逆；奇异或非有限矩阵拒绝。
   *
   * @return 二维变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat2d_invert
   */
  MvHomMat2D HomMat2dInvert() const;

  /**
   * @brief 用仿射矩阵变换点 (Px,Py)=(x,y)；坐标数组必须等长，支持两个空数组。
   *
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   * @param[out] Qy Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return X 坐标元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_point_2d
   */
  MvTuple AffineTransPoint2d(const MvTuple& Px, const MvTuple& Py, MvTuple* Qy) const;

  /**
   * @brief 用仿射矩阵变换点 (Px,Py)=(x,y)；坐标数组必须等长，支持两个空数组。
   * @overload 采用标量数值参数。
   *
   * @return X 坐标。
   */
  double AffineTransPoint2d(double Px, double Py, double* Qy) const;

  /**
   * @brief 由两点及方向角生成刚体变换，把 (Row1,Column1,Angle1) 映到第二组。
   *
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Angle1 方向或旋转角度元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Angle2 方向或旋转角度元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_vector_angle_to_rigid
   */
  void VectorAngleToRigid(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Angle1,
                          const MvTuple& Row2, const MvTuple& Column2, const MvTuple& Angle2);

  /**
   * @brief 由两点及方向角生成刚体变换，把 (Row1,Column1,Angle1) 映到第二组。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void VectorAngleToRigid(double Row1, double Column1, double Angle1, double Row2, double Column2,
                          double Angle2);

};
/**
 * @brief 表示三维齐次变换的控制元组。
 */
class MVKIT_CPP_API MvHomMat3D : public MvTuple {
 public:
  /**
   * @brief 构造空的控制值对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3D();

  /**
   * @brief 复制控制元组内容，构造当前值类型。
   *
   * @param[in] values 用于构造的控制元组内容；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvHomMat3D(const MvTuple& values);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3D(const MvHomMat3D&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvHomMat3D(MvHomMat3D&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3D& operator=(const MvHomMat3D&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvHomMat3D& operator=(MvHomMat3D&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvHomMat3D() noexcept;

  /**
   * @brief 将刚体 3x4 矩阵转为位姿；一般仿射拒绝。
   *
   * @return 三维位姿，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat3d_to_pose
   */
  MvPose HomMat3dToPose() const;

  /**
   * @brief 复合 Left*Right；12 元仿射，奇异输入拒绝。
   *
   * @param[in] HomMat3DRight 待复合的三维齐次变换矩阵；调用期间借用，不转移所有权。
   *
   * @return 三维齐次变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat3d_compose
   */
  MvHomMat3D HomMat3dCompose(const MvHomMat3D& HomMat3DRight) const;

  /**
   * @brief 求 3x4 仿射逆；线性部分奇异则拒绝。
   *
   * @return 三维齐次变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_hom_mat3d_invert
   */
  MvHomMat3D HomMat3dInvert() const;

  /**
   * @brief 用 3x4 仿射变换三维点；Px/Py/Pz 必须等长，支持全部为空。
   *
   * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
   * @param[in] Pz Z 坐标元组；调用期间借用，不转移所有权。
   * @param[out] Qy Y 坐标元组；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Qz 变换后的 Z 坐标元组；接收结果的 C++ 对象，自动管理资源。
   *
   * @return X 坐标元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_affine_trans_point_3d
   */
  MvTuple AffineTransPoint3d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Pz, MvTuple* Qy,
                             MvTuple* Qz) const;

  /**
   * @brief 用 3x4 仿射变换三维点；Px/Py/Pz 必须等长，支持全部为空。
   * @overload 采用标量数值参数。
   *
   * @return X 坐标。
   */
  double AffineTransPoint3d(double Px, double Py, double Pz, double* Qy, double* Qz) const;

};
/**
 * @brief 表示三维位置、旋转及位姿约定的控制元组。
 */
class MVKIT_CPP_API MvPose : public MvTuple {
 public:
  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   *
   * @param[out] Map2 映射图像；接收结果的 C++ 对象，自动管理资源。
   * @param[in] CamParam1 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] CamParam2 相机内参与畸变参数；调用期间借用，不转移所有权。
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
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam1, const MvCamPar& CamParam2, double SubSampling,
      const MvString& Method, const MvString& MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam1, const MvCamPar& CamParam2, double SubSampling,
      const char* Method, const char* MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief 两张编码 Map 图像及五个拥有型控制元组，全部输出同时提交。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& CamParam1, const MvCamPar& CamParam2, double SubSampling,
      const wchar_t* Method, const wchar_t* MapType, MvCamPar* CamParamRect1, MvCamPar* CamParamRect2,
      MvPose* CamPoseRect1, MvPose* CamPoseRect2, MvPose* RelPoseRect) const;

  /**
   * @brief real 单通道图像，保持对象数量及定义域，三个输出整体提交。
   *
   * @param[in] Disparity 视差结果；调用期间借用，不转移所有权。
   * @param[out] Y Y 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[out] Z Z 坐标；接收结果的 C++ 对象，自动管理资源。
   * @param[in] CamParamRect1 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] CamParamRect2 校正后的相机参数；调用期间借用，不转移所有权。
   *
   * @return X 坐标，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_disparity_image_to_xyz
   */
  MvImage DisparityImageToXyz(const MvImage& Disparity, MvImage* Y, MvImage* Z,
                             const MvCamPar& CamParamRect1, const MvCamPar& CamParamRect2) const;

  /**
   * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
   *
   * @param[in] CamParamRect1 校正后的相机参数；调用期间借用，不转移所有权。
   * @param[in] CamParamRect2 校正后的相机参数；调用期间借用，不转移所有权。
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
  void DisparityToPoint3d(const MvCamPar& CamParamRect1, const MvCamPar& CamParamRect2,
                          const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Disparity,
                          MvTuple* X, MvTuple* Y, MvTuple* Z) const;

  /**
   * @brief 校正相机控制及非空等长坐标元组；输出为拥有型 real 元组，失败保持三个原输出。
   * @overload 采用标量数值参数。
   *
   * @return 无返回值。
   */
  void DisparityToPoint3d(const MvCamPar& CamParamRect1, const MvCamPar& CamParamRect2,
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
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
   * @param[in] Method 算法方法；调用期间借用，不转移所有权。
   * @param[in] QualityType 质量评价方式元组；调用期间借用，不转移所有权。
   *
   * @return 质量评价结果元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_vector_to_pose
   */
  MvTuple VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                       const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                       const MvString& Method, const MvTuple& QualityType);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用字符串对象参数。
   *
   * @return 质量评价结果。
   */
  double VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                      const MvString& Method, const MvString& QualityType);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 质量评价结果。
   */
  double VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                      const char* Method, const char* QualityType);

  /**
   * @brief 根据世界坐标与图像坐标的对应关系估计相机位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 质量评价结果。
   */
  double VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                      const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                      const wchar_t* Method, const wchar_t* QualityType);

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   *
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
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
  MvImage GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const MvTuple& Scale,
                                  const MvString& MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const MvString& Scale,
                                  const MvString& MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const char* Scale,
                                  const char* MapType) const;

  /**
   * @brief 生成图像到世界平面的编码逆采样映射。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 映射图像，由返回对象自动管理资源。
   */
  MvImage GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn, std::int64_t HeightIn,
                                  std::int64_t WidthMapped, std::int64_t HeightMapped, const wchar_t* Scale,
                                  const wchar_t* MapType) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   *
   * @param[in] Image 输入图像；调用期间借用，不转移所有权。
   * @param[in] CameraParam 相机内参与畸变参数；调用期间借用，不转移所有权。
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
  MvImage ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                            std::int64_t Height, const MvTuple& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                            std::int64_t Height, const MvString& Scale, const MvString& Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用 UTF-8 字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                            std::int64_t Height, const char* Scale, const char* Interpolation) const;

  /**
   * @brief 将图像重采样到指定世界平面。
   * @overload 采用宽字符字符串及标量数值参数。
   *
   * @return 图像，由返回对象自动管理资源。
   */
  MvImage ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                            std::int64_t Height, const wchar_t* Scale, const wchar_t* Interpolation) const;

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   *
   * @param[in] CameraSetupModelID 包含相机配置模型句柄的值；调用期间借用，不转移所有权。
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
  static void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                                     const MvTuple& CameraType, const MvCamPar& CameraParam,
                                     const MvTuple& CameraPose);

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  static void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                                     const MvString& CameraType, const MvCamPar& CameraParam,
                                     const MvTuple& CameraPose);

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  static void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                                     const char* CameraType, const MvCamPar& CameraParam,
                                     const MvTuple& CameraPose);

  /**
   * @brief 设置相机配置模型中指定相机的参数及位姿。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  static void SetCameraSetupCamParam(const MvCameraSetupModel& CameraSetupModelID, const MvTuple& CameraIdx,
                                     const wchar_t* CameraType, const MvCamPar& CameraParam,
                                     const MvTuple& CameraPose);

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   *
   * @param[in] Contours 输入轮廓；调用期间借用，不转移所有权。
   * @param[in] CameraParam 相机内参与畸变参数元组；调用期间借用，不转移所有权。
   * @param[in] Scale 坐标、灰度或模型的缩放控制元组；调用期间借用，不转移所有权。
   *
   * @return 轮廓，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_contour_to_world_plane_xld
   */
  MvXLDCont ContourToWorldPlaneXld(const MvXLDCont& Contours, const MvTuple& CameraParam,
                                   const MvTuple& Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用字符串对象参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvXLDCont& Contours, const MvTuple& CameraParam,
                                   const MvString& Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvXLDCont& Contours, const MvTuple& CameraParam,
                                   const char* Scale) const;

  /**
   * @brief 轮廓点变换到世界平面；输出 row=Y、column=X，单位与 image_points_to_world_plane 相同。
   * @overload 采用宽字符字符串参数。
   *
   * @return 轮廓，由返回对象自动管理资源。
   */
  MvXLDCont ContourToWorldPlaneXld(const MvXLDCont& Contours, const MvTuple& CameraParam,
                                   const wchar_t* Scale) const;

  /**
   * @brief 构造空的控制值对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPose();

  /**
   * @brief 复制控制元组内容，构造当前值类型。
   *
   * @param[in] values 用于构造的控制元组内容；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvPose(const MvTuple& values);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPose(const MvPose&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvPose(MvPose&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPose& operator=(const MvPose&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvPose& operator=(MvPose&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvPose() noexcept;

  /**
   * @brief 读取 HALCON f/r/t 位姿文本及历史 MVKIT_POSE 文本。
   *
   * @param[in] PoseFile 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_read_pose
   */
  void ReadPose(const MvString& PoseFile);

  /**
   * @brief 读取 HALCON f/r/t 位姿文本及历史 MVKIT_POSE 文本。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void ReadPose(const char* PoseFile);

  /**
   * @brief 读取 HALCON f/r/t 位姿文本及历史 MVKIT_POSE 文本。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void ReadPose(const wchar_t* PoseFile);

  /**
   * @brief 写入 HALCON f/r/t 位姿文本；保留输入表示类型。
   *
   * @param[in] PoseFile 文件路径；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_write_pose
   */
  void WritePose(const MvString& PoseFile) const;

  /**
   * @brief 写入 HALCON f/r/t 位姿文本；保留输入表示类型。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void WritePose(const char* PoseFile) const;

  /**
   * @brief 写入 HALCON f/r/t 位姿文本；保留输入表示类型。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  void WritePose(const wchar_t* PoseFile) const;

  /**
   * @brief 将位姿元组 `[Tx,Ty,Tz,RxDeg,RyDeg,RzDeg,0]` 转为 12 元 3x4 齐次矩阵。
   *
   * @return 三维齐次变换矩阵，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_pose_to_hom_mat3d
   */
  MvHomMat3D PoseToHomMat3d() const;

};
struct MvPoseArrayData;
/**
 * @brief 拥有型位姿数组。
 */
class MVKIT_CPP_API MvPoseArray {
 public:
  /**
   * @brief 构造空的拥有型数组。
   */
  MvPoseArray() noexcept;

  /**
   * @brief 复制数组元素，构造拥有型数组。
   *
   * @param[in] values 保存、追加或读取的数据。
   * @param[in] length 几何长度或半长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPoseArray(const MvPose* values, std::int64_t length);

  /**
   * @brief 复制构造，复制数组元素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPoseArray(const MvPoseArray&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvPoseArray(MvPoseArray&&) noexcept;

  /**
   * @brief 复制赋值，复制数组元素。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvPoseArray& operator=(const MvPoseArray&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvPoseArray& operator=(MvPoseArray&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvPoseArray() noexcept;

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
  const MvPose* Tools() const noexcept;

  /**
   * @brief 查询当前数组的元素数量。
   *
   * @return 当前容器的元素数量。
   */
  virtual std::int64_t Length() const noexcept;

  /**
   * @brief 从控制元组更新当前对象内容。
   *
   * @details 每个数组元素对应连续的 7 个数值；输入长度必须为 7 的整数倍，空输入生成空数组。
   *
   * @param[in] values 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual void SetFromTuple(const MvTuple& values);

  /**
   * @brief 将当前对象内容转换为拥有型控制元组。
   *
   * @details 按元素顺序展平，每个元素贡献 7 个数值。
   *
   * @return 当前内容的拥有型控制元组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual MvTuple ConvertToTuple() const;

 private:
  MvPoseArrayData* data_{nullptr};
};
struct MvHomMat3DArrayData;
/**
 * @brief 拥有型三维齐次矩阵数组。
 */
class MVKIT_CPP_API MvHomMat3DArray {
 public:
  /**
   * @brief 构造空的拥有型数组。
   */
  MvHomMat3DArray() noexcept;

  /**
   * @brief 复制数组元素，构造拥有型数组。
   *
   * @param[in] values 保存、追加或读取的数据。
   * @param[in] length 几何长度或半长。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3DArray(const MvHomMat3D* values, std::int64_t length);

  /**
   * @brief 复制构造，复制数组元素。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3DArray(const MvHomMat3DArray&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvHomMat3DArray(MvHomMat3DArray&&) noexcept;

  /**
   * @brief 复制赋值，复制数组元素。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHomMat3DArray& operator=(const MvHomMat3DArray&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvHomMat3DArray& operator=(MvHomMat3DArray&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvHomMat3DArray() noexcept;

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
  const MvHomMat3D* Tools() const noexcept;

  /**
   * @brief 查询当前数组的元素数量。
   *
   * @return 当前容器的元素数量。
   */
  virtual std::int64_t Length() const noexcept;

  /**
   * @brief 从控制元组更新当前对象内容。
   *
   * @details 每个数组元素对应连续的 12 个数值；输入长度必须为 12 的整数倍，空输入生成空数组。
   *
   * @param[in] values 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual void SetFromTuple(const MvTuple& values);

  /**
   * @brief 将当前对象内容转换为拥有型控制元组。
   *
   * @details 按元素顺序展平，每个元素贡献 12 个数值。
   *
   * @return 当前内容的拥有型控制元组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  virtual MvTuple ConvertToTuple() const;

 private:
  MvHomMat3DArrayData* data_{nullptr};
};
/**
 * @brief 提供几何计算的静态便捷接口。
 */
class MVKIT_CPP_API MvMisc {
 public:
  /**
   * @brief 点到点欧氏距离；Row/Column 支持 scalar-or-N。
   *
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 距离元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_pp
   */
  static MvTuple DistancePp(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2,
                            const MvTuple& Column2);

  /**
   * @brief 点到点欧氏距离；Row/Column 支持 scalar-or-N。
   * @overload 采用标量数值参数。
   *
   * @return 距离。
   */
  static double DistancePp(double Row1, double Column1, double Row2, double Column2);

  /**
   * @brief 点到无限直线距离；直线由两点定义，重合端点拒绝。
   *
   * @param[in] Row 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row1 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column1 列坐标元组；调用期间借用，不转移所有权。
   * @param[in] Row2 行坐标元组；调用期间借用，不转移所有权。
   * @param[in] Column2 列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 距离元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_distance_pl
   */
  static MvTuple DistancePl(const MvTuple& Row, const MvTuple& Column, const MvTuple& Row1,
                            const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2);

  /**
   * @brief 点到无限直线距离；直线由两点定义，重合端点拒绝。
   * @overload 采用标量数值参数。
   *
   * @return 距离。
   */
  static double DistancePl(double Row, double Column, double Row1, double Column1, double Row2,
                           double Column2);

  /**
   * @brief 两直线方向夹角，范围 (-pi,pi]，方向从点1指向点2。
   *
   * @param[in] RowA1 直线端点的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ColumnA1 直线端点的列坐标元组；调用期间借用，不转移所有权。
   * @param[in] RowA2 直线端点的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ColumnA2 直线端点的列坐标元组；调用期间借用，不转移所有权。
   * @param[in] RowB1 直线端点的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ColumnB1 直线端点的列坐标元组；调用期间借用，不转移所有权。
   * @param[in] RowB2 直线端点的行坐标元组；调用期间借用，不转移所有权。
   * @param[in] ColumnB2 直线端点的列坐标元组；调用期间借用，不转移所有权。
   *
   * @return 方向或旋转角度元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_angle_ll
   */
  static MvTuple AngleLl(const MvTuple& RowA1, const MvTuple& ColumnA1, const MvTuple& RowA2,
                         const MvTuple& ColumnA2, const MvTuple& RowB1, const MvTuple& ColumnB1,
                         const MvTuple& RowB2, const MvTuple& ColumnB2);

  /**
   * @brief 两直线方向夹角，范围 (-pi,pi]，方向从点1指向点2。
   * @overload 采用标量数值参数。
   *
   * @return 方向或旋转角度。
   */
  static double AngleLl(double RowA1, double ColumnA1, double RowA2, double ColumnA2, double RowB1,
                        double ColumnB1, double RowB2, double ColumnB2);

};
}  // namespace mv

namespace mv {
/**
 * @brief 根据对应二维点估计仿射变换矩阵。
 *
 * @param[in] Px X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Py Y 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Qx X 坐标元组；调用期间借用，不转移所有权。
 * @param[in] Qy Y 坐标元组；调用期间借用，不转移所有权。
 * @param[out] HomMat2D 二维变换矩阵元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_vector_to_hom_mat2d
 */
MVKIT_CPP_API void VectorToHomMat2d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Qx,
                                    const MvTuple& Qy, MvTuple* HomMat2D);

}
