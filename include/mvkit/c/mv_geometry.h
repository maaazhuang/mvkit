#ifndef MVKIT_C_MV_GEOMETRY_H
#define MVKIT_C_MV_GEOMETRY_H

/** @file
 * @brief 公共二维/三维几何值类型与仿射、解析几何和位姿算子。
 */

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MvPoint2d {
  double row;
  double col;
} MvPoint2d;

/**
 * @brief 生成二维仿射单位矩阵 `[1,0,0, 0,1,0]`；图像坐标按 HALCON 惯例将 row 传给 x、column 传给 y。
 *
 * @param[out] HomMat2DIdentity 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_identity(MvTuple* HomMat2DIdentity);

/**
 * @brief 在已有仿射矩阵左侧复合平移 (Tx,Ty)；Tx/Ty 为 x/y。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Tx 两个坐标轴的平移量元组；调用期间只读借用。
 * @param[in] Ty 两个坐标轴的平移量元组；调用期间只读借用。
 * @param[out] HomMat2DTranslate 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_translate(MvTuple HomMat2D,MvTuple Tx,MvTuple Ty,MvTuple* HomMat2DTranslate);

/**
 * @brief 绕 (Px,Py) 旋转 Phi 弧度后左侧复合；Phi 为 x-y 平面数学正方向。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Phi 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[out] HomMat2DRotate 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_rotate(MvTuple HomMat2D,MvTuple Phi,MvTuple Px,MvTuple Py,MvTuple* HomMat2DRotate);

/**
 * @brief 绕 (Px,Py) 以 (Sx,Sy) 缩放后左侧复合；零缩放拒绝。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Sx 两个坐标轴的缩放系数元组；调用期间只读借用。
 * @param[in] Sy 两个坐标轴的缩放系数元组；调用期间只读借用。
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[out] HomMat2DScale 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_scale(MvTuple HomMat2D,MvTuple Sx,MvTuple Sy,MvTuple Px,MvTuple Py,MvTuple* HomMat2DScale);

/**
 * @brief 复合 Left*Right，先应用 Right 再应用 Left。奇异输入拒绝。
 *
 * @param[in] HomMat2DLeft 待复合的二维变换矩阵元组；调用期间只读借用。
 * @param[in] HomMat2DRight 待复合的二维变换矩阵元组；调用期间只读借用。
 * @param[out] HomMat2DCompose 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_compose(MvTuple HomMat2DLeft,MvTuple HomMat2DRight,MvTuple* HomMat2DCompose);

/**
 * @brief 求二维仿射逆；奇异或非有限矩阵拒绝。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[out] HomMat2DInvert 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat2d_invert(MvTuple HomMat2D,MvTuple* HomMat2DInvert);

/**
 * @brief 用仿射矩阵变换点 (Px,Py)=(x,y)；坐标数组必须等长，支持两个空数组。
 *
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[out] Qx X 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Qy Y 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_point_2d(MvTuple HomMat2D,MvTuple Px,MvTuple Py,MvTuple* Qx,MvTuple* Qy);

/**
 * @brief 由两点及方向角生成刚体变换，把 (Row1,Column1,Angle1) 映到第二组。
 *
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Column1 列坐标元组；调用期间只读借用。
 * @param[in] Angle1 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Column2 列坐标元组；调用期间只读借用。
 * @param[in] Angle2 方向或旋转角度元组；调用期间只读借用。
 * @param[out] HomMat2D 二维变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_vector_angle_to_rigid(MvTuple Row1,MvTuple Column1,MvTuple Angle1,MvTuple Row2,MvTuple Column2,MvTuple Angle2,MvTuple* HomMat2D);

/**
 * @brief 点到点欧氏距离；Row/Column 支持 scalar-or-N。
 *
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Column1 列坐标元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Column2 列坐标元组；调用期间只读借用。
 * @param[out] Distance 距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_pp(MvTuple Row1,MvTuple Column1,MvTuple Row2,MvTuple Column2,MvTuple* Distance);

/**
 * @brief 点到无限直线距离；直线由两点定义，重合端点拒绝。
 *
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Column1 列坐标元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Column2 列坐标元组；调用期间只读借用。
 * @param[out] Distance 距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_pl(MvTuple Row,MvTuple Column,MvTuple Row1,MvTuple Column1,MvTuple Row2,MvTuple Column2,MvTuple* Distance);

/**
 * @brief 两条无限直线交点。平行相离报错；重合时 IsOverlapping=1 并返回第一线首点。
 *
 * @param[in] Line1Row1 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] Line1Column1 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] Line1Row2 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] Line1Column2 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] Line2Row1 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] Line2Column1 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] Line2Row2 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] Line2Column2 直线端点的列坐标元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] IsOverlapping 几何关系判定标记元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_intersection_lines(MvTuple Line1Row1,MvTuple Line1Column1,MvTuple Line1Row2,MvTuple Line1Column2,MvTuple Line2Row1,MvTuple Line2Column1,MvTuple Line2Row2,MvTuple Line2Column2,MvTuple* Row,MvTuple* Column,MvTuple* IsOverlapping);

/**
 * @brief 两直线方向夹角，范围 (-pi,pi]，方向从点1指向点2。
 *
 * @param[in] RowA1 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] ColumnA1 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] RowA2 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] ColumnA2 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] RowB1 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] ColumnB1 直线端点的列坐标元组；调用期间只读借用。
 * @param[in] RowB2 直线端点的行坐标元组；调用期间只读借用。
 * @param[in] ColumnB2 直线端点的列坐标元组；调用期间只读借用。
 * @param[out] Angle 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_angle_ll(MvTuple RowA1,MvTuple ColumnA1,MvTuple RowA2,MvTuple ColumnA2,MvTuple RowB1,MvTuple ColumnB1,MvTuple RowB2,MvTuple ColumnB2,MvTuple* Angle);

// 三维正式几何算子。

/**
 * @brief 将位姿元组 `[Tx,Ty,Tz,RxDeg,RyDeg,RzDeg,0]` 转为 12 元 3x4 齐次矩阵。
 *
 * @param[in] Pose 三维位姿元组；调用期间只读借用。
 * @param[out] HomMat3D 三维齐次变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_pose_to_hom_mat3d(MvTuple Pose,MvTuple* HomMat3D);

/**
 * @brief 将刚体 3x4 矩阵转为位姿；一般仿射拒绝。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间只读借用。
 * @param[out] Pose 三维位姿元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat3d_to_pose(MvTuple HomMat3D,MvTuple* Pose);

/**
 * @brief 复合 Left*Right；12 元仿射，奇异输入拒绝。
 *
 * @param[in] HomMat3DLeft 待复合的三维齐次变换矩阵元组；调用期间只读借用。
 * @param[in] HomMat3DRight 待复合的三维齐次变换矩阵元组；调用期间只读借用。
 * @param[out] HomMat3DCompose 三维齐次变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat3d_compose(MvTuple HomMat3DLeft,MvTuple HomMat3DRight,MvTuple* HomMat3DCompose);

/**
 * @brief 求 3x4 仿射逆；线性部分奇异则拒绝。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间只读借用。
 * @param[out] HomMat3DInvert 三维齐次变换矩阵元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_hom_mat3d_invert(MvTuple HomMat3D,MvTuple* HomMat3DInvert);

/**
 * @brief 用 3x4 仿射变换三维点；Px/Py/Pz 必须等长，支持全部为空。
 *
 * @param[in] HomMat3D 三维齐次变换矩阵元组；调用期间只读借用。
 * @param[in] Px X 坐标元组；调用期间只读借用。
 * @param[in] Py Y 坐标元组；调用期间只读借用。
 * @param[in] Pz Z 坐标元组；调用期间只读借用。
 * @param[out] Qx X 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Qy Y 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Qz 变换后的 Z 坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_point_3d(MvTuple HomMat3D,MvTuple Px,MvTuple Py,MvTuple Pz,MvTuple* Qx,MvTuple* Qy,MvTuple* Qz);

#ifdef __cplusplus
}
#endif
#endif
