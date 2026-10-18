#ifndef MVKIT_C_MV_XLD_H
#define MVKIT_C_MV_XLD_H

/** @file
 * @brief MVKit 正式 XLD 算子，使用 HALCON 风格 控制元组参数。 */

#include <stddef.h>
#include <stdint.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_region.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 逐轮廓判断首尾坐标是否完全相等，不使用私有闭合元数据。
 *
 * @details 单点闭合，零点不闭合；零对象数组返回拥有型空元组，支持 contour 类型。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] IsClosed 轮廓闭合标记元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_test_closed_xld(MvObject XLD, MvTuple* IsClosed);

/**
 * @brief 将单轮廓坐标读入独立拥有型 Row/Col 元组。
 *
 * @details 空对象数组返回空元组，多轮廓拒绝；输出必须互异，失败保持原值。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Col 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_contour_xld(MvObject Contour, MvTuple* Row, MvTuple* Col);

/**
 * @brief 读取一个点属性：response、edge_direction 或 angle。
 *
 * @details 要求一个轮廓及单字符串 Name；属性缺失返回 NOT_FOUND。空对象数组返回拥有型空元组，不解释 Name。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Name 参数或数据项名称元组；调用期间只读借用。
 * @param[out] Attrib 属性或特征名称元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_contour_attrib_xld(MvObject Contour, MvTuple Name, MvTuple* Attrib);

/**
 * @brief 读取轮廓全局属性，当前仅支持 cont_approx。
 *
 * @details 单轮廓的 Name 为非空字符串元组，直线=-1、椭圆=0、圆=1，未知近似视为缺失。
 * 不支持/缺失属性返回 NOT_FOUND；空对象数组返回拥有型空元组，不解释 Name。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Name 参数或数据项名称元组；调用期间只读借用。
 * @param[out] Attrib 属性或特征名称元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_contour_global_attrib_xld(MvObject Contour, MvTuple Name, MvTuple* Attrib);

/**
 * @brief 由等长度、非空的 Row/Col 数值元组 生成一个 XLD 轮廓。
 *
 * @param[out] Contour 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Col 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_contour_polygon_xld(MvObject* Contour, MvTuple Row, MvTuple Col);

/**
 * @brief 生成圆弧或圆形；控制元组 支持 1/N 广播，Resolution 为相邻点弦长。
 *
 * @details 完整圆含显式闭合点；当前单轮廓最多生成一千万条线段。
 *
 * @param[out] ContCircle 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Radius 半径元组；调用期间只读借用。
 * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间只读借用。
 * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间只读借用。
 * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间只读借用。
 * @param[in] Resolution 轮廓采样分辨率元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_circle_contour_xld(MvObject* ContCircle, MvTuple Row, MvTuple Column, MvTuple Radius, MvTuple StartPhi, MvTuple EndPhi, MvTuple PointOrder, MvTuple Resolution);

/**
 * @brief 从区域对象数组生成轮廓；Mode 为 center、border 或 border_holes。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] Contours 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_contour_region_xld(MvObject Regions, MvObject* Contours, MvTuple Mode);

/**
 * @brief 每条轮廓一个长度；有效空数组返回空控制元组，调用方负责释放结果句柄。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] Length 几何长度或半长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_length_xld(MvObject XLD, MvTuple* Length);

/**
 * @brief 按轮廓特征筛选 XLD。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] SelectedContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Feature 属性或特征名称元组；调用期间只读借用。
 * @param[in] Min1 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Max1 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Min2 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Max2 筛选或测量范围界限元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_contours_xld(MvObject Contours, MvObject* SelectedContours, MvTuple Feature, MvTuple Min1, MvTuple Max1, MvTuple Min2, MvTuple Max2);

/**
 * @brief 使用局部回归平滑；NumRegrPoints 为单个不小于 3 的奇数。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] SmoothedContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] NumRegrPoints 局部回归窗口的点数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_smooth_contours_xld(MvObject Contours, MvObject* SmoothedContours, MvTuple NumRegrPoints);

/**
 * @brief 将 XLD 轮廓分割为几何段。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ContoursSplit 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 * @param[in] SmoothCont 分段前的轮廓平滑控制元组；调用期间只读借用。
 * @param[in] MaxLineDist1 分段拟合的距离阈值元组；调用期间只读借用。
 * @param[in] MaxLineDist2 分段拟合的距离阈值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_segment_contours_xld(MvObject Contours, MvObject* ContoursSplit, MvTuple Mode, MvTuple SmoothCont, MvTuple MaxLineDist1, MvTuple MaxLineDist2);

/**
 * @brief 逐轮廓拟合直线；ClippingEndPoints 只裁剪拟合点，首末原始点用于端点投影。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间只读借用。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间只读借用。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间只读借用。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间只读借用。
 * @param[out] RowBegin 起点行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ColBegin 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] RowEnd 终点行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ColEnd 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Nr 直线单位法向的行、列分量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Nc 直线单位法向的行、列分量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Dist 直线的法向距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fit_line_contour_xld(MvObject Contours, MvTuple Algorithm, MvTuple MaxNumPoints, MvTuple ClippingEndPoints, MvTuple Iterations, MvTuple ClippingFactor, MvTuple* RowBegin, MvTuple* ColBegin, MvTuple* RowEnd, MvTuple* ColEnd, MvTuple* Nr, MvTuple* Nc, MvTuple* Dist);

/**
 * @brief 逐轮廓拟合圆或圆弧；MaxClosureDist 控制是否返回完整圆。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间只读借用。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间只读借用。
 * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离元组；调用期间只读借用。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间只读借用。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Radius 半径元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] PointOrder 轮廓点的遍历方向元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fit_circle_contour_xld(MvObject Contours, MvTuple Algorithm, MvTuple MaxNumPoints, MvTuple MaxClosureDist, MvTuple ClippingEndPoints, MvTuple Iterations, MvTuple ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Radius, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder);

/**
 * @brief 对 0..N 个 Image 对象逐一提取亚像素边缘，并按输入顺序拼接 XLD 输出。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Edges 边缘轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Filter 边缘滤波器类型元组；调用期间只读借用。
 * @param[in] Alpha 边缘滤波参数元组；调用期间只读借用。
 * @param[in] Low 滞后阈值的低阈值元组；调用期间只读借用。
 * @param[in] High 滞后阈值的高阈值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_edges_sub_pix(MvObject Image, MvObject* Edges, MvTuple Filter, MvTuple Alpha, MvTuple Low, MvTuple High);

/**
 * @brief 每条轮廓一个输出。端点半值远离零取整，自动闭合，固定八连通 Bresenham 边界。
 *
 * @details Filled 添加 even-odd 多边形内部；坐标必须有限且在 [-2^30,2^30] 内，空轮廓产生空区域。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] Region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_region_contour_xld(MvObject Contour, MvObject* Region, MvTuple Mode);

/**
 * @brief 连续面积/质心，自动闭合，四个 Tuple 输出必须互异。
 *
 * @details 代数面积为零时取顶点均值（排除末端重复点）并返回正方向；空轮廓返回零/正方向。自交返回代数测量值。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] Area 区域或轮廓面积元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] PointOrder 轮廓点的遍历方向元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_area_center_xld(MvObject XLD, MvTuple* Area, MvTuple* Row, MvTuple* Column, MvTuple* PointOrder);

/**
 * @brief 合并开放非退化轮廓。距离使用回归线投影；至少一个参考同时满足 MaxDistAbs/MaxDistRel。
 *
 * @details MaxShift 为对称端点到线限制，MaxAngle 在 [0,pi/2] 弧度内；阈值有限非负，坐标有限且在 +/-2^30 内。
 * Keep 仅保留全部贡献轮廓共有属性，详见 X03 模块契约。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] UnionContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值元组；调用期间只读借用。
 * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值元组；调用期间只读借用。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_union_adjacent_contours_xld(MvObject Contours, MvObject* UnionContours, MvTuple MaxDistAbs, MvTuple MaxDistRel, MvTuple Mode);

/**
 * @brief 按距离、偏移和方向阈值合并共线轮廓。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] UnionContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaxDistAbs 轮廓合并的绝对、相对距离阈值元组；调用期间只读借用。
 * @param[in] MaxDistRel 轮廓合并的绝对、相对距离阈值元组；调用期间只读借用。
 * @param[in] MaxShift 共线轮廓的最大侧向偏移元组；调用期间只读借用。
 * @param[in] MaxAngle 轮廓合并的角度阈值元组；调用期间只读借用。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_union_collinear_contours_xld(MvObject Contours, MvObject* UnionContours, MvTuple MaxDistAbs, MvTuple MaxDistRel, MvTuple MaxShift, MvTuple MaxAngle, MvTuple Mode);

/**
 * @brief 按包含端点的亚像素矩形精确裁剪线段。CloseContours 为字符串 true/false。
 *
 * @details 闭合重建要求简单多边形，合成框边界闭合丢弃点属性；开放片段插值 response 和圆周角。
 * 边界/坐标须有限且在 [-2^30,2^30] 内，省略空片段。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] CroppedContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Col1 列坐标元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Col2 列坐标元组；调用期间只读借用。
 * @param[in] CloseContours 是否连接裁剪后轮廓的端点元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_crop_contours_xld(MvObject Contours, MvObject* CroppedContours, MvTuple Row1, MvTuple Col1, MvTuple Row2, MvTuple Col2, MvTuple CloseContours);

/**
 * @brief SortMode 为 upper_left/upper_right/lower_left/lower_right/character；
 *
 * @details Order 为 true/false，RowOrCol 为 row/column，均为标量字符串。稳定包围盒排序；
 * Character 按主区间传递重叠分组。空轮廓在两个方向均排最后，属性不变。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] SortedContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] SortMode 排序依据元组；调用期间只读借用。
 * @param[in] Order 排序、通道或点序控制元组；调用期间只读借用。
 * @param[in] RowOrCol 分块或排序使用的行列方向元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_sort_contours_xld(MvObject Contours, MvObject* SortedContours, MvTuple SortMode, MvTuple Order, MvTuple RowOrCol);

/**
 * @brief 生成椭圆或椭圆弧；Radius1 >= Radius2 > 0，Resolution > 0，角度单位为弧度。
 *
 * @details 椭圆拟合使用归一化代数初始化及可选几何 IRLS。
 *
 * @param[out] ContEllipse 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Phi 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Radius1 椭圆长半轴元组；调用期间只读借用。
 * @param[in] Radius2 椭圆短半轴元组；调用期间只读借用。
 * @param[in] StartPhi 圆弧或椭圆弧的起止角度元组；调用期间只读借用。
 * @param[in] EndPhi 圆弧或椭圆弧的起止角度元组；调用期间只读借用。
 * @param[in] PointOrder 轮廓点的遍历方向元组；调用期间只读借用。
 * @param[in] Resolution 轮廓采样分辨率元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_ellipse_contour_xld(MvObject* ContEllipse, MvTuple Row, MvTuple Column, MvTuple Phi, MvTuple Radius1, MvTuple Radius2, MvTuple StartPhi, MvTuple EndPhi, MvTuple PointOrder, MvTuple Resolution);

/**
 * @brief 逐轮廓拟合椭圆，至少 5 个未裁剪点；支持六种代数/几何算法，八个输出必须互异。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Algorithm 拟合算法元组；调用期间只读借用。
 * @param[in] MaxNumPoints 拟合使用的最大点数量元组；调用期间只读借用。
 * @param[in] MaxClosureDist 判定轮廓闭合的最大端点距离元组；调用期间只读借用。
 * @param[in] ClippingEndPoints 拟合时忽略的轮廓端点数量元组；调用期间只读借用。
 * @param[in] VossTabSize 椭圆拟合控制参数元组；调用期间只读借用。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 * @param[in] ClippingFactor 鲁棒拟合的裁剪因子元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Phi 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Radius1 椭圆长半轴元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Radius2 椭圆短半轴元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] StartPhi 圆弧或椭圆弧的起止角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] EndPhi 圆弧或椭圆弧的起止角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] PointOrder 轮廓点的遍历方向元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fit_ellipse_contour_xld(MvObject Contours, MvTuple Algorithm, MvTuple MaxNumPoints, MvTuple MaxClosureDist, MvTuple ClippingEndPoints, MvTuple VossTabSize, MvTuple Iterations, MvTuple ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Radius1, MvTuple* Radius2, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder);

/**
 * @brief 查询首个轮廓到点的线段最小距离和最大距离。
 *
 * @details Row/Column 等长，不做标量广播；Row 为空或对象数组为空时返回空控制元组。
 * 包含一个零点轮廓的对象仍不支持距离查询。
 *
 * @param[in] Contour 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[out] DistanceMin 最小距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] DistanceMax 最大距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_pc(MvObject Contour, MvTuple Row, MvTuple Column, MvTuple* DistanceMin, MvTuple* DistanceMax);

/**
 * @brief 两侧轮廓及 Mode 支持 1/N 广播，逐对返回最小/最大距离。
 *
 * @details Mode 为 point_to_point 或 point_to_segment。
 * 当共同长度为 N>1 时空操作数报错；共同长度为 0/1 且含空操作数时返回空结果。
 * 包含一个零点轮廓的对象仍不支持距离查询。
 *
 * @param[in] Contour1 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Contour2 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 * @param[out] DistanceMin 最小距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] DistanceMax 最大距离元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_cc(MvObject Contour1, MvObject Contour2, MvTuple Mode, MvTuple* DistanceMin, MvTuple* DistanceMax);

/**
 * @brief 两个单轮廓的互交/自交查询；任一对象数组为空时三个结果均为空。
 *
 * @details 非空操作数均要求恰好一个轮廓；IntersectionType 为 all、mutual 或 self。
 * 交点排序去重，重叠标志始终检查双方自交及互交。
 *
 * @param[in] Contour1 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] Contour2 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[in] IntersectionType 轮廓交点查询模式元组；调用期间只读借用。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] IsOverlapping 几何关系判定标记元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_intersection_contours_xld(MvObject Contour1, MvObject Contour2, MvTuple IntersectionType, MvTuple* Row, MvTuple* Column, MvTuple* IsOverlapping);

/**
 * @brief 单通道图像数组灰度等值线，Threshold 为单个有限数值。
 *
 * @details byte/uint2/int2 的阈值取最近整数；int4/real 保留小数。
 * 网格单元四顶点须在 domain 内；有效空对象数组跳过控制内容检查。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Border 边界处理方式；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Threshold 边缘或灰度阈值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_threshold_sub_pix(MvObject Image, MvObject* Border, MvTuple Threshold);

/**
 * @brief 高斯中心线提取，控制元组 均为单值，LightDark 为 light/dark。
 *
 * @details 当前 Sigma 范围 [0.5,64]，0 <= Low <= High。
 * ExtractWidth/CompleteJunctions 当前仅实现 false；请求 true 返回不支持。
 * LineModel 在 ExtractWidth=false 时不参与位置校正；空对象数组直接返回空结果。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Lines 中心线轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间只读借用。
 * @param[in] Low 滞后阈值的低阈值元组；调用期间只读借用。
 * @param[in] High 滞后阈值的高阈值元组；调用期间只读借用。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间只读借用。
 * @param[in] ExtractWidth 是否计算线条宽度元组；调用期间只读借用。
 * @param[in] LineModel 线条类型元组；调用期间只读借用。
 * @param[in] CompleteJunctions 是否补全线条交汇点元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_lines_gauss(MvObject Image, MvObject* Lines, MvTuple Sigma, MvTuple Low, MvTuple High, MvTuple LightDark, MvTuple ExtractWidth, MvTuple LineModel, MvTuple CompleteJunctions);

/**
 * @brief 按 area/contlength/row/column/width/height/ratio/circularity 筛选；Operation 为 and/or。
 *
 * @details Min/Max 与 Features 等长，分别支持字符串 min/max 作为开放边界。
 * 有效空对象数组直接返回空结果；当前仅支持 contour 类型。
 *
 * @param[in] XLD 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] SelectedXLD 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Features 属性或特征名称元组；调用期间只读借用。
 * @param[in] Operation 多个特征条件的组合方式元组；调用期间只读借用。
 * @param[in] Min 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Max 筛选或测量范围界限元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_shape_xld(MvObject XLD, MvObject* SelectedXLD, MvTuple Features, MvTuple Operation, MvTuple Min, MvTuple Max);

/**
 * @brief 按六项标量阈值合并同圆弧，MergeSmallContours 为 true/false。
 *
 * @details Iterations 为非负整数值，0 仍执行首次合并；点属性始终保留。
 * 当前连接代价与复杂竞争顺序尚未完全对齐 HALCON；有效空数组跳过控制内容检查。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] UnionContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaxArcAngleDiff 轮廓合并的角度阈值元组；调用期间只读借用。
 * @param[in] MaxArcOverlap 圆弧允许的最大重叠角度元组；调用期间只读借用。
 * @param[in] MaxTangentAngle 轮廓合并的角度阈值元组；调用期间只读借用。
 * @param[in] MaxDist 距离阈值元组；调用期间只读借用。
 * @param[in] MaxRadiusDiff 圆弧中心或半径差的阈值元组；调用期间只读借用。
 * @param[in] MaxCenterDist 圆弧中心或半径差的阈值元组；调用期间只读借用。
 * @param[in] MergeSmallContours 是否合并短轮廓元组；调用期间只读借用。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_union_cocircular_contours_xld(MvObject Contours, MvObject* UnionContours, MvTuple MaxArcAngleDiff, MvTuple MaxArcOverlap, MvTuple MaxTangentAngle, MvTuple MaxDist, MvTuple MaxRadiusDiff, MvTuple MaxCenterDist, MvTuple MergeSmallContours, MvTuple Iterations);

/**
 * @brief HALCON 像素角点约定：HomMat2D 作用于 (row+.5,column+.5)，变换后再减 .5。
 *
 * @details 仿射为六系数，投影接受六仿射或九投影系数；点属性使用局部 Jacobian，线段跨越投影地平线时整体失败。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ContoursAffineTrans 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_contour_xld(MvObject Contours, MvObject* ContoursAffineTrans, MvTuple HomMat2D);

/**
 * @brief 使用投影变换矩阵变换 XLD 轮廓。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ContoursProjTrans 轮廓；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_projective_trans_contour_xld(MvObject Contours, MvObject* ContoursProjTrans, MvTuple HomMat2D);

/**
 * @brief 必要时追加首点实现显式闭合，同时复制首点属性。
 *
 * @param[in] Contours 输入轮廓；调用期间借用，不转移引用所有权。
 * @param[out] ClosedContours 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_close_contours_xld(MvObject Contours, MvObject* ClosedContours);

#ifdef __cplusplus
}
#endif

#endif
