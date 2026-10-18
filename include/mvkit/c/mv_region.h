#ifndef MVKIT_C_MV_REGION_H
#define MVKIT_C_MV_REGION_H

/** @file
 * @brief MVKit 正式区域算子，使用 HALCON 风格 控制元组参数。 */

#include <stddef.h>
#include <stdint.h>
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 生成一个空区域，与零对象数组不同。
 *
 * @param[out] EmptyRegion 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_empty_region(MvObject* EmptyRegion);

/**
 * @brief 由等长 Row/ColumnBegin/ColumnEnd 元组生成一个区域，栅格坐标须在有符号 32 位范围内。
 *
 * @details 有限实数按 HALCON trunc(value+.5) 转换；游程排序并合并。空元组生成一个空区域。
 * 未实现图像格式裁剪及 store_empty_region 策略，仅支持无裁剪/默认存储子集。
 * 非法端点或溢出失败，保持原输出。
 *
 * @param[out] Region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] ColumnBegin 起点列坐标元组；调用期间只读借用。
 * @param[in] ColumnEnd 终点列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_region_runs(MvObject* Region, MvTuple Row, MvTuple ColumnBegin, MvTuple ColumnEnd);

/**
 * @brief 将单区域读入三个独立拥有型整数元组，按行及起始列排序。
 *
 * @details 空区域/零对象数组返回空元组，多区域拒绝；输出必须互异，失败保持原值。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] Row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ColumnBegin 起点列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] ColumnEnd 终点列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_region_runs(MvObject Region, MvTuple* Row, MvTuple* ColumnBegin, MvTuple* ColumnEnd);

/**
 * @brief 生成轴对齐矩形区域。
 *
 * @details 生成控制为数值标量或等长/广播 Tuple；空控制产生有效零对象数组。
 * 全部对象成功后才提交。矩形坐标半值远离零取整，圆半径向下取半像素步长，中心遵循 HALCON 奇偶规则。
 * 未实现图像格式裁剪。
 *
 * @param[out] Rectangle 矩形区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Column1 列坐标元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Column2 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_rectangle1(MvObject* Rectangle, MvTuple Row1, MvTuple Column1, MvTuple Row2, MvTuple Column2);

/**
 * @brief 生成旋转矩形区域。
 *
 * @param[out] Rectangle 矩形区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Phi 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Length1 第一方向的几何半长元组；调用期间只读借用。
 * @param[in] Length2 第二方向的几何半长元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_rectangle2(MvObject* Rectangle, MvTuple Row, MvTuple Column, MvTuple Phi, MvTuple Length1, MvTuple Length2);

/**
 * @brief 生成圆形区域。
 *
 * @param[out] Circle 圆形区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Radius 半径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_circle(MvObject* Circle, MvTuple Row, MvTuple Column, MvTuple Radius);

/**
 * @brief 将 Regions 中全部对象合并为一个 Region 对象。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_union1(MvObject regions,MvObject* output);

/**
 * @brief 按 Regions 语义执行并集。
 *
 * @param[in] left 左侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] right 右侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_union2(MvObject left,MvObject right,MvObject* output);

/**
 * @brief 按 Regions 语义执行交集。
 *
 * @param[in] left 左侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] right 右侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_intersection(MvObject left,MvObject right,MvObject* output);

/**
 * @brief 按 Regions 语义执行差集。
 *
 * @param[in] left 左侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] right 右侧图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_difference(MvObject left,MvObject right,MvObject* output);

/**
 * @brief 将区域按连通性拆分为独立对象。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_connection(MvObject regions,MvObject* output);

/**
 * @brief 填充区域内部孔洞。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fill_up(MvObject regions,MvObject* output);

/**
 * @brief 计算各 Region 对象的面积和重心。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] area 区域或轮廓面积元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_area_center(MvObject regions,MvTuple* area,MvTuple* row,MvTuple* column);

/**
 * @brief 计算各 Region 对象的最小轴对齐外接矩形。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] row1 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] column1 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] row2 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] column2 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_smallest_rectangle1(MvObject regions,MvTuple* row1,MvTuple* column1,MvTuple* row2,MvTuple* column2);

/**
 * @brief 计算各 Region 对象的最小旋转外接矩形。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] phi 方向或旋转角度元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] length1 第一方向的几何半长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] length2 第二方向的几何半长元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_smallest_rectangle2(MvObject regions,MvTuple* row,MvTuple* column,MvTuple* phi,MvTuple* length1,MvTuple* length2);

/**
 * @brief 计算各 Region 对象的最小外接圆。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] row 行坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] column 列坐标元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] radius 半径元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_smallest_circle(MvObject regions,MvTuple* row,MvTuple* column,MvTuple* radius);

/**
 * @brief 按形状特征筛选 Region 对象。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] SelectedRegions 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Features 属性或特征名称元组；调用期间只读借用。
 * @param[in] Operation 多个特征条件的组合方式元组；调用期间只读借用。
 * @param[in] Min 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Max 筛选或测量范围界限元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_shape(MvObject Regions, MvObject* SelectedRegions, MvTuple Features, MvTuple Operation, MvTuple Min, MvTuple Max);

/**
 * @brief 使用圆形结构元素膨胀区域，Radius 为 [0.5,511.5] 中的一个数值。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionDilation 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Radius 半径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_dilation_circle(MvObject Region, MvObject* RegionDilation, MvTuple Radius);

/**
 * @brief 使用圆形结构元素腐蚀区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionErosion 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Radius 半径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_erosion_circle(MvObject Region, MvObject* RegionErosion, MvTuple Radius);

/**
 * @brief 使用圆形结构元素执行开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionOpening 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Radius 半径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_opening_circle(MvObject Region, MvObject* RegionOpening, MvTuple Radius);

/**
 * @brief 使用圆形结构元素执行闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionClosing 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Radius 半径元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_closing_circle(MvObject Region, MvObject* RegionClosing, MvTuple Radius);

/**
 * @brief 使用矩形结构元素膨胀区域，Width/Height 各为一个数值并取最近整数。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionDilation 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_dilation_rectangle1(MvObject Region, MvObject* RegionDilation, MvTuple Width, MvTuple Height);

/**
 * @brief 使用矩形结构元素腐蚀区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionErosion 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_erosion_rectangle1(MvObject Region, MvObject* RegionErosion, MvTuple Width, MvTuple Height);

/**
 * @brief 使用矩形结构元素执行开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionOpening 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_opening_rectangle1(MvObject Region, MvObject* RegionOpening, MvTuple Width, MvTuple Height);

/**
 * @brief 使用矩形结构元素执行闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionClosing 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_closing_rectangle1(MvObject Region, MvObject* RegionClosing, MvTuple Width, MvTuple Height);

/**
 * @brief 按区域优先展平的 实数元组，每个请求的特征对应一个值。
 *
 * @details Features 使用支持的 HALCON 字符串，可重复。空区域返回零，零区域对象返回空控制元组（Features 为空也如此）。
 * 非空区域要求 Features；二阶矩不归一化，phi 为等效椭圆角。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Features 属性或特征名称元组；调用期间只读借用。
 * @param[out] Value 保存、追加或读取的数据元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_region_features(MvObject Regions, MvTuple Features, MvTuple* Value);

/**
 * @brief min(1,area/(pi*max_center_distance^2))；单像素或空区域为零。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] values 保存、追加或读取的数据元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_circularity(MvObject regions, MvTuple* values);

/**
 * @brief HALCON 按轮廓定向的主轴，弧度范围 [-pi,pi)；空区域为零。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] values 保存、追加或读取的数据元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_orientation_region(MvObject regions, MvTuple* values);

/**
 * @brief Anisometry=Ra/Rb，Bulkiness=pi*Ra*Rb/area，StructureFactor=Anisometry*Bulkiness-1。
 *
 * @details 半径为协方差特征值平方根的两倍。空区域输出三个零，非空退化区域输出 (0,0,-1)。
 * 输出必须非空且互异，整体提交。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] anisometry 区域形状特征元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] bulkiness 区域形状特征元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] structure_factor 区域形状特征元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_eccentricity(MvObject regions, MvTuple* anisometry, MvTuple* bulkiness, MvTuple* structure_factor);

/**
 * @brief HALCON 字符串控制：SortMode、Order=true/false、RowOrCol=row/column。
 *
 * @details Character 模式接受第二个重叠百分比（默认 15）；重叠行的传递分组仍有差分残差。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] SortedRegions 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] SortMode 排序依据元组；调用期间只读借用。
 * @param[in] Order 排序、通道或点序控制元组；调用期间只读借用。
 * @param[in] RowOrCol 分块或排序使用的行列方向元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_sort_region(MvObject Regions, MvObject* SortedRegions, MvTuple SortMode, MvTuple Order, MvTuple RowOrCol);

/**
 * @brief max_area 选择首个最大对象；Percent 必须有限，可超出 [0,100]，在 max_area 中忽略。
 *
 * @details 零对象保持为空，全空对象选择首个空对象；Rectangle1/2 选择占用率 > Percent 的非空区域。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] SelectedRegions 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Shape 区域变换的目标形状元组；调用期间只读借用。
 * @param[in] Percent 百分比控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_select_shape_std(MvObject Regions, MvObject* SelectedRegions, MvTuple Shape, MvTuple Percent);

/**
 * @brief 按整数偏移平移每个区域；偏移和结果坐标须在有符号 32 位范围内，越界保留原输出。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionMoved 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_move_region(MvObject Region, MvObject* RegionMoved, MvTuple Row, MvTuple Column);

/**
 * @brief 每个左侧对象与全部右侧对象的并集异或，保持左侧对象数量。
 *
 * @param[in] Region1 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Region2 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionDifference 区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_symm_difference(MvObject Region1, MvObject Region2, MvObject* RegionDifference);

/**
 * @brief 返回一个 Int64 布尔值，判断对象数相同且逐个像素集合相等。
 *
 * @details 任一输入为零对象时返回空控制元组。
 *
 * @param[in] Regions1 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Regions2 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] IsEqual 几何关系判定标记元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_test_equal_region(MvObject Regions1, MvObject Regions2, MvTuple* IsEqual);

/**
 * @brief 任一点命中判断：一个点配 N 个区域，或 M 个点配一个区域。
 *
 * @details Row/Column 必须等长，实数坐标按 floor(value+0.5) 取整。零对象返回空控制元组，非空区域至少要求一个点。
 * 搜索前验证全部控制，包括首次命中之后的坐标。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[out] IsInside 几何关系判定标记元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_test_region_point(MvObject Regions, MvTuple Row, MvTuple Column, MvTuple* IsInside);

/**
 * @brief 填充闭合多边形，使用 even-odd 内部规则并包含像素中心边界。
 *
 * @details Rows/Columns 为等长数值元组，栅格化前顶点取整。零顶点生成一个空区域，一/两个顶点生成点/线。
 * 坐标必须有限且在 [-2^30,2^30] 内，始终产生一个区域（可为空）。
 *
 * @param[out] Region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Rows 行坐标元组；调用期间只读借用。
 * @param[in] Columns 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_region_polygon_filled(MvObject* Region, MvTuple Rows, MvTuple Columns);

/**
 * @brief 使用 scalar-or-N 数值控制批量生成椭圆。
 *
 * @details 中心/半径须有限且在 +/-2^30 内，半径非负。零半径生成一个空区域，空控制生成零对象。
 * 等半径采用规范圆栅格；不等半径/旋转栅格保留已登记的 HALCON 差异。
 *
 * @param[out] Ellipse 椭圆区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] Phi 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Radius1 椭圆长半轴元组；调用期间只读借用。
 * @param[in] Radius2 椭圆短半轴元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gen_ellipse(MvObject* Ellipse, MvTuple Row, MvTuple Column, MvTuple Phi, MvTuple Radius1, MvTuple Radius2);

/**
 * @brief 将区域转换为指定的几何形状。
 *
 * @details 其他区域算子的拓扑和数值契约见 region-morphology.md。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionTrans 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Type 数据、模型或原语类型元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_shape_trans(MvObject Region, MvObject* RegionTrans, MvTuple Type);

/**
 * @brief 六个有限正向系数 [a,b,tx,c,d,ty]，作用于像素角点 (row,column)。
 *
 * @details 仅实现 nearest_neighbor，constant 拒绝；缩放/旋转逆采样残差见 migration-108 差分记录。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionAffineTrans 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Interpolate 插值方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_region(MvObject Region, MvObject* RegionAffineTrans, MvTuple HomMat2D, MvTuple Interpolate);

/**
 * @brief BoundaryType 为 inner/inner_filled/outer。inner/outer 使用八邻域形态学；
 *
 * @details inner_filled 填充八连通孔洞，腐蚀连通性与 runtime neighborhood 互补。保持对象数量。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionBorder 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] BoundaryType 区域边界提取方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_boundary(MvObject Region, MvObject* RegionBorder, MvTuple BoundaryType);

/**
 * @brief Feature 支持 area/anisometry/convexity/phi/ra/rb；compactness/inner_circle/outer_circle 暂未实现。
 *
 * @details 有限有序 Min/Max 包含端点，孔洞连通性与 runtime neighborhood 互补。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] RegionFillUp 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Feature 属性或特征名称元组；调用期间只读借用。
 * @param[in] Min 筛选或测量范围界限元组；调用期间只读借用。
 * @param[in] Max 筛选或测量范围界限元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_fill_up_shape(MvObject Region, MvObject* RegionFillUp, MvTuple Feature, MvTuple Min, MvTuple Max);

/**
 * @brief Zhang-Suen 候选细化并逐步检查拓扑，前景八连通、背景四连通。
 *
 * @param[in] regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] output 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_skeleton(MvObject regions,MvObject* output);

/**
 * @brief 每个区域返回一张 Int32 图像。Metric 为 euclidean/city-block/chessboard，
 *
 * @details Foreground 为 true/false，Width/Height 为正整数；输入裁剪到画布。
 * 未实现 chamfer/octagonal，以及 Foreground=false 且裁剪后为空时 HALCON 的有限哨兵值。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] DistanceImage 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Metric 距离或匹配度量元组；调用期间只读借用。
 * @param[in] Foreground 距离变换的前景选择元组；调用期间只读借用。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_distance_transform(MvObject Region, MvObject* DistanceImage, MvTuple Metric, MvTuple Foreground, MvTuple Width, MvTuple Height);

/**
 * @brief 按对象均衡网格分块，Width/Height 为有限正标量。
 *
 * @details 保留空单元，省略尾部空行带；区域包围范围和分块数量须在 Int32 内。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] Partitioned 分块区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_partition_rectangle(MvObject Region,MvObject* Partitioned,MvTuple Width,MvTuple Height);

/**
 * @brief 均衡列切分向最小列面积移动，Distance>=1，Percent 在 [0,100] 内。
 *
 * @details 保留空块，跳过被替代的切点。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] Partitioned 分块区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Distance 距离元组；调用期间只读借用。
 * @param[in] Percent 百分比控制元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_partition_dynamic(MvObject Region,MvObject* Partitioned,MvTuple Distance,MvTuple Percent);

/**
 * @brief 位置无关的自定义形态学，使用一个结构元素对象，参考点为取整质心。
 *
 * @details 膨胀使用反射元素，腐蚀使用原元素，开闭运算各组合一次 Minkowski 阶段。
 * 仅 Dilation1/Erosion1 接受正 Iterations；不支持空元素腐蚀，元素坐标在 +/-2^30 内。溢出时整批失败。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionDilation 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_dilation1(MvObject Region, MvObject StructElement, MvObject* RegionDilation, MvTuple Iterations);

/**
 * @brief 使用位置无关的结构区域腐蚀输入区域。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionErosion 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Iterations 拟合或优化迭代次数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_erosion1(MvObject Region, MvObject StructElement, MvObject* RegionErosion, MvTuple Iterations);

/**
 * @brief 使用位置无关的结构区域执行区域开运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionOpening 区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_opening(MvObject Region, MvObject StructElement, MvObject* RegionOpening);

/**
 * @brief 使用位置无关的结构区域执行区域闭运算。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionClosing 区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_closing(MvObject Region, MvObject StructElement, MvObject* RegionClosing);

/**
 * @brief 使用位置无关结构区域的区域顶帽/底帽。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionTopHat 区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_top_hat(MvObject Region, MvObject StructElement, MvObject* RegionTopHat);

/**
 * @brief 计算区域闭运算结果与原区域之差。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] StructElement 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionBottomHat 区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_bottom_hat(MvObject Region, MvObject StructElement, MvObject* RegionBottomHat);

#ifdef __cplusplus
}
#endif
#endif
