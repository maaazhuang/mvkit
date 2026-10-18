#ifndef MVKIT_C_MV_IMAGE_PROCESSING_H
#define MVKIT_C_MV_IMAGE_PROCESSING_H

/** @file
 * @brief MVKit Public C ABI 头文件。
 */

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
 * @brief 单通道图像定义域的 marker minimax 洪泛，每个 marker 一个盆地，按图像顺序输出。
 *
 * @details 后置 marker 覆盖重叠；域外种子忽略，无种子连通分量不分配，不添加隐式背景。
 * 等代价保持 FIFO（栅格种子，N/W/E/S 后对角），不删除脊线像素。域内像素须有限，支持五种类型。
 * 单像素 padding 的定义域包围盒 <=16777216 像素，marker 数 <=65535，输出整体提交。
 * 正式算子在有 marker 时使用一张 byte/uint2/real 图像的首通道，无图像或 marker 时返回零对象，默认 8 连通。
 * 独立扩展保留图像数组、五种类型及 4/8 连通控制。
 *
 * @param[in] image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] markers 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] basins 分水岭区域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_watersheds_marker(MvObject image, MvObject markers,
                                    MvObject* basins);

/**
 * @brief 逐对象逐通道转换到指定像素类型，保持定义域及对象顺序。
 *
 * @details Image：输入 0..N Image；支持全部五种 MvPixelType 和非连续 stride。
 * ImageConverted：新 owned 图像对象数组；失败不覆盖旧输出；域外像素置零。
 * NewType：输出类型；整数转换半值远离零舍入并饱和，NaN 转零，Inf 饱和。
 * MV_STATUS_OK 或结构化错误；浮点输出保留 NaN/Inf。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageConverted 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] NewType 输出像素类型元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_convert_image_type(MvObject Image,MvObject* ImageConverted,MvTuple NewType);

/**
 * @brief 按 从 1 开始的通道索引 获取单个图像的单通道图像，保持定义域。
 *
 * @details Image：共享源像素的 owned handle；源释放后仍有效，Public 写入口 detach。
 * MV_STATUS_OK 或结构化错误；空对象 tuple 返回空 tuple。
 *
 * @param[in] MultiChannelImage 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] Image 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Channel 通道索引元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_access_channel(MvObject MultiChannelImage,MvObject* Image,MvTuple Channel);

/**
 * @brief 三个等数量单通道 图像对象数组 合成三通道对象。
 *
 * @details MultiChannelImage：共享源存储；每组尺寸须相同，通道可不同类型，domain 取交集。
 * MV_STATUS_OK 或结构化错误；对象数量必须一致，包括 no-object。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image3 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] MultiChannelImage 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_compose3(MvObject Image1,MvObject Image2,MvObject Image3,MvObject* MultiChannelImage);

/**
 * @brief 逐对象拆分恰好三个通道的 图像对象数组，各结果共享像素并保留定义域。
 *
 * @details MV_STATUS_OK 或结构化错误；三个输出地址须非空且互异，整批成功才提交。
 *
 * @param[in] MultiChannelImage 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] Image1 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Image2 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] Image3 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_decompose3(MvObject MultiChannelImage,MvObject* Image1,MvObject* Image2,MvObject* Image3);

/**
 * @brief 对单个单通道图像 的各 Region 返回均值和总体标准差。
 *
 * @details MV_STATUS_OK 或结构化错误；忽略 Image domain，Region 裁剪到图像边界；空区域返回 0/0，no-object 返回空
 * Tuple。
 * 两输出地址须非空且互异；每个区域一个值；支持全部五种像素类型，NaN/Inf 报错。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Mean 灰度均值或标准差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Deviation 灰度均值或标准差元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_intensity(MvObject Regions,MvObject Image,MvTuple* Mean,MvTuple* Deviation);

/**
 * @brief 单个 Region、单个单通道图像 的 256-bin 直方图，支持五种像素类型。
 *
 * @details AbsoluteHisto：256 个 Int64 计数；byte 使用原值，其余类型按区域极值缩放到 0..255，常量映射到 0。
 * RelativeHisto：按裁剪区域面积归一化；忽略 Image domain，Region 裁剪到图像边界。
 * MV_STATUS_OK 或结构化错误；空区域返回 256 个零，no-object 返回空控制元组，输出地址互异。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] AbsoluteHisto 各灰度桶的像素计数元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] RelativeHisto 归一化灰度直方图元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gray_histo(MvObject Region,MvObject Image,MvTuple* AbsoluteHisto,MvTuple* RelativeHisto);

/**
 * @brief 对 UInt8/UInt16 单通道图像 逐对象执行最大类间方差自动阈值。
 *
 * @details Region：每幅图一个 Region；Light > threshold，Dark <= threshold。
 * UsedThreshold：每幅图的 owned integer 阈值；现有内核取首个最大评分切点，无对比度及空定义域返回
 * 0；一般直方图阈值差异继续保留。
 * MV_STATUS_OK 或结构化错误；空定义域保留一个空区域；no-object 返回空对象/控制元组。
 * 两个输出整批成功才提交；域外不参与统计；多通道只使用首通道；smooth_histo 显式 Unsupported。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间只读借用。
 * @param[out] UsedThreshold 自动选择的灰度阈值元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_binary_threshold(MvObject Image, MvObject* Region, MvTuple Method, MvTuple LightDark, MvTuple* UsedThreshold);

/**
 * @brief 逐对象获取 0..N 图像对象数组 的当前定义域。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Domain 图像定义域；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_domain(MvObject Image,MvObject* Domain);

/**
 * @brief 使用第一个 Region 收缩各 Image 的定义域，与 HALCON 的 Region 输入语义一致。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[out] ImageReduced 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_reduce_domain(MvObject Image,MvObject Region,MvObject* ImageReduced);

/**
 * @brief 逐对象恢复 0..N 图像对象数组 的完整定义域，并保持顺序。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageFull 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_full_domain(MvObject Image,MvObject* ImageFull);

/**
 * @brief 逐 Image object 按灰度范围生成 Region object。
 *
 * @details image：输入 0..N Image objects。
 * region：输出 区域对象数组，成功时 cardinality 与 image 一致。
 * min_gray：数值元组，长度为 1 或 image cardinality。
 * max_gray：数值元组，长度为 1 或 image cardinality。
 *
 * @param[in] image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] min_gray 灰度下限元组；调用期间只读借用。
 * @param[in] max_gray 灰度上限元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_threshold(MvObject image, MvObject* region,
    MvTuple min_gray, MvTuple max_gray);

/**
 * @brief 按 scalar-or-N 规则比较两个 图像对象数组 并生成 区域对象数组。
 *
 * @details 非空图像数组必须等长；整数图像的 Offset 取整，real 图像保持小数。任一输入为空时跳过控制。
 *
 * @param[in] OrigImage 原始图像；调用期间借用，不转移引用所有权。
 * @param[in] ThresholdImage 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] RegionDynThresh 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Offset 灰度或坐标偏移元组；调用期间只读借用。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_dyn_threshold(MvObject OrigImage, MvObject ThresholdImage, MvObject* RegionDynThresh, MvTuple Offset, MvTuple LightDark);

/**
 * @brief 逐对象执行均值滤波并保持 图像对象数组 顺序。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageMean 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_mean_image(MvObject Image, MvObject* ImageMean, MvTuple MaskWidth, MvTuple MaskHeight);

/**
 * @brief 逐对象执行高斯滤波并保持 图像对象数组 顺序。
 *
 * @details 奇数尺寸 1..63；尺寸 1 为恒等。3/5/7/9/11 保留既有 sigma，其他尺寸使用 (size-1)/6。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageGauss 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Size 元素或键值项数量元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gauss_filter(MvObject Image, MvObject* ImageGauss, MvTuple Size);

/**
 * @brief 逐对象执行中值滤波并保持 图像对象数组 顺序。
 *
 * @details MaskType 为 "square"/"circle"，Radius 为 1..4095。
 * Margin 为 "mirrored"、"continued"、"cyclic" 或数值常量；支持图像数组，保持对象顺序和通道。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageMedian 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskType 中值滤波窗口形状元组；调用期间只读借用。
 * @param[in] Radius 半径元组；调用期间只读借用。
 * @param[in] Margin 边缘或边界处理模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_median_image(MvObject Image, MvObject* ImageMedian,
    MvTuple MaskType, MvTuple Radius, MvTuple Margin);

/**
 * @brief 执行线性灰度缩放。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageScaled 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 * @param[in] Add 灰度加法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_scale_image(MvObject Image, MvObject* ImageScaled, MvTuple Mult, MvTuple Add);

/**
 * @brief 对等数量的 Image 对象逐对求和；控制参数各一个有限数值。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageResult 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 * @param[in] Add 灰度加法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_add_image(MvObject Image1, MvObject Image2, MvObject* ImageResult, MvTuple Mult, MvTuple Add);

/**
 * @brief 按 scalar-or-N 规则执行双 图像对象数组 减法。
 *
 * @param[in] ImageMinuend 被减图像；调用期间借用，不转移引用所有权。
 * @param[in] ImageSubtrahend 减数图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageSub 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 * @param[in] Add 灰度加法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_sub_image(MvObject ImageMinuend, MvObject ImageSubtrahend, MvObject* ImageSub, MvTuple Mult, MvTuple Add);

/**
 * @brief 按 scalar-or-N 规则执行双 图像对象数组 乘法。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageResult 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 * @param[in] Add 灰度加法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_mult_image(MvObject Image1, MvObject Image2, MvObject* ImageResult, MvTuple Mult, MvTuple Add);

/**
 * @brief 对区域数组与单张图像返回 最小值 Min、最大值 Max 和范围 Range 的拥有型实数元组；忽略图像定义域。
 *
 * @param[in] Regions 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Percent 百分比控制元组；调用期间只读借用。
 * @param[out] Min 筛选或测量范围界限元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Max 筛选或测量范围界限元组；成功后由调用方使用 mv_destroy_tuple 释放。
 * @param[out] Range 灰度动态范围元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_min_max_gray(MvObject Regions, MvObject Image, MvTuple Percent, MvTuple* Min, MvTuple* Max, MvTuple* Range);

/**
 * @brief 计算 Sobel 边缘幅值。
 *
 * @details FilterType 使用 HALCON 字符串，Size 接受一或两个值；各向同性基础滤波支持奇数尺寸 3..13。
 * 细化、各向异性尺寸和大于 3 的二项核返回 Unsupported。带符号 byte 输出暂用 int2，HALCON 使用 int1；
 * 其他输出类型/核差异保留在严格差分报告中。空 Image 跳过控制。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] EdgeAmplitude 线条或边缘的响应幅值；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] FilterType 边缘滤波器类型元组；调用期间只读借用。
 * @param[in] Size 元素或键值项数量元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_sobel_amp(MvObject Image, MvObject* EdgeAmplitude, MvTuple FilterType, MvTuple Size);

/**
 * @brief 按轴对齐矩形裁剪图像。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImagePart 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row1 行坐标元组；调用期间只读借用。
 * @param[in] Column1 列坐标元组；调用期间只读借用。
 * @param[in] Row2 行坐标元组；调用期间只读借用。
 * @param[in] Column2 列坐标元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_crop_rectangle1(MvObject Image, MvObject* ImagePart, MvTuple Row1, MvTuple Column1, MvTuple Row2, MvTuple Column2);

/**
 * @brief 镜像图像。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageMirror 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mode 处理或查询模式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_mirror_image(MvObject Image, MvObject* ImageMirror, MvTuple Mode);

/**
 * @brief 缩放图像到指定尺寸。
 *
 * @details 支持五种插值：Constant 等同 nearest，Weighted 使用面积积分，Bicubic 使用 Keys a=-0.5。
 * nearest/linear/cubic 缩小时用 dst*scale，放大时用 (dst+0.5)*scale-0.5；输出为完整定义域。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageZoom 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_zoom_image_size(MvObject Image, MvObject* ImageZoom, MvTuple Width, MvTuple Height, MvTuple Interpolation);

/**
 * @brief 将 RGB/BGR 图像转换为灰度图。
 *
 * @param[in] RGBImage 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] GrayImage 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_rgb1_to_gray(MvObject RGBImage,MvObject* GrayImage);

/**
 * @brief abs(a-b)*mult；平面类型一致，定义域取交集，对象按 scalar-or-N 配对。
 *
 * @details 整数半值远离零舍入并饱和；mult 必须有限。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageAbsDiff 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_abs_diff_image(MvObject Image1, MvObject Image2, MvObject* ImageAbsDiff, MvTuple Mult);

/**
 * @brief (a/b)*mult+add；除数为零时结果为零，不受 add 影响。类型/定义域规则同 abs_diff_image。
 *
 * @param[in] Image1 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Image2 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageResult 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Mult 灰度乘法系数元组；调用期间只读借用。
 * @param[in] Add 灰度加法系数元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_div_image(MvObject Image1, MvObject Image2, MvObject* ImageResult, MvTuple Mult, MvTuple Add);

/**
 * @brief 无符号为 max-value，有符号/浮点为 -value（有符号最小值饱和）。保持定义域，域外置零。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageInvert 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_invert_image(MvObject Image,MvObject* ImageInvert);

/**
 * @brief 裁剪到定义域包围盒并平移到新原点；空定义域报错，零对象保持为空。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImagePart 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_crop_domain(MvObject Image,MvObject* ImagePart);

/**
 * @brief 按顺序将区域绘入每张图像的独立副本，限制在图像定义域内。
 *
 * @details Grayval 可为单值、逐通道值或按区域优先展平的通道值，必须为有限数值。
 * Margin 为原区域四邻域内边界，后置区域覆盖重叠；保留未绘制栅格和定义域。无区域时复制，无图像时返回零对象。
 *
 * @param[in] Region 输入区域；调用期间借用，不转移引用所有权。
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageResult 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Grayval 灰度值元组；调用期间只读借用。
 * @param[in] Type 数据、模型或原语类型元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_paint_region(MvObject Region, MvObject Image, MvObject* ImageResult, MvTuple Grayval, MvTuple Type);

/**
 * @brief HALCON HomMat2D 使用像素角点 x=row,y=column；接受六系数或末行 (0,0,1) 的九系数矩阵。
 *
 * @details AdaptImageSize 控制输出尺寸，忽略输入定义域。支持 nearest/bilinear/bicubic，以及不缩小时的
 * constant/weighted。
 * 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageAffineTrans 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 * @param[in] AdaptImageSize 是否调整输出图像尺寸元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_affine_trans_image(MvObject Image, MvObject* ImageAffineTrans, MvTuple HomMat2D, MvTuple Interpolation, MvTuple AdaptImageSize);

/**
 * @brief HALCON 行优先 3x3 单应矩阵，使用像素角点 x=row,y=column。
 *
 * @details AdaptImageSize/TransformDomain 控制输出；奇异矩阵报错，投影分母为零时输出对应位置无效。
 * 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] TransImage 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] HomMat2D 二维变换矩阵元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 * @param[in] AdaptImageSize 是否调整输出图像尺寸元组；调用期间只读借用。
 * @param[in] TransformDomain 是否变换图像定义域元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_projective_trans_image(MvObject Image, MvObject* TransImage, MvTuple HomMat2D, MvTuple Interpolation, MvTuple AdaptImageSize, MvTuple TransformDomain);

/**
 * @brief 角度须有限，以度表示，绕 ((width-1)/2,(height-1)/2) 逆时针旋转；90/270 度交换宽高。
 *
 * @details 忽略源定义域，支持 nearest/constant/bilinear/bicubic。整批成功后提交输出，定义域/舍入见
 * image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageRotate 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Phi 方向或旋转角度元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_rotate_image(MvObject Image, MvObject* ImageRotate, MvTuple Phi, MvTuple Interpolation);

/**
 * @brief 一个 int4 线性索引 Map 或五通道 int4/uint2 权重 Map 决定输出尺寸和定义域。
 *
 * @details 图像对象共用一个 Map，忽略输入定义域；读取前拒绝非法索引。整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[in] Map 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] ImageMapped 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_map_image(MvObject Image, MvObject Map, MvObject* ImageMapped);

/**
 * @brief 角度为弧度，从列正轴逆时针计；端点有限，半径非负，角度/半径方向均可反向。
 *
 * @details 忽略输入定义域，输出完整定义域；列包含 angle_end，行包含两端半径。
 * width=1 使用 angle_start，height=1 使用 radius_start。整批成功后提交输出，定义域/舍入见
 * image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] PolarTransImage 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Row 行坐标元组；调用期间只读借用。
 * @param[in] Column 列坐标元组；调用期间只读借用。
 * @param[in] AngleStart 起始角度元组；调用期间只读借用。
 * @param[in] AngleEnd 终止角度元组；调用期间只读借用。
 * @param[in] RadiusStart 极坐标采样起始半径元组；调用期间只读借用。
 * @param[in] RadiusEnd 极坐标采样终止半径元组；调用期间只读借用。
 * @param[in] Width 图像或分块宽度元组；调用期间只读借用。
 * @param[in] Height 图像或分块高度元组；调用期间只读借用。
 * @param[in] Interpolation 插值方式元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_polar_trans_image_ext(MvObject Image, MvObject* PolarTransImage, MvTuple Row, MvTuple Column, MvTuple AngleStart, MvTuple AngleEnd, MvTuple RadiusStart, MvTuple RadiusEnd, MvTuple Width, MvTuple Height, MvTuple Interpolation);

/**
 * @brief 奇数宽高 [1,4095] 窗口内的灰度最小值，continued 边界；支持五种类型及混合平面，输入必须有限。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageMin 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gray_erosion_rect(MvObject Image, MvObject* ImageMin, MvTuple MaskHeight, MvTuple MaskWidth);

/**
 * @brief 奇数宽高 [1,4095] 窗口内的灰度最大值，continued 边界；支持五种类型及混合平面，输入必须有限。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageMax 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gray_dilation_rect(MvObject Image, MvObject* ImageMax, MvTuple MaskHeight, MvTuple MaskWidth);

/**
 * @brief 奇数矩形窗口 [1,4095] 的灰度腐蚀后膨胀；continued 边界，两阶段之间保留完整栅格 halo。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageOpening 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gray_opening_rect(MvObject Image, MvObject* ImageOpening, MvTuple MaskHeight, MvTuple MaskWidth);

/**
 * @brief 奇数矩形窗口 [1,4095] 的灰度膨胀后腐蚀；continued 边界，两阶段之间保留完整栅格 halo。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageClosing 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_gray_closing_rect(MvObject Image, MvObject* ImageClosing, MvTuple MaskHeight, MvTuple MaskWidth);

/**
 * @brief HALCON HSV 约定：三个独立平面，保持类型；real hue 使用弧度。
 *
 * @param[in] ImageRed 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] ImageGreen 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[in] ImageBlue 图标对象数组；调用期间借用，不转移引用所有权。
 * @param[out] ImageResult1 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] ImageResult2 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] ImageResult3 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] ColorSpace 颜色空间名称元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_trans_from_rgb(MvObject ImageRed, MvObject ImageGreen, MvObject ImageBlue,
                                MvObject* ImageResult1, MvObject* ImageResult2, MvObject* ImageResult3, MvTuple ColorSpace);

/**
 * @brief 将指定颜色空间的三个图像通道转换为 RGB 通道。
 *
 * @param[in] ImageInput1 输入颜色通道图像；调用期间借用，不转移引用所有权。
 * @param[in] ImageInput2 输入颜色通道图像；调用期间借用，不转移引用所有权。
 * @param[in] ImageInput3 输入颜色通道图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageRed 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] ImageGreen 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[out] ImageBlue 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] ColorSpace 颜色空间名称元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_trans_to_rgb(MvObject ImageInput1, MvObject ImageInput2, MvObject ImageInput3,
                              MvObject* ImageRed, MvObject* ImageGreen, MvObject* ImageBlue, MvTuple ColorSpace);

/**
 * @brief Sigma 在 [0.1,32] 内，order_x/order_y 非负且总和 <=2。
 *
 * @details 半径 ceil(3*sigma)，mirrored 边界，逐通道输出 Float32，离散矩归一化。
 * 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] DerivGauss 图标对象数组；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间只读借用。
 * @param[in] Component 高斯导数分量元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_derivate_gauss(MvObject Image, MvObject* DerivGauss, MvTuple Sigma, MvTuple Component);

/**
 * @brief 量化 Laplacian-of-Gaussian，sigma 在 [0.1,32] 内，mirrored 边界，逐通道输出 Int16。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageLaplace 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_laplace_of_gauss(MvObject Image, MvObject* ImageLaplace, MvTuple Sigma);

/**
 * @brief Image + factor*(Image-local_mean)；有限 factor >=0，奇数矩形宽高 [1,4095]，mirrored 边界，保持类型/定义域。
 *
 * @details 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageEmphasize 图像；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] MaskWidth 矩形窗口宽度元组；调用期间只读借用。
 * @param[in] MaskHeight 矩形窗口高度元组；调用期间只读借用。
 * @param[in] Factor 强调或缩放因子元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_emphasize(MvObject Image, MvObject* ImageEmphasize, MvTuple MaskWidth, MvTuple MaskHeight, MvTuple Factor);

/**
 * @brief 单通道 UInt8/UInt16 直方图谷值分割；Gaussian 直方图 sigma 在 [0,32] 内，0 禁用平滑。
 *
 * @details 256 个桶，UInt16 按 value/256 映射；非空类别按灰度排列，按图像优先展平。
 * 整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Regions 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Sigma 高斯平滑尺度元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_auto_threshold(MvObject Image, MvObject* Regions, MvTuple Sigma);

/**
 * @brief 使用 adapted_std_deviation，generic 参数为 mask_size、scale、range；采用 byte/uint2 首通道。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] Region 区域；成功后由调用方使用 mv_clear_obj 释放。
 * @param[in] Method 算法方法元组；调用期间只读借用。
 * @param[in] LightDark 前景亮暗或线条极性元组；调用期间只读借用。
 * @param[in] GenParamName 参数或数据项名称元组；调用期间只读借用。
 * @param[in] GenParamValue 参数或数据项的值元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_local_threshold(MvObject Image, MvObject* Region, MvTuple Method, MvTuple LightDark, MvTuple GenParamName, MvTuple GenParamValue);

/**
 * @brief 单通道 UInt8/UInt16 定义域内 CDF-min 均衡，分别使用 256/65536 个桶。
 *
 * @details 常量灰度不变，保留空定义域。整批成功后提交输出，定义域/舍入见 image-processing.md。
 *
 * @param[in] Image 输入图像；调用期间借用，不转移引用所有权。
 * @param[out] ImageEquHisto 图像；成功后由调用方使用 mv_clear_obj 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_equ_histo_image(MvObject Image, MvObject* ImageEquHisto);

#ifdef __cplusplus
}
#endif

#endif
