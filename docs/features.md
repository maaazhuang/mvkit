# 功能与使用范围

mvkit 的主要能力分为图像处理、Region、XLD、模板匹配、标定和三维六部分。下文列出常用处理链和代表接口；完整签名及支持的参数以 [C++ 公开头](../include/mvkit/cpp/) 和 [C 公开头](../include/mvkit/c/) 为准。C++ 函数式与对象式接口可组合使用，资源与错误处理见[接口使用说明](api.md)。

## 图像处理

图像对象承载像素、通道及有效定义域，可用于预处理、分割和后续定位。支持 BMP、JPEG、PNG、TIFF 读写。

| 能力 | 说明与代表接口 |
|---|---|
| 图像与通道 | 创建、读取和保存图像；像素类型转换、单通道访问、三通道组合与拆分；`ReadImage`、`WriteImage`、`ConvertImageType`、`Compose3`、`Decompose3` |
| 滤波与增强 | 均值、高斯、中值滤波，锐化、直方图均衡和灰度缩放；`MeanImage`、`GaussFilter`、`MedianImage`、`Emphasize`、`EquHistoImage`、`ScaleImage` |
| 灰度运算与颜色 | 加减乘除、绝对差、反色、RGB 灰度化及颜色空间转换；`AbsDiffImage`、`InvertImage`、`Rgb1ToGray`、`TransFromRgb`、`TransToRgb` |
| 阈值分割 | 固定阈值、自动阈值、局部和动态阈值；`Threshold`、`BinaryThreshold`、`AutoThreshold`、`LocalThreshold`、`DynThreshold` |
| 灰度形态学与导数 | 灰度腐蚀/膨胀/开闭运算，Sobel、高斯导数、LoG 和标记分水岭；`GrayOpeningRect`、`SobelAmp`、`DerivateGauss`、`LaplaceOfGauss`、`WatershedsMarker` |
| ROI 与几何映射 | 定义域缩减/恢复、裁剪、缩放、镜像、旋转、仿射/投影和极坐标映射；`ReduceDomain`、`CropDomain`、`ZoomImageSize`、`AffineTransImage`、`ProjectiveTransImage`、`MapImage` |

常用流程为读取图像 → 灰度化 → 滤波 → 阈值分割，再交给 Region 做目标筛选，或交给 XLD、模板匹配做定位。图像定义域用于限定处理范围，通道数与图像对象数量是不同概念。

## Region

Region 表达像素集合及其拓扑关系，适合目标分割、连通域分析、外观检测和 ROI 管理。私有存储使用 RLE 游程，公开接口通过区域对象操作。

| 能力 | 说明与代表接口 |
|---|---|
| 区域创建与变换 | 矩形、圆、椭圆、多边形、游程创建；平移与仿射变换；`GenRectangle1`、`GenCircle`、`GenRegionPolygonFilled`、`GenRegionRuns`、`MoveRegion`、`AffineTransRegion` |
| 连通性与集合运算 | 连通域拆分、合并、交集、差集和对称差；`Connection`、`Union1`、`Union2`、`Intersection`、`Difference`、`SymmDifference` |
| 形态学与拓扑 | 圆形/矩形及自定义结构元素的腐蚀、膨胀、开闭运算；填洞、边界、骨架与距离变换；`OpeningCircle`、`ClosingRectangle1`、`FillUp`、`Boundary`、`Skeleton`、`DistanceTransform` |
| 特征与筛选 | 面积、重心、圆度、方向、偏心率和包围几何；按特征筛选、排序及分区；`AreaCenter`、`Circularity`、`RegionFeatures`、`SelectShape`、`SortRegion`、`PartitionRectangle` |
| 灰度统计与显示 | 区域内灰度范围、均值/偏差、直方图和着色；`MinMaxGray`、`Intensity`、`GrayHisto`、`PaintRegion` |

典型外观处理链为 Threshold → OpeningCircle → Connection → SelectShape → FillUp。区域坐标及移动偏移限制为有符号 32 位；面积和溢出敏感计算使用 64 位，公开控制 Tuple 保留 int64。

## XLD

XLD 保存亚像素轮廓点及相关属性，适合边缘定位、轮廓处理和几何拟合，可与 Region 相互转换。

| 能力 | 说明与代表接口 |
|---|---|
| 轮廓提取 | Canny/Sobel 亚像素边缘、等值线与高斯线条中心；`EdgesSubPix`、`ThresholdSubPix`、`LinesGauss` |
| 生成与转换 | 多边形、圆、椭圆轮廓生成，区域边界与轮廓互转；`GenContourPolygonXld`、`GenCircleContourXld`、`GenContourRegionXld`、`GenRegionContourXld` |
| 整理与变换 | 平滑、裁剪、闭合、分割、排序、仿射/投影变换；`SmoothContoursXld`、`CropContoursXld`、`CloseContoursXld`、`SegmentContoursXld`、`AffineTransContourXld` |
| 轮廓合并 | 相邻、共线和共圆轮廓合并；`UnionAdjacentContoursXld`、`UnionCollinearContoursXld`、`UnionCocircularContoursXld` |
| 特征与筛选 | 长度、面积、重心、闭合状态和圆度等条件；`LengthXld`、`AreaCenterXld`、`TestClosedXld`、`SelectContoursXld`、`SelectShapeXld` |
| 拟合与空间关系 | 直线/圆/椭圆拟合，点到轮廓距离、轮廓间距离与交点；`FitLineContourXld`、`FitCircleContourXld`、`FitEllipseContourXld`、`DistancePc`、`DistanceCc`、`IntersectionContoursXld` |

圆形轮廓可通过 `SelectShapeXld` 的 `circularity` 特征筛选，也可组合面积、宽高比等条件；闭合性筛选使用 `SelectContoursXld` 的 `closed` 条件。图片 demo 先筛选闭合或近闭合轮廓，再保留圆度为 0.9–1.0 的圆孔边缘。圆度用于形状筛选，几何参数可进一步通过轮廓拟合获得。

## 模板匹配

模板匹配用于从搜索图像中定位已知目标，输出行列位置、角度及匹配分数。Shape 侧重轮廓结构，NCC 侧重模板内的灰度相关。

| 能力 | Shape | NCC |
|---|---|---|
| 模板来源 | 图像或 XLD 轮廓 | 灰度图像及其定义域 |
| 搜索变化 | 平移、旋转、等比例缩放 | 平移、旋转 |
| 多目标 | 多实例、多模板搜索 | 多实例、多模板搜索 |
| 结果与控制 | 亚像素定位、分数阈值、重叠控制、极性控制 | 亚像素定位、分数阈值、重叠控制、极性控制 |
| 模型管理 | 创建、参数查询、轮廓查询、读写及序列化 | 创建、参数查询、读写及序列化 |

常用流程为限定模板域 → 创建模型 → 设置搜索角度/尺度及分数阈值 → 查找模型 → 根据返回位姿变换模板轮廓或业务 ROI。模型可通过控制 Tuple 使用，也可由 `MvShapeModel`、`MvNCCModel` 类管理。

金字塔层数、搜索范围、模板纹理/轮廓质量和目标大小共同影响召回、精度及耗时。模型使用 mvkit 自有格式；同一实例的清理与并发修改需要调用方同步。

## 标定

标定模块组织相机内参、畸变参数和位姿，提供像素坐标、相机坐标及世界坐标之间的几何转换。

| 能力 | 说明与代表接口 |
|---|---|
| 参数与位姿 | 相机参数及 Pose 读写，位姿与齐次矩阵转换、组合和求逆；`ReadCamPar`、`WriteCamPar`、`ReadPose`、`PoseToHomMat3d`、`HomMat3dInvert` |
| 相机标定 | 标定数据模型、标定对象、观测点/轮廓管理、标定板检测及相机参数优化；`CreateCalibData`、`FindCalibObject`、`SetCalibDataObservPoints`、`CalibrateCameras` |
| 几何估计 | 点对应的二维仿射/投影变换估计及 PnP 位姿估计；`VectorToHomMat2d`、`VectorToProjHomMat2d`、`VectorToPose` |
| 畸变与投影 | 畸变映射、图像/点/轮廓去畸变、三维点投影；`GenRadialDistortionMap`、`ChangeRadialDistortionImage`、`ChangeRadialDistortionContoursXld`、`Project3dPoint` |
| 世界平面映射 | 图像、像素点及轮廓映射到指定世界平面；`GenImageToWorldPlaneMap`、`ImageToWorldPlane`、`ImagePointsToWorldPlane`、`ContourToWorldPlaneXld` |
| 手眼标定 | 通过标定数据模型组织机器人与相机观测并求解；`SetCalibData`、`CalibrateHandEye`、`GetCalibData` |

使用时先配置相机模型和标定对象，再添加观测、求解参数，并将参数用于畸变校正、投影或世界平面映射。所支持的相机模型、坐标约定及参数组合见 [mv_calibration.hpp](../include/mvkit/cpp/mv_calibration.hpp) 和 [mv_geometry.hpp](../include/mvkit/cpp/mv_geometry.hpp)。

标定描述文件按传入路径读取；相对路径基于进程当前工作目录解析。

## 三维

三维链路覆盖双目重建、结构光重建及点云后处理，使用 `MvCameraSetupModel`、`MvStereoModel`、`MvStructuredLightModel` 和 `MvObjectModel3D` 组织参数及结果。

| 能力 | 说明与代表接口 |
|---|---|
| 双目校正与重建 | 相机 setup、校正映射、SAD/Census 视差及 SGM、视差转 XYZ/点云；`GenBinocularRectificationMap`、`BinocularDisparity`、`BinocularDisparityMs`、`DisparityImageToXyz`、`ReconstructPointsStereo` |
| 结构光 | Gray Code、相移及单条纹的图案生成、解码和表面重建；`CreateStructuredLightModel`、`GenStructuredLightPattern`、`DecodeStructuredLightPattern`、`ReconstructSurfaceStructuredLight` |
| 点云与坐标变换 | 从点或 XYZ 图像构建对象，模型读写、刚体/仿射变换和投影；`GenObjectModel3dFromPoints`、`XyzToObjectModel3d`、`ObjectModel3dToXyz`、`RigidTransObjectModel3d` |
| 筛选与几何处理 | 属性筛选、连通分组、平面求交、采样、法向估计及邻域查询；`SelectPointsObjectModel3d`、`ConnectionObjectModel3d`、`IntersectPlaneObjectModel3d`、`SampleObjectModel3d`、`SurfaceNormalsObjectModel3d` |
| 拟合、配准与网格 | 几何基元拟合、点云距离、两模型配准和三角化；`FitPrimitivesObjectModel3d`、`DistanceObjectModel3d`、`RegisterObjectModel3dPair`、`TriangulateObjectModel3d` |

双目常用流程为相机参数与相对位姿 → 校正 → 视差 → XYZ/点云；结构光常用流程为配置模型 → 生成图案 → 解码采集图像 → 重建表面。重建后的点云可继续筛选、采样、估计法向、配准并生成网格。
