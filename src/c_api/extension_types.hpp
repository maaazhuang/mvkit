#pragma once
// 私有算法适配层共用类型，不安装到 SDK。
#include <stdint.h>
#include <stddef.h>

/** 只有对应标志置位时字段才有意义。RMS 在标定中以像素为单位，在 ICP 中以模型单位表示。
 * 收敛不代表可观测；不可用的浮点值为 NaN，不生成虚构协方差。 */
#define MV_QUALITY_RESIDUAL ((uint32_t)1)
#define MV_QUALITY_COUNTS ((uint32_t)2)
#define MV_QUALITY_CONVERGENCE ((uint32_t)4)
#define MV_QUALITY_REASON_OK ((int32_t)0)
#define MV_QUALITY_REASON_NO_RESULT ((int32_t)1)
#define MV_QUALITY_REASON_NOT_CONVERGED ((int32_t)2)
typedef struct MvResultQuality {
  uint32_t fields;
  int32_t reason;
  double rms;
  uint64_t sample_count, inlier_count, iterations;
  uint8_t converged, uncertainty_available, degeneracy_known, degenerate;
} MvResultQuality;


/** 单应/PnP 扩展的共用配置。robust=0 使用加权拟合；robust=1 使用带种子的自适应 RANSAC 后加权细化。
 * threshold 为目标坐标的欧氏距离（单应坐标或 PnP 像素）；confidence 是采样目标，不是置信证明。 */
typedef struct MvGeometryEstimationOptions {
  int32_t robust;
  int64_t max_trials, max_iterations;
  uint64_t seed;
  double threshold, confidence, min_inlier_ratio;
} MvGeometryEstimationOptions;
typedef struct MvGeometryEstimationDiagnostics {
  double all_rms, weighted_inlier_rms;
  uint64_t trials;
  int32_t geometry_rank; /**< 2 表示平面/单应，3 表示空间 PnP。 */
} MvGeometryEstimationDiagnostics;


/** 单模型求解器的强类型配置；默认值与既有 lm 求解器一致。 */
typedef struct MvCalibrationOptions {
  int64_t max_iterations;
  double function_tolerance, gradient_tolerance, parameter_tolerance, initial_damping;
} MvCalibrationOptions;


/** 可组合的重建失败标志。confidence 是评分，不是概率；depth_uncertainty 为源相机 Z 的标准差，
 * 使用模型单位。NaN 表示未知，包括补洞点。 */
typedef enum MvReconstructionQualityReason {
  MV_RECON_QUALITY_OUTSIDE_DOMAIN = 1,
  MV_RECON_QUALITY_NO_CORRESPONDENCE = 2,
  MV_RECON_QUALITY_INCONSISTENT = 4,
  MV_RECON_QUALITY_LOW_CONTRAST = 8,
  MV_RECON_QUALITY_WEAK_PHASE = 16,
  MV_RECON_QUALITY_GEOMETRY_FAILURE = 32,
  MV_RECON_QUALITY_AMBIGUOUS = 64,
  MV_RECON_QUALITY_COORDINATE_OUT_OF_RANGE = 128,
  MV_RECON_QUALITY_SATURATED = 256
} MvReconstructionQualityReason;


/** 只读借用存储有效至模型最后引用释放，支持并发读取。
 * x/y/z 为偏移 0/8/16 处的 double，记录步长 point_stride 字节，须用 memcpy 读取。
 * valid=NULL 表示全部有效，否则按 count 个字节保留有组织槽位。 */
typedef struct MvPointCloudView {
  const void* points;
  const uint8_t* valid;
  size_t count, point_stride, width, height;
} MvPointCloudView;


/** SDK 点云导入/导出的点存储类型。 */
typedef struct MvPoint3d {
  double x;
  double y;
  double z;
} MvPoint3d;
