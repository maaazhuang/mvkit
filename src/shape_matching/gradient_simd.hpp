#pragma once

#include <cstddef>
#include <cstdint>
#include "discrete_shape_point.hpp"

namespace mv::shape_matching::simd {

[[nodiscard]] bool cpu_has_avx2() noexcept;
[[nodiscard]] const char* selected_kernel_name() noexcept;

struct KernelTableEntry {
  const char* name;
  // 已编译加速器或 "none"；仅表示可用快路径，输入前提及尾部仍可能执行基线。
  const char* compiled_isa;
  const char* selected_isa;
  const char* baseline_isa;
};

[[nodiscard]] std::size_t kernel_table(KernelTableEntry* out,
                                       std::size_t capacity) noexcept;

void avx2_zeroupper() noexcept;

// 评分八个相邻中心，整数累加及逐 lane 拒绝规则同 SSE2 四中心核。
void score_i8_group8_avx2(const std::int8_t* qnx, const std::int8_t* qny,
                         const DiscreteShapePoint* points, std::size_t count,
                         std::size_t center, int live_mask, bool ignore_local,
                         bool ignore_global, double greediness,
                         double reject_below, double* scores,
                         const int* remaining_bounds = nullptr,
                         const PackedI8ShapePoint* packed_points = nullptr);

// 可选搜索局部调度，供等点数离散位姿共享。
void prepare_i8_remaining_bounds(int* bounds, std::size_t count,
                                 double greediness, double reject_below);

// 按查询像素组构造精确方向位图；调用方检查 AVX2、提供清零 Map，并使用非负证据阈值。
void build_direction_maps_avx2(const std::uint32_t* normals, std::size_t normal_count,
    const std::uint16_t* keys, std::size_t key_count, const std::size_t* starts,
    const std::size_t* positions, int threshold, bool polarity, std::size_t words,
    std::uint64_t* maps, void (*poll)(const void*) = nullptr, const void* context = nullptr);

// 不可变查询层评分常量可在等点数位姿间复用；外部 setup 须匹配 count/greediness/reject_below。
struct I8ScoreSetup {
  int threshold;
  std::size_t unchecked_prefix;
  std::size_t prefix;
  double inverse;
};
I8ScoreSetup prepare_i8_score_setup(std::size_t count, double greediness, double reject_below);

// 内部游程含 0..64 相邻中心；梯度平面或交错点对须允许不足一组时额外读七像素，可由调用方补零。
// 精确写 centers 个评分，拒绝 lane 也写零。前四点为锚点；可选 exact_evidence
// 须逐 lane 等于原锚点谓词，只改变枚举，不改变评分/拒绝序。
std::size_t scan_i8_run_avx2(const std::int8_t* qnx, const std::int8_t* qny,
                            const PackedI8ShapePoint* points, std::size_t count,
                            std::size_t center, std::size_t centers,
                            int min_hits, int evidence_threshold, bool use_polarity,
                            bool ignore_local, bool ignore_global, double greediness,
                            double reject_below, const int* remaining_bounds,
                            double* scores, std::uint64_t* accepted_mask = nullptr,
                            const std::int8_t* query_pairs = nullptr,
                            const std::uint64_t* exact_evidence = nullptr,
                            const I8ScoreSetup* score_setup = nullptr);

// 用 AVX2 十六宽 Sobel 填内部列 gx[c]/gy[c]，返回首个未处理列供调用方完成尾部。
std::size_t sobel_u8_i16_row_avx2(const std::uint8_t* __restrict top,
                                  const std::uint8_t* __restrict middle,
                                  const std::uint8_t* __restrict bottom,
                                  std::int16_t* __restrict gx,
                                  std::int16_t* __restrict gy,
                                  std::size_t width);

std::size_t unit_sobel_f32_row_avx2(const float* __restrict top,
                                    const float* __restrict middle,
                                    const float* __restrict bottom,
                                    float* __restrict nx,
                                    float* __restrict ny,
                                    std::size_t width, float min_squared);

// 相邻原点的 int8 单位法向点积证据。threshold 为 127² 定标整数。
int i8_group_evidence_mask_avx2(const std::int8_t* __restrict qnx,
                                const std::int8_t* __restrict qny,
                                const std::ptrdiff_t* __restrict offsets,
                                const std::int8_t* __restrict normal_row,
                                const std::int8_t* __restrict normal_col,
                                std::size_t anchor_count,
                                std::size_t first_center, int min_hits,
                                int threshold, bool use_polarity,
                                int lanes) noexcept;

// 半分辨率 2-of-4 预筛整幅扫描。hits 为 (sr<<16)|sc。evaluated 计入访问格数。
std::size_t i8_collect_screen_hits_avx2(
    const std::int8_t* __restrict qnx, const std::int8_t* __restrict qny,
    const std::ptrdiff_t* __restrict offsets,
    const std::int8_t* __restrict normal_row,
    const std::int8_t* __restrict normal_col, std::size_t anchor_count,
    std::size_t screen_w, std::size_t screen_h, std::size_t unit_w,
    std::size_t unit_h, double min_row, double max_row, double min_col,
    double max_col, int min_hits, int threshold, bool use_polarity,
    std::uint32_t* __restrict hits, std::size_t hits_capacity,
    std::size_t* __restrict evaluated) noexcept;

// 无变形、已判定 interior；metric=0/1/2。row/col 为当前层坐标。
double score_unit_use_polarity_avx2(
    const float* __restrict nx, const float* __restrict ny, std::size_t width,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept;

double score_i16_nearest_use_polarity_avx2(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept;

// 不可变完整查询梯度的精确模倒数，缓存评分仍采用 raw_dot*inverse 及原 mag2 守卫。
void prepare_i16_inverse_magnitude_avx2(const std::int16_t* gx,
    const std::int16_t* gy, float integer_scale, std::size_t count, float* inverse) noexcept;
void prepare_i16_packed_components_avx2(const std::int16_t* gx,
    const std::int16_t* gy, std::size_t count, std::uint32_t* packed) noexcept;
double score_i16_nearest_cached_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept;
double score_i16_nearest_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    const std::uint32_t* packed, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept;

// 预计算 scale*R(angle)*点 与旋转法向，供同一姿态的多次平移打分复用。
void rotate_model_points_avx2(const float* __restrict prow,
                              const float* __restrict pcol,
                              const float* __restrict pnr,
                              const float* __restrict pnc, std::size_t n,
                              float ca, float sa, float scale,
                              float* __restrict off_r, float* __restrict off_c,
                              float* __restrict nr, float* __restrict nc) noexcept;

double score_unit_pre_rotated_use_polarity_avx2(
    const float* __restrict nx, const float* __restrict ny, std::size_t width,
    const float* __restrict off_r, const float* __restrict off_c,
    const float* __restrict nr, const float* __restrict nc, std::size_t n,
    float row, float col, float greediness, float reject_below, int metric) noexcept;

double score_i16_pre_rotated_use_polarity_avx2(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict off_r, const float* __restrict off_c,
    const float* __restrict nr, const float* __restrict nc, std::size_t n,
    float row, float col, float greediness, float reject_below, int metric) noexcept;
double score_i16_pre_rotated_cached_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept;
double score_i16_pre_rotated_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    const std::uint32_t* packed, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept;

// 与 score_pose_level(..., bilinear=true) 同一套四点插值，供 finest 终分。
double score_i16_bilinear_use_polarity_avx2(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept;
double score_i16_bilinear_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept;

// 沿法向 0.25 px 采样几何响应，responses 长度为 2*span+1，下标 0 对应 -span。
void geometric_i16_bilinear_line_avx2(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_contrast_squared, float* __restrict responses, bool ignore_polarity = false) noexcept;
void geometric_i16_bilinear_packed_line_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_contrast_squared, float* responses, bool ignore_polarity) noexcept;

struct GeometricPeak {
  int index{};  // 带符号法向搜索偏移，与原扫描器一致。
  double left{}, center{}, right{};
};
// 读取不变的 2*span+1 响应，保持原序首个正最大值、邻居及无 winner 状态；
// 17 项使用 AVX2 选择，其他跨度保留标量扫描。
GeometricPeak geometric_response_peak_avx2(const float* responses, int span) noexcept;
// 同一 AVX 调用组合响应核及峰选择，固定 span=8。
GeometricPeak geometric_i16_bilinear_peak17_avx2(
    const std::int16_t* gx, const std::int16_t* gy, std::size_t width,
    float integer_scale, float expected_row, float expected_col,
    float normal_row, float normal_col, float search_step,
    float min_contrast_squared, bool ignore_polarity) noexcept;
GeometricPeak geometric_i16_bilinear_packed_peak17_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    float min_contrast_squared, bool ignore_polarity) noexcept;

// 八条独立内部像素格法向 profile，由调用方提供 padding。
void xld_geometric_i16_peaks_avx2(const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, const float* row, const float* col,
    const float* nr, const float* nc, const float* base, const float* step,
    const int* span, int maximum_span, float min_squared, bool ignore_polarity,
    float* left, float* center, float* right, int* best_index) noexcept;

std::size_t sobel_f32_i16_row_avx2(const float* __restrict top,
                                   const float* __restrict middle,
                                   const float* __restrict bottom,
                                   std::int16_t* __restrict gx,
                                   std::int16_t* __restrict gy,
                                   std::size_t width, std::size_t col_begin,
                                   std::size_t col_end);

// 强类型不可变绑定，不可用/禁用加速器为 nullptr。
struct Avx2Kernels {
  decltype(&score_i8_group8_avx2) score_i8_group8{};
  decltype(&sobel_u8_i16_row_avx2) sobel_u8_i16_row{};
  decltype(&unit_sobel_f32_row_avx2) unit_sobel_f32_row{};
  decltype(&i8_group_evidence_mask_avx2) i8_group_evidence_mask{};
  decltype(&score_unit_use_polarity_avx2) score_unit_use_polarity{};
  decltype(&score_i16_nearest_use_polarity_avx2) score_i16_nearest_use_polarity{};
  decltype(&prepare_i16_inverse_magnitude_avx2) prepare_i16_inverse_magnitude{};
  decltype(&prepare_i16_packed_components_avx2) prepare_i16_packed_components{};
  decltype(&score_i16_nearest_cached_use_polarity_avx2) score_i16_nearest_cached_use_polarity{};
  decltype(&score_i16_nearest_packed_use_polarity_avx2) score_i16_nearest_packed_use_polarity{};
  decltype(&rotate_model_points_avx2) rotate_model_points{};
  decltype(&score_unit_pre_rotated_use_polarity_avx2) score_unit_pre_rotated_use_polarity{};
  decltype(&score_i16_pre_rotated_use_polarity_avx2) score_i16_pre_rotated_use_polarity{};
  decltype(&score_i16_pre_rotated_cached_use_polarity_avx2) score_i16_pre_rotated_cached_use_polarity{};
  decltype(&score_i16_pre_rotated_packed_use_polarity_avx2) score_i16_pre_rotated_packed_use_polarity{};
  decltype(&score_i16_bilinear_use_polarity_avx2) score_i16_bilinear_use_polarity{};
  decltype(&score_i16_bilinear_packed_use_polarity_avx2) score_i16_bilinear_packed_use_polarity{};
  decltype(&geometric_i16_bilinear_line_avx2) geometric_i16_bilinear_line{};
  decltype(&geometric_i16_bilinear_packed_line_avx2) geometric_i16_bilinear_packed_line{};
  decltype(&geometric_response_peak_avx2) geometric_response_peak{};
  decltype(&geometric_i16_bilinear_peak17_avx2) geometric_i16_bilinear_peak17{};
  decltype(&geometric_i16_bilinear_packed_peak17_avx2) geometric_i16_bilinear_packed_peak17{};
  decltype(&xld_geometric_i16_peaks_avx2) xld_geometric_i16_peaks{};
  decltype(&sobel_f32_i16_row_avx2) sobel_f32_i16_row{};
};
[[nodiscard]] const Avx2Kernels& avx2_kernels() noexcept;

}  // namespace mv::shape_matching::simd
