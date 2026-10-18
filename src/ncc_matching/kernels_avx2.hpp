#pragma once
#include "model.hpp"
namespace mv::ncc_matching {
// 标量矩采用与各 SIMD 位置 lane 相同的有序游程。
double correlation_one_prefix_avx2(const double*,const double*,const double*,std::size_t,const Patch&,int,int,bool,bool exact_byte_lattice=false,double* cached_sum=nullptr,double* cached_squared=nullptr);
std::size_t gray_u16_avx2(const std::uint16_t*,double*,std::size_t,double);
std::size_t gray_i16_avx2(const std::int16_t*,double*,std::size_t,double);
std::size_t gray_i32_avx2(const std::int32_t*,double*,std::size_t,double);
std::size_t gray_f32_avx2(const float*,double*,std::size_t,double);

std::size_t gray_u8_avx2(const std::uint8_t*,double*,std::size_t,double);
std::size_t box2_u8_avx2(const std::uint8_t*,const std::uint8_t*,double*,std::size_t,double);
std::size_t box2_double_avx2(const double*,const double*,double*,std::size_t);
void correlation_four_avx2(const double*,std::size_t,const Patch&,int,int,bool,double*);
// exact_byte_lattice 要求 UInt8 查询值减整数 DC、金字塔层<=2、精确行前缀（宽<=32,000,000）。
// 1,048,576 样本守卫内，低方差晶格窗口为常量。可选矩缓存对应此行列起点的 4/8/16 连续窗口。
// 调用方证明不可变像素、有序游程、样本数及浮点环境相同；任一四 lane 缓存含 NaN 时按原序重算，
// 非 NaN 矩可独立于模板权重复用。
void correlation_four_prefix_avx2(const double*,const double*,const double*,std::size_t,const Patch&,int,int,bool,double*,bool exact_byte_lattice=false,double* cached_sum=nullptr,double* cached_squared=nullptr);
void correlation_sixteen_prefix_avx2(const double*,const double*,const double*,std::size_t,const Patch&,int,int,bool,double*,bool exact_byte_lattice=false,double* cached_sum=nullptr,double* cached_squared=nullptr);
void correlation_eight_prefix_avx2(const double*,const double*,const double*,std::size_t,const Patch&,int,int,bool,double*,bool exact_byte_lattice=false,double* cached_sum=nullptr,double* cached_squared=nullptr);
double correlation_avx2(const double*,std::size_t,const Patch&,int,int,bool);
struct BilinearMoments {
  double sum=0,squared=0,dot=0,gs[3]{},qg[3]{},wg[3]{},gg[3][3]{};
};
struct BilinearScoreGate {
  double minimum;bool absolute;bool skipped=false;
};
// 可选输出为 q[N] 及按轴排列的 gradients[3*N]；仅平移时第三轴梯度为零，只评分不写梯度。
// 重放仅改变模板加权 dot/wg，复用工作区占 4*N 个 Double。内部导数样本<=32768 时分阶段归约；
// 已有 q/gradient 输出即可作工作区，无需另分配/复制。完整评分不能优于 minimum 时，可仅跳过导数统计；
// skipped=true 时仅 sum/squared/dot 有效。可选零块证明借用不可变存储图像及 Patch，
// 仅在 interior、首样本偏移有限、普通最近舍入下跳过完整正零块。非零偏移仍保持原序 q 矩及样本输出，
// 尾部照常采样。空间证明要求调用方 tile_flags scratch；最后位姿缓存只在位姿逐位相同、
// 借用 Image/Patch/flag 身份相同时复用证明。scratch 不足时保留块路径。
struct BilinearZeroView;
bool bilinear_moments_avx2(const double*,std::size_t,std::size_t,const Level&,const Patch&,const Match&,bool,bool,BilinearMoments&,double* centered_samples=nullptr,double* gradient_samples=nullptr,double* derivative_workspace=nullptr,BilinearScoreGate* score_gate=nullptr,const BilinearZeroView* zero_blocks=nullptr);
void weighted_moments_avx2(const double* centered_samples,const double* gradient_samples,const Patch&,bool rotate,BilinearMoments&);
struct RefinementGradientGroup;
void weighted_moments_compressed_avx2(const double* centered_samples,const RefinementGradientGroup*,std::size_t group_count,const Patch&,bool rotate,BilinearMoments&);
double centered_dot_avx2(const double*,const Patch&);
}
