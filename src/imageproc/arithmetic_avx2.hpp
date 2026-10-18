#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <array>
namespace mv::core { class Region; struct RegionRun; }
namespace mv::imageproc {
std::size_t emphasize_u8_row_avx2(const std::uint8_t*,const float*,std::uint8_t*,std::size_t,double);
void box_mean_columns_row_avx2(const std::uint32_t*,const std::size_t*,const std::size_t*,
    float*,std::size_t,std::size_t,std::uint32_t,std::uint32_t,bool);
// 八个 byte RGB 像素使用精确整数分子和 Float32 除法，保持原 Float32 输出。
std::size_t rgb_to_hsv_u8_row_avx2(const std::uint8_t*, const std::uint8_t*,
    const std::uint8_t*, float*, float*, float*, std::size_t, std::size_t);
std::size_t transform_byte_planes_row_avx2(const std::uint8_t*,const std::uint8_t*,const std::uint8_t*,
    std::uint8_t*,std::uint8_t*,std::uint8_t*,std::size_t,std::size_t,bool);
std::size_t split_byte_interleaved_row_avx2(const std::uint8_t*,std::uint8_t*,std::uint8_t*,
    std::uint8_t*,std::size_t,std::size_t,bool);
// 精确 UInt8 查表；调用方提供 256 项 Int32 表并处理不足八像素的尾部。
std::size_t lookup_u8_row_avx2(const std::uint8_t*, std::uint8_t*,
                              std::size_t, std::size_t, const std::int32_t*);
// 调用方穷举 byte 输入验证后，使用饱和 UInt16 乘加及八位右移替代查表。
struct CertifiedByteAffine {
    std::uint16_t factor{}, offset{}, limit{};
    bool subtract{}, invert{}, valid{};
};
std::size_t certified_affine_u8_row_avx2(const std::uint8_t*, std::uint8_t*,
    std::size_t, std::size_t, const CertifiedByteAffine&);
struct ZoomNearestU8Block {
    std::uint32_t offset;
    std::array<std::uint8_t,16> low, high;
};
std::size_t zoom_nearest_u8_row_avx2(const std::uint8_t*,std::uint8_t*,
    std::span<const ZoomNearestU8Block>);
std::uint32_t local_threshold_u8_window_mask_avx2(const std::uint8_t*,const std::uint64_t*,
    const std::uint64_t*,std::size_t,std::size_t,std::uint64_t*,std::uint64_t*,double,double,double,bool);
std::uint32_t local_threshold_u8_mask_avx2(const std::uint8_t*,const double*,const double*,
    double,double,double,bool);
std::size_t zoom_bilinear_u8_row_avx2(const std::uint8_t*,const std::uint8_t*,
    std::uint8_t*,std::span<const std::uint32_t>,std::span<const double>,std::size_t,double);
std::size_t projective_u8_row_avx2(const std::uint8_t*,std::uint8_t*,std::uint8_t*,
    std::size_t,std::size_t,std::size_t,const double*,std::size_t,std::size_t,bool);
std::size_t remap_float_u8_row_avx2(const std::uint8_t*,std::uint8_t*,std::uint8_t*,
    std::size_t,std::size_t,std::size_t,const float*,const float*,std::size_t,bool&);
std::size_t remap_u8_row_avx2(const void*,std::uint8_t*,std::size_t,std::size_t,std::size_t,const double*,std::size_t,std::size_t,std::size_t,bool);
bool shifted_sum_avx2(const void*,std::size_t,std::span<const core::RegionRun>,bool,double,double,bool,double*);
int f32_block_moments_avx2(const void*,std::size_t,std::span<const core::RegionRun>,double,double*,double*,std::uint64_t*);
void u16_moments_avx2(const void*,std::size_t,std::span<const core::RegionRun>,std::int64_t*,std::uint64_t*);
void min_max_gray_avx2(const void*,std::size_t,const mv::core::Region&,int,double*,double*);
std::size_t divide_u8_row_avx2(const std::uint8_t*,const std::uint8_t*,std::uint8_t*,
                              std::size_t,std::size_t,double,double);
std::size_t multiply_u16_row_avx2(const std::uint16_t*,const std::uint16_t*,std::uint16_t*,
                                 std::size_t,std::size_t,double,double);
void threshold_masks_avx2(const void*,std::size_t,std::size_t,double,double,int,std::uint32_t*);
std::size_t binary_f32_row_avx2(const float*,const float*,float*,std::size_t,std::size_t,double,double,int);
std::size_t binary_u16_half_row_avx2(const std::uint16_t*,const std::uint16_t*,std::uint16_t*,
                                    std::size_t,std::size_t,int,int,bool);
std::size_t multiply_u8_row_avx2(const std::uint8_t*,const std::uint8_t*,std::uint8_t*,
                               std::size_t,std::size_t,double,double);
std::size_t rgb_u8_row_avx2(const std::uint8_t*,const std::uint8_t*,const std::uint8_t*,
                          std::uint8_t*,std::size_t,std::size_t);
void dynamic_u8_masks_avx2(const std::uint8_t*,const std::uint8_t*,std::size_t,
                          std::size_t,int,int,bool,std::uint32_t*);
std::size_t divide_u16_row_avx2(const std::uint16_t*, const std::uint16_t*,
                              std::uint16_t*, std::size_t, std::size_t, double, double);
}
