#pragma once
#include <core/image.hpp>
#include <span>
#include <runtime/cpu/cpu_features.hpp>
namespace mv::imageproc {
inline constexpr mv::runtime::KernelRequirement kSobel3SumAbsU8Avx2Requirement{false,true,false,false};

std::size_t sobel3_sumabs_u8_row_avx2(const std::uint8_t*,const std::uint8_t*,const std::uint8_t*,
                                     std::uint8_t*,std::size_t,std::size_t);
void convolve_double_row_avx2(const double*,std::span<const double>,double*,std::size_t);
void convolve_float_rows_avx2(std::span<const double* const>,std::span<const double>,float*,std::size_t);
void convolve_float_scaled_pair_rows_avx2(
    std::span<const double* const>,std::span<const double>,
    std::span<const double* const>,std::span<const double>,
    double*,double*,std::size_t,double);
void convolve_integer_to_float_avx2(const core::Image&,core::Image&,std::size_t,std::span<const double>,std::span<const double>);
void box_columns_avx2(std::uint32_t*,const void*,const void*,std::size_t,core::PixelType);
void box_row_avx2(const std::uint32_t*,const std::size_t*,const std::size_t*,void*,std::size_t,
                  std::size_t,std::uint32_t,std::uint32_t,core::PixelType);
void morphology_u8_avx2(const core::Image&, core::Image&, std::size_t,
                        std::size_t, std::size_t, bool);
void separable_filter_avx2(const core::Image &, core::Image &, std::size_t,
                           std::span<const double>, std::span<const double>,
                           bool box5);
std::size_t median3_row_avx2(const void *, const void *, const void *, void *,
                             std::size_t, std::size_t, core::PixelType);
} // namespace mv::imageproc

namespace mv::imageproc {
void convolve_double_pair_row_avx2(const double*,std::span<const double>,std::span<const double>,double*,double*,std::size_t);
}

namespace mv::imageproc {
std::size_t log_double_row_avx2(const double*,std::span<const std::ptrdiff_t>,std::span<const int>,std::int16_t*,std::size_t,double);
void log_byte_image_avx2(const core::Image&,core::Image&,std::size_t,std::size_t,std::span<const std::ptrdiff_t>,std::span<const int>,double);
}
