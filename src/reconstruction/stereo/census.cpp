#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <imageproc/pixel_dispatch.hpp>
#include <memory_resource>
#include <reconstruction/stereo/census.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <utility>

#include "census_simd.hpp"
#if !defined(_M_ARM64EC) && (defined(_M_X64) || defined(__SSE2__))
#include <emmintrin.h>
#define MVKIT_CENSUS_SSE2 1
#else
#define MVKIT_CENSUS_SSE2 0
#endif

#include <core/exception.hpp>

namespace mv::reconstruction {

CensusImage::CensusImage(std::size_t image_width, std::size_t image_height,
                         std::pmr::memory_resource* memory)
    : width(image_width),
      height(image_height),
      bits(image_width * image_height, std::uint64_t{0}, memory),
      valid(image_width * image_height, std::uint8_t{0}, memory) {}

std::pmr::vector<float> image_to_scalar(const core::Image& image,
                                        std::pmr::memory_resource* memory) {
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "stereo", "image must not be empty");
  }
  const auto width = image.width(), height = image.height(), stride = image.stride();
  const auto* data = static_cast<const std::byte*>(image.data());
  std::pmr::vector<float> output(width * height, 0.0F, memory);
  imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    for (std::size_t row = 0; row < height; ++row) {
      const auto* source = reinterpret_cast<const T*>(data + row * stride);
      for (std::size_t col = 0; col < width; ++col)
        output[row * width + col] = static_cast<float>(source[col]);
    }
  });
  return output;
}

namespace {

template <class OffsetRange>
CensusImage census_impl(const core::Image& image, const OffsetRange& offsets,
                        std::pmr::memory_resource* memory) {
  // 比较前保留 Float32 转换，包含 Int32 舍入及 NaN。
  const auto scalar = image_to_scalar(image, memory);
  CensusImage output(image.width(), image.height(), memory);
  const auto width = image.width(), height = image.height();
  int radius_row = 0, radius_col = 0;
  std::array<std::ptrdiff_t, 64> linear{};
  std::size_t offset_count = 0;
  for (const auto [dr, dc] : offsets) {
    radius_row = std::max(radius_row, std::abs(dr));
    radius_col = std::max(radius_col, std::abs(dc));
    linear[offset_count++] = dr * static_cast<std::ptrdiff_t>(width) + dc;
  }
  bool vectorized = false;
  bool avx2 = false;
#if MVKIT_CENSUS_SSE2
  const auto& cpu = runtime::cpu_features();
  vectorized =
      runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                              {true, false, false, false});
#endif
#if MVKIT_CENSUS_SSE2 && MVKIT_RECONSTRUCTION_HAS_AVX2
  avx2 = runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                 {true, true, false, false});
#endif
  for (std::size_t row = 0; row < height; ++row) {
    // 边界样本保留原 clamp，各列共享同一行。
    std::array<std::size_t, 64> sample_base{};
    std::size_t sample_bit = 0;
    for (const auto [dr, dc] : offsets) {
      const auto sample_row = static_cast<std::size_t>(
          std::clamp(static_cast<int>(row) + dr, 0, static_cast<int>(height) - 1));
      sample_base[sample_bit++] = sample_row * width;
    }
    std::size_t column = 0;
    const bool interior_row = row >= std::size_t(radius_row) && row + radius_row < height;
    while (column < width) {
      const bool interior =
          interior_row && column >= std::size_t(radius_col) && column + radius_col < width;
#if MVKIT_CENSUS_SSE2 && MVKIT_RECONSTRUCTION_HAS_AVX2
      if (avx2 && interior && column + 7 + radius_col < width) {
        const auto index = row * width + column;
        const auto processed =
            census_avx2_row(scalar.data() + index, width - radius_col - column, linear.data(),
                            offset_count, output.bits.data() + index);
        std::fill_n(output.valid.data() + index, processed, std::uint8_t(1));
        column += processed;
        continue;
      }
#endif
#if MVKIT_CENSUS_SSE2
      if (vectorized && interior && column + 3 + radius_col < width) {
        const auto* center = scalar.data() + row * width + column;
        const auto centers = _mm_loadu_ps(center);
        auto low = _mm_setzero_si128(), high = _mm_setzero_si128();
        auto weight = _mm_set1_epi64x(1);
        for (std::size_t bit = 0; bit < offset_count; ++bit) {
          const auto mask =
              _mm_castps_si128(_mm_cmpge_ps(_mm_loadu_ps(center + linear[bit]), centers));
          low = _mm_or_si128(low, _mm_and_si128(_mm_unpacklo_epi32(mask, mask), weight));
          high = _mm_or_si128(high, _mm_and_si128(_mm_unpackhi_epi32(mask, mask), weight));
          weight = _mm_slli_epi64(weight, 1);
        }
        auto* destination = output.bits.data() + row * width + column;
        _mm_storeu_si128(reinterpret_cast<__m128i*>(destination), low);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(destination + 2), high);
        for (std::size_t k = 0; k < 4; ++k) output.valid[row * width + column + k] = 1;
        column += 4;
        continue;
      }
#endif
      std::uint64_t bits = 0;
      int bit = 0;
      const auto* center_pointer = scalar.data() + row * width + column;
      const float center = *center_pointer;
      if (interior) {
        for (std::size_t k = 0; k < offset_count; ++k)
          bits |= std::uint64_t(center_pointer[linear[k]] >= center) << k;
      } else {
        for (const auto& offset : offsets) {
          const auto sample_column = static_cast<std::size_t>(
              std::clamp(static_cast<int>(column) + offset.second, 0, static_cast<int>(width) - 1));
          if (scalar[sample_base[bit] + sample_column] >= center) {
            bits |= std::uint64_t{1} << bit;
          }
          ++bit;
        }
      }
      const auto index = row * width + column;
      output.bits[index] = bits;
      output.valid[index] = 1;
      ++column;
    }
  }
  return output;
}

}  // namespace

CensusImage census_dense(const core::Image& image, std::pmr::memory_resource* memory) {
  std::array<std::pair<int, int>, 62> offsets{};
  std::size_t index = 0;
  for (int row = -3; row <= 3; ++row) {
    for (int column = -4; column <= 4; ++column) {
      if (row != 0 || column != 0) offsets[index++] = {row, column};
    }
  }
  return census_impl(image, offsets, memory);
}

CensusImage census_sparse(const core::Image& image, std::pmr::memory_resource* memory) {
  constexpr std::array<int, 5> positions{-7, -4, 0, 4, 7};
  std::array<std::pair<int, int>, 24> offsets{};
  std::size_t index = 0;
  for (int row : positions) {
    for (int column : positions) {
      if (row != 0 || column != 0) offsets[index++] = {row, column};
    }
  }
  return census_impl(image, offsets, memory);
}

}  // namespace mv::reconstruction
