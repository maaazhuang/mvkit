#include <immintrin.h>

#include "census_simd.hpp"
namespace mv::reconstruction {
std::size_t census_avx2_row(const float* center, std::size_t available,
                            const std::ptrdiff_t* offsets, std::size_t count,
                            std::uint64_t* output) {
  std::size_t processed = 0;
  for (; available - processed >= 8; processed += 8) {
    const auto centers = _mm256_loadu_ps(center + processed);
    auto low = _mm256_setzero_si256(), high = _mm256_setzero_si256();
    auto weight = _mm256_set1_epi64x(1);
    for (std::size_t bit = 0; bit < count; ++bit) {
      const auto samples = _mm256_loadu_ps(center + processed + offsets[bit]);
      const auto mask = _mm256_castps_si256(_mm256_cmp_ps(samples, centers, _CMP_GE_OQ));
      low = _mm256_or_si256(low, _mm256_and_si256(_mm256_unpacklo_epi32(mask, mask), weight));
      high = _mm256_or_si256(high, _mm256_and_si256(_mm256_unpackhi_epi32(mask, mask), weight));
      weight = _mm256_slli_epi64(weight, 1);
    }
    // unpack 分别作用于 128 位半部：low 为像素 0/1/4/5，high 为 2/3/6/7。
    auto* p = output + processed;
    _mm_storeu_si128(reinterpret_cast<__m128i*>(p), _mm256_castsi256_si128(low));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(p + 2), _mm256_castsi256_si128(high));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(p + 4), _mm256_extracti128_si256(low, 1));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(p + 6), _mm256_extracti128_si256(high, 1));
  }
  _mm256_zeroupper();
  return processed;
}
}  // namespace mv::reconstruction
