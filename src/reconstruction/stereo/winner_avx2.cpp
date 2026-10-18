#include <immintrin.h>

#include <bit>
#include <limits>

#include "winner.hpp"
namespace mv::reconstruction {
namespace {
__m256i min_i32(__m256i a, __m256i b) { return _mm256_min_epi32(a, b); }
std::int32_t reduce(__m256i v) {
  auto x = _mm_min_epi32(_mm256_castsi256_si128(v), _mm256_extracti128_si256(v, 1));
  x = _mm_min_epi32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(1, 0, 3, 2)));
  x = _mm_min_epi32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(x);
}
}  // namespace
StereoWinner dense_winner_avx2(const std::int32_t* cost, std::size_t n) {
  if (!n) return {};
  constexpr auto limit = std::numeric_limits<std::int32_t>::max();
  auto first = _mm256_set1_epi32(limit);
  auto minimum = limit;
  std::size_t i = 0;
  for (; i + 8 <= n; i += 8)
    first = min_i32(first, _mm256_loadu_si256(reinterpret_cast<const __m256i*>(cost + i)));
  minimum = reduce(first);
  for (; i < n; ++i) minimum = (std::min)(minimum, cost[i]);
  auto second = _mm256_set1_epi32(limit);
  auto scalar_second = limit;
  std::size_t best = n;
  i = 0;
  for (; i + 8 <= n; i += 8) {
    auto v = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(cost + i));
    if (best == n) {
      const auto mask =
          unsigned(_mm256_movemask_epi8(_mm256_cmpeq_epi32(v, _mm256_set1_epi32(minimum))));
      if (mask) {
        const auto lane = std::countr_zero(mask) / 4;
        best = i + lane;
        const auto excluded =
            _mm256_cmpeq_epi32(_mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7), _mm256_set1_epi32(lane));
        v = _mm256_or_si256(_mm256_and_si256(excluded, _mm256_set1_epi32(limit)),
                            _mm256_andnot_si256(excluded, v));
      }
    }
    second = min_i32(second, v);
  }
  for (; i < n; ++i) {
    if (best == n && cost[i] == minimum)
      best = i;
    else
      scalar_second = (std::min)(scalar_second, cost[i]);
  }
  auto other = (std::min)(scalar_second, reduce(second));
  // 原 kBadCost 种子仅在标签零仍为 winner 时保留。
  _mm256_zeroupper();
  return {best, minimum, best == 0 ? (std::min)(kStereoBadCost, other) : other};
}
}  // namespace mv::reconstruction