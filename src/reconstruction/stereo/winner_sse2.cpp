#include <bit>
#include <limits>

#include "winner.hpp"
#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#include <emmintrin.h>
namespace mv::reconstruction {
namespace {
__m128i min_i32(__m128i a, __m128i b) {
  auto m = _mm_cmpgt_epi32(a, b);
  return _mm_or_si128(_mm_and_si128(m, b), _mm_andnot_si128(m, a));
}
std::int32_t reduce(__m128i x) {
  x = min_i32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(1, 0, 3, 2)));
  x = min_i32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(x);
}
}  // namespace
StereoWinner dense_winner_sse2(const std::int32_t* cost, std::size_t n) {
  if (!n) return {};
  constexpr auto limit = std::numeric_limits<std::int32_t>::max();
  auto first = _mm_set1_epi32(limit);
  auto minimum = limit;
  std::size_t i = 0;
  for (; i + 4 <= n; i += 4)
    first = min_i32(first, _mm_loadu_si128(reinterpret_cast<const __m128i*>(cost + i)));
  minimum = reduce(first);
  for (; i < n; ++i) minimum = (std::min)(minimum, cost[i]);
  auto second = _mm_set1_epi32(limit);
  auto scalar_second = limit;
  std::size_t best = n;
  i = 0;
  for (; i + 4 <= n; i += 4) {
    auto v = _mm_loadu_si128(reinterpret_cast<const __m128i*>(cost + i));
    if (best == n) {
      const auto mask = unsigned(_mm_movemask_epi8(_mm_cmpeq_epi32(v, _mm_set1_epi32(minimum))));
      if (mask) {
        const auto lane = std::countr_zero(mask) / 4;
        best = i + lane;
        const auto excluded = _mm_cmpeq_epi32(_mm_setr_epi32(0, 1, 2, 3), _mm_set1_epi32(lane));
        v = _mm_or_si128(_mm_and_si128(excluded, _mm_set1_epi32(limit)),
                         _mm_andnot_si128(excluded, v));
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
  return {best, minimum, best == 0 ? (std::min)(kStereoBadCost, other) : other};
}
}  // namespace mv::reconstruction
#else
namespace mv::reconstruction {
StereoWinner dense_winner_sse2(const std::int32_t* cost, std::size_t n) {
  return ordered_winner([&](std::size_t i) { return cost[i]; }, n);
}
}  // namespace mv::reconstruction
#endif
