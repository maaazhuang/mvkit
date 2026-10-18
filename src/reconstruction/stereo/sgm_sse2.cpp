#include <algorithm>
#include <reconstruction/stereo/cost_semantics.hpp>

#include "sgm_simd.hpp"

#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#include <emmintrin.h>
// 公开惩罚验证保证每次路径更新不溢出 Int32；视差端点保留 infinity，向量仅处理内部标签。
namespace mv::reconstruction {
namespace {
__m128i min_i32(__m128i a, __m128i b) {
  const auto mask = _mm_cmpgt_epi32(a, b);
  return _mm_or_si128(_mm_and_si128(mask, b), _mm_andnot_si128(mask, a));
}
std::int32_t reduce_min(__m128i v) {
  v = min_i32(v, _mm_shuffle_epi32(v, _MM_SHUFFLE(1, 0, 3, 2)));
  v = min_i32(v, _mm_shuffle_epi32(v, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(v);
}
template <bool fused>
std::int32_t path(const std::int32_t* cost, const std::int32_t* previous, std::int32_t* current,
                  std::size_t n, std::int32_t p1, std::int32_t p2, bool have,
                  std::int32_t previous_minimum, std::int32_t* aggregate) {
  if (n == 0) return kStereoInfinity;
  auto minimum = kStereoInfinity;
  auto vmin = _mm_set1_epi32(kStereoInfinity);
  const auto put = [&](std::size_t i, std::int32_t value) {
    current[i] = value;
    minimum = std::min(minimum, value);
    if constexpr (fused) aggregate[i] += value;
  };
  const auto put_vector = [&](std::size_t i, __m128i v) {
    _mm_storeu_si128(reinterpret_cast<__m128i*>(current + i), v);
    vmin = min_i32(vmin, v);
    if constexpr (fused) {
      auto a = _mm_loadu_si128(reinterpret_cast<const __m128i*>(aggregate + i));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(aggregate + i), _mm_add_epi32(a, v));
    }
  };
  std::size_t i = 0;
  if (!have) {
    for (; i + 4 <= n; i += 4)
      put_vector(i, _mm_loadu_si128(reinterpret_cast<const __m128i*>(cost + i)));
    for (; i < n; ++i) put(i, cost[i]);
    return std::min(minimum, reduce_min(vmin));
  }
  const auto jump = previous_minimum + p2;
  const auto vp1 = _mm_set1_epi32(p1), vjump = _mm_set1_epi32(jump),
             vold = _mm_set1_epi32(previous_minimum);
  const auto scalar = [&](std::size_t d) {
    const auto same = previous[d];
    const auto lower = d ? previous[d - 1] + p1 : kStereoInfinity;
    const auto upper = d + 1 < n ? previous[d + 1] + p1 : kStereoInfinity;
    put(d, cost[d] + std::min(std::min(same, lower), std::min(upper, jump)) - previous_minimum);
  };
  scalar(0);
  i = 1;
  for (; i + 4 < n; i += 4) {
    auto same = _mm_loadu_si128(reinterpret_cast<const __m128i*>(previous + i));
    auto lower =
        _mm_add_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previous + i - 1)), vp1);
    auto upper =
        _mm_add_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previous + i + 1)), vp1);
    auto best = min_i32(min_i32(same, lower), min_i32(upper, vjump));
    auto v = _mm_sub_epi32(
        _mm_add_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(cost + i)), best), vold);
    put_vector(i, v);
  }
  for (; i < n; ++i) scalar(i);
  return std::min(minimum, reduce_min(vmin));
}
}  // namespace
std::int32_t sgm_path_sse2(const std::int32_t* c, const std::int32_t* p, std::int32_t* out,
                           std::size_t n, std::int32_t p1, std::int32_t p2, bool have,
                           std::int32_t old, std::int32_t* aggregate) {
  return aggregate ? path<true>(c, p, out, n, p1, p2, have, old, aggregate)
                   : path<false>(c, p, out, n, p1, p2, have, old, nullptr);
}
}  // namespace mv::reconstruction

#else
namespace mv::reconstruction {
std::int32_t sgm_path_sse2(const std::int32_t* cost, const std::int32_t* previous,
                           std::int32_t* current, std::size_t n, std::int32_t p1, std::int32_t p2,
                           bool have, std::int32_t old, std::int32_t* aggregate) {
  const auto minimum = evaluate_sgm_path_cached_min(cost, previous, current, n, p1, p2, have, old);
  if (aggregate)
    for (std::size_t i = 0; i < n; ++i) aggregate[i] += current[i];
  return minimum;
}
}  // namespace mv::reconstruction
#endif
