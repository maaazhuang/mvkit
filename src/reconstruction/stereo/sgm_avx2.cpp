#include <immintrin.h>

#include <reconstruction/stereo/cost_semantics.hpp>

#include "sgm_simd.hpp"
// 公开惩罚验证保证每次路径更新不溢出 Int32；视差端点保留 infinity，向量仅处理内部标签。
namespace mv::reconstruction {
namespace {
__m256i min_i32(__m256i a, __m256i b) { return _mm256_min_epi32(a, b); }
std::int32_t reduce_min(__m256i v) {
  auto x = _mm_min_epi32(_mm256_castsi256_si128(v), _mm256_extracti128_si256(v, 1));
  x = _mm_min_epi32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(1, 0, 3, 2)));
  x = _mm_min_epi32(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(x);
}
template <bool fused>
std::int32_t path(const std::int32_t* cost, const std::int32_t* previous, std::int32_t* current,
                  std::size_t n, std::int32_t p1, std::int32_t p2, bool have,
                  std::int32_t previous_minimum, std::int32_t* aggregate) {
  if (n == 0) return kStereoInfinity;
  auto minimum = kStereoInfinity;
  auto vmin = _mm256_set1_epi32(kStereoInfinity);
  const auto put = [&](std::size_t i, std::int32_t value) {
    current[i] = value;
    minimum = std::min(minimum, value);
    if constexpr (fused) aggregate[i] += value;
  };
  const auto put_vector = [&](std::size_t i, __m256i v) {
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(current + i), v);
    vmin = min_i32(vmin, v);
    if constexpr (fused) {
      auto a = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(aggregate + i));
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(aggregate + i), _mm256_add_epi32(a, v));
    }
  };
  std::size_t i = 0;
  if (!have) {
    for (; i + 8 <= n; i += 8)
      put_vector(i, _mm256_loadu_si256(reinterpret_cast<const __m256i*>(cost + i)));
    for (; i < n; ++i) put(i, cost[i]);
    const auto result = std::min(minimum, reduce_min(vmin));
    _mm256_zeroupper();
    return result;
  }
  const auto jump = previous_minimum + p2;
  const auto vp1 = _mm256_set1_epi32(p1), vjump = _mm256_set1_epi32(jump),
             vold = _mm256_set1_epi32(previous_minimum);
  const auto scalar = [&](std::size_t d) {
    const auto same = previous[d];
    const auto lower = d ? previous[d - 1] + p1 : kStereoInfinity;
    const auto upper = d + 1 < n ? previous[d + 1] + p1 : kStereoInfinity;
    put(d, cost[d] + std::min(std::min(same, lower), std::min(upper, jump)) - previous_minimum);
  };
  scalar(0);
  i = 1;
  for (; i + 8 < n; i += 8) {
    auto same = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(previous + i));
    auto lower = _mm256_add_epi32(
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(previous + i - 1)), vp1);
    auto upper = _mm256_add_epi32(
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(previous + i + 1)), vp1);
    auto best = min_i32(min_i32(same, lower), min_i32(upper, vjump));
    auto v = _mm256_sub_epi32(
        _mm256_add_epi32(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(cost + i)), best),
        vold);
    put_vector(i, v);
  }
  for (; i < n; ++i) scalar(i);
  const auto result = std::min(minimum, reduce_min(vmin));
  _mm256_zeroupper();
  return result;
}
}  // namespace
std::int32_t sgm_path_avx2(const std::int32_t* c, const std::int32_t* p, std::int32_t* out,
                           std::size_t n, std::int32_t p1, std::int32_t p2, bool have,
                           std::int32_t old, std::int32_t* aggregate) {
  return aggregate ? path<true>(c, p, out, n, p1, p2, have, old, aggregate)
                   : path<false>(c, p, out, n, p1, p2, have, old, nullptr);
}
}  // namespace mv::reconstruction
