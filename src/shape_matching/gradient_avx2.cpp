#include "score_pruning.hpp"
#include "gradient_simd.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <immintrin.h>
#include <type_traits>
#include <vector>

namespace mv::shape_matching::simd {

void avx2_zeroupper() noexcept { _mm256_zeroupper(); }

void prepare_i8_remaining_bounds(int* bounds, std::size_t count,
                                 double greediness, double reject_below) {
  const float n = static_cast<float>(count);
  const auto begin = greediness > 0 && reject_below > 0
      ? static_cast<std::size_t>(greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below))) : count;
  const float factor = 1.0F - static_cast<float>(greediness);
  float done = 0;
  for (std::size_t i = 0; i < count; ++i) {
    done += 1.0F;
    int remaining = static_cast<int>((n - done) * (127 * 127));
    if (i >= begin) remaining = static_cast<int>(static_cast<float>(remaining) * factor);
    bounds[i] = remaining;
  }
}

template <bool interleaved>
static __m256i load_i8_query_pairs(const std::int8_t* nx, const std::int8_t* ny,
                                  const std::int8_t* pairs, std::ptrdiff_t index) {
  if constexpr (interleaved) {
    return _mm256_cvtepi8_epi16(_mm_loadu_si128(
        reinterpret_cast<const __m128i*>(pairs + 2 * index)));
  } else {
    return _mm256_cvtepi8_epi16(_mm_unpacklo_epi8(
        _mm_loadl_epi64(reinterpret_cast<const __m128i*>(ny + index)),
        _mm_loadl_epi64(reinterpret_cast<const __m128i*>(nx + index))));
  }
}

I8ScoreSetup prepare_i8_score_setup(std::size_t count, double greediness, double reject_below) {
  constexpr int quant_sq = 127 * 127;
  const float n = static_cast<float>(count);
  const int threshold = static_cast<int>(static_cast<float>(reject_below) * n * quant_sq);
  const std::size_t begin = greediness > 0 && reject_below > 0
      ? static_cast<std::size_t>(greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below))) : count;
  const auto unchecked = static_cast<int>((n - static_cast<float>(begin)) * quant_sq) >= threshold
      ? begin : std::size_t{0};
  return {threshold, unchecked, reject_below <= 0 ? count : std::min(count, unchecked),
          1.0 / (static_cast<double>(n) * quant_sq)};
}

template <bool ignore_local, bool ignore_global, bool planned, bool cached = false,
          bool interleaved = false, class Point>
#if defined(_MSC_VER)
// 融合扫描保留缓存锚点向量，避免每个存活八中心组跨越 Windows ABI。
__forceinline
#endif
static unsigned score_i8_group8_impl(const std::int8_t* qnx, const std::int8_t* qny,
                         const Point* points, std::size_t count,
                         std::size_t center, int live_mask, double greediness,
                         double reject_below, double* scores, const int* remaining_bounds,
                         const __m256i* anchor_dots = nullptr,
                         const std::int8_t* query_pairs = nullptr,
                         const I8ScoreSetup* setup = nullptr) {
  // 融合扫描一次清零整个输出游程，拒绝组直接返回，不反复写读八个零 Double。
  if constexpr (!cached) std::fill_n(scores, 8, 0.0);
  live_mask &= 255;
  if (!live_mask || !count) return 0;
  constexpr int quant_sq = 127 * 127;
  const float n = static_cast<float>(count);
  const int threshold = cached ? setup->threshold
      : static_cast<int>(static_cast<float>(reject_below) * n * quant_sq);
  const __m256i limit = _mm256_set1_epi32(threshold);
  const __m256i zero = _mm256_setzero_si256();
  __m256i pos = zero, neg = zero;
  int rejected = 255 & ~live_mask;
  const float remaining_factor = 1.0F - static_cast<float>(greediness);
  const std::size_t greedy_begin = greediness > 0 && reject_below > 0
      ? static_cast<std::size_t>(greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below))) : count;
  // 评分非负；greediness 开始前，剩余点界不小于阈值时不可能拒绝任何 lane。
  const std::size_t unchecked_prefix = cached ? setup->unchecked_prefix :
      static_cast<int>((n - static_cast<float>(greedy_begin)) * quant_sq) >= threshold
          ? greedy_begin : 0;
  auto apply_dot = [&](__m256i dot) {
    pos = _mm256_add_epi32(pos, ignore_local ? _mm256_abs_epi32(dot) : _mm256_max_epi32(zero, dot));
    if (ignore_global) neg = _mm256_add_epi32(neg, _mm256_max_epi32(zero, _mm256_sub_epi32(zero, dot)));
  };
  auto accumulate = [&](std::size_t i) {
    const auto& point = points[i];
    const auto index = static_cast<std::ptrdiff_t>(center) + point.image_offset;
    const auto pairs = load_i8_query_pairs<interleaved>(qnx, qny, query_pairs, index);
    const auto normal_pair = [&] {
      if constexpr (std::is_same_v<Point, PackedI8ShapePoint>) return point.normal_pair;
      else return pack_i8_shape_point(point).normal_pair;
    }();
    const auto coef = _mm256_set1_epi32(static_cast<int>(normal_pair));
    apply_dot(_mm256_madd_epi16(pairs, coef));
  };
  // 将可证明不拒绝的前缀与检查后缀分开，保留全部后缀拒绝点；greediness 不能安全批量合并。
  const std::size_t prefix = cached ? setup->prefix :
      (reject_below <= 0 ? count : std::min(count, unchecked_prefix));
  std::size_t i = 0;
  if constexpr (cached) {
    static_assert(planned);
    // 缓存锚点重放一次，保留每个原拒绝位置；其余点循环不再检查缓存成员关系。
    for (; i < 4; ++i) {
      apply_dot(anchor_dots[i]);
      if (reject_below <= 0 || i < unchecked_prefix) continue;
      const auto current = ignore_global ? _mm256_max_epi32(pos, neg) : pos;
      rejected |= _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpgt_epi32(limit,
          _mm256_add_epi32(current, _mm256_set1_epi32(remaining_bounds[i])))));
      if ((rejected & live_mask) == live_mask) return 0;
    }
  }
  for (; i + 4 <= prefix; i += 4) {
    accumulate(i);
    accumulate(i + 1);
    accumulate(i + 2);
    accumulate(i + 3);
  }
  for (; i < prefix; ++i) accumulate(i);
  float done = 0;
  if constexpr (!planned) {
    // 超大模型也复现原重复 Float 增量。
    for (std::size_t j = 0; j < prefix; ++j) done += 1.0F;
  }
  for (; i < count; ++i) {
    accumulate(i);
    if constexpr (!planned) done += 1.0F;
    const __m256i current = ignore_global ? _mm256_max_epi32(pos, neg) : pos;
    int remaining;
    if constexpr (planned) {
      remaining = remaining_bounds[i];
    } else {
      remaining = static_cast<int>((n - done) * quant_sq);
      if (i >= greedy_begin) remaining = static_cast<int>(static_cast<float>(remaining) * remaining_factor);
    }
    const __m256i upper = _mm256_add_epi32(current, _mm256_set1_epi32(remaining));
    rejected |= _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpgt_epi32(limit, upper)));
    if ((rejected & live_mask) == live_mask) return 0;
  }
  alignas(32) int values[8];
  _mm256_store_si256(reinterpret_cast<__m256i*>(values), ignore_global ? _mm256_max_epi32(pos, neg) : pos);
  const double inv_n = cached ? setup->inverse : 1.0 / (static_cast<double>(n) * quant_sq);
  for (int lane = 0; lane < 8; ++lane)
    if ((live_mask & (1 << lane)) && !(rejected & (1 << lane)))
      scores[lane] = std::clamp(static_cast<double>(values[lane]) * inv_n, 0.0, 1.0);
  return static_cast<unsigned>(live_mask & ~rejected);
}

void score_i8_group8_avx2(const std::int8_t* qnx, const std::int8_t* qny,
                         const DiscreteShapePoint* points, std::size_t count,
                         std::size_t center, int live_mask, bool ignore_local,
                         bool ignore_global, double greediness,
                         double reject_below, double* scores, const int* remaining_bounds,
                         const PackedI8ShapePoint* packed_points) {
  auto invoke = [&]<bool local, bool global>(const auto* actual_points) {
    if (remaining_bounds)
      score_i8_group8_impl<local, global, true>(qnx, qny, actual_points, count, center, live_mask, greediness, reject_below, scores, remaining_bounds);
    else
      score_i8_group8_impl<local, global, false>(qnx, qny, actual_points, count, center, live_mask, greediness, reject_below, scores, nullptr);
  };
  auto run = [&]<bool local, bool global>() {
    if (packed_points) invoke.template operator()<local, global>(packed_points);
    else invoke.template operator()<local, global>(points);
  };
  if (ignore_local) {
    if (ignore_global) run.template operator()<true, true>();
    else run.template operator()<true, false>();
  } else {
    if (ignore_global) run.template operator()<false, true>();
    else run.template operator()<false, false>();
  }
}

void build_direction_maps_avx2(const std::uint32_t* normals, std::size_t normal_count,
    const std::uint16_t* keys, std::size_t key_count, const std::size_t* starts,
    const std::size_t* positions, int threshold, bool polarity, std::size_t words,
    std::uint64_t* maps, void (*poll)(const void*), const void* context) {
  const auto limit = _mm256_set1_epi32(threshold);
  if (normal_count >= 16) {
    // 每查询像素仅属于一个键，改变键的精确谓词只切换互不相交像素位；近邻法向角通常只改变少量键。
    // 角排序仅影响成本，每个整数点积仍检查，包含量化不规则性及极性。
    std::vector<std::pair<double, std::size_t>> order;
    order.reserve(normal_count);
    for (std::size_t n = 0; n < normal_count; ++n)
      order.emplace_back(std::atan2(static_cast<std::int16_t>(normals[n] & 65535),
                                   static_cast<std::int16_t>(normals[n] >> 16)), n);
    std::sort(order.begin(), order.end());
    const auto groups = (key_count + 7) / 8;
    std::vector<std::uint32_t> pairs(groups * 8, 0);
    for (std::size_t k = 0; k < key_count; ++k)
      pairs[k] = static_cast<std::uint16_t>(static_cast<std::int8_t>(keys[k] & 255)) |
          (static_cast<std::uint32_t>(static_cast<std::uint16_t>(static_cast<std::int8_t>(keys[k] >> 8))) << 16);
    std::vector<unsigned> previous(groups, 0);
    std::vector<std::uint64_t> active(words, 0);
    for (const auto& direction : order) {
      const auto n = direction.second;
      if (poll) poll(context);
      const auto normal = _mm256_set1_epi32(static_cast<int>(normals[n]));
      for (std::size_t group = 0; group < groups; ++group) {
        const auto dot = _mm256_madd_epi16(normal,
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(pairs.data() + group * 8)));
        const auto current = static_cast<unsigned>(_mm256_movemask_ps(_mm256_castsi256_ps(
            _mm256_cmpgt_epi32(polarity ? dot : _mm256_abs_epi32(dot), limit))));
        auto changed = current ^ previous[group];
        previous[group] = current;
        while (changed) {
          const auto key = keys[group * 8 + std::countr_zero(changed)];
          for (auto j = starts[key]; j < starts[key + 1]; ++j) {
            const auto index = positions[j];
            active[index / 64] ^= std::uint64_t{1} << (index % 64);
          }
          changed &= changed - 1;
        }
      }
      std::copy(active.begin(), active.end(), maps + n * words);
    }
    _mm256_zeroupper();
    return;
  }
  for (std::size_t k = 0; k < key_count; ++k) {
    if (poll && k % 32 == 0) poll(context);
    const auto key = keys[k];
    const auto row = static_cast<std::int8_t>(key & 255);
    const auto col = static_cast<std::int8_t>(key >> 8);
    const auto pair = static_cast<std::uint16_t>(row) |
        (static_cast<std::uint32_t>(static_cast<std::uint16_t>(col)) << 16);
    const auto query = _mm256_set1_epi32(static_cast<int>(pair));
    auto emit = [&](std::size_t n) {
      auto* map = maps + n * words;
      for (auto j = starts[key]; j < starts[key + 1]; ++j) {
        const auto index = positions[j];
        map[index / 64] |= std::uint64_t{1} << (index % 64);
      }
    };
    std::size_t n = 0;
    for (; n + 8 <= normal_count; n += 8) {
      const auto dot = _mm256_madd_epi16(query,
          _mm256_loadu_si256(reinterpret_cast<const __m256i*>(normals + n)));
      auto mask = static_cast<unsigned>(_mm256_movemask_ps(_mm256_castsi256_ps(
          _mm256_cmpgt_epi32(polarity ? dot : _mm256_abs_epi32(dot), limit))));
      while (mask) {
        emit(n + std::countr_zero(mask));
        mask &= mask - 1;
      }
    }
    for (; n < normal_count; ++n) {
      const int dot = row * static_cast<std::int16_t>(normals[n] & 65535) +
                      col * static_cast<std::int16_t>(normals[n] >> 16);
      if ((polarity ? dot : std::abs(dot)) > threshold) emit(n);
    }
  }
  _mm256_zeroupper();
}

std::size_t scan_i8_run_avx2(const std::int8_t* qnx, const std::int8_t* qny,
                            const PackedI8ShapePoint* points, std::size_t count,
                            std::size_t center, std::size_t centers,
                            int min_hits, int evidence_threshold, bool use_polarity,
                            bool ignore_local, bool ignore_global, double greediness,
                            double reject_below, const int* remaining_bounds,
                            double* scores, std::uint64_t* accepted_mask,
                            const std::int8_t* query_pairs,
                            const std::uint64_t* exact_evidence,
                            const I8ScoreSetup* score_setup) {
  const auto all = centers == 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << centers) - 1;
  std::fill_n(scores, centers, 0.0);
  if (accepted_mask) *accepted_mask = reject_below <= 0 ? all : 0;
  I8ScoreSetup local_setup;
  if (!score_setup) {
    local_setup = prepare_i8_score_setup(count, greediness, reject_below);
    score_setup = &local_setup;
  }
  const auto& setup = *score_setup;
  auto run = [&]<bool local, bool global, bool interleaved, bool indexed>() {
    std::size_t scored = 0;
    const auto threshold = _mm256_set1_epi32(evidence_threshold);
    auto pending = indexed ? *exact_evidence : std::uint64_t{0};
    if constexpr (indexed) {
      pending &= all;
    }
    for (std::size_t c = 0; indexed ? pending != 0 : c < centers; c += indexed ? 0 : 8) {
      if constexpr (indexed) {
        c = static_cast<std::size_t>(std::countr_zero(pending)) & ~std::size_t{7};
        pending &= ~(std::uint64_t{255} << c);
      }
      alignas(32) __m256i dots[4];
      auto hits = _mm256_setzero_si256();
      for (std::size_t a = 0; a < 4; ++a) {
        const auto index = static_cast<std::ptrdiff_t>(center + c) + points[a].image_offset;
        dots[a] = _mm256_madd_epi16(load_i8_query_pairs<interleaved>(qnx, qny, query_pairs, index),
                                  _mm256_set1_epi32(static_cast<int>(points[a].normal_pair)));
        if constexpr (!indexed) {
          const auto value = use_polarity ? dots[a] : _mm256_abs_epi32(dots[a]);
          hits = _mm256_sub_epi32(hits, _mm256_cmpgt_epi32(value, threshold));
        }
      }
      const auto lanes = std::min<std::size_t>(8, centers - c);
      const auto lane_mask = (1u << lanes) - 1;
      const int evidence = (indexed ? static_cast<int>((*exact_evidence >> c) & 255) :
          _mm256_movemask_ps(_mm256_castsi256_ps(
          _mm256_cmpgt_epi32(hits, _mm256_set1_epi32(std::max(1, min_hits) - 1))))) & lane_mask;
      if (!evidence) {
        continue;
      }
      scored += static_cast<std::size_t>(std::popcount(static_cast<unsigned>(evidence)));
      double tail_scores[8];
      auto* group_scores = lanes == 8 ? scores + c : tail_scores;
      if (lanes != 8) std::fill_n(tail_scores, 8, 0.0);
      const auto surviving = score_i8_group8_impl<local, global, true, true, interleaved>(qnx, qny, points, count,
          center + c, evidence, greediness, reject_below, group_scores, remaining_bounds, dots, query_pairs, &setup);
      if (!surviving) continue;
      if (accepted_mask && reject_below > 0) {
        const auto limit = _mm256_set1_pd(reject_below);
        const auto low = _mm256_movemask_pd(_mm256_cmp_pd(
            _mm256_loadu_pd(group_scores), limit, _CMP_GE_OQ));
        const auto high = _mm256_movemask_pd(_mm256_cmp_pd(
            _mm256_loadu_pd(group_scores + 4), limit, _CMP_GE_OQ));
        *accepted_mask |= static_cast<std::uint64_t>((low | (high << 4)) & lane_mask) << c;
      }
      if (lanes != 8) std::copy_n(tail_scores, lanes, scores + c);
    }
    return scored;
  };
  auto select_layout = [&]<bool local, bool global>() {
    if (exact_evidence) return query_pairs ? run.template operator()<local, global, true, true>()
                                          : run.template operator()<local, global, false, true>();
    return query_pairs ? run.template operator()<local, global, true, false>()
                       : run.template operator()<local, global, false, false>();
  };
  std::size_t scored;
  if (ignore_local) scored = ignore_global ? select_layout.template operator()<true, true>() : select_layout.template operator()<true, false>();
  else scored = ignore_global ? select_layout.template operator()<false, true>() : select_layout.template operator()<false, false>();
  _mm256_zeroupper();
  return scored;
}

std::size_t sobel_u8_i16_row_avx2(const std::uint8_t* __restrict top,
                                  const std::uint8_t* __restrict middle,
                                  const std::uint8_t* __restrict bottom,
                                  std::int16_t* __restrict gx,
                                  std::int16_t* __restrict gy,
                                  std::size_t width) {
  std::size_t c = 1;
  if (width < 18) return c;
  // u8→i16 零扩展后按 16 像素一组做 3x3 Sobel，结果仍是未缩放的整数响应。
  auto load16_u16 = [](const std::uint8_t* values) {
    return _mm256_cvtepu8_epi16(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(values)));
  };
  for (; c + 15 < width - 1; c += 16) {
    const __m256i top_left = load16_u16(top + c - 1);
    const __m256i top_center = load16_u16(top + c);
    const __m256i top_right = load16_u16(top + c + 1);
    const __m256i middle_left = load16_u16(middle + c - 1);
    const __m256i middle_right = load16_u16(middle + c + 1);
    const __m256i bottom_left = load16_u16(bottom + c - 1);
    const __m256i bottom_center = load16_u16(bottom + c);
    const __m256i bottom_right = load16_u16(bottom + c + 1);
    const __m256i sx_right = _mm256_add_epi16(
        _mm256_add_epi16(top_right, bottom_right),
        _mm256_slli_epi16(middle_right, 1));
    const __m256i sx_left = _mm256_add_epi16(
        _mm256_add_epi16(top_left, bottom_left),
        _mm256_slli_epi16(middle_left, 1));
    const __m256i sy_bottom = _mm256_add_epi16(
        _mm256_add_epi16(bottom_left, bottom_right),
        _mm256_slli_epi16(bottom_center, 1));
    const __m256i sy_top = _mm256_add_epi16(
        _mm256_add_epi16(top_left, top_right),
        _mm256_slli_epi16(top_center, 1));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(gx + c),
                        _mm256_sub_epi16(sx_right, sx_left));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(gy + c),
                        _mm256_sub_epi16(sy_bottom, sy_top));
  }
  _mm256_zeroupper();
  return c;
}

std::size_t unit_sobel_f32_row_avx2(const float* __restrict top,
                                    const float* __restrict middle,
                                    const float* __restrict bottom,
                                    float* __restrict nx,
                                    float* __restrict ny,
                                    std::size_t width, float min_squared) {
  std::size_t c = 1;
  if (width < 10) return c;
  const __m256 quarter = _mm256_set1_ps(0.25F);
  const __m256 one = _mm256_set1_ps(1.0F);
  const __m256 threshold = _mm256_set1_ps(min_squared);
  // 8 个相邻 float 像素做 3x3 Sobel，按幅值阈值写出单位法向。
  for (; c + 7 < width - 1; c += 8) {
    const __m256 top_left = _mm256_loadu_ps(top + c - 1);
    const __m256 top_center = _mm256_loadu_ps(top + c);
    const __m256 top_right = _mm256_loadu_ps(top + c + 1);
    const __m256 middle_left = _mm256_loadu_ps(middle + c - 1);
    const __m256 middle_right = _mm256_loadu_ps(middle + c + 1);
    const __m256 bottom_left = _mm256_loadu_ps(bottom + c - 1);
    const __m256 bottom_center = _mm256_loadu_ps(bottom + c);
    const __m256 bottom_right = _mm256_loadu_ps(bottom + c + 1);
    const __m256 sx = _mm256_sub_ps(
        _mm256_add_ps(_mm256_add_ps(top_right, bottom_right),
                      _mm256_add_ps(middle_right, middle_right)),
        _mm256_add_ps(_mm256_add_ps(top_left, bottom_left),
                      _mm256_add_ps(middle_left, middle_left)));
    const __m256 sy = _mm256_sub_ps(
        _mm256_add_ps(_mm256_add_ps(bottom_left, bottom_right),
                      _mm256_add_ps(bottom_center, bottom_center)),
        _mm256_add_ps(_mm256_add_ps(top_left, top_right),
                      _mm256_add_ps(top_center, top_center)));
    const __m256 gx = _mm256_mul_ps(sx, quarter);
    const __m256 gy = _mm256_mul_ps(sy, quarter);
    const __m256 squared =
        _mm256_add_ps(_mm256_mul_ps(gx, gx), _mm256_mul_ps(gy, gy));
    const __m256 live = _mm256_cmp_ps(squared, threshold, _CMP_GE_OQ);
    const __m256 inverse = _mm256_div_ps(one, _mm256_sqrt_ps(squared));
    _mm256_storeu_ps(nx + c, _mm256_and_ps(live, _mm256_mul_ps(gx, inverse)));
    _mm256_storeu_ps(ny + c, _mm256_and_ps(live, _mm256_mul_ps(gy, inverse)));
  }
  _mm256_zeroupper();
  return c;
}

std::size_t sobel_f32_i16_row_avx2(const float* __restrict top,
                                   const float* __restrict middle,
                                   const float* __restrict bottom,
                                   std::int16_t* __restrict gx,
                                   std::int16_t* __restrict gy,
                                   std::size_t width, std::size_t col_begin,
                                   std::size_t col_end) {
  std::size_t c = col_begin < 1 ? 1 : col_begin;
  if (width < 10 || col_end <= c) return c;
  const std::size_t last = col_end < width ? col_end : width;
  // 8 个 float 像素做未缩放 Sobel，cvt 成 int16，与 UInt8 路径同一 integer_scale=0.25。
  for (; c + 7 < last && c + 7 < width - 1; c += 8) {
    const __m256 top_left = _mm256_loadu_ps(top + c - 1);
    const __m256 top_center = _mm256_loadu_ps(top + c);
    const __m256 top_right = _mm256_loadu_ps(top + c + 1);
    const __m256 middle_left = _mm256_loadu_ps(middle + c - 1);
    const __m256 middle_right = _mm256_loadu_ps(middle + c + 1);
    const __m256 bottom_left = _mm256_loadu_ps(bottom + c - 1);
    const __m256 bottom_center = _mm256_loadu_ps(bottom + c);
    const __m256 bottom_right = _mm256_loadu_ps(bottom + c + 1);
    const __m256 sx = _mm256_sub_ps(
        _mm256_add_ps(_mm256_add_ps(top_right, bottom_right),
                      _mm256_add_ps(middle_right, middle_right)),
        _mm256_add_ps(_mm256_add_ps(top_left, bottom_left),
                      _mm256_add_ps(middle_left, middle_left)));
    const __m256 sy = _mm256_sub_ps(
        _mm256_add_ps(_mm256_add_ps(bottom_left, bottom_right),
                      _mm256_add_ps(bottom_center, bottom_center)),
        _mm256_add_ps(_mm256_add_ps(top_left, top_right),
                      _mm256_add_ps(top_center, top_center)));
    const __m256i sx_i32 = _mm256_cvtps_epi32(sx);
    const __m256i sy_i32 = _mm256_cvtps_epi32(sy);
    const __m128i gx8 = _mm_packs_epi32(_mm256_castsi256_si128(sx_i32),
                                        _mm256_extracti128_si256(sx_i32, 1));
    const __m128i gy8 = _mm_packs_epi32(_mm256_castsi256_si128(sy_i32),
                                        _mm256_extracti128_si256(sy_i32, 1));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(gx + c), gx8);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(gy + c), gy8);
  }
  _mm256_zeroupper();
  return c;
}

namespace {

int i8_group_evidence_mask_avx2_impl(const std::int8_t* __restrict qnx,
                                     const std::int8_t* __restrict qny,
                                     const std::ptrdiff_t* __restrict offsets,
                                     const std::int8_t* __restrict normal_row,
                                     const std::int8_t* __restrict normal_col,
                                     std::size_t anchor_count,
                                     std::size_t first_center, int min_hits,
                                     int threshold, bool use_polarity,
                                     int lanes) noexcept {
  const int need = min_hits < 1 ? 1 : min_hits;
  const int used = lanes < 1 ? 1 : (lanes > 8 ? 8 : lanes);
  const int lane_bits = (1 << used) - 1;
  const auto base = static_cast<std::ptrdiff_t>(first_center);
  const __m256i thr = _mm256_set1_epi32(threshold);
  if (anchor_count == 4 && need == 2) {
    auto anchor_mask = [&](std::size_t a) {
      const auto index = base + offsets[a];
      const auto nx = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(qnx + index));
      const auto ny = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(qny + index));
      const auto pairs = _mm256_cvtepi8_epi16(_mm_unpacklo_epi8(ny, nx));
      const auto nr = static_cast<std::uint16_t>(normal_row[a]);
      const auto nc = static_cast<std::uint16_t>(normal_col[a]);
      auto dot = _mm256_madd_epi16(pairs, _mm256_set1_epi32((static_cast<unsigned>(nc) << 16) | nr));
      if (!use_polarity) dot = _mm256_abs_epi32(dot);
      return _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpgt_epi32(dot, thr))) & lane_bits;
    };
    // 标量掩码记录一次/两次命中的 lane，替代八个计数器及逐锚点上界检查。
    const int first = anchor_mask(0), second = anchor_mask(1);
    int twice = first & second;
    if (twice == lane_bits) return twice;
    int once = first | second;
    const int third = anchor_mask(2);
    twice |= once & third;
    once |= third;
    if (twice == lane_bits || once == 0) return twice;
    return twice | (once & anchor_mask(3));
  }
  const __m256i need_i = _mm256_set1_epi32(need);
  const __m256i one = _mm256_set1_epi32(1);
  const __m256i need_minus_one = _mm256_sub_epi32(need_i, one);
  __m256i hits = _mm256_setzero_si256();
  for (std::size_t a = 0; a < anchor_count; ++a) {
    const auto index = static_cast<std::size_t>(base + offsets[a]);
    // 8 个相邻 int8 法向符号扩展后与模型法向做点积，阈值仍是 0.95×127²。
    const __m128i nx8 =
        _mm_loadl_epi64(reinterpret_cast<const __m128i*>(qnx + index));
    const __m128i ny8 =
        _mm_loadl_epi64(reinterpret_cast<const __m128i*>(qny + index));
    const __m256i pairs = _mm256_cvtepi8_epi16(_mm_unpacklo_epi8(ny8, nx8));
    const auto nr = static_cast<std::uint16_t>(normal_row[a]);
    const auto nc = static_cast<std::uint16_t>(normal_col[a]);
    const __m256i coef = _mm256_set1_epi32((static_cast<unsigned>(nc) << 16) | nr);
    __m256i dot = _mm256_madd_epi16(pairs, coef);
    if (!use_polarity) dot = _mm256_abs_epi32(dot);
    hits = _mm256_add_epi32(
        hits, _mm256_and_si256(_mm256_cmpgt_epi32(dot, thr), one));
    const int ge = _mm256_movemask_ps(_mm256_castsi256_ps(
                       _mm256_cmpgt_epi32(hits, need_minus_one))) &
                   lane_bits;
    if (ge == lane_bits) break;
    const int remaining = static_cast<int>(anchor_count - a - 1);
    const int possible =
        _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpgt_epi32(
            _mm256_add_epi32(hits, _mm256_set1_epi32(remaining)),
            need_minus_one))) &
        lane_bits;
    if (possible == 0) break;
  }
  return _mm256_movemask_ps(_mm256_castsi256_ps(
             _mm256_cmpgt_epi32(hits, need_minus_one))) &
         lane_bits;
}

}  // namespace

int i8_group_evidence_mask_avx2(const std::int8_t* __restrict qnx,
                                const std::int8_t* __restrict qny,
                                const std::ptrdiff_t* __restrict offsets,
                                const std::int8_t* __restrict normal_row,
                                const std::int8_t* __restrict normal_col,
                                std::size_t anchor_count,
                                std::size_t first_center, int min_hits,
                                int threshold, bool use_polarity,
                                int lanes) noexcept {
  // 热循环由调用方在切回 SSE 打分前 zeroupper，避免每个 pack 都付切换税。
  return i8_group_evidence_mask_avx2_impl(
      qnx, qny, offsets, normal_row, normal_col, anchor_count, first_center,
      min_hits, threshold, use_polarity, lanes);
}

std::size_t i8_collect_screen_hits_avx2(
    const std::int8_t* __restrict qnx, const std::int8_t* __restrict qny,
    const std::ptrdiff_t* __restrict offsets,
    const std::int8_t* __restrict normal_row,
    const std::int8_t* __restrict normal_col, std::size_t anchor_count,
    std::size_t screen_w, std::size_t screen_h, std::size_t unit_w,
    std::size_t unit_h, double min_row, double max_row, double min_col,
    double max_col, int min_hits, int threshold, bool use_polarity,
    std::uint32_t* __restrict hits, std::size_t hits_capacity,
    std::size_t* __restrict evaluated) noexcept {
  std::size_t n_hits = 0;
  std::size_t n_eval = 0;
  const double max_center_col = static_cast<double>(unit_w - 1);
  const double max_center_row = static_cast<double>(unit_h - 1);
  for (std::size_t sr = 0; sr < screen_h; ++sr) {
    const double coarse_row = 2.0 * static_cast<double>(sr);
    if (coarse_row + min_row < 0.0 ||
        coarse_row + 1.0 + max_row > max_center_row)
      continue;
    for (std::size_t sc = 0; sc < screen_w;) {
      const double coarse_col = 2.0 * static_cast<double>(sc);
      if (coarse_col + min_col < 0.0 || coarse_col + max_col > max_center_col) {
        ++sc;
        continue;
      }
      const std::size_t sindex = sr * screen_w + sc;
      int pack = 1;
      if (sc + 8 <= screen_w &&
          2.0 * static_cast<double>(sc + 7) + max_col <= max_center_col)
        pack = 8;
      n_eval += static_cast<std::size_t>(pack);
      const int evidence = i8_group_evidence_mask_avx2_impl(
          qnx, qny, offsets, normal_row, normal_col, anchor_count, sindex,
          min_hits, threshold, use_polarity, pack);
      for (int lane = 0; lane < pack; ++lane) {
        if ((evidence & (1 << lane)) == 0) continue;
        if (n_hits < hits_capacity)
          hits[n_hits] = static_cast<std::uint32_t>(
              (sr << 16) | (sc + static_cast<std::size_t>(lane)));
        ++n_hits;
      }
      sc += static_cast<std::size_t>(pack);
    }
  }
  if (evaluated) *evaluated = n_eval;
  _mm256_zeroupper();
  return n_hits;
}

namespace {

struct RotatedQuery {
  __m256 rr;
  __m256 cc;
  __m256 nr;
  __m256 nc;
};

RotatedQuery rotate_query_avx2(__m256 prow, __m256 pcol, __m256 pnr, __m256 pnc,
                               __m256 row, __m256 col, __m256 ca, __m256 sa,
                               __m256 scale) noexcept {
  // 模型点绕原点旋转并平移到候选中心，与标量路径同一组 mul/add。
  const __m256 rot_r =
      _mm256_sub_ps(_mm256_mul_ps(ca, prow), _mm256_mul_ps(sa, pcol));
  const __m256 rot_c =
      _mm256_add_ps(_mm256_mul_ps(ca, pcol), _mm256_mul_ps(sa, prow));
  RotatedQuery q;
  q.rr = _mm256_add_ps(row, _mm256_mul_ps(scale, rot_r));
  q.cc = _mm256_add_ps(col, _mm256_mul_ps(scale, rot_c));
  q.nr = _mm256_sub_ps(_mm256_mul_ps(ca, pnr), _mm256_mul_ps(sa, pnc));
  q.nc = _mm256_add_ps(_mm256_mul_ps(ca, pnc), _mm256_mul_ps(sa, pnr));
  return q;
}

__m256i nearest_index_avx2(__m256 rr, __m256 cc, int width) noexcept {
  const __m256 half = _mm256_set1_ps(0.5F);
  const __m256i ri = _mm256_cvttps_epi32(_mm256_add_ps(rr, half));
  const __m256i ci = _mm256_cvttps_epi32(_mm256_add_ps(cc, half));
  return _mm256_add_epi32(_mm256_mullo_epi32(ri, _mm256_set1_epi32(width)), ci);
}

float horizontal_sum_max0(__m256 values) noexcept {
  alignas(32) float lanes[8];
  _mm256_store_ps(lanes, _mm256_max_ps(values, _mm256_setzero_ps()));
  float sum = 0.0F;
  for (int k = 0; k < 8; ++k) sum += lanes[k];
  return sum;
}

bool reject_after(float pos, float done, float n, float greediness,
                  float reject_below, float greedy_begin) noexcept {
  if (reject_below <= 0.0F) return false;
  const float reject_total = reject_below * n;
  const float remaining = n - done;
  if (pos + remaining < reject_total) return true;
  return greediness > 0.0F && done >= greedy_begin &&
         pos + remaining * (1.0F - greediness) < reject_total;
}

__m256 extend_i16_ps(__m256i packed) noexcept {
  return _mm256_cvtepi32_ps(
      _mm256_srai_epi32(_mm256_slli_epi32(packed, 16), 16));
}

// 四点双线性：floor 后 gather 四个 int16 角点，再按分数坐标 lerp。
template<bool Packed, bool FuseTop = false>
void gather_i16_bilinear_impl(const int* __restrict gx32,
                              const int* __restrict gy32, int width, __m256 rr,
                              __m256 cc, __m256 integer_scale, __m256* gxf,
                              __m256* gyf, const std::uint32_t* packed) noexcept {
  const __m256 one = _mm256_set1_ps(1.0F);
  const __m256i r0 = _mm256_cvttps_epi32(rr);
  const __m256i c0 = _mm256_cvttps_epi32(cc);
  const __m256 fr = _mm256_sub_ps(rr, _mm256_cvtepi32_ps(r0));
  const __m256 fc = _mm256_sub_ps(cc, _mm256_cvtepi32_ps(c0));
  const __m256 omfr = _mm256_sub_ps(one, fr);
  const __m256 omfc = _mm256_sub_ps(one, fc);
  const __m256i vw = _mm256_set1_epi32(width);
  const __m256i i00 =
      _mm256_add_epi32(_mm256_mullo_epi32(r0, vw), c0);
  const __m256i i01 = _mm256_add_epi32(i00, _mm256_set1_epi32(1));
  const __m256i i10 = _mm256_add_epi32(i00, vw);
  const __m256i i11 = _mm256_add_epi32(i10, _mm256_set1_epi32(1));
  if constexpr (Packed) {
    const auto* components = reinterpret_cast<const int*>(packed);
    const auto p00 = _mm256_i32gather_epi32(components, i00, 4);
    const auto p01 = _mm256_i32gather_epi32(components, i01, 4);
    const auto p10 = _mm256_i32gather_epi32(components, i10, 4);
    const auto p11 = _mm256_i32gather_epi32(components, i11, 4);
    auto x = [&](const __m256i value) { return _mm256_mul_ps(extend_i16_ps(value), integer_scale); };
    auto y = [&](const __m256i value) {
      return _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_srai_epi32(value, 16)), integer_scale);
    };
    auto interpolate = [&](__m256 a, __m256 b, __m256 c, __m256 d) {
#if defined(_MSC_VER)
      // MSVC 将原双平面表达式收缩为 FMA：左水平项融合，右项独立舍入。
      // 打包读取改变调度，须显式保持该舍入序，避免编译器融合相反项。
      const auto top = _mm256_fmadd_ps(omfc, a, _mm256_mul_ps(fc, b));
      const auto bottom = _mm256_fmadd_ps(omfc, c, _mm256_mul_ps(fc, d));
      if constexpr (FuseTop)
        return _mm256_fmadd_ps(omfr, top, _mm256_mul_ps(fr, bottom));
      return _mm256_fmadd_ps(fr, bottom, _mm256_mul_ps(omfr, top));
#else
      return _mm256_add_ps(
          _mm256_mul_ps(omfr, _mm256_add_ps(_mm256_mul_ps(omfc, a), _mm256_mul_ps(fc, b))),
          _mm256_mul_ps(fr, _mm256_add_ps(_mm256_mul_ps(omfc, c), _mm256_mul_ps(fc, d))));
#endif
    };
    *gxf = interpolate(x(p00), x(p01), x(p10), x(p11));
    *gyf = interpolate(y(p00), y(p01), y(p10), y(p11));
    return;
  }
  auto load = [&](const int* src, __m256i index) {
    return _mm256_mul_ps(extend_i16_ps(_mm256_i32gather_epi32(src, index, 2)),
                         integer_scale);
  };
  const __m256 g00 = load(gx32, i00);
  const __m256 g01 = load(gx32, i01);
  const __m256 g10 = load(gx32, i10);
  const __m256 g11 = load(gx32, i11);
  *gxf = _mm256_add_ps(
      _mm256_mul_ps(omfr, _mm256_add_ps(_mm256_mul_ps(omfc, g00),
                                        _mm256_mul_ps(fc, g01))),
      _mm256_mul_ps(fr, _mm256_add_ps(_mm256_mul_ps(omfc, g10),
                                      _mm256_mul_ps(fc, g11))));
  const __m256 y00 = load(gy32, i00);
  const __m256 y01 = load(gy32, i01);
  const __m256 y10 = load(gy32, i10);
  const __m256 y11 = load(gy32, i11);
  *gyf = _mm256_add_ps(
      _mm256_mul_ps(omfr, _mm256_add_ps(_mm256_mul_ps(omfc, y00),
                                        _mm256_mul_ps(fc, y01))),
      _mm256_mul_ps(fr, _mm256_add_ps(_mm256_mul_ps(omfc, y10),
                                      _mm256_mul_ps(fc, y11))));
}

void gather_i16_bilinear_avx2(const int* gx32, const int* gy32, int width,
    __m256 rr, __m256 cc, __m256 integer_scale, __m256* gxf, __m256* gyf) noexcept {
  gather_i16_bilinear_impl<false>(gx32, gy32, width, rr, cc, integer_scale, gxf, gyf, nullptr);
}

// 两布局仅 gather 不同，插值共用一块；否则模板特化中的不同读取/寄存器压力
// 可能使 MSVC 对同一表达式采用相反融合顺序。
void gather_i16_bilinear_score_avx2(const int* __restrict gx32,
    const int* __restrict gy32, const std::uint32_t* __restrict packed,
    int width, __m256 rr, __m256 cc, __m256 integer_scale,
    __m256* gxf, __m256* gyf) noexcept {
  const auto one = _mm256_set1_ps(1.0F);
  const auto r0 = _mm256_cvttps_epi32(rr);
  const auto c0 = _mm256_cvttps_epi32(cc);
  const auto fr = _mm256_sub_ps(rr, _mm256_cvtepi32_ps(r0));
  const auto fc = _mm256_sub_ps(cc, _mm256_cvtepi32_ps(c0));
  const auto omfr = _mm256_sub_ps(one, fr);
  const auto omfc = _mm256_sub_ps(one, fc);
  const auto vw = _mm256_set1_epi32(width);
  const auto i00 = _mm256_add_epi32(_mm256_mullo_epi32(r0, vw), c0);
  const auto i01 = _mm256_add_epi32(i00, _mm256_set1_epi32(1));
  const auto i10 = _mm256_add_epi32(i00, vw);
  const auto i11 = _mm256_add_epi32(i10, _mm256_set1_epi32(1));
  __m256 x00, x01, x10, x11, y00, y01, y10, y11;
  if (packed) {
    const auto* components = reinterpret_cast<const int*>(packed);
    const auto p00 = _mm256_i32gather_epi32(components, i00, 4);
    const auto p01 = _mm256_i32gather_epi32(components, i01, 4);
    const auto p10 = _mm256_i32gather_epi32(components, i10, 4);
    const auto p11 = _mm256_i32gather_epi32(components, i11, 4);
    auto x = [&](const __m256i value) {
      return _mm256_mul_ps(extend_i16_ps(value), integer_scale);
    };
    auto y = [&](const __m256i value) {
      return _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_srai_epi32(value, 16)), integer_scale);
    };
    x00 = x(p00); x01 = x(p01); x10 = x(p10); x11 = x(p11);
    y00 = y(p00); y01 = y(p01); y10 = y(p10); y11 = y(p11);
  } else {
    auto load = [&](const int* src, __m256i index) {
      return _mm256_mul_ps(extend_i16_ps(_mm256_i32gather_epi32(src, index, 2)), integer_scale);
    };
    x00 = load(gx32, i00); x01 = load(gx32, i01);
    x10 = load(gx32, i10); x11 = load(gx32, i11);
    y00 = load(gy32, i00); y01 = load(gy32, i01);
    y10 = load(gy32, i10); y11 = load(gy32, i11);
  }
  auto interpolate = [&](__m256 a, __m256 b, __m256 c, __m256 d) {
#if defined(_MSC_VER)
    // 匹配原 Release 平面评分收缩树：水平左项及垂直下项融合，其他乘积先舍入。
    const auto top = _mm256_fmadd_ps(omfc, a, _mm256_mul_ps(fc, b));
    const auto bottom = _mm256_fmadd_ps(omfc, c, _mm256_mul_ps(fc, d));
    return _mm256_fmadd_ps(fr, bottom, _mm256_mul_ps(omfr, top));
#else
    return _mm256_add_ps(
        _mm256_mul_ps(omfr, _mm256_add_ps(_mm256_mul_ps(omfc, a), _mm256_mul_ps(fc, b))),
        _mm256_mul_ps(fr, _mm256_add_ps(_mm256_mul_ps(omfc, c), _mm256_mul_ps(fc, d))));
#endif
  };
  *gxf = interpolate(x00, x01, x10, x11);
  *gyf = interpolate(y00, y01, y10, y11);
}

void bilinear_i16_scalar(const std::int16_t* __restrict gx,
                         const std::int16_t* __restrict gy, std::size_t width,
                         float integer_scale, float rr, float cc, float* gxf,
                         float* gyf) noexcept {
  const auto r0 = static_cast<std::size_t>(rr);
  const auto c0 = static_cast<std::size_t>(cc);
  const float fr = rr - static_cast<float>(r0);
  const float fc = cc - static_cast<float>(c0);
  const auto i00 = r0 * width + c0;
  const auto i01 = i00 + 1;
  const auto i10 = i00 + width;
  const auto i11 = i10 + 1;
  const float omfr = 1.0F - fr;
  const float omfc = 1.0F - fc;
  auto interp = [&](const std::int16_t* values) {
    const float top = omfc * static_cast<float>(values[i00]) +
                      fc * static_cast<float>(values[i01]);
    const float bottom = omfc * static_cast<float>(values[i10]) +
                         fc * static_cast<float>(values[i11]);
    return integer_scale * (omfr * top + fr * bottom);
  };
  *gxf = interp(gx);
  *gyf = interp(gy);
}

}  // namespace

double score_unit_use_polarity_avx2(
    const float* __restrict nx, const float* __restrict ny, std::size_t width,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept {
  if (n == 0 || width == 0) return 0.0;
  const float n_f = static_cast<float>(n);
  const float greedy_begin = greedy_prefix(n_f,greediness,reject_below);
  const __m256 vrow = _mm256_set1_ps(row);
  const __m256 vcol = _mm256_set1_ps(col);
  const __m256 vca = _mm256_set1_ps(ca);
  const __m256 vsa = _mm256_set1_ps(sa);
  const __m256 vscale = _mm256_set1_ps(scale);
  const int w = static_cast<int>(width);
  float pos = 0.0F, neg = 0.0F;
  std::size_t done = 0;
  for (; done + 8 <= n; done += 8) {
    const auto q = rotate_query_avx2(
        _mm256_loadu_ps(prow + done), _mm256_loadu_ps(pcol + done),
        _mm256_loadu_ps(pnr + done), _mm256_loadu_ps(pnc + done), vrow, vcol,
        vca, vsa, vscale);
    const __m256i index = nearest_index_avx2(q.rr, q.cc, w);
    const __m256 gnx = _mm256_i32gather_ps(nx, index, 4);
    const __m256 gny = _mm256_i32gather_ps(ny, index, 4);
    const __m256 dot =
        _mm256_add_ps(_mm256_mul_ps(q.nr, gny), _mm256_mul_ps(q.nc, gnx));
    pos += horizontal_sum_max0(metric == 2 ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), dot) : dot);
    if (metric == 1) neg += horizontal_sum_max0(_mm256_sub_ps(_mm256_setzero_ps(), dot));
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 8), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  for (; done < n; ++done) {
    const float rot_r = ca * prow[done] - sa * pcol[done];
    const float rot_c = ca * pcol[done] + sa * prow[done];
    const float rr = row + scale * rot_r;
    const float cc = col + scale * rot_c;
    const float nr = ca * pnr[done] - sa * pnc[done];
    const float nc = ca * pnc[done] + sa * pnr[done];
    const auto ri = static_cast<std::size_t>(rr + 0.5F);
    const auto ci = static_cast<std::size_t>(cc + 0.5F);
    const auto index = ri * width + ci;
    const float dot = nr * ny[index] + nc * nx[index];
    pos += metric == 2 ? std::abs(dot) : std::max(0.0F, dot);
      if (metric == 1) neg += std::max(0.0F, -dot);
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 1), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  _mm256_zeroupper();
  const float raw = (metric == 1 ? std::max(pos, neg) : pos) / n_f;
  if (raw < 0.0F) return 0.0;
  if (raw > 1.0F) return 1.0;
  return static_cast<double>(raw);
}

void prepare_i16_packed_components_avx2(const std::int16_t* gx,
    const std::int16_t* gy, std::size_t count, std::uint32_t* packed) noexcept {
  std::size_t i = 0;
  for (; i + 8 <= count; i += 8) {
    const auto x = _mm_loadu_si128(reinterpret_cast<const __m128i*>(gx+i));
    const auto y = _mm_loadu_si128(reinterpret_cast<const __m128i*>(gy+i));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(packed+i), _mm_unpacklo_epi16(x,y));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(packed+i+4), _mm_unpackhi_epi16(x,y));
  }
  for (; i < count; ++i) packed[i] = static_cast<std::uint16_t>(gx[i]) |
      (static_cast<std::uint32_t>(static_cast<std::uint16_t>(gy[i])) << 16);
  _mm256_zeroupper();
}

void prepare_i16_inverse_magnitude_avx2(const std::int16_t* gx,
    const std::int16_t* gy, float integer_scale, std::size_t count, float* inverse) noexcept {
  const auto scale = _mm256_set1_ps(integer_scale);
  const auto one = _mm256_set1_ps(1.0F);
  const auto tiny = _mm256_set1_ps(1.0e-12F);
  std::size_t i = 0;
  for (; i + 8 <= count; i += 8) {
    const auto x = _mm256_mul_ps(scale, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(gx + i)))));
    const auto y = _mm256_mul_ps(scale, _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(gy + i)))));
    const auto mag2 = _mm256_add_ps(_mm256_mul_ps(x, x), _mm256_mul_ps(y, y));
    _mm256_storeu_ps(inverse + i,
        _mm256_div_ps(one, _mm256_sqrt_ps(_mm256_max_ps(mag2, tiny))));
  }
  // 缓存标量像素之后可在任意 SIMD lane 采样，使用与 SIMD 评分相同的 SSE 操作，包含 tiny。
  for (; i < count; ++i) {
    const auto x = _mm_mul_ss(_mm_set_ss(integer_scale), _mm_set_ss(static_cast<float>(gx[i])));
    const auto y = _mm_mul_ss(_mm_set_ss(integer_scale), _mm_set_ss(static_cast<float>(gy[i])));
    const auto mag2 = _mm_add_ss(_mm_mul_ss(x, x), _mm_mul_ss(y, y));
    inverse[i] = _mm_cvtss_f32(_mm_div_ss(_mm_set_ss(1.0F),
        _mm_sqrt_ss(_mm_max_ss(mag2, _mm_set_ss(1.0e-12F)))));
  }
  _mm256_zeroupper();
}

template<bool Cached, bool Packed = false>
double score_i16_nearest_impl(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    const float* __restrict inverse,
    const std::uint32_t* __restrict packed_components,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept {
  if (n == 0 || width == 0) return 0.0;
  const float n_f = static_cast<float>(n);
  const float greedy_begin = greedy_prefix(n_f,greediness,reject_below);
  const __m256 vrow = _mm256_set1_ps(row);
  const __m256 vcol = _mm256_set1_ps(col);
  const __m256 vca = _mm256_set1_ps(ca);
  const __m256 vsa = _mm256_set1_ps(sa);
  const __m256 vscale = _mm256_set1_ps(scale);
  const __m256 v_i_scale = _mm256_set1_ps(integer_scale);
  const __m256 min_sq = _mm256_set1_ps(min_mag_squared);
  const __m256 one = _mm256_set1_ps(1.0F);
  const __m256 tiny = _mm256_set1_ps(1.0e-12F);
  const int w = static_cast<int>(width);
  const int* gx32 = reinterpret_cast<const int*>(gx);
  const int* gy32 = reinterpret_cast<const int*>(gy);
  float pos = 0.0F, neg = 0.0F;
  std::size_t done = 0;
  auto extend_i16 = [](__m256i packed) {
    return _mm256_cvtepi32_ps(
        _mm256_srai_epi32(_mm256_slli_epi32(packed, 16), 16));
  };
  for (; done + 8 <= n; done += 8) {
    const auto q = rotate_query_avx2(
        _mm256_loadu_ps(prow + done), _mm256_loadu_ps(pcol + done),
        _mm256_loadu_ps(pnr + done), _mm256_loadu_ps(pnc + done), vrow, vcol,
        vca, vsa, vscale);
    const __m256i index = nearest_index_avx2(q.rr, q.cc, w);
    // scale=2：按下标从 int16 图 gather 相邻 32bit，再符号扩展低 16 位。
    __m256 gxf, gyf;
    if constexpr (Packed) {
      const auto pair = _mm256_i32gather_epi32(reinterpret_cast<const int*>(packed_components), index, 4);
      gxf = _mm256_mul_ps(extend_i16(pair), v_i_scale);
      gyf = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_srai_epi32(pair, 16)), v_i_scale);
    } else {
      gxf = _mm256_mul_ps(extend_i16(_mm256_i32gather_epi32(gx32, index, 2)), v_i_scale);
      gyf = _mm256_mul_ps(extend_i16(_mm256_i32gather_epi32(gy32, index, 2)), v_i_scale);
    }
    const __m256 mag2 =
        _mm256_add_ps(_mm256_mul_ps(gxf, gxf), _mm256_mul_ps(gyf, gyf));
    const __m256 live = _mm256_cmp_ps(mag2, min_sq, _CMP_GE_OQ);
    const __m256 inv = Cached ? _mm256_i32gather_ps(inverse, index, 4) :
        _mm256_div_ps(one, _mm256_sqrt_ps(_mm256_max_ps(mag2, tiny)));
    const __m256 raw_dot =
        _mm256_add_ps(_mm256_mul_ps(q.nr, gyf), _mm256_mul_ps(q.nc, gxf));
    const __m256 contrib = _mm256_and_ps(live, _mm256_mul_ps(raw_dot, inv));
    pos += horizontal_sum_max0(metric == 2 ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), contrib) : contrib);
    if (metric == 1) neg += horizontal_sum_max0(_mm256_sub_ps(_mm256_setzero_ps(), contrib));
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 8), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  for (; done < n; ++done) {
    const float rot_r = ca * prow[done] - sa * pcol[done];
    const float rot_c = ca * pcol[done] + sa * prow[done];
    const float rr = row + scale * rot_r;
    const float cc = col + scale * rot_c;
    const float nr = ca * pnr[done] - sa * pnc[done];
    const float nc = ca * pnc[done] + sa * pnr[done];
    const auto ri = static_cast<std::size_t>(rr + 0.5F);
    const auto ci = static_cast<std::size_t>(cc + 0.5F);
    const auto index = ri * width + ci;
    const float gxf = integer_scale * static_cast<float>(gx[index]);
    const float gyf = integer_scale * static_cast<float>(gy[index]);
    const float mag2 = gxf * gxf + gyf * gyf;
    if (mag2 >= min_mag_squared) {
      const float dot = (nr * gyf + nc * gxf) / std::sqrt(mag2);
      pos += metric == 2 ? std::abs(dot) : std::max(0.0F, dot);
      if (metric == 1) neg += std::max(0.0F, -dot);
    }
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 1), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  _mm256_zeroupper();
  const float raw = (metric == 1 ? std::max(pos, neg) : pos) / n_f;
  if (raw < 0.0F) return 0.0;
  if (raw > 1.0F) return 1.0;
  return static_cast<double>(raw);
}

double score_i16_nearest_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  return score_i16_nearest_impl<false>(gx, gy, nullptr, nullptr, width, integer_scale,
      min_mag_squared, prow, pcol, pnr, pnc, n, row, col, ca, sa, scale,
      greediness, reject_below, metric);
}
double score_i16_nearest_cached_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  return score_i16_nearest_impl<true>(gx, gy, inverse, nullptr, width, integer_scale,
      min_mag_squared, prow, pcol, pnr, pnc, n, row, col, ca, sa, scale,
      greediness, reject_below, metric);
}
double score_i16_nearest_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    const std::uint32_t* packed, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  if (inverse) return score_i16_nearest_impl<true, true>(gx, gy, inverse, packed,
      width, integer_scale, min_mag_squared, prow, pcol, pnr, pnc, n, row, col,
      ca, sa, scale, greediness, reject_below, metric);
  return score_i16_nearest_impl<false, true>(gx, gy, nullptr, packed,
      width, integer_scale, min_mag_squared, prow, pcol, pnr, pnc, n, row, col,
      ca, sa, scale, greediness, reject_below, metric);
}

void rotate_model_points_avx2(const float* __restrict prow,
                              const float* __restrict pcol,
                              const float* __restrict pnr,
                              const float* __restrict pnc, std::size_t n,
                              float ca, float sa, float scale,
                              float* __restrict off_r, float* __restrict off_c,
                              float* __restrict nr, float* __restrict nc) noexcept {
  const __m256 vca = _mm256_set1_ps(ca);
  const __m256 vsa = _mm256_set1_ps(sa);
  const __m256 vscale = _mm256_set1_ps(scale);
  std::size_t i = 0;
  // 8 个模型点：位置乘旋转矩阵再乘 scale，法向只旋转不平移。
  for (; i + 8 <= n; i += 8) {
    const __m256 pr = _mm256_loadu_ps(prow + i);
    const __m256 pc = _mm256_loadu_ps(pcol + i);
    const __m256 n_row = _mm256_loadu_ps(pnr + i);
    const __m256 n_col = _mm256_loadu_ps(pnc + i);
    const __m256 rot_r =
        _mm256_sub_ps(_mm256_mul_ps(vca, pr), _mm256_mul_ps(vsa, pc));
    const __m256 rot_c =
        _mm256_add_ps(_mm256_mul_ps(vca, pc), _mm256_mul_ps(vsa, pr));
    _mm256_storeu_ps(off_r + i, _mm256_mul_ps(vscale, rot_r));
    _mm256_storeu_ps(off_c + i, _mm256_mul_ps(vscale, rot_c));
    _mm256_storeu_ps(
        nr + i, _mm256_sub_ps(_mm256_mul_ps(vca, n_row), _mm256_mul_ps(vsa, n_col)));
    _mm256_storeu_ps(
        nc + i, _mm256_add_ps(_mm256_mul_ps(vca, n_col), _mm256_mul_ps(vsa, n_row)));
  }
  for (; i < n; ++i) {
    off_r[i] = scale * (ca * prow[i] - sa * pcol[i]);
    off_c[i] = scale * (ca * pcol[i] + sa * prow[i]);
    nr[i] = ca * pnr[i] - sa * pnc[i];
    nc[i] = ca * pnc[i] + sa * pnr[i];
  }
  _mm256_zeroupper();
}

double score_unit_pre_rotated_use_polarity_avx2(
    const float* __restrict nx, const float* __restrict ny, std::size_t width,
    const float* __restrict off_r, const float* __restrict off_c,
    const float* __restrict nr, const float* __restrict nc, std::size_t n,
    float row, float col, float greediness, float reject_below, int metric) noexcept {
  if (n == 0 || width == 0) return 0.0;
  const float n_f = static_cast<float>(n);
  const float greedy_begin = greedy_prefix(n_f,greediness,reject_below);
  const __m256 vrow = _mm256_set1_ps(row);
  const __m256 vcol = _mm256_set1_ps(col);
  const int w = static_cast<int>(width);
  float pos = 0.0F, neg = 0.0F;
  std::size_t done = 0;
  // 预旋转偏移直接加候选中心，gather 单位法向后做 UsePolarity 点积。
  for (; done + 8 <= n; done += 8) {
    const __m256 rr = _mm256_add_ps(vrow, _mm256_loadu_ps(off_r + done));
    const __m256 cc = _mm256_add_ps(vcol, _mm256_loadu_ps(off_c + done));
    const __m256i index = nearest_index_avx2(rr, cc, w);
    const __m256 gnx = _mm256_i32gather_ps(nx, index, 4);
    const __m256 gny = _mm256_i32gather_ps(ny, index, 4);
    const __m256 dot = _mm256_add_ps(
        _mm256_mul_ps(_mm256_loadu_ps(nr + done), gny),
        _mm256_mul_ps(_mm256_loadu_ps(nc + done), gnx));
    pos += horizontal_sum_max0(metric == 2 ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), dot) : dot);
    if (metric == 1) neg += horizontal_sum_max0(_mm256_sub_ps(_mm256_setzero_ps(), dot));
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 8), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  for (; done < n; ++done) {
    const float rr = row + off_r[done];
    const float cc = col + off_c[done];
    const auto ri = static_cast<std::size_t>(rr + 0.5F);
    const auto ci = static_cast<std::size_t>(cc + 0.5F);
    const auto index = ri * width + ci;
    const float d = nr[done] * ny[index] + nc[done] * nx[index];
    pos += metric == 2 ? std::abs(d) : std::max(0.0F, d);
    if (metric == 1) neg += std::max(0.0F, -d);
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 1), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  _mm256_zeroupper();
  const float raw = (metric == 1 ? std::max(pos, neg) : pos) / n_f;
  if (raw < 0.0F) return 0.0;
  if (raw > 1.0F) return 1.0;
  return static_cast<double>(raw);
}

template<bool Cached, bool Packed = false>
double score_i16_pre_rotated_impl(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    const float* __restrict inverse,
    const std::uint32_t* __restrict packed_components,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict off_r, const float* __restrict off_c,
    const float* __restrict nr, const float* __restrict nc, std::size_t n,
    float row, float col, float greediness, float reject_below, int metric) noexcept {
  if (n == 0 || width == 0) return 0.0;
  const float n_f = static_cast<float>(n);
  const float greedy_begin = greedy_prefix(n_f,greediness,reject_below);
  const __m256 vrow = _mm256_set1_ps(row);
  const __m256 vcol = _mm256_set1_ps(col);
  const __m256 v_i_scale = _mm256_set1_ps(integer_scale);
  const __m256 min_sq = _mm256_set1_ps(min_mag_squared);
  const __m256 one = _mm256_set1_ps(1.0F);
  const __m256 tiny = _mm256_set1_ps(1.0e-12F);
  const int w = static_cast<int>(width);
  const int* gx32 = reinterpret_cast<const int*>(gx);
  const int* gy32 = reinterpret_cast<const int*>(gy);
  float pos = 0.0F, neg = 0.0F;
  std::size_t done = 0;
  auto extend_i16 = [](__m256i packed) {
    return _mm256_cvtepi32_ps(
        _mm256_srai_epi32(_mm256_slli_epi32(packed, 16), 16));
  };
  // 预旋转偏移 + int16 Sobel gather，幅值阈值后归一化点积。
  for (; done + 8 <= n; done += 8) {
    const __m256 rr = _mm256_add_ps(vrow, _mm256_loadu_ps(off_r + done));
    const __m256 cc = _mm256_add_ps(vcol, _mm256_loadu_ps(off_c + done));
    const __m256i index = nearest_index_avx2(rr, cc, w);
    __m256 gxf, gyf;
    if constexpr (Packed) {
      const auto pair = _mm256_i32gather_epi32(reinterpret_cast<const int*>(packed_components), index, 4);
      gxf = _mm256_mul_ps(extend_i16(pair), v_i_scale);
      gyf = _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_srai_epi32(pair, 16)), v_i_scale);
    } else {
      gxf = _mm256_mul_ps(extend_i16(_mm256_i32gather_epi32(gx32, index, 2)), v_i_scale);
      gyf = _mm256_mul_ps(extend_i16(_mm256_i32gather_epi32(gy32, index, 2)), v_i_scale);
    }
    const __m256 mag2 =
        _mm256_add_ps(_mm256_mul_ps(gxf, gxf), _mm256_mul_ps(gyf, gyf));
    const __m256 live = _mm256_cmp_ps(mag2, min_sq, _CMP_GE_OQ);
    const __m256 inv = Cached ? _mm256_i32gather_ps(inverse, index, 4) :
        _mm256_div_ps(one, _mm256_sqrt_ps(_mm256_max_ps(mag2, tiny)));
    const __m256 raw_dot = _mm256_add_ps(
        _mm256_mul_ps(_mm256_loadu_ps(nr + done), gyf),
        _mm256_mul_ps(_mm256_loadu_ps(nc + done), gxf));
    const __m256 contrib = _mm256_and_ps(live, _mm256_mul_ps(raw_dot, inv));
    pos += horizontal_sum_max0(metric == 2 ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), contrib) : contrib);
    if (metric == 1) neg += horizontal_sum_max0(_mm256_sub_ps(_mm256_setzero_ps(), contrib));
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 8), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  for (; done < n; ++done) {
    const float rr = row + off_r[done];
    const float cc = col + off_c[done];
    const auto ri = static_cast<std::size_t>(rr + 0.5F);
    const auto ci = static_cast<std::size_t>(cc + 0.5F);
    const auto index = ri * width + ci;
    const float gxf = integer_scale * static_cast<float>(gx[index]);
    const float gyf = integer_scale * static_cast<float>(gy[index]);
    const float mag2 = gxf * gxf + gyf * gyf;
    if (mag2 >= min_mag_squared) {
      const float dot = (nr[done] * gyf + nc[done] * gxf) / std::sqrt(mag2);
      pos += metric == 2 ? std::abs(dot) : std::max(0.0F, dot);
      if (metric == 1) neg += std::max(0.0F, -dot);
    }
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 1), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  _mm256_zeroupper();
  const float raw = (metric == 1 ? std::max(pos, neg) : pos) / n_f;
  if (raw < 0.0F) return 0.0;
  if (raw > 1.0F) return 1.0;
  return static_cast<double>(raw);
}

double score_i16_pre_rotated_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept {
  return score_i16_pre_rotated_impl<false>(gx, gy, nullptr, nullptr, width, integer_scale,
      min_mag_squared, off_r, off_c, nr, nc, n, row, col, greediness,
      reject_below, metric);
}
double score_i16_pre_rotated_cached_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept {
  return score_i16_pre_rotated_impl<true>(gx, gy, inverse, nullptr, width, integer_scale,
      min_mag_squared, off_r, off_c, nr, nc, n, row, col, greediness,
      reject_below, metric);
}
double score_i16_pre_rotated_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    const std::uint32_t* packed, const float* inverse,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept {
  if (inverse) return score_i16_pre_rotated_impl<true, true>(gx, gy, inverse, packed,
      width, integer_scale, min_mag_squared, off_r, off_c, nr, nc, n, row, col,
      greediness, reject_below, metric);
  return score_i16_pre_rotated_impl<false, true>(gx, gy, nullptr, packed,
      width, integer_scale, min_mag_squared, off_r, off_c, nr, nc, n, row, col,
      greediness, reject_below, metric);
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__)
__attribute__((noinline))
#endif
double score_i16_bilinear_impl(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    const std::uint32_t* __restrict packed_components,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* __restrict prow, const float* __restrict pcol,
    const float* __restrict pnr, const float* __restrict pnc, std::size_t n,
    float row, float col, float ca, float sa, float scale, float greediness,
    float reject_below, int metric) noexcept {
  if (n == 0 || width == 0) return 0.0;
  const float n_f = static_cast<float>(n);
  const float greedy_begin = greedy_prefix(n_f,greediness,reject_below);
  const __m256 vrow = _mm256_set1_ps(row);
  const __m256 vcol = _mm256_set1_ps(col);
  const __m256 vca = _mm256_set1_ps(ca);
  const __m256 vsa = _mm256_set1_ps(sa);
  const __m256 vscale = _mm256_set1_ps(scale);
  const __m256 v_i_scale = _mm256_set1_ps(integer_scale);
  const __m256 min_sq = _mm256_set1_ps(min_mag_squared);
  const __m256 one = _mm256_set1_ps(1.0F);
  const __m256 tiny = _mm256_set1_ps(1.0e-12F);
  const int w = static_cast<int>(width);
  const int* gx32 = reinterpret_cast<const int*>(gx);
  const int* gy32 = reinterpret_cast<const int*>(gy);
  float pos = 0.0F, neg = 0.0F;
  std::size_t done = 0;
  for (; done + 8 <= n; done += 8) {
    const auto q = rotate_query_avx2(
        _mm256_loadu_ps(prow + done), _mm256_loadu_ps(pcol + done),
        _mm256_loadu_ps(pnr + done), _mm256_loadu_ps(pnc + done), vrow, vcol,
        vca, vsa, vscale);
    __m256 gxf, gyf;
    gather_i16_bilinear_score_avx2(gx32, gy32, packed_components, w,
                                 q.rr, q.cc, v_i_scale, &gxf, &gyf);
    const __m256 mag2 =
#if defined(_MSC_VER)
        _mm256_fmadd_ps(gxf, gxf, _mm256_mul_ps(gyf, gyf));
#else
        _mm256_add_ps(_mm256_mul_ps(gxf, gxf), _mm256_mul_ps(gyf, gyf));
#endif
    const __m256 live = _mm256_cmp_ps(mag2, min_sq, _CMP_GE_OQ);
    const __m256 inv =
        _mm256_div_ps(one, _mm256_sqrt_ps(_mm256_max_ps(mag2, tiny)));
    const __m256 raw_dot =
#if defined(_MSC_VER)
        _mm256_fmadd_ps(q.nr, gyf, _mm256_mul_ps(q.nc, gxf));
#else
        _mm256_add_ps(_mm256_mul_ps(q.nr, gyf), _mm256_mul_ps(q.nc, gxf));
#endif
    const __m256 contrib = _mm256_and_ps(live, _mm256_mul_ps(raw_dot, inv));
    pos += horizontal_sum_max0(metric == 2 ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), contrib) : contrib);
    if (metric == 1) neg += horizontal_sum_max0(_mm256_sub_ps(_mm256_setzero_ps(), contrib));
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 8), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  for (; done < n; ++done) {
    const float rot_r = ca * prow[done] - sa * pcol[done];
    const float rot_c = ca * pcol[done] + sa * prow[done];
    const float rr = row + scale * rot_r;
    const float cc = col + scale * rot_c;
    const float nr = ca * pnr[done] - sa * pnc[done];
    const float nc = ca * pnc[done] + sa * pnr[done];
    float gxf, gyf;
    bilinear_i16_scalar(gx, gy, width, integer_scale, rr, cc, &gxf, &gyf);
    const float mag2 = gxf * gxf + gyf * gyf;
    if (mag2 >= min_mag_squared) {
      const float dot = (nr * gyf + nc * gxf) / std::sqrt(mag2);
      pos += metric == 2 ? std::abs(dot) : std::max(0.0F, dot);
      if (metric == 1) neg += std::max(0.0F, -dot);
    }
    if (reject_after(metric == 1 ? std::max(pos, neg) : pos, static_cast<float>(done + 1), n_f, greediness,
                     reject_below, greedy_begin)) {
      _mm256_zeroupper();
      return 0.0;
    }
  }
  _mm256_zeroupper();
  const float raw = (metric == 1 ? std::max(pos, neg) : pos) / n_f;
  if (raw < 0.0F) return 0.0;
  if (raw > 1.0F) return 1.0;
  return static_cast<double>(raw);
}

double score_i16_bilinear_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  return score_i16_bilinear_impl(gx, gy, nullptr, width, integer_scale,
      min_mag_squared, prow, pcol, pnr, pnc, n, row, col, ca, sa, scale,
      greediness, reject_below, metric);
}
double score_i16_bilinear_packed_use_polarity_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  return score_i16_bilinear_impl(gx, gy, packed, width, integer_scale,
      min_mag_squared, prow, pcol, pnr, pnc, n, row, col, ca, sa, scale,
      greediness, reject_below, metric);
}

void xld_geometric_i16_peaks_avx2(const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, const float* row, const float* col,
    const float* nr, const float* nc, const float* base, const float* step,
    const int* span, int maximum_span, float min_squared, bool ignore_polarity,
    float* left, float* center, float* right, int* best_index) noexcept {
  const auto er=_mm256_loadu_ps(row), ec=_mm256_loadu_ps(col);
  const auto vr=_mm256_loadu_ps(nr), vc=_mm256_loadu_ps(nc);
  const auto vb=_mm256_loadu_ps(base), vs=_mm256_loadu_ps(step);
  const auto spans=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(span));
  const auto zero=_mm256_setzero_ps();
  const auto vscale=_mm256_set1_ps(integer_scale), threshold=_mm256_set1_ps(min_squared);
  auto previous=zero, best=zero, bl=zero, br=zero;
  auto bi=_mm256_setzero_si256();
  for (int index=-maximum_span;index<=maximum_span;++index) {
    const auto vi=_mm256_set1_epi32(index);
    // 非活跃 lane 读取有效边界样本，然后贡献零。
    const auto clamped=_mm256_max_epi32(_mm256_sub_epi32(_mm256_setzero_si256(),spans),_mm256_min_epi32(spans,vi));
    const auto distance=_mm256_add_ps(vb,_mm256_mul_ps(_mm256_cvtepi32_ps(clamped),vs));
    const auto rr=_mm256_add_ps(er,_mm256_mul_ps(distance,vr));
    const auto cc=_mm256_add_ps(ec,_mm256_mul_ps(distance,vc));
    __m256 x,y;
    gather_i16_bilinear_avx2(reinterpret_cast<const int*>(gx),reinterpret_cast<const int*>(gy),
                           static_cast<int>(width),rr,cc,vscale,&x,&y);
    const auto square=_mm256_add_ps(_mm256_mul_ps(x,x),_mm256_mul_ps(y,y));
    const auto dot=_mm256_add_ps(_mm256_mul_ps(vr,y),_mm256_mul_ps(vc,x));
    const auto aligned=ignore_polarity?_mm256_andnot_ps(_mm256_set1_ps(-0.0F),dot):_mm256_max_ps(zero,dot);
    const auto live=_mm256_and_ps(_mm256_cmp_ps(square,threshold,_CMP_GE_OQ),
        _mm256_castsi256_ps(_mm256_cmpgt_epi32(spans,_mm256_set1_epi32(std::abs(index)-1))));
    const auto value=_mm256_and_ps(live,_mm256_div_ps(_mm256_mul_ps(aligned,aligned),
        _mm256_sqrt_ps(_mm256_max_ps(square,_mm256_set1_ps(1e-12F)))));
    const auto following=_mm256_castsi256_ps(_mm256_cmpeq_epi32(vi,_mm256_add_epi32(bi,_mm256_set1_epi32(1))));
    br=_mm256_blendv_ps(br,value,following);
    const auto better=_mm256_cmp_ps(value,best,_CMP_GT_OQ);
    best=_mm256_blendv_ps(best,value,better);
    bl=_mm256_blendv_ps(bl,previous,better);
    br=_mm256_blendv_ps(br,zero,better);
    bi=_mm256_blendv_epi8(bi,vi,_mm256_castps_si256(better));
    previous=value;
  }
  _mm256_storeu_ps(left,bl);_mm256_storeu_ps(center,best);_mm256_storeu_ps(right,br);
  _mm256_storeu_si256(reinterpret_cast<__m256i*>(best_index),bi);
  _mm256_zeroupper();
}

template<bool FinishAvx = true>
void geometric_i16_bilinear_line_impl(
    const std::int16_t* __restrict gx, const std::int16_t* __restrict gy,
    const std::uint32_t* __restrict packed_components,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_contrast_squared, float* __restrict responses, bool ignore_polarity) noexcept {
  const int count = 2 * span + 1;
  if (count <= 0 || width == 0) return;
  const __m256 v_i_scale = _mm256_set1_ps(integer_scale);
  const __m256 min_sq = _mm256_set1_ps(min_contrast_squared);
  const __m256 er = _mm256_set1_ps(expected_row);
  const __m256 ec = _mm256_set1_ps(expected_col);
  const __m256 nr = _mm256_set1_ps(normal_row);
  const __m256 nc = _mm256_set1_ps(normal_col);
  const __m256 step = _mm256_set1_ps(search_step);
  const __m256 tiny = _mm256_set1_ps(1.0e-12F);
  const __m256 zero = _mm256_setzero_ps();
  const int w = static_cast<int>(width);
  const int* gx32 = reinterpret_cast<const int*>(gx);
  const int* gy32 = reinterpret_cast<const int*>(gy);
  int done = 0;
  for (; done + 8 <= count; done += 8) {
    // 连续 8 个法向距离：双线性 gather 后算 mag * max(0, cos)^2。
    const __m256 idx = _mm256_add_ps(
        _mm256_set1_ps(static_cast<float>(-span + done)),
        _mm256_setr_ps(0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F));
    const __m256 dist = _mm256_mul_ps(idx, step);
    const __m256 rr = _mm256_add_ps(er, _mm256_mul_ps(dist, nr));
    const __m256 cc = _mm256_add_ps(ec, _mm256_mul_ps(dist, nc));
    __m256 gxf, gyf;
    // 读取方式由指针选择，插值和响应共用实现，避免模板特化改变融合顺序。
    gather_i16_bilinear_score_avx2(gx32, gy32, packed_components, w,
                                 rr, cc, v_i_scale, &gxf, &gyf);
    // 两种存储布局使用同一舍入顺序，避免依赖特定 MSVC 版本的收缩行为。
    const __m256 mag2 =
        _mm256_add_ps(_mm256_mul_ps(gxf, gxf), _mm256_mul_ps(gyf, gyf));
    const __m256 live = _mm256_cmp_ps(mag2, min_sq, _CMP_GE_OQ);
    const __m256 mag =
        _mm256_sqrt_ps(_mm256_max_ps(mag2, tiny));
    const __m256 dot =
        _mm256_add_ps(_mm256_mul_ps(nr, gyf), _mm256_mul_ps(nc, gxf));
    const __m256 orientation = _mm256_div_ps(dot, mag);
    const __m256 aligned = ignore_polarity ? _mm256_andnot_ps(_mm256_set1_ps(-0.0F), orientation) : _mm256_max_ps(zero, orientation);
    const __m256 value =
        _mm256_and_ps(live, _mm256_mul_ps(mag, _mm256_mul_ps(aligned, aligned)));
    _mm256_storeu_ps(responses + done, value);
  }
  for (; done < count; ++done) {
    const float dist = static_cast<float>(-span + done) * search_step;
    const float rr = expected_row + dist * normal_row;
    const float cc = expected_col + dist * normal_col;
    float gxf, gyf;
    bilinear_i16_scalar(gx, gy, width, integer_scale, rr, cc, &gxf, &gyf);
    const float mag2 = gxf * gxf + gyf * gyf;
    if (mag2 < min_contrast_squared) {
      responses[done] = 0.0F;
      continue;
    }
    const float mag = std::sqrt(mag2);
    const float orientation =
        (normal_row * gyf + normal_col * gxf) / std::max(1.0e-12F, mag);
    const float aligned = ignore_polarity ? std::abs(orientation) : orientation > 0.0F ? orientation : 0.0F;
    responses[done] = mag * aligned * aligned;
  }
  if constexpr (FinishAvx) _mm256_zeroupper();
}

void geometric_i16_bilinear_line_avx2(
    const std::int16_t* gx, const std::int16_t* gy,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_contrast_squared, float* responses, bool ignore_polarity) noexcept {
  geometric_i16_bilinear_line_impl<>(gx, gy, nullptr, width, integer_scale,
      expected_row, expected_col, normal_row, normal_col, search_step, span,
      min_contrast_squared, responses, ignore_polarity);
}
void geometric_i16_bilinear_packed_line_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_contrast_squared, float* responses, bool ignore_polarity) noexcept {
  geometric_i16_bilinear_line_impl<>(gx, gy, packed, width, integer_scale,
      expected_row, expected_col, normal_row, normal_col, search_step, span,
      min_contrast_squared, responses, ignore_polarity);
}

static inline GeometricPeak geometric_response_peak_impl(const float* responses, int span) noexcept {
  GeometricPeak out;
  if (span != 8) {
    double previous = 0.0;
    for (int index = -span; index <= span; ++index) {
      const double value = static_cast<double>(responses[index + span]);
      if (index == out.index + 1) out.right = value;
      if (value > out.center) {
        out.center = value;
        out.index = index;
        out.left = previous;
        out.right = 0.0;
      }
      previous = value;
    }
    return out;
  }

  const __m256 zero = _mm256_setzero_ps();
  const __m256 first = _mm256_loadu_ps(responses);
  const __m256 second = _mm256_loadu_ps(responses + 8);
  // 只有有序正值可替代初始 +0；正 IEEE Float 位序与数值序相同，
  // 整数归约不改变响应算术或 NaN 载荷。
  const auto first_bits = _mm256_and_si256(_mm256_castps_si256(first),
      _mm256_castps_si256(_mm256_cmp_ps(first, zero, _CMP_GT_OQ)));
  const auto second_bits = _mm256_and_si256(_mm256_castps_si256(second),
      _mm256_castps_si256(_mm256_cmp_ps(second, zero, _CMP_GT_OQ)));
  const auto lane_max = _mm256_max_epu32(first_bits, second_bits);
  auto maximum = _mm_max_epu32(_mm256_castsi256_si128(lane_max),
                              _mm256_extracti128_si256(lane_max, 1));
  maximum = _mm_max_epu32(maximum, _mm_shuffle_epi32(maximum, _MM_SHUFFLE(1, 0, 3, 2)));
  maximum = _mm_max_epu32(maximum, _mm_shuffle_epi32(maximum, _MM_SHUFFLE(2, 3, 0, 1)));
  const auto maximum_bits = static_cast<std::uint32_t>(_mm_cvtsi128_si32(maximum));
  const double maximum_value = static_cast<double>(std::bit_cast<float>(maximum_bits));
  const double tail = static_cast<double>(responses[16]);
  if (tail > maximum_value) {
    // 标量尾部位于最终搜索边界，没有右邻居。
    out.index = 8;
    out.left = static_cast<double>(responses[15]);
    out.center = tail;
  } else if (maximum_bits != 0) {
    const auto winner = _mm256_set1_epi32(static_cast<int>(maximum_bits));
    const auto first_mask = static_cast<unsigned>(_mm256_movemask_ps(_mm256_castsi256_ps(
        _mm256_cmpeq_epi32(_mm256_castps_si256(first), winner))));
    const auto second_mask = static_cast<unsigned>(_mm256_movemask_ps(_mm256_castsi256_ps(
        _mm256_cmpeq_epi32(_mm256_castps_si256(second), winner))));
    // 最小索引保留原严格大于同值规则，含重复正 Inf 及与标量尾部同值。
    const int absolute_index = first_mask != 0 ? std::countr_zero(first_mask) :
        8 + std::countr_zero(second_mask);
    out.index = absolute_index - 8;
    out.left = absolute_index == 0 ? 0.0 : static_cast<double>(responses[absolute_index - 1]);
    out.center = maximum_value;
    out.right = static_cast<double>(responses[absolute_index + 1]);
  } else {
    // 无正 winner 时 index=0、left=center=+0；原扫描仍给带符号搜索索引 +1 处右邻居赋值。
    out.right = static_cast<double>(responses[9]);
  }
  return out;
}

GeometricPeak geometric_response_peak_avx2(const float* responses, int span) noexcept {
  const auto out = geometric_response_peak_impl(responses, span);
  _mm256_zeroupper();
  return out;
}

GeometricPeak geometric_i16_bilinear_peak17_avx2(
    const std::int16_t* gx, const std::int16_t* gy, std::size_t width,
    float integer_scale, float expected_row, float expected_col,
    float normal_row, float normal_col, float search_step,
    float min_contrast_squared, bool ignore_polarity) noexcept {
  if (width == 0) return {};
  float responses[17];
  geometric_i16_bilinear_line_impl<false>(gx, gy, nullptr, width, integer_scale,
      expected_row, expected_col, normal_row, normal_col, search_step, 8,
      min_contrast_squared, responses, ignore_polarity);
  const auto out = geometric_response_peak_impl(responses, 8);
  _mm256_zeroupper();
  return out;
}

GeometricPeak geometric_i16_bilinear_packed_peak17_avx2(
    const std::int16_t* gx, const std::int16_t* gy, const std::uint32_t* packed,
    std::size_t width, float integer_scale, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    float min_contrast_squared, bool ignore_polarity) noexcept {
  if (width == 0) return {};
  float responses[17];
  geometric_i16_bilinear_line_impl<false>(gx, gy, packed, width, integer_scale,
      expected_row, expected_col, normal_row, normal_col, search_step, 8,
      min_contrast_squared, responses, ignore_polarity);
  const auto out = geometric_response_peak_impl(responses, 8);
  _mm256_zeroupper();
  return out;
}

}  // namespace mv::shape_matching::simd
