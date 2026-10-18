#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <reconstruction/stereo/cost_volume.hpp>
namespace mv::reconstruction {
struct StereoWinner {
  std::size_t index{};
  std::int32_t cost{kStereoBadCost}, second{kStereoBadCost};
};
using DenseWinnerKernel = StereoWinner (*)(const std::int32_t*, std::size_t);
StereoWinner dense_winner_sse2(const std::int32_t*, std::size_t);
StereoWinner dense_winner_avx2(const std::int32_t*, std::size_t);
inline void update_winner(StereoWinner& out, std::size_t index, std::int32_t candidate) {
  if (candidate < out.cost) {
    out.second = out.cost;
    out.cost = candidate;
    out.index = index;
  } else
    out.second = (std::min)(out.second, candidate);
}
inline StereoWinner general_winner(const CostVolume& labels, std::size_t pixel) {
  StereoWinner out{0, labels.unpacked_cost(pixel, 0), kStereoBadCost};
  for (std::size_t i = 1; i < labels.disparities; ++i)
    update_winner(out, i, labels.unpacked_cost(pixel, i));
  return out;
}
template <class Read>
inline StereoWinner ordered_winner(Read&& read, std::size_t n) {
  if (!n) return {};
  StereoWinner out{0, read(0), kStereoBadCost};
  for (std::size_t i = 1; i < n; ++i) update_winner(out, i, read(i));
  return out;
}
template <class Read>
inline StereoWinner band_winner(Read&& read, std::size_t n, std::size_t base, std::size_t count) {
  if (!n || !count || base >= n) return {};
  // 无前缀时，原 label-zero 状态不变。
  StereoWinner out{0, base ? kStereoBadCost : read(0), kStereoBadCost};
  const auto end = base + count;
  for (std::size_t i = (std::max)(std::size_t(1), base); i < end; ++i)
    update_winner(out, i, read(i - base));
  // 重复哨兵后缀最多两次原序更新后稳定。
  const auto suffix = (std::max)(std::size_t(1), end);
  if (suffix < n) {
    update_winner(out, suffix, kStereoBadCost);
    if (n - suffix > 1) update_winner(out, suffix + 1, kStereoBadCost);
  }
  return out;
}
inline StereoWinner select_stereo_winner(const CostVolume& labels, std::size_t pixel,
                                         DenseWinnerKernel kernel = nullptr) {
  const auto n = labels.disparities;
  if (!n) return {};
  const bool packed = !labels.census.empty();
  const auto stored = packed ? labels.census.size() : labels.labels.size();
  std::size_t offset = 0, base = 0, count = n;
  if (labels.is_banded) {
    if (pixel >= labels.bands.size()) return {};
    const auto& band = labels.bands[pixel];
    offset = band.offset;
    base = band.base;
    count = band.count;
    if (!count || base >= n) return {};
    if (count > n - base) return general_winner(labels, pixel);
  } else {
    if (pixel > stored / n) return general_winner(labels, pixel);
    offset = pixel * n;
  }
  if (offset > stored || count > stored - offset) return general_winner(labels, pixel);
  if (packed) {
    const auto* p = labels.census.data() + offset;
    const auto read = [&](std::size_t i) { return unpack_census_cost(p[i]); };
    return labels.is_banded ? band_winner(read, n, base, count) : ordered_winner(read, n);
  }
  const auto* p = labels.labels.data() + offset;
  if (labels.is_banded) return band_winner([&](std::size_t i) { return p[i]; }, n, base, count);
  return kernel && n >= 32 ? kernel(p, n) : ordered_winner([&](std::size_t i) { return p[i]; }, n);
}
}  // namespace mv::reconstruction
