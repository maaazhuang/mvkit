#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mv::reconstruction {

inline constexpr std::int32_t kStereoBadCost = 1000000;
inline constexpr std::int32_t kStereoInfinity = 100000000;
inline constexpr std::uint8_t kCensusPackedInvalid = 255;
inline constexpr int kCensusDenseBitCount = 62;
inline constexpr int kCensusSparseBitCount = 24;
inline constexpr int kSadWindowRadius = 2;
inline constexpr int kSgmMaxPaths = 8;
// path <= bad_cost + P2 < infinity；8 路累加和邻居 + P1 均可用 Int32。
inline constexpr std::int32_t kSgmMaxPenalty = kStereoInfinity - kStereoBadCost - 1;

[[nodiscard]] constexpr std::int32_t unpack_census_cost(std::uint8_t packed) {
  return packed == kCensusPackedInvalid ? kStereoBadCost : static_cast<std::int32_t>(packed);
}

inline void evaluate_sgm_path(const std::int32_t* cost, const std::int32_t* previous,
                              std::int32_t* current, std::size_t disparities, std::int32_t p1,
                              std::int32_t p2, bool has_previous) {
  if (!has_previous) {
    for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
      current[disparity] = cost[disparity];
    }
    return;
  }
  std::int32_t previous_minimum = kStereoInfinity;
  for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
    previous_minimum = std::min(previous_minimum, previous[disparity]);
  }
  for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
    const auto same = previous[disparity];
    const auto lower = disparity != 0 ? previous[disparity - 1] + p1 : kStereoInfinity;
    const auto upper =
        disparity + 1 < disparities ? previous[disparity + 1] + p1 : kStereoInfinity;
    const auto jump = previous_minimum + p2;
    const auto best = std::min(std::min(same, lower), std::min(upper, jump));
    current[disparity] = cost[disparity] + best - previous_minimum;
  }
}

// 路径遍历已知前像素最小值，评估标签时同时返回新最小值，避免逐像素再次扫描前路径。
inline std::int32_t evaluate_sgm_path_cached_min(
    const std::int32_t* cost, const std::int32_t* previous, std::int32_t* current,
    std::size_t disparities, std::int32_t p1, std::int32_t p2,
    bool has_previous, std::int32_t previous_minimum) {
  std::int32_t current_minimum = kStereoInfinity;
  if (!has_previous) {
    for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
      const auto value = cost[disparity];
      current[disparity] = value;
      current_minimum = std::min(current_minimum, value);
    }
    return current_minimum;
  }
  const auto jump = previous_minimum + p2;
  for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
    const auto same = previous[disparity];
    const auto lower = disparity != 0 ? previous[disparity - 1] + p1 : kStereoInfinity;
    const auto upper = disparity + 1 < disparities ? previous[disparity + 1] + p1
                                                   : kStereoInfinity;
    const auto best = std::min(std::min(same, lower), std::min(upper, jump));
    const auto value = cost[disparity] + best - previous_minimum;
    current[disparity] = value;
    current_minimum = std::min(current_minimum, value);
  }
  return current_minimum;
}

}  // namespace mv::reconstruction
