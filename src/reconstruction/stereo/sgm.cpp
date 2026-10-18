#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <reconstruction/stereo/cost_semantics.hpp>
#include <reconstruction/stereo/cost_volume.hpp>
#include <reconstruction/stereo/sgm.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <span>
#include <utility>
#include <vector>

#include "sgm_simd.hpp"

namespace mv::reconstruction {
namespace {

void unpack_pixel_cost(const CostVolume& cost, std::size_t pixel, std::int32_t* out) {
  const std::size_t disparities = cost.disparities;
  if (!cost.is_banded) {
    const std::size_t base = pixel * disparities;
    if (!cost.census.empty()) {
      for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
        out[disparity] = unpack_census_cost(cost.census[base + disparity]);
      }
      return;
    }
    for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
      out[disparity] = cost.labels[base + disparity];
    }
    return;
  }
  for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
    out[disparity] = kStereoBadCost;
  }
  if (pixel >= cost.bands.size()) return;
  const PixelBand& band = cost.bands[pixel];
  for (std::uint32_t local = 0; local < band.count; ++local) {
    const std::size_t index = static_cast<std::size_t>(band.offset) + local;
    const std::size_t disparity = static_cast<std::size_t>(band.base) + local;
    if (!cost.census.empty()) {
      out[disparity] = unpack_census_cost(cost.census[index]);
    } else {
      out[disparity] = cost.labels[index];
    }
  }
}

void add_path_labels(CostVolume& aggregate, std::size_t pixel, const std::int32_t* path) {
  const std::size_t disparities = aggregate.disparities;
  if (!aggregate.is_banded) {
    const std::size_t base = pixel * disparities;
    for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
      aggregate.labels[base + disparity] += path[disparity];
    }
    return;
  }
  const PixelBand& band = aggregate.bands[pixel];
  for (std::uint32_t local = 0; local < band.count; ++local) {
    aggregate.labels[static_cast<std::size_t>(band.offset) + local] +=
        path[static_cast<std::size_t>(band.base) + local];
  }
}

template <bool fused, class PixelCost, class Accumulate, class Evaluate>
void aggregate_paths_impl(PixelCost&& pixel_cost, Accumulate&& accumulate, std::size_t width,
                          std::size_t height, std::size_t disparities, std::int32_t p1, std::int32_t p2,
                          int paths, std::pmr::memory_resource* memory, Evaluate&& evaluate,
                          std::int32_t* dense_aggregate) {
  std::pmr::vector<std::int32_t> previous(disparities, 0, memory);
  std::pmr::vector<std::int32_t> current(disparities, 0, memory);
  constexpr std::array<std::pair<int, int>, 8> directions{{
      {0, 1},
      {0, -1},
      {1, 0},
      {-1, 0},
      {1, 1},
      {1, -1},
      {-1, 1},
      {-1, -1},
  }};
  const int path_count = std::min(paths, kSgmMaxPaths);
  const int width_i = static_cast<int>(width);
  const int height_i = static_cast<int>(height);
  for (int path_index = 0; path_index < path_count; ++path_index) {
    const auto [row_delta, column_delta] = directions[path_index];
    for (int row = 0; row < height_i; ++row) {
      for (int column = 0; column < width_i; ++column) {
        const int previous_row = row - row_delta;
        const int previous_column = column - column_delta;
        if (previous_row >= 0 && previous_column >= 0 && previous_row < height_i &&
            previous_column < width_i) {
          continue;
        }
        bool has_previous = false;
        std::int32_t previous_minimum = kStereoInfinity;
        int walk_row = row;
        int walk_column = column;
        while (walk_row >= 0 && walk_column >= 0 && walk_row < height_i && walk_column < width_i) {
          const auto pixel =
              static_cast<std::size_t>(walk_row) * width + static_cast<std::size_t>(walk_column);
          const std::int32_t* cost = pixel_cost(pixel);
          previous_minimum =
              evaluate(cost, previous.data(), current.data(), disparities, p1, p2, has_previous,
                       previous_minimum, fused ? dense_aggregate + pixel * disparities : nullptr);
          if constexpr (!fused) accumulate(pixel, current.data());
          previous.swap(current);
          has_previous = true;
          walk_row += row_delta;
          walk_column += column_delta;
        }
      }
    }
  }
}

template <class PixelCost, class Accumulate>
void aggregate_paths(PixelCost&& pixel_cost, Accumulate&& accumulate, std::size_t width, std::size_t height,
                     std::size_t disparities, std::int32_t p1, std::int32_t p2, int paths,
                     std::pmr::memory_resource* memory, std::int32_t* dense_aggregate = nullptr) {
  // 每次拥有型聚合只选择一次，放在像素/标签循环外。
  const auto selection = mv::runtime::cpu_features().selection();
  SgmPathKernel kernel = nullptr;
#if MVKIT_RECONSTRUCTION_HAS_AVX2
  if (disparities >= 16 && selection.avx2) kernel = &sgm_path_avx2;
#endif
  if (!kernel && disparities >= 8 && selection.sse2) kernel = &sgm_path_sse2;
  if (kernel) {
    const auto evaluate = [=](const std::int32_t* c, const std::int32_t* p, std::int32_t* out, std::size_t n,
                              std::int32_t a, std::int32_t b, bool have, std::int32_t old,
                              std::int32_t* aggregate) {
      return kernel(c, p, out, n, a, b, have, old, aggregate);
    };
    if (dense_aggregate) {
      aggregate_paths_impl<true>(pixel_cost, accumulate, width, height, disparities, p1, p2, paths, memory,
                                 evaluate, dense_aggregate);
      return;
    }
    aggregate_paths_impl<false>(pixel_cost, accumulate, width, height, disparities, p1, p2, paths, memory,
                                evaluate, nullptr);
    return;
  }
  const auto scalar = [](const std::int32_t* c, const std::int32_t* p, std::int32_t* out, std::size_t n,
                         std::int32_t a, std::int32_t b, bool have, std::int32_t old, std::int32_t*) {
    return evaluate_sgm_path_cached_min(c, p, out, n, a, b, have, old);
  };
  aggregate_paths_impl<false>(pixel_cost, accumulate, width, height, disparities, p1, p2, paths, memory,
                              scalar, nullptr);
}

void copy_volume_geometry(const CostVolume& cost, CostVolume& out) {
  out.width = cost.width;
  out.height = cost.height;
  out.disparities = cost.disparities;
  out.is_banded = cost.is_banded;
  out.bands.assign(cost.bands.begin(), cost.bands.end());
}

// 常见惩罚保留 Int32/SIMD；较大 HALCON 控制使用 Int64 前驱算术，
// 仅最终非负和饱和到 Int32 最大值，该值高于全部有效 winner 阈值。
CostVolume aggregate_wide(const CostVolume& cost, std::int32_t p1, std::int32_t p2, int paths,
                          std::pmr::memory_resource* memory) {
  CostVolume out{memory};
  copy_volume_geometry(cost, out);
  out.labels.assign(cost.stored_label_count(), 0);
  std::pmr::vector<std::int64_t> previous(cost.disparities, 0, memory), current(cost.disparities, 0, memory);
  constexpr std::array<std::pair<int, int>, 8> directions{
      {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  const auto width = static_cast<int>(cost.width), height = static_cast<int>(cost.height);
  constexpr auto infinity = (std::numeric_limits<std::int64_t>::max)() / 4;
  for (int path = 0; path < (std::min)(paths, kSgmMaxPaths); ++path) {
    const auto [dr, dc] = directions[path];
    for (int r = 0; r < height; ++r)
      for (int c = 0; c < width; ++c) {
        if (r - dr >= 0 && r - dr < height && c - dc >= 0 && c - dc < width) continue;
        bool have_previous = false;
        std::int64_t old_min = infinity;
        for (int row = r, col = c; row >= 0 && row < height && col >= 0 && col < width;
             row += dr, col += dc) {
          const auto pixel = std::size_t(row) * cost.width + std::size_t(col);
          auto new_min = infinity;
          for (std::size_t d = 0; d < cost.disparities; ++d) {
            std::int64_t value = cost.unpacked_cost(pixel, d);
            if (have_previous) {
              const auto same = previous[d];
              const auto lower = d ? previous[d - 1] + std::int64_t(p1) : infinity;
              const auto upper = d + 1 < cost.disparities ? previous[d + 1] + std::int64_t(p1) : infinity;
              const auto jump = old_min + std::int64_t(p2);
              value += (std::min)({same, lower, upper, jump}) - old_min;
            }
            current[d] = value;
            new_min = (std::min)(new_min, value);
            if (cost.contains(pixel, d)) {
              const auto index = cost.is_banded ? cost.bands[pixel].offset + d - cost.bands[pixel].base
                                                : pixel * cost.disparities + d;
              const auto sum = std::int64_t(out.labels[index]) + value;
              out.labels[index] = static_cast<std::int32_t>((std::min)(sum, std::int64_t(INT32_MAX)));
            }
          }
          previous.swap(current);
          old_min = new_min;
          have_previous = true;
        }
      }
  }
  return out;
}

}  // namespace

std::pmr::vector<std::int32_t> aggregate_sgm(std::span<const std::int32_t> cost, std::size_t width,
                                             std::size_t height, std::size_t disparities, std::int32_t p1,
                                             std::int32_t p2, int paths, std::pmr::memory_resource* memory) {
  if (paths <= 0) {
    return std::pmr::vector<std::int32_t>(cost.begin(), cost.end(), memory);
  }
  if (p1 > kSgmMaxPenalty || p2 > kSgmMaxPenalty) {
    CostVolume volume{memory};
    volume.width = width;
    volume.height = height;
    volume.disparities = disparities;
    volume.labels.assign(cost.begin(), cost.end());
    return aggregate_wide(volume, p1, p2, paths, memory).labels;
  }
  std::pmr::vector<std::int32_t> aggregate(cost.size(), 0, memory);
  aggregate_paths([&](std::size_t pixel) { return cost.data() + pixel * disparities; },
                  [&](std::size_t pixel, const std::int32_t* path) {
                    std::int32_t* dest = aggregate.data() + pixel * disparities;
                    for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
                      dest[disparity] += path[disparity];
                    }
                  },
                  width, height, disparities, p1, p2, paths, memory, aggregate.data());
  return aggregate;
}

std::pmr::vector<std::int32_t> aggregate_sgm(std::span<const std::uint8_t> packed_cost, std::size_t width,
                                             std::size_t height, std::size_t disparities, std::int32_t p1,
                                             std::int32_t p2, int paths, std::pmr::memory_resource* memory) {
  if (paths <= 0) {
    std::pmr::vector<std::int32_t> unpacked(packed_cost.size(), 0, memory);
    for (std::size_t i = 0; i < packed_cost.size(); ++i) {
      unpacked[i] = unpack_census_cost(packed_cost[i]);
    }
    return unpacked;
  }
  if (p1 > kSgmMaxPenalty || p2 > kSgmMaxPenalty) {
    CostVolume volume{memory};
    volume.width = width;
    volume.height = height;
    volume.disparities = disparities;
    volume.census.assign(packed_cost.begin(), packed_cost.end());
    return aggregate_wide(volume, p1, p2, paths, memory).labels;
  }
  std::pmr::vector<std::int32_t> aggregate(packed_cost.size(), 0, memory);
  std::pmr::vector<std::int32_t> unpacked_pixel(disparities, 0, memory);
  aggregate_paths(
      [&](std::size_t pixel) {
        const std::size_t base = pixel * disparities;
        for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
          unpacked_pixel[disparity] = unpack_census_cost(packed_cost[base + disparity]);
        }
        return unpacked_pixel.data();
      },
      [&](std::size_t pixel, const std::int32_t* path) {
        std::int32_t* dest = aggregate.data() + pixel * disparities;
        for (std::size_t disparity = 0; disparity < disparities; ++disparity) {
          dest[disparity] += path[disparity];
        }
      },
      width, height, disparities, p1, p2, paths, memory, aggregate.data());
  return aggregate;
}

CostVolume aggregate_sgm(const CostVolume& cost, std::int32_t p1, std::int32_t p2, int paths,
                         std::pmr::memory_resource* memory) {
  if (paths > 0 && (p1 > kSgmMaxPenalty || p2 > kSgmMaxPenalty))
    return aggregate_wide(cost, p1, p2, paths, memory);
  CostVolume out{memory};
  copy_volume_geometry(cost, out);
  const std::size_t stored = cost.stored_label_count();
  if (paths <= 0) {
    out.labels.resize(stored);
    if (!cost.census.empty()) {
      for (std::size_t index = 0; index < stored; ++index) {
        out.labels[index] = unpack_census_cost(cost.census[index]);
      }
    } else {
      out.labels.assign(cost.labels.begin(), cost.labels.end());
    }
    return out;
  }
  if (!cost.is_banded) {
    if (!cost.census.empty()) {
      auto dense = aggregate_sgm(std::span<const std::uint8_t>{cost.census.data(), cost.census.size()},
                                 cost.width, cost.height, cost.disparities, p1, p2, paths, memory);
      out.labels = std::move(dense);
    } else {
      auto dense = aggregate_sgm(std::span<const std::int32_t>{cost.labels.data(), cost.labels.size()},
                                 cost.width, cost.height, cost.disparities, p1, p2, paths, memory);
      out.labels = std::move(dense);
    }
    return out;
  }
  out.labels.assign(stored, 0);
  std::pmr::vector<std::int32_t> unpacked_pixel(cost.disparities, 0, memory);
  aggregate_paths(
      [&](std::size_t pixel) {
        unpack_pixel_cost(cost, pixel, unpacked_pixel.data());
        return unpacked_pixel.data();
      },
      [&](std::size_t pixel, const std::int32_t* path) { add_path_labels(out, pixel, path); }, cost.width,
      cost.height, cost.disparities, p1, p2, paths, memory);
  return out;
}

}  // namespace mv::reconstruction
