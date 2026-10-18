#pragma once

#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <vector>

#include <core/image.hpp>
#include <reconstruction/stereo/cost_semantics.hpp>

namespace mv::reconstruction {

enum class StereoSimilarity { CensusDense, CensusSparse, Sad };

// Auto：有限 prior 确实缩窄任一像素时用 banded，否则 dense W×H×D。
// Dense 始终 W×H×D，禁止标签保持无效哨兵，供 dense+allowed oracle 使用。
enum class CostStorage { Auto, Dense };

struct PixelBand {
  std::size_t offset = 0;
  std::uint32_t base = 0;
  std::uint32_t count = 0;
};

struct CostVolume {
  std::size_t width = 0;
  std::size_t height = 0;
  std::size_t disparities = 0;
  bool is_banded = false;
  std::pmr::vector<PixelBand> bands;
  std::pmr::vector<std::uint8_t> census;
  std::pmr::vector<std::int32_t> labels;

  explicit CostVolume(std::pmr::memory_resource* memory = std::pmr::new_delete_resource())
      : bands(memory), census(memory), labels(memory) {}

  [[nodiscard]] std::size_t pixel_count() const { return width * height; }

  [[nodiscard]] std::size_t stored_label_count() const {
    if (!is_banded) return pixel_count() * disparities;
    if (bands.empty()) return 0;
    const PixelBand& last = bands.back();
    return static_cast<std::size_t>(last.offset) + static_cast<std::size_t>(last.count);
  }

  [[nodiscard]] bool contains(std::size_t pixel, std::size_t disparity) const {
    if (disparity >= disparities) return false;
    if (!is_banded) return true;
    if (pixel >= bands.size()) return false;
    const PixelBand& band = bands[pixel];
    return disparity >= static_cast<std::size_t>(band.base) &&
           disparity < static_cast<std::size_t>(band.base) + static_cast<std::size_t>(band.count);
  }

  [[nodiscard]] std::int32_t unpacked_cost(std::size_t pixel, std::size_t disparity) const {
    if (disparity >= disparities) return kStereoBadCost;
    std::size_t index = 0;
    if (is_banded) {
      if (pixel >= bands.size()) return kStereoBadCost;
      const PixelBand& band = bands[pixel];
      if (disparity < static_cast<std::size_t>(band.base) ||
          disparity >= static_cast<std::size_t>(band.base) + static_cast<std::size_t>(band.count)) {
        return kStereoBadCost;
      }
      index = static_cast<std::size_t>(band.offset) +
              (disparity - static_cast<std::size_t>(band.base));
    } else {
      index = pixel * disparities + disparity;
    }
    if (!census.empty()) {
      return index < census.size() ? unpack_census_cost(census[index]) : kStereoBadCost;
    }
    return index < labels.size() ? labels[index] : kStereoBadCost;
  }
};

[[nodiscard]] bool disparity_label_allowed(const core::Image* coarse_prior, int disparity_offset,
                                           std::size_t row, std::size_t column, int disparity);

[[nodiscard]] PixelBand pixel_disparity_band(const core::Image* coarse_prior, int disparity_offset,
                                             std::size_t row, std::size_t column, int min_disparity,
                                             int max_disparity);

[[nodiscard]] CostVolume build_cost_volume(const core::Image& left, const core::Image& right,
                                           int min_disparity, int max_disparity,
                                           StereoSimilarity similarity,
                                           const core::Image* coarse_prior, int disparity_offset,
                                           std::pmr::memory_resource* memory,
                                           CostStorage storage = CostStorage::Auto);

}  // namespace mv::reconstruction
