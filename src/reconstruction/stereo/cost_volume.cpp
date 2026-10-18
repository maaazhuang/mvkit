#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <reconstruction/stereo/census.hpp>
#include <reconstruction/stereo/cost_semantics.hpp>
#include <reconstruction/stereo/cost_volume.hpp>
#include <vector>

namespace mv::reconstruction {
namespace {

float prior_float(const core::Image& image, std::size_t row, std::size_t column) {
  return reinterpret_cast<const float*>(static_cast<const std::byte*>(image.data()) +
                                        row * image.stride())[column];
}

}  // namespace

bool disparity_label_allowed(const core::Image* coarse_prior, int disparity_offset, std::size_t row,
                             std::size_t column, int disparity) {
  if (coarse_prior == nullptr) return true;
  const auto prior_row = (std::min)(row / 2, coarse_prior->height() - 1);
  const auto prior_column = (std::min)(column / 2, coarse_prior->width() - 1);
  const float coarse = prior_float(*coarse_prior, prior_row, prior_column);
  if (!std::isfinite(coarse)) return true;
  return std::abs(static_cast<double>(disparity) - 2.0 * coarse) <= disparity_offset;
}

PixelBand pixel_disparity_band(const core::Image* coarse_prior, int disparity_offset,
                               std::size_t row, std::size_t column, int min_disparity,
                               int max_disparity) {
  PixelBand band{};
  if (min_disparity > max_disparity) return band;
  const auto disparities = max_disparity - min_disparity + 1;
  if (!coarse_prior) {
    band.count = disparities;
    return band;
  }
  const auto row_index = (std::min)(row / 2, coarse_prior->height() - 1);
  const auto column_index = (std::min)(column / 2, coarse_prior->width() - 1);
  const float coarse = prior_float(*coarse_prior, row_index, column_index);
  if (!std::isfinite(coarse)) {
    band.count = disparities;
    return band;
  }
  if (disparity_offset < 0) return band;
  // 原舍入 Double 差沿整数标签单调；有限 prior 时 abs(delta)<=offset 等价于 -offset<=delta<=offset。
  const auto delta = [&](int index) {
    return static_cast<double>(min_disparity + index) - 2.0 * coarse;
  };
  int first = 0, end = disparities;
  while (first < end) {
    const int middle = first + (end - first) / 2;
    if (delta(middle) < -static_cast<double>(disparity_offset))
      first = middle + 1;
    else
      end = middle;
  }
  int last = first;
  end = disparities;
  while (last < end) {
    const int middle = last + (end - last) / 2;
    if (delta(middle) <= static_cast<double>(disparity_offset))
      last = middle + 1;
    else
      end = middle;
  }
  if (first == last) return band;
  band.base = first;
  band.count = last - first;
  return band;
}

CostVolume build_cost_volume(const core::Image& left, const core::Image& right, int min_disparity,
                             int max_disparity, StereoSimilarity similarity,
                             const core::Image* coarse_prior, int disparity_offset,
                             std::pmr::memory_resource* memory, CostStorage storage) {
  CostVolume volume{memory};
  volume.width = left.width();
  volume.height = left.height();
  volume.disparities = static_cast<std::size_t>(max_disparity - min_disparity + 1);
  const std::size_t width = volume.width;
  const std::size_t height = volume.height;
  const std::size_t disparities = volume.disparities;
  const std::size_t right_width = right.width();
  const std::size_t pixels = width * height;
  const bool sad = similarity == StereoSimilarity::Sad;

  std::pmr::vector<PixelBand> bands{memory};
  bool any_narrow = false;
  std::size_t stored = 0;
  if (coarse_prior != nullptr) {
    bands.resize(pixels);
    for (std::size_t row = 0; row < height; ++row) {
      for (std::size_t column = 0; column < width; ++column) {
        const std::size_t pixel = row * width + column;
        PixelBand band = pixel_disparity_band(coarse_prior, disparity_offset, row, column,
                                              min_disparity, max_disparity);
        if (band.count != disparities || band.base != 0) any_narrow = true;
        band.offset = stored;
        stored += static_cast<std::size_t>(band.count);
        bands[pixel] = band;
      }
    }
  }
  volume.is_banded = storage == CostStorage::Auto && any_narrow;
  if (volume.is_banded) {
    volume.bands = std::move(bands);
  } else {
    stored = pixels * disparities;
    bands.clear();
  }

  if (sad) {
    volume.labels.assign(stored, kStereoBadCost);
  } else {
    volume.census.assign(stored, kCensusPackedInvalid);
  }

  const auto write_sad = [&](std::size_t pixel, std::size_t disparity_index, std::int32_t value) {
    if (volume.is_banded) {
      const PixelBand& band = volume.bands[pixel];
      if (disparity_index < band.base ||
          disparity_index >= static_cast<std::size_t>(band.base) + band.count) {
        return;
      }
      volume.labels[static_cast<std::size_t>(band.offset) + (disparity_index - band.base)] = value;
      return;
    }
    volume.labels[pixel * disparities + disparity_index] = value;
  };
  const auto write_census = [&](std::size_t pixel, std::size_t disparity_index,
                                std::uint8_t value) {
    if (volume.is_banded) {
      const PixelBand& band = volume.bands[pixel];
      if (disparity_index < band.base ||
          disparity_index >= static_cast<std::size_t>(band.base) + band.count) {
        return;
      }
      volume.census[static_cast<std::size_t>(band.offset) + (disparity_index - band.base)] = value;
      return;
    }
    volume.census[pixel * disparities + disparity_index] = value;
  };

  const auto visit_labels = [&](std::size_t row, std::size_t column, auto&& body) {
    const std::size_t pixel = row * width + column;
    const std::int64_t radius = sad ? kSadWindowRadius : 0;
    const auto first_disparity =
        (std::max)(std::int64_t(min_disparity), radius - std::int64_t(column));
    const auto last_disparity =
        (std::min)(std::int64_t(max_disparity),
                   std::int64_t(right_width) - 1 - radius - std::int64_t(column));
    if (first_disparity > last_disparity) return;
    if (volume.is_banded) {
      const PixelBand& band = volume.bands[pixel];
      const auto first = (std::max)(std::int64_t(band.base), first_disparity - min_disparity);
      const auto last =
          (std::min)(std::int64_t(band.base) + band.count - 1, last_disparity - min_disparity);
      for (auto index = first; index <= last; ++index) {
        body(pixel, static_cast<std::size_t>(index), min_disparity + static_cast<int>(index));
      }
      return;
    }
    float coarse = 0.0f;
    bool finite_prior = false;
    if (coarse_prior) {
      coarse = prior_float(*coarse_prior, (std::min)(row / 2, coarse_prior->height() - 1),
                           (std::min)(column / 2, coarse_prior->width() - 1));
      finite_prior = std::isfinite(coarse);
    }
    for (int disparity = static_cast<int>(first_disparity);
         disparity <= static_cast<int>(last_disparity); ++disparity) {
      if (finite_prior &&
          !(std::abs(static_cast<double>(disparity) - 2.0 * coarse) <= disparity_offset))
        continue;
      body(pixel, static_cast<std::size_t>(disparity - min_disparity), disparity);
    }
  };

  if (sad) {
    const auto a = image_to_scalar(left, memory);
    const auto b = image_to_scalar(right, memory);
    constexpr int radius = kSadWindowRadius;
    for (std::size_t row = radius; row + radius < height; ++row) {
      for (std::size_t column = radius; column + radius < width; ++column) {
        visit_labels(
            row, column, [&](std::size_t pixel, std::size_t disparity_index, int disparity) {
              const int right_column = static_cast<int>(column) + disparity;
              if (right_column - radius < 0 ||
                  right_column + radius >= static_cast<int>(right_width)) {
                return;
              }
              double sum = 0.0;
              for (int dr = -radius; dr <= radius; ++dr) {
                for (int dc = -radius; dc <= radius; ++dc) {
                  const auto left_index = (row + dr) * width + (column + dc);
                  const auto right_index =
                      (row + dr) * right_width + static_cast<std::size_t>(right_column + dc);
                  sum += std::abs(a[left_index] - b[right_index]);
                }
              }
              // 超过无效哨兵的代价不可能成为有效 winner，先截断再转整数。
              const auto value = std::isfinite(sum) && sum < kStereoBadCost
                                     ? static_cast<std::int32_t>(std::lround(sum))
                                     : kStereoBadCost;
              write_sad(pixel, disparity_index, value);
            });
      }
    }
    return volume;
  }

  const auto left_census = similarity == StereoSimilarity::CensusDense
                               ? census_dense(left, memory)
                               : census_sparse(left, memory);
  const auto right_census = similarity == StereoSimilarity::CensusDense
                                ? census_dense(right, memory)
                                : census_sparse(right, memory);
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      const auto left_index = row * width + column;
      visit_labels(row, column, [&](std::size_t pixel, std::size_t disparity_index, int disparity) {
        const int right_column = static_cast<int>(column) + disparity;
        if (right_column < 0 || right_column >= static_cast<int>(right_width)) return;
        const auto right_index = row * right_width + static_cast<std::size_t>(right_column);
        write_census(pixel, disparity_index,
                     static_cast<std::uint8_t>(std::popcount(left_census.bits[left_index] ^
                                                             right_census.bits[right_index])));
      });
    }
  }
  return volume;
}

}  // namespace mv::reconstruction
