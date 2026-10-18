#pragma once

#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <vector>

#include <core/image.hpp>

namespace mv::reconstruction {

struct CensusImage {
  CensusImage(std::size_t image_width, std::size_t image_height,
              std::pmr::memory_resource* memory);

  std::size_t width{};
  std::size_t height{};
  std::pmr::vector<std::uint64_t> bits;
  std::pmr::vector<std::uint8_t> valid;
};

[[nodiscard]] std::pmr::vector<float> image_to_scalar(
    const core::Image& image, std::pmr::memory_resource* memory);
[[nodiscard]] CensusImage census_dense(const core::Image& image,
                                       std::pmr::memory_resource* memory);
[[nodiscard]] CensusImage census_sparse(const core::Image& image,
                                        std::pmr::memory_resource* memory);

}  // namespace mv::reconstruction
