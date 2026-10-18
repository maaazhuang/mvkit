#pragma once

#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <span>
#include <vector>

#include <reconstruction/stereo/cost_volume.hpp>

namespace mv::reconstruction {

[[nodiscard]] std::pmr::vector<std::int32_t> aggregate_sgm(
    std::span<const std::int32_t> cost, std::size_t width,
    std::size_t height, std::size_t disparities, std::int32_t p1,
    std::int32_t p2, int paths, std::pmr::memory_resource* memory);
[[nodiscard]] std::pmr::vector<std::int32_t> aggregate_sgm(
    std::span<const std::uint8_t> packed_cost, std::size_t width,
    std::size_t height, std::size_t disparities, std::int32_t p1,
    std::int32_t p2, int paths, std::pmr::memory_resource* memory);
[[nodiscard]] CostVolume aggregate_sgm(const CostVolume& cost, std::int32_t p1, std::int32_t p2,
                                       int paths, std::pmr::memory_resource* memory);

}  // namespace mv::reconstruction
