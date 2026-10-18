#pragma once
#include <cstddef>
#include <cstdint>
namespace mv::reconstruction {
// 仅编码完整八像素组，全部偏移及中心须在边界内。
std::size_t census_avx2_row(const float* center, std::size_t available,
                            const std::ptrdiff_t* offsets, std::size_t count,
                            std::uint64_t* output);
}  // namespace mv::reconstruction
