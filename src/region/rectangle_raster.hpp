#pragma once

#include <array>
#include <cstdint>
#include <core/region.hpp>

namespace mv::regionproc {
// 输入为既有规则取整后、位于 ±2^30 内的四个角；保持原多边形及矩形边界的并集。
core::Region rectangle_corner_runs(const std::array<std::int64_t,4>& rows,
                                    const std::array<std::int64_t,4>& columns);
}
