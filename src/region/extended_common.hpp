#pragma once
#include <region/operators.hpp>
#include <core/image.hpp>
#include <core/exception.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory_resource>

namespace mv::regionproc::extended {
using namespace mv::core;
using mv::runtime::ExecutionContext;
inline void check(bool condition, const char* op, const char* parameter,
                  const char* message) {
  if (!condition)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    parameter, -1, message);
}
inline std::int64_t add(std::int64_t a, std::int64_t b) {
  if ((b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b))
    throw Exception(Status::OutOfRange, "region transform",
                    "coordinate overflow");
  return a + b;
}
inline std::int64_t sub(std::int64_t a, std::int64_t b) {
  if ((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b))
    throw Exception(Status::OutOfRange, "region transform",
                    "coordinate overflow");
  return a - b;
}
inline void geometry_bounds(const Region& r, const char* op) {
  for (const auto& x : r.runs())
    check(x.row >= -0x40000000LL && x.row <= 0x40000000LL &&
              x.col_begin >= -0x40000000LL && x.col_end <= 0x40000000LL,
          op, "Region",
          "floating geometry requires coordinates within +/-2^30");
}
inline std::size_t extent(std::int64_t lo, std::int64_t hi, const char* op) {
  const auto n = std::uint64_t(hi) - std::uint64_t(lo);
  check(n < std::uint64_t(INT32_MAX), op, "Extent", "extent exceeds INT32_MAX");
  return static_cast<std::size_t>(n + 1);
}
inline constexpr std::uint64_t kOccupancyLimit = 16ull * 1024ull * 1024ull;
inline bool occupancy_fits(std::size_t width, std::size_t height) {
  return height != 0 &&
         width <= static_cast<std::size_t>(kOccupancyLimit / height);
}
inline void dimensions(std::size_t w, std::size_t h, const char* op) {
  check(w > 0 && h > 0 && w <= INT32_MAX - 2 && h <= INT32_MAX - 2, op,
        "Width/Height", "positive dimensions below INT32_MAX-1 required");
  check(w <= std::numeric_limits<std::size_t>::max() / h / sizeof(double), op,
        "Width/Height", "workspace size overflow");
}
template <class F>
Regions map(const Regions& rs, F&& f) {
  ExecutionContext context;
  std::pmr::vector<Region> result(rs.size(), context.scratch_resource());
  std::size_t work = 0;
  for (std::size_t i = 0; i < rs.size(); ++i)
    work += std::min(rs[i].runs().size(), SIZE_MAX - work);
  ExecutionContext::for_each_partition(rs.size(), work, 32768, [&](auto range) {
    for (auto i = range.begin; i < range.end; ++i) result[i] = f(rs[i]);
  });
  return Regions{std::vector<Region>(result.begin(), result.end())};
}
inline Region mask_region(std::span<const unsigned char> mask, std::size_t w,
                          std::size_t h, std::int64_t row0, std::int64_t col0) {
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  for (std::size_t y = 0; y < h; ++y)
    for (std::size_t x = 0; x < w;) {
      if (!mask[y * w + x]) {
        ++x;
        continue;
      }
      const auto begin = x;
      while (x < w && mask[y * w + x]) ++x;
      runs.push_back({add(row0, static_cast<std::int64_t>(y)),
                      add(col0, static_cast<std::int64_t>(begin)),
                      add(col0, static_cast<std::int64_t>(x - 1))});
    }
  return Region::from_runs(runs);
}
}  // namespace mv::regionproc::extended
