#include <region/operators.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory_resource>
#include <core/exception.hpp>
#include <runtime/execution/execution_context.hpp>

namespace mv::regionproc {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
template <class F>
Regions map_regions(const Regions& input, F&& f) {
  ExecutionContext context;
  std::pmr::vector<Region> result(input.size(), context.scratch_resource());
  std::size_t work = 0;
  for (std::size_t i = 0; i < input.size(); ++i)
    work += std::min(input[i].runs().size(),
                     std::numeric_limits<std::size_t>::max() - work);
  ExecutionContext::for_each_partition(
      input.size(), work, 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) result[i] = f(input[i]);
      });
  return Regions{std::vector<Region>(result.begin(), result.end())};
}
std::int64_t coordinate(const Tuple& values, std::size_t i,
                        const char* parameter) {
  const auto value = values.at(i);
  if (value.kind() == ValueKind::Int64) return value.as_int64();
  if (value.kind() != ValueKind::Real64)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "test_region_point", parameter,
                    static_cast<std::int64_t>(i), "coordinate must be numeric");
  const auto rounded = std::round(value.as_real64());
  // 上限不包含端点：double(INT64_MAX) 向上舍入为 2^63。
  if (!std::isfinite(rounded) || rounded < -0x1p63 || rounded >= 0x1p63)
    throw Exception(Status::OutOfRange, ErrorCode::InvalidValue,
                    "test_region_point", parameter,
                    static_cast<std::int64_t>(i),
                    "rounded coordinate is outside int64");
  return static_cast<std::int64_t>(rounded);
}
}  // namespace
mv::core::Regions move_region(const mv::core::Regions& regions,
                              std::int64_t row, std::int64_t column) {
  (void)region_coordinate(row, "move_region");
  (void)region_coordinate(column, "move_region");
  if (row == 0 && column == 0) return regions;
  return map_regions(regions,
                     [&](const auto& r) { return r.shifted(row, column); });
}
mv::core::Regions symm_difference(const mv::core::Regions& left,
                                  const mv::core::Regions& right) {
  if (left.empty()) return {};
  const auto mask = union1(right);
  return map_regions(left, [&](const auto& r) {
    return r.difference(mask).united(mask.difference(r));
  });
}
mv::core::Tuple test_equal_region(const mv::core::Regions& left, const mv::core::Regions& right) {
  if (left.size() != right.size()) return mv::core::Tuple(0);
  for (std::size_t i = 0; i < left.size(); ++i) {
    const auto a = left[i].runs(), b = right[i].runs();
    // 身份比较前要求等长，共享切片可能同起点但不同范围；其他情况完整比较值。
    if (a.size() != b.size() ||
        (a.data() != b.data() && !std::equal(a.begin(), a.end(), b.begin())))
      return mv::core::Tuple(0);
  }
  return mv::core::Tuple(1);
}
mv::core::Tuple test_region_point(const mv::core::Regions& regions, const mv::core::Tuple& rows,
                                  const mv::core::Tuple& columns) {
  using namespace mv::core;
  if (rows.size() != columns.size() || (regions.size() > 1 && rows.size() > 1))
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, "test_region_point",
                    "Row/Column", -1,
                    "equal coordinate counts and one region or one point are required");
  // 同质 Int64 存储证明全部坐标合法，含两侧极值；只读视图仅活至此拥有型结果返回。
  // 其他类型仍须在首次命中前完整交错校验。
  if (rows.kind() == TupleKind::Int64 && columns.kind() == TupleKind::Int64) {
    const auto rr = rows.int64_span(), cc = columns.int64_span();
    if (regions.empty()) return {};
    for (std::size_t i = 0; i < regions.size(); ++i) {
      if (regions[i].run_count() == 0) continue;
      for (std::size_t j = 0; j < rr.size(); ++j)
        if (regions[i].contains(rr[j], cc[j])) return Tuple(1);
    }
    return Tuple(0);
  }
  // 任何提前命中前验证完整输入，零对象也如此。
  mv::runtime::ExecutionContext context;
  std::pmr::vector<std::int64_t> rr(context.scratch_resource()), cc(context.scratch_resource());
  rr.reserve(rows.size());
  cc.reserve(columns.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    rr.push_back(coordinate(rows, i, "Row"));
    cc.push_back(coordinate(columns, i, "Column"));
  }
  if (regions.empty()) return {};
  for (std::size_t i = 0; i < regions.size(); ++i) {
    if (regions[i].run_count() == 0) continue;
    for (std::size_t j = 0; j < rr.size(); ++j)
      if (regions[i].contains(rr[j], cc[j])) return Tuple(1);
  }
  return Tuple(0);
}
}  // namespace mv::regionproc
