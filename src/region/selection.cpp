#include <region/operators.hpp>
#include <core/exception.hpp>
#include <runtime/execution/execution_context.hpp>
#include <cmath>
#include <limits>
#include <memory_resource>

namespace mv::regionproc {
mv::core::Regions select_shape(const mv::core::Regions& regions,
                               const std::vector<ShapeCriterion>& criteria,
                               SelectionOperation operation, bool halcon_features) {
  using namespace mv::core;
  if (criteria.empty())
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    "select_shape", "Criteria", -1,
                    "criteria must not be empty");
  if (operation != SelectionOperation::And &&
      operation != SelectionOperation::Or)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "select_shape", "Operation", -1,
                    "invalid selection operation");
  std::uint32_t mask = 0;
  for (const auto& criterion : criteria) {
    validate_feature(criterion.feature, "select_shape");
    mask |= 1U << static_cast<unsigned>(criterion.feature);
    if ((criterion.min && !std::isfinite(*criterion.min)) ||
        (criterion.max && !std::isfinite(*criterion.max)) ||
        (criterion.min && criterion.max && *criterion.min > *criterion.max))
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                      "select_shape", "Criteria", -1,
                      "bounds must be finite and ordered");
  }
  mv::runtime::ExecutionContext context;
  const bool area_only=mask==(1U<<static_cast<unsigned>(ShapeFeature::Area));
  // 串行仅按面积选择可一次构造拥有型输出；通用特征及多 worker 保留下方分区路径。
  if (area_only && mv::runtime::max_parallelism() == 1) {
    const auto objects = regions.iconic_tuple().values();
    std::vector<IconicValue> output;
    output.reserve(objects.size());
    for (const auto& object : objects) {
      const double value = double(object.as_region().area());
      bool keep = operation == SelectionOperation::And;
      for (const auto& criterion : criteria) {
        const bool match = (!criterion.min || value >= *criterion.min) &&
                           (!criterion.max || value <= *criterion.max);
        if (operation == SelectionOperation::And) {
          keep = keep && match;
          if (!keep) break;
        } else {
          keep = keep || match;
          if (keep) break;
        }
      }
      if (keep) output.push_back(object);
    }
    return Regions{IconicTuple{std::move(output)}};
  }
  std::pmr::vector<unsigned char> selected(regions.size(), 0,
                                           context.scratch_resource());
  std::size_t work = area_only?regions.size():0;
  if(!area_only)for (std::size_t i = 0; i < regions.size(); ++i)
    work += std::min(regions[i].runs().size(),
                     std::numeric_limits<std::size_t>::max() - work);
  mv::runtime::ExecutionContext::for_each_partition(
      regions.size(), work, 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) {
          ShapeFeatures values;
          if(!area_only)values=halcon_features?halcon_shape_features(regions[i], mask):shape_features(regions[i], mask);
          bool keep = operation == SelectionOperation::And;
          for (const auto& criterion : criteria) {
            const auto value =
                area_only?double(regions[i].area()):values[static_cast<std::size_t>(criterion.feature)];
            const bool match = (!criterion.min || value >= *criterion.min) &&
                               (!criterion.max || value <= *criterion.max);
            if (operation == SelectionOperation::And) {
              keep = keep && match;
              if (!keep) break;
            } else {
              keep = keep || match;
              if (keep) break;
            }
          }
          selected[i] = keep ? 1 : 0;
        }
      });
  std::vector<IconicValue> output;
  output.reserve(regions.size());
  for (std::size_t i = 0; i < regions.size(); ++i)
    if (selected[i]) output.push_back(regions.iconic_tuple().at(i));
  return Regions{IconicTuple{std::move(output)}};
}
}  // namespace mv::regionproc
