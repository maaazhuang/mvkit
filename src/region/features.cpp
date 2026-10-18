#include <region/operators.hpp>
#include <limits>

namespace mv::regionproc {

AreaCenterResult area_center(const mv::core::Region& region) {
  AreaCenterResult out{};
  out.area = region.area();
  if (out.area == 0) return out;
  if (const auto box=region.bounding_box(); box &&
      box->row_min >= INT32_MIN && box->row_max <= INT32_MAX &&
      box->col_min >= INT32_MIN && box->col_max <= INT32_MAX) {
    const auto width=std::uint64_t(box->col_max-box->col_min)+1;
    const auto height=std::uint64_t(box->row_max-box->row_min)+1;
    if (height <= std::numeric_limits<std::uint64_t>::max()/width && out.area==width*height) {
      out.row=(double(box->row_min)+double(box->row_max))*.5;
      out.column=(double(box->col_min)+double(box->col_max))*.5;
      return out;
    }
  }
  long double row_sum = 0.0L;
  long double col_sum = 0.0L;
  for (const auto& run : region.runs()) {
    const auto len = static_cast<long double>(static_cast<std::uint64_t>(run.col_end) - static_cast<std::uint64_t>(run.col_begin) + 1u);
    row_sum += static_cast<long double>(run.row) * len;
    col_sum += (static_cast<long double>(run.col_begin) + static_cast<long double>(run.col_end)) * len * 0.5L;
  }
  out.row = static_cast<double>(row_sum / static_cast<long double>(out.area));
  out.column = static_cast<double>(col_sum / static_cast<long double>(out.area));
  return out;
}

std::vector<AreaCenterResult> area_center(const mv::core::Regions& regions) {
  std::vector<AreaCenterResult> out;
  out.reserve(regions.size());
  for (std::size_t index = 0; index < regions.size(); ++index) out.push_back(area_center(regions[index]));
  return out;
}

Rectangle1 smallest_rectangle1(const mv::core::Region& region) {
  const auto box = region.bounding_box();
  if (!box) return {};
  return {box->row_min, box->col_min, box->row_max, box->col_max};
}

std::vector<Rectangle1> smallest_rectangle1(const mv::core::Regions& regions) {
  std::vector<Rectangle1> out;
  out.reserve(regions.size());
  for (std::size_t index = 0; index < regions.size(); ++index) out.push_back(smallest_rectangle1(regions[index]));
  return out;
}

}  // namespace mv::regionproc
