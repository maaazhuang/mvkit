#pragma once

#include <cstddef>
#include <cstring>

#include "math.hpp"

namespace mv::imageproc {

// 先验证两输入，再覆盖全部有效样本；byte ROI 只初始化补集。
// 宽/混合平面保持既有分配路径，筛选未证明稳定收益。
inline mv::core::Image make_arithmetic_output(const mv::core::Image& a,
                                             const mv::core::Image& b) {
  if (!a.has_reduced_domain() && !b.has_reduced_domain())
    return make_full_output_for_overwrite(a);
  if (a.channel_count() != 1 || a.pixel_type(0) != mv::core::PixelType::UInt8)
    return make_output_like(a).reduce_domain(b.domain());
  auto output = make_full_output_for_overwrite(a);
  output = output.reduce_domain(a.domain().intersect(b.domain()));
  const auto domain = output.domain();
  const auto runs = domain.runs();
  for (std::size_t ch = 0; ch < output.channel_count(); ++ch) {
    auto* data = static_cast<std::byte*>(output.mutable_data(ch));
    const auto size = mv::core::bytes_per_sample(output.pixel_type(ch));
    const auto stride = output.stride(ch);
    std::size_t cursor = 0;
    for (const auto& run : runs) {
      const auto begin = static_cast<std::size_t>(run.row) * stride +
                         static_cast<std::size_t>(run.col_begin) * size;
      const auto end = static_cast<std::size_t>(run.row) * stride +
                       (static_cast<std::size_t>(run.col_end) + 1) * size;
      if (begin > cursor) std::memset(data + cursor, 0, begin - cursor);
      cursor = end;
    }
    const auto end = output.height() * stride;
    if (end > cursor) std::memset(data + cursor, 0, end - cursor);
  }
  return output;
}

}  // namespace mv::imageproc
