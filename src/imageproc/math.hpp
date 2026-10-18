#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>
#include <vector>

#include <core/image.hpp>

namespace mv::imageproc {

// 外部平面允许非对齐地址及步长；固定大小 memcpy 可内联且不引入对齐假设。
template <typename T>
[[nodiscard]] T load_pixel(const void* address) noexcept {
  T value;
  std::memcpy(&value, address, sizeof(T));
  return value;
}

template <typename T>
[[nodiscard]] T saturate_cast(double value) noexcept {
  if constexpr (std::is_floating_point_v<T>) {
    return static_cast<T>(value);
  } else {
    const double low = static_cast<double>(std::numeric_limits<T>::lowest());
    const double high = static_cast<double>(std::numeric_limits<T>::max());
    value = std::clamp(value, low, high);
    if constexpr (sizeof(T)<=sizeof(std::int32_t)) {
      if(std::isfinite(value)) {
        // 整数范围内截断与余数比较等价于 llround；避免加 0.5 误舍入半值前驱。
        auto whole=static_cast<std::int64_t>(value);
        const double fraction=value-static_cast<double>(whole);
        whole+=fraction>=.5;
        if constexpr(std::is_signed_v<T>)whole-=fraction<=-.5;
        return static_cast<T>(whole);
      }
    }
    return static_cast<T>(std::llround(value));
  }
}

[[nodiscard]] inline mv::core::Image make_output_like(const mv::core::Image& input,
                                                        mv::core::PixelType format) {
  auto output = mv::core::Image::create(input.width(), input.height(), format);
  if (input.has_reduced_domain()) output = output.reduce_domain(input.domain());
  return output;
}

[[nodiscard]] inline mv::core::Image make_output_like(const mv::core::Image& input) {
  std::vector<mv::core::PixelType> types;
  types.reserve(input.channel_count());
  for (std::size_t channel = 0; channel < input.channel_count(); ++channel) {
    types.push_back(input.pixel_type(channel));
  }
  auto output = mv::core::Image::create(input.width(), input.height(), types);
  if (input.has_reduced_domain()) output = output.reduce_domain(input.domain());
  return output;
}

[[nodiscard]] inline mv::core::Image make_full_output_for_overwrite(
    const mv::core::Image& input) {
  std::vector<mv::core::PixelType> types;
  types.reserve(input.channel_count());
  for (std::size_t channel = 0; channel < input.channel_count(); ++channel)
    types.push_back(input.pixel_type(channel));
  return mv::core::Image::create_uninitialized(input.width(), input.height(), types);
}

}  // namespace mv::imageproc
