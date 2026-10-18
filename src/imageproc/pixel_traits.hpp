#pragma once

#include <cstddef>
#include <core/image.hpp>

namespace mv::imageproc {

struct PixelTraitsInfo {
  std::size_t channels{0};
  std::size_t bytes_per_channel{0};
  bool is_signed{false};
  bool is_float{false};
};

[[nodiscard]] constexpr PixelTraitsInfo pixel_traits(mv::core::PixelType format) noexcept {
  using mv::core::PixelType;
  switch (format) {
    case PixelType::UInt8: return {1, 1, false, false};
    case PixelType::UInt16: return {1, 2, false, false};
    case PixelType::Int16: return {1, 2, true, false};
    case PixelType::Int32: return {1, 4, true, false};
    case PixelType::Float32: return {1, 4, true, true};
  }
  return {};
}

[[nodiscard]] constexpr bool is_scalar_format(mv::core::PixelType format) noexcept {
  return pixel_traits(format).channels == 1;
}

}  // namespace mv::imageproc
