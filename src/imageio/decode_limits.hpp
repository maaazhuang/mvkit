#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include <core/exception.hpp>

namespace mv::imageio {

struct DecodeLimits {
  std::size_t max_width{100000};
  std::size_t max_height{100000};
  std::size_t max_pixels{1000000000ULL};
  std::size_t max_decoded_bytes{
      sizeof(std::size_t) >= 8 ? static_cast<std::size_t>(4ULL * 1024ULL * 1024ULL * 1024ULL)
                               : std::numeric_limits<std::size_t>::max()};
};

[[nodiscard]] inline std::size_t checked_multiply(std::size_t a,
                                                  std::size_t b,
                                                  const char* where) {
  if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a) {
    throw core::Exception(core::Status::OutOfRange, where, "size calculation overflow");
  }
  return a * b;
}

[[nodiscard]] inline std::size_t checked_image_bytes(std::size_t width,
                                                     std::size_t height,
                                                     std::size_t bytes_per_sample,
                                                     const DecodeLimits& limits) {
  if (width == 0 || height == 0 || bytes_per_sample == 0) {
    throw core::Exception(core::Status::InvalidArgument, "ImageIO", "image dimensions and pixel size must be non-zero");
  }
  if (width > limits.max_width || height > limits.max_height) {
    throw core::Exception(core::Status::LimitExceeded, "ImageIO", "image dimensions exceed decode limits");
  }
  const auto pixels = checked_multiply(width, height, "ImageIO");
  if (pixels > limits.max_pixels) {
    throw core::Exception(core::Status::LimitExceeded, "ImageIO", "pixel count exceeds decode limits");
  }
  const auto bytes = checked_multiply(pixels, bytes_per_sample, "ImageIO");
  if (bytes > limits.max_decoded_bytes) {
    throw core::Exception(core::Status::LimitExceeded, "ImageIO", "decoded byte size exceeds decode limits");
  }
  return bytes;
}

}  // namespace mv::imageio
