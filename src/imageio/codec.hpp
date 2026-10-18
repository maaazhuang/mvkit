#pragma once

#include <cstddef>
#include <span>
#include <string_view>

#include <core/image.hpp>
#include <imageio/decode_limits.hpp>
#include <imageio/stream.hpp>

namespace mv::imageio {

enum class ImageFormat {
  Bmp,
  Jpeg,
  Png,
  Tiff,
};

struct DecodeOptions {
  DecodeLimits limits{};
};

struct EncodeOptions {
  int jpeg_quality{95};
};

class IImageCodec {
 public:
  virtual ~IImageCodec() = default;
  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
  [[nodiscard]] virtual bool can_decode(std::span<const std::byte> header) const noexcept = 0;
  [[nodiscard]] virtual bool can_encode(ImageFormat format) const noexcept = 0;
  [[nodiscard]] virtual core::Image decode(InputStream& stream, const DecodeOptions& options) const = 0;
  virtual void encode(OutputStream& stream,
                      const core::Image& image,
                      const EncodeOptions& options) const = 0;
};

}  // namespace mv::imageio
