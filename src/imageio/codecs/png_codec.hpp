#pragma once

#include <imageio/codec.hpp>

namespace mv::imageio {

class PngCodec final : public IImageCodec {
 public:
  [[nodiscard]] std::string_view name() const noexcept override { return "png"; }
  [[nodiscard]] bool can_decode(std::span<const std::byte> header) const noexcept override;
  [[nodiscard]] bool can_encode(ImageFormat format) const noexcept override { return format == ImageFormat::Png; }
  [[nodiscard]] core::Image decode(InputStream& stream, const DecodeOptions& options) const override;
  void encode(OutputStream& stream,
              const core::Image& image,
              const EncodeOptions& options) const override;
};

}  // namespace mv::imageio
