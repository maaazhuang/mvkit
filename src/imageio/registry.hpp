#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

#include <imageio/codec.hpp>

namespace mv::imageio {

class CodecRegistry {
 public:
  void register_codec(std::shared_ptr<IImageCodec> codec);
  [[nodiscard]] const IImageCodec* find_decoder(std::span<const std::byte> header) const noexcept;
  [[nodiscard]] const IImageCodec* find_encoder(ImageFormat format) const noexcept;
  [[nodiscard]] std::size_t size() const noexcept { return codecs_.size(); }

 private:
  std::vector<std::shared_ptr<IImageCodec>> codecs_;
};

}  // namespace mv::imageio
