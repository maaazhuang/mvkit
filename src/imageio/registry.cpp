#include <imageio/registry.hpp>

#include <stdexcept>
#include <utility>

namespace mv::imageio {

void CodecRegistry::register_codec(std::shared_ptr<IImageCodec> codec) {
  if (!codec) throw std::invalid_argument{"codec must not be null"};
  codecs_.push_back(std::move(codec));
}

const IImageCodec* CodecRegistry::find_decoder(std::span<const std::byte> header) const noexcept {
  for (const auto& codec : codecs_) {
    if (codec->can_decode(header)) return codec.get();
  }
  return nullptr;
}

const IImageCodec* CodecRegistry::find_encoder(ImageFormat format) const noexcept {
  for (const auto& codec : codecs_) {
    if (codec->can_encode(format)) return codec.get();
  }
  return nullptr;
}

}  // namespace mv::imageio
