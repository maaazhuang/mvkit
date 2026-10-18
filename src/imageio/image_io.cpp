#include <imageio/image_io.hpp>

#include <array>
#include <cstddef>
#include <span>

#include <core/exception.hpp>

namespace mv::imageio {

core::Image ImageIOService::read(InputStream& stream, const DecodeOptions& options) const {
  if (!stream.good()) {
    throw core::Exception(core::Status::IoError, "ImageIOService::read", "input stream is not readable");
  }
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "ImageIOService::read", "input stream cannot seek to beginning");
  }

  std::array<std::byte, 64> header{};
  const auto count = stream.read(header.data(), header.size());
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "ImageIOService::read", "input stream cannot rewind after probing");
  }

  const auto* codec = registry_.find_decoder(std::span<const std::byte>{header.data(), count});
  if (codec == nullptr) {
    throw core::Exception(core::Status::Unsupported, "ImageIOService::read", "unsupported or unrecognized image format");
  }
  return codec->decode(stream, options);
}

void ImageIOService::write(OutputStream& stream,
                           ImageFormat format,
                           const core::Image& image,
                           const EncodeOptions& options) const {
  if (!stream.good()) {
    throw core::Exception(core::Status::IoError, "ImageIOService::write", "output stream is not writable");
  }
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "ImageIOService::write", "image must not be empty");
  }
  const auto* codec = registry_.find_encoder(format);
  if (codec == nullptr) {
    throw core::Exception(core::Status::Unsupported, "ImageIOService::write", "no encoder is registered for requested format");
  }
  codec->encode(stream, image, options);
}

}  // namespace mv::imageio
