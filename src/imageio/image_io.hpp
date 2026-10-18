#pragma once

#include <utility>

#include <core/image.hpp>
#include <imageio/codec.hpp>
#include <imageio/registry.hpp>
#include <imageio/stream.hpp>

namespace mv::imageio {

class ImageIOService {
 public:
  explicit ImageIOService(CodecRegistry registry) : registry_(std::move(registry)) {}

  [[nodiscard]] core::Image read(InputStream& stream,
                                 const DecodeOptions& options = {}) const;
  void write(OutputStream& stream,
             ImageFormat format,
             const core::Image& image,
             const EncodeOptions& options = {}) const;

 private:
  CodecRegistry registry_;
};

}  // namespace mv::imageio
