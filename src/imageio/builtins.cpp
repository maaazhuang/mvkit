#include <imageio/builtins.hpp>

#include <memory>

#include <imageio/codecs/bmp_codec.hpp>
#include <imageio/codecs/jpeg_codec.hpp>
#include <imageio/codecs/png_codec.hpp>
#include <imageio/codecs/tiff_codec.hpp>

namespace mv::imageio {

CodecRegistry make_builtin_registry() {
  CodecRegistry registry;
  registry.register_codec(std::make_shared<BmpCodec>());
  registry.register_codec(std::make_shared<JpegCodec>());
  registry.register_codec(std::make_shared<PngCodec>());
  registry.register_codec(std::make_shared<TiffCodec>());
  return registry;
}

ImageIOService& default_image_io_service() {
  static ImageIOService service{make_builtin_registry()};
  return service;
}

}  // namespace mv::imageio
