#include <imageio/codecs/png_codec.hpp>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include <png.h>

#include <core/exception.hpp>
#include <imageio/decode_limits.hpp>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4611)  // setjmp 跨越 C++ 对象，诊断类别同 GCC -Wclobbered。
#endif

namespace mv::imageio {
namespace {

void png_error_callback(png_structp png_ptr, png_const_charp) {
  png_longjmp(png_ptr, 1);
}

void png_warning_callback(png_structp, png_const_charp) {}

void png_read_callback(png_structp png_ptr, png_bytep data, png_size_t length) {
  auto* stream = static_cast<InputStream*>(png_get_io_ptr(png_ptr));
  std::size_t done = 0;
  while (done < length) {
    const auto count = stream->read(data + done, static_cast<std::size_t>(length) - done);
    if (count == 0) png_error(png_ptr, "unexpected end of PNG stream");
    done += count;
  }
}

void png_write_callback(png_structp png_ptr, png_bytep data, png_size_t length) {
  auto* stream = static_cast<OutputStream*>(png_get_io_ptr(png_ptr));
  std::size_t done = 0;
  while (done < length) {
    const auto count = stream->write(data + done, static_cast<std::size_t>(length) - done);
    if (count == 0) png_error(png_ptr, "failed to write PNG stream");
    done += count;
  }
}

void png_flush_callback(png_structp) {}

struct PngProbe {
  std::size_t width{};
  std::size_t height{};
  int bit_depth{};
  int color_type{};
  bool has_trns{};
};

PngProbe probe_png(InputStream& stream) {
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "PngCodec::decode", "cannot seek PNG stream");
  }
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, png_error_callback, png_warning_callback);
  if (png_ptr == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to create libpng read context");
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == nullptr) {
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to create libpng info context");
  }
  if (setjmp(png_jmpbuf(png_ptr)) != 0) {
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    throw core::Exception(core::Status::DecodeError, "PngCodec::decode", "invalid or truncated PNG stream");
  }
  png_set_read_fn(png_ptr, &stream, png_read_callback);
  png_read_info(png_ptr, info_ptr);
  const auto width = png_get_image_width(png_ptr, info_ptr);
  const auto height = png_get_image_height(png_ptr, info_ptr);
  PngProbe probe{static_cast<std::size_t>(width),
                 static_cast<std::size_t>(height),
                 png_get_bit_depth(png_ptr, info_ptr),
                 png_get_color_type(png_ptr, info_ptr),
                 png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS) != 0};
  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
  return probe;
}

core::PixelType output_format_for(const PngProbe& probe) {
  switch (probe.color_type) {
    case PNG_COLOR_TYPE_GRAY:
    case PNG_COLOR_TYPE_GRAY_ALPHA:
      return probe.bit_depth == 16 ? core::PixelType::UInt16 : core::PixelType::UInt8;
    case PNG_COLOR_TYPE_PALETTE:
    case PNG_COLOR_TYPE_RGB:
    case PNG_COLOR_TYPE_RGB_ALPHA:
      return core::PixelType::UInt8;
    default:
      throw core::Exception(core::Status::Unsupported, "PngCodec::decode", "PNG color type is not supported");
  }
}

void configure_read_transforms(png_structp png_ptr, const PngProbe& probe, core::PixelType output_format) {
  if (probe.color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png_ptr);
  if (probe.color_type == PNG_COLOR_TYPE_GRAY && probe.bit_depth < 8) png_set_expand_gray_1_2_4_to_8(png_ptr);
  if (probe.has_trns) png_set_tRNS_to_alpha(png_ptr);
  if ((probe.color_type & PNG_COLOR_MASK_ALPHA) != 0 || probe.has_trns) png_set_strip_alpha(png_ptr);
  if (probe.bit_depth == 16 && output_format == core::PixelType::UInt8) png_set_strip_16(png_ptr);
  if (output_format == core::PixelType::UInt16 && std::endian::native == std::endian::little) png_set_swap(png_ptr);
}

}  // namespace

bool PngCodec::can_decode(std::span<const std::byte> header) const noexcept {
  if (header.size() < 8) return false;
  const auto* data = reinterpret_cast<const png_byte*>(header.data());
  return png_sig_cmp(data, 0, 8) == 0;
}

core::Image PngCodec::decode(InputStream& stream, const DecodeOptions& options) const {
  const auto probe = probe_png(stream);
  const auto output_format = output_format_for(probe);
  const bool color = probe.color_type == PNG_COLOR_TYPE_PALETTE ||
                     probe.color_type == PNG_COLOR_TYPE_RGB ||
                     probe.color_type == PNG_COLOR_TYPE_RGB_ALPHA;
  const std::size_t channels = color ? 3u : 1u;
  const auto bpp = core::bytes_per_sample(output_format);
  const auto decoded_bytes = checked_image_bytes(probe.width, probe.height, bpp * channels, options.limits);
  auto* raw = static_cast<png_bytep>(std::malloc(decoded_bytes));
  if (raw == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to allocate PNG decode buffer");
  }
  auto** rows = static_cast<png_bytep*>(std::malloc(probe.height * sizeof(png_bytep)));
  if (rows == nullptr) {
    std::free(raw);
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to allocate PNG row table");
  }
  const std::size_t expected_row_bytes = probe.width * bpp * channels;
  for (std::size_t y = 0; y < probe.height; ++y) rows[y] = raw + y * expected_row_bytes;

  if (!stream.seek(0)) {
    std::free(rows);
    std::free(raw);
    throw core::Exception(core::Status::IoError, "PngCodec::decode", "cannot rewind PNG stream");
  }

  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, png_error_callback, png_warning_callback);
  if (png_ptr == nullptr) {
    std::free(rows); std::free(raw);
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to create libpng read context");
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == nullptr) {
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    std::free(rows); std::free(raw);
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::decode", "failed to create libpng info context");
  }
  if (setjmp(png_jmpbuf(png_ptr)) != 0) {
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    std::free(rows); std::free(raw);
    throw core::Exception(core::Status::DecodeError, "PngCodec::decode", "PNG decode failed");
  }

  png_set_read_fn(png_ptr, &stream, png_read_callback);
  png_read_info(png_ptr, info_ptr);
  configure_read_transforms(png_ptr, probe, output_format);
  png_read_update_info(png_ptr, info_ptr);
  if (png_get_rowbytes(png_ptr, info_ptr) != expected_row_bytes) {
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    std::free(rows); std::free(raw);
    throw core::Exception(core::Status::DecodeError, "PngCodec::decode", "unexpected PNG row layout after transforms");
  }
  png_read_image(png_ptr, rows);
  png_read_end(png_ptr, info_ptr);
  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
  std::free(rows);

  std::unique_ptr<std::byte, decltype(&std::free)> guard(reinterpret_cast<std::byte*>(raw), &std::free);
  if (color) {
    return core::Image::import_packed_rgb(
        probe.width, probe.height, core::ChannelOrder::RGB, raw, expected_row_bytes);
  }
  auto image = core::Image::create(probe.width, probe.height, output_format);
  auto* dst = static_cast<std::byte*>(image.mutable_data());
  for (std::size_t y = 0; y < probe.height; ++y) {
    std::memcpy(dst + y * image.stride(), raw + y * expected_row_bytes, expected_row_bytes);
  }
  return image;
}

void PngCodec::encode(OutputStream& stream,
                      const core::Image& image,
                      const EncodeOptions&) const {
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "PngCodec::encode", "image must not be empty");
  }
  const bool gray8 = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt8;
  const bool gray16 = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt16;
  const bool rgb = image.channel_count() == 3 &&
                   image.pixel_type(0) == core::PixelType::UInt8 &&
                   image.pixel_type(1) == core::PixelType::UInt8 &&
                   image.pixel_type(2) == core::PixelType::UInt8;
  if (!gray8 && !gray16 && !rgb) {
    throw core::Exception(core::Status::Unsupported, "PngCodec::encode", "pixel format is not supported by PNG encoder");
  }
  const int color_type = rgb ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_GRAY;
  const int bit_depth = gray16 ? 16 : 8;
  if (image.width() > std::numeric_limits<png_uint_32>::max() || image.height() > std::numeric_limits<png_uint_32>::max()) {
    throw core::Exception(core::Status::OutOfRange, "PngCodec::encode", "image dimensions exceed PNG limits");
  }

  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, png_error_callback, png_warning_callback);
  if (png_ptr == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::encode", "failed to create libpng write context");
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == nullptr) {
    png_destroy_write_struct(&png_ptr, nullptr);
    throw core::Exception(core::Status::OutOfMemory, "PngCodec::encode", "failed to create libpng info context");
  }
  if (setjmp(png_jmpbuf(png_ptr)) != 0) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw core::Exception(core::Status::EncodeError, "PngCodec::encode", "PNG encode failed");
  }

  png_set_write_fn(png_ptr, &stream, png_write_callback, png_flush_callback);
  png_set_IHDR(png_ptr,
               info_ptr,
               static_cast<png_uint_32>(image.width()),
               static_cast<png_uint_32>(image.height()),
               bit_depth,
               color_type,
               PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png_ptr, info_ptr);
  if (gray16 && std::endian::native == std::endian::little) png_set_swap(png_ptr);
  std::vector<std::uint8_t> packed;
  if (rgb) {
    packed.resize(image.width() * image.height() * 3);
    image.export_packed_rgb(core::ChannelOrder::RGB, packed.data(), image.width() * 3);
  }
  const auto* base = rgb ? packed.data() : static_cast<const png_byte*>(image.data(0));
  const auto stride = rgb ? image.width() * 3 : image.stride(0);
  for (std::size_t y = 0; y < image.height(); ++y) {
    png_write_row(png_ptr, const_cast<png_bytep>(base + y * stride));
  }
  png_write_end(png_ptr, info_ptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
}

}  // namespace mv::imageio
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
