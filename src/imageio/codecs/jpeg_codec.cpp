#include <imageio/codecs/jpeg_codec.hpp>

#include <algorithm>
#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include <jpeglib.h>
}

#include <core/exception.hpp>
#include <imageio/decode_limits.hpp>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)  // JpegErrorManager 中 jmp_buf 的对齐 padding。
#pragma warning(disable : 4611)  // setjmp 跨越 C++ 对象，诊断类别同 GCC -Wclobbered。
#endif

namespace mv::imageio {
namespace {

struct JpegErrorManager {
  jpeg_error_mgr base{};
  std::jmp_buf jump{};
  char message[JMSG_LENGTH_MAX]{};
};


void jpeg_silent_output(j_common_ptr) {}

void jpeg_error_exit(j_common_ptr common) {
  auto* error = reinterpret_cast<JpegErrorManager*>(common->err);
  (*common->err->format_message)(common, error->message);
  std::longjmp(error->jump, 1);
}

std::vector<unsigned char> read_all(InputStream& stream, const DecodeLimits& limits) {
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "JpegCodec::decode", "cannot seek JPEG stream");
  }
  const auto size64 = stream.size();
  if (size64 == 0) {
    throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", "empty JPEG stream");
  }
  if (size64 > limits.max_decoded_bytes || size64 > std::numeric_limits<std::size_t>::max() ||
      size64 > std::numeric_limits<unsigned long>::max()) {
    throw core::Exception(core::Status::LimitExceeded, "JpegCodec::decode", "JPEG input exceeds configured size limits");
  }
  std::vector<unsigned char> bytes(static_cast<std::size_t>(size64));
  std::size_t done = 0;
  while (done < bytes.size()) {
    const auto count = stream.read(bytes.data() + done, bytes.size() - done);
    if (count == 0) {
      throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", "truncated JPEG stream");
    }
    done += count;
  }
  return bytes;
}

struct JpegProbe {
  std::size_t width{};
  std::size_t height{};
  J_COLOR_SPACE color_space{JCS_UNKNOWN};
};

JpegProbe probe_jpeg(const unsigned char* data, unsigned long size) {
  jpeg_decompress_struct cinfo{};
  JpegErrorManager error{};
  cinfo.err = jpeg_std_error(&error.base);
  error.base.error_exit = jpeg_error_exit;
  error.base.output_message = jpeg_silent_output;

  if (setjmp(error.jump) != 0) {
    jpeg_destroy_decompress(&cinfo);
    throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", error.message[0] ? error.message : "invalid JPEG stream");
  }

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, data, size);
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&cinfo);
    throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", "invalid JPEG header");
  }
  JpegProbe result{static_cast<std::size_t>(cinfo.image_width),
                   static_cast<std::size_t>(cinfo.image_height),
                   cinfo.jpeg_color_space};
  jpeg_destroy_decompress(&cinfo);
  return result;
}

void write_all(OutputStream& stream, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::byte*>(data);
  std::size_t done = 0;
  while (done < size) {
    const auto count = stream.write(bytes + done, size - done);
    if (count == 0) {
      throw core::Exception(core::Status::EncodeError, "JpegCodec::encode", "failed to write JPEG stream");
    }
    done += count;
  }
}

}  // namespace

bool JpegCodec::can_decode(std::span<const std::byte> header) const noexcept {
  return header.size() >= 2 && header[0] == std::byte{0xff} && header[1] == std::byte{0xd8};
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclobbered"
#endif
core::Image JpegCodec::decode(InputStream& stream, const DecodeOptions& options) const {
  const auto compressed = read_all(stream, options.limits);
  const auto probe = probe_jpeg(compressed.data(), static_cast<unsigned long>(compressed.size()));
  if (probe.color_space == JCS_CMYK || probe.color_space == JCS_YCCK) {
    throw core::Exception(core::Status::Unsupported, "JpegCodec::decode", "CMYK/YCCK JPEG is not supported in ImageIO v1");
  }

  const auto decoded_size = checked_image_bytes(
      probe.width,
      probe.height,
      probe.color_space == JCS_GRAYSCALE ? 1u : 3u,
      options.limits);
  auto* raw = static_cast<unsigned char*>(std::malloc(decoded_size));
  if (raw == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "JpegCodec::decode", "failed to allocate JPEG decode buffer");
  }

  jpeg_decompress_struct cinfo{};
  JpegErrorManager error{};
  cinfo.err = jpeg_std_error(&error.base);
  error.base.error_exit = jpeg_error_exit;
  error.base.output_message = jpeg_silent_output;
  if (setjmp(error.jump) != 0) {
    jpeg_destroy_decompress(&cinfo);
    std::free(raw);
    throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", error.message[0] ? error.message : "JPEG decode failed");
  }

  const bool grayscale = probe.color_space == JCS_GRAYSCALE;
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, compressed.data(), static_cast<unsigned long>(compressed.size()));
  (void)jpeg_read_header(&cinfo, TRUE);
  cinfo.out_color_space = grayscale ? JCS_GRAYSCALE : JCS_RGB;
  jpeg_start_decompress(&cinfo);
  const std::size_t row_bytes =
      static_cast<std::size_t>(cinfo.output_width) * static_cast<std::size_t>(cinfo.output_components);
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row = raw + static_cast<std::size_t>(cinfo.output_scanline) * row_bytes;
    if (jpeg_read_scanlines(&cinfo, &row, 1) != 1) {
      jpeg_destroy_decompress(&cinfo);
      std::free(raw);
      throw core::Exception(core::Status::DecodeError, "JpegCodec::decode", "JPEG scanline decode failed");
    }
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  std::unique_ptr<unsigned char, decltype(&std::free)> guard(raw, &std::free);
  if (!grayscale) {
    return core::Image::import_packed_rgb(
        probe.width, probe.height, core::ChannelOrder::RGB, raw, row_bytes);
  }
  auto image = core::Image::create(probe.width, probe.height, core::PixelType::UInt8);
  auto* dst = static_cast<std::byte*>(image.mutable_data());
  for (std::size_t y = 0; y < probe.height; ++y) {
    std::memcpy(dst + y * image.stride(), raw + y * row_bytes, row_bytes);
  }
  return image;
}

void JpegCodec::encode(OutputStream& stream,
                       const core::Image& image,
                       const EncodeOptions& options) const {
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "JpegCodec::encode", "image must not be empty");
  }
  if (options.jpeg_quality < 1 || options.jpeg_quality > 100) {
    throw core::Exception(core::Status::InvalidArgument, "JpegCodec::encode", "JPEG quality must be in [1, 100]");
  }
  const bool grayscale = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt8;
  const bool rgb = image.channel_count() == 3 &&
                   image.pixel_type(0) == core::PixelType::UInt8 &&
                   image.pixel_type(1) == core::PixelType::UInt8 &&
                   image.pixel_type(2) == core::PixelType::UInt8;
  if (!grayscale && !rgb) {
    throw core::Exception(core::Status::Unsupported, "JpegCodec::encode", "pixel format is not supported by JPEG encoder");
  }
  if (image.width() > std::numeric_limits<JDIMENSION>::max() ||
      image.height() > std::numeric_limits<JDIMENSION>::max()) {
    throw core::Exception(core::Status::OutOfRange, "JpegCodec::encode", "image dimensions exceed JPEG limits");
  }

  const std::size_t channels = grayscale ? 1u : 3u;
  std::vector<std::uint8_t> packed;
  if (rgb) {
    packed.resize(image.width() * image.height() * 3);
    image.export_packed_rgb(core::ChannelOrder::RGB, packed.data(), image.width() * 3);
  }

  jpeg_compress_struct cinfo{};
  JpegErrorManager error{};
  unsigned char* output_buffer = nullptr;
  unsigned long output_size = 0;
  cinfo.err = jpeg_std_error(&error.base);
  error.base.error_exit = jpeg_error_exit;
  error.base.output_message = jpeg_silent_output;
  if (setjmp(error.jump) != 0) {
    jpeg_destroy_compress(&cinfo);
    std::free(output_buffer);
    throw core::Exception(core::Status::EncodeError, "JpegCodec::encode", error.message[0] ? error.message : "JPEG encode failed");
  }

  jpeg_create_compress(&cinfo);
  jpeg_mem_dest(&cinfo, &output_buffer, &output_size);
  cinfo.image_width = static_cast<JDIMENSION>(image.width());
  cinfo.image_height = static_cast<JDIMENSION>(image.height());
  cinfo.input_components = static_cast<int>(channels);
  cinfo.in_color_space = grayscale ? JCS_GRAYSCALE : JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, options.jpeg_quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  const auto* base = rgb ? packed.data() : static_cast<const std::uint8_t*>(image.data(0));
  const auto stride = rgb ? image.width() * 3 : image.stride(0);
  while (cinfo.next_scanline < cinfo.image_height) {
    const auto* source = base + static_cast<std::size_t>(cinfo.next_scanline) * stride;
    JSAMPROW row = const_cast<JSAMPROW>(source);
    if (jpeg_write_scanlines(&cinfo, &row, 1) != 1) {
      jpeg_destroy_compress(&cinfo);
      std::free(output_buffer);
      throw core::Exception(core::Status::EncodeError, "JpegCodec::encode", "JPEG scanline encode failed");
    }
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  std::unique_ptr<unsigned char, decltype(&std::free)> guard(output_buffer, &std::free);
  write_all(stream, output_buffer, static_cast<std::size_t>(output_size));
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

}  // namespace mv::imageio
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
