#include <imageio/codecs/tiff_codec.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include <tiffio.h>

#if defined(TIFFLIB_AT_LEAST)
#if TIFFLIB_AT_LEAST(4, 5, 0)
#define MVKIT_TIFF_HAS_OPEN_OPTIONS 1
#endif
#endif

#include <core/exception.hpp>
#include <imageio/decode_limits.hpp>

namespace {

struct TiffClient {
  mv::imageio::InputStream* input{};
  mv::imageio::OutputStream* output{};
};

}  // namespace

extern "C" {

static tmsize_t tiff_read(thandle_t handle, void* buffer, tmsize_t size) {
  if (size < 0) return -1;
  auto* client = static_cast<TiffClient*>(handle);
  if (client == nullptr || client->input == nullptr) return -1;
  const auto count = client->input->read(buffer, static_cast<std::size_t>(size));
  if (count > static_cast<std::size_t>(std::numeric_limits<tmsize_t>::max())) return -1;
  return static_cast<tmsize_t>(count);
}

static tmsize_t tiff_write(thandle_t handle, void* buffer, tmsize_t size) {
  if (size < 0) return -1;
  auto* client = static_cast<TiffClient*>(handle);
  if (client == nullptr || client->output == nullptr) return -1;
  const auto count = client->output->write(buffer, static_cast<std::size_t>(size));
  if (count > static_cast<std::size_t>(std::numeric_limits<tmsize_t>::max())) return -1;
  return static_cast<tmsize_t>(count);
}

static toff_t tiff_seek(thandle_t handle, toff_t offset, int whence) {
  auto* client = static_cast<TiffClient*>(handle);
  if (client == nullptr) return static_cast<toff_t>(-1);
  const bool input_mode = client->input != nullptr;
  const std::uint64_t current = input_mode ? client->input->tell() : client->output->tell();
  const std::uint64_t end = input_mode ? client->input->size() : client->output->size();
  std::uint64_t target = 0;
  switch (whence) {
    case SEEK_SET: target = offset; break;
    case SEEK_CUR:
      if (offset > std::numeric_limits<std::uint64_t>::max() - current) return static_cast<toff_t>(-1);
      target = current + offset;
      break;
    case SEEK_END:
      if (offset > std::numeric_limits<std::uint64_t>::max() - end) return static_cast<toff_t>(-1);
      target = end + offset;
      break;
    default: return static_cast<toff_t>(-1);
  }
  const bool ok = input_mode ? client->input->seek(target) : client->output->seek(target);
  return ok ? static_cast<toff_t>(target) : static_cast<toff_t>(-1);
}

static int tiff_close(thandle_t) { return 0; }

static toff_t tiff_size(thandle_t handle) {
  auto* client = static_cast<TiffClient*>(handle);
  if (client == nullptr) return 0;
  return client->input != nullptr ? static_cast<toff_t>(client->input->size())
                                  : static_cast<toff_t>(client->output->size());
}

static int tiff_map(thandle_t, void**, toff_t*) { return 0; }
static void tiff_unmap(thandle_t, void*, toff_t) {}
#if defined(MVKIT_TIFF_HAS_OPEN_OPTIONS)
static int tiff_silent_handler(TIFF*, void*, const char*, const char*, va_list) { return 1; }
#endif

}

namespace mv::imageio {
namespace {

struct TiffCloser {
  void operator()(TIFF* tif) const noexcept { if (tif != nullptr) TIFFClose(tif); }
};
using TiffPtr = std::unique_ptr<TIFF, TiffCloser>;

#if defined(MVKIT_TIFF_HAS_OPEN_OPTIONS)
TIFFOpenOptions* make_open_options(const DecodeLimits* limits) {
  auto* options = TIFFOpenOptionsAlloc();
  if (options == nullptr) return nullptr;
  TIFFOpenOptionsSetErrorHandlerExtR(options, tiff_silent_handler, nullptr);
  TIFFOpenOptionsSetWarningHandlerExtR(options, tiff_silent_handler, nullptr);
  if (limits != nullptr) {
    const auto max_tmsize = static_cast<std::uint64_t>(std::numeric_limits<tmsize_t>::max());
    const auto requested = std::min<std::uint64_t>(limits->max_decoded_bytes, max_tmsize);
    TIFFOpenOptionsSetMaxSingleMemAlloc(options, static_cast<tmsize_t>(requested));
#if TIFFLIB_AT_LEAST(4, 6, 0)
    TIFFOpenOptionsSetMaxCumulatedMemAlloc(options, static_cast<tmsize_t>(requested));
#endif
  }
  return options;
}
#endif

TiffPtr open_for_read(InputStream& stream, const DecodeLimits& limits, TiffClient& client) {
  (void)limits;
  client.input = &stream;
  client.output = nullptr;
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "TiffCodec::decode", "cannot seek TIFF stream");
  }
#if defined(MVKIT_TIFF_HAS_OPEN_OPTIONS)
  TIFFOpenOptions* options = make_open_options(&limits);
  if (options == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "TiffCodec::decode", "failed to allocate libtiff open options");
  }
  TIFF* raw = TIFFClientOpenExt("MVKit", "r", &client, tiff_read, tiff_write, tiff_seek, tiff_close,
                               tiff_size, tiff_map, tiff_unmap, options);
  TIFFOpenOptionsFree(options);
#else
  TIFF* raw = TIFFClientOpen("MVKit", "r", &client, tiff_read, tiff_write, tiff_seek, tiff_close,
                            tiff_size, tiff_map, tiff_unmap);
#endif
  if (raw == nullptr) {
    throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "invalid or truncated TIFF stream");
  }
  return TiffPtr{raw};
}

TiffPtr open_for_write(OutputStream& stream, TiffClient& client) {
  client.input = nullptr;
  client.output = &stream;
  if (!stream.seek(0)) {
    throw core::Exception(core::Status::IoError, "TiffCodec::encode", "cannot seek TIFF output stream");
  }
#if defined(MVKIT_TIFF_HAS_OPEN_OPTIONS)
  TIFFOpenOptions* options = make_open_options(nullptr);
  if (options == nullptr) {
    throw core::Exception(core::Status::OutOfMemory, "TiffCodec::encode", "failed to allocate libtiff open options");
  }
  TIFF* raw = TIFFClientOpenExt("MVKit", "w", &client, tiff_read, tiff_write, tiff_seek, tiff_close,
                               tiff_size, tiff_map, tiff_unmap, options);
  TIFFOpenOptionsFree(options);
#else
  TIFF* raw = TIFFClientOpen("MVKit", "w", &client, tiff_read, tiff_write, tiff_seek, tiff_close,
                            tiff_size, tiff_map, tiff_unmap);
#endif
  if (raw == nullptr) {
    throw core::Exception(core::Status::EncodeError, "TiffCodec::encode", "failed to initialize TIFF writer");
  }
  return TiffPtr{raw};
}

struct TiffLayout {
  std::size_t width{};
  std::size_t height{};
  std::uint16_t bits{};
  std::uint16_t samples{};
  std::uint16_t photometric{};
  core::PixelType format{core::PixelType::UInt8};
  std::size_t bytes_per_sample{};
};

TiffLayout read_layout(TIFF* tif, const DecodeLimits& limits) {
  std::uint32_t width = 0, height = 0;
  std::uint16_t bits = 0, samples = 1, photometric = 0, planar = PLANARCONFIG_CONTIG;
  std::uint16_t sample_format = SAMPLEFORMAT_UINT, orientation = ORIENTATION_TOPLEFT;
  if (!TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width) || !TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height)) {
    throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "TIFF dimensions are missing");
  }
  TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &bits);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &samples);
  TIFFGetFieldDefaulted(tif, TIFFTAG_PHOTOMETRIC, &photometric);
  TIFFGetFieldDefaulted(tif, TIFFTAG_PLANARCONFIG, &planar);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &sample_format);
  TIFFGetFieldDefaulted(tif, TIFFTAG_ORIENTATION, &orientation);

  if (planar != PLANARCONFIG_CONTIG) {
    throw core::Exception(core::Status::Unsupported, "TiffCodec::decode", "planar-separate TIFF is not supported in ImageIO v1");
  }
  if (sample_format != SAMPLEFORMAT_UINT) {
    throw core::Exception(core::Status::Unsupported, "TiffCodec::decode", "non-unsigned TIFF samples are not supported in ImageIO v1");
  }
  if (orientation != ORIENTATION_TOPLEFT) {
    throw core::Exception(core::Status::Unsupported, "TiffCodec::decode", "TIFF orientation other than top-left is not supported in ImageIO v1");
  }

  TiffLayout layout;
  layout.width = width;
  layout.height = height;
  layout.bits = bits;
  layout.samples = samples;
  layout.photometric = photometric;
  if (samples == 1 && (photometric == PHOTOMETRIC_MINISBLACK || photometric == PHOTOMETRIC_MINISWHITE) &&
      (bits == 8 || bits == 16)) {
    layout.format = bits == 8 ? core::PixelType::UInt8 : core::PixelType::UInt16;
    layout.bytes_per_sample = bits / 8;
  } else if (samples == 3 && photometric == PHOTOMETRIC_RGB && bits == 8) {
    layout.format = core::PixelType::UInt8;
    layout.bytes_per_sample = 3;
  } else {
    throw core::Exception(core::Status::Unsupported, "TiffCodec::decode", "TIFF pixel layout is not supported in ImageIO v1");
  }
  (void)checked_image_bytes(layout.width, layout.height, layout.bytes_per_sample, limits);
  return layout;
}

void invert_miniswhite(core::Image& image) {
  if (image.pixel_type(0) == core::PixelType::UInt8) {
    auto* p = static_cast<std::uint8_t*>(image.mutable_data(0));
    for (std::size_t y = 0; y < image.height(); ++y) {
      for (std::size_t x = 0; x < image.width(); ++x) p[y * image.stride(0) + x] = static_cast<std::uint8_t>(255u - p[y * image.stride(0) + x]);
    }
  } else if (image.pixel_type() == core::PixelType::UInt16) {
    auto* p = static_cast<std::uint16_t*>(image.mutable_data(0));
    const auto stride_values = image.stride(0) / sizeof(std::uint16_t);
    for (std::size_t y = 0; y < image.height(); ++y) {
      for (std::size_t x = 0; x < image.width(); ++x) p[y * stride_values + x] = static_cast<std::uint16_t>(65535u - p[y * stride_values + x]);
    }
  }
}

void decode_strips(TIFF* tif, core::Image& image, const TiffLayout& layout, const DecodeLimits& limits) {
  std::uint32_t rows_per_strip = 0;
  TIFFGetFieldDefaulted(tif, TIFFTAG_ROWSPERSTRIP, &rows_per_strip);
  if (rows_per_strip == 0) {
    throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "invalid TIFF rows-per-strip");
  }
  const auto strip_size64 = TIFFStripSize64(tif);
  if (strip_size64 == 0 || strip_size64 > limits.max_decoded_bytes || strip_size64 > std::numeric_limits<std::size_t>::max() ||
      strip_size64 > static_cast<std::uint64_t>(std::numeric_limits<tmsize_t>::max())) {
    throw core::Exception(core::Status::LimitExceeded, "TiffCodec::decode", "TIFF strip buffer exceeds limits");
  }
  std::vector<std::byte> buffer(static_cast<std::size_t>(strip_size64));
  const auto row_bytes = layout.width * layout.bytes_per_sample;
  const auto strips = TIFFNumberOfStrips(tif);
  for (std::uint32_t strip = 0; strip < strips; ++strip) {
    const auto row_start = static_cast<std::size_t>(strip) * rows_per_strip;
    if (row_start >= layout.height) break;
    const auto rows = std::min<std::size_t>(rows_per_strip, layout.height - row_start);
    const auto needed = rows * row_bytes;
    const auto decoded = TIFFReadEncodedStrip(tif, strip, buffer.data(), static_cast<tmsize_t>(buffer.size()));
    if (decoded < 0 || static_cast<std::size_t>(decoded) < needed) {
      throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "failed to decode TIFF strip");
    }
    if (layout.samples == 1) {
      auto* dst = static_cast<std::byte*>(image.mutable_data(0)) + row_start * image.stride(0);
      for (std::size_t y = 0; y < rows; ++y) {
        std::memcpy(dst + y * image.stride(0), buffer.data() + y * row_bytes, row_bytes);
      }
    } else {
      auto* red = static_cast<std::uint8_t*>(image.mutable_data(0));
      auto* green = static_cast<std::uint8_t*>(image.mutable_data(1));
      auto* blue = static_cast<std::uint8_t*>(image.mutable_data(2));
      const auto* src = reinterpret_cast<const std::uint8_t*>(buffer.data());
      for (std::size_t y = 0; y < rows; ++y) {
        for (std::size_t x = 0; x < layout.width; ++x) {
          const auto offset = y * row_bytes + x * 3;
          red[(row_start + y) * image.stride(0) + x] = src[offset];
          green[(row_start + y) * image.stride(1) + x] = src[offset + 1];
          blue[(row_start + y) * image.stride(2) + x] = src[offset + 2];
        }
      }
    }
  }
}

void decode_tiles(TIFF* tif, core::Image& image, const TiffLayout& layout, const DecodeLimits& limits) {
  std::uint32_t tile_width = 0, tile_height = 0;
  if (!TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tile_width) || !TIFFGetField(tif, TIFFTAG_TILELENGTH, &tile_height) ||
      tile_width == 0 || tile_height == 0) {
    throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "invalid TIFF tile geometry");
  }
  const auto tile_size64 = TIFFTileSize64(tif);
  const auto tile_row64 = TIFFTileRowSize64(tif);
  if (tile_size64 == 0 || tile_row64 == 0 || tile_size64 > limits.max_decoded_bytes ||
      tile_size64 > std::numeric_limits<std::size_t>::max() || tile_row64 > std::numeric_limits<std::size_t>::max() ||
      tile_size64 > static_cast<std::uint64_t>(std::numeric_limits<tmsize_t>::max())) {
    throw core::Exception(core::Status::LimitExceeded, "TiffCodec::decode", "TIFF tile buffer exceeds limits");
  }
  std::vector<std::byte> buffer(static_cast<std::size_t>(tile_size64));
  const auto tile_row_bytes = static_cast<std::size_t>(tile_row64);
  for (std::uint32_t y0 = 0; y0 < layout.height; y0 += tile_height) {
    for (std::uint32_t x0 = 0; x0 < layout.width; x0 += tile_width) {
      const auto tile = TIFFComputeTile(tif, x0, y0, 0, 0);
      const auto decoded = TIFFReadEncodedTile(tif, tile, buffer.data(), static_cast<tmsize_t>(buffer.size()));
      if (decoded < 0) {
        throw core::Exception(core::Status::DecodeError, "TiffCodec::decode", "failed to decode TIFF tile");
      }
      const auto copy_width = std::min<std::size_t>(tile_width, layout.width - x0);
      const auto copy_height = std::min<std::size_t>(tile_height, layout.height - y0);
      if (layout.samples == 1) {
        const auto copy_bytes = copy_width * layout.bytes_per_sample;
        auto* dst_base = static_cast<std::byte*>(image.mutable_data(0));
        for (std::size_t y = 0; y < copy_height; ++y) {
          std::memcpy(dst_base + (static_cast<std::size_t>(y0) + y) * image.stride(0) + static_cast<std::size_t>(x0) * layout.bytes_per_sample,
                      buffer.data() + y * tile_row_bytes,
                      copy_bytes);
        }
      } else {
        auto* red = static_cast<std::uint8_t*>(image.mutable_data(0));
        auto* green = static_cast<std::uint8_t*>(image.mutable_data(1));
        auto* blue = static_cast<std::uint8_t*>(image.mutable_data(2));
        const auto* src = reinterpret_cast<const std::uint8_t*>(buffer.data());
        for (std::size_t y = 0; y < copy_height; ++y) {
          for (std::size_t x = 0; x < copy_width; ++x) {
            const auto src_offset = y * tile_row_bytes + x * 3;
            const auto row = static_cast<std::size_t>(y0) + y;
            const auto column = static_cast<std::size_t>(x0) + x;
            red[row * image.stride(0) + column] = src[src_offset];
            green[row * image.stride(1) + column] = src[src_offset + 1];
            blue[row * image.stride(2) + column] = src[src_offset + 2];
          }
        }
      }
    }
  }
}

}  // namespace

bool TiffCodec::can_decode(std::span<const std::byte> header) const noexcept {
  if (header.size() < 4) return false;
  const bool little = header[0] == std::byte{'I'} && header[1] == std::byte{'I'} &&
                      (header[2] == std::byte{42} || header[2] == std::byte{43}) && header[3] == std::byte{0};
  const bool big = header[0] == std::byte{'M'} && header[1] == std::byte{'M'} && header[2] == std::byte{0} &&
                   (header[3] == std::byte{42} || header[3] == std::byte{43});
  return little || big;
}

core::Image TiffCodec::decode(InputStream& stream, const DecodeOptions& options) const {
  TiffClient client;
  auto tif = open_for_read(stream, options.limits, client);
  const auto layout = read_layout(tif.get(), options.limits);
  auto image = layout.samples == 3
                   ? core::Image::create(layout.width, layout.height,
                                         std::vector<core::PixelType>(3, core::PixelType::UInt8))
                   : core::Image::create(layout.width, layout.height, layout.format);
  if (TIFFIsTiled(tif.get())) decode_tiles(tif.get(), image, layout, options.limits);
  else decode_strips(tif.get(), image, layout, options.limits);
  if (layout.photometric == PHOTOMETRIC_MINISWHITE) invert_miniswhite(image);
  return image;
}

void TiffCodec::encode(OutputStream& stream,
                       const core::Image& image,
                       const EncodeOptions&) const {
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "TiffCodec::encode", "image must not be empty");
  }
  const bool gray8 = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt8;
  const bool gray16 = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt16;
  const bool rgb = image.channel_count() == 3 &&
                   image.pixel_type(0) == core::PixelType::UInt8 &&
                   image.pixel_type(1) == core::PixelType::UInt8 &&
                   image.pixel_type(2) == core::PixelType::UInt8;
  if (!gray8 && !gray16 && !rgb) {
    throw core::Exception(core::Status::Unsupported, "TiffCodec::encode", "pixel format is not supported by TIFF encoder");
  }
  if (image.width() > std::numeric_limits<std::uint32_t>::max() || image.height() > std::numeric_limits<std::uint32_t>::max()) {
    throw core::Exception(core::Status::OutOfRange, "TiffCodec::encode", "image dimensions exceed TIFF tag range");
  }

  TiffClient client;
  auto tif = open_for_write(stream, client);
  const auto width = static_cast<std::uint32_t>(image.width());
  const auto height = static_cast<std::uint32_t>(image.height());
  const std::uint16_t bits = gray16 ? 16 : 8;
  const std::uint16_t samples = rgb ? 3 : 1;
  TIFFSetField(tif.get(), TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tif.get(), TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tif.get(), TIFFTAG_BITSPERSAMPLE, bits);
  TIFFSetField(tif.get(), TIFFTAG_SAMPLESPERPIXEL, samples);
  TIFFSetField(tif.get(), TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
  TIFFSetField(tif.get(), TIFFTAG_PHOTOMETRIC, rgb ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tif.get(), TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tif.get(), TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  TIFFSetField(tif.get(), TIFFTAG_COMPRESSION,
               TIFFIsCODECConfigured(COMPRESSION_LZW) ? COMPRESSION_LZW : COMPRESSION_NONE);
  const auto rows_per_strip = TIFFDefaultStripSize(tif.get(), 0);
  TIFFSetField(tif.get(), TIFFTAG_ROWSPERSTRIP, rows_per_strip);

  const auto row_bytes = image.width() * (rgb ? 3u : core::bytes_per_sample(image.pixel_type(0)));
  std::vector<std::byte> strip_buffer(static_cast<std::size_t>(rows_per_strip) * row_bytes);
  const auto* src_base = static_cast<const std::uint8_t*>(image.data(0));
  std::uint32_t strip_index = 0;
  for (std::size_t row_start = 0; row_start < image.height(); row_start += rows_per_strip, ++strip_index) {
    const auto rows = std::min<std::size_t>(rows_per_strip, image.height() - row_start);
    for (std::size_t y = 0; y < rows; ++y) {
      const auto* src = src_base + (row_start + y) * image.stride(0);
      auto* dst = reinterpret_cast<std::uint8_t*>(strip_buffer.data() + y * row_bytes);
      if (!rgb) {
        std::memcpy(dst, src, row_bytes);
      } else {
        const auto* green = static_cast<const std::uint8_t*>(image.data(1)) + (row_start + y) * image.stride(1);
        const auto* blue = static_cast<const std::uint8_t*>(image.data(2)) + (row_start + y) * image.stride(2);
        for (std::size_t x = 0; x < image.width(); ++x) {
          dst[x * 3 + 0] = src[x];
          dst[x * 3 + 1] = green[x];
          dst[x * 3 + 2] = blue[x];
        }
      }
    }
    const auto bytes = rows * row_bytes;
    if (bytes > static_cast<std::size_t>(std::numeric_limits<tmsize_t>::max()) ||
        TIFFWriteEncodedStrip(tif.get(), strip_index, strip_buffer.data(), static_cast<tmsize_t>(bytes)) < 0) {
      throw core::Exception(core::Status::EncodeError, "TiffCodec::encode", "failed to encode TIFF strip");
    }
  }
  if (!TIFFWriteDirectory(tif.get())) {
    throw core::Exception(core::Status::EncodeError, "TiffCodec::encode", "failed to finalize TIFF directory");
  }
}

}  // namespace mv::imageio
