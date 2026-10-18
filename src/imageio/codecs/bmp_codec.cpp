#include <imageio/codecs/bmp_codec.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <core/exception.hpp>
#include <imageio/decode_limits.hpp>

namespace mv::imageio {
namespace {

[[noreturn]] void decode_error(const char* message) {
  throw core::Exception(core::Status::DecodeError, "BmpCodec::decode", message);
}

[[noreturn]] void encode_error(const char* message) {
  throw core::Exception(core::Status::EncodeError, "BmpCodec::encode", message);
}

void read_exact(InputStream& stream, void* dst, std::size_t bytes) {
  auto* out = static_cast<std::byte*>(dst);
  std::size_t done = 0;
  while (done < bytes) {
    const auto count = stream.read(out + done, bytes - done);
    if (count == 0) decode_error("unexpected end of BMP stream");
    done += count;
  }
}

void write_exact(OutputStream& stream, const void* src, std::size_t bytes) {
  const auto* input = static_cast<const std::byte*>(src);
  std::size_t done = 0;
  while (done < bytes) {
    const auto count = stream.write(input + done, bytes - done);
    if (count == 0) encode_error("failed to write BMP stream");
    done += count;
  }
}

std::uint16_t read_u16(InputStream& stream) {
  std::array<std::byte, 2> b{};
  read_exact(stream, b.data(), b.size());
  return static_cast<std::uint16_t>(std::to_integer<std::uint8_t>(b[0])) |
         static_cast<std::uint16_t>(std::to_integer<std::uint8_t>(b[1]) << 8);
}

std::uint32_t read_u32(InputStream& stream) {
  std::array<std::byte, 4> b{};
  read_exact(stream, b.data(), b.size());
  return static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(b[0])) |
         (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(b[1])) << 8) |
         (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(b[2])) << 16) |
         (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(b[3])) << 24);
}

std::int32_t read_i32(InputStream& stream) {
  return static_cast<std::int32_t>(read_u32(stream));
}

void write_u16(OutputStream& stream, std::uint16_t value) {
  const std::array<std::byte, 2> b{
      static_cast<std::byte>(value & 0xff),
      static_cast<std::byte>((value >> 8) & 0xff)};
  write_exact(stream, b.data(), b.size());
}

void write_u32(OutputStream& stream, std::uint32_t value) {
  const std::array<std::byte, 4> b{
      static_cast<std::byte>(value & 0xff),
      static_cast<std::byte>((value >> 8) & 0xff),
      static_cast<std::byte>((value >> 16) & 0xff),
      static_cast<std::byte>((value >> 24) & 0xff)};
  write_exact(stream, b.data(), b.size());
}

std::size_t bmp_row_stride(std::size_t width, std::uint16_t bits_per_pixel) {
  const auto bits = checked_multiply(width, static_cast<std::size_t>(bits_per_pixel), "BmpCodec");
  if (bits > std::numeric_limits<std::size_t>::max() - 31) {
    throw core::Exception(core::Status::OutOfRange, "BmpCodec", "BMP row size overflow");
  }
  return ((bits + 31) / 32) * 4;
}

std::uint32_t checked_u32(std::size_t value, const char* message) {
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw core::Exception(core::Status::OutOfRange, "BmpCodec::encode", message);
  }
  return static_cast<std::uint32_t>(value);
}

struct PaletteEntry {
  std::uint8_t blue{};
  std::uint8_t green{};
  std::uint8_t red{};
};

}  // namespace

bool BmpCodec::can_decode(std::span<const std::byte> header) const noexcept {
  return header.size() >= 2 && header[0] == std::byte{'B'} && header[1] == std::byte{'M'};
}

core::Image BmpCodec::decode(InputStream& stream, const DecodeOptions& options) const {
  if (!stream.seek(0)) decode_error("cannot seek BMP stream to beginning");

  std::array<std::byte, 2> signature{};
  read_exact(stream, signature.data(), signature.size());
  if (!can_decode(signature)) decode_error("invalid BMP signature");

  const auto declared_file_size = read_u32(stream);
  (void)read_u16(stream);
  (void)read_u16(stream);
  const auto pixel_offset = read_u32(stream);
  const auto dib_size = read_u32(stream);
  if (dib_size != 40) {
    throw core::Exception(core::Status::Unsupported, "BmpCodec::decode", "only 40-byte BITMAPINFOHEADER is supported");
  }

  const auto signed_width = read_i32(stream);
  const auto signed_height = read_i32(stream);
  const auto planes = read_u16(stream);
  const auto bits_per_pixel = read_u16(stream);
  const auto compression = read_u32(stream);
  (void)read_u32(stream);  // 图像尺寸。
  (void)read_i32(stream);  // x 方向每米像素数。
  (void)read_i32(stream);  // y 方向每米像素数。
  const auto colors_used = read_u32(stream);
  (void)read_u32(stream);  // 重要颜色数量。

  if (signed_width <= 0 || signed_height == 0 || signed_height == std::numeric_limits<std::int32_t>::min()) {
    decode_error("invalid BMP dimensions");
  }
  if (planes != 1) decode_error("BMP planes must equal one");
  if (compression != 0) {
    throw core::Exception(core::Status::Unsupported, "BmpCodec::decode", "compressed BMP is not supported");
  }
  if (bits_per_pixel != 8 && bits_per_pixel != 24 && bits_per_pixel != 32) {
    throw core::Exception(core::Status::Unsupported, "BmpCodec::decode", "BMP bit depth is not supported");
  }

  const auto width = static_cast<std::size_t>(signed_width);
  const auto height = static_cast<std::size_t>(signed_height < 0 ? -static_cast<std::int64_t>(signed_height)
                                                                 : signed_height);
  const bool top_down = signed_height < 0;
  const auto row_stride = bmp_row_stride(width, bits_per_pixel);
  const auto pixel_bytes = checked_multiply(row_stride, height, "BmpCodec::decode");
  if (static_cast<std::uint64_t>(pixel_offset) + pixel_bytes > stream.size()) {
    decode_error("BMP pixel data is truncated or outside the stream");
  }
  if (declared_file_size != 0 && declared_file_size > stream.size()) {
    decode_error("declared BMP file size exceeds stream size");
  }

  std::vector<PaletteEntry> palette;
  bool grayscale_palette = true;
  if (bits_per_pixel == 8) {
    const std::size_t palette_count = colors_used == 0 ? 256u : static_cast<std::size_t>(colors_used);
    if (palette_count == 0 || palette_count > 256) decode_error("invalid BMP palette size");
    if (54u + palette_count * 4u > pixel_offset) decode_error("BMP palette overlaps pixel data");
    if (!stream.seek(54)) decode_error("cannot seek to BMP palette");
    palette.resize(palette_count);
    for (auto& entry : palette) {
      std::array<std::byte, 4> raw{};
      read_exact(stream, raw.data(), raw.size());
      entry.blue = std::to_integer<std::uint8_t>(raw[0]);
      entry.green = std::to_integer<std::uint8_t>(raw[1]);
      entry.red = std::to_integer<std::uint8_t>(raw[2]);
      grayscale_palette = grayscale_palette && entry.blue == entry.green && entry.green == entry.red;
    }
  }

  std::size_t output_bpp = 3;
  if (bits_per_pixel == 8 && grayscale_palette) {
    output_bpp = 1;
  }
  (void)checked_image_bytes(width, height, output_bpp, options.limits);
  const core::PixelType rgb_types[]{core::PixelType::UInt8, core::PixelType::UInt8,
                                    core::PixelType::UInt8};
  auto image = output_bpp == 1 ? core::Image::create(width, height, core::PixelType::UInt8)
                               : core::Image::create(width, height, rgb_types);

  std::vector<std::byte> row(row_stride);
  // 文件行按物理顺序连续读取；底部向上的方向只影响目标行，无须逐行重新定位。
  if (!stream.seek(pixel_offset)) decode_error("cannot seek to BMP pixel row");
  bool identity_palette = bits_per_pixel == 8 && grayscale_palette && palette.size() == 256;
  if (identity_palette)
    for (std::size_t i = 0; i < palette.size(); ++i) identity_palette = identity_palette && palette[i].red == i;
  for (std::size_t file_row = 0; file_row < height; ++file_row) {
    read_exact(stream, row.data(), row.size());
    const std::size_t destination_row = top_down ? file_row : (height - 1 - file_row);
    auto* dst = static_cast<std::uint8_t*>(image.mutable_data(0)) + destination_row * image.stride(0);
    auto* green = output_bpp == 3 ? static_cast<std::uint8_t*>(image.mutable_data(1)) + destination_row * image.stride(1) : nullptr;
    auto* blue = output_bpp == 3 ? static_cast<std::uint8_t*>(image.mutable_data(2)) + destination_row * image.stride(2) : nullptr;

    if (bits_per_pixel == 24) {
      const auto* src = reinterpret_cast<const std::uint8_t*>(row.data());
      for (std::size_t x = 0; x < width; ++x) {
        dst[x] = src[x * 3 + 2]; green[x] = src[x * 3 + 1]; blue[x] = src[x * 3];
      }
    } else if (bits_per_pixel == 32) {
      for (std::size_t x = 0; x < width; ++x) {
        const auto* src = reinterpret_cast<const std::uint8_t*>(row.data()) + x * 4;
        dst[x] = src[2]; green[x] = src[1]; blue[x] = src[0];
      }
    } else if (identity_palette) {
      std::memcpy(dst, row.data(), width);
    } else if (grayscale_palette) {
      for (std::size_t x = 0; x < width; ++x) {
        const auto index = std::to_integer<std::uint8_t>(row[x]);
        if (index >= palette.size()) decode_error("BMP pixel references palette entry outside palette");
        dst[x] = palette[index].red;
      }
    } else {
      for (std::size_t x = 0; x < width; ++x) {
        const auto index = std::to_integer<std::uint8_t>(row[x]);
        if (index >= palette.size()) decode_error("BMP pixel references palette entry outside palette");
        const auto& color = palette[index];
        dst[x] = color.red; green[x] = color.green; blue[x] = color.blue;
      }
    }
  }
  return image;
}

void BmpCodec::encode(OutputStream& stream,
                      const core::Image& image,
                      const EncodeOptions&) const {
  if (image.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "BmpCodec::encode", "image must not be empty");
  }
  const bool gray = image.channel_count() == 1 && image.pixel_type(0) == core::PixelType::UInt8;
  const bool rgb = image.channel_count() == 3 && image.pixel_type(0) == core::PixelType::UInt8 &&
                   image.pixel_type(1) == core::PixelType::UInt8 && image.pixel_type(2) == core::PixelType::UInt8;
  if (!gray && !rgb) {
    throw core::Exception(core::Status::Unsupported, "BmpCodec::encode", "BMP requires one or three UInt8 channels");
  }
  if (image.width() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) ||
      image.height() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    throw core::Exception(core::Status::OutOfRange, "BmpCodec::encode", "BMP dimensions exceed 32-bit header range");
  }

  const std::uint16_t bits_per_pixel = gray ? 8 : 24;
  const auto row_stride = bmp_row_stride(image.width(), bits_per_pixel);
  const auto pixel_bytes = checked_multiply(row_stride, image.height(), "BmpCodec::encode");
  const std::size_t palette_bytes = gray ? 256u * 4u : 0u;
  const std::size_t pixel_offset = 14u + 40u + palette_bytes;
  if (pixel_offset > std::numeric_limits<std::size_t>::max() - pixel_bytes) {
    throw core::Exception(core::Status::OutOfRange, "BmpCodec::encode", "BMP file size overflow");
  }
  const std::size_t file_size = pixel_offset + pixel_bytes;

  if (!stream.seek(0)) encode_error("cannot seek output stream to beginning");
  const std::array<std::byte, 2> signature{std::byte{'B'}, std::byte{'M'}};
  write_exact(stream, signature.data(), signature.size());
  write_u32(stream, checked_u32(file_size, "BMP file exceeds 4 GiB header limit"));
  write_u16(stream, 0); write_u16(stream, 0);
  write_u32(stream, checked_u32(pixel_offset, "BMP pixel offset exceeds 32-bit header range"));
  write_u32(stream, 40);
  write_u32(stream, static_cast<std::uint32_t>(image.width()));
  write_u32(stream, static_cast<std::uint32_t>(image.height()));
  write_u16(stream, 1);
  write_u16(stream, bits_per_pixel);
  write_u32(stream, 0);
  write_u32(stream, checked_u32(pixel_bytes, "BMP pixel data exceeds 32-bit header range"));
  write_u32(stream, 0); write_u32(stream, 0);
  write_u32(stream, gray ? 256 : 0); write_u32(stream, 0);

  if (gray) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      const std::array<std::byte, 4> entry{
          static_cast<std::byte>(i), static_cast<std::byte>(i), static_cast<std::byte>(i), std::byte{0}};
      write_exact(stream, entry.data(), entry.size());
    }
  }

  std::vector<std::byte> row(row_stride, std::byte{0});
  const auto* red = static_cast<const std::uint8_t*>(image.data(0));
  const auto* green = rgb ? static_cast<const std::uint8_t*>(image.data(1)) : nullptr;
  const auto* blue = rgb ? static_cast<const std::uint8_t*>(image.data(2)) : nullptr;
  for (std::size_t file_row = 0; file_row < image.height(); ++file_row) {
    const std::size_t source_row = image.height() - 1 - file_row;
    const auto* src = red + source_row * image.stride(0);
    std::fill(row.begin(), row.end(), std::byte{0});
    if (gray) {
      std::memcpy(row.data(), src, image.width());
    } else {
      auto* dst = reinterpret_cast<std::uint8_t*>(row.data());
      const auto* g = green + source_row * image.stride(1);
      const auto* b = blue + source_row * image.stride(2);
      for (std::size_t x = 0; x < image.width(); ++x) {
        dst[x * 3 + 0] = b[x]; dst[x * 3 + 1] = g[x]; dst[x * 3 + 2] = src[x];
      }
    }
    write_exact(stream, row.data(), row.size());
  }
}

}  // namespace mv::imageio
