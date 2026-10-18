#include <mvkit/c/mv_image.h>
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "image_controls.hpp"
#include <imageproc/color_planes.hpp>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>

namespace {
using mv::c_api::TupleInput;
using mv::core::Image;
using mv::core::PixelType;
PixelType pixel_type(::MvTuple value, const char* op) {
  const TupleInput input(value, op, "Type");
  const auto name = mv::c_api::image_string(input);
  if (name == "byte") return PixelType::UInt8;
  if (name == "uint2") return PixelType::UInt16;
  if (name == "int2") return PixelType::Int16;
  if (name == "int4") return PixelType::Int32;
  if (name == "real") return PixelType::Float32;
  input.fail(mv::core::ErrorCode::UnsupportedValue, 0, "unsupported image type");
}
const char* type_name(PixelType type) {
  switch (type) {
    case PixelType::UInt8: return "byte";
    case PixelType::UInt16: return "uint2";
    case PixelType::Int16: return "int2";
    case PixelType::Int32: return "int4";
    case PixelType::Float32: return "real";
  }
  return "byte";
}
std::int64_t integer(::MvTuple value, const char* op, const char* parameter) {
  return TupleInput(value, op, parameter).scalar_integer();
}
std::size_t extent(::MvTuple value, const char* op, const char* parameter) {
  const auto number = integer(value, op, parameter);
  if (number <= 0 || static_cast<std::uint64_t>(number) > (std::numeric_limits<std::size_t>::max)())
    TupleInput(value, op, parameter).fail(mv::core::ErrorCode::InvalidValue, 0, "positive image size required");
  return static_cast<std::size_t>(number);
}
std::uintptr_t address(::MvTuple value, const char* op, const char* parameter, bool allow_zero = false) {
  const auto number = integer(value, op, parameter);
  const auto bits = static_cast<std::uint64_t>(number);
  if ((!allow_zero && bits == 0) || bits > (std::numeric_limits<std::uintptr_t>::max)())
    TupleInput(value, op, parameter).fail(mv::core::ErrorCode::InvalidValue, 0, "invalid pointer value");
  return static_cast<std::uintptr_t>(bits);
}
void copy_plane(Image& image, std::size_t channel, const void* pixels) {
  const auto bytes = image.width() * mv::core::bytes_per_sample(image.pixel_type(channel));
  auto* destination = static_cast<std::byte*>(image.mutable_data(channel));
  const auto* source = static_cast<const std::byte*>(pixels);
  for (std::size_t row = 0; row < image.height(); ++row)
    std::memcpy(destination + row * image.stride(channel), source + row * bytes, bytes);
}
void image_input(::MvObject value, const char* op) {
  mv::c_api::require_iconic(value, op, "Image");
  value->value.require_kind(mv::core::IconicKind::Image, op);
}
}

extern "C" ::MvStatus mv_gen_image_const(::MvObject* ImageOut, ::MvTuple Type, ::MvTuple Width, ::MvTuple Height) {
  return mv::c_api::translate([&] {
    constexpr auto op = "gen_image_const";
    mv::c_api::require_output(ImageOut, op, "Image");
    auto result = mv::c_api::make_image_handle(Image::create(extent(Width, op, "Width"), extent(Height, op, "Height"), pixel_type(Type, op)));
    *ImageOut = result.release();
  });
}
extern "C" ::MvStatus mv_gen_image1(::MvObject* ImageOut, ::MvTuple Type, ::MvTuple Width, ::MvTuple Height, ::MvTuple PixelPointer) {
  return mv::c_api::translate([&] {
    constexpr auto op = "gen_image1";
    mv::c_api::require_output(ImageOut, op, "Image");
    const auto pointer = address(PixelPointer, op, "PixelPointer");
    auto image = Image::create(extent(Width, op, "Width"), extent(Height, op, "Height"), pixel_type(Type, op));
    copy_plane(image, 0, reinterpret_cast<const void*>(pointer));
    auto result = mv::c_api::make_image_handle(std::move(image));
    *ImageOut = result.release();
  });
}
extern "C" ::MvStatus mv_gen_image3(::MvObject* ImageRGB, ::MvTuple Type, ::MvTuple Width, ::MvTuple Height, ::MvTuple PixelPointerRed, ::MvTuple PixelPointerGreen, ::MvTuple PixelPointerBlue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "gen_image3";
    mv::c_api::require_output(ImageRGB, op, "ImageRGB");
    const std::array types{pixel_type(Type, op), pixel_type(Type, op), pixel_type(Type, op)};
    const std::array pointers{address(PixelPointerRed, op, "PixelPointerRed"), address(PixelPointerGreen, op, "PixelPointerGreen"), address(PixelPointerBlue, op, "PixelPointerBlue")};
    auto image = Image::create(extent(Width, op, "Width"), extent(Height, op, "Height"), types);
    for (std::size_t channel = 0; channel < 3; ++channel) copy_plane(image, channel, reinterpret_cast<const void*>(pointers[channel]));
    auto result = mv::c_api::make_image_handle(std::move(image));
    *ImageRGB = result.release();
  });
}
extern "C" ::MvStatus mv_gen_image1_extern(::MvObject* ImageOut, ::MvTuple Type, ::MvTuple Width, ::MvTuple Height, ::MvTuple PixelPointer, ::MvTuple ClearProc) {
  return mv::c_api::translate([&] {
    constexpr auto op = "gen_image1_extern";
    mv::c_api::require_output(ImageOut, op, "Image");
    const auto type = pixel_type(Type, op);
    const auto width = extent(Width, op, "Width"), height = extent(Height, op, "Height");
    const auto bytes = mv::core::bytes_per_sample(type);
    if (width > (std::numeric_limits<std::size_t>::max)() / bytes)
      TupleInput(Width, op, "Width").fail(mv::core::ErrorCode::InvalidValue, 0, "image row size overflow");
    auto* pointer = reinterpret_cast<void*>(address(PixelPointer, op, "PixelPointer"));
    const auto callback = reinterpret_cast<void (*)(void*)>(address(ClearProc, op, "ClearProc", true));
    // 全部构造成功后才接管清理回调；回调接收原像素指针，没有私有上下文参数。
    auto armed = std::make_shared<std::atomic<bool>>(false);
    std::shared_ptr<void> owner(pointer, [callback, armed](void* pixels) {
      if (callback && armed->load(std::memory_order_acquire)) callback(pixels);
    });
    auto result = mv::c_api::make_image_handle(Image::wrap(width, height, type, pointer, width * bytes, std::move(owner)));
    armed->store(true, std::memory_order_release);
    *ImageOut = result.release();
  });
}
extern "C" ::MvStatus mv_get_image_pointer1(::MvObject ImageIn, ::MvTuple* Pointer, ::MvTuple* Type, ::MvTuple* Width, ::MvTuple* Height) {
  return mv::c_api::translate([&] {
    constexpr auto op = "get_image_pointer1";
    image_input(ImageIn, op);
    mv::c_api::ControlOutputs<4> out(op, {Pointer, Type, Width, Height});
    for (std::size_t i = 0; i < ImageIn->value.size(); ++i) {
      const auto& image = ImageIn->value.at(i).as_image();
      if (image.stride() != image.width() * mv::core::bytes_per_sample(image.pixel_type()))
        throw mv::core::Exception(mv::core::Status::Unsupported, mv::core::ErrorCode::UnsupportedValue, op, "Image", static_cast<std::int64_t>(i), "pointer protocol requires contiguous rows");
      out[0].append(static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(image.data())));
      out[1].append(type_name(image.pixel_type()));
      out[2].append(static_cast<std::int64_t>(image.width()));
      out[3].append(static_cast<std::int64_t>(image.height()));
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_get_image_pointer3(::MvObject ImageRGB, ::MvTuple* PointerRed, ::MvTuple* PointerGreen, ::MvTuple* PointerBlue, ::MvTuple* Type, ::MvTuple* Width, ::MvTuple* Height) {
  return mv::c_api::translate([&] {
    constexpr auto op = "get_image_pointer3";
    image_input(ImageRGB, op);
    mv::c_api::ControlOutputs<6> out(op, {PointerRed, PointerGreen, PointerBlue, Type, Width, Height});
    for (std::size_t i = 0; i < ImageRGB->value.size(); ++i) {
      const auto& image = ImageRGB->value.at(i).as_image();
      if (image.channel_count() != 3) throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidCardinality, op, "ImageRGB", static_cast<std::int64_t>(i), "three channels required");
      for (std::size_t channel = 0; channel < 3; ++channel) {
        if (image.pixel_type(channel) != image.pixel_type() || image.stride(channel) != image.width() * mv::core::bytes_per_sample(image.pixel_type(channel)))
          throw mv::core::Exception(mv::core::Status::TypeMismatch, mv::core::ErrorCode::InvalidType, op, "ImageRGB", static_cast<std::int64_t>(i), "uniform contiguous channel type required");
        out[channel].append(static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(image.data(channel))));
      }
      out[3].append(type_name(image.pixel_type()));out[4].append(static_cast<std::int64_t>(image.width()));out[5].append(static_cast<std::int64_t>(image.height()));
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_get_image_type(::MvObject ImageIn, ::MvTuple* Type) {
  return mv::c_api::translate([&] {
    constexpr auto op = "get_image_type";image_input(ImageIn, op);
    mv::c_api::ControlOutputs<1> out(op, {Type});
    for (std::size_t i = 0; i < ImageIn->value.size(); ++i) out[0].append(type_name(ImageIn->value.at(i).as_image().pixel_type()));
    out.Commit();
  });
}

extern "C" ::MvStatus mv_gen_image_interleaved(::MvObject* ImageRGB, ::MvTuple PixelPointer, ::MvTuple ColorFormat, ::MvTuple OriginalWidth, ::MvTuple OriginalHeight, ::MvTuple Alignment, ::MvTuple Type, ::MvTuple ImageWidth, ::MvTuple ImageHeight, ::MvTuple StartRow, ::MvTuple StartColumn, ::MvTuple BitsPerChannel, ::MvTuple BitShift) {
  return mv::c_api::translate([&] {
    constexpr auto op = "gen_image_interleaved";
    mv::c_api::require_output(ImageRGB, op, "ImageRGB");
    const auto* pixels = reinterpret_cast<const std::uint8_t*>(address(PixelPointer, op, "PixelPointer"));
    const auto format = mv::c_api::image_string(TupleInput(ColorFormat, op, "ColorFormat"));
    if (format != "rgb" && format != "bgr" && format != "rgbx" && format != "bgrx")
      TupleInput(ColorFormat, op, "ColorFormat").fail(mv::core::ErrorCode::UnsupportedValue, 0, "unsupported packed color format");
    if (pixel_type(Type, op) != PixelType::UInt8)
      TupleInput(Type, op, "Type").fail(mv::core::ErrorCode::UnsupportedValue, 0, "packed conversion supports byte output");
    const auto bits = integer(BitsPerChannel, op, "BitsPerChannel");
    if ((bits != -1 && bits != 8) || integer(BitShift, op, "BitShift") != 0)
      TupleInput(BitsPerChannel, op, "BitsPerChannel").fail(mv::core::ErrorCode::UnsupportedValue, 0, "only full 8-bit channels without bit shift are supported");
    (void)integer(Alignment, op, "Alignment"); // HALCON 的保留参数不定义私有行步长协议。
    const auto width = extent(OriginalWidth, op, "OriginalWidth"), height = extent(OriginalHeight, op, "OriginalHeight");
    auto out_width = integer(ImageWidth, op, "ImageWidth"), out_height = integer(ImageHeight, op, "ImageHeight");
    auto row = integer(StartRow, op, "StartRow"), column = integer(StartColumn, op, "StartColumn");
    if (out_width == 0) out_width = static_cast<std::int64_t>(width);
    if (out_height == 0) out_height = static_cast<std::int64_t>(height);
    if (out_width <= 0 || out_height <= 0 || row < 0 || column < 0 || static_cast<std::uint64_t>(column) > width || static_cast<std::uint64_t>(row) > height || static_cast<std::uint64_t>(out_width) > width - column || static_cast<std::uint64_t>(out_height) > height - row)
      TupleInput(ImageWidth, op, "ImageWidth").fail(mv::core::ErrorCode::InvalidValue, 0, "crop is outside source image");
    const std::size_t components = format.ends_with("x") ? 4 : 3;
    if (width > (std::numeric_limits<std::size_t>::max)() / components || height > (std::numeric_limits<std::size_t>::max)() / (width * components))
      TupleInput(OriginalWidth, op, "OriginalWidth").fail(mv::core::ErrorCode::InvalidValue, 0, "packed image size overflow");
    const std::array types{PixelType::UInt8, PixelType::UInt8, PixelType::UInt8};
    // 已验证的裁剪逐像素覆盖三个完整平面，无需分配时先清零。
    auto image = Image::create_uninitialized(static_cast<std::size_t>(out_width), static_cast<std::size_t>(out_height), types);
    const bool bgr = format.starts_with("bgr");
    std::array<std::uint8_t*, 3> planes;
    std::array<std::size_t, 3> strides;
    for (std::size_t ch = 0; ch < 3; ++ch) {
      planes[ch] = static_cast<std::uint8_t*>(image.mutable_data(ch));
      strides[ch] = image.stride(ch);
    }
    // 格式只分派一次；每像素的组件步长与 RGB/BGR 索引为编译期常量。
    const auto copy = [&]<std::size_t Components, bool Bgr>() {
      for (std::size_t r = 0; r < image.height(); ++r) {
        const auto* input = pixels + ((r + row) * width + column) * Components;
        auto* red = planes[0] + r * strides[0];
        auto* green = planes[1] + r * strides[1];
        auto* blue = planes[2] + r * strides[2];
        const auto first=mv::imageproc::split_byte_interleaved(input,red,green,blue,image.width(),Components,Bgr);
        for (std::size_t c = first; c < image.width(); ++c) {
          red[c] = input[c * Components + (Bgr ? 2 : 0)];
          green[c] = input[c * Components + 1];
          blue[c] = input[c * Components + (Bgr ? 0 : 2)];
        }
      }
    };
    if (components == 3) {
      if (bgr) copy.template operator()<3, true>(); else copy.template operator()<3, false>();
    } else {
      if (bgr) copy.template operator()<4, true>(); else copy.template operator()<4, false>();
    }
    auto result = mv::c_api::make_image_handle(std::move(image));
    *ImageRGB = result.release();
  });
}

extern "C" ::MvStatus mv_get_image_size(::MvObject ImageIn, ::MvTuple* Width, ::MvTuple* Height) {
  return mv::c_api::translate([&] {
    constexpr auto op = "get_image_size"; image_input(ImageIn, op);
    mv::c_api::ControlOutputs<2> out(op, {Width, Height});
    for (std::size_t i = 0; i < ImageIn->value.size(); ++i) {
      const auto& image = ImageIn->value.at(i).as_image();
      out[0].append(static_cast<std::int64_t>(image.width()));
      out[1].append(static_cast<std::int64_t>(image.height()));
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_count_channels(::MvObject MultiChannelImage, ::MvTuple* Channels) {
  return mv::c_api::translate([&] {
    constexpr auto op = "count_channels"; image_input(MultiChannelImage, op);
    mv::c_api::ControlOutputs<1> out(op, {Channels});
    for (std::size_t i = 0; i < MultiChannelImage->value.size(); ++i)
      out[0].append(static_cast<std::int64_t>(MultiChannelImage->value.at(i).as_image().channel_count()));
    out.Commit();
  });
}
