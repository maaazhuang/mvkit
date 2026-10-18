#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>

#include <core/exception.hpp>
#include <core/image.hpp>

namespace mv::imageproc {

template <typename F>
decltype(auto) dispatch_scalar(mv::core::PixelType format, F&& fn) {
  using mv::core::PixelType;
  switch (format) {
    case PixelType::UInt8: return std::forward<F>(fn)(std::type_identity<std::uint8_t>{});
    case PixelType::UInt16: return std::forward<F>(fn)(std::type_identity<std::uint16_t>{});
    case PixelType::Int16: return std::forward<F>(fn)(std::type_identity<std::int16_t>{});
    case PixelType::Int32: return std::forward<F>(fn)(std::type_identity<std::int32_t>{});
    case PixelType::Float32: return std::forward<F>(fn)(std::type_identity<float>{});
  }
  throw mv::core::Exception(mv::core::Status::Unsupported, "dispatch_scalar", "unsupported pixel type");
}

}  // namespace mv::imageproc
