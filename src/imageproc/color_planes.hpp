#pragma once
#include <core/image.hpp>
#include <array>
#include <optional>

namespace mv::imageproc {
// 完整 byte 平面的融合路径；其他类型、尺寸或域仍使用原通用实现。
std::optional<core::Image> try_transform_byte_planes(const std::array<core::Image,3>& source,
                                                     bool inverse);
std::size_t split_byte_interleaved(const std::uint8_t* source,std::uint8_t* red,
    std::uint8_t* green,std::uint8_t* blue,std::size_t count,std::size_t components,bool bgr);
}
