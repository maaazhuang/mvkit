#pragma once
#include <core/image.hpp>
#include <optional>

namespace mv::imageproc {
// 仅对已证明 Float32 舍入等价的整数小窗口返回结果，其他输入交还通用卷积。
std::optional<core::Image> exact_integer_box_mean(const core::Image&, std::size_t, std::size_t);
// byte 后处理：Float32 快算只在误差界不能跨越半值舍入时提交，否则用 Double。
void emphasize_byte_row(const std::uint8_t*,const float*,std::uint8_t*,std::size_t,double);
}
