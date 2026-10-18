#pragma once
#include "tuple_input.hpp"
#include <cmath>
#include <cstdint>
#include <limits>

namespace mv::c_api {
inline std::string_view image_string(const TupleInput& input) {
  if(input.size()!=1)input.fail(core::ErrorCode::InvalidCardinality,-1,"exactly one string required");
  return input.string(0);
}
// HALCON 图像坐标/尺寸的半值向 +infinity 取整；整数输入保持精确，不将 int64 经 double 转换。
inline std::int64_t image_integer(const TupleInput& input,std::size_t index=0) {
  const auto value=input.at(index);
  if(value.kind()==core::ValueKind::Int64)return value.as_int64();
  if(value.kind()!=core::ValueKind::Real64)input.fail(core::ErrorCode::InvalidType,index,"numeric image control required");
  const auto number=std::floor(value.as_real64()+0.5);
  if(!std::isfinite(number)||number < -9223372036854775808.0 || number >= 9223372036854775808.0)
    input.fail(core::ErrorCode::InvalidValue,index,"image control is outside int64 range");
  return static_cast<std::int64_t>(number);
}
inline std::int64_t image_scalar_integer(const TupleInput& input) {
  if(input.size()!=1)input.fail(core::ErrorCode::InvalidCardinality,-1,"exactly one numeric control required");
  return image_integer(input);
}
}
