#pragma once

#include <core/iconic_tuple.hpp>
#include <core/tuple.hpp>

namespace mv::reconstruction {

struct StereoDisparityImages {
  core::IconicTuple disparity;
  core::IconicTuple score;
};

struct StereoDisparityQualityImages {
  core::IconicTuple disparity;
  core::IconicTuple score;
  core::IconicTuple confidence;
  core::IconicTuple validity;
  core::IconicTuple reason;
};

// HALCON 风格控制协议供公开 C/C++ 适配共享；数值兼容差异与 API 符合性分开登记。
[[nodiscard]] StereoDisparityImages binocular_disparity_ms_protocol(
    const core::IconicTuple& left, const core::IconicTuple& right, const core::Tuple& minimum,
    const core::Tuple& maximum, const core::Tuple& surface, const core::Tuple& edge, const core::Tuple& names,
    const core::Tuple& values);

// MVKit 扩展采用正式 MS 控制及显式置信阈值；诊断图保留完整栅格域，包含输入 ROI 无效像素。
[[nodiscard]] StereoDisparityQualityImages binocular_disparity_ms_with_quality_protocol(
    const core::IconicTuple& left, const core::IconicTuple& right, const core::Tuple& minimum,
    const core::Tuple& maximum, const core::Tuple& surface, const core::Tuple& edge, const core::Tuple& names,
    const core::Tuple& values, const core::Tuple& min_confidence);

}  // namespace mv::reconstruction
