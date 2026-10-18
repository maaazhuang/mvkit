#pragma once

#include <array>
#include <core/tuple.hpp>
#include <string_view>

namespace mv::calibration {

struct ProjectiveEstimate {
  core::Tuple homography;
  core::Tuple covariance;
};

// HALCON 契约区别于既有归一化/RMS 辅助函数：DLT 保留齐次尺度，
// gold_standard 使用完整 2x2 不确定度矩阵估计两组测量坐标。
[[nodiscard]] ProjectiveEstimate estimate_projective_transform(const core::Tuple& px, const core::Tuple& py,
                                                               const core::Tuple& qx, const core::Tuple& qy,
                                                               std::string_view method,
                                                               const std::array<core::Tuple, 6>& covariances);

}  // namespace mv::calibration
