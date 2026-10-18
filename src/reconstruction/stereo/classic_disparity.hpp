#pragma once
#include <core/image.hpp>
#include <core/tuple.hpp>
#include <cstdint>
namespace mv::reconstruction {
// 经典窗口相关，独立于 SGM/MS 内核。
struct ClassicDisparityParameters {
  enum class Method { Sad, Ssd, Ncc } method{Method::Ncc};
  std::int64_t mask_width{}, mask_height{}, minimum{}, maximum{}, levels{};
  double texture_threshold{}, score_threshold{};
  bool left_right_check{}, interpolation{};
};
struct ClassicDisparityResult {
  core::Image disparity, score;
};
[[nodiscard]] ClassicDisparityParameters classic_disparity_parameters(
    const core::Tuple& Method, const core::Tuple& MaskWidth, const core::Tuple& MaskHeight,
    const core::Tuple& TextureThresh, const core::Tuple& MinDisparity, const core::Tuple& MaxDisparity,
    const core::Tuple& NumLevels, const core::Tuple& ScoreThresh, const core::Tuple& Filter,
    const core::Tuple& SubDisparity);
[[nodiscard]] ClassicDisparityResult classic_binocular_disparity(
    const core::Image& left, const core::Image& right, const ClassicDisparityParameters& parameters);
}  // namespace mv::reconstruction
