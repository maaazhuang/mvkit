#pragma once
#include <cstdint>
#include <vector>
#include <core/image.hpp>
#include <core/tuple.hpp>
namespace mv::reconstruction {
struct DisparityResult {
  core::Image disparity; core::Image score;
  // 质量布局：confidence 为 Float32，validity 为 UInt8(255)，reason 为 UInt16 位掩码。
  std::vector<core::Image> quality_channels;
  double disparity_stddev{}; std::size_t hole_max_width{}; double hole_max_depth_jump{.01};
};
[[nodiscard]] DisparityResult binocular_disparity_ms(const core::Image& image_rect1,const core::Image& image_rect2,std::int64_t min_disparity,std::int64_t max_disparity,std::int64_t surface_smoothing,std::int64_t edge_smoothing,const core::Tuple& gen_param_name,const core::Tuple& gen_param_value);
[[nodiscard]] DisparityResult binocular_disparity(const core::Image& image_rect1,const core::Image& image_rect2,std::int64_t min_disparity,std::int64_t max_disparity,const core::Tuple& gen_param_name,const core::Tuple& gen_param_value);
}
