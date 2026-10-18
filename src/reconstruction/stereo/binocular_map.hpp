#pragma once
#include <array>
#include <core/image.hpp>
#include <core/tuple.hpp>
#include <string_view>

namespace mv::reconstruction {
struct BinocularMaps {
  std::array<core::Image, 2> maps;
  std::array<core::Tuple, 2> cameras;
  std::array<core::Tuple, 2> camera_poses;
  core::Tuple relative_pose;
};
// 显式编码 Map 和控制值，独立于旧缓存。
[[nodiscard]] BinocularMaps binocular_rectification_maps(const core::Tuple& camera1,
                                                         const core::Tuple& camera2,
                                                         const core::Tuple& relative_pose, double subsampling,
                                                         std::string_view method, std::string_view map_type);
}  // namespace mv::reconstruction
