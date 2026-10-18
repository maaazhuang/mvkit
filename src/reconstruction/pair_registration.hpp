#pragma once
#include <reconstruction/object_model3d.hpp>
namespace mv::reconstruction {
enum class PairRegistrationPreset { Fast, Accurate, Robust };
struct PairRegistrationOptions {
  PairRegistrationPreset preset = PairRegistrationPreset::Accurate;
  double sampling_distance[2]{.05, .05};
  double key_point_fraction[2]{.3, .3};
  std::size_t refinement_steps = 5, sub_sampling = 2;
  double relative_distance_threshold = .1, absolute_distance_threshold = 0;
  bool invert_normals = false, absolute_threshold_set = false;
};
struct PairRegistrationResult {
  core::Tuple pose, score;
};
[[nodiscard]] PairRegistrationResult register_pair(const ObjectModel3DData& first,
                                                   const ObjectModel3DData& second, std::string_view method,
                                                   const PairRegistrationOptions& options);
}  // namespace mv::reconstruction
