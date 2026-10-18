#pragma once
#include <array>
#include <optional>
#include <reconstruction/object_model3d.hpp>
#include <string_view>
namespace mv::reconstruction {
struct ObjectDistanceOptions {
  std::string_view distance_to = "auto", method = "auto";
  double maximum = 0;
  bool signed_distances = false, store_closest_index = false;
  std::optional<std::array<double, 12>> pose;
};
struct ObjectDistanceResult {
  std::pmr::vector<double> distances{runtime::scratch_resource()}, closest{runtime::scratch_resource()};
};
ObjectDistanceResult object_distances(const ObjectModel3DData& source, const ObjectModel3DData& target,
                                      const ObjectDistanceOptions&,
                                      const NeighborhoodIndex* cached = nullptr);
}  // namespace mv::reconstruction
