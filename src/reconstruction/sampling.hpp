#pragma once
#include <reconstruction/object_model3d.hpp>
namespace mv::reconstruction {
struct SamplingOptions {
  std::size_t minimum_points = 1;
  double maximum_normal_angle = 180.;
};
ObjectModel3DData sample_points(const ObjectModel3DData& model, std::string_view method, double parameter,
                                const SamplingOptions& options = {});
}  // namespace mv::reconstruction
