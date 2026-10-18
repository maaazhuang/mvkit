#pragma once
#include <reconstruction/object_model3d.hpp>
namespace mv::reconstruction {
struct NormalEstimationOptions {
  std::size_t neighbors = 60;
  int order = 2;
  double relative_sigma = 1.;
  double absolute_sigma = 0.;
  bool force_inwards = true;
};
// 正式方法保持独立，不改变旧可配置 kNN/radius 工作流。
ObjectModel3DData estimate_normals(const ObjectModel3DData& model, std::string_view method,
                                   const NormalEstimationOptions& options = {});
}  // namespace mv::reconstruction
