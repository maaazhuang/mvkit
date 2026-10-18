#pragma once

#include <cstddef>
#include <vector>

namespace mv::shape_matching {

enum class CoarseAnglePhase { Full, Primary, Complementary };

// 划分既有稠密晶格，包含可能不规则的末端点；两阶段合计每个原角度恰访问一次。
inline std::vector<double> select_angle_phase(std::vector<double> angles, CoarseAnglePhase phase) {
  if (phase == CoarseAnglePhase::Full) return angles;
  std::size_t used = 0;
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const bool primary = i % 2 == 0 || i + 1 == angles.size();
    if (primary == (phase == CoarseAnglePhase::Primary)) angles[used++] = angles[i];
  }
  angles.resize(used);
  return angles;
}

}  // namespace mv::shape_matching
