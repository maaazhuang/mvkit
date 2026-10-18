#pragma once
#include <algorithm>
#include <cmath>

namespace mv::shape_matching {
// 延迟 greedy 剪枝，直至完美前缀也能满足请求阈值；固定 15% 前缀会在高 MinScore 拒绝完美匹配。
template<class T> T greedy_prefix(T count,T greediness,T threshold) {
  if(greediness<=0 || threshold<=0)return count;
  const T fraction=std::max(T(.15),(threshold-(T(1)-greediness))/greediness+T(.05));
  return std::min(count,std::max(T(8),std::ceil(count*fraction)));
}
}
