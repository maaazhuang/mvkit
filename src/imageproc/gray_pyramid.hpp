#pragma once

#include <cstddef>

namespace mv::imageproc {

// 既有 Shape 2x2 灰度金字塔的共享标量尾部；坐标锚定 (2*row,2*column)，保持从左到右算术序。
template<class T>
inline T box2_gray(const T* row0,const T* row1,std::size_t column,std::size_t width) {
  const auto c0=2*column;
  const auto c1=c0+1<width ? c0+1 : c0;
  return T(.25)*(row0[c0]+row0[c1]+row1[c0]+row1[c1]);
}

} // namespace mv::imageproc
