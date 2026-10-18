#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <core/exception.hpp>
#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {
// 仅表示可观测失败，不宣称由某种材料导致。
enum QualityReason : std::uint16_t {
  OutsideDomain=1, NoCorrespondence=2, Inconsistent=4, LowContrast=8,
  WeakPhase=16, GeometryFailure=32, Ambiguous=64, CoordinateOutOfRange=128, Saturated=256
};
inline void allocate_quality(ObjectModel3DData& model) {
  const auto n=model.points.size();
  model.observed_valid.assign(n,0); model.filled.assign(n,0);
  model.quality_reason.assign(n,NoCorrespondence);
  model.depth_uncertainty.assign(n,std::numeric_limits<float>::quiet_NaN());
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for(auto& point:model.points)point={nan,nan,nan};
}
// 仅补两侧原始观测支撑的有界水平孔洞，不迭代传播、扩边、填域或修复几何。
inline void fill_quality_holes(ObjectModel3DData& model,std::size_t max_gap,double max_depth_jump) {
  if(!max_gap || !model.width || model.observed_valid.empty())return;
  for(std::size_t row=0;row<model.height;++row) {
    const auto base=row*model.width;
    for(std::size_t col=0;col<model.width;) {
      if(model.valid[base+col]){++col;continue;}
      const auto first=col; bool allowed=true;
      while(col<model.width && !model.valid[base+col]) {
        allowed=allowed && !(model.quality_reason[base+col] &
          (OutsideDomain|Inconsistent|GeometryFailure|CoordinateOutOfRange|Saturated)); ++col;
      }
      const auto gap=col-first;
      if(!allowed || !first || col==model.width || gap>max_gap ||
         !model.observed_valid[base+first-1] || !model.observed_valid[base+col])continue;
      const auto a=model.points[base+first-1],b=model.points[base+col];
      if(std::abs(a.z-b.z)>max_depth_jump)continue;
      for(std::size_t k=0;k<gap;++k) {
        const double t=double(k+1)/double(gap+1);const auto i=base+first+k;
        model.points[i]={a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
        model.valid[i]=1;model.filled[i]=1;model.confidence[i]=0;
        // 插值不构成新观测，sigma 仍未知。
      }
    }
  }
}
template<class Function> float depth_sigma(Function&& evaluate,double coordinate,double sigma) {
  if(!(sigma>0))return std::numeric_limits<float>::quiet_NaN();
  const double step=1e-3;
  try {
    const double derivative=(evaluate(coordinate+step).z-evaluate(coordinate-step).z)/(2*step);
    const double value=std::abs(derivative)*sigma;
    if(std::isfinite(value) && std::isfinite(static_cast<float>(value)))return static_cast<float>(value);
  } catch(const core::Exception&) {}
  return std::numeric_limits<float>::quiet_NaN();
}
} // namespace mv::reconstruction
