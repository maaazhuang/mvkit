#pragma once
#include <core/region.hpp>
#include <algorithm>
#include <cstdint>

namespace mv::regionproc {
// 输入端点已验证在 int32 内。由原 Bresenham 递推独立推导逐行闭区间，
// 保持从上到下及中点取整规则，不逐像素构造后再合并边界。
template<class Emit>
void raster_edge_runs(std::int64_t y0,std::int64_t x0,std::int64_t y1,std::int64_t x1,Emit&& emit) {
  if(y0==y1){emit(core::RegionRun{y0,std::min(x0,x1),std::max(x0,x1)});return;}
  if(y0>y1){std::swap(y0,y1);std::swap(x0,x1);}
  const auto dx=static_cast<std::uint64_t>(x1>=x0?x1-x0:x0-x1);
  const auto dy=static_cast<std::uint64_t>(y1-y0);
  const std::int64_t direction=x0<x1?1:-1;
  for(std::uint64_t row=0;row<=dy;++row) {
    if(dy>=dx) {
      const auto steps=(row*dx+dy/2)/dy;
      const auto x=x0+direction*static_cast<std::int64_t>(steps);
      emit(core::RegionRun{y0+static_cast<std::int64_t>(row),x,x});
    } else {
      // int32 差值最多 2^32-1，乘积及舍入偏置均可由 uint64 精确容纳。
      const auto threshold=row*dx;
      const auto first=threshold<=dx/2?0:(threshold-dx/2+dy-1)/dy;
      const auto last=std::min(dx,((row+1)*dx-dx/2-1)/dy);
      const auto a=x0+direction*static_cast<std::int64_t>(first);
      const auto b=x0+direction*static_cast<std::int64_t>(last);
      emit(core::RegionRun{y0+static_cast<std::int64_t>(row),std::min(a,b),std::max(a,b)});
    }
  }
}
}
