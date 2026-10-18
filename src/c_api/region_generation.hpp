#pragma once
#include <region/operators.hpp>
#include <cmath>
#include <vector>
namespace mv::c_api {
using mv::core::Region;
inline Region generated_circle(double row,double column,double radius) {
  radius=std::floor(radius*2)*.5;
  const bool even=std::floor(radius)==radius;
  row=even?std::ceil(row)-.5:std::floor(row+.5);
  column=even?std::ceil(column)-.5:std::floor(column+.5);
  // 直径三像素的圆使用 HALCON 十字小圆掩码。
  if(even||radius>=2.5)return mv::regionproc::gen_circle(row,column,radius);
  const auto r=static_cast<std::int64_t>(radius-.5),cr=static_cast<std::int64_t>(row),cc=static_cast<std::int64_t>(column);
  std::vector<mv::core::RegionRun> runs;auto x=r,y=std::int64_t(0),error=1-r;
  while(x>=y) {
    for(auto sign:{-1,1}){runs.push_back({cr+sign*y,cc-x,cc+x});runs.push_back({cr+sign*x,cc-y,cc+y});}
    ++y;if(error<0)error+=2*y+1;else{--x;error+=2*(y-x)+1;}
  }
  return Region::from_runs(runs);
}
}
