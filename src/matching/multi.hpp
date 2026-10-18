#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <memory_resource>
#include <limits>

namespace mv::matching {
struct Selection { std::int64_t num_matches{-1}; double max_overlap{-1}; };
struct Bounds { double top,bottom,left,right; };
struct Point { double x,y; };
using Box=std::array<Point,4>;
inline Bounds enclosure(const Box& b) {
  Bounds out{b[0].y,b[0].y,b[0].x,b[0].x};
  for(const auto& p:b){out.top=std::min(out.top,p.y);out.bottom=std::max(out.bottom,p.y);out.left=std::min(out.left,p.x);out.right=std::max(out.right,p.x);}
  return out;
}
inline bool disjoint(const Bounds& a,const Bounds& b) {
  return a.right<b.left || b.right<a.left || a.bottom<b.top || b.bottom<a.top;
}
// 凸包后按边方向精确扫描最小面积；模型点不可变，调用方缓存 O(N log N+H^2) 准备。
inline Box minimum_rectangle(std::span<const Point> input,std::pmr::memory_resource* memory) {
  if(input.empty())return {};
  const auto origin=input.front();std::pmr::vector<Point> points{memory};points.reserve(input.size());
  for(const auto p:input)points.push_back({p.x-origin.x,p.y-origin.y});
  std::sort(points.begin(),points.end(),[](const auto& a,const auto& b){return a.x<b.x || (a.x==b.x && a.y<b.y);});
  points.erase(std::unique(points.begin(),points.end(),[](const auto& a,const auto& b){return a.x==b.x && a.y==b.y;}),points.end());
  if(points.size()<3){auto a=points.front(),b=points.back();a.x+=origin.x;a.y+=origin.y;b.x+=origin.x;b.y+=origin.y;return {{a,b,b,a}};}
  const auto cross=[](Point a,Point b,Point c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);};
  std::pmr::vector<Point> hull(points.size()*2,Point{},memory);std::size_t n=0;
  for(const auto p:points){while(n>=2 && cross(hull[n-2],hull[n-1],p)<=0)--n;hull[n++]=p;}
  const auto upper=n+1;
  for(std::size_t i=points.size()-1;i>0;--i){const auto p=points[i-1];while(n>=upper && cross(hull[n-2],hull[n-1],p)<=0)--n;hull[n++]=p;}
  hull.resize(n-1);Box best{};double best_area=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<hull.size();++i) {
    const auto a=hull[i],b=hull[(i+1)%hull.size()];const auto norm=std::hypot(b.x-a.x,b.y-a.y);if(norm==0)continue;
    const auto x=(b.x-a.x)/norm,y=(b.y-a.y)/norm;
    double u0=std::numeric_limits<double>::infinity(),u1=-u0,v0=u0,v1=-u0;
    for(const auto p:hull){const auto u=x*p.x+y*p.y,v=-y*p.x+x*p.y;u0=std::min(u0,u);u1=std::max(u1,u);v0=std::min(v0,v);v1=std::max(v1,v);}
    const auto candidate_area=(u1-u0)*(v1-v0);if(candidate_area>=best_area)continue;best_area=candidate_area;
    best={{{u0,v0},{u1,v0},{u1,v1},{u0,v1}}};for(auto& p:best){const auto u=p.x,v=p.y;p={origin.x+x*u-y*v,origin.y+y*u+x*v};}
  }
  return best;
}
template<class Pose> Box box(const Bounds& b,const Pose& p,double scale=1) {
  const double ca=std::cos(p.angle)*scale,sa=std::sin(p.angle)*scale;
  Box q{{{b.left,b.top},{b.right,b.top},{b.right,b.bottom},{b.left,b.bottom}}};
  for(auto& v:q){const auto x=v.x,y=v.y;v={p.column+ca*x+sa*y,p.row-sa*x+ca*y};}
  return q;
}
inline double area(std::span<const Point> p) {
  double a=0;for(std::size_t i=0;i<p.size();++i){const auto& u=p[i];const auto& v=p[(i+1)%p.size()];a+=u.x*v.y-v.x*u.y;}return std::abs(a)*.5;
}
inline double overlap(Box a,Box b) {
  // 裁剪前平移，避免大原点造成精度损失。
  const auto origin=a[0];for(auto& p:a){p.x-=origin.x;p.y-=origin.y;}for(auto& p:b){p.x-=origin.x;p.y-=origin.y;}
  const auto denominator=std::min(area(a),area(b));if(denominator<=1e-12)return 0;
  std::array<Point,16> polygon{},next{};std::copy(a.begin(),a.end(),polygon.begin());std::size_t count=4;
  const auto cross=[](Point u,Point v,Point p){return (v.x-u.x)*(p.y-u.y)-(v.y-u.y)*(p.x-u.x);};
  for(int edge=0;edge<4 && count;++edge) {
    const auto u=b[edge],v=b[(edge+1)%4];std::size_t n=0;
    for(std::size_t i=0;i<count;++i) {
      const auto p=polygon[i],q=polygon[(i+1)%count];const auto dp=cross(u,v,p),dq=cross(u,v,q);
      if(dp>=0)next[n++]=p;
      if((dp>=0)!=(dq>=0)){const auto t=dp/(dp-dq);next[n++]={p.x+t*(q.x-p.x),p.y+t*(q.y-p.y)};}
    }
    polygon=next;count=n;
  }
  return std::clamp(area({polygon.data(),count})/denominator,0.,1.);
}
template<class Pose> struct Match { Pose pose; std::size_t model_index; };

// 抑制后应用配额，被抑制候选不消耗配额。
template<class Pose,class Options,class MakeBox,class Refine>
std::vector<Match<Pose>> select_refined(std::vector<Match<Pose>> candidates,
    std::span<const Options> options,Selection selection,MakeBox make_box,Refine refine) {
  std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){
    if(a.pose.score!=b.pose.score)return a.pose.score>b.pose.score;
    if(a.model_index!=b.model_index)return a.model_index<b.model_index;
    if(a.pose.row!=b.pose.row)return a.pose.row<b.pose.row;
    if(a.pose.column!=b.pose.column)return a.pose.column<b.pose.column;
    return a.pose.angle<b.pose.angle;
  });
  std::vector<Match<Pose>> result;std::vector<Box> boxes;std::vector<Bounds> enclosures;std::vector<std::size_t> counts(options.size());
  for(auto candidate:candidates) {
    const auto index=candidate.model_index;
    const auto limit=selection.num_matches>=0?selection.num_matches:std::int64_t(options[index].num_matches);
    if(selection.num_matches<0 && limit && counts[index]>=std::uint64_t(limit))continue;
    const auto threshold=selection.max_overlap>=0?selection.max_overlap:options[index].max_overlap;
    auto rectangle=make_box(candidate);auto aabb=enclosure(rectangle);bool suppressed=false;
    for(std::size_t i=0;i<result.size();++i) {
      if(selection.max_overlap<0 && result[i].model_index!=index)continue;
      if(disjoint(aabb,enclosures[i]))continue;
      if(overlap(rectangle,boxes[i])>threshold+1e-12){suppressed=true;break;}
    }
    if(suppressed)continue;
    if(!refine(candidate))continue;
    rectangle=make_box(candidate);aabb=enclosure(rectangle);suppressed=false;
    for(std::size_t i=0;i<result.size();++i) {
      if(selection.max_overlap<0 && result[i].model_index!=index)continue;
      if(disjoint(aabb,enclosures[i]))continue;
      if(overlap(rectangle,boxes[i])>threshold+1e-12){suppressed=true;break;}
    }
    if(suppressed)continue;
    result.push_back(candidate);boxes.push_back(rectangle);enclosures.push_back(aabb);++counts[index];
    if(selection.num_matches>0 && result.size()>=std::uint64_t(selection.num_matches))break;
  }
  std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){
    if(a.pose.score!=b.pose.score)return a.pose.score>b.pose.score;
    if(a.model_index!=b.model_index)return a.model_index<b.model_index;
    if(a.pose.row!=b.pose.row)return a.pose.row<b.pose.row;
    if(a.pose.column!=b.pose.column)return a.pose.column<b.pose.column;return a.pose.angle<b.pose.angle;
  });return result;
}
template<class Pose,class Options,class MakeBox>
std::vector<Match<Pose>> select(std::vector<Match<Pose>> candidates,
    std::span<const Options> options,Selection selection,MakeBox make_box) {
  return select_refined(std::move(candidates),options,selection,make_box,[](auto&){return true;});
}
} // namespace mv::matching
