#pragma once
#include <xld/spatial_index.hpp>

namespace mv::xld::detail {
// 每调用一个精确线段树供全部查询点共享；远/近遍历分别尽早建立界，
// 平方界减少超越运算，不改变端点投影算术。
class ContourDistanceIndex {
 public:
  ContourDistanceIndex(const XldContour& contour,std::pmr::memory_resource* memory)
      :segments_(contour_segments(contour,memory)),nodes_(memory){
    nodes_.reserve(segments_.size()/4+1);build(0,segments_.size());
  }
  std::array<double,2> query(Point2d point)const{
    Best near{INFINITY,{}},far{-1,{}};nearest(0,point,near);farthest(0,point,far);
    return {std::hypot(near.delta.row,near.delta.col),std::hypot(far.delta.row,far.delta.col)};
  }
  bool localized()const{
    const auto& root=nodes_.front();const auto dr=root.r1-root.r0,dc=root.c1-root.c0;
    const auto full=dr*dr+dc*dc;if(full==0)return false;
    double sum=0;std::size_t count=0;
    for(const auto& n:nodes_)if(n.end-n.begin<=8){const auto r=n.r1-n.r0,c=n.c1-n.c0;sum+=r*r+c*c;++count;}
    return sum<=.125*full*count;
  }
 private:
  struct Node {double r0,r1,c0,c1;std::size_t begin,end,left{},right{};};
  struct Best {double squared;Point2d delta;};
  std::size_t build(std::size_t begin,std::size_t end){
    const auto id=nodes_.size();const auto& s=segments_[begin];
    Node n{s.r0,s.r1,s.c0,s.c1,begin,end};nodes_.push_back(n);
    if(end-begin>8){const auto middle=begin+(end-begin)/2;
      const auto left=build(begin,middle),right=build(middle,end);
      const auto a=nodes_[left],b=nodes_[right];n.left=left;n.right=right;
      n.r0=std::min(a.r0,b.r0);n.r1=std::max(a.r1,b.r1);n.c0=std::min(a.c0,b.c0);n.c1=std::max(a.c1,b.c1);
    }else for(auto i=begin+1;i<end;++i){const auto& q=segments_[i];n.r0=std::min(n.r0,q.r0);n.r1=std::max(n.r1,q.r1);n.c0=std::min(n.c0,q.c0);n.c1=std::max(n.c1,q.c1);}
    nodes_[id]=n;return id;
  }
  static double lower(const Node& n,Point2d p){const double r=std::max({n.r0-p.row,p.row-n.r1,0.}),c=std::max({n.c0-p.col,p.col-n.c1,0.});return r*r+c*c;}
  static double upper(const Node& n,Point2d p){const double r=std::max(std::abs(n.r0-p.row),std::abs(n.r1-p.row)),c=std::max(std::abs(n.c0-p.col),std::abs(n.c1-p.col));return r*r+c*c;}
  static double margin(double bound){return 64*std::numeric_limits<double>::epsilon()*(1+bound);}
  static void update(Point2d delta,Best& best,bool minimum){const auto d=delta.row*delta.row+delta.col*delta.col;
    if((minimum?d<best.squared:d>best.squared) || (d==best.squared &&
        (minimum?std::hypot(delta.row,delta.col)<std::hypot(best.delta.row,best.delta.col):std::hypot(delta.row,delta.col)>std::hypot(best.delta.row,best.delta.col))))best={d,delta};}
  void nearest(std::size_t id,Point2d p,Best& best)const{
    const auto& n=nodes_[id];const auto bound=lower(n,p);if(bound-margin(bound)>best.squared)return;
    if(n.end-n.begin<=8){for(auto i=n.begin;i<n.end;++i){const auto& s=segments_[i];
      const auto dr=s.b.row-s.a.row,dc=s.b.col-s.a.col,den=dr*dr+dc*dc;
      const auto t=den==0?0:std::clamp(((p.row-s.a.row)*dr+(p.col-s.a.col)*dc)/den,0.,1.);
      const auto q=t==0?s.a:t==1?s.b:Point2d{std::lerp(s.a.row,s.b.row,t),std::lerp(s.a.col,s.b.col,t)};
      update({p.row-q.row,p.col-q.col},best,true);
    }}else{const bool left=lower(nodes_[n.left],p)<=lower(nodes_[n.right],p);
      nearest(left?n.left:n.right,p,best);nearest(left?n.right:n.left,p,best);}
  }
  void farthest(std::size_t id,Point2d p,Best& best)const{
    const auto& n=nodes_[id];const auto bound=upper(n,p);if(bound+margin(bound)<best.squared)return;
    if(n.end-n.begin<=8){for(auto i=n.begin;i<n.end;++i){const auto& s=segments_[i];
      // 相邻线段共享相同端点位，每叶只访问一次。
      if(i==n.begin)update({p.row-s.a.row,p.col-s.a.col},best,false);
      update({p.row-s.b.row,p.col-s.b.col},best,false);
    }}else{const bool left=upper(nodes_[n.left],p)>=upper(nodes_[n.right],p);
      farthest(left?n.left:n.right,p,best);farthest(left?n.right:n.left,p,best);}
  }
  std::pmr::vector<SegmentBox> segments_;std::pmr::vector<Node> nodes_;
};
}
