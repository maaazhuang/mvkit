#include <region/rectangle_raster.hpp>
#include <region/raster_edge.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <algorithm>
#include <cmath>
#include <memory_resource>
#include <numeric>
#include <vector>

namespace mv::regionproc {
namespace {
// 与原逐像素边界相同的递推和并列决定；单调行内立即合并，避免对所有边界像素排序。
template<class Emit>
void polygon_edge(std::int64_t y,std::int64_t x,std::int64_t yy,std::int64_t xx,
                   bool variant,Emit emit) {
  if(y>yy){std::swap(y,yy);std::swap(x,xx);}
  if(y==yy){emit(core::RegionRun{y,std::min(x,xx),std::max(x,xx)});return;}
  const auto dx=std::abs(xx-x),dy=-std::abs(yy-y),sx=x<xx?std::int64_t{1}:std::int64_t{-1};
  auto error=dx+dy;
  core::RegionRun run{y,x,x};
  for(;;) {
    if(run.row!=y){emit(run);run={y,x,x};}
    else {run.col_begin=std::min<std::int64_t>(run.col_begin,x);run.col_end=std::max<std::int64_t>(run.col_end,x);}
    if(x==xx && y==yy)break;
    const auto twice=2*error;
    const bool xtie=2*x+sx>0?sx>0:sx<0,ytie=2*y+1>0;
    if(twice>dy || (twice==dy && (variant?!xtie:xtie))){error+=dy;x+=sx;}
    if(twice<dx || (twice==dx && (variant?!ytie:ytie))){error+=dx;++y;}
  }
  emit(run);
}

core::Region convex_boundary_runs(const std::array<std::int64_t,4>& rows,
                                  const std::array<std::int64_t,4>& columns) {
  // q*dy+r=n，且 0<=r<dy；每行只加 dx，用进位保持原整数商和余数。
  struct Quotient {
    std::int64_t q{},r{},step_q{},step_r{};
    static Quotient prepare(std::int64_t n,std::int64_t dx,std::int64_t dy) {
      Quotient result{n/dy,n%dy,dx/dy,dx%dy};
      if(result.r<0){--result.q;result.r+=dy;}
      return result;
    }
    void advance(std::int64_t dy) {
      q+=step_q;r+=step_r;
      if(r>=dy){r-=dy;++q;}
    }
  };
  struct Edge {
    std::int64_t y,x,last,dx,dy,direction,end_x;
    Quotient lower,upper;
  };
  std::array<Edge,4> edges;
  for(std::size_t i=0;i<4;++i) {
    const auto j=(i+1)%4;auto y=rows[i],x=columns[i],yy=rows[j],xx=columns[j];
    if(y>yy){std::swap(y,yy);std::swap(x,xx);}
    edges[i]={y,x,yy,std::abs(xx-x),yy-y,x<xx?1:-1,xx};
    auto& edge=edges[i];
    if(edge.dy) {
      edge.lower=Quotient::prepare(edge.dy>=edge.dx?0:-edge.dx/2+edge.dy-1,edge.dx,edge.dy);
      if(edge.dx>edge.dy)
        edge.upper=Quotient::prepare(edge.dx-edge.dx/2-1,edge.dx,edge.dy);
    }
  }
  const auto first=*std::min_element(rows.begin(),rows.end()),last=*std::max_element(rows.begin(),rows.end());
  std::pmr::vector<core::RegionRun> output{runtime::persist_resource()};
  output.resize(static_cast<std::size_t>(last-first+1));
  for(auto row=first;row<=last;++row) {
    auto& run=output[static_cast<std::size_t>(row-first)];
    run.row=static_cast<std::int32_t>(row);run.col_begin=INT32_MAX;run.col_end=INT32_MIN;
  }
  // 每条边顺序更新它覆盖的行，商余数留在局部寄存器中，不逐行扫描四条边。
  for(auto edge:edges) {
    for(auto row=edge.y;row<=edge.last;++row) {
      std::int64_t a,b;
      if(!edge.dy){a=std::min(edge.x,edge.end_x);b=std::max(edge.x,edge.end_x);}
      else if(edge.dy>=edge.dx) {
        const auto quotient=edge.lower.q,remainder=edge.lower.r;
        const auto standard=quotient+(2*remainder>=edge.dy);
        const bool tie=2*remainder==edge.dy;
        // 多边形路径在绝对坐标半值处远离零取整；标准路径在步数半值处向上取整。
        const bool away=(2*edge.x+edge.direction*(2*quotient+1)>0)==(edge.direction>0);
        const auto polygon=quotient+(2*remainder>edge.dy || (tie && away));
        const auto x0=edge.x+edge.direction*standard,x1=edge.x+edge.direction*polygon;
        a=std::min(x0,x1);b=std::max(x0,x1);
      } else {
        const auto start=std::max(std::int64_t{0},edge.lower.q);
        auto finish=std::min(edge.dx,edge.upper.q);
        // 负坐标半值的额外端点等价于偶数 dx 且上界余数为 dy-1。
        if(row<0 && edge.dx%2==0 && edge.upper.r==edge.dy-1)
          finish=std::min(edge.dx,edge.upper.q+1);
        const auto x0=edge.x+edge.direction*start,x1=edge.x+edge.direction*finish;
        a=std::min(x0,x1);b=std::max(x0,x1);
      }
      auto& run=output[static_cast<std::size_t>(row-first)];
      run.col_begin=static_cast<std::int32_t>(std::min<std::int64_t>(run.col_begin,a));
      run.col_end=static_cast<std::int32_t>(std::max<std::int64_t>(run.col_end,b));
      if(edge.dy) {
        edge.lower.advance(edge.dy);
        if(edge.dx>edge.dy)edge.upper.advance(edge.dy);
      }
    }
  }
  return core::Region::from_canonical_runs(std::move(output));
}
}

core::Region rectangle_corner_runs(const std::array<std::int64_t,4>& rows,
                                    const std::array<std::int64_t,4>& columns) {
  bool collinear=true;std::int64_t direction_row=0,direction_column=0;
  bool strict_convex=true;int winding=0;
  for(std::size_t i=0;i<4;++i) {
    if(std::abs(rows[i])>1048576 || std::abs(columns[i])>1048576){strict_convex=false;break;}
  }
  if(strict_convex)for(std::size_t i=0;i<4;++i) {
    const auto j=(i+1)%4,k=(i+2)%4;
    const auto cross=(rows[j]-rows[i])*(columns[k]-columns[j])-(columns[j]-columns[i])*(rows[k]-rows[j]);
    const int sign=(cross>0)-(cross<0);
    if(!sign || (winding && winding!=sign)){strict_convex=false;break;}
    winding=sign;
  }
  if(strict_convex)return convex_boundary_runs(rows,columns);
  runtime::ScratchArena scratch;
  // 通用回退：每条边最多两种多边形并列路径及一条标准 Bresenham 路径。
  std::pmr::vector<std::pmr::vector<core::RegionRun>> edges{scratch.resource()};
  edges.resize(12);
  for(std::size_t i=1;i<4;++i) {
    auto dr=rows[i]-rows[0],dc=columns[i]-columns[0];
    const auto divisor=std::gcd(std::abs(dr),std::abs(dc));
    if(!divisor)continue;
    dr/=divisor;dc/=divisor;
    if(dr<0 || (dr==0 && dc<0)){dr=-dr;dc=-dc;}
    if(direction_row==0 && direction_column==0){direction_row=dr;direction_column=dc;}
    else if(dr!=direction_row || dc!=direction_column){collinear=false;break;}
  }
  for(std::size_t i=0;i<4;++i) {
    const auto next=(i+1)%4;
    polygon_edge(rows[i],columns[i],rows[next],columns[next],false,
                 [&](auto run){edges[i*3].push_back(run);});
    if(collinear && rows[i]!=rows[next])
      polygon_edge(rows[i],columns[i],rows[next],columns[next],true,
                   [&](auto run){edges[i*3+1].push_back(run);});
    raster_edge_runs(rows[i],columns[i],rows[next],columns[next],
                      [&](auto run){edges[i*3+2].push_back(run);});
  }
  const auto first=*std::min_element(rows.begin(),rows.end()),last=*std::max_element(rows.begin(),rows.end());
  std::size_t positions[12]{};
  std::pmr::vector<core::RegionRun> output{runtime::persist_resource()};
  output.reserve(static_cast<std::size_t>(last-first+1));
  for(auto row=first;row<=last;++row) {
    struct Interval {std::int64_t first,last;};
    // 12 条边流、每边至多三个解析边界区间和两段填充，总数最多 26。
    Interval intervals[32];std::size_t count=0;
    const auto append=[&](std::int64_t begin,std::int64_t end) {
      if(begin<=end)intervals[count++]={begin,end};
    };
    for(std::size_t i=0;i<12;++i) {
      const auto& edge=edges[i];auto& position=positions[i];
      if(position<edge.size() && edge[position].row==row) {
        const auto& run=edge[position++];append(run.col_begin,run.col_end);
      }
    }
    double intersections[4];std::size_t crossings=0;
    const auto interval=[&](double left,double right) {
      // 坐标已限于 ±2^30，截断后按大小关系校正，精确实现 ceil/floor，省去逐交点库调用。
      auto begin=static_cast<std::int64_t>(left),end=static_cast<std::int64_t>(right);
      begin+=double(begin)<left;end-=double(end)>right;
      append(begin,end);
    };
    for(std::size_t i=0;i<4;++i) {
      const auto next=(i+1)%4;
      double ar=double(rows[i]),ac=double(columns[i]),br=double(rows[next]),bc=double(columns[next]);
      if(ar>br){std::swap(ar,br);std::swap(ac,bc);}
      if(row==ar)interval(ac,ac);
      if(row==br)interval(bc,bc);
      if(ar==br){if(row==ar)interval(std::min(ac,bc),std::max(ac,bc));continue;}
      if(row<ar || row>br)continue;
      // 保留原扫描线表达式及半开端点，不用递增坐标近似。
      const auto x=ac+(double(row)-ar)/(br-ar)*(bc-ac);
      interval(x,x);
      if(row<br)intersections[crossings++]=x;
    }
    std::sort(intersections,intersections+crossings);
    for(std::size_t i=1;i<crossings;i+=2)interval(intersections[i-1],intersections[i]);
    std::sort(intervals,intervals+count,[](auto a,auto b){return a.first<b.first;});
    for(std::size_t i=0;i<count;) {
      const auto begin=intervals[i].first;auto end=intervals[i++].last;
      while(i<count && intervals[i].first<=end+1)end=std::max(end,intervals[i++].last);
      output.push_back({row,begin,end});
    }
  }
  return core::Region::from_canonical_runs(std::move(output));
}
}
