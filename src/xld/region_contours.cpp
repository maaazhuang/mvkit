#include <xld/operators.hpp>
#include <runtime/execution/execution_context.hpp>

#include <algorithm>
#include <cstdint>
#include <memory_resource>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mv::xld {
namespace {
using mv::runtime::ExecutionContext;
struct Cell { std::int64_t r{}, c{}; friend bool operator==(const Cell&,const Cell&)=default; };
struct Vertex { std::int64_t r2{}, c2{}; friend bool operator==(const Vertex&,const Vertex&)=default; };
struct CellHash { std::size_t operator()(const Cell& v) const noexcept { return std::hash<std::int64_t>{}(v.r) ^ (std::hash<std::int64_t>{}(v.c)<<1U); } };
struct VertexHash { std::size_t operator()(const Vertex& v) const noexcept { return std::hash<std::int64_t>{}(v.r2) ^ (std::hash<std::int64_t>{}(v.c2)<<1U); } };
struct Edge { Vertex a,b; bool used=false; };
using PmrEdges = std::pmr::vector<Edge>;
using PmrVertices = std::pmr::vector<Vertex>;
using PmrIndexList = std::pmr::vector<std::size_t>;

PmrEdges boundary_edges(const mv::core::Region& region, std::pmr::memory_resource* memory){
  std::pmr::unordered_set<Cell,CellHash> cells{memory};
  for(const auto& run:region.runs()) for(std::int64_t c=run.col_begin;c<=run.col_end;++c) cells.insert({run.row,c});
  const auto bounds=region.bounding_box();
  const auto width=bounds ? std::uint64_t(bounds->col_max)-std::uint64_t(bounds->col_min)+1 : 0;
  const auto height=bounds ? std::uint64_t(bounds->row_max)-std::uint64_t(bounds->row_min)+1 : 0;
  // 规范 Region 面积等于包围盒面积即为满矩形，保留原单元插入/遍历及边序。
  const bool rectangle=width && height && width<=UINT64_MAX/height && width*height==region.area();
  PmrEdges edges{memory};
  auto has=[&](std::int64_t r,std::int64_t c){return cells.contains({r,c});};
  if(rectangle) {
  for(const auto& p:cells){
    const auto r=p.r,c=p.c;
    if(r==bounds->row_min) edges.push_back({{2*r-1,2*c-1},{2*r-1,2*c+1}});
    if(c==bounds->col_max) edges.push_back({{2*r-1,2*c+1},{2*r+1,2*c+1}});
    if(r==bounds->row_max) edges.push_back({{2*r+1,2*c+1},{2*r+1,2*c-1}});
    if(c==bounds->col_min) edges.push_back({{2*r+1,2*c-1},{2*r-1,2*c-1}});
  }
  } else {
  for(const auto& p:cells){
    const auto r=p.r,c=p.c;
    if(!has(r-1,c)) edges.push_back({{2*r-1,2*c-1},{2*r-1,2*c+1}});
    if(!has(r,c+1)) edges.push_back({{2*r-1,2*c+1},{2*r+1,2*c+1}});
    if(!has(r+1,c)) edges.push_back({{2*r+1,2*c+1},{2*r+1,2*c-1}});
    if(!has(r,c-1)) edges.push_back({{2*r+1,2*c-1},{2*r-1,2*c-1}});
  }
  }
  return edges;
}

double signed_area(std::span<const Vertex> v){
  if(v.size()<3) return 0.0;
  double s=0.0;
  for(std::size_t i=0;i<v.size();++i){const auto&a=v[i];const auto&b=v[(i+1)%v.size()];s+=static_cast<double>(a.c2)*b.r2-static_cast<double>(b.c2)*a.r2;}
  return s*0.125;
}

std::pmr::vector<PmrVertices> trace_loops(PmrEdges& edges, std::pmr::memory_resource* memory){
  std::pmr::unordered_map<Vertex,PmrIndexList,VertexHash> out{memory};
  auto append_adj=[&](const Vertex& key, std::size_t i){
    auto it=out.find(key);
    if(it==out.end()) it=out.emplace(key,PmrIndexList{memory}).first;
    it->second.push_back(i);
  };
  for(std::size_t i=0;i<edges.size();++i) append_adj(edges[i].a,i);
  std::pmr::vector<PmrVertices> loops{memory};
  for(std::size_t seed=0;seed<edges.size();++seed){
    if(edges[seed].used) continue;
    PmrVertices loop{memory};
    const Vertex start=edges[seed].a;
    std::size_t idx=seed;
    for(std::size_t guard=0;guard<=edges.size();++guard){
      auto& e=edges[idx]; if(e.used) break; e.used=true; loop.push_back(e.a);
      const Vertex next=e.b; if(next==start) break;
      auto it=out.find(next); if(it==out.end()) break;
      bool found=false;
      for(auto cand:it->second) if(!edges[cand].used){idx=cand;found=true;break;}
      if(!found) break;
    }
    if(loop.size()>=3) loops.push_back(std::move(loop));
  }
  return loops;
}

mv::core::XldContour loop_to_contour(std::span<const Vertex> loop, RegionContourMode mode, std::pmr::memory_resource* memory){
  std::pmr::vector<mv::core::Point2d> pts{memory}; pts.reserve(loop.size());
  if(mode==RegionContourMode::Center){
    for(std::size_t i=0;i<loop.size();++i){
      const auto&a=loop[i]; const auto&b=loop[(i+1)%loop.size()];
      const double ar=0.5*a.r2, ac=0.5*a.c2, br=0.5*b.r2, bc=0.5*b.c2;
      const double dr=br-ar, dc=bc-ac;
      mv::core::Point2d p{0.5*(ar+br)+0.5*dc,0.5*(ac+bc)-0.5*dr};
      if(pts.empty() || pts.back().row!=p.row || pts.back().col!=p.col) pts.push_back(p);
    }
  }else{
    for(const auto&v:loop) pts.push_back({0.5*v.r2,0.5*v.c2});
  }
  // HALCON 从最上轮廓行的最右点起步；单元边发现序依赖哈希表，须显式规范化。
  if(!pts.empty()) {
    if(pts.size()>1 && pts.front()==pts.back())pts.pop_back();
    const auto first=std::min_element(pts.begin(),pts.end(),[](auto a,auto b){
      return a.row<b.row || (a.row==b.row && a.col>b.col);
    });
    std::rotate(pts.begin(),first,pts.end());
    pts.push_back(pts.front());
  }
  return mv::core::XldContour::from_points(pts,true);
}
}

mv::core::XldContours gen_contour_region_xld(const mv::core::Region &region,
                                             RegionContourMode mode) {
  ExecutionContext context;
  auto *memory = context.scratch_resource();
  const auto bounds = region.bounding_box();
  if (bounds) {
    const auto width = std::int64_t(bounds->col_max) - bounds->col_min + 1;
    const auto height = std::int64_t(bounds->row_max) - bounds->row_min + 1;
    // 满矩形直接生成像素边界，工作量只随周长增长；复用点序及中心转换规则。
    if (std::uint64_t(width) <= UINT64_MAX / std::uint64_t(height) &&
        std::uint64_t(width) * std::uint64_t(height) == region.area()) {
      PmrVertices loop{memory};
      loop.reserve(static_cast<std::size_t>(2 * (width + height)));
      for (auto r = std::int64_t(bounds->row_min); r <= bounds->row_max; ++r)
        loop.push_back({2 * r - 1, 2 * std::int64_t(bounds->col_max) + 1});
      for (auto c = std::int64_t(bounds->col_max); c >= bounds->col_min; --c)
        loop.push_back({2 * std::int64_t(bounds->row_max) + 1, 2 * c + 1});
      for (auto r = std::int64_t(bounds->row_max); r >= bounds->row_min; --r)
        loop.push_back({2 * r + 1, 2 * std::int64_t(bounds->col_min) - 1});
      for (auto c = std::int64_t(bounds->col_min); c <= bounds->col_max; ++c)
        loop.push_back({2 * std::int64_t(bounds->row_min) - 1, 2 * c - 1});
      mv::core::XldContours out;
      out.append(loop_to_contour(loop, mode, memory));
      return out;
    }
  }
  auto edges = boundary_edges(region, memory);
  auto loops = trace_loops(edges, memory);
  mv::core::XldContours out;
  for (const auto &loop : loops) {
    const bool hole = signed_area(loop) < 0.0;
    if (mode != RegionContourMode::BorderHoles && hole)
      continue;
    out.append(loop_to_contour(loop, mode, memory));
  }
  return out;
}

mv::core::XldContours gen_contour_region_xld(const mv::core::Regions& regions,RegionContourMode mode){
  mv::core::XldContours out;
  for(std::size_t region_index=0;region_index<regions.size();++region_index){
    auto x=gen_contour_region_xld(regions[region_index],mode);
    for(std::size_t contour_index=0;contour_index<x.contour_count();++contour_index) {
      out.append(x.contour(contour_index));
    }
  }
  return out;
}
} // namespace mv::xld
