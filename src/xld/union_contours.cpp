#include <xld/operators.hpp>
#include <xld/contour_utils.hpp>
#include <core/exception.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory_resource>
#include <numbers>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mv::xld {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
constexpr auto pi = std::numbers::pi;
[[noreturn]] void fail_check(const char* op, const char* parameter) {
  throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                  parameter, -1, "invalid union parameter or contour data");
}
inline void check(bool ok, const char* op, const char* parameter) {
  if (!ok) fail_check(op,parameter);
}

struct Node {
  XldContour contour;
  double length = 0, row = 0, col = 0, dr = 0, dc = 0;
  std::size_t version = 0;
  bool alive = true, eligible = false, described = false;
};
void prepare(Node& n) {
  const auto p = n.contour.points();
  n.eligible = p.size() > 1 && !n.contour.closed() && p.front() != p.back();
  n.described = false;
}
void describe(Node& n) {
  n.described = true;
  auto p = n.contour.points();
  n.length = length_xld(n.contour);
  n.eligible = p.size() > 1 && !n.contour.closed() && p.front() != p.back() &&
               n.length > 0;
  if (!n.eligible) return;
  // 相对首点计算，保持大平移下精度。
  double r = 0, c = 0;
  for (auto v : p) {
    r += v.row - p[0].row;
    c += v.col - p[0].col;
  }
  r /= p.size();
  c /= p.size();
  n.row = p[0].row + r;
  n.col = p[0].col + c;
  double rr = 0, cc = 0, rc = 0;
  for (auto v : p) {
    const auto y = (v.row - p[0].row) - r, x = (v.col - p[0].col) - c;
    rr += y * y;
    cc += x * x;
    rc += y * x;
  }
  const auto angle = .5 * std::atan2(2 * rc, cc - rr);
  n.dr = std::sin(angle);
  n.dc = std::cos(angle);
  if (rc == 0) {
    n.dr = rr > cc ? 1 : 0;
    n.dc = rr > cc ? 0 : 1;
  }
}
struct Candidate {
  double distance, short_length;
  std::size_t a, b, va, vb;
  int ea, eb;
};
auto key(const Candidate& c) {
  return std::tuple{c.distance, -c.short_length, c.a, c.b, c.ea, c.eb};
}
bool later(const Candidate& a, const Candidate& b) { return key(a) > key(b); }
struct EndpointGrid {
  struct FlatEndpoint { std::int32_t row,col;std::size_t node;int end; };
  std::vector<FlatEndpoint> flat;
  bool use_flat=false;
  explicit EndpointGrid(std::size_t count=129) : use_flat(count<=128) {
    if(use_flat)flat.reserve(2*count);
  }
  static auto cell_key(const FlatEndpoint& e) { return std::pair{e.row,e.col}; }
  double cell = 1;
  std::unordered_map<std::uint64_t, std::vector<std::pair<std::size_t, int>>>
      bins;
  std::unordered_map<std::uint32_t,std::size_t> occupied_rows;
  static std::uint64_t pack(double row, double col, double cell) {
    const auto ir = static_cast<std::int32_t>(std::floor(row / cell));
    const auto ic = static_cast<std::int32_t>(std::floor(col / cell));
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ir)) << 32) |
           static_cast<std::uint32_t>(ic);
  }
  void insert(std::size_t i, int end, Point2d p) {
    if(use_flat) {
      FlatEndpoint e{static_cast<std::int32_t>(std::floor(p.row/cell)),
                     static_cast<std::int32_t>(std::floor(p.col/cell)),i,end};
      const auto pos=std::upper_bound(flat.begin(),flat.end(),cell_key(e),
          [](auto key,const auto& entry){return key<cell_key(entry);});
      flat.insert(pos,e);return;
    }
    const auto key=pack(p.row,p.col,cell);
    auto [bin,created]=bins.try_emplace(key);
    if(created)++occupied_rows[static_cast<std::uint32_t>(key>>32)];
    bin->second.push_back({i,end});
  }
  void erase(std::size_t i, int end, Point2d p) {
    if(use_flat) {
      flat.erase(std::remove_if(flat.begin(),flat.end(),[&](const auto& e){
        return e.node==i && e.end==end;
      }),flat.end());return;
    }
    const auto k = pack(p.row, p.col, cell);
    auto it = bins.find(k);
    if (it == bins.end()) return;
    auto& v = it->second;
    v.erase(std::remove(v.begin(), v.end(), std::pair<std::size_t, int>{i, end}),
            v.end());
    if (v.empty()) {
      bins.erase(it);
      auto row=occupied_rows.find(static_cast<std::uint32_t>(k>>32));
      if(--row->second==0)occupied_rows.erase(row);
    }
  }
  template <class F>
  void query(Point2d p, double radius, F&& fn) const {
    const auto r0 =
        static_cast<std::int32_t>(std::floor((p.row - radius) / cell));
    const auto r1 =
        static_cast<std::int32_t>(std::floor((p.row + radius) / cell));
    const auto c0 =
        static_cast<std::int32_t>(std::floor((p.col - radius) / cell));
    const auto c1 =
        static_cast<std::int32_t>(std::floor((p.col + radius) / cell));
    if(use_flat) {
      auto first=std::lower_bound(flat.begin(),flat.end(),r0,
          [](const auto& e,auto row){return e.row<row;});
      for(;first!=flat.end() && first->row<=r1;++first)
        if(first->col>=c0 && first->col<=c1)fn(first->node);
      return;
    }
    for (auto r = r0; r <= r1; ++r) {
      if(!occupied_rows.contains(static_cast<std::uint32_t>(r)))continue;
      for (auto c = c0; c <= c1; ++c) {
        const auto k =
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(r)) << 32) |
            static_cast<std::uint32_t>(c);
        const auto it = bins.find(k);
        if (it == bins.end()) continue;
        for (auto item : it->second) fn(item.first);
      }
    }
  }
};
XldContour join(const XldContour& a, const XldContour& b, int ea, int eb,
                XldAttributeMode mode) {
  mv::runtime::ScratchArena context;
  auto* memory = context.resource();
  std::pmr::vector<Point2d> points{memory};
  std::pmr::vector<double> response{memory}, edge{memory}, normal{memory};
  const std::array attrs{XldPointAttribute::Response,
                         XldPointAttribute::EdgeDirection,
                         XldPointAttribute::NormalAngle};
  std::array<std::pmr::vector<double>*, 3> output{&response, &edge, &normal};
  std::array<bool, 3> keep{};
  for (std::size_t k = 0; k < 3; ++k)
    keep[k] = mode == XldAttributeMode::Keep &&
              a.has_point_attribute(attrs[k]) &&
              b.has_point_attribute(attrs[k]);
  const auto append = [&](const XldContour& contour, bool reverse) {
    for (std::size_t i = 0; i < contour.size(); ++i) {
      const auto j = reverse ? contour.size() - 1 - i : i;
      points.push_back(contour.points()[j]);
      for (std::size_t k = 0; k < 3; ++k)
        if (keep[k]) {
          auto value = contour.point_attribute(attrs[k])[j];
          if (k == 2 && reverse) {
            value = std::fmod(value, 2 * pi) + pi;
            if (value < 0) value += 2 * pi;
            if (value >= 2 * pi) value -= 2 * pi;
          }
          output[k]->push_back(value);
        }
    }
  };
  // 接缝处重合两样本均保留，其测量属性可能不同。
  append(a, ea == 0);
  append(b, eb == 1);
  const bool closed = points.size() > 1 && points.front() == points.back();
  return XldContour::from_points_and_attributes(
      points, closed, ContourApproximation::Unknown, response, edge, normal);
}
XldContours unite(const XldContours& input, double abs, double rel,
                  double shift, double angle, XldAttributeMode mode,
                  bool collinear) {
  const char* op = collinear ? "union_collinear_contours_xld"
                             : "union_adjacent_contours_xld";
  check(mv::core::is_finite(abs) && abs >= 0, op, "MaxDistAbs");
  check(mv::core::is_finite(rel) && rel >= 0, op, "MaxDistRel");
  check(mode == XldAttributeMode::Keep || mode == XldAttributeMode::Forget, op,
        "Mode");
  if (collinear) {
    check(mv::core::is_finite(shift) && shift >= 0, op, "MaxShift");
    check(mv::core::is_finite(angle) && angle >= 0 && angle <= pi / 2, op,
          "MaxAngle");
  }
  mv::runtime::ScratchArena context;
  std::pmr::vector<Node> nodes(input.contour_count(),
                               context.resource());
  std::size_t work = 0;
  for (std::size_t i = 0; i < input.contour_count(); ++i)
    work += std::min(input.contour(i).size(), SIZE_MAX - work);
  ExecutionContext::for_each_partition(
      nodes.size(), work, 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i) {
          auto& n = nodes[i];
          n.contour = input.contour(i);
          for (auto p : n.contour.points())
            check(std::abs(p.row) <= 0x1p30 && std::abs(p.col) <= 0x1p30,
                  op, "Contours");
          if (mode == XldAttributeMode::Keep)
            for (auto attr :
                 {XldPointAttribute::Response, XldPointAttribute::EdgeDirection,
                  XldPointAttribute::NormalAngle})
              for (auto v : n.contour.point_attribute(attr))
                check(mv::core::is_finite(v), op, "Attributes");
          prepare(n);
        }
      });
  std::pmr::vector<Candidate> heap{context.resource()};
  auto add = [&](std::size_t i, std::size_t j) {
    if (i > j) std::swap(i, j);
    auto& a = nodes[i];
    auto& b = nodes[j];
    if (!a.alive || !b.alive || !a.eligible || !b.eligible) return;
    // 完整几何扫描前由端点查找拒绝远点对，描述缓存保留至合并替换不可变点载荷。
    if (!a.described) describe(a);
    if (!b.described) describe(b);
    if (!a.eligible || !b.eligible) return;
    if (collinear) {
      const auto cross = std::abs(a.dr * b.dc - a.dc * b.dr),
                 dot = std::abs(a.dr * b.dr + a.dc * b.dc);
      if (std::atan2(cross, dot) > angle) return;
      // 对称端点到回归线限制，每次拼接后重算。
      auto offset = [](Point2d p, const Node& n) {
        return std::abs((p.row - n.row) * n.dc - (p.col - n.col) * n.dr);
      };
      for (auto p : {a.contour.points().front(), a.contour.points().back()})
        if (offset(p, b) > shift) return;
      for (auto p : {b.contour.points().front(), b.contour.points().back()})
        if (offset(p, a) > shift) return;
    }
    bool found = false;
    Candidate best{};
    for (int ea = 0; ea < 2; ++ea)
      for (int eb = 0; eb < 2; ++eb) {
        auto p = ea ? a.contour.points().back() : a.contour.points().front();
        auto q = eb ? b.contour.points().back() : b.contour.points().front();
        const auto dr = q.row - p.row, dc = q.col - p.col;
        const auto ga = std::abs(dr * a.dr + dc * a.dc),
                   gb = std::abs(dr * b.dr + dc * b.dc);
        if (!((ga <= abs && ga / a.length <= rel) ||
              (gb <= abs && gb / b.length <= rel)))
          continue;
        Candidate candidate{std::hypot(dr, dc),
                            std::min(a.length, b.length),
                            i,
                            j,
                            a.version,
                            b.version,
                            ea,
                            eb};
        if (!found || key(candidate) < key(best)) {
          best = candidate;
          found = true;
        }
      }
    if (found) {
      heap.push_back(best);
      std::push_heap(heap.begin(), heap.end(), later);
    }
  };
  EndpointGrid grid(nodes.size());
  grid.cell = std::max({abs, collinear ? shift : 0.0, 1.0});
  const double radius = abs + (collinear ? shift : abs);
  auto ends = [](const Node& n) {
    return std::array<Point2d, 2>{n.contour.points().front(),
                                  n.contour.points().back()};
  };
  auto insert_node = [&](std::size_t i) {
    if (!nodes[i].alive || !nodes[i].eligible) return;
    const auto e = ends(nodes[i]);
    grid.insert(i, 0, e[0]);
    grid.insert(i, 1, e[1]);
  };
  auto erase_node = [&](std::size_t i) {
    if (nodes[i].contour.size() < 2) return;
    const auto e = ends(nodes[i]);
    grid.erase(i, 0, e[0]);
    grid.erase(i, 1, e[1]);
  };
  auto consider = [&](std::size_t i, bool later_only) {
    if (!nodes[i].alive || !nodes[i].eligible) return;
    const auto e = ends(nodes[i]);
    auto visit = [&](std::size_t j) {
      if (j == i || (later_only && j < i)) return;
      add(i, j);
    };
    grid.query(e[0], radius, visit);
    grid.query(e[1], radius, visit);
  };
  for (std::size_t i = 0; i < nodes.size(); ++i) insert_node(i);
  for (std::size_t i = 0; i < nodes.size(); ++i) consider(i, true);
  bool joined=false;
  while (!heap.empty()) {
    std::pop_heap(heap.begin(), heap.end(), later);
    const auto c = heap.back();
    heap.pop_back();
    auto& a = nodes[c.a];
    auto& b = nodes[c.b];
    if (!a.alive || !b.alive || a.version != c.va || b.version != c.vb)
      continue;
    erase_node(c.a);
    erase_node(c.b);
    a.contour = join(a.contour, b.contour, c.ea, c.eb, mode);
    joined=true;
    ++a.version;
    b.alive = false;
    prepare(a);
    insert_node(c.a);
    consider(c.a, false);
  }
  if(!joined && mode==XldAttributeMode::Keep)return input;
  std::vector<IconicValue> values;
  for (auto& n : nodes)
    if (n.alive) {
      if (mode == XldAttributeMode::Forget)
        n.contour = n.contour.geometry_view(n.contour.closed(), n.contour.approximation());
      values.push_back(IconicValue::from_xld_contour(std::move(n.contour)));
    }
  return XldContours{IconicTuple{std::move(values)}};
}
}  // namespace
mv::core::XldContours union_adjacent_contours_xld(
    const mv::core::XldContours& c, double a, double r, XldAttributeMode m) {
  return unite(c, a, r, 0, 0, m, false);
}
mv::core::XldContours union_collinear_contours_xld(
    const mv::core::XldContours& c, double a, double r, double s, double angle,
    XldAttributeMode m) {
  return unite(c, a, r, s, angle, m, true);
}
}  // namespace mv::xld

namespace mv::xld {
mv::core::XldContours union_cocircular_contours_xld(const mv::core::XldContours& input,const CocircularOptions& options,XldAttributeMode mode) {
 constexpr auto op="union_cocircular_contours_xld";
 for(auto v:{options.max_arc_angle_diff,options.max_arc_overlap,options.max_tangent_angle,options.max_dist,options.max_radius_diff,options.max_center_dist})detail::require(mv::core::is_finite(v)&&v>=0,op,"Options","finite nonnegative thresholds required");
 detail::require(options.max_arc_angle_diff<=pi&&options.max_arc_overlap<=pi&&options.max_tangent_angle<=pi/2,op,"Angles","gap/overlap <= pi and tangent <= pi/2 required");
 detail::require(mode==XldAttributeMode::Keep||mode==XldAttributeMode::Forget,op,"Mode","invalid attribute mode");
 struct Arc {XldContour c;CircleFitResult fit;double span=0;bool valid=false,alive=true;std::size_t version=0;};
 mv::runtime::ScratchArena ctx;auto* memory=ctx.resource();std::pmr::vector<Arc> arcs(input.contour_count(),memory);
 auto describe_arc=[&](Arc& arc){arc.valid=false;if(arc.c.size()<options.min_num_points||arc.c.closed()||arc.c.points().front()==arc.c.points().back())return;
   ExecutionContext local;std::pmr::vector<Point2d> points{local.scratch_resource()};auto origin=arc.c.points().front();double scale=0;for(auto p:arc.c.points())scale=std::max(scale,detail::point_distance(p,origin));if(scale==0)return;
   for(auto p:arc.c.points())points.push_back({(p.row-origin.row)/scale,(p.col-origin.col)/scale});
   try {arc.fit=fit_circle_contour_xld(XldContour::from_points(points),{CircleFitAlgorithm::Geometric,-1,2,5});}catch(const mv::core::Exception& e){if(e.status()!=mv::core::Status::InvalidArgument)throw;return;}
   arc.fit.row=origin.row+scale*arc.fit.row;arc.fit.column=origin.col+scale*arc.fit.column;arc.fit.radius*=scale;
   if(!mv::core::is_finite(arc.fit.radius)||arc.fit.radius<=0)return;
   double previous=std::atan2(-(arc.c.points()[0].row-arc.fit.row),arc.c.points()[0].col-arc.fit.column),total=0;
   for(std::size_t i=0;i<arc.c.size();++i){auto p=arc.c.points()[i];if(std::abs(std::hypot(p.row-arc.fit.row,p.col-arc.fit.column)-arc.fit.radius)>options.max_radius_diff)return;if(i){double now=std::atan2(-(p.row-arc.fit.row),p.col-arc.fit.column);total+=std::remainder(now-previous,2*pi);previous=now;}}
   arc.span=total;arc.valid=std::abs(total)>1e-6&&std::abs(total)<=2*pi+1e-8;
 };
 ExecutionContext::for_each_partition(arcs.size(),detail::work(input),32768,[&](auto range){for(auto i=range.begin;i<range.end;++i){arcs[i].c=input.contour(i);detail::validate(arcs[i].c,op);if(mode==XldAttributeMode::Keep)for(auto attr:{XldPointAttribute::Response,XldPointAttribute::EdgeDirection,XldPointAttribute::NormalAngle})for(auto v:arcs[i].c.point_attribute(attr))detail::require(mv::core::is_finite(v),op,"Attributes","finite attributes required");describe_arc(arcs[i]);}});
 std::pmr::vector<Candidate> heap{memory};
 auto add=[&](std::size_t i,std::size_t j){if(i>j)std::swap(i,j);const auto& a=arcs[i];const auto& b=arcs[j];if(!a.alive||!b.alive||!a.valid||!b.valid)return;
   if(std::abs(a.fit.radius-b.fit.radius)>options.max_radius_diff||std::hypot(a.fit.row-b.fit.row,a.fit.column-b.fit.column)>options.max_center_dist)return;
   bool found=false;Candidate best{};double row=(a.fit.row+b.fit.row)*.5,col=(a.fit.column+b.fit.column)*.5;
   for(int ea=0;ea<2;++ea)for(int eb=0;eb<2;++eb){double sa=ea==0?-a.span:a.span,sb=eb==1?-b.span:b.span;if(sa*sb<=0)continue;auto p=ea?a.c.points().back():a.c.points().front(),q=eb?b.c.points().back():b.c.points().front();double distance=detail::point_distance(p,q);if(distance>options.max_dist)continue;
     double pa=std::atan2(-(p.row-row),p.col-col),pb=std::atan2(-(q.row-row),q.col-col),gap=std::remainder((sa>0?1:-1)*(pb-pa),2*pi);if(gap>options.max_arc_angle_diff||gap < -options.max_arc_overlap)continue;
     double la=std::abs(sa),lb=std::abs(sb),start=la+gap,overlap=0;for(double shift:{-2*pi,0.,2*pi})overlap+=std::max(0.,std::min(la,start+lb+shift)-std::max(0.,start+shift));if(overlap>options.max_arc_overlap+1e-12||la+gap+lb>2*pi+options.max_arc_overlap)continue;
     if(distance>0){auto tangent=[&](Point2d endpoint,const CircleFitResult& fit){double nr=endpoint.row-fit.row,nc=endpoint.col-fit.column;double dot=std::abs((q.row-p.row)*nc-(q.col-p.col)*nr)/(distance*std::hypot(nr,nc));return std::acos(std::clamp(dot,0.,1.));};if(tangent(p,a.fit)>options.max_tangent_angle||tangent(q,b.fit)>options.max_tangent_angle)continue;}
     Candidate candidate{distance,std::min(length_xld(a.c),length_xld(b.c)),i,j,a.version,b.version,ea,eb};if(!found||key(candidate)<key(best)){best=candidate;found=true;}
   }if(found){heap.push_back(best);std::push_heap(heap.begin(),heap.end(),later);}
 };
 EndpointGrid grid;
 grid.cell=std::max(options.max_dist,1.0);
 const double radius=options.max_dist;
 auto ends=[&](const Arc& a){return std::array<Point2d,2>{a.c.points().front(),a.c.points().back()};};
 auto insert_arc=[&](std::size_t i){if(!arcs[i].alive||!arcs[i].valid)return;auto e=ends(arcs[i]);grid.insert(i,0,e[0]);grid.insert(i,1,e[1]);};
 auto erase_arc=[&](std::size_t i){if(arcs[i].c.size()<2)return;auto e=ends(arcs[i]);grid.erase(i,0,e[0]);grid.erase(i,1,e[1]);};
 auto consider=[&](std::size_t i,bool later_only){if(!arcs[i].alive||!arcs[i].valid)return;auto e=ends(arcs[i]);auto visit=[&](std::size_t j){if(j==i||(later_only&&j<i))return;add(i,j);};grid.query(e[0],radius,visit);grid.query(e[1],radius,visit);};
 for(int iteration=0;options.iterations<0 || iteration<options.iterations;++iteration) {
 grid=EndpointGrid{};grid.cell=std::max(options.max_dist,1.0);heap.clear();bool changed=false;
 for(std::size_t i=0;i<arcs.size();++i)insert_arc(i);
 for(std::size_t i=0;i<arcs.size();++i)consider(i,true);
 while(!heap.empty()){std::pop_heap(heap.begin(),heap.end(),later);auto c=heap.back();heap.pop_back();auto& a=arcs[c.a];auto& b=arcs[c.b];if(!a.alive||!b.alive||a.version!=c.va||b.version!=c.vb)continue;erase_arc(c.a);erase_arc(c.b);a.c=join(a.c,b.c,c.ea,c.eb,mode);++a.version;b.alive=false;changed=true;describe_arc(a);insert_arc(c.a);if(options.iterations<0)consider(c.a,false);}

 // 第二遍按拟合圆检查小轮廓：全部顶点满足径向残差，全部非零段符合局部切线；端点距离决定拼接。
 if(options.merge_small_contours) {
  for(std::size_t i=0;i<arcs.size();++i)if(arcs[i].alive&&arcs[i].valid) {
   for(std::size_t j=0;j<arcs.size();++j) {
    auto& large=arcs[i];auto& small=arcs[j];
    if(i==j||!small.alive||small.valid||small.c.empty()||small.c.closed())continue;
    bool fits=true;const auto ps=small.c.points();
    for(std::size_t k=0;k<ps.size();++k) {
     const auto p=ps[k];
     if(std::abs(std::hypot(p.row-large.fit.row,p.col-large.fit.column)-large.fit.radius)>options.max_radius_diff){fits=false;break;}
     if(k) {
      const double dr=p.row-ps[k-1].row,dc=p.col-ps[k-1].col,length=std::hypot(dr,dc);
      const double nr=(p.row+ps[k-1].row)*.5-large.fit.row,nc=(p.col+ps[k-1].col)*.5-large.fit.column;
      const double radius=std::hypot(nr,nc);
      if(length>0 && radius>0 && std::acos(std::clamp(std::abs(dr*nc-dc*nr)/(length*radius),0.,1.))>options.max_tangent_angle){fits=false;break;}
     }
    }
    if(!fits)continue;
    double best=INFINITY;int ea=0,eb=0;
    for(int x=0;x<2;++x)for(int y=0;y<2;++y) {
     const auto p=x?large.c.points().back():large.c.points().front(),q=y?ps.back():ps.front();
     const double d=detail::point_distance(p,q);if(d<best){best=d;ea=x;eb=y;}
    }
    if(best>options.max_dist)continue;
    large.c=join(large.c,small.c,ea,eb,mode);++large.version;small.alive=false;changed=true;describe_arc(large);
   }
  }
 }
 if(!changed)break;
 }
 std::pmr::vector<XldContour> output{memory};for(auto& a:arcs)if(a.alive){if(mode==XldAttributeMode::Forget)a.c=XldContour::from_points_and_attributes(a.c.points(),a.c.closed(),a.c.approximation());output.push_back(a.c);}return detail::objects(output);
}
}
