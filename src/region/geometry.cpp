#include <region/operators.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace mv::regionproc {
namespace {
struct Point { double x; double y; };

double cross(const Point& o, const Point& a, const Point& b) {
  return (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x);
}

std::vector<Point> hull_points(const mv::core::Region& region) {
  std::vector<Point> pts;
  pts.reserve(region.runs().size()*2);
  for (const auto& run : region.runs()) {
    pts.push_back({static_cast<double>(run.col_begin), static_cast<double>(run.row)});
    if (run.col_end != run.col_begin) pts.push_back({static_cast<double>(run.col_end), static_cast<double>(run.row)});
  }
  std::sort(pts.begin(), pts.end(), [](const Point& a,const Point& b){ return a.x<b.x || (a.x==b.x && a.y<b.y); });
  pts.erase(std::unique(pts.begin(), pts.end(), [](const Point&a,const Point&b){return a.x==b.x&&a.y==b.y;}),pts.end());
  if (pts.size() <= 2) return pts;
  std::vector<Point> h(pts.size()*2);
  std::size_t k=0;
  for (const auto& p:pts) { while(k>=2 && cross(h[k-2],h[k-1],p)<=0) --k; h[k++]=p; }
  for (std::size_t i=pts.size()-1,t=k+1;i>0;--i) { const auto&p=pts[i-1]; while(k>=t && cross(h[k-2],h[k-1],p)<=0) --k; h[k++]=p; }
  h.resize(k-1);
  return h;
}

double normalize_phi(double phi) {
  constexpr double pi = 3.1415926535897932384626433832795;
  while (phi >= pi/2) phi -= pi;
  while (phi < -pi/2) phi += pi;
  return phi;
}

struct RawCircle { Point c{}; double r{-1.0}; };
bool inside(const RawCircle& c,const Point&p) { if(c.r<0) return false; const double dx=p.x-c.c.x,dy=p.y-c.c.y; return dx*dx+dy*dy <= (c.r+1e-10)*(c.r+1e-10); }
RawCircle diameter(const Point&a,const Point&b){ Point c{(a.x+b.x)/2,(a.y+b.y)/2}; return {c,std::hypot(a.x-b.x,a.y-b.y)/2}; }
RawCircle circum(const Point&a,const Point&b,const Point&c){
  const double d=2*(a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y));
  if(std::abs(d)<1e-15) return {};
  const double aa=a.x*a.x+a.y*a.y,bb=b.x*b.x+b.y*b.y,cc=c.x*c.x+c.y*c.y;
  Point o{(aa*(b.y-c.y)+bb*(c.y-a.y)+cc*(a.y-b.y))/d,
          (aa*(c.x-b.x)+bb*(a.x-c.x)+cc*(b.x-a.x))/d};
  return {o,std::hypot(o.x-a.x,o.y-a.y)};
}
}

Rectangle2 smallest_rectangle2(const mv::core::Region& region) {
  if (const auto box = region.bounding_box();
      box && box->row_min >= INT32_MIN && box->row_max <= INT32_MAX && box->col_min >= INT32_MIN &&
      box->col_max <= INT32_MAX) {
    const auto width = std::uint64_t(box->col_max - box->col_min) + 1;
    const auto height = std::uint64_t(box->row_max - box->row_min) + 1;
    if (height <= std::numeric_limits<std::uint64_t>::max() / width &&
        region.area() == width * height) {
      const auto row = (double(box->row_min) + double(box->row_max)) * .5;
      const auto col = (double(box->col_min) + double(box->col_max)) * .5;
      const auto a = double(width - 1) * .5, b = double(height - 1) * .5;
      return a >= b ? Rectangle2{row, col, 0., a, b}
                    : Rectangle2{row, col, -3.14159265358979323846 / 2, b, a};
    }
  }
  const auto hull = hull_points(region);
  if (hull.empty()) return {};
  if (hull.size() == 1) return {hull[0].y, hull[0].x, 0.0, 0.0, 0.0};
  // 此有界整数范围给出严格凸包，叉积在 Double 中精确，非零方向间隙大于舍入噪声；
  // 小凸包或宽坐标保留原扫描。
  const bool calipers =
      hull.size() >= 32 && std::all_of(hull.begin(), hull.end(), [](const Point& p) {
        return std::abs(p.x) <= 0x1p20 && std::abs(p.y) <= 0x1p20;
      });
  std::array<std::size_t, 4> support{};
  double best_area = std::numeric_limits<double>::infinity();
  Rectangle2 best{};
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    double phi = std::atan2(b.y - a.y, b.x - a.x);
    const double c = std::cos(phi), s = std::sin(phi);
    double umin = std::numeric_limits<double>::infinity(), umax = -umin, vmin = umin, vmax = -umin;
    if (!calipers) {
      for (const auto& p : hull) {
        const double u = p.x * c + p.y * s, v = -p.x * s + p.y * c;
        umin = std::min(umin, u);
        umax = std::max(umax, u);
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
      }
    } else if (i == 0) {
      // 四个极值以同一表达式及首值同值次序初始化。
      for (std::size_t j = 0; j < hull.size(); ++j) {
        const auto& p = hull[j];
        const double u = p.x * c + p.y * s, v = -p.x * s + p.y * c;
        if (u < umin) {
          umin = u;
          support[0] = j;
        }
        if (u > umax) {
          umax = u;
          support[1] = j;
        }
        if (v < vmin) {
          vmin = v;
          support[2] = j;
        }
        if (v > vmax) {
          vmax = v;
          support[3] = j;
        }
      }
    } else {
      const auto extreme = [&](unsigned which) {
        const auto projection = [&](std::size_t j) {
          const auto& p = hull[j];
          return which < 2 ? p.x * c + p.y * s : -p.x * s + p.y * c;
        };
        auto j = support[which];
        auto value = projection(j);
        for (std::size_t steps = 0; steps < hull.size(); ++steps) {
          const auto next = j + 1 == hull.size() ? 0 : j + 1;
          const auto next_value = projection(next);
          if ((which & 1) ? next_value >= value : next_value <= value) {
            j = next;
            value = next_value;
          } else
            break;
        }
        auto best_index = j;
        // 支撑面有两个端点；评估相邻端点，保留原数组首项同值规则，含带符号零。
        for (const auto neighbor :
             {j == 0 ? hull.size() - 1 : j - 1, j + 1 == hull.size() ? 0 : j + 1}) {
          const auto candidate = projection(neighbor);
          if (((which & 1) ? candidate > value : candidate < value) ||
              (candidate == value && neighbor < best_index)) {
            value = candidate;
            best_index = neighbor;
          }
        }
        support[which] = best_index;
        return value;
      };
      umin = extreme(0);
      umax = extreme(1);
      vmin = extreme(2);
      vmax = extreme(3);
    }
    const double area = (umax - umin) * (vmax - vmin);
    if (area < best_area - 1e-12) {
      best_area = area;
      double l1 = (umax - umin) / 2, l2 = (vmax - vmin) / 2;
      double uc = (umin + umax) / 2, vc = (vmin + vmax) / 2;
      double x = uc * c - vc * s, y = uc * s + vc * c;
      if (l2 > l1) {
        std::swap(l1, l2);
        phi += 3.14159265358979323846 / 2;
      }
      best = {y, x, normalize_phi(phi), l1, l2};
    }
  }
  return best;
}

std::vector<Rectangle2> smallest_rectangle2(const mv::core::Regions& regions){ std::vector<Rectangle2> out;out.reserve(regions.size());for(std::size_t index=0;index<regions.size();++index)out.push_back(smallest_rectangle2(regions[index]));return out; }

Circle smallest_circle(const mv::core::Region& region) {
  if(const auto box=region.bounding_box();box &&
      box->row_min>=INT32_MIN && box->row_max<=INT32_MAX &&
      box->col_min>=INT32_MIN && box->col_max<=INT32_MAX) {
    const auto width=std::uint64_t(box->col_max-box->col_min)+1;
    const auto height=std::uint64_t(box->row_max-box->row_min)+1;
    if(height<=UINT64_MAX/width && region.area()==width*height)
      return {(double(box->row_min)+double(box->row_max))*.5,
              (double(box->col_min)+double(box->col_max))*.5,
              std::hypot(double(width-1),double(height-1))*.5};
  }
  const auto pts=hull_points(region); if(pts.empty()) return {};
  RawCircle c{{pts[0].x,pts[0].y},0.0};
  for(std::size_t i=0;i<pts.size();++i) if(!inside(c,pts[i])){
    c={{pts[i].x,pts[i].y},0.0};
    for(std::size_t j=0;j<i;++j) if(!inside(c,pts[j])){
      c=diameter(pts[i],pts[j]);
      for(std::size_t k=0;k<j;++k) if(!inside(c,pts[k])){
        const auto candidate=circum(pts[i],pts[j],pts[k]); if(candidate.r>=0) c=candidate;
      }
    }
  }
  return {c.c.y,c.c.x,c.r};
}
std::vector<Circle> smallest_circle(const mv::core::Regions& regions) {
  const auto objects=regions.iconic_tuple().values();
  std::vector<Circle> out;out.reserve(objects.size());
  std::uint64_t previous_width=0,previous_height=0;double radius=0;
  for(std::size_t index=0;index<objects.size();++index) {
    const auto& region=objects[index].as_region();const auto box=region.bounding_box();
    if(box && box->row_min>=INT32_MIN && box->row_max<=INT32_MAX &&
       box->col_min>=INT32_MIN && box->col_max<=INT32_MAX) {
      const auto width=std::uint64_t(box->col_max-box->col_min)+1;
      const auto height=std::uint64_t(box->row_max-box->row_min)+1;
      if(height<=UINT64_MAX/width && region.area()==width*height) {
        if(width!=previous_width || height!=previous_height) {
          radius=std::hypot(double(width-1),double(height-1))*.5;
          previous_width=width;previous_height=height;
        }
        out.push_back({(double(box->row_min)+double(box->row_max))*.5,
                       (double(box->col_min)+double(box->col_max))*.5,radius});
        continue;
      }
    }
    out.push_back(smallest_circle(region));
  }
  return out;
}

}  // namespace mv::regionproc
