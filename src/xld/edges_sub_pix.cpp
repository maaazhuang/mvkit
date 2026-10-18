#include <xld/operators.hpp>
#include <xld/contour_utils.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory_resource>
#include <numbers>
#include <span>
#include <bit>
#include <vector>

#include <core/exception.hpp>
#include <imageproc/border.hpp>
#include <imageproc/operators.hpp>
#include <imageproc/advanced.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <runtime/memory/overwrite_buffer.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <xld/fast_paths_avx2.hpp>

// 亚像素边缘主链路：梯度/NMS -> 法向二次定位 -> 滞后连接 -> 有序 XLD。

namespace mv::xld {
namespace {
using mv::core::ContourApproximation;
using mv::core::Exception;
using mv::core::Image;
using mv::core::Point2d;
using mv::core::Status;
using mv::core::XldContour;
using mv::core::XldContours;
using mv::core::XldPointAttribute;
double normal_length(double x, double y) {
  const double squared = x * x + y * y;
  if (squared >= 1e-300 && squared < std::numeric_limits<double>::infinity())
    return std::sqrt(squared);
  if (x == 0 && y == 0) return 0;
  return std::hypot(x, y);
}
struct Candidate {
  std::size_t packed=0;
  bool valid() const noexcept {return packed!=0;}
  bool strong() const noexcept {return (packed&1)!=0;}
  std::size_t location() const noexcept {return (packed>>1)-1;}
  static Candidate from_location(std::size_t index,bool strong) noexcept {
    // location 向量按 32 字节元素有界，所有支持 size_t 宽度下 index+1 均为低位 strong 标记留出空间。
    return {((index+1)<<1)|static_cast<std::size_t>(strong)};
  }
};
struct Localization {double row,col,response,edge_direction;};

std::pmr::vector<double> image_to_double(const Image& image, std::pmr::memory_resource* resource, double scale=1) {
  mv::imageproc::ImageView view{image};
  const auto width = image.width(), height = image.height();
  return mv::imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    std::pmr::vector<double> out(width * height, resource);
    for (std::size_t r = 0; r < height; ++r) {
      const auto* row = view.row_as<T>(r);
      for (std::size_t c = 0; c < width; ++c) {const double value=static_cast<double>(row[c]);out[r * width + c]=scale==1 ? value : value*scale;}
    }
    return out;
  });
}
std::pmr::vector<double> gaussian_kernel(double sigma,std::pmr::memory_resource* resource){const int radius=std::max(1,static_cast<int>(std::ceil(3.0*sigma)));std::pmr::vector<double>k(static_cast<std::size_t>(2*radius+1),resource);double sum=0;for(int i=-radius;i<=radius;++i){const double v=std::exp(-0.5*(i*i)/(sigma*sigma));k[static_cast<std::size_t>(i+radius)]=v;sum+=v;}for(auto&v:k)v/=sum;return k;}
std::pmr::vector<double> gaussian_blur(std::span<const double>src,std::size_t w,std::size_t h,double sigma,std::pmr::memory_resource* resource){auto k=gaussian_kernel(sigma,resource);const int radius=static_cast<int>(k.size()/2);std::pmr::vector<double>tmp(w*h,resource),dst(w*h,resource);for(std::size_t r=0;r<h;++r)for(std::size_t c=0;c<w;++c){double s=0;for(int d=-radius;d<=radius;++d){const auto x=mv::imageproc::border_index(static_cast<std::int64_t>(c)+d,w,mv::imageproc::BorderMode::Mirrored).value();s+=src[r*w+x]*k[static_cast<std::size_t>(d+radius)];}tmp[r*w+c]=s;}for(std::size_t r=0;r<h;++r)for(std::size_t c=0;c<w;++c){double s=0;for(int d=-radius;d<=radius;++d){const auto y=mv::imageproc::border_index(static_cast<std::int64_t>(r)+d,h,mv::imageproc::BorderMode::Mirrored).value();s+=tmp[y*w+c]*k[static_cast<std::size_t>(d+radius)];}dst[r*w+c]=s;}return dst;}
void sobel_gradient(std::span<const double>src,std::size_t w,std::size_t h,std::pmr::vector<double>&gx,std::pmr::vector<double>&gy){gx.assign(w*h,0);gy.assign(w*h,0);static constexpr int kx[3][3]={{-1,0,1},{-2,0,2},{-1,0,1}},ky[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};for(std::size_t r=0;r<h;++r)for(std::size_t c=0;c<w;++c){double sx=0,sy=0;for(int dr=-1;dr<=1;++dr){const auto rr=mv::imageproc::border_index(static_cast<std::int64_t>(r)+dr,h,mv::imageproc::BorderMode::Mirrored).value();for(int dc=-1;dc<=1;++dc){const auto cc=mv::imageproc::border_index(static_cast<std::int64_t>(c)+dc,w,mv::imageproc::BorderMode::Mirrored).value();const double v=src[rr*w+cc];sx+=v*kx[dr+1][dc+1];sy+=v*ky[dr+1][dc+1];}}gx[r*w+c]=sx/4.0;gy[r*w+c]=sy/4.0;}}
std::pmr::vector<std::uint8_t> domain_mask(const Image&image,std::pmr::memory_resource* resource){std::pmr::vector<std::uint8_t>m(image.width()*image.height(),0,resource);const auto domain=image.domain();for(const auto&run:domain.runs())for(std::int64_t c=run.col_begin;c<=run.col_end;++c)if(run.row>=0&&c>=0&&static_cast<std::size_t>(run.row)<image.height()&&static_cast<std::size_t>(c)<image.width())m[static_cast<std::size_t>(run.row)*image.width()+static_cast<std::size_t>(c)]=1;return m;}
void nms_candidates(const Image&image,std::span<const double>gx,std::span<const double>gy,double low,double high,std::pmr::vector<Candidate>&cand,std::pmr::vector<Localization>&locations,std::pmr::memory_resource* resource,bool facet){const auto w=image.width(),h=image.height();mv::runtime::OverwriteBuffer<double> mag(w*h,resource);std::size_t start=0;
#if MVKIT_XLD_HAS_AVX2
const auto& cpu=mv::runtime::cpu_features();
const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false});
if(avx2)start=gradient_magnitudes_avx2(gx,gy,mag);
#endif
for(std::size_t i=start;i<mag.size();++i)mag[i]=normal_length(gx[i],gy[i]);const bool full_domain=!image.has_reduced_domain();
const auto domain=full_domain ? std::pmr::vector<std::uint8_t>{resource} : domain_mask(image,resource);
cand.assign(w*h,{});if(w<3||h<3)return;for(std::size_t r=0;r<h;++r){
[[maybe_unused]] std::size_t pending_end=0;
[[maybe_unused]] std::uint32_t pending=0;
for(std::size_t c=0;c<w;++c){
#if MVKIT_XLD_HAS_AVX2
if(avx2 && full_domain) {
  const auto end=w;
  if(c>=pending_end && end-c>=32) {
    pending=gradient_candidates_mask32_avx2(mag.data()+r*w+c,low);pending_end=c+32;
  }
  if(c<pending_end) {
    if(!pending){c=pending_end-1;continue;}
    c=pending_end-32+std::countr_zero(pending);pending&=pending-1;
  }
}
#endif
const auto idx=r*w+c;if(!full_domain && !domain[idx])continue;const double m=mag[idx];if(m<low||m<=0)continue;const double ax=std::abs(gx[idx]),ay=std::abs(gy[idx]);const auto index=[&](int dr,int dc){
// Canny 仅访问内部，八邻偏移均有效；Facet 访问边界，仍用原镜像映射。
if(r>0 && r+1<h && c>0 && c+1<w)return std::size_t(std::ptrdiff_t(idx)+dr*std::ptrdiff_t(w)+dc);
return mv::imageproc::border_index(std::int64_t(r)+dr,h,mv::imageproc::BorderMode::Mirrored).value()*w+mv::imageproc::border_index(std::int64_t(c)+dc,w,mv::imageproc::BorderMode::Mirrored).value();};
if(facet){
 const auto at=[&](int dr,int dc){return mag[index(dr,dc)];};
 const double tl=at(-1,-1),tc=at(-1,0),tr=at(-1,1),ml=at(0,-1),mc=m,mr=at(0,1),bl=at(1,-1),bc=at(1,0),br=at(1,1);
 const double dx=(tr+mr+br-tl-ml-bl)/6.,dy=(bl+bc+br-tl-tc-tr)/6.;
 const double xx=(tr+mr+br+tl+ml+bl-2*(tc+mc+bc))/3.,yy=(bl+bc+br+tl+tc+tr-2*(ml+mc+mr))/3.,xy=(br+tl-bl-tr)/4.;
 const double disc=normal_length(xx-yy,2*xy),positive=(xx+yy+disc)*.5,negative=(xx+yy-disc)*.5;const double curvature=std::abs(positive)>std::abs(negative)?positive:negative;
 if(curvature>=-1e-12)continue;
 double nx=xy,ny=curvature-xx;if(normal_length(curvature-yy,xy)>normal_length(nx,ny)){nx=curvature-yy;ny=xy;}const double length=normal_length(nx,ny);if(length==0)continue;nx/=length;ny/=length;
 const double t=-(dx*nx+dy*ny)/curvature,dr=t*ny,dc=t*nx;
 if(std::abs(dr)>.5+1e-6||std::abs(dc)>.5+1e-6)continue;
 locations.push_back({double(r)+dr,double(c)+dc,m,positive_atan2(-gy[idx],gx[idx])});cand[idx]=Candidate::from_location(locations.size()-1,m>=high);
}else{
 std::size_t pidx{},nidx{};if(ay<=0.41421356237*ax){const int sc=gx[idx]>=0?1:-1;pidx=index(0,-sc);nidx=index(0,sc);}else if(ax<=0.41421356237*ay){const int sr=gy[idx]>=0?1:-1;pidx=index(-sr,0);nidx=index(sr,0);}else{const int sr=gy[idx]>=0?1:-1,sc=gx[idx]>=0?1:-1;pidx=index(-sr,-sc);nidx=index(sr,sc);}const double pm=mag[pidx],nm=mag[nidx];const double inv=1.0/m;
 if(!(m>pm&&m>=nm))continue;const double denom=pm-2*m+nm;double offset=0;if(std::abs(denom)>1e-12)offset=0.5*(pm-nm)/denom;offset=std::clamp(offset,-0.5,0.5);locations.push_back({static_cast<double>(r)+offset*gy[idx]*inv,static_cast<double>(c)+offset*gx[idx]*inv,m,positive_atan2(-gy[idx],gx[idx])});cand[idx]=Candidate::from_location(locations.size()-1,m>=high);
}}}}
std::pmr::vector<std::uint8_t> hysteresis(std::span<const Candidate>cand,std::size_t w,std::size_t h,std::pmr::memory_resource* resource){std::pmr::vector<std::uint8_t>keep(cand.size(),0,resource);std::pmr::deque<std::size_t>q(resource);for(std::size_t i=0;i<cand.size();++i)if(cand[i].valid()&&cand[i].strong()){keep[i]=1;q.push_back(i);}while(!q.empty()){const auto idx=q.front();q.pop_front();const auto r=idx/w,c=idx%w;for(int dr=-1;dr<=1;++dr)for(int dc=-1;dc<=1;++dc){if(dr==0&&dc==0)continue;const auto rr=static_cast<std::int64_t>(r)+dr,cc=static_cast<std::int64_t>(c)+dc;if(rr<0||cc<0||rr>=static_cast<std::int64_t>(h)||cc>=static_cast<std::int64_t>(w))continue;const auto ni=static_cast<std::size_t>(rr)*w+static_cast<std::size_t>(cc);if(cand[ni].valid()&&!keep[ni]){keep[ni]=1;q.push_back(ni);}}}return keep;}
struct NeighborList {
  std::array<std::size_t, 8> values{};
  std::size_t count{};
  [[nodiscard]] bool empty() const noexcept { return count == 0; }
  [[nodiscard]] std::size_t size() const noexcept { return count; }
  [[nodiscard]] std::size_t front() const noexcept { return values[0]; }
  [[nodiscard]] const std::size_t* begin() const noexcept {
    return values.data();
  }
  [[nodiscard]] const std::size_t* end() const noexcept {
    return values.data() + count;
  }
};

NeighborList neighbors(std::size_t idx,
                       std::span<const std::uint8_t> active,
                       std::size_t w, std::size_t h) {
  const auto r = idx / w;
  const auto c = idx % w;
  NeighborList result;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) continue;
      const auto rr = static_cast<std::int64_t>(r) + dr;
      const auto cc = static_cast<std::int64_t>(c) + dc;
      if (rr < 0 || cc < 0 || rr >= static_cast<std::int64_t>(h) ||
          cc >= static_cast<std::int64_t>(w))
        continue;
      const auto neighbor = static_cast<std::size_t>(rr) * w +
                            static_cast<std::size_t>(cc);
      if (active[neighbor]) result.values[result.count++] = neighbor;
    }
  }
  return result;
}
XldContours link_contours(std::span<const Candidate> cand,
                          std::span<const Localization> locations,
                          std::span<const std::uint8_t> keep,
                          std::size_t w, std::size_t h,
                          std::pmr::memory_resource* resource) {
  std::vector<mv::core::IconicValue> out;
  std::pmr::vector<std::uint8_t> unused(keep.begin(), keep.end(), resource);
  std::pmr::vector<std::uint8_t> degree(unused.size(), 0, resource);
  std::pmr::deque<std::size_t> endpoints(resource);
  std::size_t remaining = 0;
  std::size_t raster_cursor = 0;
  for (std::size_t index = 0; index < unused.size(); ++index) {
    if (!unused[index]) continue;
    ++remaining;
    degree[index] = static_cast<std::uint8_t>(
        neighbors(index, unused, w, h).size());
    if (degree[index] <= 1) endpoints.push_back(index);
  }

  while (remaining > 0) {
    std::size_t start = unused.size();
    while (!endpoints.empty()) {
      const auto candidate = endpoints.front();
      endpoints.pop_front();
      if (unused[candidate] && degree[candidate] <= 1) {
        start = candidate;
        break;
      }
    }
    if (start == unused.size()) {
      // 移除像素不会增加更早活跃点，保持栅格序但不重复扫描已耗尽前缀。
      for (std::size_t index = raster_cursor; index < unused.size(); ++index) {
        if (unused[index]) {
          start = index;
          raster_cursor = index+1;
          break;
        }
      }
    }
    if (start == unused.size()) break;

    std::pmr::vector<std::size_t> path(resource);
    std::size_t current = start;
    std::size_t previous = unused.size();
    while (current < unused.size() && unused[current]) {
      path.push_back(current);
      const auto active_neighbors = neighbors(current, unused, w, h);
      unused[current] = 0;
      degree[current] = 0;
      --remaining;
      for (const auto neighbor : active_neighbors) {
        if (!unused[neighbor] || degree[neighbor] == 0) continue;
        --degree[neighbor];
        if (degree[neighbor] == 1) endpoints.push_back(neighbor);
      }
      if (active_neighbors.empty()) break;
      std::size_t next = active_neighbors.front();
      if (previous < unused.size() && active_neighbors.size() > 1) {
        double best = -std::numeric_limits<double>::infinity();
        const double previous_row = locations[cand[current].location()].row - locations[cand[previous].location()].row;
        const double previous_col = locations[cand[current].location()].col - locations[cand[previous].location()].col;
        for (const auto neighbor : active_neighbors) {
          const double row = locations[cand[neighbor].location()].row - locations[cand[current].location()].row;
          const double col = locations[cand[neighbor].location()].col - locations[cand[current].location()].col;
          const double score = previous_row * row + previous_col * col;
          if (score > best) {
            best = score;
            next = neighbor;
          }
        }
      }
      previous = current;
      current = next;
    }
    if (path.size() < 2) continue;

    std::pmr::vector<Point2d> points(resource);
    std::pmr::vector<double> response(resource);
    std::pmr::vector<double> edge_direction(resource);
    std::pmr::vector<double> normal_angle(path.size(), resource);
    points.reserve(path.size());
    response.reserve(path.size());
    edge_direction.reserve(path.size());
    for (const auto index : path) {
      points.push_back({locations[cand[index].location()].row, locations[cand[index].location()].col});
      response.push_back(locations[cand[index].location()].response);
      edge_direction.push_back(locations[cand[index].location()].edge_direction);
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto& before = points[index == 0 ? index : index - 1];
      const auto& after = points[index + 1 < points.size() ? index + 1 : index];
      normal_angle[index] = positive_atan2(-(after.row - before.row), after.col - before.col);
    }
    const bool closed =
        path.size() > 2 &&
        std::max(std::llabs(static_cast<long long>(path.front() / w) -
                            static_cast<long long>(path.back() / w)),
                 std::llabs(static_cast<long long>(path.front() % w) -
                            static_cast<long long>(path.back() % w))) <= 1;
    out.push_back(mv::core::IconicValue::from_xld_contour(XldContour::from_points_and_attributes(
        points, closed, ContourApproximation::Unknown, response,
        edge_direction, normal_angle)));
  }
  return XldContours{mv::core::IconicTuple{std::move(out)}};
}
}

mv::core::XldContours edges_sub_pix(const Image& image,EdgeFilter filter,double alpha,double low,double high) {
  if(image.empty())throw Exception(Status::InvalidArgument,"edges_sub_pix","image must not be empty");
  if(!std::isfinite(alpha)||!std::isfinite(low)||!std::isfinite(high)||low<0||high<low)
    throw Exception(Status::InvalidArgument,"edges_sub_pix","invalid filter parameters");
  mv::runtime::ScratchArena arena;auto* resource=arena.resource();
  std::pmr::vector<double> src(resource),gx(resource),gy(resource);
  mv::runtime::OverwriteBuffer<double> paired_x(0,resource),paired_y(0,resource);
  bool paired=false;
  if(filter!=EdgeFilter::Canny||alpha<.1||alpha>32)src=image_to_double(image,resource);
  if(filter==EdgeFilter::Canny) {
    if(alpha<=0)throw Exception(Status::InvalidArgument,"edges_sub_pix","Canny alpha must be positive");
    if(alpha<.1||alpha>32) {
      src=gaussian_blur(src,image.width(),image.height(),alpha,resource);
      sobel_gradient(src,image.width(),image.height(),gx,gy);
    } else {
      const double response_scale=std::sqrt(2*std::numbers::pi_v<double>)*alpha;
      paired=mv::imageproc::advanced::gaussian_gradient_pair(image,alpha,response_scale,paired_x,paired_y);
      if(!paired) {
        gx=image_to_double(mv::imageproc::derivate_gauss(image,alpha,1,0),resource,response_scale);
        gy=image_to_double(mv::imageproc::derivate_gauss(image,alpha,0,1),resource,response_scale);
      }
    }
  } else sobel_gradient(src,image.width(),image.height(),gx,gy);
  const auto x=paired ? std::span<const double>{paired_x.data(),paired_x.size()} : std::span<const double>{gx};
  const auto y=paired ? std::span<const double>{paired_y.data(),paired_y.size()} : std::span<const double>{gy};
  std::pmr::vector<Candidate> cand(resource);
  std::pmr::vector<Localization> locations(resource);
  nms_candidates(image,x,y,low,high,cand,locations,resource,filter==EdgeFilter::Canny&&alpha>=.1&&alpha<=32);
  // 相邻 Taylor 拟合可在两像素描述同一物理边，图遍历前合并位置，避免重复路径。
  const auto width=image.width(),height=image.height();
  for(std::size_t i=0;i<cand.size();++i)if(cand[i].valid()) {
    const auto row=i/width,col=i%width;
    for(const auto [dr,dc]:std::array<std::pair<int,int>,4>{{{0,1},{1,-1},{1,0},{1,1}}}) {
      const auto r=std::int64_t(row)+dr,c=std::int64_t(col)+dc;
      if(r<0||c<0||r>=std::int64_t(height)||c>=std::int64_t(width))continue;
      const auto j=std::size_t(r)*width+std::size_t(c);if(!cand[j].valid())continue;
      const auto& a=locations[cand[i].location()];const auto& b=locations[cand[j].location()];
      if(std::hypot(a.row-b.row,a.col-b.col)>=.5)continue;
      if(a.response>=b.response)cand[j]={};else{cand[i]={};break;}
    }
  }
  auto keep=hysteresis(cand,image.width(),image.height(),resource);
  return link_contours(cand,locations,keep,image.width(),image.height(),resource);
}
} // namespace mv::xld
