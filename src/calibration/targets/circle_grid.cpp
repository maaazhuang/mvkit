#include <calibration/targets/find_calib_object.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {
struct Blob { double row{},col{}; std::size_t area{}; };
}
std::vector<core::Point2d> detect_circle_grid_points(const core::Image& image,
                                                     std::int64_t rows,
                                                     std::int64_t columns) {
  if(image.empty()||image.pixel_type()!=core::PixelType::UInt8)
    throw core::Exception(core::Status::InvalidArgument,"detect_circle_grid_points","Gray8 image required");
  if(rows<2||columns<2) throw core::Exception(core::Status::InvalidArgument,"detect_circle_grid_points","invalid grid size");
  const auto* p=static_cast<const std::uint8_t*>(image.data());
  std::uint8_t lo=255,hi=0;
  for(std::size_t r=0;r<image.height();++r) for(std::size_t c=0;c<image.width();++c){auto v=p[r*image.stride()+c];lo=std::min(lo,v);hi=std::max(hi,v);}
  if(static_cast<int>(hi)-static_cast<int>(lo)<40) throw core::Exception(core::Status::NotFound,"detect_circle_grid_points","insufficient image contrast");
  const int threshold=(static_cast<int>(lo)+static_cast<int>(hi))/2;
  const std::size_t w=image.width(),h=image.height();
  std::vector<std::uint8_t> seen(w*h,0); std::vector<Blob> blobs;
  constexpr int drs[4]={-1,1,0,0},dcs[4]={0,0,-1,1};
  for(std::size_t r=0;r<h;++r) for(std::size_t c=0;c<w;++c){
    const std::size_t idx=r*w+c; if(seen[idx]||p[r*image.stride()+c]>=threshold) continue;
    std::deque<std::pair<int,int>> q; q.emplace_back(static_cast<int>(r),static_cast<int>(c)); seen[idx]=1;
    double sr=0,sc=0; std::size_t area=0;
    while(!q.empty()){auto [rr,cc]=q.front();q.pop_front();sr+=rr;sc+=cc;++area;
      for(int k=0;k<4;++k){int nr=rr+drs[k],nc=cc+dcs[k];if(nr<0||nc<0||nr>=static_cast<int>(h)||nc>=static_cast<int>(w))continue;std::size_t ni=static_cast<std::size_t>(nr)*w+static_cast<std::size_t>(nc);if(!seen[ni]&&p[static_cast<std::size_t>(nr)*image.stride()+static_cast<std::size_t>(nc)]<threshold){seen[ni]=1;q.emplace_back(nr,nc);}}
    }
    if(area>=20&&area<=2000) blobs.push_back({sr/area,sc/area,area});
  }
  const std::size_t need=static_cast<std::size_t>(rows*columns);
  if(blobs.size()<need) throw core::Exception(core::Status::NotFound,"detect_circle_grid_points","not enough circle marks");
  std::sort(blobs.begin(),blobs.end(),[](const Blob&a,const Blob&b){return a.area>b.area;}); if(blobs.size()>need)blobs.resize(need);
  std::sort(blobs.begin(),blobs.end(),[](const Blob&a,const Blob&b){return a.row<b.row;});
  std::vector<core::Point2d> out;out.reserve(need);
  for(std::int64_t r=0;r<rows;++r){auto first=blobs.begin()+r*columns,last=first+columns;std::sort(first,last,[](const Blob&a,const Blob&b){return a.col<b.col;});for(auto it=first;it!=last;++it)out.push_back({it->row,it->col});}
  return out;
}
}
