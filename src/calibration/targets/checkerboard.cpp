#include <calibration/targets/find_calib_object.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <core/exception.hpp>

namespace mv::calibration {
namespace {
struct Candidate { double row{}, col{}, score{}; };

const std::uint8_t* gray8(const core::Image& image) {
  if (image.empty() || image.pixel_type() != core::PixelType::UInt8)
    throw core::Exception(core::Status::InvalidArgument,"detect_checkerboard_points","Gray8 image required");
  return static_cast<const std::uint8_t*>(image.data());
}

double quadrant_mean(const core::Image& im,int row,int col,int dr0,int dr1,int dc0,int dc1) {
  const auto* p=gray8(im); double sum=0; int n=0;
  for(int dr=dr0;dr<=dr1;++dr) for(int dc=dc0;dc<=dc1;++dc){
    const int r=row+dr,c=col+dc;
    if(r<0||c<0||r>=static_cast<int>(im.height())||c>=static_cast<int>(im.width())) continue;
    sum += p[static_cast<std::size_t>(r)*im.stride()+static_cast<std::size_t>(c)]; ++n;
  }
  return n?sum/static_cast<double>(n):0.0;
}

std::vector<core::Point2d> order_grid(std::vector<Candidate> pts,std::int64_t rows,std::int64_t cols) {
  const std::size_t n=static_cast<std::size_t>(rows*cols);
  if(pts.size()<n) throw core::Exception(core::Status::NotFound,"detect_checkerboard_points","not enough checkerboard corners");
  std::sort(pts.begin(),pts.end(),[](const Candidate&a,const Candidate&b){return a.score>b.score;});
  if(pts.size()>n) pts.resize(n);
  std::sort(pts.begin(),pts.end(),[](const Candidate&a,const Candidate&b){return a.row<b.row;});
  std::vector<core::Point2d> out; out.reserve(n);
  for(std::int64_t r=0;r<rows;++r){
    auto first=pts.begin()+r*cols, last=first+cols;
    std::sort(first,last,[](const Candidate&a,const Candidate&b){return a.col<b.col;});
    for(auto it=first;it!=last;++it) out.push_back({it->row,it->col});
  }
  return out;
}
}

std::vector<core::Point2d> detect_checkerboard_points(const core::Image& image,
                                                      std::int64_t rows,
                                                      std::int64_t columns) {
  (void)gray8(image);
  if(rows<2||columns<2) throw core::Exception(core::Status::InvalidArgument,"detect_checkerboard_points","invalid grid size");
  constexpr int rad=4;
  std::vector<Candidate> raw;
  for(int r=rad+1;r<static_cast<int>(image.height())-rad-1;++r){
    for(int c=rad+1;c<static_cast<int>(image.width())-rad-1;++c){
      const double q00=quadrant_mean(image,r,c,-rad,-1,-rad,-1);
      const double q01=quadrant_mean(image,r,c,-rad,-1,1,rad);
      const double q10=quadrant_mean(image,r,c,1,rad,-rad,-1);
      const double q11=quadrant_mean(image,r,c,1,rad,1,rad);
      const double diag_same=std::abs(q00-q11)+std::abs(q01-q10);
      const double contrast=std::abs((q00+q11)-(q01+q10))*0.5;
      if(contrast<90.0||diag_same>75.0) continue;
      raw.push_back({static_cast<double>(r),static_cast<double>(c),contrast-0.25*diag_same});
    }
  }
  std::sort(raw.begin(),raw.end(),[](const Candidate&a,const Candidate&b){return a.score>b.score;});
  std::vector<Candidate> nms;
  const std::size_t need=static_cast<std::size_t>(rows*columns);
  for(const auto& p:raw){
    bool near=false;
    for(const auto& q:nms) if(std::hypot(p.row-q.row,p.col-q.col)<8.0){near=true;break;}
    if(!near) nms.push_back(p);
    if(nms.size()>=need) break;
  }
  // 亚像素细化取 3x3 内按灰度加权的棋盘响应质心。
  for(auto& p:nms){
    double sr=0,sc=0,sw=0;
    const int br=static_cast<int>(std::lround(p.row)),bc=static_cast<int>(std::lround(p.col));
    for(int dr=-1;dr<=1;++dr) for(int dc=-1;dc<=1;++dc){
      const int rr=br+dr,cc=bc+dc;
      const double q00=quadrant_mean(image,rr,cc,-rad,-1,-rad,-1);
      const double q01=quadrant_mean(image,rr,cc,-rad,-1,1,rad);
      const double q10=quadrant_mean(image,rr,cc,1,rad,-rad,-1);
      const double q11=quadrant_mean(image,rr,cc,1,rad,1,rad);
      const double w=std::max(0.0,std::abs((q00+q11)-(q01+q10))*0.5-(std::abs(q00-q11)+std::abs(q01-q10))*0.25);
      sr+=w*rr; sc+=w*cc; sw+=w;
    }
    if(sw>0){p.row=sr/sw;p.col=sc/sw;}
  }
  return order_grid(std::move(nms),rows,columns);
}
}
