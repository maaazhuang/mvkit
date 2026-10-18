#include <xld/operators.hpp>
#include <xld/contour_utils.hpp>
#include <runtime/execution/execution_context.hpp>
#include <runtime/memory/memory_resource.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory_resource>
#include <limits>
#include <numbers>
#include <numeric>
#include <span>
#include <type_traits>
#include <vector>
#include <core/exception.hpp>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_RDP_SSE2 1
#else
#define MVKIT_RDP_SSE2 0
#endif

namespace mv::xld {
namespace {
using mv::core::XldContour;
using mv::core::ContourApproximation;
using mv::core::Point2d;
using mv::runtime::ExecutionContext;

struct Regression { double row{}, col{}, dr{}, dc{1.0}, mean_distance{}, std_distance{}; };
Regression regression(std::span<const Point2d> p, std::pmr::memory_resource* memory){
  Regression r{}; if(p.empty()) return r;
  for(const auto&q:p){r.row+=q.row;r.col+=q.col;} r.row/=p.size();r.col/=p.size();
  double rr=0,cc=0,rc=0;
  for(const auto&q:p){const double dy=q.row-r.row,dx=q.col-r.col;rr+=dy*dy;cc+=dx*dx;rc+=dy*dx;}
  const double theta=0.5*std::atan2(2.0*rc,cc-rr); r.dc=std::cos(theta); r.dr=std::sin(theta);
  if(p.size()>1){const double er=p.back().row-p.front().row,ec=p.back().col-p.front().col;if(er*r.dr+ec*r.dc<0){r.dr=-r.dr;r.dc=-r.dc;}}
  std::pmr::vector<double> d{memory}; d.reserve(p.size());
  for(const auto&q:p)d.push_back(std::abs((q.row-r.row)*r.dc-(q.col-r.col)*r.dr));
  r.mean_distance=std::accumulate(d.begin(),d.end(),0.0)/d.size(); double var=0;for(double v:d){const double dv=v-r.mean_distance;var+=dv*dv;}r.std_distance=std::sqrt(var/d.size()); return r;
}

template<std::size_t Extent>
Point2d smooth_projection(Point2d p, std::span<const Point2d,Extent> neighborhood) {
  double row = 0, col = 0;
  for (const auto q : neighborhood) { row += q.row; col += q.col; }
  row /= neighborhood.size(); col /= neighborhood.size();
  double rr = 0, cc = 0, rc = 0;
  for (const auto q : neighborhood) {
    const double dy = q.row-row, dx = q.col-col;
    rr += dy*dy; cc += dx*dx; rc += dy*dx;
  }
  const double difference = cc-rr, cross = 2*rc;
  const double squared = difference*difference+cross*cross;
  const double disc = squared >= 1e-300 && squared < std::numeric_limits<double>::infinity()
      ? std::sqrt(squared) : std::hypot(difference,cross);
  if (disc == 0) return {row,p.col};
  // 投影到协方差主轴的谱投影器；方向符号不影响投影，避免 atan2/sin/cos 及无用残差统计。
  const double dy = p.row-row, dx = p.col-col;
  if (!mv::core::is_finite(disc)) {
    const double theta = .5*std::atan2(cross,difference);
    const double dr = std::sin(theta), dc = std::cos(theta);
    const double t = dy*dr+dx*dc;
    return {row+t*dr,col+t*dc};
  }
  const double diagonal = difference/disc, off_diagonal = cross/disc;
  return {row + .5*((1-diagonal)*dy+off_diagonal*dx),
          col + .5*(off_diagonal*dy+(1+diagonal)*dx)};
}

#if MVKIT_RDP_SSE2
// 独立相邻窗口在各 Double lane 保留标量加法序，端点及异常范数使用原投影器。
template<std::size_t Window>
void smooth_projection_pair(std::span<const Point2d> points,std::size_t index,Point2d* output) {
  constexpr std::size_t radius=Window/2;
  auto row=_mm_setzero_pd(),col=_mm_setzero_pd();
  const auto coordinates=[&](std::size_t j) {
    const auto a=_mm_loadu_pd(&points[j].row),b=_mm_loadu_pd(&points[j+1].row);
    return std::pair{_mm_unpacklo_pd(a,b),_mm_unpackhi_pd(a,b)};
  };
  for(std::size_t j=0;j<Window;++j) {
    const auto [r,c]=coordinates(index-radius+j);row=_mm_add_pd(row,r);col=_mm_add_pd(col,c);
  }
  const auto count=_mm_set1_pd(double(Window));row=_mm_div_pd(row,count);col=_mm_div_pd(col,count);
  auto rr=_mm_setzero_pd(),cc=_mm_setzero_pd(),rc=_mm_setzero_pd();
  for(std::size_t j=0;j<Window;++j) {
    const auto [r,c]=coordinates(index-radius+j);const auto dy=_mm_sub_pd(r,row),dx=_mm_sub_pd(c,col);
    rr=_mm_add_pd(rr,_mm_mul_pd(dy,dy));cc=_mm_add_pd(cc,_mm_mul_pd(dx,dx));rc=_mm_add_pd(rc,_mm_mul_pd(dy,dx));
  }
  const auto difference=_mm_sub_pd(cc,rr),cross=_mm_mul_pd(_mm_set1_pd(2.),rc);
  const auto squared=_mm_add_pd(_mm_mul_pd(difference,difference),_mm_mul_pd(cross,cross));
  const auto ordinary=_mm_and_pd(_mm_cmpge_pd(squared,_mm_set1_pd(1e-300)),
      _mm_cmplt_pd(squared,_mm_set1_pd(std::numeric_limits<double>::infinity())));
  if(_mm_movemask_pd(ordinary)!=3) {
    for(std::size_t k=0;k<2;++k)output[k]=smooth_projection(points[index+k],
        std::span<const Point2d,Window>{points.data()+index+k-radius,Window});
    return;
  }
  const auto disc=_mm_sqrt_pd(squared),diagonal=_mm_div_pd(difference,disc),off_diagonal=_mm_div_pd(cross,disc);
  const auto [r,c]=coordinates(index);const auto dy=_mm_sub_pd(r,row),dx=_mm_sub_pd(c,col);
  const auto one=_mm_set1_pd(1.),half=_mm_set1_pd(.5);
  const auto result_row=_mm_add_pd(row,_mm_mul_pd(half,_mm_add_pd(_mm_mul_pd(_mm_sub_pd(one,diagonal),dy),_mm_mul_pd(off_diagonal,dx))));
  const auto result_col=_mm_add_pd(col,_mm_mul_pd(half,_mm_add_pd(_mm_mul_pd(off_diagonal,dy),_mm_mul_pd(_mm_add_pd(one,diagonal),dx))));
  _mm_storeu_pd(&output[0].row,_mm_unpacklo_pd(result_row,result_col));
  _mm_storeu_pd(&output[1].row,_mm_unpackhi_pd(result_row,result_col));
}
#endif


double endpoint_distance(const XldContour& c){const auto p=c.points();if(p.size()<2)return 0;return std::hypot(p.front().row-p.back().row,p.front().col-p.back().col);}
double maximum_extent(const XldContour& c){const auto p=c.points();double best=0;for(std::size_t i=0;i<p.size();++i)for(std::size_t j=i+1;j<p.size();++j)best=std::max(best,std::hypot(p[i].row-p[j].row,p[i].col-p[j].col));return best;}
double direction(const XldContour& c, std::pmr::memory_resource* memory){auto r=regression(c.points(),memory);double a=std::atan2(-r.dr,r.dc);if(a<0)a+=2.0*std::numbers::pi_v<double>;return a;}
bool angle_in_range(double a,double lo,double hi){constexpr double t=2.0*std::numbers::pi_v<double>;auto norm=[&](double x){x=std::fmod(x,t);if(x<0)x+=t;return x;};if(hi-lo>=t)return true;a=norm(a);lo=norm(lo);hi=norm(hi);return lo<=hi?(a>=lo&&a<=hi):(a>=lo||a<=hi);}
XldContour make_with_points(const XldContour& src,std::span<const Point2d> pts,ContourApproximation approx){
  if(pts.size()==src.size()) return XldContour::from_points_and_attributes(pts,src.closed(),approx,src.point_attribute(mv::core::XldPointAttribute::Response),src.point_attribute(mv::core::XldPointAttribute::EdgeDirection),src.point_attribute(mv::core::XldPointAttribute::NormalAngle));
  return XldContour::from_points(pts,false);
}
void rdp_splits(std::span<const Point2d> p,std::size_t a,std::size_t b,double tol,std::pmr::vector<std::size_t>& splits,bool simd,bool minimum_points=false,std::span<const double> length_prefix={}){
  if(b<=a+1||(minimum_points&&b-a<10)) return;
  const auto A=p[a],B=p[b];
  const double vr=B.row-A.row,vc=B.col-A.col,len=std::hypot(vr,vc);
  double best=-1;
  std::size_t idx=a;
  const double below=std::nextafter(tol*len,0.);
  std::size_t i=a+(minimum_points?5:1);
  auto search_end=b-(minimum_points?4:0);
  if(!length_prefix.empty()){while(i<search_end&&length_prefix[i]-length_prefix[a]<3)++i;while(search_end>i&&length_prefix[b]-length_prefix[search_end-1]<3)--search_end;}
#if MVKIT_RDP_SSE2
  if(simd && len!=0) {
    const auto ar=_mm_set1_pd(A.row),ac=_mm_set1_pd(A.col),vr2=_mm_set1_pd(vr),vc2=_mm_set1_pd(vc);
    for(;search_end-i>=2;i+=2) {
      const auto p0=_mm_loadu_pd(&p[i].row),p1=_mm_loadu_pd(&p[i+1].row);
      const auto dy=_mm_sub_pd(_mm_unpacklo_pd(p0,p1),ar),dx=_mm_sub_pd(_mm_unpackhi_pd(p0,p1),ac);
      const auto numerator=_mm_andnot_pd(_mm_set1_pd(-0.),_mm_sub_pd(_mm_mul_pd(dy,vc2),_mm_mul_pd(dx,vr2)));
      const auto mask=_mm_movemask_pd(_mm_cmpgt_pd(numerator,_mm_set1_pd(below)));
      if(mask) {double values[2];_mm_storeu_pd(values,numerator);for(int j=0;j<2;++j)if(mask&(1<<j)) {
        const double d=values[j]/len;if(d>best){best=d;idx=i+j;}
      }}
    }
  }
#else
  (void)simd;
#endif
  for(;i<search_end;++i){
    double d;
    if(len==0)d=std::hypot(p[i].row-A.row,p[i].col-A.col);
    else {const auto numerator=std::abs((p[i].row-A.row)*vc-(p[i].col-A.col)*vr);if(numerator<=below)continue;d=numerator/len;}
    if(d>best){best=d;idx=i;}
  }
  if(best>tol){rdp_splits(p,a,idx,tol,splits,simd,minimum_points,length_prefix);splits.push_back(idx);rdp_splits(p,idx,b,tol,splits,simd,minimum_points,length_prefix);}
}
double max_chord_distance(std::span<const Point2d> p){
  if(p.size()<3) return 0;
  const auto A=p.front(),B=p.back();
  const double vr=B.row-A.row,vc=B.col-A.col,len=std::hypot(vr,vc);
  double best=0;
  for(std::size_t i=1;i+1<p.size();++i){
    const double d=len==0?std::hypot(p[i].row-A.row,p[i].col-A.col)
                         :std::abs((p[i].row-A.row)*vc-(p[i].col-A.col)*vr)/len;
    best=std::max(best,d);
  }
  return best;
}
bool algebraic_circle(std::span<const Point2d> p,double& row,double& col,double& radius){
  std::array<std::array<double,3>,3> a{};
  std::array<double,3> b{},x{};
  for(const auto& q:p){
    const double px=q.col,py=q.row,z[3]{px,py,1.0},rhs=-(px*px+py*py);
    for(int r=0;r<3;++r){b[r]+=z[r]*rhs;for(int c=0;c<3;++c)a[r][c]+=z[r]*z[c];}
  }
  if(!detail::solve(a,b,x)) return false;
  col=-0.5*x[0];row=-0.5*x[1];
  const double rsq=col*col+row*row-x[2];
  if(!(rsq>0.0)||!std::isfinite(rsq)) return false;
  radius=std::sqrt(rsq);
  return std::isfinite(row)&&std::isfinite(col)&&std::isfinite(radius)&&radius>0.0;
}
bool circle_residual_ok(std::span<const Point2d> p,double row,double col,double radius,double tol){
  for(const auto& q:p){
    if(std::abs(std::hypot(q.row-row,q.col-col)-radius)>tol) return false;
  }
  return true;
}
bool algebraic_ellipse(std::span<const Point2d> p,double& row,double& col,double& phi,double& radius1,double& radius2){
  std::array<std::array<double,5>,5> matrix{};
  std::array<double,5> rhs{},q{};
  for(const auto& pt:p){
    const double x=pt.col,y=-pt.row;
    std::array<double,5> z{x*x,2*x*y,y*y,x,y};
    for(int i=0;i<5;++i){rhs[i]+=z[i];for(int j=0;j<5;++j)matrix[i][j]+=z[i]*z[j];}
  }
  if(!detail::solve(matrix,rhs,q)) return false;
  const double det=q[0]*q[2]-q[1]*q[1];
  if(!(det>0.0)) return false;
  const double x=(q[1]*q[4]-q[2]*q[3])/(2*det),y=(q[1]*q[3]-q[0]*q[4])/(2*det);
  double theta=0.5*std::atan2(2*q[1],q[0]-q[2]),ct=std::cos(theta),st=std::sin(theta);
  const double aa=q[0]*ct*ct+2*q[1]*ct*st+q[2]*st*st,bb=q[0]*st*st-2*q[1]*ct*st+q[2]*ct*ct,
               k=1+q[0]*x*x+2*q[1]*x*y+q[2]*y*y;
  if(!(k/aa>0.0&&k/bb>0.0)) return false;
  double a=std::sqrt(k/aa),b=std::sqrt(k/bb);
  if(a<b){std::swap(a,b);theta+=detail::pi/2;}
  if(!(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(theta)&&std::isfinite(a)&&std::isfinite(b)&&a>=b&&b>0.0)) return false;
  row=-y;col=x;phi=theta;radius1=a;radius2=b;return true;
}
bool ellipse_residual_ok(std::span<const Point2d> p,double row,double col,double phi,double a,double b,double tol){
  const double c=std::cos(phi),s=std::sin(phi);
  for(const auto& q:p){
    const double dx=q.col-col,dy=-(q.row-row),xr=c*dx+s*dy,yr=-s*dx+c*dy,n=std::hypot(xr/a,yr/b);
    if(n<=1e-15) continue;
    const double dist=std::abs(1.0-n)*std::hypot(xr,yr)/n;
    if(dist>tol) return false;
  }
  return true;
}
bool classify_arc(std::span<const Point2d> p,double tol,ContourSegmentationMode mode,ContourApproximation& kind){
  if(max_chord_distance(p)<=tol) return false;
  double crow=0,ccol=0,cr=0;
  const bool circle=p.size()>=5&&algebraic_circle(p,crow,ccol,cr)&&cr>tol&&circle_residual_ok(p,crow,ccol,cr,tol);
  if(mode==ContourSegmentationMode::LinesCircles){
    if(!circle) return false;
    kind=ContourApproximation::Circle;
    return true;
  }
  double erow=0,ecol=0,ephi=0,ea=0,eb=0;
  const bool ellipse=p.size()>=6&&algebraic_ellipse(p,erow,ecol,ephi,ea,eb)&&eb>tol&&ea/eb<=50.0&&
                     ellipse_residual_ok(p,erow,ecol,ephi,ea,eb,tol);
  if(circle&&(!ellipse||(ea-eb)<=0.08*ea)){kind=ContourApproximation::Circle;return true;}
  if(ellipse){kind=ContourApproximation::Ellipse;return true;}
  return false;
}
}

mv::core::XldContours select_contours_xld(const mv::core::XldContours& contours,XldSelectionFeature feature,double min1,double max1,double min2,double max2){
  ExecutionContext context; auto* memory=context.scratch_resource();
  std::vector<mv::core::IconicValue> out;
  out.reserve(contours.contour_count());
  for(std::size_t index=0;index<contours.contour_count();++index){const auto&c=contours.contour(index);bool keep=false;switch(feature){
    case XldSelectionFeature::ContourLength:{const double v=length_xld(c);keep=v>=min1&&v<=max1;break;}
    case XldSelectionFeature::MaximumExtent:{const double v=maximum_extent(c);keep=v>=min1&&v<=max1;break;}
    case XldSelectionFeature::Direction:{const double a=direction(c,memory);keep=angle_in_range(a,min1,max1)||angle_in_range(a+std::numbers::pi,min1,max1);break;}
    case XldSelectionFeature::Curvature:{auto r=regression(c.points(),memory);const bool m=(min1==0&&max1==0)||(r.mean_distance>=min1&&r.mean_distance<=max1);const bool s=(min2==0&&max2==0)||(r.std_distance>=min2&&r.std_distance<=max2);keep=m&&s;break;}
    case XldSelectionFeature::Closed:keep=endpoint_distance(c)<=max1;break;
    case XldSelectionFeature::Open:keep=endpoint_distance(c)>min1;break;}
    if(keep)out.push_back(mv::core::IconicValue::from_xld_contour(c));
  }return mv::core::XldContours{mv::core::IconicTuple{std::move(out)}};
}

mv::core::XldContours smooth_contours_xld(const mv::core::XldContours& contours,int n) {
  if(n<3||(n%2)==0)throw mv::core::Exception(mv::core::Status::InvalidArgument,"smooth_contours_xld","num_regr_points must be odd and >= 3");
  std::vector<mv::core::IconicValue> out;
  out.reserve(contours.contour_count());
  const std::size_t half = static_cast<std::size_t>(n/2);
  const auto& cpu=mv::runtime::cpu_features();
  const bool simd=MVKIT_RDP_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{true,false,false,false});
  for (std::size_t index=0; index<contours.contour_count(); ++index) {
    const auto& c=contours.contour(index); const auto p=c.points();
    std::pmr::vector<Point2d> q(p.size(),mv::runtime::persist_resource());
    auto* destination=q.data();
    const auto fixed_window = [&](auto size) {
      constexpr std::size_t window = decltype(size)::value, radius = window/2;
      std::size_t i=0;
      for(;i<std::min(radius,p.size());++i) {
        const auto b=std::min(p.size(),i+radius+1);
        destination[i]=smooth_projection(p[i],p.first(b));
      }
#if MVKIT_RDP_SSE2
      if(simd)for(;p.size()-i>radius+1;i+=2)smooth_projection_pair<window>(p,i,destination+i);
#endif
      for(;i<p.size();++i) {
        if(i>=radius && p.size()-i>radius)
          destination[i]=smooth_projection(p[i],std::span<const Point2d,window>{p.data()+i-radius,window});
        else {
          const auto a=i>radius?i-radius:0,b=std::min(p.size(),i+radius+1);
          destination[i]=smooth_projection(p[i],p.subspan(a,b-a));
        }
      }
    };
    if (n==3) fixed_window(std::integral_constant<std::size_t,3>{});
    else if (n==5) fixed_window(std::integral_constant<std::size_t,5>{});
    else if (n==7) fixed_window(std::integral_constant<std::size_t,7>{});
    else for (std::size_t i=0;i<p.size();++i) {
      const std::size_t a=i>half?i-half:0,b=std::min(p.size(),i+half+1);
      destination[i]=smooth_projection(p[i],p.subspan(a,b-a));
    }
    out.push_back(mv::core::IconicValue::from_xld_contour(XldContour::from_owned_points_and_attributes(std::move(q),c.closed(),c.approximation(),
        c.point_attribute(mv::core::XldPointAttribute::Response),
        c.point_attribute(mv::core::XldPointAttribute::EdgeDirection),
        c.point_attribute(mv::core::XldPointAttribute::NormalAngle))));
  }
  return mv::core::XldContours{mv::core::IconicTuple{std::move(out)}};
}

static mv::core::XldContours segment_contours_impl(const mv::core::XldContours& contours,ContourSegmentationMode mode,int smooth_cont,double max_line_dist,bool preserve_coordinates){
  if(max_line_dist<0||!std::isfinite(max_line_dist))throw mv::core::Exception(mv::core::Status::InvalidArgument,"segment_contours_xld","unsupported mode or invalid distance");
  switch(mode){case ContourSegmentationMode::Lines:case ContourSegmentationMode::LinesCircles:case ContourSegmentationMode::LinesEllipses:break;default:throw mv::core::Exception(mv::core::Status::InvalidArgument,"segment_contours_xld","unsupported mode or invalid distance");}
  auto source=smooth_cont>0?smooth_contours_xld(contours,smooth_cont):contours;std::vector<XldContour> out;out.reserve(source.contour_count());
  const bool merge_arcs=mode!=ContourSegmentationMode::Lines;
  ExecutionContext context; auto* memory=context.scratch_resource();
  std::pmr::vector<std::size_t> cuts{memory};
  std::pmr::vector<double> length_prefix{memory};
  const auto& cpu=context.cpu();
  const bool simd=MVKIT_RDP_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{true,false,false,false});
  for(std::size_t index=0;index<source.contour_count();++index){
    const auto&c=source.contour(index);const auto p=c.points();const auto& stored=preserve_coordinates?contours.contour(index):c;const auto output_points=stored.points();
    if(p.size()<2){if(!p.empty())out.push_back(XldContour::from_points(p,false));continue;}
    if(preserve_coordinates){length_prefix.resize(output_points.size());length_prefix[0]=0;for(std::size_t i=1;i<output_points.size();++i)length_prefix[i]=length_prefix[i-1]+detail::point_distance(output_points[i-1],output_points[i]);}
    cuts.clear(); cuts.push_back(0);rdp_splits(p,0,p.size()-1,max_line_dist,cuts,simd,preserve_coordinates,length_prefix);cuts.push_back(p.size()-1);
    std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    if(!merge_arcs){
      for(std::size_t k=1;k<cuts.size();++k){const auto a=cuts[k-1],b=cuts[k];
        out.push_back(a==0 && b+1==p.size() ? stored.geometry_view(false,ContourApproximation::Line)
            : XldContour::from_points_and_attributes(output_points.subspan(a,b-a+1),false,ContourApproximation::Line));}
      continue;
    }
    std::size_t piece=0;
    while(piece+1<cuts.size()){
      std::size_t best_end=piece+1;auto best=ContourApproximation::Line;
      for(std::size_t end=piece+2;end<cuts.size();++end){
        ContourApproximation kind=ContourApproximation::Line;
        if(!classify_arc(p.subspan(cuts[piece],cuts[end]-cuts[piece]+1),max_line_dist,mode,kind)) break;
        best_end=end;best=kind;
      }
      out.push_back(XldContour::from_points_and_attributes(output_points.subspan(cuts[piece],cuts[best_end]-cuts[piece]+1),false,best));
      piece=best_end;
    }
  }
  return detail::objects(out);
}

mv::core::XldContours segment_contours_xld(const mv::core::XldContours& contours,ContourSegmentationMode mode,int smooth_cont,double distance) {
 return segment_contours_impl(contours,mode,smooth_cont,distance,false);
}

mv::core::XldContours segment_contours_xld(const mv::core::XldContours& input,ContourSegmentationMode mode,int smooth,double first,double second) {
 constexpr auto op="segment_contours_xld";
 detail::require(smooth>=0&&std::isfinite(first)&&first>=0&&std::isfinite(second)&&second>=0,op,"Parameters","nonnegative finite controls required");
 mv::core::XldContours result;
 for(std::size_t i=0;i<input.contour_count();++i) {
  const auto& contour=input.contour(i);detail::validate(contour,op);
  // HALCON 在平滑/近似前保留短输入轮廓。
  if(contour.size()<6||length_xld(contour)<3){result.append(contour);continue;}
  mv::core::XldContours one;one.append(contour);
  const int window=smooth<=1?0:2*(smooth/2)+1;
  const auto coarse=segment_contours_impl(one,mode,window,first,true);
  if(mode==ContourSegmentationMode::Lines||first==second){result=mv::core::XldContours{result.iconic_tuple().concat(coarse.iconic_tuple())};continue;}
  for(std::size_t j=0;j<coarse.contour_count();++j) {
   const auto& piece=coarse.contour(j);
   if(piece.approximation()!=ContourApproximation::Line){result.append(piece);continue;}
   mv::core::XldContours line;line.append(piece);
   result=mv::core::XldContours{result.iconic_tuple().concat(segment_contours_impl(line,mode,0,second,true).iconic_tuple())};
  }
 }
 return result;
}
} // namespace mv::xld
