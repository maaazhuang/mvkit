#include <runtime/cpu/cpu_features.hpp>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_FIT_SSE2 1
#else
#define MVKIT_FIT_SSE2 0
#endif
#include <xld/operators.hpp>
#include <runtime/execution/execution_context.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory_resource>
#include <numbers>
#include <span>
#include <vector>
#include <core/exception.hpp>

// 几何拟合统一在私有 XldContour 上执行，Public cardinality 在外层保持。

namespace mv::xld {
namespace {
using mv::core::Exception;
using mv::core::Point2d;
using mv::core::Status;
using mv::core::XldContour;
using mv::runtime::ExecutionContext;
using PmrPoints = std::pmr::vector<Point2d>;
using PmrReals = std::pmr::vector<double>;

PmrPoints sample_points(std::span<const Point2d> points,int max_num_points,std::pmr::memory_resource* memory){
  if(max_num_points<0||static_cast<std::size_t>(max_num_points)>=points.size()) return PmrPoints(points.begin(),points.end(),memory);
  if(max_num_points<2) throw Exception(Status::InvalidArgument,"fit_xld","max_num_points must be -1 or >= 2");
  PmrPoints out{memory};out.reserve(static_cast<std::size_t>(max_num_points));
  for(int i=0;i<max_num_points;++i){const double t=static_cast<double>(i)/(max_num_points-1);const auto idx=static_cast<std::size_t>(std::llround(t*(points.size()-1)));out.push_back(points[idx]);}
  return out;
}
double robust_sigma(std::span<const double> residuals,std::pmr::memory_resource* memory){if(residuals.empty())return 1.0;PmrReals a{memory};a.reserve(residuals.size());for(double r:residuals)a.push_back(std::abs(r));auto mid=a.begin()+static_cast<std::ptrdiff_t>(a.size()/2);std::nth_element(a.begin(),mid,a.end());return std::max(1e-12,1.4826*(*mid));}
void line_weights(std::span<const double> res,double sigma,double clip,LineFitAlgorithm alg,PmrReals& w){
  w.assign(res.size(),1.0);const double c=std::max(1e-12,clip*sigma);
  for(std::size_t i=0;i<res.size();++i){const double a=std::abs(res[i]);switch(alg){case LineFitAlgorithm::Regression:w[i]=1;break;case LineFitAlgorithm::Huber:w[i]=a<=c?1:c/a;break;case LineFitAlgorithm::Tukey:{const double u=a/c;w[i]=u<1?std::pow(1-u*u,2):0;break;}case LineFitAlgorithm::Drop:w[i]=a<=c?1:0;break;case LineFitAlgorithm::Gaussian:{const double u=a/c;w[i]=std::exp(-0.5*u*u);break;}}}
}
struct LineModel{double row{},col{},dr{},dc{1},nr{1},nc{},dist{};};
template<bool UnitWeights=false>
LineModel fit_line_weighted(std::span<const Point2d> p,std::span<const double> w){double sw=0,mr=0,mc=0;for(std::size_t i=0;i<p.size();++i){const double weight=UnitWeights?1.0:w[i];sw+=weight;mr+=weight*p[i].row;mc+=weight*p[i].col;}if(sw<=1e-15)throw Exception(Status::InvalidArgument,"fit_line_contour_xld","all fitting weights are zero");mr/=sw;mc/=sw;double rr=0,cc=0,rc=0;for(std::size_t i=0;i<p.size();++i){const double dy=p[i].row-mr,dx=p[i].col-mc;const double weight=UnitWeights?1.0:w[i];rr+=weight*dy*dy;cc+=weight*dx*dx;rc+=weight*dy*dx;}const double theta=0.5*std::atan2(2*rc,cc-rr);double dc=std::cos(theta),dr=std::sin(theta);const double er=p.back().row-p.front().row,ec=p.back().col-p.front().col;if(er*dr+ec*dc<0){dr=-dr;dc=-dc;}const double nr=dc,nc=-dr;return {mr,mc,dr,dc,nr,nc,nr*mr+nc*mc};}
void line_residuals(std::span<const Point2d> p,const LineModel& m,PmrReals& r){r.resize(p.size());for(std::size_t i=0;i<p.size();++i)r[i]=m.nr*p[i].row+m.nc*p[i].col-m.dist;}
Point2d project_line(Point2d p,const LineModel&m){const double d=m.nr*p.row+m.nc*p.col-m.dist;return {p.row-d*m.nr,p.col-d*m.nc};}

bool solve3(double a[3][3],double b[3],double x[3]){for(int k=0;k<3;++k){int piv=k;for(int i=k+1;i<3;++i)if(std::abs(a[i][k])>std::abs(a[piv][k]))piv=i;if(std::abs(a[piv][k])<1e-15)return false;if(piv!=k){for(int j=k;j<3;++j)std::swap(a[piv][j],a[k][j]);std::swap(b[piv],b[k]);}const double d=a[k][k];for(int j=k;j<3;++j)a[k][j]/=d;b[k]/=d;for(int i=0;i<3;++i)if(i!=k){const double f=a[i][k];for(int j=k;j<3;++j)a[i][j]-=f*a[k][j];b[i]-=f*b[k];}}for(int i=0;i<3;++i)x[i]=b[i];return true;}
struct CircleModel{double row{},col{},radius{};};
template<bool UnitWeights=false>
CircleModel fit_circle_algebraic(std::span<const Point2d> p,std::span<const double> w){double a[3][3]{};double b[3]{};for(std::size_t i=0;i<p.size();++i){const double x=p[i].col,y=p[i].row,z[3]{x,y,1.0},rhs=-(x*x+y*y);for(int r=0;r<3;++r){b[r]+=(UnitWeights?1.0:w[i])*z[r]*rhs;for(int c=0;c<3;++c)a[r][c]+=(UnitWeights?1.0:w[i])*z[r]*z[c];}}double sol[3]{};if(!solve3(a,b,sol))throw Exception(Status::InvalidArgument,"fit_circle_contour_xld","degenerate contour");const double col=-0.5*sol[0],row=-0.5*sol[1],rsq=col*col+row*row-sol[2];if(rsq<=0)throw Exception(Status::InvalidArgument,"fit_circle_contour_xld","invalid fitted radius");return {row,col,std::sqrt(rsq)};}
void circle_residuals(std::span<const Point2d> p,const CircleModel& m,PmrReals& r){r.resize(p.size());for(std::size_t i=0;i<p.size();++i)r[i]=std::hypot(p[i].row-m.row,p[i].col-m.col)-m.radius;}
void circle_weights(std::span<const double> res,double sigma,double clip,bool tukey,bool robust,PmrReals& w){w.assign(res.size(),1);if(!robust)return;const double c=std::max(1e-12,sigma*clip);for(std::size_t i=0;i<res.size();++i){const double a=std::abs(res[i]);if(tukey){const double u=a/c;w[i]=u<1?std::pow(1-u*u,2):0;}else w[i]=a<=c?1:c/a;}}
void geometric_step(std::span<const Point2d> p,std::span<const double> w,CircleModel& m){double a[3][3]{};double b[3]{};for(std::size_t i=0;i<p.size();++i){const double dy=m.row-p[i].row,dx=m.col-p[i].col,d=std::max(1e-12,std::hypot(dy,dx)),res=d-m.radius;const double j[3]{dy/d,dx/d,-1};for(int r=0;r<3;++r){b[r]-=w[i]*j[r]*res;for(int c=0;c<3;++c)a[r][c]+=w[i]*j[r]*j[c];}}double delta[3]{};if(solve3(a,b,delta)){m.row+=delta[0];m.col+=delta[1];m.radius=std::max(1e-12,m.radius+delta[2]);}}
double phi_of(Point2d p,const CircleModel&m){double v=std::atan2(-(p.row-m.row),p.col-m.col);if(v<0)v+=2*std::numbers::pi_v<double>;return v;}
PointOrder order_of(std::span<const Point2d>p,const CircleModel&m){
  // 单调短角步序列有相同绕向，无需逐点 atan2；含糊步保留原计算。
  int sign=0;bool monotone=true;
  for(std::size_t i=1;i<p.size();++i) {
    const auto ax=p[i-1].col-m.col,ay=-(p[i-1].row-m.row);
    const auto bx=p[i].col-m.col,by=-(p[i].row-m.row);
    const auto cross=ax*by-ay*bx,dot=ax*bx+ay*by;
    const auto uncertainty=16*std::numeric_limits<double>::epsilon()*(std::abs(ax*by)+std::abs(ay*bx));
    if(dot<=0 || !std::isfinite(cross) || std::abs(cross)<=uncertainty) {monotone=false;break;}
    const int current=cross>0 ? 1 : -1;
    if(sign && current!=sign){monotone=false;break;}sign=current;
  }
  if(monotone && sign)return sign>0 ? PointOrder::Positive : PointOrder::Negative;
if(p.size()<2)return PointOrder::Positive;double total=0,prev=phi_of(p[0],m);for(std::size_t i=1;i<p.size();++i){double cur=phi_of(p[i],m),d=cur-prev;while(d>std::numbers::pi_v<double>)d-=2*std::numbers::pi_v<double>;while(d<-std::numbers::pi_v<double>)d+=2*std::numbers::pi_v<double>;total+=d;prev=cur;}return total>=0?PointOrder::Positive:PointOrder::Negative;}

LineFitResult fit_line_contour_xld(const XldContour& c, const LineFitOptions& o,
                                   std::pmr::memory_resource* memory) {
  auto p=c.points();
  for(const auto point:p)if(!std::isfinite(point.row)||!std::isfinite(point.col))throw Exception(Status::InvalidArgument,"fit_line_contour_xld","finite coordinates required");
  if(o.clipping_end_points<0 || std::size_t(o.clipping_end_points)>p.size()/2 || p.size()-2*std::size_t(o.clipping_end_points)<2)
    throw Exception(Status::InvalidArgument,"fit_line_contour_xld",o.clipping_end_points?"at least two unclipped points required":"at least two points required");
  p=p.subspan(o.clipping_end_points,p.size()-2*std::size_t(o.clipping_end_points));
  PmrPoints sampled{memory};
  if(o.max_num_points>=0 && static_cast<std::size_t>(o.max_num_points)<p.size()) {
    sampled=sample_points(p,o.max_num_points,memory);p=sampled;
  }
  if (p.size() < 2)
    throw Exception(Status::InvalidArgument, "fit_line_contour_xld", "at least two points required");
  if (!std::isfinite(o.clipping_factor) || o.iterations < 0 || (o.algorithm!=LineFitAlgorithm::Regression && o.iterations>0 && o.clipping_factor<=0))
    throw Exception(Status::InvalidArgument, "fit_line_contour_xld", "invalid robust options");
  LineModel m{};
  if (o.algorithm == LineFitAlgorithm::Regression || o.iterations==0) m=fit_line_weighted<true>(p,{});
  else {
    PmrReals w(p.size(), 1, memory), r{memory};
    for (int i = 0; i < o.iterations; ++i) {
      m = fit_line_weighted(p,w);
      line_residuals(p,m,r);
      line_weights(r,robust_sigma(r,memory),o.clipping_factor,o.algorithm,w);
    }
    m=fit_line_weighted(p,w);
  }
  return {project_line(c.points().front(), m), project_line(c.points().back(), m), m.nr, m.nc, m.dist};
}

CircleFitResult fit_circle_contour_xld(const XldContour& c, const CircleFitOptions& o,
                                       std::pmr::memory_resource* memory,bool diagnostics=false) {
  if(o.algorithm<CircleFitAlgorithm::Algebraic || o.algorithm>CircleFitAlgorithm::GeometricTukey ||
      !std::isfinite(o.clipping_factor))
    throw Exception(Status::InvalidArgument,"fit_circle_contour_xld","finite clipping and valid algorithm required");
  for(const auto point:c.points())if(!std::isfinite(point.row) || !std::isfinite(point.col))
    throw Exception(Status::InvalidArgument,"fit_circle_contour_xld","finite contour coordinates required");
  auto source=c.points();
  if(o.clipping_end_points<0 || std::size_t(o.clipping_end_points)>source.size()/2 || source.size()-2*std::size_t(o.clipping_end_points)<3)
    throw Exception(Status::InvalidArgument,"fit_circle_contour_xld",o.clipping_end_points?"at least three unclipped points required":"at least three points required");
  source=source.subspan(o.clipping_end_points,source.size()-2*std::size_t(o.clipping_end_points));
  auto p = sample_points(source, o.max_num_points, memory);
  if (p.size() < 3)
    throw Exception(Status::InvalidArgument, "fit_circle_contour_xld",
                    "at least three points required");
  if (o.clipping_factor <= 0 || o.iterations < 0)
    throw Exception(Status::InvalidArgument, "fit_circle_contour_xld", "invalid robust options");
  if (!std::isfinite(o.min_arc_angle) || o.min_arc_angle < 0 ||
      o.min_arc_angle > 2 * std::numbers::pi_v<double>)
    throw Exception(Status::InvalidArgument, "fit_circle_contour_xld", "invalid minimum arc angle");
  const Point2d anchor = p.front();
  double mean_row = 0, mean_col = 0;
#if MVKIT_FIT_SSE2
  const auto& cpu=mv::runtime::cpu_features();
  const bool simd=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{true,false,false,false});
#endif
#if MVKIT_FIT_SSE2
  if(simd) {
    // 独立 row/column lane 保留逐点除法及累加序。
    auto mean=_mm_setzero_pd();const auto a=_mm_loadu_pd(&anchor.row),count=_mm_set1_pd(static_cast<double>(p.size()));
    for(const auto& point:p)mean=_mm_add_pd(mean,_mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&point.row),a),count));
    double values[2];_mm_storeu_pd(values,mean);mean_row=values[0];mean_col=values[1];
  }else
#endif
  {  for (const auto& point : p) {
    mean_row += (point.row - anchor.row) / static_cast<double>(p.size());
    mean_col += (point.col - anchor.col) / static_cast<double>(p.size());
  }}
  const Point2d origin{anchor.row + mean_row, anchor.col + mean_col};
  double scale = 0;
  for (const auto& point : p)
    scale = std::max(scale, std::hypot(point.row - origin.row, point.col - origin.col));
  if (!std::isfinite(scale) || scale <= 0)
    throw Exception(Status::InvalidArgument, "fit_circle_contour_xld", "degenerate contour");
#if MVKIT_FIT_SSE2
  if(simd) {
    const auto o=_mm_loadu_pd(&origin.row),s=_mm_set1_pd(scale);
    for(auto& point:p)_mm_storeu_pd(&point.row,_mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&point.row),o),s));
  }else
#endif
  {  for (auto& point : p) {
    point.row = (point.row - origin.row) / scale;
    point.col = (point.col - origin.col) / scale;
  }}
  const bool geometric = o.algorithm == CircleFitAlgorithm::Geometric ||
                         o.algorithm == CircleFitAlgorithm::GeometricHuber ||
                         o.algorithm == CircleFitAlgorithm::GeometricTukey;
  const bool robust = o.algorithm == CircleFitAlgorithm::AlgebraicHuber ||
                      o.algorithm == CircleFitAlgorithm::AlgebraicTukey ||
                      o.algorithm == CircleFitAlgorithm::GeometricHuber ||
                      o.algorithm == CircleFitAlgorithm::GeometricTukey;
  const bool tukey = o.algorithm == CircleFitAlgorithm::AlgebraicTukey ||
                     o.algorithm == CircleFitAlgorithm::GeometricTukey;
  PmrReals w{memory}, res{memory};
  if(geometric || robust)w.assign(p.size(),1);
  CircleModel m = geometric || robust ? fit_circle_algebraic(p,w)
                                     : fit_circle_algebraic<true>(p,{});
  if (geometric || robust) for (int i = 0; i < o.iterations; ++i) {
    circle_residuals(p, m, res);
    circle_weights(res, robust_sigma(res, memory), o.clipping_factor, tukey, robust, w);
    if (geometric)
      geometric_step(p, w, m);
    else if (robust)
      m = fit_circle_algebraic(p, w);
    else
      break;
  }
  m.row = origin.row + scale * m.row;
  m.col = origin.col + scale * m.col;
  m.radius *= scale;
  if (o.min_arc_angle > 0) {
    const auto points = c.points();
    double angle = 0, minimum = 0, maximum = 0;
    double previous = phi_of(points.front(), m);
    for (std::size_t i = 1; i < points.size(); ++i) {
      const double current = phi_of(points[i], m);
      angle += std::remainder(current - previous, 2 * std::numbers::pi_v<double>);
      minimum = std::min(minimum, angle);
      maximum = std::max(maximum, angle);
      previous = current;
    }
    if (maximum - minimum < o.min_arc_angle)
      throw Exception(Status::InvalidArgument, "fit_circle_contour_xld",
                      "arc too short for configured minimum coverage");
  }
  CircleFitResult result{m.row,m.col,m.radius,phi_of(c.points().front(),m),phi_of(c.points().back(),m),order_of(c.points(),m)};
  if(o.halcon_controls && std::hypot(c.points().front().row-c.points().back().row,c.points().front().col-c.points().back().col)<=o.max_closure_dist){
    result.start_phi=0;result.end_phi=2*std::numbers::pi_v<double>;result.point_order=PointOrder::Positive;
  }
  if(diagnostics){
    double sum=0,angle=0,minimum=0,maximum=0,previous=phi_of(c.points().front(),m),gram[3][3]{};
    for(const auto point:c.points()){
      const auto dy=point.row-m.row,dx=point.col-m.col,d=std::hypot(dy,dx);
      const auto residual=d-m.radius;sum+=residual*residual;
      const auto phi=phi_of(point,m);angle+=std::remainder(phi-previous,2*std::numbers::pi_v<double>);
      minimum=std::min(minimum,angle);maximum=std::max(maximum,angle);previous=phi;
      const double j[]{d>0?dy/d:0,d>0?dx/d:0,-1};
      for(int r=0;r<3;++r)for(int col=0;col<3;++col)gram[r][col]+=j[r]*j[col];
    }
    // 无权径向 Jacobian Gram 特征值比，为无量纲几何信息，不是虚构协方差。
    for(int iteration=0;iteration<32;++iteration){
      int a=0,b=1;for(int r=0;r<3;++r)for(int col=r+1;col<3;++col)
        if(std::abs(gram[r][col])>std::abs(gram[a][b])){a=r;b=col;}
      if(std::abs(gram[a][b])<=1e-15*c.size())break;
      const auto theta=.5*std::atan2(2*gram[a][b],gram[b][b]-gram[a][a]);
      const auto cs=std::cos(theta),sn=std::sin(theta),aa=gram[a][a],bb=gram[b][b],ab=gram[a][b];
      gram[a][a]=cs*cs*aa-2*sn*cs*ab+sn*sn*bb;
      gram[b][b]=sn*sn*aa+2*sn*cs*ab+cs*cs*bb;gram[a][b]=gram[b][a]=0;
      for(int r=0;r<3;++r)if(r!=a && r!=b){const auto x=gram[r][a],y=gram[r][b];
        gram[r][a]=gram[a][r]=cs*x-sn*y;gram[r][b]=gram[b][r]=sn*x+cs*y;}
    }
    result.rms=std::sqrt(sum/c.size());result.arc_angle=std::min(2*std::numbers::pi_v<double>,maximum-minimum);
    const auto lo=std::min({gram[0][0],gram[1][1],gram[2][2]}),hi=std::max({gram[0][0],gram[1][1],gram[2][2]});
    result.information_ratio=hi>0?std::clamp(lo/hi,0.,1.):0;
  }
  return result;
}

template <class Result, class Fit>
std::vector<Result> fit_contour_batch(const mv::core::XldContours& cs, Fit&& fit) {
  const std::size_t count = cs.contour_count();
  std::vector<Result> results(count);
  std::vector<std::exception_ptr> errors(count);
  std::size_t work = 0;
  for (std::size_t i = 0; i < count; ++i) {
    work += (std::min)(cs.contour(i).size(), (std::numeric_limits<std::size_t>::max)() - work);
  }
  ExecutionContext::for_each_partition(count, work, 32768, [&](auto range) {
    mv::runtime::ScratchArena local;
    auto* memory = local.resource();
    for (auto i = range.begin; i < range.end; ++i) {
      try {
        results[i] = fit(cs.contour(i), memory);
      } catch (...) {
        errors[i] = std::current_exception();
      }
      local.reset();
    }
  });
  for (std::size_t i = 0; i < count; ++i) {
    if (errors[i]) std::rethrow_exception(errors[i]);
  }
  return results;
}
}  // namespace

LineFitResult fit_line_contour_xld(const XldContour& c, const LineFitOptions& o) {
  mv::runtime::ScratchArena context;
  return fit_line_contour_xld(c, o, context.resource());
}
std::vector<LineFitResult> fit_line_contour_xld(const mv::core::XldContours& cs,
                                                const LineFitOptions& o) {
  return fit_contour_batch<LineFitResult>(
      cs, [&](const XldContour& c, std::pmr::memory_resource* memory) {
        return fit_line_contour_xld(c, o, memory);
      });
}
CircleFitResult fit_circle_contour_xld(const XldContour& c, const CircleFitOptions& o) {
  mv::runtime::ScratchArena context;
  return fit_circle_contour_xld(c, o, context.resource());
}
std::vector<CircleFitResult> fit_circle_contour_xld(const mv::core::XldContours& cs,
                                                    const CircleFitOptions& o) {
  return fit_contour_batch<CircleFitResult>(
      cs, [&](const XldContour& c, std::pmr::memory_resource* memory) {
        return fit_circle_contour_xld(c, o, memory);
      });
}
std::vector<CircleFitResult> fit_circle_contour_xld_diagnostics(const mv::core::XldContours& cs,const CircleFitOptions& o){
  return fit_contour_batch<CircleFitResult>(cs,[&](const XldContour& c,std::pmr::memory_resource* memory){
    return fit_circle_contour_xld(c,o,memory,true);
  });
}
}  // namespace mv::xld
