#include <xld/contour_utils.hpp>
#include <limits>
#include <runtime/cpu/cpu_features.hpp>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_ELLIPSE_MOMENT_SSE2 1
#else
#define MVKIT_ELLIPSE_MOMENT_SSE2 0
#endif
namespace mv::xld {
namespace {
using namespace detail;
using Model = std::array<double, 5>;  // 参数为 x/y/phi/log(radius1)/log(radius2)，使用归一化笛卡尔坐标。
Model initial(std::span<const Point2d> points,std::span<const double> weights={}) {
  // 直接约束最小二乘（Fitzgibbon/Halir）：消去线性项，再解 3x3 椭圆约束特征问题。
  std::array<std::array<double, 3>, 3> s1{}, s2{}, s3{};
#if MVKIT_ELLIPSE_MOMENT_SSE2
  const auto& cpu=mv::runtime::cpu_features();
  if(weights.empty() && mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{true,false,false,false})) {
    // 独立矩分量在各 lane 保留原点序。
    __m128d a[3]{_mm_setzero_pd(),_mm_setzero_pd(),_mm_setzero_pd()},
            b[3]{_mm_setzero_pd(),_mm_setzero_pd(),_mm_setzero_pd()},
            c[3]{_mm_setzero_pd(),_mm_setzero_pd(),_mm_setzero_pd()};
    for(auto p:points) {
      const double x=p.col,y=-p.row;
      const std::array<double,3> quad{x*x,x*y,y*y},linear{x,y,1};
      const auto qp=_mm_loadu_pd(quad.data()),lp=_mm_loadu_pd(linear.data());
      for(int i=0;i<3;++i) {
        const auto qi=_mm_set1_pd(quad[i]),li=_mm_set1_pd(linear[i]);
        a[i]=_mm_add_pd(a[i],_mm_mul_pd(qi,qp));
        b[i]=_mm_add_pd(b[i],_mm_mul_pd(qi,lp));
        c[i]=_mm_add_pd(c[i],_mm_mul_pd(li,lp));
        s1[i][2]+=quad[i]*quad[2];s2[i][2]+=quad[i]*linear[2];s3[i][2]+=linear[i]*linear[2];
      }
    }
    for(int i=0;i<3;++i){_mm_storeu_pd(s1[i].data(),a[i]);_mm_storeu_pd(s2[i].data(),b[i]);_mm_storeu_pd(s3[i].data(),c[i]);}
  } else
#endif
  {
  for (std::size_t i=0;i<points.size();++i) {
    const auto p=points[i];const double weight=weights.empty()?1.:weights[i];
    double x = p.col, y = -p.row;
    const std::array<double, 3> quad{x * x, x * y, y * y}, linear{x, y, 1};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        s1[i][j] += weight * quad[i] * quad[j];
        s2[i][j] += weight * quad[i] * linear[j];
        s3[i][j] += weight * linear[i] * linear[j];
      }
    }
  }
  }
  std::array<std::array<double, 3>, 3> t{};
  for (int col = 0; col < 3; ++col) {
    std::array<double, 3> rhs{}, solved{};
    for (int row = 0; row < 3; ++row) rhs[row] = -s2[col][row];
    require(solve(s3, rhs, solved), "fit_ellipse_contour_xld", "Contour",
            "rank deficient ellipse fit");
    for (int row = 0; row < 3; ++row) t[row][col] = solved[row];
  }
  std::array<std::array<double, 3>, 3> reduced{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      reduced[i][j] = s1[i][j];
      for (int k = 0; k < 3; ++k) reduced[i][j] += s2[i][k] * t[k][j];
    }
  const auto m = std::array<std::array<double, 3>, 3>{
      std::array<double, 3>{reduced[2][0] / 2, reduced[2][1] / 2, reduced[2][2] / 2},
      std::array<double, 3>{-reduced[1][0], -reduced[1][1], -reduced[1][2]},
      std::array<double, 3>{reduced[0][0] / 2, reduced[0][1] / 2, reduced[0][2] / 2}};
  const double trace = m[0][0] + m[1][1] + m[2][2];
  const double minors = m[0][0] * m[1][1] + m[0][0] * m[2][2] + m[1][1] * m[2][2] -
      m[0][1] * m[1][0] - m[0][2] * m[2][0] - m[1][2] * m[2][1];
  const double determinant =
      m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  const double cubic_a = -trace, cubic_b = minors, cubic_c = -determinant;
  const double p = cubic_b - cubic_a * cubic_a / 3;
  const double q = 2 * cubic_a * cubic_a * cubic_a / 27 - cubic_a * cubic_b / 3 + cubic_c;
  const double discriminant = q * q / 4 + p * p * p / 27;
  std::array<double, 3> roots{};
  int root_count = 1;
  if (discriminant >= 0) {
    const double s = std::sqrt(discriminant);
    roots[0] = std::cbrt(-q / 2 + s) + std::cbrt(-q / 2 - s) - cubic_a / 3;
  } else {
    const double amplitude = 2 * std::sqrt(-p / 3);
    const double phase = std::acos(std::clamp(-q / 2 / std::sqrt(-p * p * p / 27), -1., 1.)) / 3;
    for (int i = 0; i < 3; ++i)
      roots[i] = amplitude * std::cos(phase - 2 * pi * i / 3) - cubic_a / 3;
    root_count = 3;
  }
  for (int root = 0; root < root_count; ++root) {
    auto r = m;
    for (int i = 0; i < 3; ++i) r[i][i] -= roots[root];
    std::array<double, 3> eigen{};
    double best_norm = 0;
    for (int i = 0; i < 3; ++i)
      for (int j = i + 1; j < 3; ++j) {
        std::array<double, 3> candidate{
            r[i][1] * r[j][2] - r[i][2] * r[j][1],
            r[i][2] * r[j][0] - r[i][0] * r[j][2],
            r[i][0] * r[j][1] - r[i][1] * r[j][0]};
        const double norm = candidate[0] * candidate[0] + candidate[1] * candidate[1] +
                            candidate[2] * candidate[2];
        if (norm > best_norm) { eigen = candidate; best_norm = norm; }
      }
    if (!(best_norm > 1e-30)) continue;
    const double a = eigen[0], b = eigen[1], c = eigen[2];
    if (!(4 * a * c - b * b > 0)) continue;
    const double d = t[0][0] * a + t[0][1] * b + t[0][2] * c;
    const double e = t[1][0] * a + t[1][1] * b + t[1][2] * c;
    const double f = t[2][0] * a + t[2][1] * b + t[2][2] * c;
    const double det = a * c - b * b / 4;
    const double x = (b * e / 2 - c * d) / (2 * det);
    const double y = (b * d / 2 - a * e) / (2 * det);
    const double theta = .5 * std::atan2(b, a - c), ct = std::cos(theta), st = std::sin(theta);
    const double aa = a * ct * ct + b * ct * st + c * st * st;
    const double bb = a * st * st - b * ct * st + c * ct * ct;
    const double height = -(a * x * x + b * x * y + c * y * y + d * x + e * y + f);
    if (!(height / aa > 0 && height / bb > 0)) continue;
    double major = std::sqrt(height / aa), minor = std::sqrt(height / bb);
    double phi = theta;
    if (major < minor) { std::swap(major, minor); phi += pi / 2; }
    if (std::isfinite(major) && std::isfinite(minor))
      return {x, y, phi, std::log(major), std::log(minor)};
  }
  throw Exception(Status::InvalidArgument, "fit_ellipse_contour_xld", "algebraic fit is not an ellipse");
}
double residual(Point2d p, const Model& m) {
  double c = std::cos(m[2]), s = std::sin(m[2]), dx = p.col - m[0],
         dy = -p.row - m[1];
  double x = std::abs(c * dx + s * dy), y = std::abs(-s * dx + c * dy),
         a = std::exp(m[3]), b = std::exp(m[4]);
  auto squared = [&](double t) {
    double u = a * std::cos(t) - x, v = b * std::sin(t) - y;
    return u * u + v * v;
  };
  double lo = 0, hi = pi / 2;
  constexpr double golden = .6180339887498948482;
  double t1 = hi - golden * (hi - lo), t2 = lo + golden * (hi - lo),
         f1 = squared(t1), f2 = squared(t2);
  for (int i = 0; i < 48; ++i) {
    if (f1 < f2) {
      hi = t2;
      t2 = t1;
      f2 = f1;
      t1 = hi - golden * (hi - lo);
      f1 = squared(t1);
    } else {
      lo = t1;
      t1 = t2;
      f1 = f2;
      t2 = lo + golden * (hi - lo);
      f2 = squared(t2);
    }
  }
  double d = std::sqrt(std::min({f1, f2, squared(0), squared(pi / 2)}));
  return x * x / (a * a) + y * y / (b * b) >= 1 ? d : -d;
}
double phase(Point2d p, const Model& m) {
  double dx = p.col - m[0], dy = -p.row - m[1], c = std::cos(m[2]),
         s = std::sin(m[2]);
  return angle(std::atan2((-s * dx + c * dy) / std::exp(m[4]),
                          (c * dx + s * dy) / std::exp(m[3])));
}
double closest_phase(Point2d p,const Model& m){
 const double dx=p.col-m[0],dy=-p.row-m[1],c=std::cos(m[2]),s=std::sin(m[2]);
 const double x=c*dx+s*dy,y=-s*dx+c*dy,a=std::exp(m[3]),b=std::exp(m[4]);
 const auto cost=[&](double t){const double u=a*std::cos(t)-std::abs(x),v=b*std::sin(t)-std::abs(y);return u*u+v*v;};
 double lo=0,hi=pi/2;constexpr double golden=.6180339887498948482;double t1=hi-golden*(hi-lo),t2=lo+golden*(hi-lo),f1=cost(t1),f2=cost(t2);
 for(int i=0;i<48;++i){if(f1<f2){hi=t2;t2=t1;f2=f1;t1=hi-golden*(hi-lo);f1=cost(t1);}else{lo=t1;t1=t2;f1=f2;t2=lo+golden*(hi-lo);f2=cost(t2);}}
 double t=f1<f2?t1:t2,best=std::min(f1,f2);if(cost(0)<best){t=0;best=cost(0);}if(cost(pi/2)<best)t=pi/2;
 return angle(std::atan2(std::copysign(std::sin(t),y),std::copysign(std::cos(t),x)));
}
EllipseFitResult fit(const XldContour& contour,
                     const EllipseFitOptions& options) {
  constexpr auto op = "fit_ellipse_contour_xld";
  validate(contour, op);
  const auto original=contour.points();auto source=original;
  if(options.clipping_end_points<0 || std::size_t(options.clipping_end_points)>source.size()/2 || source.size()-2*std::size_t(options.clipping_end_points)<5)
    throw Exception(Status::InvalidArgument,op,"at least five unclipped points required");
  source=source.subspan(options.clipping_end_points,source.size()-2*std::size_t(options.clipping_end_points));
  if (source.size() > 1 && source.front() == source.back())
    source = source.first(source.size() - 1);
  cardinality(source.size() >= 5, op, "Contour");
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  Point2d origin = source.front();
  double mr = 0, mc = 0;
  for (auto p : source) {
    mr += p.row - origin.row;
    mc += p.col - origin.col;
  }
  origin.row += mr / source.size();
  origin.col += mc / source.size();
  double squared_extent=0;Point2d farthest=origin;
  for(auto p:source) {
    const auto dx=p.col-origin.col,dy=p.row-origin.row,squared=dx*dx+dy*dy;
    if(squared>squared_extent){squared_extent=squared;farthest=p;}
  }
  double scale=point_distance(farthest,origin);
  // 平方跨度下溢的 subnormal 轮廓保留尺度。
  if(squared_extent<std::numeric_limits<double>::min())
    for(auto p:source)scale=std::max(scale,point_distance(p,origin));
  require(scale > 0, op, "Contour", "zero extent contour");
  const auto count =
      options.max_num_points < 0
          ? source.size()
          : std::min(source.size(), std::size_t(options.max_num_points));
  std::pmr::vector<Point2d> points{memory};
  points.reserve(count);
  if(count==source.size()) {
    for(auto p:source)points.push_back({(p.row-origin.row)/scale,(p.col-origin.col)/scale});
  } else for(std::size_t i=0;i<count;++i) {
    const auto p=source[i*(source.size()-1)/(count-1)];
    points.push_back({(p.row-origin.row)/scale,(p.col-origin.col)/scale});
  }
  auto m = initial(points);
  if (options.algorithm != EllipseFitAlgorithm::Algebraic) {
    std::pmr::vector<double> res(count, memory),weights(count,1.,memory),sorted(count,memory);
    for (int iteration = 0; iteration < options.iterations; ++iteration) {
      for (std::size_t i = 0; i < count; ++i) {
        res[i] = residual(points[i], m);
        sorted[i] = std::abs(res[i]);
      }
      std::nth_element(sorted.begin(), sorted.begin() + count / 2,
                       sorted.end());
      double cutoff =
          options.clipping_factor * std::max(1e-9, 1.4826 * sorted[count / 2]);
      std::array<std::array<double, 5>, 5> matrix{};
      std::array<double, 5> rhs{}, delta{};
      double old_cost = 0;
      for (std::size_t i = 0; i < count; ++i) {
        double u = std::abs(res[i]) / cutoff;
        weights[i] = (options.algorithm == EllipseFitAlgorithm::Huber || options.algorithm == EllipseFitAlgorithm::AlgebraicHuber)
                         ? (u > 1 ? 1 / u : 1)
                     : (options.algorithm == EllipseFitAlgorithm::Tukey || options.algorithm == EllipseFitAlgorithm::AlgebraicTukey)
                         ? (u < 1 ? std::pow(1 - u * u, 2) : 0)
                         : 1.;
        old_cost += weights[i] * res[i] * res[i];
        std::array<double, 5> jac{};
        for (int k = 0; k < 5; ++k) {
          auto candidate = m;
          double step = 1e-5;
          candidate[k] += step;
          double upper = residual(points[i], candidate);
          candidate[k] -= 2 * step;
          jac[k] = (upper - residual(points[i], candidate)) / (2 * step);
        }
        for (int r = 0; r < 5; ++r) {
          rhs[r] -= weights[i] * jac[r] * res[i];
          for (int c = 0; c < 5; ++c)
            matrix[r][c] += weights[i] * jac[r] * jac[c];
        }
      }
      if(options.algorithm==EllipseFitAlgorithm::AlgebraicHuber || options.algorithm==EllipseFitAlgorithm::AlgebraicTukey){m=initial(points,weights);continue;}
      for (int k = 0; k < 5; ++k) matrix[k][k] += 1e-7 * count;
      if (!solve(matrix, rhs, delta)) break;
      bool accepted = false;
      for (double alpha = 1; alpha >= 1. / 128; alpha *= .5) {
        auto candidate = m;
        for (int k = 0; k < 5; ++k) candidate[k] += alpha * delta[k];
        if (std::abs(candidate[3]) > 25 || std::abs(candidate[4]) > 25)
          continue;
        double cost = 0;
        for (std::size_t i = 0; i < count; ++i) {
          double d = residual(points[i], candidate);
          cost += weights[i] * d * d;
        }
        if (cost < old_cost) {
          m = candidate;
          accepted = true;
          break;
        }
      }
      if (!accepted) break;
    }
  }
  if (m[3] < m[4]) {
    std::swap(m[3], m[4]);
    m[2] += pi / 2;
  }
  m[2] = std::fmod(m[2], pi);
  if (m[2] < 0) m[2] += pi;
  if(options.halcon_controls && m[2]>pi/2)m[2]-=pi;
  const auto normalized=[&](Point2d p){return Point2d{(p.row-origin.row)/scale,(p.col-origin.col)/scale};};
  const bool closed=options.halcon_controls ? point_distance(original.front(),original.back())<=options.max_closure_dist : contour.closed();
  const auto endpoint_phase=[&](Point2d p){return options.halcon_controls?closest_phase(normalized(p),m):phase(normalized(p),m);};
  double start = endpoint_phase(original.front()), previous = phase(normalized(original.front()),m), total = 0,
         min_phase = 0, max_phase = 0;
  if (!closed) for (std::size_t i = 1; i < original.size(); ++i) {
    double current = phase(normalized(original[i]), m);
    total += std::remainder(current - previous, tau);
    previous = current;
    min_phase = std::min(min_phase, total);
    max_phase = std::max(max_phase, total);
  }
  require(closed || max_phase - min_phase >= options.min_arc_angle,
          op, "Contour", "arc too short for configured minimum coverage");
  EllipseFitResult r{origin.row - scale * m[1],
                     origin.col + scale * m[0],
                     m[2],
                     scale * std::exp(m[3]),
                     scale * std::exp(m[4]),
                     start,
                     endpoint_phase(original.back()),
                     total >= 0 ? PointOrder::Positive : PointOrder::Negative};
  if (closed) {
    r.start_phi = 0;
    r.end_phi = tau;
    r.point_order = PointOrder::Positive;
  }
  for (auto v : {r.row, r.column, r.radius1, r.radius2}) coordinate(v, op);
  require(r.radius2 > 0, op, "Contour", "invalid refined ellipse");
  return r;
}
}  // namespace
mv::core::XldContours gen_ellipse_contour_xld(double row, double column,
                                              double phi, double a, double b,
                                              double start, double end,
                                              PointOrder order,
                                              double resolution) {
  using namespace detail;
  constexpr auto op = "gen_ellipse_contour_xld";
  for (auto v : {row, column, a, b}) coordinate(v, op);
  require(a >= b && b > 0, op, "Radius1/Radius2",
          "positive ordered semiaxes required");
  require(std::isfinite(phi) && std::isfinite(start) && std::isfinite(end) &&
              std::isfinite(resolution) && resolution > 0,
          op, "Angles/Resolution",
          "finite angles and positive resolution required");
  require(order == PointOrder::Positive || order == PointOrder::Negative, op,
          "PointOrder", "invalid order");
  double raw = end - start;
  require(std::isfinite(raw), op, "StartPhi/EndPhi",
          "angle difference overflow");
  double delta = std::fmod(raw, tau);
  if(order == PointOrder::Positive) {
    if(delta < 0)delta += tau;
    if(delta == 0 && raw > 0)delta = tau;
  } else {
    if(delta > 0)delta -= tau;
    if(delta == 0 && raw < 0)delta = -tau;
  }
  const bool closed = std::abs(delta) == tau;
  const double requested = std::ceil(std::abs(delta) * a / resolution);
  require(std::isfinite(requested) && requested <= 10000000, op, "Resolution",
          "sample budget exceeds ten million points");
  ExecutionContext context;
  std::pmr::vector<Point2d> points{context.scratch_resource()};
  const double c = std::cos(phi), s = std::sin(phi), begin = angle(start);
  const auto local = [&](double t){return Point2d{b*std::sin(t),a*std::cos(t)};};
  const auto append = [&](Point2d p){
    Point2d v{row-s*p.col-c*p.row,column+c*p.col-s*p.row};
    coordinate(v.row,op);coordinate(v.col,op);
    require(points.size()<10000001,op,"Resolution","sample budget exceeds ten million points");
    points.push_back(v);
  };
  append(local(begin));
  // 象限边界处二分，长轴极点附近保持较粗角步；椭圆分辨率由欧氏间距控制，不按 a/radius 角度。
  const auto split = [&](auto&& self,double t0,double t1,Point2d p0,Point2d p1,unsigned depth)->void {
    if(std::hypot(p1.row-p0.row,p1.col-p0.col)<=resolution || depth==64) {
      append(p1);return;
    }
    const double mid=t0+(t1-t0)*.5;
    require(mid!=t0 && mid!=t1,op,"Resolution","resolution exceeds angular precision");
    const auto pm=local(mid);
    self(self,t0,mid,p0,pm,depth+1);
    self(self,mid,t1,pm,p1,depth+1);
  };
  const auto quadrants=delta==0 ? 0U : std::max(2U,static_cast<unsigned>(std::ceil(std::abs(delta)/(tau*.25))));
  double previous=begin;auto p0=local(begin);
  for(unsigned i=1;i<=quadrants;++i) {
    const double next=begin+delta*double(i)/quadrants;const auto p1=local(next);
    split(split,previous,next,p0,p1,0);previous=next;p0=p1;
  }
  if(closed)points.back()=points.front();
  auto contour = XldContour::from_points_and_attributes(
      points, closed, ContourApproximation::Ellipse);
  return objects(std::span<const XldContour>{&contour, 1});
}
std::vector<EllipseFitResult> fit_ellipse_contour_xld(
    const mv::core::XldContours& contours, const EllipseFitOptions& o) {
  using namespace detail;
  constexpr auto op = "fit_ellipse_contour_xld";
  require(o.algorithm >= EllipseFitAlgorithm::Algebraic &&
              o.algorithm <= EllipseFitAlgorithm::AlgebraicTukey,
          op, "Algorithm", "invalid fit algorithm");
  require(o.max_num_points == -1 || o.max_num_points >= 5, op, "MaxNumPoints",
          "expected -1 or >=5");
  require(std::isfinite(o.clipping_factor) && o.clipping_factor > 0 &&
              o.iterations >= 0,
          op, "Options", "positive clipping and nonnegative iterations required");
  require(std::isfinite(o.min_arc_angle) && o.min_arc_angle >= 0 &&
              o.min_arc_angle <= tau,
          op, "MinArcAngle", "expected [0,2pi]");
  ExecutionContext context;
  std::pmr::vector<EllipseFitResult> values(contours.contour_count(),
                                            context.scratch_resource());
  ExecutionContext::for_each_partition(
      values.size(), work(contours), 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          values[i] = fit(contours.contour(i), o);
      });
  return {values.begin(), values.end()};
}
}  // namespace mv::xld
