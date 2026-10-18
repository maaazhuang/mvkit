#include <calibration/estimation.hpp>
#include <calibration/calibration/homography.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <core/exception.hpp>
#include <core/broadcast.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace mv::calibration {
namespace {
constexpr auto op = "geometry_estimation";
void require(bool value, const char* parameter, const char* message,
             core::Status status = core::Status::InvalidArgument) {
  if (!value) throw core::Exception(status, core::ErrorCode::InvalidValue, op, parameter, -1, message);
}
double number(const core::Tuple& tuple, std::size_t i, const char* name) {
  double value;
  try { value = tuple.as_real64(tuple.size() == 1 ? 0 : i); }
  catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, name, i, "numeric values required");
  }
  require(std::isfinite(value), name, "finite values required"); return value;
}
void validate(const GeometryEstimationOptions& o, std::size_t count) {
  require(count <= 65536, "correspondences", "at most 65536 correspondences");
  require(o.max_trials >= 1 && o.max_trials <= 100000 && o.max_iterations <= 100,
          "options", "trial budget 1..100000 and refinement budget 0..100 required");
  require(std::isfinite(o.threshold) && o.threshold > 0 && o.threshold <= 1e12,
          "threshold", "threshold must be finite and positive, at most 1e12");
  require(std::isfinite(o.confidence) && o.confidence > 0 && o.confidence < 1,
          "confidence", "confidence must be in (0,1)");
  require(std::isfinite(o.min_inlier_ratio) && o.min_inlier_ratio > 0 && o.min_inlier_ratio <= 1,
          "min_inlier_ratio", "inlier ratio must be in (0,1]");
  require(!o.robust || count <= 50000000 / o.max_trials, "options", "RANSAC scoring budget exceeded");
}
struct Inputs {
  std::vector<std::vector<double>> coordinates;
  std::vector<double> weights;
  std::vector<std::size_t> active;
  Inputs(std::initializer_list<const core::Tuple*> values, const core::Tuple& w,
         const GeometryEstimationOptions& options) {
    std::size_t count = 1;
    for (auto* value : values) {
      require(!value->empty(), "coordinates", "coordinate arrays must not be empty");
      count = std::max(count, value->size());
    }
    validate(options, count);
    for (auto* value : values) require(value->size() == 1 || value->size() == count,
                                     "coordinates", "scalar-or-N lengths required");
    require(w.empty() || w.size() == 1 || w.size() == count, "weights", "weights must be empty, scalar or N");
    coordinates.resize(values.size());
    std::size_t k = 0;
    for (auto* value : values) {
      auto& c = coordinates[k++]; c.resize(count);
      if (value->kind() == core::TupleKind::Real64 && value->size() == count) {
        const auto data = value->real64_span();
        for (std::size_t i = 0; i < count; ++i) {
          require(std::isfinite(data[i]), "coordinates", "finite values required"); c[i] = data[i];
        }
      } else for (std::size_t i = 0; i < count; ++i) c[i] = number(*value, i, "coordinates");
    }
    weights.resize(count); double maximum = 0;
    for (std::size_t i = 0; i < count; ++i) {
      weights[i] = w.empty() ? 1 : number(w, i, "weights");
      require(weights[i] >= 0, "weights", "weights must be nonnegative");
      maximum = std::max(maximum, weights[i]);
    }
    require(maximum > 0, "weights", "at least one positive weight required");
    for (std::size_t i = 0; i < count; ++i) if (weights[i] > 0) {
      weights[i] /= maximum;
      require(weights[i] >= 1e-12, "weights", "positive weight dynamic range must be at most 1e12");
      active.push_back(i);
    }
  }
};
template<class Model, class Fit, class Error>
Model consensus(const Inputs& input, std::size_t minimum, const GeometryEstimationOptions& o,
                Fit fit, Error error, std::vector<std::size_t>* inliers, std::size_t* trials) {
  require(input.active.size() >= minimum, "weights", "insufficient positively weighted correspondences");
  *trials = 0;
  if (!o.robust) { *inliers = input.active; return fit(*inliers); }
  std::mt19937_64 rng(o.seed);
  // 模偏差不超过 N/2^64，跨标准库保持确定性。
  std::vector<std::size_t> sample(minimum), accepted;
  Model best{}; std::size_t best_count = 0, limit = o.max_trials;
  double best_cost = std::numeric_limits<double>::infinity();
  const double threshold2 = o.threshold * o.threshold;
  for (std::size_t t = 0; t < limit; ++t) {
    ++*trials;
    for (std::size_t j = 0; j < minimum; ++j) {
      do { sample[j] = input.active[rng() % input.active.size()]; }
      while (std::find(sample.begin(), sample.begin()+j, sample[j]) != sample.begin()+j);
    }
    Model candidate;
    try { candidate = fit(sample); }
    catch (const core::Exception& ex) {
      if (ex.status() == core::Status::InvalidState) continue;
      throw;
    }
    accepted.clear(); double cost = 0;
    for (const auto i : input.active) {
      const double e = error(candidate, i);
      if (e <= threshold2) { accepted.push_back(i); cost += input.weights[i] * e; }
    }
    if (accepted.size() > best_count || (accepted.size() == best_count && cost < best_cost)) {
      best = candidate; best_count = accepted.size(); best_cost = cost; *inliers = accepted;
      const double probability = std::pow(double(best_count)/input.active.size(), double(minimum));
      if (probability >= 1) limit = std::min(limit, std::max<std::size_t>(16, t+1));
      else if (probability > 0) {
        const double needed = std::ceil(std::log1p(-o.confidence)/std::log1p(-probability));
        if (std::isfinite(needed) && needed < double(limit))
          limit = std::min(limit, std::max<std::size_t>({16, t+1, static_cast<std::size_t>(needed)}));
      }
    }
  }
  const auto required = std::max(minimum, static_cast<std::size_t>(std::ceil(o.min_inlier_ratio * input.active.size())));
  require(best_count >= required, "consensus", "insufficient nondegenerate consensus", core::Status::InvalidState);
  // 使用全部共识点及其原相对权重重新拟合。
  best = fit(*inliers);
  accepted.clear();
  for (auto i : input.active) if (error(best, i) <= threshold2) accepted.push_back(i);
  require(accepted.size() >= required, "consensus", "consensus collapsed on refit", core::Status::InvalidState);
  *inliers = accepted;
  return fit(*inliers);
}
template<class Model, class Error>
void quality(GeometryEstimationResult* out, const Model& model, const Inputs& input,
             const GeometryEstimationOptions& o, Error error) {
  double sum = 0, weighted = 0, sum_weights = 0, all = 0;
  std::vector<double> residuals(input.weights.size());
  std::vector<std::int64_t> inliers(input.weights.size());
  for (std::size_t i = 0; i < input.weights.size(); ++i) {
    const double e = error(model, i);
    residuals[i] = std::sqrt(e);
    const bool inlier = input.weights[i] > 0 && std::isfinite(e) && (!o.robust || e <= o.threshold*o.threshold);
    inliers[i] = std::int64_t(inlier);
    if (inlier) { ++out->inlier_count; sum += e; weighted += e*input.weights[i]; sum_weights += input.weights[i]; }
    all += e;
  }
  out->samples = input.weights.size();
  require(out->inlier_count > 0, "consensus", "no valid inliers", core::Status::InvalidState);
  out->rms = std::sqrt(sum/out->inlier_count);
  out->weighted_rms = std::sqrt(weighted/sum_weights);
  out->all_rms = std::sqrt(all/out->samples);
  out->residuals = core::Tuple(std::move(residuals));
  out->inliers = core::Tuple(std::move(inliers));
}
struct Norm {
  double x{}, y{}, scale{1};
};
Norm normalize(std::vector<double>* x, std::vector<double>* y) {
  Norm n; const double ox=x->front(), oy=y->front();
  for (std::size_t i=0;i<x->size();++i) { n.x += (*x)[i]-ox; n.y += (*y)[i]-oy; }
  n.x = ox+n.x/x->size(); n.y = oy+n.y/y->size();
  double xx=0,xy=0,yy=0;
  for (std::size_t i=0;i<x->size();++i) { const double dx=(*x)[i]-n.x,dy=(*y)[i]-n.y;xx+=dx*dx;xy+=dx*dy;yy+=dy*dy; }
  const double radius=xx+yy;
  require(std::isfinite(radius) && radius>1e-24, "coordinates", "degenerate spread", core::Status::InvalidState);
  require((xx*yy-xy*xy)/(radius*radius)>1e-12,"coordinates","nearly collinear homography geometry",core::Status::InvalidState);
  n.scale=std::sqrt(2*x->size()/radius);
  for (std::size_t i=0;i<x->size();++i) { (*x)[i]=((*x)[i]-n.x)*n.scale; (*y)[i]=((*y)[i]-n.y)*n.scale; }
  return n;
}
double hom_error(const std::array<double,9>& h, double x, double y, double u, double v) {
  const double d=h[6]*x+h[7]*y+h[8];
  if (!std::isfinite(d) || std::abs(d)<1e-15) return std::numeric_limits<double>::infinity();
  const double du=(h[0]*x+h[1]*y+h[2])/d-u, dv=(h[3]*x+h[4]*y+h[5])/d-v;
  return du*du+dv*dv;
}
std::array<double,9> multiply(const std::array<double,9>& a,const std::array<double,9>& b) {
  std::array<double,9> c{};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)c[i*3+j]+=a[i*3+k]*b[k*3+j];
  return c;
}
}

GeometryEstimationResult vector_to_proj_hom_mat2d(const core::Tuple& px,const core::Tuple& py,
    const core::Tuple& qx,const core::Tuple& qy,const core::Tuple& weights,const GeometryEstimationOptions& options) {
  Inputs input({&px,&py,&qx,&qy},weights,options);
  auto& c=input.coordinates;
  const auto source=normalize(&c[0],&c[1]), dest=normalize(&c[2],&c[3]);
  auto fit=[&](const std::vector<std::size_t>& ids) {
    if(ids.size()==c[0].size()) {
      const bool uniform=std::all_of(input.weights.begin(),input.weights.end(),[](double w){return w==1;});
      return fit_planar_homography_2d(std::span<const double>(c[0]),c[1],c[2],c[3],numeric::default_backend_kind(),
          uniform?std::span<const double>{}:std::span<const double>(input.weights),true);
    }
    std::array<std::vector<double>,5> a;
    for(auto& values:a)values.reserve(ids.size());
    for(auto i:ids) { for(int k=0;k<4;++k)a[k].push_back(c[k][i]); a[4].push_back(input.weights[i]); }
    return fit_planar_homography_2d(std::span<const double>(a[0]),a[1],a[2],a[3],numeric::default_backend_kind(),a[4]);
  };
  auto error=[&](const auto& h,std::size_t i) { return hom_error(h,c[0][i],c[1][i],c[2][i],c[3][i])/(dest.scale*dest.scale); };
  GeometryEstimationResult out; std::vector<std::size_t> ids;
  auto matrix=consensus<std::array<double,9>>(input,4,options,fit,error,&ids,&out.trials);
  refine_planar_homography(&matrix,c[0],c[1],c[2],c[3],ids,input.weights,options.max_iterations,&out.iterations,&out.converged);
  quality(&out,matrix,input,options,error);
  require(out.inlier_count >= std::max<std::size_t>(4,options.robust?std::ceil(options.min_inlier_ratio*input.active.size()):4),
          "consensus","insufficient final inliers",core::Status::InvalidState);
  matrix=multiply({1/dest.scale,0,dest.x,0,1/dest.scale,dest.y,0,0,1},
      multiply(matrix,{source.scale,0,-source.scale*source.x,0,source.scale,-source.scale*source.y,0,0,1}));
  require(std::isfinite(matrix[8]) && std::abs(matrix[8])>1e-18,"model","cannot normalize homography",core::Status::InvalidState);
  const double gauge=matrix[8]; for(auto v:matrix) { require(std::isfinite(v),"model","nonfinite homography",core::Status::InvalidState);out.model.append(v/gauge); }
  out.geometry_rank=2;return out;
}

namespace {
struct WorldFrame {
  Point3d center{};
  std::array<double,9> basis{};
  double scale{};
  bool planar{};
};
WorldFrame world_frame(const std::vector<Point3d>& world) {
  WorldFrame f;const auto anchor=world.front();
  for(const auto p:world){f.center.x+=p.x-anchor.x;f.center.y+=p.y-anchor.y;f.center.z+=p.z-anchor.z;}
  f.center={anchor.x+f.center.x/world.size(),anchor.y+f.center.y/world.size(),anchor.z+f.center.z/world.size()};
  std::vector<double> covariance(9);
  for(const auto p:world){const double a[]{p.x-f.center.x,p.y-f.center.y,p.z-f.center.z};
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)covariance[r*3+c]+=a[r]*a[c];}
  std::vector<double> values,vectors;
  require(numeric::shared_backend(numeric::default_backend_kind()).symmetric_eigen(covariance,3,&values,&vectors),
          "World","geometry decomposition failed",core::Status::InvalidState);
  require(values[2]>1e-24 && values[1]>values[2]*1e-10,"World","collinear or near-collinear world geometry",core::Status::InvalidState);
  f.planar=values[0]<=values[2]*1e-10;
  for(int r=0;r<3;++r){f.basis[r*3]=vectors[r*3+2];f.basis[r*3+1]=vectors[r*3+1];}
  f.basis[2]=f.basis[3]*f.basis[7]-f.basis[6]*f.basis[4];
  f.basis[5]=f.basis[6]*f.basis[1]-f.basis[0]*f.basis[7];
  f.basis[8]=f.basis[0]*f.basis[4]-f.basis[3]*f.basis[1];
  f.scale=std::sqrt((values[0]+values[1]+values[2])/world.size());
  require(std::isfinite(f.scale),"World","world spread overflow",core::Status::InvalidState);
  return f;
}
Pose3d initialize(const CameraParameters& camera,const std::vector<Point3d>& points,
                  const std::vector<double>& row,const std::vector<double>& col,
                  const std::vector<double>& weight) {
  const auto frame=world_frame(points);
  std::vector<Point3d> local; std::vector<std::int64_t> indices;
  for(const auto p:points){const double a[]{p.x-frame.center.x,p.y-frame.center.y,p.z-frame.center.z};Point3d q{};
    q.x=(a[0]*frame.basis[0]+a[1]*frame.basis[3]+a[2]*frame.basis[6])/frame.scale;
    q.y=(a[0]*frame.basis[1]+a[1]*frame.basis[4]+a[2]*frame.basis[7])/frame.scale;
    q.z=frame.planar?0:(a[0]*frame.basis[2]+a[1]*frame.basis[5]+a[2]*frame.basis[8])/frame.scale;
    local.push_back(q);indices.push_back(indices.size());}
  Pose3d pose=frame.planar?estimate_planar_pose_perspective(camera,local,row,col,indices,numeric::default_backend_kind(),nullptr,weight):
                          initialize_nonplanar_pose(camera,local,row,col,weight);
  const auto r=rotation_matrix(pose.rotation);
  std::array<double,9> global{};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)global[i*3+j]+=r[i*3+k]*frame.basis[j*3+k];
  pose.rotation=quaternion_from_rotation_matrix(global);
  const auto rotated=rotate_point(pose.rotation,frame.center);
  pose.translation={pose.translation.x*frame.scale-rotated.x,pose.translation.y*frame.scale-rotated.y,pose.translation.z*frame.scale-rotated.z};
  return pose;
}
Pose3d step_pose(const Pose3d& p,std::span<const double> step) {
  const double angle=std::hypot(step[0],std::hypot(step[1],step[2]));
  const double factor=angle<1e-12?.5:std::sin(.5*angle)/angle;
  Pose3d delta{{step[3],step[4],step[5]},{std::cos(.5*angle),factor*step[0],factor*step[1],factor*step[2]}};
  return compose_pose(delta,p);
}
Point2d project(const CameraParameters& camera,const Pose3d& pose,const Point3d& point) {
  const auto transformed=transform_point(pose,point);
  const Point2d invalid{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
  if(transformed.z<=1e-12)return invalid;
  try { return project_camera_point(camera,transformed); }
  catch(const core::Exception& ex) {
    // 采样前已验证相机和输入。试探位姿或 LM 步可能离开畸变可逆域，
    // 仅拒绝该候选，不中止其余有效共识候选；其他失败继续传播。
    if(ex.status()==core::Status::InvalidState || ex.status()==core::Status::InvalidArgument)return invalid;
    throw;
  }
}
void refine_pose(Pose3d* pose,const CameraParameters& camera,const std::vector<Point3d>& world,
    const std::vector<double>& rows,const std::vector<double>& cols,const std::vector<double>& weights,
    const std::vector<std::size_t>& ids,const GeometryEstimationOptions& options,GeometryEstimationResult* out) {
  auto cost=[&](const Pose3d& p){double sum=0;for(auto i:ids){auto q=project(camera,p,world[i]);
    sum+=weights[i]*(std::pow(q.row-rows[i],2)+std::pow(q.col-cols[i],2));}return sum;};
  double current=cost(*pose),lambda=1e-3;
  require(std::isfinite(current),"Pose","initial pose projects points behind camera",core::Status::InvalidState);
  std::vector<double> normal(36),rhs(6),solution(6);
  for(std::size_t iteration=0;iteration<options.max_iterations;++iteration){
    out->iterations=iteration+1;std::fill(normal.begin(),normal.end(),0);std::fill(rhs.begin(),rhs.end(),0);
    std::array<Pose3d,6> perturbed;
    const double epsilon=1e-6;
    for(int k=0;k<6;++k){std::array<double,6> d{};d[k]=epsilon;perturbed[k]=step_pose(*pose,d);}
    for(auto i:ids){const auto q=project(camera,*pose,world[i]);double jr[6],jc[6];
      for(int k=0;k<6;++k){const auto other=project(camera,perturbed[k],world[i]);jr[k]=(other.row-q.row)/epsilon;jc[k]=(other.col-q.col)/epsilon;}
      for(int r=0;r<6;++r){rhs[r]+=weights[i]*(jr[r]*(rows[i]-q.row)+jc[r]*(cols[i]-q.col));
        for(int c=0;c<6;++c)normal[r*6+c]+=weights[i]*(jr[r]*jr[c]+jc[r]*jc[c]);}}
    double maximum=0;for(auto value:rhs)maximum=std::max(maximum,std::abs(value));
    if(maximum<=1e-8){out->converged=true;break;}
    for(int k=0;k<6;++k)normal[k*6+k]+=lambda*(1+normal[k*6+k]);
    if(!numeric::shared_backend(numeric::default_backend_kind()).solve_symmetric_into(normal,rhs,solution))break;
    const auto candidate=step_pose(*pose,solution);const double next=cost(candidate);
    double norm=0;for(auto value:solution)norm+=value*value;
    if(next<current){const double improvement=current-next;*pose=candidate;current=next;lambda=std::max(1e-9,lambda*.3);
      if(norm<1e-18 || improvement<=1e-10*(1+current)){out->converged=true;break;}}
    else lambda=std::min(lambda*10,1e10);
  }
}
}
GeometryEstimationResult vector_to_pose(const core::Tuple& wx,const core::Tuple& wy,const core::Tuple& wz,
    const core::Tuple& row,const core::Tuple& col,const core::Tuple& camera_tuple,const core::Tuple& weights,
    const GeometryEstimationOptions& options) {
  Inputs input({&wx,&wy,&wz,&row,&col},weights,options);
  const auto camera=parse_camera_parameters(camera_tuple);
  require(is_perspective(camera),"CameraParam","extended pose requires perspective camera",core::Status::Unsupported);
  const auto& a=input.coordinates;
  std::vector<Point3d> world,active;
  for(std::size_t i=0;i<a[0].size();++i)world.push_back({a[0][i],a[1][i],a[2][i]});
  for(auto i:input.active)active.push_back(world[i]);
  require(active.size()>=4,"weights","at least four positive observations required");
  const auto frame=world_frame(active);const std::size_t minimum=frame.planar?4:6;
  auto fit=[&](const std::vector<std::size_t>& ids){std::vector<Point3d> points;std::vector<double> r,c,w;
    for(auto i:ids){points.push_back(world[i]);r.push_back(a[3][i]);c.push_back(a[4][i]);w.push_back(input.weights[i]);}
    return initialize(camera,points,r,c,w);};
  auto error=[&](const Pose3d& p,std::size_t i){const auto q=project(camera,p,world[i]);return std::pow(q.row-a[3][i],2)+std::pow(q.col-a[4][i],2);};
  GeometryEstimationResult out;out.geometry_rank=frame.planar?2:3;std::vector<std::size_t> ids;
  auto pose=consensus<Pose3d>(input,minimum,options,fit,error,&ids,&out.trials);
  refine_pose(&pose,camera,world,a[3],a[4],input.weights,ids,options,&out);
  quality(&out,pose,input,options,error);
  require(out.inlier_count>=std::max(minimum,options.robust?static_cast<std::size_t>(std::ceil(options.min_inlier_ratio*active.size())):minimum),
          "consensus","insufficient final pose inliers",core::Status::InvalidState);
  out.model=pose_to_halcon_tuple(pose);return out;
}
} // namespace mv::calibration
