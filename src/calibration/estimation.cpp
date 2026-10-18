#include <algorithm>
#include <array>
#include <calibration/calibration/homography.hpp>
#include <calibration/calibration/initialize.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/estimation.hpp>
#include <calibration/estimation_avx2.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/numeric/linear_backend.hpp>
#include <cmath>
#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <geometry/hom_mat2d.hpp>
#include <limits>
#include <runtime/cpu/cpu_features.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_AFFINE_FIT_SSE2 1
#else
#define MVKIT_AFFINE_FIT_SSE2 0
#endif

namespace mv::calibration {
namespace {

void require(bool ok, const char* op, const char* parameter, const char* message) {
  if (!ok) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                          message);
  }
}

double finite_number(const core::Tuple& tuple, std::size_t index, const char* op, const char* parameter) {
  double v = 0;
  try {
    v = tuple.as_real64(index);
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, parameter,
                          static_cast<std::int64_t>(index), "values must be numeric");
  }
  require(mv::core::is_finite(v), op, parameter, "finite values required");
  return v;
}

struct NumericInput {
  const core::Tuple& tuple;
  bool scalar;
  std::span<const double> reals;
  std::span<const std::int64_t> integers;
  explicit NumericInput(const core::Tuple& value) : tuple(value), scalar(value.size() == 1) {
    if (value.kind() == core::TupleKind::Real64)
      reals = value.real64_span();
    else if (value.kind() == core::TupleKind::Int64)
      integers = value.int64_span();
  }
  double read(std::size_t index, const char* op, const char* parameter) const {
    index = scalar ? 0 : index;
    if (reals.empty() && integers.empty()) return finite_number(tuple, index, op, parameter);
    const double value = reals.empty() ? static_cast<double>(integers[index]) : reals[index];
    require(mv::core::is_finite(value), op, parameter, "finite values required");
    return value;
  }
};

// 已拥有连续实数的点对与归一化工作区共享同一验证及矩归约，保持点优先错误顺序。
bool validate_pairs(const std::array<std::span<const double>,4>& values, std::size_t count,
                    const char* op, double* normal, double* rhs) {
  constexpr const char* parameters[]{"Px", "Py", "Qx", "Qy"};
  bool ready=false;
      // 成功扫描不进入外部错误处理；非法点对保留原参数顺序及诊断。
      std::size_t validated = 0;
#if MVKIT_CALIBRATION_HAS_AVX2
      const auto& avx_cpu = mv::runtime::cpu_features();
      if (mv::runtime::kernel_allowed(
              mv::runtime::effective_capabilities(avx_cpu.selection(), avx_cpu.hardware()),
              {false, true, false, false})) {
        if (normal && rhs) {
          validated = checked_affine_moments_avx2(values[0].data(), values[1].data(), values[2].data(),
                                                  values[3].data(), count, normal, rhs);
          ready = validated == count;
        } else
          validated = finite_pairs_avx2(values[0].data(), values[1].data(), values[2].data(),
                                        values[3].data(), count);
      }
#endif
#if MVKIT_AFFINE_FIT_SSE2
      const auto& cpu = mv::runtime::cpu_features();
      if (mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                      {true, false, false, false})) {
        const auto sign = _mm_set1_pd(-0.), limit = _mm_set1_pd(std::numeric_limits<double>::max());
        for (; count - validated >= 2; validated += 2) {
          auto valid = _mm_castsi128_pd(_mm_set1_epi32(-1));
          for (int k = 0; k < 4; ++k)
            valid = _mm_and_pd(
                valid, _mm_cmple_pd(_mm_andnot_pd(sign, _mm_loadu_pd(values[k].data() + validated)), limit));
          if (_mm_movemask_pd(valid) != 3) break;
        }
      }
#endif
      for (std::size_t i = validated; i < count; ++i) {
        if (mv::core::is_finite(values[0][i]) && mv::core::is_finite(values[1][i]) &&
            mv::core::is_finite(values[2][i]) && mv::core::is_finite(values[3][i]))
          continue;
        for (int k = 0; k < 4; ++k)
          require(mv::core::is_finite(values[k][i]), op, parameters[k], "finite values required");
      }
  return ready;
}

struct NumericPairs {
  std::array<std::vector<double>, 4> owned;
  std::array<std::span<const double>, 4> values;
  bool affine_moments_ready = false;
  NumericPairs(const core::Tuple& px, const core::Tuple& py, const core::Tuple& qx, const core::Tuple& qy,
               const core::BroadcastPlan& plan, const char* op, double* normal = nullptr,
               double* rhs = nullptr) {
    const NumericInput inputs[]{NumericInput(px), NumericInput(py), NumericInput(qx), NumericInput(qy)};
    constexpr const char* parameters[]{"Px", "Py", "Qx", "Qy"};
    bool borrow = true;
    for (const auto& input : inputs) borrow = borrow && input.reals.size() == plan.size();
    if (borrow) {
      for (int k = 0; k < 4; ++k) values[k] = inputs[k].reals;
      affine_moments_ready=validate_pairs(values,plan.size(),op,normal,rhs);
    } else {
      for (int k = 0; k < 4; ++k) {
        owned[k].resize(plan.size());
        values[k] = owned[k];
      }
      for (std::size_t i = 0; i < plan.size(); ++i)
        for (int k = 0; k < 4; ++k) owned[k][i] = inputs[k].read(i, op, parameters[k]);
    }
  }
};

struct PoseMethod {
  bool robust{false};
  RobustPlanarPoseOptions options{};
};

PoseMethod parse_pose_method(const core::Tuple& method, const char* op) {
  if (method.empty()) return {};
  try {
    const auto name = std::string(method.string_view(0));
    if (name == "analytic" && method.size() == 1) return {};
    if (name == "ransac" && method.size() == 4) {
      const double threshold = finite_number(method, 1, op, "Method");
      const auto trials = method.as_int64(2);
      const auto seed = method.as_int64(3);
      if (threshold <= 0 || trials < 1 || trials > 100000)
        throw core::Exception(core::Status::InvalidArgument, op,
                              "ransac requires positive pixel threshold and 1..100000 trials");
      return {true, {threshold, static_cast<std::size_t>(trials), static_cast<std::uint64_t>(seed)}};
    }
    if (name != "analytic" && name != "ransac")
      throw core::Exception(core::Status::Unsupported, op, "unsupported pose estimation method");
    throw core::Exception(core::Status::InvalidArgument, op,
                          "Method must be analytic or [ransac, threshold_pixels, max_trials, seed]");
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::Unsupported || ex.status() == core::Status::InvalidArgument) throw;
    throw core::Exception(core::Status::InvalidArgument, op, "invalid Method tuple");
  }
}

bool all_zero_z(const std::vector<Point3d>& points) {
  return std::all_of(points.begin(), points.end(), [](const Point3d& p) { return std::abs(p.z) <= 1e-10; });
}

bool collinear(const std::vector<Point3d>& points) {
  if (points.size() < 3) return true;
  const auto& a = points.front();
  Point3d dir{};
  std::size_t start = 1;
  for (; start < points.size(); ++start) {
    dir = {points[start].x - a.x, points[start].y - a.y, points[start].z - a.z};
    if (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z > 1e-24) break;
  }
  if (start >= points.size()) return true;
  for (std::size_t i = start + 1; i < points.size(); ++i) {
    const Point3d v{points[i].x - a.x, points[i].y - a.y, points[i].z - a.z};
    const Point3d cross{dir.y * v.z - dir.z * v.y, dir.z * v.x - dir.x * v.z, dir.x * v.y - dir.y * v.x};
    if (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z > 1e-20) return false;
  }
  return true;
}

bool collinear_2d(std::span<const double> x, std::span<const double> y) {
  if (x.size() < 3) return true;
  double dx = 0;
  double dy = 0;
  std::size_t start = 1;
  for (; start < x.size(); ++start) {
    dx = x[start] - x[0];
    dy = y[start] - y[0];
    if (dx * dx + dy * dy > 1e-24) break;
  }
  if (start >= x.size()) return true;
  for (std::size_t i = start + 1; i < x.size(); ++i) {
    if (std::abs(dx * (y[i] - y[0]) - dy * (x[i] - x[0])) > 1e-10) return false;
  }
  return true;
}

Pose3d dlt_pnp(const CameraParameters& camera, const std::vector<Point3d>& world,
               const std::vector<double>& rows, const std::vector<double>& columns,
               std::span<const double> weights = {}) {
  constexpr auto op = "vector_to_pose";
  if (!is_perspective(camera) || world.size() < 6) {
    throw core::Exception(core::Status::InvalidArgument, op,
                          "non-planar pose estimation requires a perspective camera and six points");
  }
  std::vector<double> design;
  design.reserve(world.size() * 24);
  for (std::size_t i = 0; i < world.size(); ++i) {
    const auto ray = image_point_to_ray(camera, {rows[i], columns[i]});
    if (std::abs(ray.direction.z) < 1e-14) {
      throw core::Exception(core::Status::InvalidState, op, "image ray is parallel to the image plane");
    }
    const double u = ray.direction.x / ray.direction.z;
    const double v = ray.direction.y / ray.direction.z;
    const double X = world[i].x;
    const double Y = world[i].y;
    const double Z = world[i].z;
    std::array<double, 12> ru{X, Y, Z, 1, 0, 0, 0, 0, -u * X, -u * Y, -u * Z, -u};
    std::array<double, 12> rv{0, 0, 0, 0, X, Y, Z, 1, -v * X, -v * Y, -v * Z, -v};
    if (!weights.empty()) {
      const double factor = std::sqrt(weights[i]);
      for (auto& value : ru) value *= factor;
      for (auto& value : rv) value *= factor;
    }
    design.insert(design.end(), ru.begin(), ru.end());
    design.insert(design.end(), rv.begin(), rv.end());
  }
  auto backend = numeric::make_backend(numeric::default_backend_kind());
  std::vector<double> p;
  if (!backend->smallest_right_singular_vector(design, world.size() * 2, 12, &p) || p.size() != 12) {
    throw core::Exception(core::Status::InvalidState, op, "PnP DLT solve is rank deficient");
  }
  std::array<double, 9> a{p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]};
  std::array<double, 9> ata{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) ata[i * 3 + j] += a[k * 3 + i] * a[k * 3 + j];
  std::vector<double> eval;
  std::vector<double> evec;
  if (!backend->symmetric_eigen(std::vector<double>(ata.begin(), ata.end()), 3, &eval, &evec) ||
      eval.size() != 3) {
    throw core::Exception(core::Status::InvalidState, op, "PnP rotation polar decomposition failed");
  }
  // 特征向量按列存储，最大特征值排最后。
  std::array<double, 9> v{evec[0], evec[3], evec[6], evec[1], evec[4], evec[7], evec[2], evec[5], evec[8]};
  std::array<double, 9> r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r[i * 3 + j] += a[i * 3 + k] * v[k * 3 + j] / std::sqrt(eval[j] + 1e-30);
  // 由 A=U*S*V^T，通过极分解 A*(A^T*A)^(-1/2) 重建 R≈U*V^T。
  std::array<double, 9> sinv{};
  for (int i = 0; i < 3; ++i) sinv[i * 3 + i] = 1.0 / std::sqrt(std::max(eval[i], 1e-30));
  std::array<double, 9> atainvsqrt{};
  std::array<double, 9> tmp{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) tmp[i * 3 + j] += evec[i * 3 + k] * sinv[k * 3 + j];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) atainvsqrt[i * 3 + j] += tmp[i * 3 + k] * evec[j * 3 + k];
  r = {};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r[i * 3 + j] += a[i * 3 + k] * atainvsqrt[k * 3 + j];
  const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                     r[2] * (r[3] * r[7] - r[4] * r[6]);
  if (det < 0) {
    for (double& entry : r) entry = -entry;
    for (int i = 0; i < 12; ++i) p[i] = -p[i];
  }
  const double scale = (std::sqrt(std::max(eval[0], 0.0)) + std::sqrt(std::max(eval[1], 0.0)) +
                        std::sqrt(std::max(eval[2], 0.0))) /
                       3.0;
  if (scale < 1e-18) {
    throw core::Exception(core::Status::InvalidState, op, "PnP scale is degenerate");
  }
  Point3d t{p[3] / scale, p[7] / scale, p[11] / scale};
  Pose3d pose{t, quaternion_from_rotation_matrix(r)};
  std::size_t behind = 0;
  for (const auto& w : world) {
    if (transform_point(pose, w).z <= 0) ++behind;
  }
  if (behind > world.size() / 2) {
    const auto rneg = rotation_matrix(pose.rotation);
    std::array<double, 9> flipped{};
    for (int i = 0; i < 9; ++i) flipped[i] = -rneg[i];
    pose.rotation = quaternion_from_rotation_matrix(flipped);
    pose.translation = {-pose.translation.x, -pose.translation.y, -pose.translation.z};
  }
  return pose;
}

double pose_rmse(const CameraParameters& camera, const Pose3d& pose, const std::vector<Point3d>& world,
                 const std::vector<double>& rows, const std::vector<double>& columns) {
  double sum = 0;
  for (std::size_t i = 0; i < world.size(); ++i) {
    const auto proj = project_camera_point(camera, transform_point(pose, world[i]));
    const double dr = proj.row - rows[i];
    const double dc = proj.col - columns[i];
    sum += dr * dr + dc * dc;
  }
  return std::sqrt(sum / static_cast<double>(world.size()));
}

}  // namespace

Pose3d initialize_nonplanar_pose(const CameraParameters& camera, const std::vector<Point3d>& world,
                                 const std::vector<double>& rows, const std::vector<double>& columns,
                                 std::span<const double> weights) {
  return dlt_pnp(camera, world, rows, columns, weights);
}

void vector_to_pose(const core::Tuple& world_x, const core::Tuple& world_y, const core::Tuple& world_z,
                    const core::Tuple& image_row, const core::Tuple& image_column,
                    const core::Tuple& camera_param, const core::Tuple& method, core::Tuple* pose,
                    core::Tuple* quality) {
  constexpr auto op = "vector_to_pose";
  if (pose == nullptr || quality == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op, "outputs", -1,
                          "outputs must not be null");
  }
  if (pose == quality) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "outputs", -1,
                          "parallel Tuple outputs must be distinct");
  }
  const auto pose_method = parse_pose_method(method, op);
  const auto camera = parse_camera_parameters(camera_param);
  const auto plan = core::BroadcastPlan::make({{world_x.size(), "WorldX", true},
                                               {world_y.size(), "WorldY", true},
                                               {world_z.size(), "WorldZ", true},
                                               {image_row.size(), "ImageRow", true},
                                               {image_column.size(), "ImageColumn", true}},
                                              op);
  std::vector<Point3d> world;
  std::vector<double> rows;
  std::vector<double> columns;
  std::vector<std::int64_t> indices;
  world.reserve(plan.size());
  rows.reserve(plan.size());
  columns.reserve(plan.size());
  indices.reserve(plan.size());
  for (std::size_t i = 0; i < plan.size(); ++i) {
    world.push_back({finite_number(world_x, plan.source_index(0, i), op, "WorldX"),
                     finite_number(world_y, plan.source_index(1, i), op, "WorldY"),
                     finite_number(world_z, plan.source_index(2, i), op, "WorldZ")});
    rows.push_back(finite_number(image_row, plan.source_index(3, i), op, "ImageRow"));
    columns.push_back(finite_number(image_column, plan.source_index(4, i), op, "ImageColumn"));
    indices.push_back(static_cast<std::int64_t>(i));
  }
  if (collinear(world)) {
    throw core::Exception(core::Status::InvalidState, op, "object points are collinear");
  }
  Pose3d estimated;
  if (pose_method.robust && (!all_zero_z(world) || !is_perspective(camera)))
    throw core::Exception(core::Status::Unsupported, op,
                          "ransac currently supports planar perspective correspondences only");
  if (all_zero_z(world)) {
    if (rows.size() < 4) {
      throw core::Exception(core::Status::InvalidArgument, op,
                            "planar pose estimation requires at least four points");
    }
    estimated = is_perspective(camera)
                    ? estimate_planar_pose_perspective(camera, world, rows, columns, indices,
                                                       numeric::default_backend_kind(),
                                                       pose_method.robust ? &pose_method.options : nullptr)
                    : estimate_planar_pose_telecentric(camera, world, rows, columns, indices);
  } else {
    estimated = dlt_pnp(camera, world, rows, columns);
  }
  *pose = pose_to_halcon_tuple(estimated);
  *quality = core::Tuple{pose_rmse(camera, estimated, world, rows, columns)};
}

namespace {
std::array<double,6> fit_affine_pairs(const std::array<std::span<const double>,4>& values,std::size_t count,
                      bool halcon_contract,bool coordinates_normalized,
                      bool affine_moments_ready,std::array<double,36>& ata,std::array<double,6>& atb) {
  constexpr auto op="vector_to_hom_mat2d";
  const auto [x,y,u,v]=values;
  const bool deficient = collinear_2d(x, y);
  if (halcon_contract && !deficient && !coordinates_normalized) {
    const auto center = [&](const auto& values) {
      long double sum = 0.;
      for (auto value : values) sum += static_cast<long double>(value) - values[0];
      return values[0] + static_cast<double>(sum / count);
    };
    const double mx = center(x), my = center(y), mu = center(u), mv = center(v);
    double sx = 0., sy = 0.;
    for (std::size_t i = 0; i < count; ++i) {
      sx = std::max(sx, std::abs(x[i] - mx));
      sy = std::max(sy, std::abs(y[i] - my));
    }
    // 常见少量标定点使用固定栈工作区，大输入仍逐列分配；不改变归一化公式及求解顺序。
    std::array<std::array<double,16>,4> small;
    std::array<std::vector<double>,4> large;
    std::array<std::span<double>,4> workspace;
    for(std::size_t k=0;k<4;++k) {
      if(count<=16)workspace[k]=std::span{small[k]}.first(count);
      else {large[k].resize(count);workspace[k]=large[k];}
    }
    const auto [xx,yy,uu,vv]=workspace;
    for (std::size_t i = 0; i < count; ++i) {
      xx[i] = (x[i] - mx) / sx;
      yy[i] = (y[i] - my) / sy;
      uu[i] = u[i] - mu;
      vv[i] = v[i] - mv;
    }
    const std::array<std::span<const double>,4> normalized_values{xx,yy,uu,vv};
    std::array<double,36> next_ata{};
    std::array<double,6> next_atb{};
    const bool ready=validate_pairs(normalized_values,count,op,next_ata.data(),next_atb.data());
    // 内部只传递六个系数，保留原中间结果有限值校验，不创建临时拥有型 Tuple。
    const auto normalized=fit_affine_pairs(normalized_values,count,true,true,ready,next_ata,next_atb);
    for(double value:normalized)if(!std::isfinite(value))
      throw core::Exception(core::Status::InvalidArgument,"hom_mat2d","affine result is not finite");
    std::array<double, 6> values{};
    values[0] = normalized[0] / sx;
    values[1] = normalized[1] / sy;
    values[2] = mu + normalized[2] - values[0] * mx - values[1] * my;
    values[3] = normalized[3] / sx;
    values[4] = normalized[4] / sy;
    values[5] = mv + normalized[5] - values[3] * mx - values[4] * my;
    return values;
  }
  if (!halcon_contract && (deficient || collinear_2d(u, v)))
    throw core::Exception(core::Status::InvalidState, op, "affine correspondences are collinear");
  if (halcon_contract && deficient) {
    // 带主元及重复正交化的修正 Gram-Schmidt；秩不足的仿射映射有效，不可观测主元坐标置零。
    std::array<std::vector<double>, 3> a{std::vector<double>(x.begin(), x.end()),
                                         std::vector<double>(y.begin(), y.end()),
                                         std::vector<double>(count, 1.)};
    std::array<int, 3> permutation{0, 1, 2};
    double triangular[3][3]{}, rhs[2][3]{};
    int rank = 0;
    double initial = 0.;
    for (int j = 0; j < 3; ++j) {
      double norm = 0.;
      for (auto value : a[j]) norm = std::hypot(norm, value);
      initial = std::max(initial, norm);
    }
    for (int k = 0; k < 3; ++k) {
      int pivot = k;
      double largest = 0.;
      for (int j = k; j < 3; ++j) {
        double norm = 0.;
        for (auto value : a[j]) norm = std::hypot(norm, value);
        if (norm >= largest) {
          largest = norm;
          pivot = j;
        }
      }
      if (largest <= initial * std::numeric_limits<double>::epsilon() * double(count) * 8.) break;
      std::swap(a[k], a[pivot]);
      std::swap(permutation[k], permutation[pivot]);
      for (int i = 0; i < k; ++i) std::swap(triangular[i][k], triangular[i][pivot]);
      triangular[k][k] = largest;
      for (auto& value : a[k]) value /= largest;
      for (std::size_t i = 0; i < count; ++i) {
        rhs[0][k] += a[k][i] * u[i];
        rhs[1][k] += a[k][i] * v[i];
      }
      for (int j = k + 1; j < 3; ++j)
        for (int pass = 0; pass < 2; ++pass) {
          double dot = 0.;
          for (std::size_t i = 0; i < count; ++i) dot += a[k][i] * a[j][i];
          triangular[k][j] += dot;
          for (std::size_t i = 0; i < count; ++i) a[j][i] -= dot * a[k][i];
        }
      ++rank;
    }
    std::array<double, 6> solution{};
    if (rank < 2)
      throw core::Exception(core::Status::InvalidState, op, "affine source points have no spatial extent");
    for (int axis = 0; axis < 2; ++axis) {
      double values[3]{};
      for (int i = rank - 1; i >= 0; --i) {
        double value = rhs[axis][i];
        for (int j = i + 1; j < rank; ++j) value -= triangular[i][j] * values[j];
        values[i] = value / triangular[i][i];
      }
      for (int j = 0; j < 3; ++j) solution[axis * 3 + permutation[j]] = values[j];
    }
    return solution;
  }

  std::size_t k = affine_moments_ready ? count : 0;
#if MVKIT_CALIBRATION_HAS_AVX2
  const auto& avx_cpu = mv::runtime::cpu_features();
  if (k == 0 && mv::runtime::kernel_allowed(
                    mv::runtime::effective_capabilities(avx_cpu.selection(), avx_cpu.hardware()),
                    {false, true, false, false})) {
    affine_moments_avx2(x.data(), y.data(), u.data(), v.data(), count, ata.data(), atb.data());
    k = count;
  }
#endif
#if MVKIT_AFFINE_FIT_SSE2
  const auto& cpu = mv::runtime::cpu_features();
  if (k == 0 &&
      mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                  {true, false, false, false})) {
    __m128d moment[5]{_mm_setzero_pd(), _mm_setzero_pd(), _mm_setzero_pd(), _mm_setzero_pd(),
                      _mm_setzero_pd()};
    for (; count - k >= 2; k += 2) {
      const auto a = _mm_loadu_pd(x.data() + k), b = _mm_loadu_pd(y.data() + k);
      moment[0] = _mm_add_pd(moment[0], _mm_mul_pd(a, a));
      moment[1] = _mm_add_pd(moment[1], _mm_mul_pd(a, b));
      moment[2] = _mm_add_pd(moment[2], _mm_mul_pd(b, b));
      moment[3] = _mm_add_pd(moment[3], a);
      moment[4] = _mm_add_pd(moment[4], b);
    }
    constexpr int positions[]{0, 1, 7, 2, 8};
    for (int i = 0; i < 5; ++i) {
      double lanes[2];
      _mm_storeu_pd(lanes, moment[i]);
      ata[positions[i]] = lanes[0] + lanes[1];
    }
    __m128d rhs[6]{_mm_setzero_pd(), _mm_setzero_pd(), _mm_setzero_pd(),
                   _mm_setzero_pd(), _mm_setzero_pd(), _mm_setzero_pd()};
    for (std::size_t i = 0; i < k; i += 2) {
      const auto a = _mm_loadu_pd(x.data() + i), b = _mm_loadu_pd(y.data() + i),
                 c = _mm_loadu_pd(u.data() + i), d = _mm_loadu_pd(v.data() + i);
      rhs[0] = _mm_add_pd(rhs[0], _mm_mul_pd(a, c));
      rhs[1] = _mm_add_pd(rhs[1], _mm_mul_pd(b, c));
      rhs[2] = _mm_add_pd(rhs[2], c);
      rhs[3] = _mm_add_pd(rhs[3], _mm_mul_pd(a, d));
      rhs[4] = _mm_add_pd(rhs[4], _mm_mul_pd(b, d));
      rhs[5] = _mm_add_pd(rhs[5], d);
    }
    for (int i = 0; i < 6; ++i) {
      double lanes[2];
      _mm_storeu_pd(lanes, rhs[i]);
      atb[i] = lanes[0] + lanes[1];
    }
  }
#endif
  for (; k < count; ++k) {
    ata[0] += x[k] * x[k];
    ata[1] += x[k] * y[k];
    ata[7] += y[k] * y[k];
    ata[2] += x[k];
    ata[8] += y[k];
    atb[0] += x[k] * u[k];
    atb[1] += y[k] * u[k];
    atb[2] += u[k];
    atb[3] += x[k] * v[k];
    atb[4] += y[k] * v[k];
    atb[5] += v[k];
  }
  ata[6] = ata[1];
  ata[12] = ata[2];
  ata[13] = ata[8];
  ata[14] = double(count);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) ata[(i + 3) * 6 + j + 3] = ata[i * 6 + j];
  const auto& backend = numeric::shared_backend(numeric::default_backend_kind());
  std::array<double, 6> solution;
  if (!backend.solve_symmetric_into(ata, atb, solution)) {
    throw core::Exception(core::Status::InvalidState, op, "affine fit is rank deficient");
  }
  return solution;
}
}  // namespace

void vector_to_hom_mat2d(const core::Tuple& px, const core::Tuple& py, const core::Tuple& qx,
                         const core::Tuple& qy, core::Tuple* hom_mat2d, bool halcon_contract,
                         bool coordinates_normalized) {
  constexpr auto op = "vector_to_hom_mat2d";
  if (hom_mat2d == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op, "HomMat2D", -1,
                          "output must not be null");
  }
  const auto plan = core::BroadcastPlan::make(
      {{px.size(), "Px", true}, {py.size(), "Py", true}, {qx.size(), "Qx", true}, {qy.size(), "Qy", true}},
      op);
  if (plan.size() < 3) {
    throw core::Exception(core::Status::InvalidArgument, op, "at least three point pairs are required");
  }
  std::array<double, 36> ata{};
  std::array<double, 6> atb{};
  // 正式未归一化输入后续必走归一化或秩不足路径，二者都不消费这份原坐标矩。
  // 只执行同顺序有限值验证，避免计算即将丢弃的矩；实际求解仍归约同一份坐标。
  const bool needs_moments=!halcon_contract||coordinates_normalized;
  const NumericPairs coordinates(px,py,qx,qy,plan,op,needs_moments?ata.data():nullptr,
                                 needs_moments?atb.data():nullptr);
  const auto values=fit_affine_pairs(coordinates.values,plan.size(),halcon_contract,coordinates_normalized,
                                   coordinates.affine_moments_ready,ata,atb);
  *hom_mat2d=geometry::hom_mat2d_from_values(values);
}

}  // namespace mv::calibration
