#include "normal_estimation.hpp"

#include <algorithm>
#include <calibration/numeric/linear_backend.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <numeric>
#include <reconstruction/neighborhood_index.hpp>
#include <vector>
namespace mv::reconstruction {
namespace {
constexpr auto op = "surface_normals_object_model_3d";
using P = Point3d;
P add(P a, P b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
P sub(P a, P b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
P mul(P a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(P a, P b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
P cross(P a, P b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
P unit(P a) {
  const auto n = std::hypot(a.x, a.y, a.z);
  return n > 0 && std::isfinite(n) ? mul(a, 1 / n) : P{};
}
bool valid(const ObjectModel3DData& m, std::size_t i) {
  const auto p = m.points[i];
  return (m.valid.empty() || m.valid[i]) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
const auto& backend() {
  return calibration::numeric::shared_backend(calibration::numeric::default_backend_kind());
}
struct NormalWorkspace {
  std::vector<double> weights, covariance, eigenvalues, eigenvectors, hessian, rhs, coefficients;
};
bool plane(const std::vector<P>& points, const std::vector<double>& weights, P& center, P& normal,
           NormalWorkspace& workspace) {
  if (points.size() < 3) return false;
  double sum = 0;
  center = {};
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double w = weights.empty() ? 1 : weights[i];
    center = add(center, mul(points[i], w));
    sum += w;
  }
  if (!(sum > 0)) return false;
  center = mul(center, 1 / sum);
  auto& cov = workspace.covariance;
  auto& ev = workspace.eigenvalues;
  auto& vec = workspace.eigenvectors;
  cov.assign(9, 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto p = sub(points[i], center);
    const double q[]{p.x, p.y, p.z}, w = weights.empty() ? 1 : weights[i];
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) cov[a * 3 + b] += w * q[a] * q[b];
  }
  if (!backend().symmetric_eigen(cov, 3, &ev, &vec) || !(ev[1] > 1e-12 * std::max(ev[2], 1e-30)))
    return false;
  normal = unit({vec[0], vec[3], vec[6]});
  return true;
}
P mls_normal(const std::vector<P>& points, const std::vector<double>& d2, P query,
             const NormalEstimationOptions& options, NormalWorkspace& workspace) {
  if (points.size() < 3) return {};
  const double sigma2 = options.absolute_sigma > 0
                            ? options.absolute_sigma * options.absolute_sigma
                            : options.relative_sigma * options.relative_sigma * d2[d2.size() / 2];
  if (!(sigma2 > 0) || !std::isfinite(sigma2)) return {};
  P center{}, normal{};
  // MLS 参考平面采用无权 k 邻居协方差；Gaussian 权重用于多项式，不用于初始 PCA。
  if (!plane(points, {}, center, normal, workspace)) return {};
  if (options.order >= 2) {
    // 一阶 MLS 只返回无权 PCA 法向，不计算不会被消费的 Gaussian 权重。
    auto& weights = workspace.weights;
    weights.resize(points.size());
    for (std::size_t j = 0; j < points.size(); ++j) weights[j] = std::exp(-d2[j] / sigma2);
    const P u = unit(cross(std::abs(normal.x) < .8 ? P{1, 0, 0} : P{0, 1, 0}, normal)), v = cross(normal, u);
    // 在以查询投影为中心的无量纲切平面拟合。
    const auto origin = sub(query, mul(normal, dot(sub(query, center), normal)));
    const double scale = std::sqrt(sigma2);
    const std::size_t count = options.order == 2 ? 6 : 10;
    auto& h = workspace.hessian;
    auto& rhs = workspace.rhs;
    auto& coefficients = workspace.coefficients;
    h.assign(count * count, 0);
    rhs.assign(count, 0);
    for (std::size_t j = 0; j < points.size(); ++j) {
      const auto p = sub(points[j], origin);
      const double x = dot(p, u) / scale, y = dot(p, v) / scale, z = dot(p, normal) / scale;
      const double a[]{1, x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y};
      for (std::size_t r = 0; r < count; ++r) {
        rhs[r] += weights[j] * a[r] * z;
        for (std::size_t c = 0; c < count; ++c) h[r * count + c] += weights[j] * a[r] * a[c];
      }
    }
    auto& eigenvalues = workspace.eigenvalues;
    auto& eigenvectors = workspace.eigenvectors;
    const bool full_rank = points.size() >= count &&
                           backend().symmetric_eigen(h, count, &eigenvalues, &eigenvectors) &&
                           eigenvalues.front() > eigenvalues.back() * 1e-10;
    if (full_rank && backend().solve_symmetric(h, rhs, count, &coefficients))
      normal = unit(sub(normal, add(mul(u, coefficients[1]), mul(v, coefficients[2]))));
  }
  if (options.force_inwards && dot(normal, query) > 0) normal = mul(normal, -1);
  return normal;
}
void bad(const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                        message);
}
}  // namespace
ObjectModel3DData estimate_normals(const ObjectModel3DData& model, std::string_view method,
                                   const NormalEstimationOptions& options) {
  validate_object_model_3d(model);
  ObjectModel3DData out = model;
  out.normals.assign(model.points.size(), {});
  // 每次调用独占工作区；逐点重置数值并保留容量，线程之间不共享可变缓冲。
  NormalWorkspace workspace;
  if (method == "mls") {
    if (!options.neighbors) bad("mls_kNN", "positive neighborhood required");
    if (options.order < 1 || options.order > 3) bad("mls_order", "polynomial order must be in [1,3]");
    if (!(options.relative_sigma > 0) || !std::isfinite(options.relative_sigma) ||
        options.absolute_sigma < 0 || !std::isfinite(options.absolute_sigma))
      bad("mls_relative_sigma", "positive finite sigma required");
    NeighborhoodIndex index(model);
    if (!index.size()) bad("ObjectModel3D", "point data required for MLS");
    NeighborhoodQueryScratch scratch;
    const std::size_t k = std::min(options.neighbors, index.size());
    if (k < 3) return out;
    std::vector<std::size_t> ids(k - 1);
    std::vector<double> distances(k - 1);
    std::vector<P> points;
    std::vector<double> d2;
    scratch.prepare(k - 1);
    points.reserve(k);
    d2.reserve(k);
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      if (!valid(model, i)) continue;
      const auto n = index.knn(i, k - 1, ids.data(), distances.data(), &scratch);
      points.clear();
      d2.clear();
      points.push_back(model.points[i]);
      d2.push_back(0);
      for (std::size_t j = 0; j < n; ++j) {
        points.push_back(model.points[ids[j]]);
        d2.push_back(distances[j]);
      }
      out.normals[i] = mls_normal(points, d2, model.points[i], options, workspace);
    }
  } else if (method == "xyz_mapping") {
    const auto width = model.mapping_width ? model.mapping_width : model.width,
               height = model.mapping_height ? model.mapping_height : model.height;
    if (!width || !height) bad("ObjectModel3D", "XYZ mapping required");
    constexpr auto missing = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> raster(width * height, missing);
    for (std::size_t i = 0; i < model.points.size(); ++i)
      if (valid(model, i)) {
        const auto slot = model.source_indices.empty() ? i : model.source_indices[i];
        if (slot >= raster.size()) bad("ObjectModel3D", "mapping index outside image");
        raster[slot] = i;
      }
    std::vector<P> points;
    points.reserve(121);
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      if (!valid(model, i)) continue;
      const auto slot = model.source_indices.empty() ? i : model.source_indices[i];
      const auto row = slot / width, col = slot % width;
      points.clear();
      for (std::size_t r = row > 5 ? row - 5 : 0; r <= std::min(row + 5, height - 1); ++r)
        for (std::size_t c = col > 5 ? col - 5 : 0; c <= std::min(col + 5, width - 1); ++c)
          if (raster[r * width + c] != missing) points.push_back(model.points[raster[r * width + c]]);
      P center{}, normal{};
      if (plane(points, {}, center, normal, workspace)) {
        // 使用映射切向保持表面方向，包含镜像传感器。
        P tangent_x{}, tangent_y{};
        for (std::size_t c = col + 1; c < width && c <= col + 5; ++c)
          if (raster[row * width + c] != missing) {
            tangent_x = sub(model.points[raster[row * width + c]], model.points[i]);
            break;
          }
        if (dot(tangent_x, tangent_x) == 0)
          for (std::size_t c = col; c > 0 && col - c < 5; --c)
            if (raster[row * width + c - 1] != missing) {
              tangent_x = sub(model.points[i], model.points[raster[row * width + c - 1]]);
              break;
            }
        for (std::size_t r = row + 1; r < height && r <= row + 5; ++r)
          if (raster[r * width + col] != missing) {
            tangent_y = sub(model.points[raster[r * width + col]], model.points[i]);
            break;
          }
        if (dot(tangent_y, tangent_y) == 0)
          for (std::size_t r = row; r > 0 && row - r < 5; --r)
            if (raster[(r - 1) * width + col] != missing) {
              tangent_y = sub(model.points[i], model.points[raster[(r - 1) * width + col]]);
              break;
            }
        if (dot(normal, cross(tangent_x, tangent_y)) < 0) normal = mul(normal, -1);
        out.normals[i] = normal;
      }
    }
  } else if (method == "triangles") {
    if (model.triangles.empty()) bad("ObjectModel3D", "triangles required");
    for (const auto& face : model.triangles) {
      if (!valid(model, face[0]) || !valid(model, face[1]) || !valid(model, face[2])) continue;
      const auto n = unit(cross(sub(model.points[face[1]], model.points[face[0]]),
                                sub(model.points[face[2]], model.points[face[0]])));
      for (int corner = 0; corner < 3; ++corner) {
        const auto a = unit(sub(model.points[face[(corner + 1) % 3]], model.points[face[corner]])),
                   b = unit(sub(model.points[face[(corner + 2) % 3]], model.points[face[corner]]));
        if (dot(a, a) > 0 && dot(b, b) > 0)
          out.normals[face[corner]] =
              add(out.normals[face[corner]], mul(n, std::acos(std::clamp(dot(a, b), -1., 1.))));
      }
    }
    for (auto& n : out.normals) n = unit(n);
  } else
    bad("Method", "unknown surface normal method");
  return out;
}
}  // namespace mv::reconstruction
