#include <algorithm>
#include <array>
#include <calibration/hom_mat3d.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <core/value.hpp>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <reconstruction/neighborhood_index.hpp>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/pair_registration.hpp>
#include <reconstruction/sampling.hpp>
#include <runtime/execution/execution_context.hpp>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace mv::reconstruction {
namespace {

constexpr double kEps = 1e-18;
constexpr std::size_t kObjectModelParallelWork = 4096;

[[noreturn]] void fail(core::Status status, const char* op, const char* message) {
  throw core::Exception(status, op, message);
}

void fail_value(const char* op, const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                        message);
}

bool point_valid(const ObjectModel3DData& model, std::size_t index) {
  return model.valid.empty() || model.valid[index] != 0U;
}

std::size_t valid_count(const ObjectModel3DData& model) {
  if (model.valid.empty()) return model.points.size();
  return static_cast<std::size_t>(
      std::count_if(model.valid.begin(), model.valid.end(), [](std::uint8_t v) { return v != 0U; }));
}

double length3(const Point3d& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

Point3d sub(const Point3d& a, const Point3d& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

double dot(const Point3d& a, const Point3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

double dist2(const Point3d& a, const Point3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

Point3d cross(const Point3d& a, const Point3d& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

void orient_normal(Point3d* n) {
  if (std::abs(n->z) > 1e-12) {
    if (n->z < 0.0) {
      n->x = -n->x;
      n->y = -n->y;
      n->z = -n->z;
    }
  } else if (std::abs(n->y) > 1e-12) {
    if (n->y < 0.0) {
      n->x = -n->x;
      n->y = -n->y;
      n->z = -n->z;
    }
  } else if (n->x < 0.0) {
    n->x = -n->x;
    n->y = -n->y;
    n->z = -n->z;
  }
}

bool normalize(Point3d* n) {
  const double len = length3(*n);
  if (!(len > 0.0) || !std::isfinite(len)) return false;
  n->x /= len;
  n->y /= len;
  n->z /= len;
  return true;
}

void jacobi_eigen3(double a[3][3], double eval[3], double vec[3][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) vec[i][j] = i == j ? 1.0 : 0.0;
  }
  for (int sweep = 0; sweep < 32; ++sweep) {
    int p = 0;
    int q = 1;
    double max_off = std::abs(a[0][1]);
    if (std::abs(a[0][2]) > max_off) {
      max_off = std::abs(a[0][2]);
      p = 0;
      q = 2;
    }
    if (std::abs(a[1][2]) > max_off) {
      max_off = std::abs(a[1][2]);
      p = 1;
      q = 2;
    }
    if (max_off < 1e-18) break;
    const double app = a[p][p];
    const double aqq = a[q][q];
    const double apq = a[p][q];
    const double tau = (aqq - app) / (2.0 * apq);
    const double t = std::copysign(1.0, tau) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;
    a[p][p] = app - t * apq;
    a[q][q] = aqq + t * apq;
    a[p][q] = 0.0;
    a[q][p] = 0.0;
    for (int r = 0; r < 3; ++r) {
      if (r == p || r == q) continue;
      const double arp = a[r][p];
      const double arq = a[r][q];
      a[r][p] = c * arp - s * arq;
      a[p][r] = a[r][p];
      a[r][q] = s * arp + c * arq;
      a[q][r] = a[r][q];
    }
    for (int r = 0; r < 3; ++r) {
      const double vrp = vec[r][p];
      const double vrq = vec[r][q];
      vec[r][p] = c * vrp - s * vrq;
      vec[r][q] = s * vrp + c * vrq;
    }
  }
  int order[3] = {0, 1, 2};
  eval[0] = a[0][0];
  eval[1] = a[1][1];
  eval[2] = a[2][2];
  if (eval[order[0]] > eval[order[1]]) std::swap(order[0], order[1]);
  if (eval[order[1]] > eval[order[2]]) std::swap(order[1], order[2]);
  if (eval[order[0]] > eval[order[1]]) std::swap(order[0], order[1]);
  double sorted_eval[3];
  double sorted_vec[3][3];
  for (int i = 0; i < 3; ++i) {
    sorted_eval[i] = eval[order[i]];
    for (int r = 0; r < 3; ++r) sorted_vec[r][i] = vec[r][order[i]];
  }
  for (int i = 0; i < 3; ++i) {
    eval[i] = sorted_eval[i];
    for (int r = 0; r < 3; ++r) vec[r][i] = sorted_vec[r][i];
  }
}

bool pca_normal_raw(const Point3d* pts, const double* weights, std::size_t count, Point3d* normal) {
  if (count < 3) return false;
  Point3d c{0, 0, 0};
  double wsum = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double w = weights == nullptr ? 1.0 : weights[i];
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    c.x += w * pts[i].x;
    c.y += w * pts[i].y;
    c.z += w * pts[i].z;
    wsum += w;
  }
  if (!(wsum > 0.0)) return false;
  c.x /= wsum;
  c.y /= wsum;
  c.z /= wsum;
  double cov[3][3] = {};
  for (std::size_t i = 0; i < count; ++i) {
    const double w = weights == nullptr ? 1.0 : weights[i];
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    const double dx = pts[i].x - c.x;
    const double dy = pts[i].y - c.y;
    const double dz = pts[i].z - c.z;
    cov[0][0] += w * dx * dx;
    cov[0][1] += w * dx * dy;
    cov[0][2] += w * dx * dz;
    cov[1][1] += w * dy * dy;
    cov[1][2] += w * dy * dz;
    cov[2][2] += w * dz * dz;
  }
  cov[1][0] = cov[0][1];
  cov[2][0] = cov[0][2];
  cov[2][1] = cov[1][2];
  double eval[3];
  double vec[3][3];
  jacobi_eigen3(cov, eval, vec);
  const double span = std::max(std::abs(eval[2]), 1e-30);
  if (!(eval[1] > 1e-12 * span)) return false;
  *normal = {vec[0][0], vec[1][0], vec[2][0]};
  return normalize(normal);
}

bool pca_normal(const Point3d* pts, std::size_t count, Point3d* normal) {
  if (!pca_normal_raw(pts, nullptr, count, normal)) return false;
  orient_normal(normal);
  return true;
}

enum class NormalOrientation { Base, Viewpoint, Direction, None };

void apply_normal_orientation(Point3d* n, const Point3d& point, NormalOrientation orientation,
                              const Point3d& ref) {
  if (orientation == NormalOrientation::None) return;
  if (orientation == NormalOrientation::Base) {
    orient_normal(n);
    return;
  }
  const Point3d dir = orientation == NormalOrientation::Viewpoint ? sub(ref, point) : ref;
  if (!(length3(dir) > 1e-18)) {
    orient_normal(n);
    return;
  }
  if (dot(*n, dir) < 0.0) {
    n->x = -n->x;
    n->y = -n->y;
    n->z = -n->z;
  }
}

bool plane_from_three(const Point3d& a, const Point3d& b, const Point3d& c, Point3d* n, double* d) {
  Point3d normal = cross(sub(b, a), sub(c, a));
  if (!normalize(&normal)) return false;
  *n = normal;
  *d = -dot(normal, a);
  return true;
}

bool pca_plane(std::span<const Point3d> pts, Point3d* normal, double* d) {
  if (!pca_normal(pts.data(), pts.size(), normal)) return false;
  Point3d c{0, 0, 0};
  for (const auto& p : pts) {
    c.x += p.x;
    c.y += p.y;
    c.z += p.z;
  }
  const double n = static_cast<double>(pts.size());
  c.x /= n;
  c.y /= n;
  c.z /= n;
  *d = -(normal->x * c.x + normal->y * c.y + normal->z * c.z);
  return true;
}

bool pca_plane_weighted(std::span<const Point3d> pts, std::span<const double> weights, Point3d* normal,
                        double* d) {
  if (!pca_normal_raw(pts.data(), weights.data(), pts.size(), normal)) return false;
  orient_normal(normal);
  Point3d c{0, 0, 0};
  double wsum = 0.0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double w = i < weights.size() ? weights[i] : 1.0;
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    c.x += w * pts[i].x;
    c.y += w * pts[i].y;
    c.z += w * pts[i].z;
    wsum += w;
  }
  if (!(wsum > 0.0)) return false;
  c.x /= wsum;
  c.y /= wsum;
  c.z /= wsum;
  *d = -(normal->x * c.x + normal->y * c.y + normal->z * c.z);
  return true;
}

Point3d transform_point(const std::array<double, 12>& m, const Point3d& p) {
  return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
          m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
}

Point3d transform_direction(const std::array<double, 12>& m, const Point3d& n) {
  return {m[0] * n.x + m[1] * n.y + m[2] * n.z, m[4] * n.x + m[5] * n.y + m[6] * n.z,
          m[8] * n.x + m[9] * n.y + m[10] * n.z};
}

std::array<double, 12> invert_rigid(const std::array<double, 12>& m) {
  std::array<double, 12> o{};
  o[0] = m[0];
  o[1] = m[4];
  o[2] = m[8];
  o[4] = m[1];
  o[5] = m[5];
  o[6] = m[9];
  o[8] = m[2];
  o[9] = m[6];
  o[10] = m[10];
  o[3] = -(o[0] * m[3] + o[1] * m[7] + o[2] * m[11]);
  o[7] = -(o[4] * m[3] + o[5] * m[7] + o[6] * m[11]);
  o[11] = -(o[8] * m[3] + o[9] * m[7] + o[10] * m[11]);
  return o;
}

double rigid_step_size(const std::array<double, 12>& inc) {
  const double trans = std::sqrt(inc[3] * inc[3] + inc[7] * inc[7] + inc[11] * inc[11]);
  const double c = std::clamp((inc[0] + inc[5] + inc[10] - 1.0) * 0.5, -1.0, 1.0);
  return trans + std::acos(c);
}

std::array<double, 12> multiply_mat(const std::array<double, 12>& a, const std::array<double, 12>& b) {
  std::array<double, 12> o{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      o[r * 4 + c] = a[r * 4 + 0] * b[c] + a[r * 4 + 1] * b[4 + c] + a[r * 4 + 2] * b[8 + c];
    }
    o[r * 4 + 3] = a[r * 4 + 0] * b[3] + a[r * 4 + 1] * b[7] + a[r * 4 + 2] * b[11] + a[r * 4 + 3];
  }
  return o;
}

bool kabsch(std::span<const Point3d> src, std::span<const Point3d> dst, const double* weights,
            std::array<double, 12>* matrix);
bool kabsch(std::span<const Point3d> src, std::span<const Point3d> dst, std::array<double, 12>* matrix);

core::Tuple pose_from_rigid(const std::array<double, 12>& matrix, const char* op) {
  try {
    return calibration::hom_mat3d_to_pose(calibration::hom_mat3d_from_values(matrix));
  } catch (const core::Exception&) {
    std::vector<Point3d> src{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};
    std::vector<Point3d> dst;
    dst.reserve(4);
    for (const auto& p : src) dst.push_back(transform_point(matrix, p));
    std::array<double, 12> projected{};
    if (!kabsch(src, dst, &projected)) {
      fail(core::Status::InvalidState, op, "registration pose is not rigid");
    }
    return calibration::hom_mat3d_to_pose(calibration::hom_mat3d_from_values(projected));
  }
}

struct IcpQuality {
  double rms{std::numeric_limits<double>::quiet_NaN()};
  double overlap{};
  std::size_t correspondences{};
  bool finite{false};
};

std::pmr::vector<std::size_t> collect_valid_indices(const ObjectModel3DData& model) {
  std::pmr::vector<std::size_t> ids{runtime::scratch_resource()};
  ids.reserve(model.points.size());
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (point_valid(model, i)) ids.push_back(i);
  }
  return ids;
}

struct CorrespondenceSlots {
  std::pmr::vector<std::size_t> ids{runtime::scratch_resource()};
  std::pmr::vector<std::uint8_t> hit{runtime::scratch_resource()};
  std::pmr::vector<std::size_t> neighbor{runtime::scratch_resource()};
  std::pmr::vector<double> d2{runtime::scratch_resource()};
  std::pmr::vector<Point3d> transformed{runtime::scratch_resource()};
};

CorrespondenceSlots query_correspondences(const ObjectModel3DData& source, const NeighborhoodIndex& index,
                                          const std::array<double, 12>& T, double max_d2) {
  CorrespondenceSlots slots;
  slots.ids = collect_valid_indices(source);
  const std::size_t n = slots.ids.size();
  slots.hit.assign(n, 0U);
  slots.neighbor.assign(n, 0);
  slots.d2.assign(n, 0.0);
  slots.transformed.resize(n);
  runtime::ExecutionContext::for_each_partition(n, n, kObjectModelParallelWork, [&](auto range) {
    for (std::size_t slot = range.begin; slot < range.end; ++slot) {
      const auto p = transform_point(T, source.points[slots.ids[slot]]);
      slots.transformed[slot] = p;
      std::size_t idx = 0;
      double d2 = 0.0;
      if (!index.nearest(p, max_d2, &idx, &d2) || !std::isfinite(d2)) continue;
      slots.hit[slot] = 1U;
      slots.neighbor[slot] = idx;
      slots.d2[slot] = d2;
    }
  });
  return slots;
}

struct GatheredCorrespondences {
  std::pmr::vector<Point3d> src{runtime::scratch_resource()};
  std::pmr::vector<Point3d> dst{runtime::scratch_resource()};
  std::pmr::vector<Point3d> nrm{runtime::scratch_resource()};
  std::pmr::vector<Point3d> src_n{runtime::scratch_resource()};
  std::pmr::vector<double> d2{runtime::scratch_resource()};
  std::pmr::vector<double> w{runtime::scratch_resource()};
  double rss = 0.0;
};

GatheredCorrespondences pack_correspondences(const CorrespondenceSlots& slots,
                                             const ObjectModel3DData& source, const ObjectModel3DData& target,
                                             bool transformed_src, bool store_normals,
                                             bool store_src_normals) {
  GatheredCorrespondences out;
  out.src.reserve(slots.ids.size());
  out.dst.reserve(slots.ids.size());
  out.d2.reserve(slots.ids.size());
  if (store_normals) out.nrm.reserve(slots.ids.size());
  if (store_src_normals) out.src_n.reserve(slots.ids.size());
  for (std::size_t slot = 0; slot < slots.ids.size(); ++slot) {
    if (slots.hit[slot] == 0U) continue;
    const std::size_t src_index = slots.ids[slot];
    const std::size_t dst_index = slots.neighbor[slot];
    out.src.push_back(transformed_src ? slots.transformed[slot] : source.points[src_index]);
    out.dst.push_back(target.points[dst_index]);
    out.d2.push_back(slots.d2[slot]);
    if (store_normals) out.nrm.push_back(target.normals[dst_index]);
    if (store_src_normals) out.src_n.push_back(source.normals[src_index]);
    out.rss += slots.d2[slot];
  }
  return out;
}

void compact_correspondences(GatheredCorrespondences* packed, std::span<const std::uint8_t> keep) {
  GatheredCorrespondences out;
  out.src.reserve(packed->src.size());
  out.dst.reserve(packed->dst.size());
  out.d2.reserve(packed->d2.size());
  if (!packed->nrm.empty()) out.nrm.reserve(packed->nrm.size());
  if (!packed->src_n.empty()) out.src_n.reserve(packed->src_n.size());
  if (!packed->w.empty()) out.w.reserve(packed->w.size());
  for (std::size_t i = 0; i < packed->src.size(); ++i) {
    if (keep[i] == 0U) continue;
    out.src.push_back(packed->src[i]);
    out.dst.push_back(packed->dst[i]);
    if (i < packed->d2.size()) {
      out.d2.push_back(packed->d2[i]);
      out.rss += packed->d2[i];
    }
    if (i < packed->nrm.size()) out.nrm.push_back(packed->nrm[i]);
    if (i < packed->src_n.size()) out.src_n.push_back(packed->src_n[i]);
    if (i < packed->w.size()) out.w.push_back(packed->w[i]);
  }
  *packed = std::move(out);
}

void trim_correspondences(GatheredCorrespondences* packed, double trim_ratio) {
  const std::size_t n = packed->src.size();
  if (!(trim_ratio < 1.0) || n <= 3) return;
  const auto keep_n =
      std::max(std::size_t{3}, static_cast<std::size_t>(std::ceil(trim_ratio * static_cast<double>(n))));
  if (keep_n >= n) return;
  std::pmr::vector<std::size_t> order(n, runtime::scratch_resource());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    if (packed->d2[a] < packed->d2[b]) return true;
    if (packed->d2[b] < packed->d2[a]) return false;
    return a < b;
  });
  std::pmr::vector<std::uint8_t> keep(n, 0U, runtime::scratch_resource());
  for (std::size_t i = 0; i < keep_n; ++i) keep[order[i]] = 1U;
  compact_correspondences(packed, keep);
}

bool zero_normal(const Point3d& n) { return !(length3(n) > 1e-12); }

IcpQuality evaluate_icp_quality(const ObjectModel3DData& source, const NeighborhoodIndex& index,
                                const std::array<double, 12>& T, double max_d2) {
  const auto slots = query_correspondences(source, index, T, max_d2);
  double rss = 0.0;
  std::size_t n = 0;
  for (std::size_t slot = 0; slot < slots.ids.size(); ++slot) {
    if (slots.hit[slot] == 0U) continue;
    rss += slots.d2[slot];
    ++n;
  }
  IcpQuality quality;
  quality.correspondences = n;
  quality.overlap = slots.ids.empty() ? 0.0 : static_cast<double>(n) / static_cast<double>(slots.ids.size());
  quality.finite = n > 0 && std::isfinite(rss);
  if (quality.finite) quality.rms = std::sqrt(rss / static_cast<double>(n));
  return quality;
}

bool solve6(double a[6][6], double b[6], double x[6]) {
  double m[6][7];
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) m[i][j] = a[i][j];
    m[i][6] = b[i];
  }
  for (int col = 0; col < 6; ++col) {
    int pivot = col;
    double best = std::abs(m[col][col]);
    for (int row = col + 1; row < 6; ++row) {
      const double v = std::abs(m[row][col]);
      if (v > best) {
        best = v;
        pivot = row;
      }
    }
    if (best < 1e-14) return false;
    if (pivot != col) {
      for (int j = col; j < 7; ++j) std::swap(m[col][j], m[pivot][j]);
    }
    const double diag = m[col][col];
    for (int j = col; j < 7; ++j) m[col][j] /= diag;
    for (int row = 0; row < 6; ++row) {
      if (row == col) continue;
      const double f = m[row][col];
      for (int j = col; j < 7; ++j) m[row][j] -= f * m[col][j];
    }
  }
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(m[i][6])) return false;
    x[i] = m[i][6];
  }
  return true;
}

bool kabsch(std::span<const Point3d> src, std::span<const Point3d> dst, const double* weights,
            std::array<double, 12>* matrix) {
  if (src.size() < 3 || src.size() != dst.size()) return false;
  Point3d cs{0, 0, 0};
  Point3d ct{0, 0, 0};
  double wsum = 0.0;
  std::size_t positive_weights = 0;
  for (std::size_t i = 0; i < src.size(); ++i) {
    const double w = weights == nullptr ? 1.0 : weights[i];
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    ++positive_weights;
    cs.x += w * src[i].x;
    cs.y += w * src[i].y;
    cs.z += w * src[i].z;
    ct.x += w * dst[i].x;
    ct.y += w * dst[i].y;
    ct.z += w * dst[i].z;
    wsum += w;
  }
  if (!(wsum > 0.0) || positive_weights < 3) return false;
  cs.x /= wsum;
  cs.y /= wsum;
  cs.z /= wsum;
  ct.x /= wsum;
  ct.y /= wsum;
  ct.z /= wsum;
  double h[3][3] = {};
  for (std::size_t i = 0; i < src.size(); ++i) {
    const double w = weights == nullptr ? 1.0 : weights[i];
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    const double sx = src[i].x - cs.x;
    const double sy = src[i].y - cs.y;
    const double sz = src[i].z - cs.z;
    const double dx = dst[i].x - ct.x;
    const double dy = dst[i].y - ct.y;
    const double dz = dst[i].z - ct.z;
    h[0][0] += w * sx * dx;
    h[0][1] += w * sx * dy;
    h[0][2] += w * sx * dz;
    h[1][0] += w * sy * dx;
    h[1][1] += w * sy * dy;
    h[1][2] += w * sy * dz;
    h[2][0] += w * sz * dx;
    h[2][1] += w * sz * dy;
    h[2][2] += w * sz * dz;
  }
  double ata[3][3] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ata[i][j] = h[0][i] * h[0][j] + h[1][i] * h[1][j] + h[2][i] * h[2][j];
    }
  }
  double eval[3];
  double v[3][3];
  jacobi_eigen3(ata, eval, v);
  double u[3][3] = {};
  int rank = 0;
  for (int c = 2; c >= 0; --c) {
    const double s = eval[c] > 0.0 ? std::sqrt(eval[c]) : 0.0;
    if (s < 1e-12) continue;
    ++rank;
    for (int r = 0; r < 3; ++r) {
      u[r][c] = (h[r][0] * v[0][c] + h[r][1] * v[1][c] + h[r][2] * v[2][c]) / s;
    }
    Point3d col{u[0][c], u[1][c], u[2][c]};
    if (!normalize(&col)) return false;
    u[0][c] = col.x;
    u[1][c] = col.y;
    u[2][c] = col.z;
  }
  if (rank == 0) return false;
  auto set_col = [](double m[3][3], int c, const Point3d& p) {
    m[0][c] = p.x;
    m[1][c] = p.y;
    m[2][c] = p.z;
  };
  auto get_col = [](const double m[3][3], int c) { return Point3d{m[0][c], m[1][c], m[2][c]}; };
  if (rank == 1) {
    Point3d a = get_col(u, 2);
    Point3d axis = std::abs(a.x) < 0.9 ? Point3d{1, 0, 0} : Point3d{0, 1, 0};
    Point3d b = cross(a, axis);
    if (!normalize(&b)) return false;
    set_col(u, 1, b);
    Point3d va = get_col(v, 2);
    Point3d vb = cross(va, axis);
    if (!normalize(&vb)) return false;
    set_col(v, 1, vb);
    rank = 2;
  }
  if (rank < 3) {
    Point3d u0 = cross(get_col(u, 1), get_col(u, 2));
    Point3d v0 = cross(get_col(v, 1), get_col(v, 2));
    if (!normalize(&u0) || !normalize(&v0)) return false;
    set_col(u, 0, u0);
    set_col(v, 0, v0);
  }
  double r[3][3] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r[i][j] = v[i][0] * u[j][0] + v[i][1] * u[j][1] + v[i][2] * u[j][2];
    }
  }
  const double det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                     r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                     r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  if (det < 0.0) {
    for (int i = 0; i < 3; ++i) v[i][0] = -v[i][0];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r[i][j] = v[i][0] * u[j][0] + v[i][1] * u[j][1] + v[i][2] * u[j][2];
      }
    }
  }
  const Point3d t{ct.x - (r[0][0] * cs.x + r[0][1] * cs.y + r[0][2] * cs.z),
                  ct.y - (r[1][0] * cs.x + r[1][1] * cs.y + r[1][2] * cs.z),
                  ct.z - (r[2][0] * cs.x + r[2][1] * cs.y + r[2][2] * cs.z)};
  *matrix = {r[0][0], r[0][1], r[0][2], t.x, r[1][0], r[1][1], r[1][2], t.y, r[2][0], r[2][1], r[2][2], t.z};
  return std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z);
}

bool kabsch(std::span<const Point3d> src, std::span<const Point3d> dst, std::array<double, 12>* matrix) {
  return kabsch(src, dst, nullptr, matrix);
}

std::array<double, 12> rodrigues(const double w[3], const double t[3]) {
  const double theta = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  if (theta > 1e-12) {
    const double wx = w[0] / theta;
    const double wy = w[1] / theta;
    const double wz = w[2] / theta;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double ic = 1.0 - c;
    r[0][0] = c + wx * wx * ic;
    r[0][1] = wx * wy * ic - wz * s;
    r[0][2] = wx * wz * ic + wy * s;
    r[1][0] = wy * wx * ic + wz * s;
    r[1][1] = c + wy * wy * ic;
    r[1][2] = wy * wz * ic - wx * s;
    r[2][0] = wz * wx * ic - wy * s;
    r[2][1] = wz * wy * ic + wx * s;
    r[2][2] = c + wz * wz * ic;
  } else {
    r[0][1] = -w[2];
    r[0][2] = w[1];
    r[1][0] = w[2];
    r[1][2] = -w[0];
    r[2][0] = -w[1];
    r[2][1] = w[0];
  }
  return {r[0][0], r[0][1], r[0][2], t[0], r[1][0], r[1][1], r[1][2], t[1], r[2][0], r[2][1], r[2][2], t[2]};
}

bool point_to_plane_step(std::span<const Point3d> src, std::span<const Point3d> dst,
                         std::span<const Point3d> nrm, const double* weights, std::array<double, 12>* inc) {
  double a[6][6] = {};
  double b[6] = {};
  for (std::size_t i = 0; i < src.size(); ++i) {
    const double w = weights == nullptr ? 1.0 : weights[i];
    if (!(w > 0.0) || !std::isfinite(w)) continue;
    const auto& p = src[i];
    const auto& q = dst[i];
    const auto& n = nrm[i];
    if (zero_normal(n)) continue;
    const double row[6] = {
        p.y * n.z - p.z * n.y, p.z * n.x - p.x * n.z, p.x * n.y - p.y * n.x, n.x, n.y, n.z};
    const double rhs = n.x * (q.x - p.x) + n.y * (q.y - p.y) + n.z * (q.z - p.z);
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) a[r][c] += w * row[r] * row[c];
      b[r] += w * row[r] * rhs;
    }
  }
  double x[6];
  if (!solve6(a, b, x)) return false;
  *inc = rodrigues(x, x + 3);
  return true;
}

constexpr double kPi = 3.14159265358979323846;

void require_matching_gen_params(const core::Tuple& names, const core::Tuple& values, const char* op) {
  if (names.size() != values.size()) {
    fail(core::Status::InvalidArgument, op, "generic parameter tuple sizes must match");
  }
}

bool parse_boolean_gen(const core::Value& value, const char* op, const char* parameter) {
  if (value.kind() != core::ValueKind::String) {
    fail_value(op, parameter, "boolean generic parameters must be 'true' or 'false'");
  }
  const std::string text(value.as_string().view());
  if (text == "true") return true;
  if (text == "false") return false;
  fail_value(op, parameter, "boolean generic parameters must be 'true' or 'false'");
  return false;
}

double parse_real_gen(const core::Value& value, const char* op, const char* parameter) {
  try {
    const double number = value.as_real64();
    if (!std::isfinite(number)) fail_value(op, parameter, "value must be finite");
    return number;
  } catch (const core::Exception& error) {
    if (error.status() == core::Status::TypeMismatch) {
      fail_value(op, parameter, "value must be numeric");
    }
    throw;
  }
}

std::int64_t parse_int_gen(const core::Value& value, const char* op, const char* parameter) {
  try {
    return value.as_int64();
  } catch (const core::Exception& error) {
    if (error.status() == core::Status::TypeMismatch) {
      fail_value(op, parameter, "value must be an integer");
    }
    throw;
  }
}

void mark_seen(bool* seen, const char* op, const char* parameter) {
  if (*seen) fail_value(op, parameter, "generic parameter specified more than once");
  *seen = true;
}

struct PlaneGenOptions {
  std::uint64_t seed = 1;
  std::int64_t iterations = 200;
  bool irls = false;
  double min_inlier_ratio = 0.0;
};

PlaneGenOptions parse_plane_gen(const core::Tuple& names, const core::Tuple& values, const char* op,
                                bool ransac) {
  require_matching_gen_params(names, values, op);
  PlaneGenOptions options;
  bool seen_seed = false;
  bool seen_iterations = false;
  bool seen_irls = false;
  bool seen_ratio = false;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string name(names.string_view(i));
    if (!ransac) fail_value(op, "GenParamName", "Type plane does not accept generic parameters");
    const auto& value = values.at(i);
    if (name == "seed") {
      mark_seen(&seen_seed, op, "seed");
      const auto seed = parse_int_gen(value, op, "seed");
      if (seed < 0) fail_value(op, "seed", "seed must be >= 0");
      options.seed = static_cast<std::uint64_t>(seed);
    } else if (name == "iterations") {
      mark_seen(&seen_iterations, op, "iterations");
      options.iterations = parse_int_gen(value, op, "iterations");
      if (options.iterations < 1) fail_value(op, "iterations", "iterations must be >= 1");
    } else if (name == "irls") {
      mark_seen(&seen_irls, op, "irls");
      options.irls = parse_boolean_gen(value, op, "irls");
    } else if (name == "min_inlier_ratio") {
      mark_seen(&seen_ratio, op, "min_inlier_ratio");
      options.min_inlier_ratio = parse_real_gen(value, op, "min_inlier_ratio");
      if (options.min_inlier_ratio < 0.0 || options.min_inlier_ratio > 1.0) {
        fail_value(op, "min_inlier_ratio", "min_inlier_ratio must be in [0, 1]");
      }
    } else {
      fail_value(op, "GenParamName", "unknown generic parameter");
    }
  }
  return options;
}

struct NormalGenOptions {
  bool have_radius = false;
  double radius = 0.0;
  NormalOrientation orientation = NormalOrientation::Base;
  Point3d viewpoint{};
  Point3d direction{};
  bool have_viewpoint[3]{};
  bool have_direction[3]{};
};

NormalGenOptions parse_normal_gen(const core::Tuple& names, const core::Tuple& values, const char* op,
                                  bool radius_method) {
  require_matching_gen_params(names, values, op);
  NormalGenOptions options;
  bool seen_radius = false;
  bool seen_orientation = false;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string name(names.string_view(i));
    const auto& value = values.at(i);
    if (name == "radius") {
      mark_seen(&seen_radius, op, "radius");
      options.have_radius = true;
      options.radius = parse_real_gen(value, op, "radius");
      if (!(options.radius > 0.0)) fail_value(op, "radius", "radius must be finite and positive");
    } else if (name == "orientation") {
      mark_seen(&seen_orientation, op, "orientation");
      if (value.kind() != core::ValueKind::String) {
        fail_value(op, "orientation", "orientation must be a string");
      }
      const std::string text(value.as_string().view());
      if (text == "base")
        options.orientation = NormalOrientation::Base;
      else if (text == "viewpoint")
        options.orientation = NormalOrientation::Viewpoint;
      else if (text == "direction")
        options.orientation = NormalOrientation::Direction;
      else if (text == "none")
        options.orientation = NormalOrientation::None;
      else
        fail_value(op, "orientation", "orientation must be base, viewpoint, direction, or none");
    } else if (name == "viewpoint_x" || name == "viewpoint_y" || name == "viewpoint_z") {
      const int axis = name == "viewpoint_x" ? 0 : name == "viewpoint_y" ? 1 : 2;
      mark_seen(&options.have_viewpoint[axis], op, name.c_str());
      const double number = parse_real_gen(value, op, name.c_str());
      if (axis == 0)
        options.viewpoint.x = number;
      else if (axis == 1)
        options.viewpoint.y = number;
      else
        options.viewpoint.z = number;
    } else if (name == "direction_x" || name == "direction_y" || name == "direction_z") {
      const int axis = name == "direction_x" ? 0 : name == "direction_y" ? 1 : 2;
      mark_seen(&options.have_direction[axis], op, name.c_str());
      const double number = parse_real_gen(value, op, name.c_str());
      if (axis == 0)
        options.direction.x = number;
      else if (axis == 1)
        options.direction.y = number;
      else
        options.direction.z = number;
    } else {
      fail_value(op, "GenParamName", "unknown generic parameter");
    }
  }
  if (radius_method) {
    if (!options.have_radius) fail_value(op, "radius", "Method radius requires GenParam radius");
  } else if (options.have_radius) {
    fail_value(op, "radius", "radius is only valid for Method radius");
  }
  const bool any_viewpoint =
      options.have_viewpoint[0] || options.have_viewpoint[1] || options.have_viewpoint[2];
  const bool any_direction =
      options.have_direction[0] || options.have_direction[1] || options.have_direction[2];
  if (options.orientation == NormalOrientation::Viewpoint) {
    if (!(options.have_viewpoint[0] && options.have_viewpoint[1] && options.have_viewpoint[2])) {
      fail_value(op, "viewpoint", "orientation viewpoint requires viewpoint_x, viewpoint_y, viewpoint_z");
    }
  } else if (any_viewpoint) {
    fail_value(op, "viewpoint", "viewpoint_* is only valid for orientation viewpoint");
  }
  if (options.orientation == NormalOrientation::Direction) {
    if (!(options.have_direction[0] && options.have_direction[1] && options.have_direction[2])) {
      fail_value(op, "direction", "orientation direction requires direction_x, direction_y, direction_z");
    }
    if (!normalize(&options.direction)) {
      fail_value(op, "direction", "direction must be a non-zero vector");
    }
  } else if (any_direction) {
    fail_value(op, "direction", "direction_* is only valid for orientation direction");
  }
  return options;
}

IcpOptions parse_icp_gen(const core::Tuple& names, const core::Tuple& values, const char* op) {
  require_matching_gen_params(names, values, op);
  IcpOptions options;
  bool seen_trim = false;
  bool seen_angle = false;
  bool seen_weighting = false;
  bool seen_clip = false;
  bool seen_delta = false;
  bool seen_coarse = false;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string name(names.string_view(i));
    const auto& value = values.at(i);
    if (name == "trim_ratio") {
      mark_seen(&seen_trim, op, "trim_ratio");
      options.trim_ratio = parse_real_gen(value, op, "trim_ratio");
      if (!(options.trim_ratio > 0.0) || options.trim_ratio > 1.0) {
        fail_value(op, "trim_ratio", "trim_ratio must be in (0, 1]");
      }
    } else if (name == "max_normal_angle") {
      mark_seen(&seen_angle, op, "max_normal_angle");
      options.max_normal_angle = parse_real_gen(value, op, "max_normal_angle");
      if (options.max_normal_angle < 0.0 || options.max_normal_angle > kPi) {
        fail_value(op, "max_normal_angle", "max_normal_angle must be in [0, pi]");
      }
    } else if (name == "weighting") {
      mark_seen(&seen_weighting, op, "weighting");
      if (value.kind() != core::ValueKind::String) {
        fail_value(op, "weighting", "weighting must be a string");
      }
      const std::string text(value.as_string().view());
      if (text == "none")
        options.weighting = IcpWeighting::None;
      else if (text == "huber")
        options.weighting = IcpWeighting::Huber;
      else if (text == "tukey")
        options.weighting = IcpWeighting::Tukey;
      else
        fail_value(op, "weighting", "weighting must be none, huber, or tukey");
    } else if (name == "clipping_factor") {
      mark_seen(&seen_clip, op, "clipping_factor");
      options.clipping_factor = parse_real_gen(value, op, "clipping_factor");
      if (!(options.clipping_factor > 0.0)) {
        fail_value(op, "clipping_factor", "clipping_factor must be finite and positive");
      }
    } else if (name == "pose_delta") {
      mark_seen(&seen_delta, op, "pose_delta");
      options.pose_delta = parse_real_gen(value, op, "pose_delta");
      if (options.pose_delta < 0.0) fail_value(op, "pose_delta", "pose_delta must be >= 0");
    } else if (name == "coarse_seed") {
      mark_seen(&seen_coarse, op, "coarse_seed");
      const double enabled = parse_real_gen(value, op, "coarse_seed");
      if (enabled != 0.0 && enabled != 1.0) fail_value(op, "coarse_seed", "coarse_seed must be 0 or 1");
      options.coarse_seed = enabled == 1.0;
    } else {
      fail_value(op, "GenParamName", "unknown generic parameter");
    }
  }
  return options;
}

struct Lcg {
  std::uint64_t state;
  explicit Lcg(std::uint64_t seed) : state(seed == 0 ? 1 : seed) {}
  std::uint64_t next() {
    state = state * 6364136223846793005ULL + 1ULL;
    return state;
  }
  std::size_t index(std::size_t n) { return static_cast<std::size_t>(next() >> 33) % n; }
};

// 最近邻 ICP 超捕获范围时，用刚体不变局部距离签名提供种子。
// 签名匹配为二次复杂度且要求重叠几何邻域，因此此有界路径须显式启用。
std::optional<std::array<double, 12>> fingerprint_seed(const ObjectModel3DData& source,
                                                       const ObjectModel3DData& target,
                                                       const std::array<double, 12>& initial,
                                                       double max_distance) {
  const auto source_ids = collect_valid_indices(source);
  const auto target_ids = collect_valid_indices(target);
  if (source_ids.size() < 8 || target_ids.size() < 8 || source_ids.size() > 2048 || target_ids.size() > 2048)
    return std::nullopt;
  constexpr std::size_t kNeighbors = 5;
  using Signature = std::array<double, kNeighbors>;
  auto signatures = [](const ObjectModel3DData& model, const std::pmr::vector<std::size_t>& ids) {
    NeighborhoodIndex index(model, runtime::scratch_resource());
    NeighborhoodQueryScratch scratch(runtime::scratch_resource());
    std::array<std::size_t, kNeighbors> neighbors{};
    std::array<double, kNeighbors> distances{};
    std::vector<Signature> out(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto found = index.knn(ids[i], kNeighbors, neighbors.data(), distances.data(), &scratch);
      if (found != kNeighbors) return std::vector<Signature>{};
      for (std::size_t k = 0; k < kNeighbors; ++k) out[i][k] = std::sqrt(distances[k]);
    }
    return out;
  };
  const auto ss = signatures(source, source_ids), ts = signatures(target, target_ids);
  if (ss.empty() || ts.empty()) return std::nullopt;
  auto signature_cost = [](const Signature& a, const Signature& b) {
    double sum = 0;
    for (std::size_t k = 0; k < kNeighbors; ++k) {
      const double d = a[k] - b[k];
      sum += d * d;
    }
    return sum;
  };
  std::vector<std::size_t> forward(ss.size()), backward(ts.size());
  for (std::size_t i = 0; i < ss.size(); ++i) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < ts.size(); ++j) {
      const double cost = signature_cost(ss[i], ts[j]);
      if (cost < best) {
        best = cost;
        forward[i] = j;
      }
    }
  }
  for (std::size_t j = 0; j < ts.size(); ++j) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < ss.size(); ++i) {
      const double cost = signature_cost(ss[i], ts[j]);
      if (cost < best) {
        best = cost;
        backward[j] = i;
      }
    }
  }
  std::vector<std::pair<Point3d, Point3d>> matches;
  for (std::size_t i = 0; i < ss.size(); ++i) {
    const auto j = forward[i];
    if (backward[j] == i) matches.emplace_back(source.points[source_ids[i]], target.points[target_ids[j]]);
  }
  const std::size_t required = std::max<std::size_t>(6, std::min(ss.size(), ts.size()) / 8);
  if (matches.size() < required) return std::nullopt;
  std::vector<double> neighbor_scale;
  neighbor_scale.reserve(ts.size());
  for (const auto& sig : ts) neighbor_scale.push_back(sig[0]);
  std::nth_element(neighbor_scale.begin(), neighbor_scale.begin() + neighbor_scale.size() / 2,
                   neighbor_scale.end());
  const double threshold = std::min(max_distance * .1, neighbor_scale[neighbor_scale.size() / 2] * .4);
  if (!(threshold > 0)) return std::nullopt;
  const double threshold2 = threshold * threshold;
  auto support = [&](const std::array<double, 12>& transform) {
    std::size_t count = 0;
    for (const auto& [a, b] : matches) count += dist2(transform_point(transform, a), b) <= threshold2;
    return count;
  };
  const std::size_t initial_support = support(initial);
  std::size_t best_support = 0;
  std::array<double, 12> best{};
  Lcg random(0x8e29c6d5ULL);
  for (int trial = 0; trial < 192; ++trial) {
    const auto a = random.index(matches.size());
    auto b = random.index(matches.size()), c = random.index(matches.size());
    if (a == b || a == c || b == c) continue;
    std::array<Point3d, 3> from{matches[a].first, matches[b].first, matches[c].first};
    std::array<Point3d, 3> to{matches[a].second, matches[b].second, matches[c].second};
    std::array<double, 12> candidate{};
    if (!kabsch(from, to, &candidate)) continue;
    const auto count = support(candidate);
    if (count > best_support) {
      best_support = count;
      best = candidate;
    }
  }
  if (best_support < required || best_support <= initial_support + 2) return std::nullopt;
  std::vector<Point3d> from, to;
  from.reserve(best_support);
  to.reserve(best_support);
  for (const auto& [a, b] : matches)
    if (dist2(transform_point(best, a), b) <= threshold2) {
      from.push_back(a);
      to.push_back(b);
    }
  std::array<double, 12> refined{};
  if (kabsch(from, to, &refined) && support(refined) >= best_support) return refined;
  return best;
}

double huber_weight(double residual, double scale) {
  const double abs_r = std::abs(residual);
  if (!(scale > 0.0) || abs_r <= scale) return 1.0;
  return scale / abs_r;
}

double robust_weight(double residual, IcpWeighting weighting, double scale) {
  const double abs_r = std::abs(residual);
  if (weighting == IcpWeighting::None || !(scale > 0.0)) return 1.0;
  if (weighting == IcpWeighting::Huber) return huber_weight(residual, scale);
  if (!(abs_r < scale)) return 0.0;
  const double u = abs_r / scale;
  const double t = 1.0 - u * u;
  return t * t;
}

void assign_icp_weights(GatheredCorrespondences* packed, bool point_to_plane, IcpWeighting weighting,
                        double scale) {
  packed->w.assign(packed->src.size(), 1.0);
  if (weighting == IcpWeighting::None) return;
  for (std::size_t i = 0; i < packed->src.size(); ++i) {
    double residual = 0.0;
    if (point_to_plane && i < packed->nrm.size()) {
      const auto& n = packed->nrm[i];
      residual = n.x * (packed->dst[i].x - packed->src[i].x) + n.y * (packed->dst[i].y - packed->src[i].y) +
                 n.z * (packed->dst[i].z - packed->src[i].z);
    } else {
      residual = std::sqrt(packed->d2[i]);
    }
    packed->w[i] = robust_weight(residual, weighting, scale);
  }
}

bool fit_plane_ransac(const ObjectModel3DData& model, double max_distance, const PlaneGenOptions& options,
                      Point3d* n, double* d, std::pmr::vector<std::uint8_t>* keep, std::size_t* inliers,
                      double* rss, const char* op) {
  const auto ids = collect_valid_indices(model);
  if (ids.size() < 3) fail(core::Status::InvalidState, op, "plane fit needs at least three valid points");
  std::vector<Point3d> pts;
  pts.reserve(ids.size());
  for (std::size_t id : ids) pts.push_back(model.points[id]);

  Lcg rng(options.seed);
  std::size_t best_count = 0;
  double best_rss = std::numeric_limits<double>::infinity();
  Point3d best_n{};
  double best_d = 0.0;
  bool found = false;
  for (std::int64_t iter = 0; iter < options.iterations; ++iter) {
    std::size_t a = rng.index(pts.size());
    std::size_t b = rng.index(pts.size());
    std::size_t c = rng.index(pts.size());
    int tries = 0;
    while ((a == b || a == c || b == c) && tries < 32) {
      b = rng.index(pts.size());
      c = rng.index(pts.size());
      ++tries;
    }
    if (a == b || a == c || b == c) continue;
    Point3d sample_n{};
    double sample_d = 0.0;
    if (!plane_from_three(pts[a], pts[b], pts[c], &sample_n, &sample_d)) continue;
    std::size_t count = 0;
    double sample_rss = 0.0;
    for (const auto& p : pts) {
      const double res = sample_n.x * p.x + sample_n.y * p.y + sample_n.z * p.z + sample_d;
      if (std::abs(res) <= max_distance) {
        sample_rss += res * res;
        ++count;
      }
    }
    if (count < 3) continue;
    if (!found || count > best_count || (count == best_count && sample_rss < best_rss)) {
      found = true;
      best_count = count;
      best_rss = sample_rss;
      best_n = sample_n;
      best_d = sample_d;
    }
  }
  if (!found) fail(core::Status::InvalidState, op, "plane RANSAC found no inlier set");

  auto collect = [&](const Point3d& nn, double dd, std::vector<Point3d>* inlier_pts) {
    inlier_pts->clear();
    keep->assign(model.points.size(), 0U);
    std::size_t count = 0;
    double sum = 0.0;
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      if (!point_valid(model, i)) continue;
      const double res = nn.x * model.points[i].x + nn.y * model.points[i].y + nn.z * model.points[i].z + dd;
      if (std::abs(res) <= max_distance) {
        (*keep)[i] = 1U;
        inlier_pts->push_back(model.points[i]);
        sum += res * res;
        ++count;
      }
    }
    *inliers = count;
    *rss = sum;
  };

  std::vector<Point3d> inlier_pts;
  collect(best_n, best_d, &inlier_pts);
  if (inlier_pts.size() < 3)
    fail(core::Status::InvalidState, op, "plane fit produced fewer than three inliers");
  if (!pca_plane(inlier_pts, n, d))
    fail(core::Status::InvalidState, op, "inliers are degenerate for a plane");
  collect(*n, *d, &inlier_pts);
  if (inlier_pts.size() < 3)
    fail(core::Status::InvalidState, op, "plane fit produced fewer than three inliers");

  if (options.irls) {
    std::vector<double> weights(inlier_pts.size(), 1.0);
    for (int it = 0; it < 8; ++it) {
      if (!pca_plane_weighted(inlier_pts, weights, n, d)) {
        fail(core::Status::InvalidState, op, "inliers are degenerate for a plane");
      }
      for (std::size_t i = 0; i < inlier_pts.size(); ++i) {
        const double res = n->x * inlier_pts[i].x + n->y * inlier_pts[i].y + n->z * inlier_pts[i].z + *d;
        weights[i] = huber_weight(res, max_distance);
      }
    }
    collect(*n, *d, &inlier_pts);
    if (inlier_pts.size() < 3)
      fail(core::Status::InvalidState, op, "plane fit produced fewer than three inliers");
    // 保留 IRLS 的加权结果，只更新最终内点与统计，不能再用无权 PCA 覆盖。
  }

  const auto required = std::max(
      std::size_t{3},
      static_cast<std::size_t>(std::ceil(options.min_inlier_ratio * static_cast<double>(ids.size()))));
  if (*inliers < required)
    fail(core::Status::InvalidState, op, "plane RANSAC inlier ratio is below min_inlier_ratio");
  return true;
}

}  // namespace

ObjectModel3DData filter_object_model_3d(const ObjectModel3DData& model, std::string_view method,
                                         std::int64_t num_neighbors, double std_mul,
                                         const NeighborhoodIndex* shared_index) {
  constexpr auto op = "filter_object_model_3d";
  validate_object_model_3d(model);
  if (method != "statistical") fail_value(op, "Method", "v1 supports only statistical outlier removal");
  if (num_neighbors < 1) fail_value(op, "NumNeighbors", "NumNeighbors must be >= 1");
  if (!std::isfinite(std_mul) || std_mul < 0.0) {
    fail_value(op, "StdDevMul", "StdDevMul must be finite and >= 0");
  }
  const auto k = static_cast<std::size_t>(num_neighbors);
  const std::size_t nvalid = valid_count(model);
  if (nvalid == 0) return {};
  if (nvalid < k + 1) {
    fail(core::Status::InvalidArgument, op, "not enough valid points for the requested neighborhood");
  }

  runtime::ExecutionContext context;
  std::optional<NeighborhoodIndex> owned_index;
  if (shared_index == nullptr) {
    owned_index.emplace(model, context.scratch_resource());
    shared_index = &*owned_index;
  }
  const auto& index = *shared_index;
  std::vector<double> means(model.points.size(), 0.0);
  std::vector<std::uint8_t> eligible(model.points.size(), 0U);
  runtime::ExecutionContext::for_each_partition(
      model.points.size(), runtime::ExecutionContext::saturated_work(model.points.size(), k),
      kObjectModelParallelWork, [&](auto range) {
        runtime::ExecutionContext local;
        NeighborhoodQueryScratch knn_scratch(local.scratch_resource());
        knn_scratch.prepare(k);
        std::vector<std::size_t> nn(k);
        std::vector<double> d2(k);
        for (std::size_t i = range.begin; i < range.end; ++i) {
          if (!point_valid(model, i)) continue;
          if (index.knn(i, k, nn.data(), d2.data(), &knn_scratch) < k) continue;
          double sum = 0.0;
          for (std::size_t j = 0; j < k; ++j) sum += std::sqrt(d2[j]);
          means[i] = sum / static_cast<double>(k);
          eligible[i] = 1U;
        }
      });
  std::vector<double> samples;
  samples.reserve(nvalid);
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (eligible[i] != 0U) samples.push_back(means[i]);
  }
  if (samples.empty()) return {};
  const double mu =
      std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
  double var = 0.0;
  for (double v : samples) var += (v - mu) * (v - mu);
  const double denom = samples.size() > 1 ? static_cast<double>(samples.size() - 1) : 1.0;
  const double sigma = std::sqrt(var / denom);
  const double limit = mu + std_mul * sigma;
  std::pmr::vector<std::uint8_t> keep(model.points.size(), 0U, context.scratch_resource());
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (eligible[i] != 0U && means[i] <= limit) keep[i] = 1U;
  }
  auto out = compact_object_model_3d(model, keep.data());
  validate_object_model_3d(out);
  return out;
}

ObjectModel3DData surface_normals_object_model_3d(const ObjectModel3DData& model, std::string_view method,
                                                  std::int64_t num_neighbors,
                                                  const core::Tuple& gen_param_name,
                                                  const core::Tuple& gen_param_value,
                                                  const NeighborhoodIndex* shared_index) {
  constexpr auto op = "surface_normals_object_model_3d";
  validate_object_model_3d(model);
  const bool knn = method == "knn";
  const bool radius = method == "radius";
  if (!knn && !radius) fail_value(op, "Method", "v1 supports knn or radius normals");
  const auto options = parse_normal_gen(gen_param_name, gen_param_value, op, radius);
  if (knn && num_neighbors < 3) fail_value(op, "NumNeighbors", "NumNeighbors must be >= 3");
  const auto k = knn ? static_cast<std::size_t>(num_neighbors) : std::size_t{0};
  const std::size_t nvalid = valid_count(model);
  if (nvalid == 0) {
    ObjectModel3DData out = model;
    out.normals.clear();
    return out;
  }
  if (knn && nvalid < k + 1) {
    fail(core::Status::InvalidArgument, op, "not enough valid points for the requested neighborhood");
  }
  runtime::ExecutionContext context;
  std::optional<NeighborhoodIndex> owned_index;
  if (shared_index == nullptr) {
    owned_index.emplace(model, context.scratch_resource());
    shared_index = &*owned_index;
  }
  const auto& index = *shared_index;
  ObjectModel3DData out = model;
  out.normals.assign(out.points.size(), Point3d{});
  const Point3d orient_ref =
      options.orientation == NormalOrientation::Viewpoint ? options.viewpoint : options.direction;
  const std::size_t work =
      knn ? runtime::ExecutionContext::saturated_work(model.points.size(), k) : model.points.size();
  runtime::ExecutionContext::for_each_partition(
      model.points.size(), work, kObjectModelParallelWork, [&](auto range) {
        runtime::ExecutionContext local;
        NeighborhoodQueryScratch knn_scratch(local.scratch_resource());
        if (knn) knn_scratch.prepare(k);
        std::vector<std::size_t> nn(knn ? k : 0);
        std::vector<double> d2(knn ? k : 0);
        std::vector<std::size_t> nbrs;
        std::vector<Point3d> pts;
        pts.reserve(knn ? k + 1 : 16);
        for (std::size_t i = range.begin; i < range.end; ++i) {
          if (!point_valid(model, i)) continue;
          pts.clear();
          pts.push_back(model.points[i]);
          if (knn) {
            const std::size_t found = index.knn(i, k, nn.data(), d2.data(), &knn_scratch);
            for (std::size_t j = 0; j < found; ++j) pts.push_back(model.points[nn[j]]);
          } else {
            index.radius(i, options.radius, &nbrs);
            for (std::size_t nb : nbrs) pts.push_back(model.points[nb]);
          }
          Point3d n{};
          if (pca_normal_raw(pts.data(), nullptr, pts.size(), &n)) {
            apply_normal_orientation(&n, model.points[i], options.orientation, orient_ref);
            out.normals[i] = n;
          }
        }
      });
  validate_object_model_3d(out);
  return out;
}

std::vector<ObjectModel3DData> connection_object_model_3d(const ObjectModel3DData& model, double max_distance,
                                                          std::int64_t min_points,
                                                          const NeighborhoodIndex* shared_index) {
  constexpr auto op = "connection_object_model_3d";
  validate_object_model_3d(model);
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {
    fail_value(op, "MaxDistance", "MaxDistance must be finite and positive");
  }
  if (min_points < 1) fail_value(op, "MinPoints", "MinPoints must be >= 1");
  const auto min_n = static_cast<std::size_t>(min_points);
  if (valid_count(model) == 0) return {};

  runtime::ExecutionContext context;
  std::optional<NeighborhoodIndex> owned_index;
  if (shared_index == nullptr) {
    owned_index.emplace(model, context.scratch_resource());
    shared_index = &*owned_index;
  }
  const auto& index = *shared_index;
  NeighborhoodVisitedState visited(index, context.scratch_resource());
  std::vector<std::uint8_t> seen(model.points.size(), 0U);
  std::vector<ObjectModel3DData> clusters;
  std::vector<std::size_t> stack;
  std::vector<std::size_t> nbrs;
  for (std::size_t seed : index.valid_indices()) {
    if (seen[seed] != 0U) continue;
    stack.clear();
    stack.push_back(seed);
    seen[seed] = 1U;
    visited.exclude(seed);
    std::vector<std::size_t> members;
    members.push_back(seed);
    while (!stack.empty()) {
      const std::size_t cur = stack.back();
      stack.pop_back();
      index.radius_unvisited(cur, max_distance, &nbrs, seen, visited);
      for (std::size_t nb : nbrs) {
        if (seen[nb] != 0U) continue;
        seen[nb] = 1U;
        visited.exclude(nb);
        stack.push_back(nb);
        members.push_back(nb);
      }
    }
    if (members.size() < min_n) continue;
    std::sort(members.begin(), members.end());
    auto cluster =
        compact_object_model_3d(model, std::span<const std::size_t>{members.data(), members.size()});
    validate_object_model_3d(cluster);
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

ObjectModel3DData fit_primitives_object_model_3d(const ObjectModel3DData& model, std::string_view type,
                                                 double max_distance, const core::Tuple& gen_param_name,
                                                 const core::Tuple& gen_param_value) {
  constexpr auto op = "fit_primitives_object_model_3d";
  validate_object_model_3d(model);
  if (type == "sphere" || type == "sphere_ransac" || type == "cylinder" || type == "cylinder_ransac")
    return fit_curved_primitive(model, type, max_distance, gen_param_name, gen_param_value);
  const bool ransac = type == "plane_ransac";
  if (type != "plane" && !ransac) fail_value(op, "Type", "v1 supports plane or plane_ransac");
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {
    fail_value(op, "MaxDistance", "MaxDistance must be finite and positive");
  }
  const auto options = parse_plane_gen(gen_param_name, gen_param_value, op, ransac);
  runtime::ExecutionContext context;
  std::pmr::vector<std::uint8_t> keep(model.points.size(), 0U, context.scratch_resource());
  Point3d n{};
  double d = 0.0;
  std::size_t inliers = 0;
  double rss = 0.0;
  if (ransac) {
    fit_plane_ransac(model, max_distance, options, &n, &d, &keep, &inliers, &rss, op);
  } else {
    std::vector<Point3d> pts;
    pts.reserve(model.points.size());
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      if (point_valid(model, i)) pts.push_back(model.points[i]);
    }
    if (pts.size() < 3) fail(core::Status::InvalidState, op, "plane fit needs at least three valid points");
    if (!pca_plane(pts, &n, &d)) fail(core::Status::InvalidState, op, "points are degenerate for a plane");
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      if (!point_valid(model, i)) continue;
      const double res = n.x * model.points[i].x + n.y * model.points[i].y + n.z * model.points[i].z + d;
      if (std::abs(res) <= max_distance) {
        keep[i] = 1U;
        rss += res * res;
        ++inliers;
      }
    }
    if (inliers < 3) fail(core::Status::InvalidState, op, "plane fit produced fewer than three inliers");
  }
  auto out = compact_object_model_3d(model, keep.data());
  out.primitive_kind = PrimitiveKind::Plane;
  out.plane = {n.x, n.y, n.z, d};
  out.primitive_num_inliers = inliers;
  out.primitive_rms = std::sqrt(rss / static_cast<double>(inliers));
  validate_object_model_3d(out);
  return out;
}

core::Tuple distance_object_model_3d(const ObjectModel3DData& model, const ObjectModel3DData& reference,
                                     std::string_view method, const NeighborhoodIndex* reference_index) {
  constexpr auto op = "distance_object_model_3d";
  validate_object_model_3d(model);
  validate_object_model_3d(reference);
  if (method == "primitive" || method == "triangles") return distance_to_surface(model, reference, method);
  core::Tuple distances;
  if (method == "points") {
    if (valid_count(reference) == 0) {
      for (std::size_t i = 0; i < model.points.size(); ++i) {
        if (point_valid(model, i)) distances.append(std::numeric_limits<double>::quiet_NaN());
      }
      return distances;
    }
    runtime::ExecutionContext context;
    std::optional<NeighborhoodIndex> owned_index;
    if (reference_index == nullptr) {
      owned_index.emplace(reference, context.scratch_resource());
      reference_index = &*owned_index;
    }
    const auto ids = collect_valid_indices(model);
    std::pmr::vector<double> slots(ids.size(), std::numeric_limits<double>::quiet_NaN(),
                                   runtime::scratch_resource());
    runtime::ExecutionContext::for_each_partition(
        ids.size(), ids.size(), kObjectModelParallelWork, [&](auto range) {
          for (std::size_t slot = range.begin; slot < range.end; ++slot) {
            std::size_t idx = 0;
            double d2 = 0.0;
            if (reference_index->nearest(model.points[ids[slot]], std::numeric_limits<double>::infinity(),
                                         &idx, &d2)) {
              slots[slot] = std::sqrt(d2);
            }
          }
        });
    for (double value : slots) distances.append(value);
    return distances;
  }
  if (method == "plane") {
    if (reference.primitive_kind != PrimitiveKind::Plane) {
      fail(core::Status::InvalidState, op, "reference object model has no fitted plane");
    }
    const auto& pl = reference.plane;
    const auto ids = collect_valid_indices(model);
    std::pmr::vector<double> slots(ids.size(), 0.0, runtime::scratch_resource());
    runtime::ExecutionContext::for_each_partition(
        ids.size(), ids.size(), kObjectModelParallelWork, [&](auto range) {
          for (std::size_t slot = range.begin; slot < range.end; ++slot) {
            const auto& p = model.points[ids[slot]];
            slots[slot] = pl[0] * p.x + pl[1] * p.y + pl[2] * p.z + pl[3];
          }
        });
    for (double value : slots) distances.append(value);
    return distances;
  }
  fail_value(op, "Method", "v1 supports only points or plane distances");
  return distances;
}

RegisterObjectModel3DResult register_object_model_3d_pair(
    const ObjectModel3DData& source, const ObjectModel3DData& target, const core::Tuple& initial_pose,
    std::string_view method, std::int64_t max_iterations, double max_distance,
    const core::Tuple& gen_param_name, const core::Tuple& gen_param_value,
    const NeighborhoodIndex* target_index) {
  return register_object_model_3d_pair(
      source, target, initial_pose, method, max_iterations, max_distance,
      parse_icp_gen(gen_param_name, gen_param_value, "register_object_model_3d_pair"), target_index);
}
RegisterObjectModel3DResult register_object_model_3d_pair(
    const ObjectModel3DData& source, const ObjectModel3DData& target, const core::Tuple& initial_pose,
    std::string_view method, std::int64_t max_iterations, double max_distance, const IcpOptions& options,
    const NeighborhoodIndex* target_index) {
  constexpr auto op = "register_object_model_3d_pair";
  validate_object_model_3d(source);
  validate_object_model_3d(target);
  const bool point_to_plane = method == "point_to_plane";
  if (method != "point_to_point" && !point_to_plane) {
    fail_value(op, "Method", "v1 supports point_to_point or point_to_plane");
  }
  if (point_to_plane && target.normals.empty()) {
    fail(core::Status::InvalidState, op, "point-to-plane registration requires target normals");
  }
  if (max_iterations < 1) fail_value(op, "MaxIterations", "MaxIterations must be >= 1");
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {
    fail_value(op, "MaxDistance", "MaxDistance must be finite and positive");
  }
  if (valid_count(source) == 0 || valid_count(target) == 0) {
    fail(core::Status::InvalidArgument, op, "source and target must contain valid points");
  }
  const bool angle_filter = options.max_normal_angle < kPi;
  if (angle_filter) {
    if (source.normals.empty() || target.normals.empty()) {
      fail(core::Status::InvalidState, op, "max_normal_angle requires source and target normals");
    }
  }

  auto T = calibration::parse_hom_mat3d(calibration::pose_to_hom_mat3d(initial_pose), op);
  if (options.coarse_seed && !point_to_plane) {
    if (auto seeded = fingerprint_seed(source, target, T, max_distance)) T = *seeded;
  }
  runtime::ExecutionContext context;
  std::optional<NeighborhoodIndex> owned_index;
  if (!target_index) {
    owned_index.emplace(target, context.scratch_resource());
    target_index = &*owned_index;
  }
  const auto& index = *target_index;
  const double max_d2 = max_distance * max_distance;
  const double weight_scale = options.clipping_factor * max_distance;
  RegisterObjectModel3DResult result;
  result.converged = false;
  bool prev_finite = false;
  double prev_rms = 0.0;
  std::int64_t last_success_iter = 0;
  for (std::int64_t iter = 1; iter <= max_iterations; ++iter) {
    const auto matches = query_correspondences(source, index, T, max_d2);
    auto euclid =
        pack_correspondences(matches, source, target, false, point_to_plane || angle_filter, angle_filter);
    trim_correspondences(&euclid, options.trim_ratio);
    if (angle_filter) {
      std::pmr::vector<std::uint8_t> keep(euclid.src.size(), 1U, runtime::scratch_resource());
      for (std::size_t i = 0; i < euclid.src.size(); ++i) {
        Point3d ns = transform_direction(T, euclid.src_n[i]);
        Point3d nt = euclid.nrm[i];
        if (zero_normal(ns) || zero_normal(nt) || !normalize(&ns) || !normalize(&nt)) {
          keep[i] = 0U;
          continue;
        }
        const double c = std::clamp(dot(ns, nt), -1.0, 1.0);
        if (std::acos(c) > options.max_normal_angle) keep[i] = 0U;
      }
      compact_correspondences(&euclid, keep);
    }
    result.iterations = iter;
    if (euclid.src.size() < 3) {
      if (iter == 1) fail(core::Status::InvalidState, op, "not enough correspondences for ICP");
      result.iterations = last_success_iter;
      break;
    }
    const double iter_rms = std::sqrt(euclid.rss / static_cast<double>(euclid.src.size()));
    if (!std::isfinite(iter_rms)) {
      if (iter == 1) fail(core::Status::InvalidState, op, "registration residual is not finite");
      result.iterations = last_success_iter;
      break;
    }

    GatheredCorrespondences solver;
    if (point_to_plane) {
      auto transformed = pack_correspondences(matches, source, target, true, true, angle_filter);
      trim_correspondences(&transformed, options.trim_ratio);
      std::pmr::vector<std::uint8_t> keep(transformed.src.size(), 1U, runtime::scratch_resource());
      for (std::size_t i = 0; i < transformed.src.size(); ++i) {
        if (zero_normal(transformed.nrm[i])) {
          keep[i] = 0U;
          continue;
        }
        if (!angle_filter) continue;
        Point3d ns = transform_direction(T, transformed.src_n[i]);
        Point3d nt = transformed.nrm[i];
        if (zero_normal(ns) || !normalize(&ns) || !normalize(&nt)) {
          keep[i] = 0U;
          continue;
        }
        const double c = std::clamp(dot(ns, nt), -1.0, 1.0);
        if (std::acos(c) > options.max_normal_angle) keep[i] = 0U;
      }
      compact_correspondences(&transformed, keep);
      solver = std::move(transformed);
    }

    std::array<double, 12> update{};
    bool ok = false;
    const auto T_before = T;
    if (point_to_plane) {
      assign_icp_weights(&solver, true, options.weighting, weight_scale);
      ok = solver.src.size() >= 3 &&
           point_to_plane_step(solver.src, solver.dst, solver.nrm, solver.w.data(), &update);
      if (ok) T = multiply_mat(update, T);
    }
    if (!point_to_plane || !ok) {
      // 回退只改变残差模型，仍保留显式的法向筛选、trim 和稳健权重。
      assign_icp_weights(&euclid, false, options.weighting, weight_scale);
      ok = kabsch(euclid.src, euclid.dst, euclid.w.data(), &T);
      update = multiply_mat(T, invert_rigid(T_before));
      if (!ok) fail(core::Status::InvalidState, op, "rigid alignment is degenerate");
    }
    last_success_iter = iter;
    result.iterations = iter;
    bool converged = prev_finite && std::abs(iter_rms - prev_rms) <= 1e-9 * std::max(1.0, prev_rms);
    if (options.pose_delta > 0.0 && rigid_step_size(update) <= options.pose_delta) converged = true;
    if (converged) {
      result.converged = true;
      break;
    }
    prev_rms = iter_rms;
    prev_finite = true;
  }
  result.pose = pose_from_rigid(T, op);
  const auto scored_T = calibration::parse_hom_mat3d(calibration::pose_to_hom_mat3d(result.pose), op);
  const auto quality = evaluate_icp_quality(source, index, scored_T, max_d2);
  result.rms = quality.rms;
  result.overlap = quality.overlap;
  result.sample_count = valid_count(source);
  result.inlier_count = quality.correspondences;
  return result;
}

ObjectModel3DData intersect_plane_object_model_3d(const ObjectModel3DData& model, double nx, double ny,
                                                  double nz, double d) {
  constexpr auto op = "intersect_plane_object_model_3d";
  validate_object_model_3d(model);
  if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz) || !std::isfinite(d)) {
    fail_value(op, "Plane", "plane coefficients must be finite");
  }
  Point3d n{nx, ny, nz};
  if (!normalize(&n)) fail_value(op, "Plane", "plane normal is degenerate");
  d /= length3({nx, ny, nz});
  if (model.triangles.empty()) {
    fail(core::Status::InvalidState, op, "plane intersection requires triangle topology");
  }

  auto signed_d = [&](const Point3d& p) { return n.x * p.x + n.y * p.y + n.z * p.z + d; };
  ObjectModel3DData out;
  out.points.reserve(model.triangles.size() * 2);
  // 源边穿越产生一个输出顶点，由相邻面共享；键按拓扑，不模糊合并不相连表面。
  std::map<std::array<std::uint32_t, 2>, std::uint32_t> intersections;
  std::set<std::array<std::uint32_t, 2>> segments;
  struct CrossedLine {
    std::array<std::uint32_t, 2> vertices;
    std::array<bool, 2> from_positive;
  };
  std::vector<CrossedLine> crossed_lines;
  for (const auto& tri : model.triangles) {
    if (!point_valid(model, tri[0]) || !point_valid(model, tri[1]) || !point_valid(model, tri[2])) {
      continue;
    }
    const Point3d p[3] = {model.points[tri[0]], model.points[tri[1]], model.points[tri[2]]};
    const double s[3] = {signed_d(p[0]), signed_d(p[1]), signed_d(p[2])};
    if (!std::isfinite(s[0]) || !std::isfinite(s[1]) || !std::isfinite(s[2])) continue;
    const bool positive = s[0] > 1e-12 || s[1] > 1e-12 || s[2] > 1e-12;
    const bool negative = s[0] < -1e-12 || s[1] < -1e-12 || s[2] < -1e-12;
    // 仅接触平面的边/顶点不属于穿越。
    if (!positive || !negative) continue;
    Point3d hits[2];
    std::array<std::uint32_t, 2> keys[2];
    std::array<bool, 2> from_positive{};
    int nh = 0;
    for (int e = 0; e < 3; ++e) {
      const int a = e;
      const int b = (e + 1) % 3;
      if (s[a] * s[b] > 0.0) continue;
      if (std::abs(s[a]) <= 1e-12 && std::abs(s[b]) <= 1e-12) continue;
      const double denom = s[a] - s[b];
      if (std::abs(denom) < kEps) continue;
      const double t = s[a] / denom;
      if (t < -1e-12 || t > 1.0 + 1e-12) continue;
      const double u = std::min(1.0, std::max(0.0, t));
      Point3d h{p[a].x + u * (p[b].x - p[a].x), p[a].y + u * (p[b].y - p[a].y),
                p[a].z + u * (p[b].z - p[a].z)};
      if (nh < 2 && (nh == 0 || dist2(hits[0], h) > kEps)) {
        auto first = tri[a], second = tri[b];
        if (u == 0)
          second = first;
        else if (u == 1)
          first = second;
        if (first > second) std::swap(first, second);
        keys[nh] = {first, second};
        from_positive[nh] = s[a] > s[b];
        hits[nh++] = h;
      }
    }
    if (nh != 2) continue;
    if (dist2(hits[0], hits[1]) <= kEps) continue;
    std::array<std::uint32_t, 2> line;
    for (std::size_t j = 0; j < 2; ++j) {
      auto found = intersections.find(keys[j]);
      if (found == intersections.end()) {
        if (out.points.size() >= std::numeric_limits<std::uint32_t>::max())
          fail(core::Status::LimitExceeded, op, "intersection exceeds line index capacity");
        const auto index = static_cast<std::uint32_t>(out.points.size());
        out.points.push_back(hits[j]);
        found = intersections.emplace(keys[j], index).first;
      }
      line[j] = found->second;
    }
    auto canonical = line;
    if (canonical[0] > canonical[1]) std::swap(canonical[0], canonical[1]);
    if (segments.insert(canonical).second) crossed_lines.push_back({line, from_positive});
  }
  // HALCON 顶点按规范源边而非相邻面遍历排序，将此序作为模型结果发布。
  std::vector<std::uint32_t> remap(out.points.size());
  auto points = std::move(out.points);
  out.points.reserve(points.size());
  for (const auto& [edge, old] : intersections) {
    remap[old] = static_cast<std::uint32_t>(out.points.size());
    out.points.push_back(points[old]);
  }
  for (auto& line : crossed_lines) {
    line.vertices = {remap[line.vertices[0]], remap[line.vertices[1]]};
    if (line.vertices[0] > line.vertices[1]) {
      std::swap(line.vertices[0], line.vertices[1]);
      std::swap(line.from_positive[0], line.from_positive[1]);
    }
  }
  // 共享交点处，沿共同边从正半空间走向负半空间的邻接面先于另一面。
  std::stable_sort(crossed_lines.begin(), crossed_lines.end(), [](const auto& a, const auto& b) {
    if (a.vertices[0] != b.vertices[0]) return a.vertices[0] < b.vertices[0];
    return a.from_positive[0] > b.from_positive[0];
  });
  out.lines.reserve(crossed_lines.size());
  for (const auto& line : crossed_lines) out.lines.push_back(line.vertices);
  validate_object_model_3d(out);
  return out;
}

}  // namespace mv::reconstruction

namespace mv::reconstruction {
namespace {
constexpr std::array<double, 12> pair_identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
double pair_diameter(const ObjectModel3DData& model) {
  Point3d lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (!point_valid(model, i)) continue;
    auto p = model.points[i];
    lo.x = std::min(lo.x, p.x);
    lo.y = std::min(lo.y, p.y);
    lo.z = std::min(lo.z, p.z);
    hi.x = std::max(hi.x, p.x);
    hi.y = std::max(hi.y, p.y);
    hi.z = std::max(hi.z, p.z);
  }
  return length3(sub(hi, lo));
}
ObjectModel3DData pair_surface(const ObjectModel3DData& input, double distance) {
  const auto indices = collect_valid_indices(input);
  auto model = compact_object_model_3d(input, indices);
  if (std::isnan(distance)) {
    if (model.normals.empty())
      fail(core::Status::InvalidState, "register_object_model_3d_pair",
           "nonfinite sampling radius cannot estimate missing surface normals");
    return model;
  }
  if (model.normals.empty()) {
    model.normals.resize(model.points.size());
    NeighborhoodIndex index(model, runtime::scratch_resource());
    std::vector<std::size_t> neighbors;
    std::vector<Point3d> points;
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      index.radius(i, distance, &neighbors);
      points.clear();
      for (auto j : neighbors) points.push_back(model.points[j]);
      Point3d normal{};
      if (pca_normal_raw(points.data(), nullptr, points.size(), &normal)) {
        if (dot(normal, model.points[i]) > 0) normal = {-normal.x, -normal.y, -normal.z};
        model.normals[i] = normal;
      }
    }
  }
  return sample_points(model, "fast", distance);
}
struct PairFrame {
  Point3d center{};
  double axes[3][3]{};
};
PairFrame pair_frame(const ObjectModel3DData& model) {
  PairFrame frame;
  for (auto p : model.points) {
    frame.center.x += p.x;
    frame.center.y += p.y;
    frame.center.z += p.z;
  }
  const auto n = double(model.points.size());
  frame.center.x /= n;
  frame.center.y /= n;
  frame.center.z /= n;
  double covariance[3][3]{};
  for (auto p : model.points) {
    const auto q = sub(p, frame.center);
    const double v[3]{q.x, q.y, q.z};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) covariance[i][j] += v[i] * v[j] / n;
  }
  double eigenvalues[3];
  jacobi_eigen3(covariance, eigenvalues, frame.axes);
  return frame;
}
std::array<double, 12> pair_initial(const ObjectModel3DData& source, const ObjectModel3DData& target,
                                    const NeighborhoodIndex& index, double threshold, double key_fraction,
                                    bool* found) {
  *found = false;
  auto best = pair_identity;
  if (!std::isfinite(key_fraction) || !(key_fraction > 0)) return best;
  if (std::floor(std::min(source.points.size(), target.points.size()) * key_fraction) < 3) return best;
  const auto a = pair_frame(source), b = pair_frame(target);
  std::size_t support = 0;
  double residual = INFINITY;
  const auto assess = [&](const std::array<double, 12>& transform) {
    std::size_t count = 0, j;
    double rss = 0, d;
    const auto stride = std::max<std::size_t>(1, std::size_t(std::floor(1 / key_fraction)));
    for (std::size_t i = 0; i < source.points.size(); i += stride)
      if (index.nearest(transform_point(transform, source.points[i]), threshold * threshold, &j, &d)) {
        ++count;
        rss += d;
      }
    if (count > support || (count == support && rss < residual)) {
      best = transform;
      support = count;
      residual = rss;
    }
  };
  // 枚举两协方差框架之间全部正常旋转，包含同值轴。
  int permutation[3]{0, 1, 2};
  do {
    for (int sign = 0; sign < 8; ++sign) {
      std::array<double, 12> candidate{};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          for (int k = 0; k < 3; ++k)
            candidate[4 * i + j] += b.axes[i][permutation[k]] * ((sign & (1 << k)) ? -1. : 1.) * a.axes[j][k];
      const double det = candidate[0] * (candidate[5] * candidate[10] - candidate[6] * candidate[9]) -
                         candidate[1] * (candidate[4] * candidate[10] - candidate[6] * candidate[8]) +
                         candidate[2] * (candidate[4] * candidate[9] - candidate[5] * candidate[8]);
      if (det < 0) continue;
      const double ca[3]{a.center.x, a.center.y, a.center.z}, cb[3]{b.center.x, b.center.y, b.center.z};
      for (int i = 0; i < 3; ++i) {
        candidate[4 * i + 3] = cb[i];
        for (int k = 0; k < 3; ++k) candidate[4 * i + 3] -= candidate[4 * i + k] * ca[k];
      }
      assess(candidate);
    }
  } while (std::next_permutation(permutation, permutation + 3));
  if (auto seed = fingerprint_seed(source, target, pair_identity, threshold)) assess(*seed);
  *found = support >= 3;
  return best;
}
bool pair_pseudoinverse(double matrix[6][6], double rhs[6], double solution[6]) {
  double axes[6][6]{};
  for (int i = 0; i < 6; ++i) axes[i][i] = 1;
  double scale = 0;
  for (int i = 0; i < 6; ++i) scale = std::max(scale, std::abs(matrix[i][i]));
  if (!(scale > 0) || !std::isfinite(scale)) return false;
  for (int sweep = 0; sweep < 256; ++sweep) {
    int p = 0, q = 1;
    double off = 0;
    for (int i = 0; i < 6; ++i)
      for (int j = i + 1; j < 6; ++j)
        if (std::abs(matrix[i][j]) > off) {
          off = std::abs(matrix[i][j]);
          p = i;
          q = j;
        }
    if (off <= scale * 1e-15) break;
    const double angle = .5 * std::atan2(2 * matrix[p][q], matrix[q][q] - matrix[p][p]), c = std::cos(angle),
                 s = std::sin(angle);
    const double app = matrix[p][p], aqq = matrix[q][q], apq = matrix[p][q];
    matrix[p][p] = c * c * app - 2 * c * s * apq + s * s * aqq;
    matrix[q][q] = s * s * app + 2 * c * s * apq + c * c * aqq;
    matrix[p][q] = matrix[q][p] = 0;
    for (int i = 0; i < 6; ++i)
      if (i != p && i != q) {
        const double x = matrix[i][p], y = matrix[i][q];
        matrix[i][p] = matrix[p][i] = c * x - s * y;
        matrix[i][q] = matrix[q][i] = s * x + c * y;
      }
    for (int i = 0; i < 6; ++i) {
      const double x = axes[i][p], y = axes[i][q];
      axes[i][p] = c * x - s * y;
      axes[i][q] = s * x + c * y;
    }
  }
  std::fill_n(solution, 6, 0.);
  bool observable = false;
  for (int i = 0; i < 6; ++i)
    if (matrix[i][i] > scale * 1e-12) {
      double weight = 0;
      for (int j = 0; j < 6; ++j) weight += axes[j][i] * rhs[j];
      weight /= matrix[i][i];
      for (int j = 0; j < 6; ++j) solution[j] += weight * axes[j][i];
      observable = true;
    }
  return observable && std::all_of(solution, solution + 6, [](double value) { return std::isfinite(value); });
}
std::array<double, 12> pair_refine(const ObjectModel3DData& source, const ObjectModel3DData& target,
                                   const NeighborhoodIndex& index, std::array<double, 12> transform,
                                   const PairRegistrationOptions& options, double threshold) {
  if (std::isnan(threshold)) return transform;
  for (std::size_t iteration = 0; iteration < options.refinement_steps; ++iteration) {
    double matrix[6][6]{}, rhs[6]{};
    std::size_t matches = 0, j;
    double d;
    const auto stride = options.sub_sampling + 1;
    for (std::size_t i = 0; i < source.points.size(); i += stride) {
      const auto p = transform_point(transform, source.points[i]);
      if (!index.nearest(p, threshold * threshold, &j, &d)) continue;
      Point3d normal = target.normals[j];
      if (!normalize(&normal)) continue;
      const auto delta = sub(target.points[j], p);
      const double row[6]{p.y * normal.z - p.z * normal.y,
                          p.z * normal.x - p.x * normal.z,
                          p.x * normal.y - p.y * normal.x,
                          normal.x,
                          normal.y,
                          normal.z};
      const double residual = dot(delta, normal);
      const double weight =
          options.preset == PairRegistrationPreset::Robust ? huber_weight(residual, threshold * .1) : 1.;
      for (int a = 0; a < 6; ++a) {
        rhs[a] += weight * row[a] * residual;
        for (int b = 0; b < 6; ++b) matrix[a][b] += weight * row[a] * row[b];
      }
      ++matches;
    }
    double solution[6];
    if (matches < 3 || !pair_pseudoinverse(matrix, rhs, solution)) break;
    const auto increment = rodrigues(solution, solution + 3);
    transform = multiply_mat(increment, transform);
    if (rigid_step_size(increment) <= 1e-12) break;
  }
  return transform;
}
double pair_score(const ObjectModel3DData& source, const ObjectModel3DData& target,
                  const NeighborhoodIndex& index, const std::array<double, 12>& transform, double tolerance,
                  bool invert_normals) {
  std::size_t matches = 0, total = 0, j;
  double d;
  for (std::size_t i = 0; i < source.points.size(); ++i) {
    auto normal = source.normals[i];
    if (!normalize(&normal)) continue;
    ++total;
    if (!index.nearest(transform_point(transform, source.points[i]), tolerance * tolerance, &j, &d)) continue;
    auto other = target.normals[j];
    if (!normalize(&other)) continue;
    const auto rotated = transform_direction(transform, normal);
    const auto angle = dot(rotated, other) * (invert_normals ? -1. : 1.);
    if (angle <= 0) continue;
    ++matches;
  }
  return total ? static_cast<double>(static_cast<float>(double(matches) / total)) : 0.;
}
}  // namespace
PairRegistrationResult register_pair(const ObjectModel3DData& first, const ObjectModel3DData& second,
                                     std::string_view method, const PairRegistrationOptions& options) {
  constexpr auto op = "register_object_model_3d_pair";
  validate_object_model_3d(first);
  validate_object_model_3d(second);
  if (method != "matching" && method != "icp") fail_value(op, "Method", "icp or matching required");
  if (valid_count(first) < 2 || valid_count(second) < 2)
    fail(core::Status::InvalidState, op, "each registration model requires at least two points");
  runtime::ExecutionContext context;
  const double diameter1 = pair_diameter(first), diameter2 = pair_diameter(second);
  if (!(diameter1 > 0) || !(diameter2 > 0) || !std::isfinite(diameter1) || !std::isfinite(diameter2))
    fail(core::Status::InvalidState, op, "registration model diameter must be positive and finite");
  auto source = pair_surface(first, options.sampling_distance[0] * diameter1),
       target = pair_surface(second, options.sampling_distance[1] * diameter2);
  NeighborhoodIndex index(target, context.scratch_resource());
  const double threshold = options.absolute_threshold_set
                               ? options.absolute_distance_threshold
                               : options.relative_distance_threshold * std::max(diameter1, diameter2);
  auto transform = pair_identity;
  bool found = true;
  if (method == "matching")
    transform = pair_initial(source, target, index, threshold,
                             std::min(options.key_point_fraction[0], options.key_point_fraction[1]), &found);
  if (!found) return {};
  if (method != "matching" && method != "icp") fail_value(op, "Method", "icp or matching required");
  transform = pair_refine(source, target, index, transform, options, threshold);
  PairRegistrationResult result;
  const auto pose = pose_from_rigid(transform, op);
  // HALCON 返回非负度数 Euler 位姿；保持实际变换并选择文档规定表示。
  for (std::size_t i = 0; i < pose.size(); ++i) {
    if (i >= 3 && i < 6) {
      const double angle = pose.as_real64(i);
      result.pose.append(angle < 0 ? angle + 360. : angle);
    } else
      result.pose.append(pose.at(i));
  }
  // 独立最近表面评分近似 HALCON 表面重叠；使用较小采样表面，不复用旧内点/RMS 诊断。
  const double tolerance =
      .1 *
      std::min((std::isnan(options.sampling_distance[0]) ? .05 : options.sampling_distance[0]) * diameter1,
               (std::isnan(options.sampling_distance[1]) ? .05 : options.sampling_distance[1]) * diameter2);
  double score;
  if (source.points.size() <= target.points.size())
    score = pair_score(source, target, index, transform, tolerance, options.invert_normals);
  else {
    NeighborhoodIndex reverse(source, context.scratch_resource());
    score = pair_score(target, source, reverse, invert_rigid(transform), tolerance, options.invert_normals);
  }
  result.score.append(std::isnan(threshold) ? 0. : score);
  return result;
}
}  // namespace mv::reconstruction
