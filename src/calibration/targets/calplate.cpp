#include <calibration/targets/calplate.hpp>
#include <calibration/calibration/homography.hpp>
#include <calibration/calibration/initialize.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/model.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

#include <core/exception.hpp>

namespace mv::calibration {
namespace {

struct Blob {
  double row{};
  double col{};
  double hole_score{};
  bool hole{false};
  bool clipped{false};
};

struct Vec2 {
  double x{};
  double y{};
};

struct Axial {
  int q{};
  int r{};
  bool ok{false};
};

[[noreturn]] void fail_arg(std::string_view message) {
  throw core::Exception(core::Status::InvalidArgument, "detect_calplate", message);
}

[[noreturn]] void fail_not_found(std::string_view message) {
  throw core::Exception(core::Status::NotFound, "detect_calplate", message);
}

constexpr int kHexDq[6] = {1, 0, -1, -1, 0, 1};
constexpr int kHexDr[6] = {0, 1, 1, 0, -1, -1};
constexpr double kPi = 3.14159265358979323846;
constexpr double kHoleScoreMark = 15.0;       // 比较内部与平均灰度，判断斑点是否有孔。
constexpr double kHoleScoreFinder = 8.0;      // 定位中心候选的最小孔洞评分。
constexpr double kOuterEdgeRayLo = 0.85;      // 径向搜索起点，以标记半径比例表示。
constexpr double kOuterEdgeRayHi = 1.25;      // 径向搜索终点，以标记半径比例表示。
constexpr int kOuterEdgeRays = 32;            // 每个标记周围的射线数。
constexpr std::size_t kOuterEdgeMinRays = 16;  // 实测标记。
constexpr std::size_t kFillMinRays = 8;        // 预测补齐标记，使用较宽松条件。
constexpr double kFillCenterInsetPx = 2.0;     // HALCON 保留中心在图像内的标记；双线性采样约需 2 px 边距。
constexpr double kFillAcceptInsetFrac = 0.015; // 拒绝距离图像边框过近的拟合中心。

std::size_t min_required_marks(std::size_t n_points) {
  const auto half = static_cast<std::size_t>(std::ceil(0.5 * static_cast<double>(n_points)));
  return std::max<std::size_t>(5, half);
}

double median_of(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const std::size_t n = values.size();
  const std::size_t mid = n / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  if (n % 2 == 1) return values[mid];
  const double upper = values[mid];
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid - 1),
                   values.end());
  return 0.5 * (upper + values[mid - 1]);
}

double median_nn_spacing(const std::vector<Vec2>& pts) {
  if (pts.size() < 2) return 0.0;
  std::vector<double> nns;
  nns.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < pts.size(); ++j) {
      if (i == j) continue;
      best = std::min(best, std::hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y));
    }
    if (std::isfinite(best)) nns.push_back(best);
  }
  return median_of(std::move(nns));
}

std::vector<std::vector<std::size_t>> lattice_graph(const std::vector<Vec2>& pts, double spacing) {
  std::vector<std::vector<std::size_t>> adj(pts.size());
  if (!(spacing > 0.0)) return adj;
  const double lo = 0.6 * spacing;
  const double hi = 1.4 * spacing;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    for (std::size_t j = i + 1; j < pts.size(); ++j) {
      const double dist = std::hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y);
      if (dist < lo || dist > hi) continue;
      adj[i].push_back(j);
      adj[j].push_back(i);
    }
  }
  return adj;
}

bool snap_hex_step(double a, double b, int& dq, int& dr) {
  double best = std::numeric_limits<double>::infinity();
  int best_k = 0;
  for (int k = 0; k < 6; ++k) {
    const double d =
        std::hypot(a - static_cast<double>(kHexDq[k]), b - static_cast<double>(kHexDr[k]));
    if (d < best) {
      best = d;
      best_k = k;
    }
  }
  if (!(best <= 0.5)) return false;
  dq = kHexDq[best_k];
  dr = kHexDr[best_k];
  return true;
}

bool solve_uv(const Vec2& u, const Vec2& v, const Vec2& d, double& a, double& b) {
  const double det = u.x * v.y - u.y * v.x;
  if (!(std::abs(det) > 1e-12)) return false;
  a = (d.x * v.y - d.y * v.x) / det;
  b = (u.x * d.y - u.y * d.x) / det;
  return std::isfinite(a) && std::isfinite(b);
}

std::vector<Axial> label_hex_axial(const std::vector<Vec2>& pts,
                                   const std::vector<std::vector<std::size_t>>& adj,
                                   std::size_t seed) {
  std::vector<Axial> labels(pts.size());
  if (seed >= pts.size() || adj[seed].size() < 2) return labels;

  std::vector<std::size_t> order = adj[seed];
  std::sort(order.begin(), order.end(), [&](std::size_t i, std::size_t j) {
    const double ai = std::atan2(pts[i].y - pts[seed].y, pts[i].x - pts[seed].x);
    const double aj = std::atan2(pts[j].y - pts[seed].y, pts[j].x - pts[seed].x);
    return ai < aj;
  });

  const std::size_t n = order.size();
  std::size_t best_i = 0;
  double best_err = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t ia = order[i];
    const std::size_t ib = order[(i + 1) % n];
    const double ang_a = std::atan2(pts[ia].y - pts[seed].y, pts[ia].x - pts[seed].x);
    const double ang_b = std::atan2(pts[ib].y - pts[seed].y, pts[ib].x - pts[seed].x);
    double dang = ang_b - ang_a;
    if (dang < 0.0) dang += 2.0 * kPi;
    const double err = std::abs(dang - kPi / 3.0);
    if (err < best_err) {
      best_err = err;
      best_i = i;
    }
  }

  const Vec2 u{pts[order[best_i]].x - pts[seed].x, pts[order[best_i]].y - pts[seed].y};
  const Vec2 v{pts[order[(best_i + 1) % n]].x - pts[seed].x,
               pts[order[(best_i + 1) % n]].y - pts[seed].y};

  labels[seed] = Axial{0, 0, true};
  std::deque<std::size_t> queue;
  queue.push_back(seed);

  while (!queue.empty()) {
    const std::size_t cur = queue.front();
    queue.pop_front();
    for (const std::size_t nxt : adj[cur]) {
      if (labels[nxt].ok) continue;
      const Vec2 d{pts[nxt].x - pts[cur].x, pts[nxt].y - pts[cur].y};
      double a = 0.0;
      double b = 0.0;
      if (!solve_uv(u, v, d, a, b)) continue;
      int dq = 0;
      int dr = 0;
      if (!snap_hex_step(a, b, dq, dr)) continue;
      labels[nxt] = Axial{labels[cur].q + dq, labels[cur].r + dr, true};
      queue.push_back(nxt);
    }
  }
  return labels;
}

void rotate_axial(int& q, int& r, int times) {
  for (int i = 0; i < times; ++i) {
    const int nq = -r;
    const int nr = q + r;
    q = nq;
    r = nr;
  }
}

bool solve3(double A[3][3], const double b[3], double x[3]) {
  double m[3][4] = {
      {A[0][0], A[0][1], A[0][2], b[0]},
      {A[1][0], A[1][1], A[1][2], b[1]},
      {A[2][0], A[2][1], A[2][2], b[2]},
  };
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(m[row][col]) > std::abs(m[pivot][col])) pivot = row;
    }
    if (!(std::abs(m[pivot][col]) > 1e-18)) return false;
    if (pivot != col) {
      for (int k = col; k < 4; ++k) std::swap(m[col][k], m[pivot][k]);
    }
    const double inv = 1.0 / m[col][col];
    for (int k = col; k < 4; ++k) m[col][k] *= inv;
    for (int row = 0; row < 3; ++row) {
      if (row == col) continue;
      const double f = m[row][col];
      for (int k = col; k < 4; ++k) m[row][k] -= f * m[col][k];
    }
  }
  x[0] = m[0][3];
  x[1] = m[1][3];
  x[2] = m[2][3];
  return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
}

bool solve5(double A[5][5], const double b[5], double x[5]) {
  double m[5][6];
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) m[i][j] = A[i][j];
    m[i][5] = b[i];
  }
  for (int col = 0; col < 5; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 5; ++row) {
      if (std::abs(m[row][col]) > std::abs(m[pivot][col])) pivot = row;
    }
    if (!(std::abs(m[pivot][col]) > 1e-18)) return false;
    if (pivot != col) {
      for (int k = col; k < 6; ++k) std::swap(m[col][k], m[pivot][k]);
    }
    const double inv = 1.0 / m[col][col];
    for (int k = col; k < 6; ++k) m[col][k] *= inv;
    for (int row = 0; row < 5; ++row) {
      if (row == col) continue;
      const double f = m[row][col];
      for (int k = col; k < 6; ++k) m[row][k] -= f * m[col][k];
    }
  }
  for (int i = 0; i < 5; ++i) x[i] = m[i][5];
  return std::isfinite(x[0]);
}

bool score_affine(const std::vector<Vec2>& descr_xy, const std::vector<Blob>& blobs,
                  const std::vector<std::size_t>& blob_ids, const std::vector<std::size_t>& descr_ids,
                  double& rms, double& d_row_dy) {
  const std::size_t n = blob_ids.size();
  if (n < 3) return false;
  double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0;
  double sxc = 0, syc = 0, sc = 0, sxr = 0, syr = 0, sr = 0;
  for (std::size_t k = 0; k < n; ++k) {
    const double x = descr_xy[descr_ids[k]].x;
    const double y = descr_xy[descr_ids[k]].y;
    const double col = blobs[blob_ids[k]].col;
    const double row = blobs[blob_ids[k]].row;
    sxx += x * x;
    sxy += x * y;
    syy += y * y;
    sx += x;
    sy += y;
    sxc += x * col;
    syc += y * col;
    sc += col;
    sxr += x * row;
    syr += y * row;
    sr += row;
  }
  const double nn = static_cast<double>(n);
  double Acol[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, nn}};
  double Arow[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, nn}};
  const double bc[3] = {sxc, syc, sc};
  const double br[3] = {sxr, syr, sr};
  double pc[3]{};
  double pr[3]{};
  if (!solve3(Acol, bc, pc) || !solve3(Arow, br, pr)) return false;
  d_row_dy = pr[1];
  double se = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const double x = descr_xy[descr_ids[k]].x;
    const double y = descr_xy[descr_ids[k]].y;
    const double dc = (pc[0] * x + pc[1] * y + pc[2]) - blobs[blob_ids[k]].col;
    const double dr = (pr[0] * x + pr[1] * y + pr[2]) - blobs[blob_ids[k]].row;
    se += dc * dc + dr * dr;
  }
  rms = std::sqrt(se / nn);
  return std::isfinite(rms);
}

bool better_match(std::size_t count, double rms, double fy, std::size_t best_count, double best_rms,
                  double best_fy) {
  if (count != best_count) return count > best_count;
  // 微小单定位标定板在 y 翻转下对称；优先 y 向上的相机手性
  // （板 y 增大时图像 row 减小），再比较仿射 RMS。
  const bool y_up = fy < 0.0;
  const bool best_y_up = best_fy < 0.0;
  if (y_up != best_y_up) return y_up;
  return rms < best_rms;
}

std::size_t seed_nearest_centroid(const std::vector<Vec2>& blob_xy) {
  Vec2 centroid{};
  for (const auto& p : blob_xy) {
    centroid.x += p.x;
    centroid.y += p.y;
  }
  const double inv = 1.0 / static_cast<double>(blob_xy.size());
  centroid.x *= inv;
  centroid.y *= inv;
  std::size_t best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < blob_xy.size(); ++i) {
    const double d = std::hypot(blob_xy[i].x - centroid.x, blob_xy[i].y - centroid.y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

// 多定位标记匹配：选择邻距集合与 descr 定位邻域最相符的图像节点，先归一化尺度。
std::size_t select_seed_blob(const std::vector<Vec2>& blob_xy,
                             const std::vector<std::vector<std::size_t>>& blob_adj,
                             const std::vector<Vec2>& descr_xy,
                             const std::vector<std::vector<std::size_t>>& descr_adj,
                             std::size_t finder_index, std::size_t finder_count) {
  if (blob_xy.empty()) return 0;
  if (finder_count <= 1) return seed_nearest_centroid(blob_xy);

  std::vector<double> finder_nb;
  for (const std::size_t j : descr_adj[finder_index]) {
    finder_nb.push_back(std::hypot(descr_xy[j].x - descr_xy[finder_index].x,
                                   descr_xy[j].y - descr_xy[finder_index].y));
  }
  std::sort(finder_nb.begin(), finder_nb.end());
  if (finder_nb.empty()) return seed_nearest_centroid(blob_xy);

  std::size_t best = blob_xy.size();
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < blob_xy.size(); ++i) {
    if (blob_adj[i].size() < 4) continue;
    std::vector<double> nb;
    nb.reserve(blob_adj[i].size());
    for (const std::size_t j : blob_adj[i]) {
      nb.push_back(std::hypot(blob_xy[j].x - blob_xy[i].x, blob_xy[j].y - blob_xy[i].y));
    }
    std::sort(nb.begin(), nb.end());
    const std::size_t m = std::min(nb.size(), finder_nb.size());
    if (m == 0) continue;
    const double scale = (nb[0] > 1e-12 && finder_nb[0] > 1e-12) ? (nb[0] / finder_nb[0]) : 1.0;
    double score = 0.0;
    for (std::size_t k = 0; k < m; ++k) {
      score += std::abs(nb[k] - scale * finder_nb[k]);
    }
    score += std::abs(static_cast<double>(nb.size()) - static_cast<double>(finder_nb.size()));
    if (score < best_score) {
      best_score = score;
      best = i;
    }
  }
  if (best == blob_xy.size()) return seed_nearest_centroid(blob_xy);
  return best;
}

std::vector<std::size_t> finder_point_indices(const CalplateDescription& descr) {
  std::vector<std::size_t> ids;
  ids.reserve(descr.finders.size());
  for (const auto& finder : descr.finders) {
    if (finder.column < 0 || finder.row < 0 || finder.column >= descr.columns ||
        finder.row >= descr.rows) {
      continue;
    }
    const auto index = static_cast<std::size_t>(finder.row * descr.columns + finder.column);
    if (index < descr.points.size()) ids.push_back(index);
  }
  return ids;
}

int otsu_threshold(const std::uint8_t* pixels, std::size_t width, std::size_t height,
                   std::size_t stride) {
  std::array<int, 256> hist{};
  int total = 0;
  for (std::size_t r = 0; r < height; ++r) {
    const auto* row = pixels + r * stride;
    for (std::size_t c = 0; c < width; ++c) {
      hist[row[c]] += 1;
      ++total;
    }
  }
  if (total <= 0) return 128;
  double sum = 0.0;
  for (int i = 0; i < 256; ++i) sum += static_cast<double>(i) * static_cast<double>(hist[i]);
  double sum_b = 0.0;
  int w_b = 0;
  double best_var = -1.0;
  int best_t = 128;
  for (int t = 0; t < 256; ++t) {
    w_b += hist[t];
    if (w_b == 0) continue;
    const int w_f = total - w_b;
    if (w_f == 0) break;
    sum_b += static_cast<double>(t) * static_cast<double>(hist[t]);
    const double m_b = sum_b / static_cast<double>(w_b);
    const double m_f = (sum - sum_b) / static_cast<double>(w_f);
    const double diff = m_b - m_f;
    const double var = static_cast<double>(w_b) * static_cast<double>(w_f) * diff * diff;
    if (var > best_var) {
      best_var = var;
      best_t = t;
    }
  }
  return best_t;
}


struct Rc {
  int r{};
  int c{};
};

bool fit_taubin_center(const std::vector<Vec2>& pts, double& col, double& row) {
  const std::size_t n = pts.size();
  if (n < 4) return false;
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto& p : pts) {
    mean_x += p.x;
    mean_y += p.y;
  }
  const double inv = 1.0 / static_cast<double>(n);
  mean_x *= inv;
  mean_y *= inv;
  if (n < 8) {
    col = mean_x;
    row = mean_y;
    return true;
  }

  double Mxx = 0.0;
  double Myy = 0.0;
  double Mxy = 0.0;
  double Mxz = 0.0;
  double Myz = 0.0;
  double Mzz = 0.0;
  for (const auto& p : pts) {
    const double Xi = p.x - mean_x;
    const double Yi = p.y - mean_y;
    const double Zi = Xi * Xi + Yi * Yi;
    Mxy += Xi * Yi;
    Mxx += Xi * Xi;
    Myy += Yi * Yi;
    Mxz += Xi * Zi;
    Myz += Yi * Zi;
    Mzz += Zi * Zi;
  }
  Mxx *= inv;
  Myy *= inv;
  Mxy *= inv;
  Mxz *= inv;
  Myz *= inv;
  Mzz *= inv;

  const double Mz = Mxx + Myy;
  const double Cov_xy = Mxx * Myy - Mxy * Mxy;
  const double Var_z = Mzz - Mz * Mz;
  const double A3 = 4.0 * Mz;
  const double A2 = -3.0 * Mz * Mz - Mzz;
  const double A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  const double A0 =
      Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
  const double A22 = A2 + A2;
  const double A33 = A3 + A3 + A3;

  double x = 0.0;
  for (int i = 0; i < 24; ++i) {
    const double y = A0 + x * (A1 + x * (A2 + x * A3));
    const double Dy = A1 + x * (A22 + x * A33);
    if (!(std::abs(Dy) > 1e-18)) break;
    const double xnew = x - y / Dy;
    if (std::abs(xnew - x) <= 1e-12 * std::max(1.0, std::abs(xnew))) {
      x = xnew;
      break;
    }
    x = xnew;
  }

  const double det = x * x - x * Mz + Cov_xy;
  if (!(std::abs(det) > 1e-14)) {
    col = mean_x;
    row = mean_y;
    return true;
  }
  col = mean_x + 0.5 * (Mxz * (Myy - x) - Myz * Mxy) / det;
  row = mean_y + 0.5 * (Myz * (Mxx - x) - Mxz * Mxy) / det;
  if (!std::isfinite(col) || !std::isfinite(row)) {
    col = mean_x;
    row = mean_y;
  }
  return true;
}

bool fit_ellipse_center(const std::vector<Vec2>& pts, double& col, double& row) {
  const std::size_t n = pts.size();
  if (n < 8) return false;
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto& p : pts) {
    mean_x += p.x;
    mean_y += p.y;
  }
  const double inv = 1.0 / static_cast<double>(n);
  mean_x *= inv;
  mean_y *= inv;
  double AtA[5][5]{};
  double Atb[5]{};
  for (const auto& p : pts) {
    const double x = p.x - mean_x;
    const double y = p.y - mean_y;
    const double d[5] = {x * x, x * y, y * y, x, y};
    for (int i = 0; i < 5; ++i) {
      Atb[i] += d[i];
      for (int j = 0; j < 5; ++j) AtA[i][j] += d[i] * d[j];
    }
  }
  double coef[5]{};
  if (!solve5(AtA, Atb, coef)) return false;
  const double a = coef[0];
  const double b = coef[1];
  const double c = coef[2];
  const double d = coef[3];
  const double e = coef[4];
  const double den = b * b - 4.0 * a * c;
  if (!(den < -1e-14)) return false;
  col = mean_x + (2.0 * c * d - b * e) / den;
  row = mean_y + (2.0 * a * e - b * d) / den;
  if (!std::isfinite(col) || !std::isfinite(row)) return false;
  if (std::hypot(col - mean_x, row - mean_y) > 6.0) return false;
  return true;
}

template <typename ValueAt>
bool center_from_outer_iso(const std::vector<Rc>& members, ValueAt value_at, double iso,
                           double& out_row, double& out_col) {
  if (members.size() < 4) return false;
  int rmin = members[0].r;
  int rmax = members[0].r;
  int cmin = members[0].c;
  int cmax = members[0].c;
  for (const auto& p : members) {
    rmin = std::min(rmin, p.r);
    rmax = std::max(rmax, p.r);
    cmin = std::min(cmin, p.c);
    cmax = std::max(cmax, p.c);
  }
  const int r0 = rmin - 1;
  const int c0 = cmin - 1;
  const int bh = rmax - rmin + 3;
  const int bw = cmax - cmin + 3;
  if (bh < 3 || bw < 3) return false;
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(bh) * static_cast<std::size_t>(bw), 0);
  auto at = [&](int r, int c) -> std::uint8_t& {
    return mask[static_cast<std::size_t>(r - r0) * static_cast<std::size_t>(bw) +
                static_cast<std::size_t>(c - c0)];
  };
  for (const auto& p : members) at(p.r, p.c) = 1;

  std::deque<Rc> q;
  for (int r = r0; r < r0 + bh; ++r) {
    for (int c = c0; c < c0 + bw; ++c) {
      if (r != r0 && r != r0 + bh - 1 && c != c0 && c != c0 + bw - 1) continue;
      if (at(r, c) != 0) continue;
      at(r, c) = 2;
      q.push_back({r, c});
    }
  }
  constexpr int kDr[4] = {-1, 1, 0, 0};
  constexpr int kDc[4] = {0, 0, -1, 1};
  while (!q.empty()) {
    const Rc p = q.front();
    q.pop_front();
    for (int k = 0; k < 4; ++k) {
      const int nr = p.r + kDr[k];
      const int nc = p.c + kDc[k];
      if (nr < r0 || nc < c0 || nr >= r0 + bh || nc >= c0 + bw) continue;
      if (at(nr, nc) != 0) continue;
      at(nr, nc) = 2;
      q.push_back({nr, nc});
    }
  }

  std::vector<Vec2> contour;
  contour.reserve(members.size() * 2);
  double sum_r = 0.0;
  double sum_c = 0.0;
  int n_outer = 0;
  for (const auto& p : members) {
    bool outer = false;
    for (int k = 0; k < 4; ++k) {
      const int nr = p.r + kDr[k];
      const int nc = p.c + kDc[k];
      const bool ext = (nr < r0 || nc < c0 || nr >= r0 + bh || nc >= c0 + bw) || at(nr, nc) == 2;
      if (!ext) continue;
      outer = true;
      const double v = value_at(p.r, p.c);
      const double vn = value_at(nr, nc);
      const double dv = v - vn;
      double t = 0.0;
      if (std::abs(dv) > 1e-9) t = (v - iso) / dv;
      if (!std::isfinite(t)) t = 0.0;
      t = std::clamp(t, 0.0, 1.0);
      contour.push_back({static_cast<double>(p.c) + t * static_cast<double>(nc - p.c),
                         static_cast<double>(p.r) + t * static_cast<double>(nr - p.r)});
    }
    if (outer) {
      sum_r += static_cast<double>(p.r);
      sum_c += static_cast<double>(p.c);
      ++n_outer;
    }
  }

  double col = 0.0;
  double row = 0.0;
  if (fit_ellipse_center(contour, col, row) || fit_taubin_center(contour, col, row)) {
    out_row = row;
    out_col = col;
    return true;
  }
  if (n_outer < 4) return false;
  out_row = sum_r / static_cast<double>(n_outer);
  out_col = sum_c / static_cast<double>(n_outer);
  return std::isfinite(out_row) && std::isfinite(out_col);
}

bool gray_centroid(const std::uint8_t* pixels, int width, int height, int stride, bool light,
                   double row, double col, double radius, double& out_row, double& out_col) {
  if (!(radius >= 1.2) || pixels == nullptr) return false;
  double cr = row;
  double cc = col;
  bool ok = false;
  for (int iter = 0; iter < 2; ++iter) {
    const int rad = std::max(2, static_cast<int>(std::ceil(1.70 * radius)));
    const int r0 = static_cast<int>(std::lround(cr));
    const int c0 = static_cast<int>(std::lround(cc));
    double ata[3][3]{};
    double atb[3]{};
    int n_bg = 0;
    double peak = light ? 0.0 : 255.0;
    const double r_out = 1.70 * radius;
    const double r_bg0 = 1.22 * radius;
    const double r_in = 1.12 * radius;
    const double r_out2 = r_out * r_out;
    const double r_bg02 = r_bg0 * r_bg0;
    const double r_in2 = r_in * r_in;
    for (int dr = -rad; dr <= rad; ++dr) {
      for (int dc = -rad; dc <= rad; ++dc) {
        const double d2 = static_cast<double>(dr * dr + dc * dc);
        if (d2 > r_out2) continue;
        const int rr = r0 + dr;
        const int cc_i = c0 + dc;
        if (rr < 0 || cc_i < 0 || rr >= height || cc_i >= width) continue;
        const double value = static_cast<double>(pixels[rr * stride + cc_i]);
        if (light)
          peak = std::max(peak, value);
        else
          peak = std::min(peak, value);
        if (d2 < r_bg02) continue;
        const double p[3] = {static_cast<double>(dr), static_cast<double>(dc), 1.0};
        for (int i = 0; i < 3; ++i) {
          atb[i] += p[i] * value;
          for (int j = 0; j < 3; ++j) ata[i][j] += p[i] * p[j];
        }
        ++n_bg;
      }
    }
    double plane[3] = {0.0, 0.0, light ? peak : peak};
    if (n_bg >= 8) {
      double x[3]{};
      if (solve3(ata, atb, x)) {
        plane[0] = x[0];
        plane[1] = x[1];
        plane[2] = x[2];
      }
    }
    double best_res = 0.0;
    for (int dr = -rad; dr <= rad; ++dr) {
      for (int dc = -rad; dc <= rad; ++dc) {
        const double d2 = static_cast<double>(dr * dr + dc * dc);
        if (d2 > r_in2) continue;
        const int rr = r0 + dr;
        const int cc_i = c0 + dc;
        if (rr < 0 || cc_i < 0 || rr >= height || cc_i >= width) continue;
        const double value = static_cast<double>(pixels[rr * stride + cc_i]);
        const double bg = plane[0] * static_cast<double>(dr) + plane[1] * static_cast<double>(dc) + plane[2];
        const double res = light ? (value - bg) : (bg - value);
        if (res > best_res) best_res = res;
      }
    }
    if (!(best_res > 4.0)) continue;
    const double t = 0.28 * best_res;
    const double iso = 0.5 * (best_res + t);
    std::vector<Rc> members;
    members.reserve(64);
    auto residual_at = [&](int rr, int cc_i) -> double {
      if (rr < 0 || cc_i < 0 || rr >= height || cc_i >= width) return 0.0;
      const double value = static_cast<double>(pixels[rr * stride + cc_i]);
      const double bg = plane[0] * static_cast<double>(rr - r0) +
                        plane[1] * static_cast<double>(cc_i - c0) + plane[2];
      return light ? (value - bg) : (bg - value);
    };
    for (int dr = -rad; dr <= rad; ++dr) {
      for (int dc = -rad; dc <= rad; ++dc) {
        const double d2 = static_cast<double>(dr * dr + dc * dc);
        if (d2 > r_in2) continue;
        const int rr = r0 + dr;
        const int cc_i = c0 + dc;
        if (rr < 0 || cc_i < 0 || rr >= height || cc_i >= width) continue;
        if (residual_at(rr, cc_i) <= t) continue;
        members.push_back({rr, cc_i});
      }
    }
    if (members.size() < 4) continue;
    double nr = cr;
    double nc = cc;
    if (!center_from_outer_iso(members, residual_at, iso, nr, nc)) {
      double sum_w = 0.0;
      double sum_wr = 0.0;
      double sum_wc = 0.0;
      for (const auto& p : members) {
        sum_w += 1.0;
        sum_wr += static_cast<double>(p.r);
        sum_wc += static_cast<double>(p.c);
      }
      nr = sum_wr / sum_w;
      nc = sum_wc / sum_w;
    }
    cr = nr;
    cc = nc;
    ok = true;
  }
  if (!ok) return false;
  out_row = cr;
  out_col = cc;
  return true;
}

std::vector<Blob> extract_blobs(const std::uint8_t* pixels, std::size_t width, std::size_t height,
                                std::size_t stride, bool light, double threshold, std::size_t min_area,
                                std::size_t max_area) {
  threshold = std::clamp(threshold, 1.0, 254.0);
  const auto is_foreground = [light, threshold](std::uint8_t value) {
    const double v = static_cast<double>(value);
    return light ? v > threshold : v < threshold;
  };
  const auto weight_of = [light, threshold](std::uint8_t value) {
    const double v = static_cast<double>(value);
    return light ? v - threshold : threshold - v;
  };

  std::vector<std::uint8_t> seen(width * height, 0);
  std::vector<Blob> blobs;
  constexpr int kDrs[4] = {-1, 1, 0, 0};
  constexpr int kDcs[4] = {0, 0, -1, 1};
  const int height_i = static_cast<int>(height);
  const int width_i = static_cast<int>(width);

  for (std::size_t r = 0; r < height; ++r) {
    for (std::size_t c = 0; c < width; ++c) {
      const std::size_t idx = r * width + c;
      if (seen[idx] || !is_foreground(pixels[r * stride + c])) continue;

      std::deque<std::pair<int, int>> queue;
      queue.emplace_back(static_cast<int>(r), static_cast<int>(c));
      seen[idx] = 1;

      double sum_w = 0.0;
      double sum_wr = 0.0;
      double sum_wc = 0.0;
      double sum_v = 0.0;
      double blob_peak = light ? 0.0 : 255.0;
      std::size_t area = 0;
      bool clipped = false;
      std::vector<Rc> members;
      members.reserve(64);
      while (!queue.empty()) {
        const auto [rr, cc] = queue.front();
        queue.pop_front();
        if (rr == 0 || cc == 0 || rr == height_i - 1 || cc == width_i - 1) clipped = true;
        const auto value =
            pixels[static_cast<std::size_t>(rr) * stride + static_cast<std::size_t>(cc)];
        const double weight = weight_of(value);
        const double v = static_cast<double>(value);
        sum_w += weight;
        sum_wr += weight * static_cast<double>(rr);
        sum_wc += weight * static_cast<double>(cc);
        sum_v += v;
        if (light)
          blob_peak = std::max(blob_peak, v);
        else
          blob_peak = std::min(blob_peak, v);
        members.push_back({rr, cc});
        ++area;

        for (int k = 0; k < 4; ++k) {
          const int nr = rr + kDrs[k];
          const int nc = cc + kDcs[k];
          if (nr < 0 || nc < 0 || nr >= height_i || nc >= width_i) continue;
          const std::size_t neighbor =
              static_cast<std::size_t>(nr) * width + static_cast<std::size_t>(nc);
          if (seen[neighbor]) continue;
          if (!is_foreground(
                  pixels[static_cast<std::size_t>(nr) * stride + static_cast<std::size_t>(nc)])) {
            continue;
          }
          seen[neighbor] = 1;
          queue.emplace_back(nr, nc);
        }
      }

      if (area < min_area || area > max_area || !(sum_w > 0.0)) continue;
      const double hole_cut = light ? threshold + 0.25 * (blob_peak - threshold)
                                    : threshold - 0.25 * (threshold - blob_peak);
      double clip_w = 0.0;
      double clip_wr = 0.0;
      double clip_wc = 0.0;
      for (const auto& p : members) {
        const double v = static_cast<double>(
            pixels[static_cast<std::size_t>(p.r) * stride + static_cast<std::size_t>(p.c)]);
        if (light && v < hole_cut) continue;
        if (!light && v > hole_cut) continue;
        const double w = light ? v - threshold : threshold - v;
        if (w <= 0.0) continue;
        clip_w += w;
        clip_wr += w * static_cast<double>(p.r);
        clip_wc += w * static_cast<double>(p.c);
      }
      Blob blob;
      blob.row = clip_w > 0.0 ? clip_wr / clip_w : sum_wr / sum_w;
      blob.col = clip_w > 0.0 ? clip_wc / clip_w : sum_wc / sum_w;
      auto gray_at = [&](int rr, int cc) -> double {
        if (rr < 0 || cc < 0 || rr >= height_i || cc >= width_i) return light ? 0.0 : 255.0;
        return static_cast<double>(
            pixels[static_cast<std::size_t>(rr) * stride + static_cast<std::size_t>(cc)]);
      };
      const double iso = 0.5 * (blob_peak + threshold);
      double er = blob.row;
      double ec = blob.col;
      if (center_from_outer_iso(members, gray_at, iso, er, ec)) {
        blob.row = er;
        blob.col = ec;
      }
      const double mean_v = sum_v / static_cast<double>(area);
      const double radius = std::max(1.0, std::sqrt(static_cast<double>(area) / 3.14159265358979323846));
      const int rad = std::max(1, static_cast<int>(std::lround(0.35 * radius)));
      double inner_sum = 0.0;
      int inner_n = 0;
      const int ir0 = static_cast<int>(std::lround(blob.row));
      const int ic0 = static_cast<int>(std::lround(blob.col));
      for (int dr = -rad; dr <= rad; ++dr) {
        for (int dc = -rad; dc <= rad; ++dc) {
          if (dr * dr + dc * dc > rad * rad) continue;
          const int rr = ir0 + dr;
          const int cc = ic0 + dc;
          if (rr < 0 || cc < 0 || rr >= height_i || cc >= width_i) continue;
          inner_sum += static_cast<double>(
              pixels[static_cast<std::size_t>(rr) * stride + static_cast<std::size_t>(cc)]);
          ++inner_n;
        }
      }
      const double inner_mean = inner_n > 0 ? inner_sum / static_cast<double>(inner_n) : mean_v;
      blob.hole_score = light ? (mean_v - inner_mean) : (inner_mean - mean_v);
      blob.hole = blob.hole_score > kHoleScoreMark;
      blob.clipped = clipped;
      blobs.push_back(blob);
    }
  }

  return blobs;
}

std::size_t largest_component_seed(const std::vector<std::vector<std::size_t>>& adj) {
  std::vector<std::uint8_t> seen(adj.size(), 0);
  std::size_t best_seed = 0;
  std::size_t best_size = 0;
  std::size_t best_deg = 0;
  for (std::size_t i = 0; i < adj.size(); ++i) {
    if (seen[i] != 0) continue;
    std::deque<std::size_t> queue;
    queue.push_back(i);
    seen[i] = 1;
    std::size_t size = 0;
    std::size_t seed = i;
    std::size_t deg = adj[i].size();
    while (!queue.empty()) {
      const std::size_t cur = queue.front();
      queue.pop_front();
      ++size;
      if (adj[cur].size() > deg) {
        deg = adj[cur].size();
        seed = cur;
      }
      for (const std::size_t nxt : adj[cur]) {
        if (seen[nxt] != 0) continue;
        seen[nxt] = 1;
        queue.push_back(nxt);
      }
    }
    if (size > best_size || (size == best_size && deg > best_deg)) {
      best_size = size;
      best_deg = deg;
      best_seed = seed;
    }
  }
  return best_seed;
}

bool better_ranked_match(std::size_t finder_hits, std::size_t count, double rms, double fy,
                         double east_pref, std::size_t best_hits, std::size_t best_count, double best_rms,
                         double best_fy, double best_east_pref, bool use_chirality, bool use_east) {
  if (finder_hits != best_hits) return finder_hits > best_hits;
  if (use_east && std::abs(east_pref - best_east_pref) > 2.0) return east_pref > best_east_pref;
  if (use_chirality) return better_match(count, rms, fy, best_count, best_rms, best_fy);
  if (count != best_count) return count > best_count;
  return rms < best_rms;
}

// 调用局部不可变行索引，供这些斑点的全部候选匹配查询共享。
struct BlobSearch {
  const std::vector<Blob>* source;
  std::vector<std::size_t> order;
  bool finite{true};
  explicit BlobSearch(const std::vector<Blob>& blobs) : source(&blobs) {
    order.reserve(blobs.size());
    for (std::size_t i = 0; i < blobs.size(); ++i) {
      order.push_back(i);
      finite = finite && std::isfinite(blobs[i].row) && std::isfinite(blobs[i].col);
    }
    if (!finite) return;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return blobs[a].row < blobs[b].row ||
             (blobs[a].row == blobs[b].row && a < b);
    });
  }
};

std::size_t nearest_index(const std::vector<Blob>& blobs, double row, double col, double max_dist,
                          const std::vector<std::uint8_t>& used,
                          const BlobSearch* search = nullptr) {
  std::size_t best = blobs.size();
  double best_d = max_dist;
  const auto consider = [&](std::size_t i) {
    if (i < used.size() && used[i] != 0) return;
    const double dr = blobs[i].row - row;
    const double dc = blobs[i].col - col;
    if (0.5 * std::abs(dr) > best_d || 0.5 * std::abs(dc) > best_d) return;
    const double d = std::hypot(dr, dc);
    // 精确同值时，原线性扫描选择最小原始索引。
    if (d < best_d || (best < blobs.size() && d == best_d && i < best)) {
      best_d = d;
      best = i;
    }
  };
  if (search && search->source == &blobs && search->order.size() == blobs.size() &&
      search->finite && std::isfinite(row) && std::isfinite(col) &&
      std::isfinite(max_dist) && max_dist > 0.0) {
    const double span = 2.0 * max_dist;
    const double lower = row - span, upper = row + span;
    if (span > 0.0 && std::isfinite(span) && std::isfinite(lower) && std::isfinite(upper)) {
      const auto first = std::lower_bound(search->order.begin(), search->order.end(), lower,
          [&](std::size_t i, double v) { return blobs[i].row < v; });
      const auto last = std::upper_bound(first, search->order.end(), upper,
          [&](double v, std::size_t i) { return v < blobs[i].row; });
      for (auto it = first; it != last; ++it) consider(*it);
      return best;
    }
  }
  // 非有限、溢出/下溢边界输入保留原扫描。
  for (std::size_t i = 0; i < blobs.size(); ++i) consider(i);
  return best;
}

bool solve8(double A[8][8], const double b[8], double x[8]) {
  double m[8][9];
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) m[i][j] = A[i][j];
    m[i][8] = b[i];
  }
  for (int col = 0; col < 8; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 8; ++row) {
      if (std::abs(m[row][col]) > std::abs(m[pivot][col])) pivot = row;
    }
    if (!(std::abs(m[pivot][col]) > 1e-12)) return false;
    if (pivot != col) {
      for (int k = col; k < 9; ++k) std::swap(m[col][k], m[pivot][k]);
    }
    const double inv = 1.0 / m[col][col];
    for (int k = col; k < 9; ++k) m[col][k] *= inv;
    for (int row = 0; row < 8; ++row) {
      if (row == col) continue;
      const double f = m[row][col];
      for (int k = col; k < 9; ++k) m[row][k] -= f * m[col][k];
    }
  }
  for (int i = 0; i < 8; ++i) x[i] = m[i][8];
  return std::isfinite(x[0]);
}

bool fit_homography(const std::vector<Vec2>& descr_xy, const std::vector<Blob>& blobs,
                    const std::vector<std::size_t>& descr_ids, const std::vector<std::size_t>& blob_ids,
                    double h[8]) {
  if (descr_ids.size() < 4 || descr_ids.size() != blob_ids.size()) return false;
  double AtA[8][8]{};
  double Atb[8]{};
  for (std::size_t n = 0; n < descr_ids.size(); ++n) {
    const double X = descr_xy[descr_ids[n]].x;
    const double Y = descr_xy[descr_ids[n]].y;
    const double u = blobs[blob_ids[n]].col;
    const double v = blobs[blob_ids[n]].row;
    const double ju[8] = {X, Y, 1.0, 0, 0, 0, -X * u, -Y * u};
    const double jv[8] = {0, 0, 0, X, Y, 1.0, -X * v, -Y * v};
    for (int i = 0; i < 8; ++i) {
      Atb[i] += ju[i] * u + jv[i] * v;
      for (int j = 0; j < 8; ++j) AtA[i][j] += ju[i] * ju[j] + jv[i] * jv[j];
    }
  }
  return solve8(AtA, Atb, h);
}

void project_h(const double h[8], double x, double y, double& col, double& row) {
  const double w = h[6] * x + h[7] * y + 1.0;
  col = (h[0] * x + h[1] * y + h[2]) / w;
  row = (h[3] * x + h[4] * y + h[5]) / w;
}


void refine_gray_marks(const std::uint8_t* pixels, std::size_t width, std::size_t height, std::size_t stride,
                       bool light, const CalplateDescription& descr, const std::vector<Vec2>& descr_xy,
                       CalplateObservation& obs) {
  if (obs.indices.size() < 4 || pixels == nullptr) return;
  std::vector<Blob> pts;
  std::vector<std::size_t> d_ids;
  std::vector<std::size_t> b_ids;
  pts.reserve(obs.indices.size());
  d_ids.reserve(obs.indices.size());
  b_ids.reserve(obs.indices.size());
  std::vector<Vec2> xy;
  xy.reserve(obs.indices.size());
  for (std::size_t k = 0; k < obs.indices.size(); ++k) {
    Blob b;
    b.row = obs.rows[k];
    b.col = obs.columns[k];
    pts.push_back(b);
    d_ids.push_back(static_cast<std::size_t>(obs.indices[k]));
    b_ids.push_back(k);
    xy.push_back({obs.columns[k], obs.rows[k]});
  }
  const double spacing = median_nn_spacing(xy);
  double h[8]{};
  const bool have_h = fit_homography(descr_xy, pts, d_ids, b_ids, h);
  for (std::size_t k = 0; k < obs.indices.size(); ++k) {
    const std::size_t id = static_cast<std::size_t>(obs.indices[k]);
    double radius = 0.22 * spacing;
    if (have_h && id < descr.radii.size() && descr.radii[id] > 1e-9) {
      double col0 = 0.0, row0 = 0.0, col1 = 0.0, row1 = 0.0;
      project_h(h, descr_xy[id].x, descr_xy[id].y, col0, row0);
      project_h(h, descr_xy[id].x + descr.radii[id], descr_xy[id].y, col1, row1);
      const double r_px = std::hypot(col1 - col0, row1 - row0);
      if (r_px > 1.2) radius = r_px;
    }
    double nr = obs.rows[k];
    double nc = obs.columns[k];
    if (gray_centroid(pixels, static_cast<int>(width), static_cast<int>(height), static_cast<int>(stride),
                      light, obs.rows[k], obs.columns[k], radius, nr, nc)) {
      if (std::hypot(nr - obs.rows[k], nc - obs.columns[k]) <= 1.5) {
        obs.rows[k] = nr;
        obs.columns[k] = nc;
      }
    }
  }
}

bool observation_from_homography(const std::vector<Vec2>& descr_xy, const std::vector<Blob>& blobs,
                                 const CalplateDescription& descr, const double h[8], double max_dist,
                                 std::size_t need, double width, double height, double margin,
                                 CalplateObservation& out, const BlobSearch* search = nullptr) {
  std::vector<std::uint8_t> used(blobs.size(), 0);
  out = {};
  out.rows.reserve(descr_xy.size());
  out.columns.reserve(descr_xy.size());
  out.indices.reserve(descr_xy.size());
  (void)descr;
  for (std::size_t i = 0; i < descr_xy.size(); ++i) {
    double col = 0.0;
    double row = 0.0;
    project_h(h, descr_xy[i].x, descr_xy[i].y, col, row);
    if (col < margin - 0.5 || row < margin - 0.5 || col > width - 0.5 - margin ||
        row > height - 0.5 - margin) {
      continue;
    }
    const std::size_t b = nearest_index(blobs, row, col, max_dist, used, search);
    if (b >= blobs.size()) continue;
    if (blobs[b].clipped) continue;
    used[b] = 1;
    out.rows.push_back(blobs[b].row);
    out.columns.push_back(blobs[b].col);
    out.indices.push_back(static_cast<std::int64_t>(i));
  }
  return out.indices.size() >= need;
}

void hex_neighbor_rc(int row, int col, int dir, int& nr, int& nc) {
  // dir：0=E、1=NE、2=SE、3=W、4=SW、5=NW；even-r 偏移，偶数行右移。
  static constexpr int kEven[6][2] = {{0, 1}, {-1, 1}, {1, 1}, {0, -1}, {1, 0}, {-1, 0}};
  static constexpr int kOdd[6][2] = {{0, 1}, {-1, 0}, {1, 0}, {0, -1}, {1, -1}, {-1, -1}};
  const auto& d = (row % 2 == 0) ? kEven[dir] : kOdd[dir];
  nr = row + d[0];
  nc = col + d[1];
}

void homography_rotate_180(const double h[8], double h180[8]) {
  // F0 位于板原点，descr 中的 180° 对应 (x,y)→(-x,-y)。
  h180[0] = -h[0];
  h180[1] = -h[1];
  h180[2] = h[2];
  h180[3] = -h[3];
  h180[4] = -h[4];
  h180[5] = h[5];
  h180[6] = -h[6];
  h180[7] = -h[7];
}

double finder_pattern_score(const std::vector<Vec2>& descr_xy, const std::vector<Blob>& blobs,
                            const CalplateDescription& descr, const double h[8], double s_img,
                            int nfind = 5, const BlobSearch* search = nullptr) {
  // 官方 calplate_80mm 定位孔掩码顺序为 C/E/NE/SE/W/SW/NW，true 表示孔；仅用于标准五定位描述。
  static constexpr bool kHole[5][7] = {
      {true, false, true, true, true, true, true},
      {false, true, false, false, true, true, true},
      {true, true, false, true, true, false, false},
      {true, true, false, false, true, true, false},
      {false, false, true, false, true, true, true},
  };
  nfind = std::min(nfind, static_cast<int>(descr.finders.size()));
  nfind = std::min(nfind, 5);
  double score = 0.0;
  std::vector<std::uint8_t> unused(blobs.size(), 0);
  const double max_d = 0.55 * s_img;
  for (int f = 0; f < nfind; ++f) {
    const int r0 = static_cast<int>(descr.finders[static_cast<std::size_t>(f)].row);
    const int c0 = static_cast<int>(descr.finders[static_cast<std::size_t>(f)].column);
    for (int k = 0; k < 7; ++k) {
      int r = r0;
      int c = c0;
      if (k > 0) hex_neighbor_rc(r0, c0, k - 1, r, c);
      if (r < 0 || c < 0 || r >= descr.rows || c >= descr.columns) continue;
      const std::size_t id = static_cast<std::size_t>(r * descr.columns + c);
      if (id >= descr_xy.size()) continue;
      double col = 0.0;
      double row = 0.0;
      project_h(h, descr_xy[id].x, descr_xy[id].y, col, row);
      const std::size_t b = nearest_index(blobs, row, col, max_d, unused, search);
      const double hs = b < blobs.size() ? blobs[b].hole_score : 0.0;
      score += kHole[f][k] ? hs : -hs;
    }
  }
  return score;
}

bool refine_observation(const std::vector<Vec2>& descr_xy, const std::vector<Blob>& blobs,
                        const CalplateDescription& descr, double h[8], double max_dist,
                        std::size_t need, double width, double height, double margin,
                        CalplateObservation& out, const BlobSearch* search = nullptr) {
  for (int iter = 0; iter < 2; ++iter) {
    if (!observation_from_homography(descr_xy, blobs, descr, h, max_dist, need, width, height,
                                     margin, out, search)) {
      return false;
    }
    std::vector<std::size_t> d_ids;
    std::vector<std::size_t> b_ids;
    std::vector<std::uint8_t> used(blobs.size(), 0);
    d_ids.reserve(out.indices.size());
    b_ids.reserve(out.indices.size());
    for (std::size_t k = 0; k < out.indices.size(); ++k) {
      const std::size_t b = nearest_index(blobs, out.rows[k], out.columns[k], 1.5, used, search);
      if (b >= blobs.size()) continue;
      used[b] = 1;
      d_ids.push_back(static_cast<std::size_t>(out.indices[k]));
      b_ids.push_back(b);
    }
    if (d_ids.size() < 4) break;
    double h2[8]{};
    if (!fit_homography(descr_xy, blobs, d_ids, b_ids, h2)) break;
    for (int i = 0; i < 8; ++i) h[i] = h2[i];
  }
  return observation_from_homography(descr_xy, blobs, descr, h, max_dist, need, width, height,
                                     margin, out, search);
}

double wrap_pi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

bool detect_with_hole_finders(const std::vector<Blob>& blobs, const std::vector<Vec2>& descr_xy,
                              const CalplateDescription& descr,
                              const std::vector<std::size_t>& finder_ids, std::size_t need,
                              double width, double height, CalplateObservation& out,
                              double* score_out) {
  if (finder_ids.size() < 5 || blobs.size() < 4) return false;
  const auto& f0 = descr.finders[0];
  if (f0.column < 1 || f0.column + 1 >= descr.columns) return false;
  const std::size_t east_id =
      static_cast<std::size_t>(f0.row * descr.columns + (f0.column + 1));
  const std::size_t west_id =
      static_cast<std::size_t>(f0.row * descr.columns + (f0.column - 1));
  if (east_id >= descr_xy.size() || west_id >= descr_xy.size() || finder_ids[0] >= descr_xy.size()) {
    return false;
  }

  std::vector<Vec2> blob_xy;
  blob_xy.reserve(blobs.size());
  for (const auto& b : blobs) blob_xy.push_back({b.col, b.row});
  const double s_img = median_nn_spacing(blob_xy);
  if (!(s_img > 0.0)) return false;
  const BlobSearch search{blobs};

  const double lo = 0.45 * s_img;
  const double hi = 1.70 * s_img;
  std::vector<std::vector<std::size_t>> nbr(blobs.size());
  for (std::size_t i = 0; i < blobs.size(); ++i) {
    for (std::size_t j = i + 1; j < blobs.size(); ++j) {
      const double d = std::hypot(blobs[i].row - blobs[j].row, blobs[i].col - blobs[j].col);
      if (d < lo || d > hi) continue;
      nbr[i].push_back(j);
      nbr[j].push_back(i);
    }
  }

  struct CenterCand {
    std::size_t i{};
    int hole_n{};
    double score{};
  };
  std::vector<CenterCand> centers;
  for (std::size_t i = 0; i < blobs.size(); ++i) {
    if (blobs[i].hole_score <= kHoleScoreFinder) continue;
    int hole_n = 0;
    for (const std::size_t j : nbr[i]) {
      if (blobs[j].hole_score > kHoleScoreFinder) ++hole_n;
    }
    if (hole_n >= 2) centers.push_back({i, hole_n, blobs[i].hole_score});
  }
  std::sort(centers.begin(), centers.end(), [](const CenterCand& a, const CenterCand& b) {
    if (a.hole_n != b.hole_n) return a.hole_n > b.hole_n;
    return a.score > b.score;
  });
  if (centers.size() > 36) centers.resize(36);
  if (centers.empty()) return false;

  std::vector<std::size_t> descr_ring;
  {
    const int r0 = static_cast<int>(f0.row);
    const int c0 = static_cast<int>(f0.column);
    for (int dir = 0; dir < 6; ++dir) {
      int nr = 0;
      int nc = 0;
      hex_neighbor_rc(r0, c0, dir, nr, nc);
      if (nr < 0 || nc < 0 || nr >= descr.rows || nc >= descr.columns) continue;
      const std::size_t id = static_cast<std::size_t>(nr * descr.columns + nc);
      if (id < descr_xy.size()) descr_ring.push_back(id);
    }
  }

  const Vec2 p0 = descr_xy[finder_ids[0]];
  const Vec2 pe = descr_xy[east_id];
  const double dx = pe.x - p0.x;
  const double dy = pe.y - p0.y;
  const double nd = dx * dx + dy * dy;
  if (!(nd > 1e-18)) return false;

  struct Map {
    double a{};
    double b{};
    bool reflect{};
  };
  const auto apply = [&](const Map& m, const Vec2& p, const Blob& origin, double& col, double& row) {
    const double ddx = p.x - p0.x;
    const double ddy = m.reflect ? -(p.y - p0.y) : (p.y - p0.y);
    col = origin.col + m.a * ddx - m.b * ddy;
    row = origin.row + m.b * ddx + m.a * ddy;
  };

  CalplateObservation best_obs;
  double best_score = -1e100;
  bool have = false;

  double best_pattern = -1e100;
  double best_pattern_180 = 1e100;

  const auto consider = [&](const std::vector<std::size_t>& d_ids,
                            const std::vector<std::size_t>& b_ids, double err) {
    if (d_ids.size() < 4) return;
    double h[8]{};
    if (!fit_homography(descr_xy, blobs, d_ids, b_ids, h)) return;

    const double pattern = finder_pattern_score(descr_xy, blobs, descr, h, s_img, 5, &search);
    double h180[8]{};
    homography_rotate_180(h, h180);
    const double pattern_180 = finder_pattern_score(descr_xy, blobs, descr, h180, s_img, 5, &search);

    const double margin = 0.0;
    CalplateObservation cand_out;
    double h_copy[8];
    for (int i = 0; i < 8; ++i) h_copy[i] = h[i];
    if (!refine_observation(descr_xy, blobs, descr, h_copy, 0.36 * s_img, need, width, height,
                            margin, cand_out, &search)) {
      return;
    }
    const double pattern2 = finder_pattern_score(descr_xy, blobs, descr, h_copy, s_img, 5, &search);
    double h180b[8]{};
    homography_rotate_180(h_copy, h180b);
    const double pattern2_180 = finder_pattern_score(descr_xy, blobs, descr, h180b, s_img, 5, &search);
    const double score = static_cast<double>(cand_out.indices.size()) + 4.0 * pattern2 - 0.05 * err;
    if (score > best_score) {
      best_score = score;
      best_pattern = pattern2;
      best_pattern_180 = pattern2_180;
      best_obs = std::move(cand_out);
      have = true;
    }
    (void)pattern;
    (void)pattern_180;
  };

  for (const auto& cand : centers) {
    const std::size_t c0 = cand.i;
    if (nbr[c0].size() < 3) continue;
    for (const std::size_t east : nbr[c0]) {
      const double ix = blobs[east].col - blobs[c0].col;
      const double iy = blobs[east].row - blobs[c0].row;
      if (std::hypot(ix, iy) < 0.4 * s_img) continue;

      std::array<Map, 2> maps{};
      maps[0].a = (ix * dx + iy * dy) / nd;
      maps[0].b = (iy * dx - ix * dy) / nd;
      maps[0].reflect = false;
      const double dy2 = -dy;
      const double nd2 = dx * dx + dy2 * dy2;
      maps[1].a = (ix * dx + iy * dy2) / nd2;
      maps[1].b = (iy * dx - ix * dy2) / nd2;
      maps[1].reflect = true;

      for (int mi = 0; mi < 2; ++mi) {
        std::vector<std::uint8_t> used(blobs.size(), 0);
        used[c0] = 1;
        std::vector<std::size_t> d_ids{finder_ids[0]};
        std::vector<std::size_t> b_ids{c0};
        double err = 0.0;
        for (std::size_t k = 1; k < finder_ids.size(); ++k) {
          double col = 0.0;
          double row = 0.0;
          apply(maps[static_cast<std::size_t>(mi)], descr_xy[finder_ids[k]], blobs[c0], col, row);
          const std::size_t b = nearest_index(blobs, row, col, 1.20 * s_img, used, &search);
          if (b >= blobs.size()) continue;
          err += std::hypot(blobs[b].row - row, blobs[b].col - col);
          used[b] = 1;
          d_ids.push_back(finder_ids[k]);
          b_ids.push_back(b);
        }
        consider(d_ids, b_ids, err);
      }

      const double ang0 = std::atan2(iy, ix);
      std::vector<std::pair<double, std::size_t>> img_ang;
      img_ang.reserve(nbr[c0].size());
      for (const std::size_t n : nbr[c0]) {
        const double a =
            wrap_pi(std::atan2(blobs[n].row - blobs[c0].row, blobs[n].col - blobs[c0].col) - ang0);
        img_ang.push_back({a, n});
      }
      std::vector<std::pair<double, std::size_t>> dsc_ang;
      dsc_ang.reserve(descr_ring.size());
      const double dang0 = std::atan2(descr_xy[east_id].y - p0.y, descr_xy[east_id].x - p0.x);
      for (const std::size_t id : descr_ring) {
        const double a = wrap_pi(std::atan2(descr_xy[id].y - p0.y, descr_xy[id].x - p0.x) - dang0);
        dsc_ang.push_back({a, id});
      }

      for (int reflect = 0; reflect < 2; ++reflect) {
        std::vector<std::size_t> ring_d{finder_ids[0], east_id};
        std::vector<std::size_t> ring_b{c0, east};
        std::vector<std::uint8_t> used(blobs.size(), 0);
        used[c0] = 1;
        used[east] = 1;
        for (const auto& dcell : dsc_ang) {
          if (dcell.second == east_id) continue;
          const double want = reflect != 0 ? -dcell.first : dcell.first;
          std::size_t best_n = blobs.size();
          double best_da = 0.55;
          for (const auto& icell : img_ang) {
            if (used[icell.second] != 0) continue;
            const double da = std::abs(wrap_pi(icell.first - want));
            if (da < best_da) {
              best_da = da;
              best_n = icell.second;
            }
          }
          if (best_n >= blobs.size()) continue;
          used[best_n] = 1;
          ring_d.push_back(dcell.second);
          ring_b.push_back(best_n);
        }
        if (ring_d.size() < 4) continue;
        double h_local[8]{};
        if (!fit_homography(descr_xy, blobs, ring_d, ring_b, h_local)) continue;

        std::vector<std::size_t> d_ids{finder_ids[0]};
        std::vector<std::size_t> b_ids{c0};
        std::vector<std::uint8_t> fused(blobs.size(), 0);
        fused[c0] = 1;
        double err = 0.0;
        for (std::size_t k = 1; k < finder_ids.size(); ++k) {
          double col = 0.0;
          double row = 0.0;
          project_h(h_local, descr_xy[finder_ids[k]].x, descr_xy[finder_ids[k]].y, col, row);
          const std::size_t b = nearest_index(blobs, row, col, 2.00 * s_img, fused, &search);
          if (b >= blobs.size()) continue;
          err += std::hypot(blobs[b].row - row, blobs[b].col - col);
          fused[b] = 1;
          d_ids.push_back(finder_ids[k]);
          b_ids.push_back(b);
        }
        consider(d_ids, b_ids, err);
      }
    }
  }
  if (!have) return false;
  // 唯一的五定位孔模式打破晶格 180° 对称；要求明确余量，防止位姿相关 fy<0/East-only 检查翻转标签。
  if (!(best_pattern > best_pattern_180 + 40.0)) return false;
  out = std::move(best_obs);
  if (score_out != nullptr) *score_out = best_score;
  return true;
}

double median_nearest_spacing(const std::vector<double>& rows, const std::vector<double>& columns) {
  const std::size_t n = rows.size();
  if (n < 2) return 0.0;
  std::vector<double> nns;
  nns.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < n; ++j) {
      if (i == j) continue;
      best = std::min(best, std::hypot(rows[i] - rows[j], columns[i] - columns[j]));
    }
    if (std::isfinite(best)) nns.push_back(best);
  }
  if (nns.empty()) return 0.0;
  const std::size_t mid = nns.size() / 2;
  std::nth_element(nns.begin(), nns.begin() + static_cast<std::ptrdiff_t>(mid), nns.end());
  if (nns.size() % 2 == 1) return nns[mid];
  const double upper = nns[mid];
  std::nth_element(nns.begin(), nns.begin() + static_cast<std::ptrdiff_t>(mid - 1), nns.end());
  return 0.5 * (upper + nns[mid - 1]);
}

bool project_camera_ok(const CameraParameters& cam, const Point3d& cam_pt, Point2d& out) {
  if (is_perspective(cam) && !(cam_pt.z > 1e-12)) return false;
  try {
    out = project_camera_point(cam, cam_pt);
  } catch (const core::Exception&) {
    return false;
  }
  return std::isfinite(out.row) && std::isfinite(out.col);
}

bool project_plate_point(const CameraParameters& cam, const Pose3d& pose, const Point3d& plate,
                         Point2d& out) {
  return project_camera_ok(cam, transform_point(pose, plate), out);
}

bool fit_ellipse_center(const std::vector<Point2d>& pts, Point2d& center, double max_from_mean = 6.0) {
  const std::size_t n = pts.size();
  if (n < 8) return false;
  double mean_c = 0.0;
  double mean_r = 0.0;
  for (const auto& p : pts) {
    mean_c += p.col;
    mean_r += p.row;
  }
  const double inv = 1.0 / static_cast<double>(n);
  mean_c *= inv;
  mean_r *= inv;
  double AtA[5][5]{};
  double Atb[5]{};
  for (const auto& p : pts) {
    const double x = p.col - mean_c;
    const double y = p.row - mean_r;
    const double d[5] = {x * x, x * y, y * y, x, y};
    for (int i = 0; i < 5; ++i) {
      Atb[i] += d[i];
      for (int j = 0; j < 5; ++j) AtA[i][j] += d[i] * d[j];
    }
  }
  double coef[5]{};
  if (!solve5(AtA, Atb, coef)) return false;
  const double a = coef[0];
  const double b = coef[1];
  const double c = coef[2];
  const double d = coef[3];
  const double e = coef[4];
  const double den = b * b - 4.0 * a * c;
  if (!(den < -1e-14)) return false;
  center.col = mean_c + (2.0 * c * d - b * e) / den;
  center.row = mean_r + (2.0 * a * e - b * d) / den;
  if (!std::isfinite(center.row) || !std::isfinite(center.col)) return false;
  if (std::hypot(center.col - mean_c, center.row - mean_r) > max_from_mean) return false;
  return true;
}

bool bilinear_gray8(const std::uint8_t* pixels, int width, int height, int stride, double row,
                    double col, double& value) {
  if (pixels == nullptr) return false;
  if (!(row >= 0.0 && col >= 0.0 && row < static_cast<double>(height - 1) &&
        col < static_cast<double>(width - 1))) {
    return false;
  }
  const int r0 = static_cast<int>(row);
  const int c0 = static_cast<int>(col);
  const int r1 = r0 + 1;
  const int c1 = c0 + 1;
  const double fr = row - static_cast<double>(r0);
  const double fc = col - static_cast<double>(c0);
  const double i00 = static_cast<double>(pixels[r0 * stride + c0]);
  const double i01 = static_cast<double>(pixels[r0 * stride + c1]);
  const double i10 = static_cast<double>(pixels[r1 * stride + c0]);
  const double i11 = static_cast<double>(pixels[r1 * stride + c1]);
  value = (1.0 - fr) * ((1.0 - fc) * i00 + fc * i01) + fr * ((1.0 - fc) * i10 + fc * i11);
  return true;
}

bool sample_outer_edge_on_ray(const std::uint8_t* pixels, int width, int height, int stride,
                              bool light_on_dark, double seed_row, double seed_col, double dr,
                              double dc, double t_lo, double t_hi, Point2d& edge) {
  constexpr double kDt = 0.25;
  constexpr double kMinGrad = 12.0;
  constexpr int kMaxSamples = 256;
  double samples[kMaxSamples];
  int n = 0;
  for (double t = t_lo; t <= t_hi + 1e-12; t += kDt) {
    if (n >= kMaxSamples) break;
    double v = 0.0;
    if (!bilinear_gray8(pixels, width, height, stride, seed_row + t * dr, seed_col + t * dc, v)) {
      break;
    }
    samples[n++] = v;
  }
  if (n < 4) return false;
  const int n_d = n - 1;
  auto score_at = [&](int i) -> double {
    const double d_i = (samples[i + 1] - samples[i]) / kDt;
    return light_on_dark ? -d_i : d_i;
  };
  // 外缘取最外侧强局部峰，而非全局 |dI| 最大值；孔洞及内部阴影沿射线也呈亮→暗。
  int best = -1;
  for (int i = n_d - 2; i >= 1; --i) {
    const double score = score_at(i);
    if (!(score >= kMinGrad)) continue;
    if (score >= score_at(i - 1) && score >= score_at(i + 1)) {
      best = i;
      break;
    }
  }
  if (best < 1 || best > n_d - 2) return false;

  const double ym1 = (samples[best] - samples[best - 1]) / kDt;
  const double y0 = (samples[best + 1] - samples[best]) / kDt;
  const double yp1 = (samples[best + 2] - samples[best + 1]) / kDt;
  const double denom = ym1 - 2.0 * y0 + yp1;
  double offset = 0.0;
  if (std::abs(denom) > 1e-12) offset = 0.5 * (ym1 - yp1) / denom;
  offset = std::clamp(offset, -0.75, 0.75);
  const double t_edge = t_lo + (static_cast<double>(best) + 0.5 + offset) * kDt;
  if (!(t_edge >= t_lo && t_edge <= t_hi)) return false;
  edge.row = seed_row + t_edge * dr;
  edge.col = seed_col + t_edge * dc;
  return std::isfinite(edge.row) && std::isfinite(edge.col);
}

bool refine_mark_outer_edge_ellipse(const std::uint8_t* pixels, int width, int height, int stride,
                                    bool light_on_dark, double seed_row, double seed_col, double r_px,
                                    double& out_row, double& out_col,
                                    std::size_t min_pts = kOuterEdgeMinRays, double max_shift = 1.5,
                                    double max_from_mean = 6.0) {
  if (!(r_px >= 1.5)) return false;
  const double t_lo = kOuterEdgeRayLo * r_px;
  const double t_hi = kOuterEdgeRayHi * r_px;
  if (!(t_hi > t_lo + 0.5)) return false;
  std::vector<Point2d> edges;
  edges.reserve(static_cast<std::size_t>(kOuterEdgeRays));
  for (int k = 0; k < kOuterEdgeRays; ++k) {
    const double a = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(kOuterEdgeRays);
    Point2d edge;
    if (!sample_outer_edge_on_ray(pixels, width, height, stride, light_on_dark, seed_row, seed_col,
                                  std::sin(a), std::cos(a), t_lo, t_hi, edge)) {
      continue;
    }
    edges.push_back(edge);
  }
  if (edges.size() < min_pts) return false;
  Point2d center;
  if (!fit_ellipse_center(edges, center, max_from_mean)) return false;
  if (std::hypot(center.row - seed_row, center.col - seed_col) > max_shift) return false;
  out_row = center.row;
  out_col = center.col;
  return true;
}

double projected_mark_radius_px(const CameraParameters& cam, const Pose3d& pose, const Point3d& plate,
                                double radius) {
  if (!(radius > 0.0)) return 0.0;
  Point2d p0;
  Point2d p1;
  if (!project_plate_point(cam, pose, plate, p0)) return 0.0;
  if (!project_plate_point(cam, pose, {plate.x + radius, plate.y, plate.z}, p1)) return 0.0;
  const double r_px = std::hypot(p1.row - p0.row, p1.col - p0.col);
  return std::isfinite(r_px) ? r_px : 0.0;
}

bool center_inside_image(double row, double col, int width, int height, double inset) {
  return row >= inset && col >= inset && row < static_cast<double>(height) - inset &&
         col < static_cast<double>(width) - inset;
}

int observed_hex_neighbors(std::size_t index, std::int64_t rows, std::int64_t columns,
                           const std::vector<std::uint8_t>& have) {
  if (columns < 1 || rows < 1) return 0;
  const int r = static_cast<int>(index / static_cast<std::size_t>(columns));
  const int c = static_cast<int>(index % static_cast<std::size_t>(columns));
  int n = 0;
  for (int dir = 0; dir < 6; ++dir) {
    int nr = 0;
    int nc = 0;
    hex_neighbor_rc(r, c, dir, nr, nc);
    if (nr < 0 || nc < 0 || nr >= rows || nc >= columns) continue;
    const auto id = static_cast<std::size_t>(nr) * static_cast<std::size_t>(columns) +
                    static_cast<std::size_t>(nc);
    if (id < have.size() && have[id] != 0) ++n;
  }
  return n;
}

void fill_predicted_calplate_marks(const std::uint8_t* pixels, int width, int height, int stride,
                                   const CameraParameters& cam, const CalplateDescription& descr,
                                   CalibrationObservation& obs) {
  if (!obs.initial_pose.has_value() || pixels == nullptr) return;
  if (descr.rows < 1 || descr.columns < 1) return;
  const Pose3d& pose = *obs.initial_pose;
  std::vector<std::uint8_t> have(descr.points.size(), 0);
  for (const auto idx : obs.indices) {
    if (idx >= 0 && static_cast<std::size_t>(idx) < have.size()) have[static_cast<std::size_t>(idx)] = 1;
  }
  const double spacing = median_nearest_spacing(obs.rows, obs.columns);
  const double min_sep = 0.35 * spacing;
  const double r_fallback = 0.5 * spacing;

  // 仅从实测晶格扩展；孤立投影边缘行有两个邻居，裁剪后的板角至少三侧相连。
  for (int pass = 0; pass < 8; ++pass) {
    bool added = false;
    for (std::size_t i = 0; i < descr.points.size(); ++i) {
      if (have[i] != 0) continue;
      if (observed_hex_neighbors(i, descr.rows, descr.columns, have) < 3) continue;
      Point2d proj;
      if (!project_plate_point(cam, pose, descr.points[i], proj)) continue;
      // HALCON 只要求标记中心在图像内，即使圆盘被裁剪；内缩约 2 px 保证种子周围双线性采样有效。
      if (!center_inside_image(proj.row, proj.col, width, height, kFillCenterInsetPx)) continue;

      bool too_close = false;
      for (std::size_t k = 0; k < obs.rows.size(); ++k) {
        if (std::hypot(proj.row - obs.rows[k], proj.col - obs.columns[k]) < min_sep) {
          too_close = true;
          break;
        }
      }
      if (too_close) continue;

      double r_px = r_fallback;
      if (i < descr.radii.size()) {
        const double rp = projected_mark_radius_px(cam, pose, descr.points[i], descr.radii[i]);
        if (rp > 1.2) r_px = rp;
      }
      if (!(r_px >= 1.5)) continue;
      r_px = std::min(r_px, 64.0);

      double nr = proj.row;
      double nc = proj.col;
      if (!refine_mark_outer_edge_ellipse(pixels, width, height, stride, descr.light_on_dark, proj.row,
                                          proj.col, r_px, nr, nc, kFillMinRays, 2.0, 32.0)) {
        continue;
      }
      // 裁剪圆盘的种子可在图内约 2 px；贴框拟合中心对应 HALCON 省略的标记。
      // 用 min(width,height) 的约 1.5% 匹配该集合，不使用 10.5 px 匹配边距。
      const double accept_inset =
          std::max(kFillCenterInsetPx, kFillAcceptInsetFrac * static_cast<double>((std::min)(width, height)));
      if (!center_inside_image(nr, nc, width, height, accept_inset)) continue;

      obs.rows.push_back(nr);
      obs.columns.push_back(nc);
      obs.indices.push_back(static_cast<std::int64_t>(i));
      have[i] = 1;
      added = true;
    }
    if (!added) break;
  }
}

void estimate_calplate_pose(const CameraParameters& cam, const std::vector<Point3d>& points,
                            CalibrationObservation& obs) {
  if (is_perspective(cam)) {
    obs.initial_pose = estimate_planar_pose_perspective(cam, points, obs.rows, obs.columns, obs.indices);
  } else {
    obs.initial_pose = estimate_planar_pose_telecentric(cam, points, obs.rows, obs.columns, obs.indices);
  }
}

}  // namespace

void refine_calplate_observation(const core::Image& image, const CalplateDescription& descr,
                                 const CameraParameters* camera, CalibrationObservation& obs) {
  if (camera != nullptr) {
    estimate_calplate_pose(*camera, descr.points, obs);
  }
  if (image.empty() || image.pixel_type() != core::PixelType::UInt8) return;
  const auto* pixels = static_cast<const std::uint8_t*>(image.data());
  if (pixels == nullptr) return;
  const int width = static_cast<int>(image.width());
  const int height = static_cast<int>(image.height());
  const int stride = static_cast<int>(image.stride());

  if (camera != nullptr) {
    fill_predicted_calplate_marks(pixels, width, height, stride, *camera, descr, obs);
    estimate_calplate_pose(*camera, descr.points, obs);
  }

  const Pose3d* pose = obs.initial_pose.has_value() ? &*obs.initial_pose : nullptr;
  const double r_fallback = 0.5 * median_nearest_spacing(obs.rows, obs.columns);
  for (std::size_t k = 0; k < obs.indices.size(); ++k) {
    if (obs.indices[k] < 0) continue;
    const auto i = static_cast<std::size_t>(obs.indices[k]);
    double r_px = r_fallback;
    if (camera != nullptr && pose != nullptr && i < descr.points.size() && i < descr.radii.size()) {
      const double rp = projected_mark_radius_px(*camera, *pose, descr.points[i], descr.radii[i]);
      if (rp > 1.2) r_px = rp;
    }
    if (!(r_px >= 1.5)) continue;
    r_px = std::min(r_px, 64.0);
    double nr = obs.rows[k];
    double nc = obs.columns[k];
    if (refine_mark_outer_edge_ellipse(pixels, width, height, stride, descr.light_on_dark,
                                       obs.rows[k], obs.columns[k], r_px, nr, nc)) {
      obs.rows[k] = nr;
      obs.columns[k] = nc;
    }
  }
}

CalplateObservation detect_calplate(const core::Image& image, const CalplateDescription& descr) {
  if (image.empty() || image.pixel_type() != core::PixelType::UInt8) {
    fail_arg("Gray8 image required");
  }
  if (descr.finders.empty() || descr.columns < 1 || descr.rows < 1) {
    fail_not_found("calplate description missing finders or grid size");
  }

  const auto* pixels = static_cast<const std::uint8_t*>(image.data());
  const std::size_t width = image.width();
  const std::size_t height = image.height();
  const std::size_t stride = image.stride();
  const bool light = descr.light_on_dark;
  const std::size_t need = min_required_marks(descr.points.size());
  const auto finder_ids = finder_point_indices(descr);
  if (finder_ids.empty()) fail_not_found("finder is outside the calplate grid");
  const bool use_chirality = finder_ids.size() <= 1;
  const std::size_t min_finder_hits = finder_ids.size() >= 5 ? finder_ids.size() - 1 : 1;
  const std::size_t max_area =
      std::max<std::size_t>(400, static_cast<std::size_t>(width * height / 200));

  std::uint8_t lo = 255;
  std::uint8_t hi = 0;
  for (std::size_t r = 0; r < height; ++r) {
    const auto* row = pixels + r * stride;
    for (std::size_t c = 0; c < width; ++c) {
      lo = std::min(lo, row[c]);
      hi = std::max(hi, row[c]);
    }
  }
  const double mid = 0.5 * (static_cast<double>(lo) + static_cast<double>(hi));
  const double otsu = static_cast<double>(otsu_threshold(pixels, width, height, stride));
  const double sign = light ? 1.0 : -1.0;
  const std::array<double, 8> finder_thresholds{
      otsu, mid + sign * 15.0, otsu + sign * 10.0, mid + sign * 22.0, otsu - sign * 8.0,
      mid + sign * 8.0, otsu + sign * 18.0, mid + sign * 30.0};

  std::vector<Vec2> descr_xy;
  descr_xy.reserve(descr.points.size());
  for (const auto& p : descr.points) descr_xy.push_back({p.x, p.y});
  const double s_descr = median_nn_spacing(descr_xy);
  const auto descr_adj = lattice_graph(descr_xy, s_descr);

  if (finder_ids.size() >= 5) {
    CalplateObservation best_found;
    double best_score = -1e100;
    bool have = false;
    for (double threshold : finder_thresholds) {
      auto blobs = extract_blobs(pixels, width, height, stride, light, threshold, 3, max_area);
      if (blobs.size() < 4) continue;
      CalplateObservation cand;
      double score = -1e100;
      if (detect_with_hole_finders(blobs, descr_xy, descr, finder_ids, need,
                                  static_cast<double>(width), static_cast<double>(height), cand,
                                  &score) &&
          score > best_score) {
        best_score = score;
        best_found = std::move(cand);
        have = true;
      }
    }
    if (have) {
      refine_gray_marks(pixels, width, height, stride, light, descr, descr_xy, best_found);
      return best_found;
    }
    fail_not_found("failed to match official calplate finder constellation");
  }

  const std::array<double, 1> hex_thresholds{mid + sign * 15.0};
  const bool use_east = false;
  const std::size_t east_descr =
      use_east ? static_cast<std::size_t>(descr.finders[0].row * descr.columns +
                                          descr.finders[0].column + 1)
               : descr.points.size();
  const std::size_t west_descr =
      use_east ? static_cast<std::size_t>(descr.finders[0].row * descr.columns +
                                          descr.finders[0].column - 1)
               : descr.points.size();

  std::vector<std::size_t> best_blob;
  std::vector<std::size_t> best_descr;
  std::vector<Blob> best_blob_store;
  std::size_t best_count = 0;
  std::size_t best_hits = 0;
  double best_rms = std::numeric_limits<double>::infinity();
  double best_fy = 0.0;
  double best_east_pref = -1e100;
  bool done = false;

  for (double threshold : hex_thresholds) {
    if (done) break;
    auto blobs = extract_blobs(pixels, width, height, stride, light, threshold, 3, max_area);
    if (blobs.size() < 4) continue;

    std::vector<Vec2> blob_xy;
    blob_xy.reserve(blobs.size());
    for (const auto& b : blobs) blob_xy.push_back({b.col, b.row});
    const double s_img = median_nn_spacing(blob_xy);
    const auto blob_adj = lattice_graph(blob_xy, s_img);
    std::vector<std::size_t> origins;
    std::size_t seed = largest_component_seed(blob_adj);
    if (use_chirality) {
      seed = select_seed_blob(blob_xy, blob_adj, descr_xy, descr_adj, finder_ids[0],
                              finder_ids.size());
      origins.push_back(seed);
    }
    const auto blob_lab = label_hex_axial(blob_xy, blob_adj, seed);
    if (!use_chirality) {
      for (std::size_t b = 0; b < blob_lab.size(); ++b) {
        if (blob_lab[b].ok && blobs[b].hole && blob_adj[b].size() >= 4) origins.push_back(b);
      }
      if (origins.empty()) {
        for (std::size_t b = 0; b < blob_lab.size(); ++b) {
          if (blob_lab[b].ok && blob_adj[b].size() >= 5) origins.push_back(b);
        }
      }
    }

    for (const std::size_t finder_index : finder_ids) {
      if (done) break;
      const auto descr_lab = label_hex_axial(descr_xy, descr_adj, finder_index);
      if (!descr_lab[finder_index].ok) continue;
      std::map<std::pair<int, int>, std::size_t> descr_at;
      for (std::size_t i = 0; i < descr_lab.size(); ++i) {
        if (!descr_lab[i].ok) continue;
        descr_at.emplace(std::make_pair(descr_lab[i].q, descr_lab[i].r), i);
      }
      std::vector<std::pair<int, int>> finder_qr;
      finder_qr.reserve(finder_ids.size());
      for (const std::size_t id : finder_ids) {
        if (id < descr_lab.size() && descr_lab[id].ok) {
          finder_qr.emplace_back(descr_lab[id].q, descr_lab[id].r);
        }
      }

      for (const std::size_t origin : origins) {
        if (done) break;
        if (origin >= blob_lab.size() || !blob_lab[origin].ok) continue;
        const int q0 = blob_lab[origin].q;
        const int r0 = blob_lab[origin].r;
        for (int reflect = 0; reflect < 2; ++reflect) {
          if (done) break;
          for (int rot = 0; rot < 6; ++rot) {
            std::map<std::pair<int, int>, std::size_t> blob_at;
            std::vector<std::uint8_t> used_descr(descr.points.size(), 0);
            std::vector<std::size_t> mb;
            std::vector<std::size_t> md;
            mb.reserve(blobs.size());
            md.reserve(blobs.size());
            for (std::size_t b = 0; b < blob_lab.size(); ++b) {
              if (!blob_lab[b].ok) continue;
              int q = blob_lab[b].q - q0;
              int r = blob_lab[b].r - r0;
              if (reflect != 0) std::swap(q, r);
              rotate_axial(q, r, rot);
              blob_at.emplace(std::make_pair(q, r), b);
              const auto it = descr_at.find(std::make_pair(q, r));
              if (it == descr_at.end()) continue;
              const std::size_t di = it->second;
              if (used_descr[di] != 0) continue;
              used_descr[di] = 1;
              mb.push_back(b);
              md.push_back(di);
            }
            std::size_t hits = 0;
            for (const auto& qr : finder_qr) {
              if (blob_at.find(qr) != blob_at.end()) ++hits;
            }
            if (hits < min_finder_hits) continue;
            double rms = std::numeric_limits<double>::infinity();
            double fy = 0.0;
            if (!score_affine(descr_xy, blobs, mb, md, rms, fy)) {
              rms = std::numeric_limits<double>::infinity();
              fy = 0.0;
            }
            double east_pref = 0.0;
            if (use_east) {
              double east_score = 0.0;
              double west_score = 0.0;
              for (std::size_t k = 0; k < md.size(); ++k) {
                if (md[k] == east_descr) east_score = blobs[mb[k]].hole_score;
                if (md[k] == west_descr) west_score = blobs[mb[k]].hole_score;
              }
              east_pref = west_score - east_score;
            }
            if (better_ranked_match(hits, mb.size(), rms, fy, east_pref, best_hits, best_count, best_rms,
                                    best_fy, best_east_pref, use_chirality, use_east)) {
              best_hits = hits;
              best_count = mb.size();
              best_rms = rms;
              best_fy = fy;
              best_east_pref = east_pref;
              best_blob = std::move(mb);
              best_descr = std::move(md);
              best_blob_store = blobs;
              if (best_hits >= finder_ids.size() && best_count >= need && best_rms < 2.0 &&
                  !use_chirality && !use_east) {
                done = true;
                break;
              }
            }
          }
        }
      }
    }
  }

  if (best_count < need || best_blob_store.empty()) fail_not_found("too few calplate marks");

  CalplateObservation found;
  found.rows.reserve(best_count);
  found.columns.reserve(best_count);
  found.indices.reserve(best_count);
  for (std::size_t k = 0; k < best_count; ++k) {
    const auto& blob = best_blob_store[best_blob[k]];
    if (blob.clipped) continue;
    found.rows.push_back(blob.row);
    found.columns.push_back(blob.col);
    found.indices.push_back(static_cast<std::int64_t>(best_descr[k]));
  }
  if (found.indices.size() < 4) fail_not_found("too few calplate marks after border filter");
  refine_gray_marks(pixels, width, height, stride, light, descr, descr_xy, found);
  return found;
}

}  // namespace mv::calibration
