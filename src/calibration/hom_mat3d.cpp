#include <calibration/hom_mat3d.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <calibration/geometry3d.hpp>
#include <core/broadcast.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

void require(bool ok, const char* op, const char* parameter, const char* message) {
  if (!ok) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op,
                          parameter, -1, message);
  }
}

void require_finite(double v, const char* op, const char* parameter) {
  require(std::isfinite(v), op, parameter, "finite values required");
}

double det3(const std::array<double, 12>& m) {
  return m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) +
         m[2] * (m[4] * m[9] - m[5] * m[8]);
}

std::array<double, 9> rotation_part(const std::array<double, 12>& m) {
  return {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]};
}

std::array<double, 12> multiply(const std::array<double, 12>& a, const std::array<double, 12>& b) {
  std::array<double, 12> r{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      r[row * 4 + col] = a[row * 4 + 0] * b[0 * 4 + col] + a[row * 4 + 1] * b[1 * 4 + col] +
                         a[row * 4 + 2] * b[2 * 4 + col];
    }
    r[row * 4 + 3] = a[row * 4 + 0] * b[3] + a[row * 4 + 1] * b[7] + a[row * 4 + 2] * b[11] +
                     a[row * 4 + 3];
  }
  return r;
}

}  // namespace

std::array<double, 12> parse_hom_mat3d(const core::Tuple& hom_mat3d, const char* op, bool require_inverse) {
  if (hom_mat3d.size() != 12) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, op,
                          "HomMat3D", -1, "homogeneous matrix must contain twelve coefficients");
  }
  std::array<double, 12> m{};
  for (std::size_t i = 0; i < 12; ++i) {
    try {
      m[i] = hom_mat3d.as_real64(i);
    } catch (const core::Exception&) {
      throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op,
                            "HomMat3D", static_cast<std::int64_t>(i),
                            "matrix coefficients must be numeric");
    }
    require_finite(m[i], op, "HomMat3D");
  }
  if (require_inverse)
    require(std::isfinite(det3(m)) && det3(m) != 0.0, op, "HomMat3D",
            "singular or nonfinite affine matrix");
  return m;
}

core::Tuple hom_mat3d_from_values(const std::array<double, 12>& values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      throw core::Exception(core::Status::InvalidArgument, "hom_mat3d",
                            "affine result is not finite");
    }
  }
  // 固定十二项一次构造，避免逐项追加导致多次扩容。
  return core::Tuple{std::vector<double>{values.begin(), values.end()}};
}

core::Tuple pose_to_hom_mat3d(const core::Tuple& pose) {
  const auto p = pose_from_halcon_tuple(pose);
  const auto r = rotation_matrix(p.rotation);
  return hom_mat3d_from_values({r[0], r[1], r[2], p.translation.x, r[3], r[4], r[5],
                                p.translation.y, r[6], r[7], r[8], p.translation.z});
}

core::Tuple hom_mat3d_to_pose(const core::Tuple& hom_mat3d) {
  constexpr auto op = "hom_mat3d_to_pose";
  const auto m = parse_hom_mat3d(hom_mat3d, op);
  const auto r = rotation_part(m);
  double max_off = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double acc = 0.0;
      for (int k = 0; k < 3; ++k) acc += r[i * 3 + k] * r[j * 3 + k];
      max_off = std::max(max_off, std::abs(acc - (i == j ? 1.0 : 0.0)));
    }
  }
  const double d = det3(m);
  if (max_off > 1e-6 || std::abs(d - 1.0) > 1e-6) {
    throw core::Exception(core::Status::InvalidArgument, op,
                          "general affine cannot be converted to a rigid pose");
  }
  Pose3d pose{{m[3], m[7], m[11]}, quaternion_from_rotation_matrix(r)};
  return pose_to_halcon_tuple(pose);
}

core::Tuple hom_mat3d_compose(const core::Tuple& left, const core::Tuple& right) {
  constexpr auto op = "hom_mat3d_compose";
  return hom_mat3d_from_values(
      multiply(parse_hom_mat3d(left, op), parse_hom_mat3d(right, op)));
}

core::Tuple hom_mat3d_invert(const core::Tuple& hom_mat3d) {
  constexpr auto op = "hom_mat3d_invert";
  const auto m = parse_hom_mat3d(hom_mat3d, op);
  const double d = det3(m);
  const double i00 = (m[5] * m[10] - m[6] * m[9]) / d;
  const double i01 = (m[2] * m[9] - m[1] * m[10]) / d;
  const double i02 = (m[1] * m[6] - m[2] * m[5]) / d;
  const double i10 = (m[6] * m[8] - m[4] * m[10]) / d;
  const double i11 = (m[0] * m[10] - m[2] * m[8]) / d;
  const double i12 = (m[2] * m[4] - m[0] * m[6]) / d;
  const double i20 = (m[4] * m[9] - m[5] * m[8]) / d;
  const double i21 = (m[1] * m[8] - m[0] * m[9]) / d;
  const double i22 = (m[0] * m[5] - m[1] * m[4]) / d;
  const double tx = -(i00 * m[3] + i01 * m[7] + i02 * m[11]);
  const double ty = -(i10 * m[3] + i11 * m[7] + i12 * m[11]);
  const double tz = -(i20 * m[3] + i21 * m[7] + i22 * m[11]);
  return hom_mat3d_from_values({i00, i01, i02, tx, i10, i11, i12, ty, i20, i21, i22, tz});
}

void affine_trans_point_3d(const core::Tuple& hom_mat3d, const core::Tuple& px, const core::Tuple& py,
                           const core::Tuple& pz, core::Tuple* qx, core::Tuple* qy, core::Tuple* qz) {
  constexpr auto op = "affine_trans_point_3d";
  if (qx == nullptr || qy == nullptr || qz == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op,
                          "outputs", -1, "outputs must not be null");
  }
  if (qx == qy || qx == qz || qy == qz) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op,
                          "outputs", -1, "parallel Tuple outputs must be distinct");
  }
  const auto m = parse_hom_mat3d(hom_mat3d, op);
  const auto plan = core::BroadcastPlan::make(
      {{px.size(), "Px", true}, {py.size(), "Py", true}, {pz.size(), "Pz", true}}, op);
  const bool real_coordinates = px.kind() == core::TupleKind::Real64 && py.kind() == core::TupleKind::Real64 && pz.kind() == core::TupleKind::Real64;
  const auto x_values = real_coordinates ? px.real64_span() : std::span<const double>{};
  const bool broadcast_x = px.size() == 1;
  const auto y_values = real_coordinates ? py.real64_span() : std::span<const double>{};
  const bool broadcast_y = py.size() == 1;
  const auto z_values = real_coordinates ? pz.real64_span() : std::span<const double>{};
  const bool broadcast_z = pz.size() == 1;
  std::vector<double> ox;
  ox.reserve(plan.size());
  std::vector<double> oy;
  oy.reserve(plan.size());
  std::vector<double> oz;
  oz.reserve(plan.size());
  // 同质数值和通用输入只分派一次；定长结果去除逐点追加的容量分支。
  ox.resize(plan.size());
  oy.resize(plan.size());
  oz.resize(plan.size());
  const auto transform = [&]<bool Real>() {
    for (std::size_t i = 0; i < plan.size(); ++i) {
      double x = 0;
      double y = 0;
      double z = 0;
      try {
        if constexpr (Real) {
          x = x_values[broadcast_x ? 0 : i];
          y = y_values[broadcast_y ? 0 : i];
          z = z_values[broadcast_z ? 0 : i];
        } else {
          x = px.as_real64(plan.source_index(0, i));
          y = py.as_real64(plan.source_index(1, i));
          z = pz.as_real64(plan.source_index(2, i));
        }
      } catch (const core::Exception&) {
        throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, "Px",
                              static_cast<std::int64_t>(i), "point coordinates must be numeric");
      }
      require_finite(x, op, "Px");
      require_finite(y, op, "Py");
      require_finite(z, op, "Pz");
      const double xo = m[0] * x + m[1] * y + m[2] * z + m[3];
      const double yo = m[4] * x + m[5] * y + m[6] * z + m[7];
      const double zo = m[8] * x + m[9] * y + m[10] * z + m[11];
      require(std::isfinite(xo) && std::isfinite(yo) && std::isfinite(zo), op, "HomMat3D",
              "transformed point is not finite");
      ox[i] = xo;
      oy[i] = yo;
      oz[i] = zo;
    }
  };
  if (real_coordinates) transform.template operator()<true>();
  else transform.template operator()<false>();
  *qx = ox.empty() ? core::Tuple{} : core::Tuple{std::move(ox)};
  *qy = oy.empty() ? core::Tuple{} : core::Tuple{std::move(oy)};
  *qz = oz.empty() ? core::Tuple{} : core::Tuple{std::move(oz)};
}

}  // namespace mv::calibration
