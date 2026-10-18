#include <geometry/hom_mat2d.hpp>

#include <cmath>
#include <utility>
#include <vector>

#include <core/broadcast.hpp>
#include <core/exception.hpp>
#if MVKIT_GEOMETRY_HAS_AVX2
#include <geometry/analytic2d_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>
#endif

namespace mv::geometry {
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

std::array<double, 6> multiply(const std::array<double, 6>& left,
                               const std::array<double, 6>& right) {
  return {left[0] * right[0] + left[1] * right[3], left[0] * right[1] + left[1] * right[4],
          left[0] * right[2] + left[1] * right[5] + left[2],
          left[3] * right[0] + left[4] * right[3], left[3] * right[1] + left[4] * right[4],
          left[3] * right[2] + left[4] * right[5] + left[5]};
}

double det(const std::array<double, 6>& m) { return m[0] * m[4] - m[1] * m[3]; }

void require_nonsingular(const std::array<double, 6>& m, const char* op) {
  const double d = det(m);
  require(std::isfinite(d) && d != 0.0, op, "HomMat2D", "singular or nonfinite affine matrix");
}

}  // namespace

std::array<double, 6> parse_hom_mat2d(const core::Tuple& hom_mat2d, const char* op) {
  if (hom_mat2d.size() != 6) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, op,
                          "HomMat2D", -1, "affine matrix must contain six coefficients");
  }
  std::array<double, 6> m{};
  for (std::size_t i = 0; i < 6; ++i) {
    try {
      m[i] = hom_mat2d.as_real64(i);
    } catch (const core::Exception&) {
      throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op,
                            "HomMat2D", static_cast<std::int64_t>(i),
                            "affine coefficients must be numeric");
    }
    require_finite(m[i], op, "HomMat2D");
  }
  require_nonsingular(m, op);
  return m;
}

core::Tuple hom_mat2d_from_values(const std::array<double, 6>& values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      throw core::Exception(core::Status::InvalidArgument, "hom_mat2d",
                            "affine result is not finite");
    }
  }
  return core::Tuple{std::vector<double>{values.begin(),values.end()}};
}

core::Tuple hom_mat2d_identity() { return hom_mat2d_from_values({1.0, 0.0, 0.0, 0.0, 1.0, 0.0}); }

core::Tuple hom_mat2d_translate(const core::Tuple& hom_mat2d, double tx, double ty) {
  constexpr auto op = "hom_mat2d_translate";
  require_finite(tx, op, "Tx");
  require_finite(ty, op, "Ty");
  const auto m = parse_hom_mat2d(hom_mat2d, op);
  return hom_mat2d_from_values(multiply({1.0, 0.0, tx, 0.0, 1.0, ty}, m));
}

core::Tuple hom_mat2d_rotate(const core::Tuple& hom_mat2d, double phi, double px, double py) {
  constexpr auto op = "hom_mat2d_rotate";
  require_finite(phi, op, "Phi");
  require_finite(px, op, "Px");
  require_finite(py, op, "Py");
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  // 固定点 (Px,Py) 位于 x=column、y=row；phi 为该平面内的数学正方向。
  const std::array<double, 6> r{c, -s, px - c * px + s * py, s, c, py - s * px - c * py};
  return hom_mat2d_from_values(multiply(r, parse_hom_mat2d(hom_mat2d, op)));
}

core::Tuple hom_mat2d_scale(const core::Tuple& hom_mat2d, double sx, double sy, double px,
                            double py) {
  constexpr auto op = "hom_mat2d_scale";
  require_finite(sx, op, "Sx");
  require_finite(sy, op, "Sy");
  require_finite(px, op, "Px");
  require_finite(py, op, "Py");
  require(sx != 0.0 && sy != 0.0, op, "Sx", "scale factors must be nonzero");
  const std::array<double, 6> s{sx, 0.0, px - sx * px, 0.0, sy, py - sy * py};
  return hom_mat2d_from_values(multiply(s, parse_hom_mat2d(hom_mat2d, op)));
}

core::Tuple hom_mat2d_compose(const core::Tuple& left, const core::Tuple& right) {
  constexpr auto op = "hom_mat2d_compose";
  return hom_mat2d_from_values(
      multiply(parse_hom_mat2d(left, op), parse_hom_mat2d(right, op)));
}

core::Tuple hom_mat2d_invert(const core::Tuple& hom_mat2d) {
  constexpr auto op = "hom_mat2d_invert";
  const auto m = parse_hom_mat2d(hom_mat2d, op);
  const double d = det(m);
  const std::array<double, 6> inv{m[4] / d, -m[1] / d, (m[1] * m[5] - m[4] * m[2]) / d,
                                  -m[3] / d, m[0] / d, (m[3] * m[2] - m[0] * m[5]) / d};
  return hom_mat2d_from_values(inv);
}

void affine_trans_point_2d(const core::Tuple& hom_mat2d, const core::Tuple& px,
                           const core::Tuple& py, core::Tuple* qx, core::Tuple* qy) {
  constexpr auto op = "affine_trans_point_2d";
  if (qx == nullptr || qy == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op,
                          "outputs", -1, "outputs must not be null");
  }
  if (qx == qy) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op,
                          "outputs", -1, "parallel Tuple outputs must be distinct");
  }
  const auto m = parse_hom_mat2d(hom_mat2d, op);
  const auto plan =
      core::BroadcastPlan::make({{px.size(), "Px", true}, {py.size(), "Py", true}}, op);
  const bool real_coordinates = px.kind() == core::TupleKind::Real64 && py.kind() == core::TupleKind::Real64;
  const auto x_values = real_coordinates ? px.real64_span() : std::span<const double>{};
  const bool broadcast_x = px.size() == 1;
  const auto y_values = real_coordinates ? py.real64_span() : std::span<const double>{};
  const bool broadcast_y = py.size() == 1;
  std::vector<double> ox;
  ox.reserve(plan.size());
  std::vector<double> oy;
  oy.reserve(plan.size());
  // 同质数值和通用输入只分派一次；定长结果去除逐点追加的容量分支。
  ox.resize(plan.size());
  oy.resize(plan.size());
  const auto transform = [&]<bool Real>() {
    std::size_t first=0;
#if MVKIT_GEOMETRY_HAS_AVX2
    if constexpr(Real) {
      const auto& cpu=mv::runtime::cpu_features();
      if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
          cpu.selection(),cpu.hardware()),{false,true,false,false}))
        first=affine_points_real_avx2(x_values,y_values,m.data(),ox,oy);
    }
#endif
    for (std::size_t i = first; i < plan.size(); ++i) {
      double x = 0;
      double y = 0;
      try {
        if constexpr (Real) {
          x = x_values[broadcast_x ? 0 : i];
          y = y_values[broadcast_y ? 0 : i];
        } else {
          x = px.as_real64(plan.source_index(0, i));
          y = py.as_real64(plan.source_index(1, i));
        }
      } catch (const core::Exception&) {
        throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, "Px",
                              static_cast<std::int64_t>(i), "point coordinates must be numeric");
      }
      require_finite(x, op, "Px");
      require_finite(y, op, "Py");
      const double xo = m[0] * x + m[1] * y + m[2];
      const double yo = m[3] * x + m[4] * y + m[5];
      require(std::isfinite(xo) && std::isfinite(yo), op, "HomMat2D",
              "transformed point is not finite");
      ox[i] = xo;
      oy[i] = yo;
    }
  };
  if (real_coordinates) transform.template operator()<true>();
  else transform.template operator()<false>();
  *qx = ox.empty() ? core::Tuple{} : core::Tuple{std::move(ox)};
  *qy = oy.empty() ? core::Tuple{} : core::Tuple{std::move(oy)};
}

core::Tuple vector_angle_to_rigid(double row1, double column1, double angle1, double row2,
                                  double column2, double angle2) {
  constexpr auto op = "vector_angle_to_rigid";
  require_finite(row1, op, "Row1");
  require_finite(column1, op, "Column1");
  require_finite(angle1, op, "Angle1");
  require_finite(row2, op, "Row2");
  require_finite(column2, op, "Column2");
  require_finite(angle2, op, "Angle2");
  const double phi = angle2 - angle1;
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  const double x1 = column1;
  const double y1 = row1;
  const double x2 = column2;
  const double y2 = row2;
  return hom_mat2d_from_values(
      {c, -s, x2 - c * x1 + s * y1, s, c, y2 - s * x1 - c * y1});
}

}  // namespace mv::geometry
