#include <calibration/hand_eye/linear.hpp>

#include <array>
#include <cmath>
#include <vector>

#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

using M4 = std::array<double, 16>;

M4 left_q(const Quaterniond& q) {
  return {q.w, -q.x, -q.y, -q.z,
          q.x, q.w, -q.z, q.y,
          q.y, q.z, q.w, -q.x,
          q.z, -q.y, q.x, q.w};
}

M4 right_q(const Quaterniond& q) {
  return {q.w, -q.x, -q.y, -q.z,
          q.x, q.w, q.z, -q.y,
          q.y, -q.z, q.w, q.x,
          q.z, q.y, -q.x, q.w};
}

std::array<double, 3> axis_vec(const Quaterniond& q0) {
  const auto q = normalize_quaternion(q0);
  const double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (s < 1e-10) return {};
  const double angle = 2.0 * std::atan2(s, q.w);
  return {q.x / s * angle, q.y / s * angle, q.z / s * angle};
}

Quaterniond smallest_eigenvector4(const M4& matrix,
                                  const numeric::LinearAlgebraBackend& backend) {
  std::vector<double> values;
  std::vector<double> vectors;
  if (!backend.symmetric_eigen(std::vector<double>(matrix.begin(), matrix.end()),
                               4, &values, &vectors) ||
      values.size() != 4 || vectors.size() != 16) {
    throw core::Exception(core::Status::InvalidState,
                          "solve_ax_xb",
                          "hand-eye rotation eigen decomposition failed");
  }
  return normalize_quaternion({vectors[0], vectors[4], vectors[8], vectors[12]});
}

}  // namespace

Pose3d solve_ax_xb(const std::vector<Pose3d>& a,
                   const std::vector<Pose3d>& b,
                   numeric::LinearBackendKind backend_kind) {
  if (a.size() != b.size() || a.size() < 4) {
    throw core::Exception(core::Status::InvalidArgument,
                          "solve_ax_xb",
                          "at least four relative motions required");
  }

  bool excited = false;
  std::array<double, 3> first{};
  bool have_first = false;
  for (const auto& motion : a) {
    const auto vector = axis_vec(motion.rotation);
    const double norm = std::hypot(vector[0], std::hypot(vector[1], vector[2]));
    if (norm < 1e-3) continue;
    if (!have_first) {
      first = vector;
      have_first = true;
      continue;
    }
    const double cx = first[1] * vector[2] - first[2] * vector[1];
    const double cy = first[2] * vector[0] - first[0] * vector[2];
    const double cz = first[0] * vector[1] - first[1] * vector[0];
    if (std::sqrt(cx * cx + cy * cy + cz * cz) > 1e-3) {
      excited = true;
      break;
    }
  }
  if (!excited) {
    throw core::Exception(core::Status::InvalidState,
                          "solve_ax_xb",
                          "hand-eye rotations do not excite at least two axes");
  }

  M4 normal{};
  for (std::size_t i = 0; i < a.size(); ++i) {
    const auto left = left_q(a[i].rotation);
    const auto right = right_q(b[i].rotation);
    M4 difference{};
    for (std::size_t k = 0; k < 16; ++k) difference[k] = left[k] - right[k];
    for (std::size_t r = 0; r < 4; ++r) {
      for (std::size_t c = 0; c < 4; ++c) {
        for (std::size_t k = 0; k < 4; ++k) {
          normal[r * 4 + c] += difference[k * 4 + r] * difference[k * 4 + c];
        }
      }
    }
  }

  auto backend = numeric::make_backend(backend_kind);
  const auto qx = smallest_eigenvector4(normal, *backend);
  const auto rx = rotation_matrix(qx);
  std::vector<double> translation_normal(9, 0.0);
  std::vector<double> translation_rhs(3, 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    const auto ra = rotation_matrix(a[i].rotation);
    const double m[9] = {
        ra[0] - 1.0, ra[1], ra[2],
        ra[3], ra[4] - 1.0, ra[5],
        ra[6], ra[7], ra[8] - 1.0};
    const auto& tb = b[i].translation;
    const std::array<double, 3> value{
        rx[0] * tb.x + rx[1] * tb.y + rx[2] * tb.z - a[i].translation.x,
        rx[3] * tb.x + rx[4] * tb.y + rx[5] * tb.z - a[i].translation.y,
        rx[6] * tb.x + rx[7] * tb.y + rx[8] * tb.z - a[i].translation.z};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        translation_rhs[c] += m[r * 3 + c] * value[r];
        for (std::size_t d = 0; d < 3; ++d) {
          translation_normal[c * 3 + d] += m[r * 3 + c] * m[r * 3 + d];
        }
      }
    }
  }

  std::vector<double> translation;
  if (!backend->solve_symmetric(translation_normal, translation_rhs, 3, &translation) ||
      translation.size() != 3) {
    throw core::Exception(core::Status::InvalidState,
                          "solve_ax_xb",
                          "hand-eye translation system is rank deficient");
  }
  return {{translation[0], translation[1], translation[2]}, qx};
}

}  // namespace mv::calibration
