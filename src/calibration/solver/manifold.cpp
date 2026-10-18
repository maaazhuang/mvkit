#include <calibration/solver/manifold.hpp>

#include <algorithm>
#include <cmath>

namespace mv::calibration::solver {
namespace {
constexpr double kTiny = 1e-15;

void canonicalize_quaternion(double* q) noexcept {
  const double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (!(n > kTiny) || !std::isfinite(n)) return;
  for (int i = 0; i < 4; ++i) q[i] /= n;
  if (q[0] < 0.0) for (int i = 0; i < 4; ++i) q[i] = -q[i];
}

void left_multiply_delta_quaternion(double* q, const double* w) noexcept {
  const double theta = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
  double dq[4];
  if (theta < 1e-12) {
    dq[0] = 1.0;
    dq[1] = 0.5 * w[0];
    dq[2] = 0.5 * w[1];
    dq[3] = 0.5 * w[2];
  } else {
    const double half = 0.5 * theta;
    const double s = std::sin(half) / theta;
    dq[0] = std::cos(half);
    dq[1] = s * w[0];
    dq[2] = s * w[1];
    dq[3] = s * w[2];
  }
  const double old[4] = {q[0], q[1], q[2], q[3]};
  q[0] = dq[0]*old[0] - dq[1]*old[1] - dq[2]*old[2] - dq[3]*old[3];
  q[1] = dq[0]*old[1] + dq[1]*old[0] + dq[2]*old[3] - dq[3]*old[2];
  q[2] = dq[0]*old[2] - dq[1]*old[3] + dq[2]*old[0] + dq[3]*old[1];
  q[3] = dq[0]*old[3] + dq[1]*old[2] - dq[2]*old[1] + dq[3]*old[0];
  canonicalize_quaternion(q);
}

void apply_bounds(ParameterBlock& block) noexcept {
  const auto n = block.values.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (i < block.lower_bounds.size() && block.lower_bounds[i]) {
      block.values[i] = std::max(block.values[i], *block.lower_bounds[i]);
    }
    if (i < block.upper_bounds.size() && block.upper_bounds[i]) {
      block.values[i] = std::min(block.values[i], *block.upper_bounds[i]);
    }
  }
}
}  // namespace

std::size_t expected_tangent_size(const ParameterBlock& block) noexcept {
  switch (block.manifold) {
    case ManifoldKind::Euclidean: return block.values.size();
    case ManifoldKind::UnitQuaternion: return 3;
    case ManifoldKind::PoseSE3: return 6;
  }
  return 0;
}

core::Status validate_parameter_block(const ParameterBlock& block) noexcept {
  if (block.values.empty()) return core::Status::InvalidArgument;
  if (block.manifold == ManifoldKind::UnitQuaternion && block.values.size() != 4) return core::Status::InvalidArgument;
  if (block.manifold == ManifoldKind::PoseSE3 && block.values.size() != 7) return core::Status::InvalidArgument;
  if (block.tangent_size != expected_tangent_size(block)) return core::Status::InvalidArgument;
  if (!block.lower_bounds.empty() && block.lower_bounds.size() != block.values.size()) return core::Status::InvalidArgument;
  if (!block.upper_bounds.empty() && block.upper_bounds.size() != block.values.size()) return core::Status::InvalidArgument;
  for (double v : block.values) if (!std::isfinite(v)) return core::Status::InvalidArgument;
  return core::Status::Ok;
}

core::Status apply_local_delta(ParameterBlock& block, std::span<const double> delta) noexcept {
  if (validate_parameter_block(block) != core::Status::Ok || delta.size() != block.tangent_size) {
    return core::Status::InvalidArgument;
  }
  switch (block.manifold) {
    case ManifoldKind::Euclidean:
      for (std::size_t i = 0; i < delta.size(); ++i) block.values[i] += delta[i];
      apply_bounds(block);
      break;
    case ManifoldKind::UnitQuaternion:
      left_multiply_delta_quaternion(block.values.data(), delta.data());
      apply_bounds(block);
      canonicalize_quaternion(block.values.data());
      break;
    case ManifoldKind::PoseSE3:
      block.values[0] += delta[0];
      block.values[1] += delta[1];
      block.values[2] += delta[2];
      left_multiply_delta_quaternion(block.values.data() + 3, delta.data() + 3);
      apply_bounds(block);
      canonicalize_quaternion(block.values.data() + 3);
      break;
  }
  for (double v : block.values) if (!std::isfinite(v)) return core::Status::InvalidState;
  return core::Status::Ok;
}

}  // namespace mv::calibration::solver
