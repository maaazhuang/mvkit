#include <algorithm>
#include <calibration/geometry3d.hpp>
#include <cmath>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEps = 1e-15;

Quaterniond multiply(const Quaterniond& a, const Quaterniond& b) {
  return normalize_quaternion({
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  });
}

Quaterniond conjugate(const Quaterniond& q) { return {q.w, -q.x, -q.y, -q.z}; }
}  // namespace

Quaterniond normalize_quaternion(Quaterniond q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(n) || n <= kEps) {
    throw core::Exception(core::Status::InvalidArgument, "normalize_quaternion",
                          "quaternion norm must be positive");
  }
  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
  if (q.w < 0.0) {
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
  }
  return q;
}

Quaterniond quaternion_from_gba(double rot_x, double rot_y, double rot_z) {
  // HALCON gba 旋转约定：R=Rx(rot_x)*Ry(rot_y)*Rz(rot_z)。
  const double hx = 0.5 * rot_x;
  const double hy = 0.5 * rot_y;
  const double hz = 0.5 * rot_z;
  const Quaterniond qx{std::cos(hx), std::sin(hx), 0.0, 0.0};
  const Quaterniond qy{std::cos(hy), 0.0, std::sin(hy), 0.0};
  const Quaterniond qz{std::cos(hz), 0.0, 0.0, std::sin(hz)};
  return multiply(multiply(qx, qy), qz);
}

std::array<double, 9> rotation_matrix(const Quaterniond& input) {
  const auto q = normalize_quaternion(input);
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  return {
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), 2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz),
      2.0 * (yz - wx),       2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
  };
}

Quaterniond quaternion_from_rotation_matrix(const std::array<double, 9>& r) {
  const double trace = r[0] + r[4] + r[8];
  Quaterniond q;
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    q.w = 0.25 * s;
    q.x = (r[7] - r[5]) / s;
    q.y = (r[2] - r[6]) / s;
    q.z = (r[3] - r[1]) / s;
  } else if (r[0] > r[4] && r[0] > r[8]) {
    const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r[0] - r[4] - r[8]));
    if (s <= kEps)
      throw core::Exception(core::Status::InvalidArgument, "quaternion_from_rotation_matrix",
                            "rotation matrix is degenerate");
    q.w = (r[7] - r[5]) / s;
    q.x = 0.25 * s;
    q.y = (r[1] + r[3]) / s;
    q.z = (r[2] + r[6]) / s;
  } else if (r[4] > r[8]) {
    const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r[4] - r[0] - r[8]));
    if (s <= kEps)
      throw core::Exception(core::Status::InvalidArgument, "quaternion_from_rotation_matrix",
                            "rotation matrix is degenerate");
    q.w = (r[2] - r[6]) / s;
    q.x = (r[1] + r[3]) / s;
    q.y = 0.25 * s;
    q.z = (r[5] + r[7]) / s;
  } else {
    const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r[8] - r[0] - r[4]));
    if (s <= kEps)
      throw core::Exception(core::Status::InvalidArgument, "quaternion_from_rotation_matrix",
                            "rotation matrix is degenerate");
    q.w = (r[3] - r[1]) / s;
    q.x = (r[2] + r[6]) / s;
    q.y = (r[5] + r[7]) / s;
    q.z = 0.25 * s;
  }
  return normalize_quaternion(q);
}

Point3d rotate_point(const Quaterniond& q, const Point3d& p) {
  const auto r = rotation_matrix(q);
  return {
      r[0] * p.x + r[1] * p.y + r[2] * p.z,
      r[3] * p.x + r[4] * p.y + r[5] * p.z,
      r[6] * p.x + r[7] * p.y + r[8] * p.z,
  };
}

Point3d transform_point(const Pose3d& pose, const Point3d& p) {
  const auto r = rotate_point(pose.rotation, p);
  return {r.x + pose.translation.x, r.y + pose.translation.y, r.z + pose.translation.z};
}

Pose3d compose_pose(const Pose3d& left, const Pose3d& right) {
  const auto rt = rotate_point(left.rotation, right.translation);
  return {
      {rt.x + left.translation.x, rt.y + left.translation.y, rt.z + left.translation.z},
      multiply(left.rotation, right.rotation),
  };
}

Pose3d inverse_pose(const Pose3d& pose) {
  const auto q = normalize_quaternion(pose.rotation);
  const auto qi = conjugate(q);
  const auto t = rotate_point(qi, {-pose.translation.x, -pose.translation.y, -pose.translation.z});
  return {t, qi};
}

core::Tuple pose_to_halcon_tuple(const Pose3d& pose) {
  const auto r = rotation_matrix(pose.rotation);
  const double sb = std::clamp(r[2], -1.0, 1.0);
  const double beta = std::asin(sb);
  const double cb = std::cos(beta);
  double alpha = 0.0;
  double gamma = 0.0;
  if (std::abs(cb) > 1e-12) {
    alpha = std::atan2(-r[5], r[8]);
    gamma = std::atan2(-r[1], r[0]);
  } else {
    // 万向节锁采用确定表示，将 gamma 设为零。
    alpha = std::atan2(r[3], r[4]);
  }
  return core::Tuple({
      core::Value{pose.translation.x},
      core::Value{pose.translation.y},
      core::Value{pose.translation.z},
      core::Value{alpha * kRadToDeg},
      core::Value{beta * kRadToDeg},
      core::Value{gamma * kRadToDeg},
      core::Value{std::int64_t{0}},
  });
}

Pose3d pose_from_halcon_tuple(const core::Tuple& tuple) {
  if (tuple.size() != 7) {
    throw core::Exception(core::Status::InvalidArgument, "pose_from_halcon_tuple",
                          "pose tuple must contain 7 values");
  }
  std::int64_t code = 0;
  try {
    code = tuple.as_int64(6);
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, "pose_from_halcon_tuple",
                          "pose code must be integer");
  }
  if (code < 0 || code > 13 || (code & 6) == 6) {
    throw core::Exception(core::Status::InvalidArgument, "pose_from_halcon_tuple",
                          "invalid HALCON pose representation code");
  }
  try {
    Point3d t{tuple.as_real64(0), tuple.as_real64(1), tuple.as_real64(2)};
    const double scale = (code & 6) == 4 ? 1.0 : kDegToRad;
    const double sign = (code & 1) ? -1.0 : 1.0;
    const double rx = tuple.as_real64(3) * scale * sign;
    const double ry = tuple.as_real64(4) * scale * sign;
    const double rz = tuple.as_real64(5) * scale * sign;
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) || !std::isfinite(rx) ||
        !std::isfinite(ry) || !std::isfinite(rz)) {
      throw core::Exception(core::Status::InvalidArgument, "pose_from_halcon_tuple",
                            "pose values must be finite");
    }
    Quaterniond rotation;
    if ((code & 6) == 4) {
      rotation = normalize_quaternion({1.0, rx * sign, ry * sign, rz * sign});
      if (code & 1) {
        // 旧 coordinate_system 将三个标准 Euler 角取负，输入为 Rodriguez 向量时也如此。
        const auto standard = pose_to_halcon_tuple({{}, rotation});
        rotation = quaternion_from_gba(-standard.as_real64(3) * kDegToRad, -standard.as_real64(4) * kDegToRad,
                                       -standard.as_real64(5) * kDegToRad);
      }
    } else if ((code & 6) == 2) {
      const Quaterniond qx{std::cos(rx / 2), std::sin(rx / 2), 0, 0};
      const Quaterniond qy{std::cos(ry / 2), 0, std::sin(ry / 2), 0};
      const Quaterniond qz{std::cos(rz / 2), 0, 0, std::sin(rz / 2)};
      rotation = multiply(multiply(qz, qy), qx);
    } else {
      rotation = quaternion_from_gba(rx, ry, rz);
    }
    // HALCON 旧 R(p-T) 约定对取负平移再旋转。
    if (code & 8) t = rotate_point(rotation, {-t.x, -t.y, -t.z});
    return {t, rotation};
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::InvalidArgument) throw;
    throw core::Exception(core::Status::InvalidArgument, "pose_from_halcon_tuple",
                          "pose values must be numeric");
  }
}

}  // namespace mv::calibration
