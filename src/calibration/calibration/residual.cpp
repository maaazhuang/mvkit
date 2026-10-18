#include <algorithm>
#include <array>
#include <calibration/calibration/residual.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/solver/manifold.hpp>
#include <cmath>
namespace mv::calibration {
ReprojectionResidual::ReprojectionResidual(
    CameraParameters base, std::vector<IntrinsicBinding> intrinsics,
    solver::ParameterBlockId pose_id, Point3d object, double row, double col)
    : base_(std::move(base)), intrinsics_(std::move(intrinsics)),
      pose_id_(pose_id), object_(object), row_(row), col_(col) {
  for (const auto &b : intrinsics_)
    ids_.push_back(b.id);
  ids_.push_back(pose_id_);
}
CameraParameters ReprojectionResidual::camera_from(
    std::span<const solver::ParameterBlockView> p) const {
  auto c = base_;
  for (std::size_t i = 0; i < intrinsics_.size(); ++i)
    set_camera_parameter_value(c, intrinsics_[i].name,
                               p[i].values[0] * intrinsics_[i].scale);
  return c;
}
Pose3d
ReprojectionResidual::pose_from(const solver::ParameterBlockView &p) const {
  return {{p.values[0], p.values[1], p.values[2]},
          normalize_quaternion(
              {p.values[3], p.values[4], p.values[5], p.values[6]})};
}
std::array<double, 2> ReprojectionResidual::residual(const CameraParameters &c,
                                                     const Pose3d &p) const {
  const auto cp = transform_point(p, object_);
  const auto ip = project_camera_point(c, cp);
  return {ip.row - row_, ip.col - col_};
}
core::Status
ReprojectionResidual::evaluate(std::span<const solver::ParameterBlockView> p,
                               std::span<double> r,
                               std::span<solver::JacobianView> j) const {
  try {
    if (p.size() != intrinsics_.size() + 1 || r.size() != 2)
      return core::Status::InvalidArgument;
    const auto camera = camera_from(p);
    const auto pose = pose_from(p.back());
    return evaluate_prepared(camera, pose, transform_point(pose, object_), p, r,
                             j);
  } catch (const core::Exception &ex) {
    return ex.status();
  }
}
core::Status ReprojectionResidual::evaluate_prepared(
    const CameraParameters &camera, const Pose3d &pose, const Point3d &point,
    std::span<const solver::ParameterBlockView> p, std::span<double> r,
    std::span<solver::JacobianView> j) const {
  try {
    const auto ip = project_camera_point(camera, point);
    const std::array<double, 2> base_r{ip.row - row_, ip.col - col_};
    r[0] = base_r[0];
    r[1] = base_r[1];
    if (j.empty())
      return core::Status::Ok;

    // division 模型使用解析投影 Jacobian；其他模型及奇异畸变保留有限差分。
    if (const auto *c = std::get_if<AreaScanDivisionParameters>(&camera)) {
      const double u = c->focus * point.x / point.z,
                   v = c->focus * point.y / point.z, r2 = u * u + v * v;
      const double radicand = 1 - 4 * c->kappa * r2;
      if (radicand > 1e-12) {
        const double root = std::sqrt(radicand), scale = 2 / (1 + root),
                     a = c->kappa * scale * scale / root;
        const double ru = 2 * u * v * a / c->sy,
                     rv = (scale + 2 * v * v * a) / c->sy;
        const double cu = (scale + 2 * u * u * a) / c->sx,
                     cv = 2 * u * v * a / c->sx;
        for (std::size_t i = 0; i < intrinsics_.size(); ++i) {
          const auto &b = intrinsics_[i];
          double dr = 0, dc = 0;
          if (b.name == "focus") {
            dr = (ru * point.x + rv * point.y) / point.z;
            dc = (cu * point.x + cv * point.y) / point.z;
          } else if (b.name == "kappa") {
            const double dk = r2 * scale * scale / root;
            dr = v * dk / c->sy;
            dc = u * dk / c->sx;
          } else if (b.name == "sx")
            dc = -u * scale / (c->sx * c->sx);
          else if (b.name == "sy")
            dr = -v * scale / (c->sy * c->sy);
          else if (b.name == "cx")
            dc = 1;
          else if (b.name == "cy")
            dr = 1;
          j[i](0, 0) = dr * b.scale;
          j[i](1, 0) = dc * b.scale;
        }
        const double rotated[3]{point.x - pose.translation.x,
                                point.y - pose.translation.y,
                                point.z - pose.translation.z};
        const double gradients[2][3]{
            {ru * c->focus / point.z, rv * c->focus / point.z,
             -(ru * u + rv * v) / point.z},
            {cu * c->focus / point.z, cv * c->focus / point.z,
             -(cu * u + cv * v) / point.z}};
        for (std::size_t r = 0; r < 2; ++r) {
          const auto *g = gradients[r];
          for (std::size_t k = 0; k < 3; ++k)
            j.back()(r, k) = g[k];
          j.back()(r, 3) = -g[1] * rotated[2] + g[2] * rotated[1];
          j.back()(r, 4) = g[0] * rotated[2] - g[2] * rotated[0];
          j.back()(r, 5) = -g[0] * rotated[1] + g[1] * rotated[0];
        }
        return core::Status::Ok;
      }
    }
    for (std::size_t i = 0; i < intrinsics_.size(); ++i) {
      auto c = camera;
      double value = p[i].values[0];
      double h = 1e-7 * std::max(1.0, std::abs(value));
      set_camera_parameter_value(c, intrinsics_[i].name,
                                 (value + h) * intrinsics_[i].scale);
      const auto rp = residual(c, pose);
      j[i](0, 0) = (rp[0] - base_r[0]) / h;
      j[i](1, 0) = (rp[1] - base_r[1]) / h;
    }
    auto temp = solver::ParameterBlock{
        pose_id_,
        std::vector<double>(p.back().values.begin(), p.back().values.end()),
        6,
        solver::ManifoldKind::PoseSE3,
        false,
        {},
        {}};
    auto pert = temp;
    for (std::size_t c = 0; c < 6; ++c) {
      double d[6] = {0, 0, 0, 0, 0, 0};
      const double h = (c < 3 ? 1e-7 : 1e-7);
      d[c] = h;
      std::copy(temp.values.begin(), temp.values.end(), pert.values.begin());
      if (solver::apply_local_delta(pert, d) != core::Status::Ok)
        return core::Status::InvalidState;
      solver::ParameterBlockView view{pose_id_, pert.values, 6,
                                      solver::ManifoldKind::PoseSE3};
      const auto rp = residual(camera, pose_from(view));
      j.back()(0, c) = (rp[0] - base_r[0]) / h;
      j.back()(1, c) = (rp[1] - base_r[1]) / h;
    }
    return core::Status::Ok;
  } catch (const core::Exception &ex) {
    return ex.status();
  }
}
core::Status ReprojectionBatchResidual::evaluate(
    std::span<const solver::ParameterBlockView> p, std::span<double> r,
    std::span<solver::JacobianView> j) const {
  try {
    if (points_.empty() || p.size() != points_.front().intrinsics_.size() + 1 ||
        r.size() != residual_size())
      return core::Status::InvalidArgument;
    const auto camera = points_.front().camera_from(p);
    const auto pose = points_.front().pose_from(p.back());
    const auto rotation = rotation_matrix(pose.rotation);
    std::vector<solver::JacobianView> local(j.size());
    for (std::size_t i = 0; i < points_.size(); ++i) {
      for (std::size_t k = 0; k < j.size(); ++k)
        local[k] = {j[k].values.subspan(2 * i * j[k].cols, 2 * j[k].cols), 2,
                    j[k].cols};
      const auto &o = points_[i].object_;
      const Point3d point{rotation[0] * o.x + rotation[1] * o.y +
                              rotation[2] * o.z + pose.translation.x,
                          rotation[3] * o.x + rotation[4] * o.y +
                              rotation[5] * o.z + pose.translation.y,
                          rotation[6] * o.x + rotation[7] * o.y +
                              rotation[8] * o.z + pose.translation.z};
      const auto status = points_[i].evaluate_prepared(
          camera, pose, point, p, r.subspan(2 * i, 2), local);
      if (status != core::Status::Ok)
        return status;
    }
    return core::Status::Ok;
  } catch (const core::Exception &ex) {
    return ex.status();
  }
}
} // namespace mv::calibration
