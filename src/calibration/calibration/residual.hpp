#pragma once
#include <calibration/camera_parameters.hpp>
#include <calibration/solver/problem.hpp>
#include <core/geometry_types.hpp>
#include <string>
#include <vector>
namespace mv::calibration {
struct IntrinsicBinding {
  std::string name;
  solver::ParameterBlockId id{};
  double scale{1.0};
};
class ReprojectionResidual final : public solver::ResidualFunction {
public:
  ReprojectionResidual(CameraParameters base,
                       std::vector<IntrinsicBinding> intrinsics,
                       solver::ParameterBlockId pose_id, Point3d object,
                       double row, double col);
  std::size_t residual_size() const noexcept override { return 2; }
  std::span<const solver::ParameterBlockId>
  parameter_blocks() const noexcept override {
    return ids_;
  }
  core::Status evaluate(std::span<const solver::ParameterBlockView> p,
                        std::span<double> r,
                        std::span<solver::JacobianView> j) const override;

private:
  friend class ReprojectionBatchResidual;
  core::Status evaluate_prepared(const CameraParameters &, const Pose3d &,
                                 const Point3d &,
                                 std::span<const solver::ParameterBlockView>,
                                 std::span<double>,
                                 std::span<solver::JacobianView>) const;
  CameraParameters
  camera_from(std::span<const solver::ParameterBlockView> p) const;
  Pose3d pose_from(const solver::ParameterBlockView &p) const;
  std::array<double, 2> residual(const CameraParameters &camera,
                                 const Pose3d &pose) const;
  CameraParameters base_;
  std::vector<IntrinsicBinding> intrinsics_;
  solver::ParameterBlockId pose_id_{};
  std::vector<solver::ParameterBlockId> ids_;
  Point3d object_{};
  double row_{}, col_{};
};
class ReprojectionBatchResidual final : public solver::ResidualFunction {
public:
  explicit ReprojectionBatchResidual(std::vector<ReprojectionResidual> points)
      : points_(std::move(points)) {}
  std::size_t residual_size() const noexcept override {
    return 2 * points_.size();
  }
  std::span<const solver::ParameterBlockId>
  parameter_blocks() const noexcept override {
    return points_.empty() ? std::span<const solver::ParameterBlockId>{}
                           : points_.front().parameter_blocks();
  }
  core::Status evaluate(std::span<const solver::ParameterBlockView>,
                        std::span<double>,
                        std::span<solver::JacobianView>) const override;

private:
  std::vector<ReprojectionResidual> points_;
};
} // namespace mv::calibration
