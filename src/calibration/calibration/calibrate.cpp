#include <calibration/model.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <calibration/calibration/homography.hpp>
#include <calibration/calibration/initialize.hpp>
#include <calibration/calibration/residual.hpp>
#include <calibration/calibration/quality.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/solver/backend.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

std::vector<double> pose_values(const Pose3d& pose) {
  const auto q = normalize_quaternion(pose.rotation);
  return {pose.translation.x, pose.translation.y, pose.translation.z, q.w, q.x, q.y, q.z};
}

Pose3d pose_from_values(const std::vector<double>& values) {
  return {{values[0], values[1], values[2]},
          normalize_quaternion({values[3], values[4], values[5], values[6]})};
}

solver::ParameterBlock scalar_block(solver::ParameterBlockId id,
                                    double value,
                                    bool constant,
                                    std::string_view name) {
  solver::ParameterBlock block;
  block.id = id;
  block.values = {value};
  block.tangent_size = 1;
  block.manifold = solver::ManifoldKind::Euclidean;
  block.constant = constant;
  block.lower_bounds.resize(1);
  block.upper_bounds.resize(1);
  if (name == "focus" || name == "magnification" || name == "sx" || name == "sy") {
    block.lower_bounds[0] = 1e-12;
  }
  return block;
}

double intrinsic_scale(std::string_view name) {
  if (name == "k1") return 1e4;
  if (name == "k2") return 1e8;
  if (name == "k3") return 1e12;
  return 1.0;
}

Point3d target_plane_normal_in_camera(const Pose3d& pose) {
  return rotate_point(pose.rotation, {0.0, 0.0, 1.0});
}

void validate_planar_view_diversity(const std::vector<Pose3d>& poses) {
  if (poses.size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_cameras",
                          "at least three calibration views are required");
  }
  double largest_cross_norm = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto ni = target_plane_normal_in_camera(poses[i]);
    for (std::size_t j = i + 1; j < poses.size(); ++j) {
      const auto nj = target_plane_normal_in_camera(poses[j]);
      const double cx = ni.y * nj.z - ni.z * nj.y;
      const double cy = ni.z * nj.x - ni.x * nj.z;
      const double cz = ni.x * nj.y - ni.y * nj.x;
      largest_cross_norm = std::max(largest_cross_norm, std::sqrt(cx * cx + cy * cy + cz * cz));
    }
  }
  if (largest_cross_norm < 1e-3) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_cameras",
                          "calibration views do not provide sufficient orientation diversity");
  }
}

}  // namespace

void calibrate_cameras(CalibrationDataModel& data, double* reprojection_rmse) {
  if (reprojection_rmse == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, "calibrate_cameras", "rmse output is null");
  }
  if (data.setup_kind() != SetupKind::CalibrationObject) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_cameras",
                          "data is not a camera-calibration setup");
  }

  if (data.num_cameras() != 1) {
    throw core::Exception(core::Status::Unsupported, "calibrate_cameras", "joint multi-camera solving is not implemented");
  }
  for (std::size_t i = 0; i < data.num_calibration_objects(); ++i) {
    for (const auto& point : data.target(static_cast<std::int64_t>(i)).points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        throw core::Exception(core::Status::InvalidArgument, "calibrate_cameras", "target coordinates must be finite for fitting");
    }
  }
  auto& camera = data.camera(0);
  if (!camera.parameters) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_cameras",
                          "camera parameters are not configured");
  }
  if (data.observations().size() < 3) {
    throw core::Exception(core::Status::InvalidState,
                          "calibrate_cameras",
                          "at least three observations are required");
  }

  const CameraParameters configured_start = *camera.parameters;
  CameraParameters start = configured_start;
  const bool needs_pose_initialization = std::any_of(
      data.observations().begin(), data.observations().end(),
      [](const auto& item) { return !item.second.initial_pose.has_value(); });
  if (needs_pose_initialization) {
    if (is_perspective(start)) {
      start = initialize_perspective_intrinsics_planar(start, data, camera.excluded_settings, data.options().linear_backend);
    } else {
      start = initialize_telecentric_intrinsics_planar(start, data, camera.excluded_settings, data.options().linear_backend);
    }
  }

  solver::SolverProblem problem;
  std::vector<IntrinsicBinding> bindings;
  solver::ParameterBlockId next_id = 1;
  for (const auto& name : optimizable_parameter_names(start)) {
    const auto id = next_id++;
    const double scale = intrinsic_scale(name);
    bindings.push_back({name, id, scale});
    problem.parameter_blocks.push_back(
        scalar_block(id, camera_parameter_value(start, name) / scale,
                     camera.excluded_settings.count(name) != 0, name));
  }

  std::map<ObservationKey, solver::ParameterBlockId> pose_ids;
  std::vector<Pose3d> initial_poses;
  initial_poses.reserve(data.observations().size());
  solver::ParameterBlockId pose_id = 1000;
  for (const auto& [key, observation] : data.observations()) {
    const auto& target = data.target(key.calib_object_index);
    if (observation.rows.size() < 4 || observation.columns.size() != observation.rows.size() ||
        observation.indices.size() != observation.rows.size())
      throw core::Exception(core::Status::InvalidArgument, "calibrate_cameras", "each view requires at least four corresponding points");
    for (std::size_t i = 0; i < observation.rows.size(); ++i)
      if (!std::isfinite(observation.rows[i]) || !std::isfinite(observation.columns[i]))
        throw core::Exception(core::Status::InvalidArgument, "calibrate_cameras", "observation coordinates must be finite for fitting");
    for(std::size_t i=0;i<observation.reported_pose.size();++i)
      if(!std::isfinite(observation.reported_pose.as_real64(i)))
        throw core::Exception(core::Status::InvalidArgument,"calibrate_cameras","observation pose must be finite for fitting");
    if (target.kind == TargetKind::Unconfigured) {
      throw core::Exception(core::Status::InvalidState,
                            "calibrate_cameras",
                            "observation target is not configured");
    }

    Pose3d pose;
    if (observation.initial_pose) {
      pose = *observation.initial_pose;
    } else if (is_perspective(start)) {
      pose = estimate_planar_pose_perspective(
          start, target.points, observation.rows, observation.columns, observation.indices, data.options().linear_backend);
    } else {
      pose = estimate_planar_pose_telecentric(
          start, target.points, observation.rows, observation.columns, observation.indices, data.options().linear_backend);
    }

    initial_poses.push_back(pose);

    solver::ParameterBlock block;
    block.id = pose_id;
    block.values = pose_values(pose);
    block.tangent_size = 6;
    block.manifold = solver::ManifoldKind::PoseSE3;
    block.lower_bounds.resize(7);
    block.upper_bounds.resize(7);
    problem.parameter_blocks.push_back(std::move(block));
    pose_ids.emplace(key, pose_id++);
  }

  validate_planar_view_diversity(initial_poses);

  solver::ResidualBlockId residual_id = 1;
  for (const auto& [key, observation] : data.observations()) {
    const auto& target = data.target(key.calib_object_index);
    std::vector<ReprojectionResidual> residuals;residuals.reserve(observation.rows.size());
    for (std::size_t i = 0; i < observation.rows.size(); ++i) {
      const auto object_index = static_cast<std::size_t>(observation.indices[i]);
      residuals.emplace_back(start,bindings,pose_ids.at(key),target.points.at(object_index),observation.rows[i],observation.columns[i]);
    }
    problem.residual_blocks.push_back({residual_id++,std::make_unique<ReprojectionBatchResidual>(std::move(residuals))});
  }

  auto backend = solver::make_optimizer_backend(solver::OptimizerBackendKind::BuiltinLm);
  solver::SolverSummary summary;
  const auto status = backend->solve(problem, data.options(), &summary);
  if (status != core::Status::Ok) {
    throw core::Exception(status, "calibrate_cameras", summary.message);
  }

  CameraParameters final_parameters = start;
  std::map<solver::ParameterBlockId, const solver::ParameterBlock*> blocks_by_id;
  for (const auto& block : problem.parameter_blocks) {
    blocks_by_id.emplace(block.id, &block);
  }
  for (const auto& binding : bindings) {
    set_camera_parameter_value(final_parameters,
                               binding.name,
                               blocks_by_id.at(binding.id)->values[0] * binding.scale);
  }

  CalibrationResult result;
  result.solver_summary = summary;
  double sum_squared = 0.0;
  double sum_norm = 0.0;
  double max_norm = 0.0;
  std::size_t point_count = 0;

  for (const auto& [key, observation] : data.observations()) {
    const auto pose = pose_from_values(blocks_by_id.at(pose_ids.at(key))->values);
    result.poses.emplace(key, pose);
    double view_squared = 0.0;
    const auto& target = data.target(key.calib_object_index);

    for (std::size_t i = 0; i < observation.rows.size(); ++i) {
      const auto object_index = static_cast<std::size_t>(observation.indices[i]);
      const auto image_point = project_camera_point(
          final_parameters, transform_point(pose, target.points.at(object_index)));
      const double dr = image_point.row - observation.rows[i];
      const double dc = image_point.col - observation.columns[i];
      const double squared = dr * dr + dc * dc;
      const double norm = std::sqrt(squared);
      sum_squared += squared;
      view_squared += squared;
      sum_norm += norm;
      max_norm = std::max(max_norm, norm);
      ++point_count;
    }

    result.per_view_rmse.emplace(
        key, std::sqrt(view_squared / static_cast<double>(observation.rows.size())));
  }

  if (point_count == 0) {
    throw core::Exception(core::Status::InvalidState, "calibrate_cameras", "no calibration points");
  }

  result.num_points = point_count;
  result.rmse = std::sqrt(sum_squared / static_cast<double>(point_count));
  result.mean_error = sum_norm / static_cast<double>(point_count);
  result.max_error = max_norm;
  compute_calibration_uncertainty(problem, bindings, &result, data.options().linear_backend);

  camera.parameters = final_parameters;
  data.set_calibration_result(result);
  *reprojection_rmse = result.rmse;
}

}  // namespace mv::calibration
