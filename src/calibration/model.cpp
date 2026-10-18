#include <calibration/geometry3d.hpp>
#include <calibration/model.hpp>
#include <calibration/targets/calplate_descr.hpp>
#include <calibration/targets/caltab.hpp>
#include <cctype>
#include <cmath>
#include <core/exception.hpp>
#include <filesystem>
#include <limits>
#include <numeric>
#include <set>
#include <string>

namespace mv::calibration {
namespace {

std::string ascii_lower(std::string text) {
  for (char& ch : text) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return text;
}

std::string extension_lower(const std::filesystem::path& path) {
  return ascii_lower(path.extension().string());
}

std::size_t checked_index(std::int64_t index, std::size_t size, std::string_view where) {
  if (index < 0 || static_cast<std::uint64_t>(index) >= static_cast<std::uint64_t>(size)) {
    throw core::Exception(core::Status::OutOfRange, where, "index is out of range");
  }
  return static_cast<std::size_t>(index);
}

std::vector<double> numeric_tuple(const core::Tuple& tuple, std::string_view where) {
  std::vector<double> values;
  values.reserve(tuple.size());
  for (std::size_t i = 0; i < tuple.size(); ++i) {
    try {
      const double v = tuple.at(i).as_real64();
      if (!std::isfinite(v)) {
        throw core::Exception(core::Status::InvalidArgument, where, "numeric values must be finite");
      }
      values.push_back(v);
    } catch (const core::Exception& ex) {
      if (ex.status() == core::Status::InvalidArgument) throw;
      throw core::Exception(core::Status::InvalidArgument, where, "tuple values must be numeric");
    }
  }
  return values;
}

CalibrationTarget grid_target(TargetKind kind, std::int64_t rows, std::int64_t columns, double spacing_row,
                              double spacing_column, std::string_view where) {
  if (rows < 2 || columns < 2 || rows > 100000 || columns > 100000 || !std::isfinite(spacing_row) ||
      !std::isfinite(spacing_column) || spacing_row <= 0.0 || spacing_column <= 0.0) {
    throw core::Exception(core::Status::InvalidArgument, where,
                          "grid dimensions and spacing must be positive");
  }
  if (rows > std::numeric_limits<std::int64_t>::max() / columns) {
    throw core::Exception(core::Status::InvalidArgument, where, "grid point count overflow");
  }
  CalibrationTarget candidate;
  candidate.kind = kind;
  candidate.rows = rows;
  candidate.columns = columns;
  candidate.spacing_row = spacing_row;
  candidate.spacing_column = spacing_column;
  candidate.points.reserve(static_cast<std::size_t>(rows * columns));
  for (std::int64_t r = 0; r < rows; ++r) {
    for (std::int64_t c = 0; c < columns; ++c) {
      candidate.points.push_back(
          {static_cast<double>(c) * spacing_column, static_cast<double>(r) * spacing_row, 0.0});
    }
  }
  return candidate;
}

}  // namespace

CalibrationDataModel::CalibrationDataModel(SetupKind kind, std::size_t cameras, std::size_t targets)
    : setup_kind_(kind),
      cameras_(cameras),
      targets_(targets),
      optimization_method_(kind == SetupKind::CalibrationObject ? "lm" : "nonlinear") {}

CalibrationDataModel CalibrationDataModel::create(std::string_view setup_model, std::int64_t num_cameras,
                                                  std::int64_t num_calib_objects) {
  SetupKind kind;
  if (setup_model == "calibration_object") {
    kind = SetupKind::CalibrationObject;
  } else if (setup_model == "hand_eye_moving_cam") {
    kind = SetupKind::HandEyeMovingCamera;
  } else if (setup_model == "hand_eye_stationary_cam") {
    kind = SetupKind::HandEyeStationaryCamera;
  } else if (setup_model == "hand_eye_scara_moving_cam") {
    kind = SetupKind::HandEyeScaraMovingCamera;
  } else if (setup_model == "hand_eye_scara_stationary_cam") {
    kind = SetupKind::HandEyeScaraStationaryCamera;
  } else {
    throw core::Exception(core::Status::Unsupported, "CalibrationDataModel::create",
                          "unsupported calibration setup model");
  }
  if (num_cameras < 0 || num_calib_objects < 0) {
    throw core::Exception(core::Status::InvalidArgument, "CalibrationDataModel::create",
                          "counts must be nonnegative");
  }
  if (kind != SetupKind::CalibrationObject &&
      !((num_cameras == 1 && num_calib_objects == 1) || (num_cameras == 0 && num_calib_objects == 0))) {
    throw core::Exception(core::Status::InvalidArgument, "CalibrationDataModel::create",
                          "hand-eye setup requires counts 1/1 or 0/0");
  }
  return CalibrationDataModel(kind, static_cast<std::size_t>(num_cameras),
                              static_cast<std::size_t>(num_calib_objects));
}

const CameraSlot& CalibrationDataModel::camera(std::int64_t index) const {
  return cameras_.at(checked_index(index, cameras_.size(), "CalibrationDataModel::camera"));
}
CameraSlot& CalibrationDataModel::camera(std::int64_t index) {
  return cameras_.at(checked_index(index, cameras_.size(), "CalibrationDataModel::camera"));
}
const CalibrationTarget& CalibrationDataModel::target(std::int64_t index) const {
  return targets_.at(checked_index(index, targets_.size(), "CalibrationDataModel::target"));
}
CalibrationTarget& CalibrationDataModel::target(std::int64_t index) {
  return targets_.at(checked_index(index, targets_.size(), "CalibrationDataModel::target"));
}

const CalibrationObservation& CalibrationDataModel::observation(const ObservationKey& key) const {
  const auto it = observations_.find(key);
  if (it == observations_.end()) {
    throw core::Exception(core::Status::NotFound, "CalibrationDataModel::observation",
                          "observation not found");
  }
  return it->second;
}

void CalibrationDataModel::replace_observation(ObservationKey key, CalibrationObservation observation) {
  observations_.insert_or_assign(key, std::move(observation));
}

void CalibrationDataModel::remove_observation(const ObservationKey& key) {
  if (observations_.erase(key) == 0) {
    throw core::Exception(core::Status::NotFound, "remove_calib_data_observ", "observation not found");
  }
  clear_calibration_result();
  clear_hand_eye_result();
}

CalibrationDataModel CalibrationDataModel::clone() const {
  CalibrationDataModel copy{setup_kind_, cameras_.size(), targets_.size()};
  copy.cameras_ = cameras_;
  copy.targets_ = targets_;
  copy.observations_ = observations_;
  copy.options_ = options_;
  copy.optimization_method_ = optimization_method_;
  copy.calibration_result_ = calibration_result_;
  copy.tool_in_base_poses_ = tool_in_base_poses_;
  copy.hand_eye_result_ = hand_eye_result_;
  copy.has_automatic_detection_ = has_automatic_detection_;
  return copy;
}

void CalibrationDataModel::set_tool_in_base_pose(std::int64_t pose_index, Pose3d pose) {
  if (pose_index < 0) {
    throw core::Exception(core::Status::InvalidArgument, "CalibrationDataModel::set_tool_in_base_pose",
                          "pose index must be non-negative");
  }
  tool_in_base_poses_.insert_or_assign(pose_index, std::move(pose));
  clear_hand_eye_result();
}

void set_camera_parameters(CalibrationDataModel& data, std::int64_t camera_index,
                           const core::Tuple& camera_param) {
  auto candidate = parse_camera_parameters(camera_param);
  auto& camera = data.camera(camera_index);
  // 三个字段均由新配置覆盖，直接构造候选，避免复制后清空旧排除项。
  CameraSlot next;
  next.initial_parameters = candidate;
  next.parameters = std::move(candidate);
  next.excluded_settings.insert("sy");
  if (data.num_cameras() == 1) {
    for (const auto* name : {"transx", "transy", "transz", "alpha", "beta", "gamma"})
      next.excluded_settings.insert(name);
  }
  data.clear_calibration_result();
  camera = std::move(next);
}

void set_custom_target_points(CalibrationDataModel& data, std::int64_t calib_object_index,
                              const core::Tuple& x, const core::Tuple& y, const core::Tuple& z) {
  (void)data.target(calib_object_index);  // 构造候选前验证索引。
  if (x.size() != y.size() || x.size() != z.size() || x.size() < 4) {
    throw core::Exception(core::Status::InvalidArgument, "set_custom_target_points",
                          "x/y/z must have equal length of at least four");
  }
  const auto xs = numeric_tuple(x, "set_custom_target_points");
  const auto ys = numeric_tuple(y, "set_custom_target_points");
  const auto zs = numeric_tuple(z, "set_custom_target_points");
  CalibrationTarget candidate;
  candidate.kind = TargetKind::CustomPoints;
  candidate.points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) candidate.points.push_back({xs[i], ys[i], zs[i]});
  data.clear_calibration_result();
  data.target(calib_object_index) = std::move(candidate);
}

void set_checkerboard_target(CalibrationDataModel& data, std::int64_t calib_object_index, std::int64_t rows,
                             std::int64_t columns, double spacing_row, double spacing_column) {
  (void)data.target(calib_object_index);
  auto candidate = grid_target(TargetKind::Checkerboard, rows, columns, spacing_row, spacing_column,
                               "set_checkerboard_target");
  data.clear_calibration_result();
  data.target(calib_object_index) = std::move(candidate);
}

void set_circle_grid_target(CalibrationDataModel& data, std::int64_t calib_object_index, std::int64_t rows,
                            std::int64_t columns, double spacing_row, double spacing_column) {
  (void)data.target(calib_object_index);
  auto candidate = grid_target(TargetKind::CircleGrid, rows, columns, spacing_row, spacing_column,
                               "set_circle_grid_target");
  data.clear_calibration_result();
  data.target(calib_object_index) = std::move(candidate);
}

void set_calplate_target(CalibrationDataModel& data, std::int64_t calib_object_index,
                         std::string_view descr) {
  (void)data.target(calib_object_index);
  const std::filesystem::path descr_path{std::string(descr)};
  auto ext = extension_lower(descr_path);
  if (!ext.empty() && ext != ".cpd" && ext != ".descr") {
    throw core::Exception(core::Status::InvalidArgument, "set_calplate_target",
                          "unsupported calibration object description extension");
  }
  auto path = resolve_calib_descr(descr);
  if (ext.empty()) {
    ext = extension_lower(path);
    if (ext != ".cpd" && ext != ".descr") {
      throw core::Exception(core::Status::InvalidArgument, "set_calplate_target",
                            "unsupported calibration object description extension");
    }
  }
  if (ext == ".descr") {
    auto parsed = parse_caltab_descr(path);
    CalibrationTarget candidate;
    candidate.kind = TargetKind::Caltab;
    candidate.points = std::move(parsed.points);
    candidate.mark_radii = std::move(parsed.radii);
    candidate.rows = parsed.rows;
    candidate.columns = parsed.columns;
    candidate.light_on_dark = parsed.light_on_dark;
    candidate.caltab_rim = parsed.rim;
    candidate.caltab_frame = parsed.frame;
    candidate.caltab_triangle = parsed.triangle;
    candidate.caltab_frame_width = parsed.frame_width;
    data.clear_calibration_result();
    data.target(calib_object_index) = std::move(candidate);
    return;
  }
  auto parsed = parse_calplate_cpd(path);
  CalibrationTarget candidate;
  candidate.kind = TargetKind::Calplate;
  candidate.points = std::move(parsed.points);
  candidate.mark_radii = std::move(parsed.radii);
  candidate.finders.reserve(parsed.finders.size());
  for (const auto& finder : parsed.finders) {
    candidate.finders.push_back(CalplateFinder{finder.column, finder.row});
  }
  candidate.rows = parsed.rows;
  candidate.columns = parsed.columns;
  candidate.light_on_dark = parsed.light_on_dark;
  data.clear_calibration_result();
  data.target(calib_object_index) = std::move(candidate);
}

void set_observation_points(CalibrationDataModel& data, std::int64_t camera_index,
                            std::int64_t calib_object_index, std::int64_t pose_index, const core::Tuple& row,
                            const core::Tuple& column, const core::Tuple& index,
                            const core::Tuple& initial_pose) {
  (void)data.camera(camera_index);
  const auto& target = data.target(calib_object_index);
  if (target.kind == TargetKind::Unconfigured || target.points.size() < 4) {
    throw core::Exception(core::Status::InvalidState, "set_observation_points",
                          "calibration object is not configured");
  }
  if (pose_index < 0) {
    throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                          "pose index must be non-negative");
  }
  if (row.size() != column.size() || row.size() < 4) {
    throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                          "row/column must have equal length of at least four");
  }
  CalibrationObservation candidate;
  candidate.rows = numeric_tuple(row, "set_observation_points");
  candidate.columns = numeric_tuple(column, "set_observation_points");

  const bool all = index.size() == 1 && [&] {
    try {
      return index.string_view(0) == "all";
    } catch (const core::Exception&) {
      return false;
    }
  }();
  if (all) {
    if (row.size() != target.points.size()) {
      throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                            "index='all' requires one observation per target point");
    }
    candidate.indices.resize(row.size());
    std::iota(candidate.indices.begin(), candidate.indices.end(), std::int64_t{0});
  } else {
    if (index.size() != row.size()) {
      throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                            "index length must match row/column length");
    }
    std::set<std::int64_t> unique;
    candidate.indices.reserve(index.size());
    for (std::size_t i = 0; i < index.size(); ++i) {
      std::int64_t value;
      try {
        value = index.at(i).as_int64();
      } catch (const core::Exception&) {
        throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                              "indices must be integer");
      }
      if (value < 0 || static_cast<std::uint64_t>(value) >= target.points.size() ||
          !unique.insert(value).second) {
        throw core::Exception(core::Status::InvalidArgument, "set_observation_points",
                              "indices must be unique and in range");
      }
      candidate.indices.push_back(value);
    }
  }

  if (!initial_pose.empty()) candidate.initial_pose = pose_from_halcon_tuple(initial_pose);
  data.replace_observation({camera_index, calib_object_index, pose_index}, std::move(candidate));
  data.clear_calibration_result();
  data.clear_hand_eye_result();
}

}  // namespace mv::calibration
