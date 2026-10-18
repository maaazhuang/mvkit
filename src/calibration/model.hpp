#pragma once

#include <array>
#include <calibration/camera_parameters.hpp>
#include <calibration/solver/backend.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <cstddef>
#include <cstdint>
#include <map>
#include <core/geometry_types.hpp>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace mv::calibration {

enum class SetupKind {
  CalibrationObject,
  HandEyeMovingCamera,
  HandEyeStationaryCamera,
  HandEyeScaraMovingCamera,
  HandEyeScaraStationaryCamera,
};

enum class TargetKind {
  Unconfigured,
  CustomPoints,
  Checkerboard,
  CircleGrid,
  Calplate,
  Caltab,
};

#ifndef MVKIT_CALIBRATION_CALPLATE_FINDER_DEFINED
#define MVKIT_CALIBRATION_CALPLATE_FINDER_DEFINED
struct CalplateFinder {
  std::int64_t column{};
  std::int64_t row{};
};
#endif

struct CalibrationTarget {
  TargetKind kind{TargetKind::Unconfigured};
  std::vector<Point3d> points;
  std::int64_t rows{};
  std::int64_t columns{};
  double spacing_row{};
  double spacing_column{};
  std::vector<double> mark_radii;
  std::vector<CalplateFinder> finders;
  bool light_on_dark{true};
  std::optional<std::array<double, 4>> caltab_rim;
  std::optional<std::array<double, 4>> caltab_frame;
  std::optional<std::array<double, 4>> caltab_triangle;
  std::optional<double> caltab_frame_width;
};

struct CameraSlot {
  // 拟合保留初值，parameters 保存当前拟合结果。
  std::optional<CameraParameters> initial_parameters;
  std::optional<CameraParameters> parameters;
  std::set<std::string> excluded_settings;
};

struct ObservationKey {
  std::int64_t camera_index{};
  std::int64_t calib_object_index{};
  std::int64_t pose_index{};

  friend bool operator<(const ObservationKey& a, const ObservationKey& b) noexcept {
    if (a.camera_index != b.camera_index) return a.camera_index < b.camera_index;
    if (a.calib_object_index != b.calib_object_index) return a.calib_object_index < b.calib_object_index;
    return a.pose_index < b.pose_index;
  }
};

struct CalibrationObservation {
  std::vector<double> rows;
  std::vector<double> columns;
  std::vector<std::int64_t> indices;
  std::optional<Pose3d> initial_pose;
  // 公开观测分别保存报告的标准位姿与拟合用有限数值位姿；私有内核中此项为空。
  core::Tuple reported_pose;
  core::XldContours mark_contours;
};

struct HandEyeResult {
  Pose3d first_pose;
  Pose3d second_pose;
  double translation_rmse{};
  double rotation_rmse{};
  double translation_max{};
  double rotation_max{};
  solver::SolverSummary solver_summary;
};

struct CalibrationResult {
  double rmse{};
  double mean_error{};
  double max_error{};
  std::size_t num_points{};
  solver::SolverSummary solver_summary;
  std::map<ObservationKey, Pose3d> poses;
  std::map<ObservationKey, double> per_view_rmse;
  std::vector<std::string> camera_parameter_names;
  std::vector<double> camera_stddev;
  std::vector<double> camera_covariance;
  bool covariance_available{false};
  std::string quality_message;
};

class CalibrationDataModel {
 public:
  static CalibrationDataModel create(std::string_view setup_model, std::int64_t num_cameras,
                                     std::int64_t num_calib_objects);

  CalibrationDataModel(CalibrationDataModel&&) noexcept = default;
  CalibrationDataModel& operator=(CalibrationDataModel&&) noexcept = default;
  CalibrationDataModel(const CalibrationDataModel&) = delete;
  CalibrationDataModel& operator=(const CalibrationDataModel&) = delete;

  [[nodiscard]] CalibrationDataModel clone() const;

  [[nodiscard]] SetupKind setup_kind() const noexcept { return setup_kind_; }
  [[nodiscard]] std::size_t num_cameras() const noexcept { return cameras_.size(); }
  [[nodiscard]] std::size_t num_calibration_objects() const noexcept { return targets_.size(); }
  [[nodiscard]] const CameraSlot& camera(std::int64_t index) const;
  [[nodiscard]] CameraSlot& camera(std::int64_t index);
  [[nodiscard]] const CalibrationTarget& target(std::int64_t index) const;
  [[nodiscard]] CalibrationTarget& target(std::int64_t index);
  [[nodiscard]] const CalibrationObservation& observation(const ObservationKey& key) const;
  [[nodiscard]] const std::map<ObservationKey, CalibrationObservation>& observations() const noexcept {
    return observations_;
  }
  [[nodiscard]] bool has_automatic_detection() const noexcept { return has_automatic_detection_; }
  void mark_automatic_detection() noexcept { has_automatic_detection_ = true; }
  [[nodiscard]] solver::SolverOptions& options() noexcept { return options_; }
  [[nodiscard]] const solver::SolverOptions& options() const noexcept { return options_; }
  [[nodiscard]] std::string_view optimization_method() const noexcept { return optimization_method_; }
  [[nodiscard]] const std::optional<CalibrationResult>& calibration_result() const noexcept {
    return calibration_result_;
  }
  [[nodiscard]] const std::optional<HandEyeResult>& hand_eye_result() const noexcept {
    return hand_eye_result_;
  }
  [[nodiscard]] const std::map<std::int64_t, Pose3d>& tool_in_base_poses() const noexcept {
    return tool_in_base_poses_;
  }
  void set_calibration_result(CalibrationResult result) { calibration_result_ = std::move(result); }
  void clear_calibration_result() noexcept { calibration_result_.reset(); }
  void set_hand_eye_result(HandEyeResult result) { hand_eye_result_ = std::move(result); }
  void clear_hand_eye_result() noexcept { hand_eye_result_.reset(); }
  void set_tool_in_base_pose(std::int64_t pose_index, Pose3d pose);
  void set_optimization_method(std::string method) { optimization_method_ = std::move(method); }

  void replace_observation(ObservationKey key, CalibrationObservation observation);
  void remove_observation(const ObservationKey& key);

 private:
  CalibrationDataModel(SetupKind kind, std::size_t cameras, std::size_t targets);

  SetupKind setup_kind_{SetupKind::CalibrationObject};
  std::vector<CameraSlot> cameras_;
  std::vector<CalibrationTarget> targets_;
  std::map<ObservationKey, CalibrationObservation> observations_;
  solver::SolverOptions options_{};
  std::string optimization_method_{"lm"};
  std::optional<CalibrationResult> calibration_result_;
  std::map<std::int64_t, Pose3d> tool_in_base_poses_;
  std::optional<HandEyeResult> hand_eye_result_;
  bool has_automatic_detection_{false};
};

void set_camera_parameters(CalibrationDataModel& data, std::int64_t camera_index,
                           const core::Tuple& camera_param);
void set_custom_target_points(CalibrationDataModel& data, std::int64_t calib_object_index,
                              const core::Tuple& x, const core::Tuple& y, const core::Tuple& z);
void set_checkerboard_target(CalibrationDataModel& data, std::int64_t calib_object_index, std::int64_t rows,
                             std::int64_t columns, double spacing_row, double spacing_column);
void set_circle_grid_target(CalibrationDataModel& data, std::int64_t calib_object_index, std::int64_t rows,
                            std::int64_t columns, double spacing_row, double spacing_column);
void set_calplate_target(CalibrationDataModel& data, std::int64_t calib_object_index, std::string_view descr);
void set_observation_points(CalibrationDataModel& data, std::int64_t camera_index,
                            std::int64_t calib_object_index, std::int64_t pose_index, const core::Tuple& row,
                            const core::Tuple& column, const core::Tuple& index,
                            const core::Tuple& initial_pose);

// 模型优化入口与数据模型契约集中维护。
void calibrate_cameras(CalibrationDataModel& data, double* reprojection_rmse);
void calibrate_hand_eye(CalibrationDataModel& data, std::string_view method,
                        std::array<double, 4>* errors);

}  // namespace mv::calibration
