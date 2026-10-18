#include <calibration/targets/find_calib_object.hpp>
#include <calibration/targets/calplate.hpp>
#include <calibration/targets/caltab.hpp>
#include <calibration/calibration/homography.hpp>
#include <calibration/calibration/initialize.hpp>
#include <calibration/camera_parameters.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <core/exception.hpp>
#include <core/tuple.hpp>

namespace mv::calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;

double mean_nearest_spacing(const std::vector<double>& rows, const std::vector<double>& columns) {
  const std::size_t n = rows.size();
  if (n < 2) return 0.0;
  double sum = 0.0;
  std::size_t counted = 0;
  for (std::size_t i = 0; i < n; ++i) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < n; ++j) {
      if (i == j) continue;
      best = std::min(best, std::hypot(rows[i] - rows[j], columns[i] - columns[j]));
    }
    if (std::isfinite(best)) {
      sum += best;
      ++counted;
    }
  }
  return counted == 0 ? 0.0 : sum / static_cast<double>(counted);
}

core::XldContours checkerboard_box_contours(const std::vector<core::Point2d>& points) {
  core::XldContours contours;
  for (const auto& p : points) {
    const std::array<core::Point2d, 4> box{
        {{p.row - 2, p.col - 2}, {p.row - 2, p.col + 2}, {p.row + 2, p.col + 2}, {p.row + 2, p.col - 2}}};
    contours.append(core::XldContour::from_points(box, true));
  }
  return contours;
}

core::XldContours calplate_circle_contours(const std::vector<double>& rows,
                                           const std::vector<double>& columns) {
  core::XldContours contours;
  const double radius = std::max(2.0, 0.5 * mean_nearest_spacing(rows, columns));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    std::array<core::Point2d, 8> ring{};
    for (int k = 0; k < 8; ++k) {
      const double a = 2.0 * kPi * static_cast<double>(k) / 8.0;
      ring[static_cast<std::size_t>(k)] = {rows[i] + radius * std::sin(a),
                                           columns[i] + radius * std::cos(a)};
    }
    contours.append(core::XldContour::from_points(ring, true));
  }
  return contours;
}

void estimate_observation_pose(const CalibrationDataModel& data, std::int64_t camera_index,
                               const CalibrationTarget& target, CalibrationObservation& obs) {
  if (const auto& camera = data.camera(camera_index); camera.parameters.has_value()) {
    if (is_perspective(*camera.parameters)) {
      obs.initial_pose = estimate_planar_pose_perspective(*camera.parameters, target.points, obs.rows,
                                                          obs.columns, obs.indices);
    } else {
      obs.initial_pose = estimate_planar_pose_telecentric(*camera.parameters, target.points, obs.rows,
                                                          obs.columns, obs.indices);
    }
  }
}

void commit_observation(CalibrationDataModel& data, std::int64_t camera_index,
                        std::int64_t calib_object_index, std::int64_t pose_index,
                        CalibrationObservation obs, core::XldContours contours,
                        core::XldContours* mark_contours) {
  obs.mark_contours = contours;
  data.replace_observation({camera_index, calib_object_index, pose_index}, std::move(obs));
  data.clear_calibration_result();
  data.clear_hand_eye_result();
  data.mark_automatic_detection();
  if (mark_contours) *mark_contours = std::move(contours);
}

}  // namespace

void find_calib_object(const core::Image& image, CalibrationDataModel& data,
                       std::int64_t camera_index, std::int64_t calib_object_index,
                       std::int64_t pose_index, core::XldContours* mark_contours) {
  (void)data.camera(camera_index);
  const auto& target = data.target(calib_object_index);
  if (pose_index < 0) {
    throw core::Exception(core::Status::InvalidArgument, "find_calib_object",
                          "pose index must be non-negative");
  }

  if (target.kind == TargetKind::Calplate) {
    CalplateDescription descr;
    descr.rows = target.rows;
    descr.columns = target.columns;
    descr.points = target.points;
    descr.radii = target.mark_radii;
    descr.finders = target.finders;
    descr.light_on_dark = target.light_on_dark;
    const auto found = detect_calplate(image, descr);
    if (found.indices.size() < 4) {
      throw core::Exception(core::Status::NotFound, "find_calib_object", "too few calplate marks");
    }
    CalibrationObservation obs;
    obs.rows = found.rows;
    obs.columns = found.columns;
    obs.indices = found.indices;
    const auto& camera = data.camera(camera_index);
    const CameraParameters* cam = camera.parameters.has_value() ? &*camera.parameters : nullptr;
    refine_calplate_observation(image, descr, cam, obs);
    estimate_observation_pose(data, camera_index, target, obs);
    auto contours = calplate_circle_contours(obs.rows, obs.columns);
    commit_observation(data, camera_index, calib_object_index, pose_index, std::move(obs),
                       std::move(contours), mark_contours);
    return;
  }

  std::vector<core::Point2d> points;
  switch (target.kind) {
    case TargetKind::Checkerboard:
      points = detect_checkerboard_points(image, target.rows, target.columns);
      break;
    case TargetKind::CircleGrid:
      points = detect_circle_grid_points(image, target.rows, target.columns);
      break;
    case TargetKind::CustomPoints:
      throw core::Exception(core::Status::Unsupported, "find_calib_object",
                            "automatic detection is not supported for custom targets");
    case TargetKind::Caltab:
      points = detect_caltab(
          image, CaltabDescription{target.rows, target.columns, 0.0, target.light_on_dark,
                                   target.points, target.mark_radii, target.caltab_rim,
                                   target.caltab_frame, target.caltab_triangle,
                                   target.caltab_frame_width});
      break;
    default:
      throw core::Exception(core::Status::InvalidState, "find_calib_object",
                            "calibration object is not configured");
  }
  if (points.size() != target.points.size()) {
    throw core::Exception(core::Status::NotFound, "find_calib_object",
                          "detected grid has unexpected size");
  }
  CalibrationObservation obs;
  obs.rows.reserve(points.size());
  obs.columns.reserve(points.size());
  obs.indices.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    obs.rows.push_back(points[i].row);
    obs.columns.push_back(points[i].col);
    obs.indices.push_back(static_cast<std::int64_t>(i));
  }
  estimate_observation_pose(data, camera_index, target, obs);
  auto contours = checkerboard_box_contours(points);
  commit_observation(data, camera_index, calib_object_index, pose_index, std::move(obs),
                     std::move(contours), mark_contours);
}
}  // namespace mv::calibration
