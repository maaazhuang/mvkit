#pragma once

#include <string_view>

#include <calibration/model.hpp>
#include <core/tuple.hpp>

namespace mv::calibration {

void write_cam_par(std::string_view path, const core::Tuple& camera_param);
[[nodiscard]] core::Tuple read_cam_par(std::string_view path);
void write_pose(std::string_view path, const core::Tuple& pose);
[[nodiscard]] core::Tuple read_pose(std::string_view path);
void write_calib_data(std::string_view path, const CalibrationDataModel& data);
[[nodiscard]] CalibrationDataModel read_calib_data(std::string_view path);

}  // namespace mv::calibration
