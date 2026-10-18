#pragma once
#include <cstdint>
#include <vector>
#include <calibration/model.hpp>
#include <core/image.hpp>
#include <core/xld.hpp>
namespace mv::calibration {
std::vector<core::Point2d> detect_checkerboard_points(const core::Image& image,
                                                      std::int64_t rows,
                                                      std::int64_t columns);
std::vector<core::Point2d> detect_circle_grid_points(const core::Image& image,
                                                     std::int64_t rows,
                                                     std::int64_t columns);

void find_calib_object(const core::Image& image,
                       CalibrationDataModel& data,
                       std::int64_t camera_index,
                       std::int64_t calib_object_index,
                       std::int64_t pose_index,
                       core::XldContours* mark_contours);
}
