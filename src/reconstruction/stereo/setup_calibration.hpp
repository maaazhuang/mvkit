#pragma once

#include <cstdint>

#include <calibration/model.hpp>
#include <reconstruction/camera_setup.hpp>

namespace mv::reconstruction {

[[nodiscard]] CameraSetupModel calibrate_binocular_camera_setup(
    const calibration::CalibrationDataModel& camera0,
    const calibration::CalibrationDataModel& camera1,
    std::int64_t calib_object_index);

}  // namespace mv::reconstruction
