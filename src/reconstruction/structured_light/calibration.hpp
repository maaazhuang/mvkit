#pragma once

#include <cstdint>

#include <calibration/model.hpp>
#include <reconstruction/camera_setup.hpp>
#include <reconstruction/structured_light/model.hpp>

namespace mv::reconstruction {

void add_structured_light_calibration_observation(StructuredLightModel& model,
                                                  std::int64_t calib_object_index,
                                                  std::int64_t pose_index,
                                                  const core::Tuple& projector_row,
                                                  const core::Tuple& projector_column,
                                                  const core::Tuple& index);

void sample_structured_light_calibration_observation(StructuredLightModel& model,
                                                     const calibration::CalibrationDataModel& camera_calibration,
                                                     std::int64_t calib_object_index,
                                                     std::int64_t pose_index);

[[nodiscard]] CameraSetupModel calibrate_structured_light_setup(
    StructuredLightModel& model,
    const calibration::CalibrationDataModel& camera_calibration,
    calibration::CameraParameters projector_initial,
    std::int64_t calib_object_index,
    double* reprojection_rmse);

void set_structured_light_camera_setup(StructuredLightModel& model,
                                       const CameraSetupModel& setup);

}  // namespace mv::reconstruction
