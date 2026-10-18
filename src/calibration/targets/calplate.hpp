#pragma once

#include <cstdint>
#include <vector>

#include <calibration/camera_parameters.hpp>
#include <calibration/model.hpp>
#include <calibration/targets/calplate_descr.hpp>
#include <core/image.hpp>

namespace mv::calibration {

struct CalplateObservation {
  std::vector<double> rows;
  std::vector<double> columns;
  std::vector<std::int64_t> indices;
};

[[nodiscard]] CalplateObservation detect_calplate(const core::Image& image,
                                                const CalplateDescription& descr);

void refine_calplate_observation(const core::Image& image, const CalplateDescription& descr,
                                 const CameraParameters* camera, CalibrationObservation& obs);

}  // namespace mv::calibration
