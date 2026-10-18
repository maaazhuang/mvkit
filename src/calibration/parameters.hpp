#pragma once

#include <string_view>

#include <calibration/model.hpp>

namespace mv::calibration {

void set_calib_data(CalibrationDataModel& data,
                    std::string_view item_type,
                    const core::Tuple& item_index,
                    std::string_view data_name,
                    const core::Tuple& data_value);

[[nodiscard]] core::Tuple get_calib_data(const CalibrationDataModel& data,
                                         std::string_view item_type,
                                         const core::Tuple& item_index,
                                         std::string_view data_name);

}  // namespace mv::calibration
