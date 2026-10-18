#pragma once

#include <string_view>

#include <core/iconic_tuple.hpp>
#include <reconstruction/structured_light/model.hpp>

namespace mv::reconstruction {

void decode_structured_light_pattern(const core::IconicTuple& camera_images,
                                     StructuredLightModel& model);
[[nodiscard]] core::IconicTuple get_structured_light_object(const StructuredLightModel& model,
                                                            std::string_view object_name);

}  // namespace mv::reconstruction
