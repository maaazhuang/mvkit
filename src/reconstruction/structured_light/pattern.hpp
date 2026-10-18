#pragma once

#include <core/iconic_tuple.hpp>
#include <reconstruction/structured_light/model.hpp>

namespace mv::reconstruction {

[[nodiscard]] core::IconicTuple gen_structured_light_pattern(StructuredLightModel& model);

}  // namespace mv::reconstruction
