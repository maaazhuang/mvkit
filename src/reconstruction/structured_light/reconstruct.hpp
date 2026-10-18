#pragma once

#include <reconstruction/object_model3d.hpp>
#include <reconstruction/structured_light/model.hpp>

namespace mv::reconstruction {

[[nodiscard]] ObjectModel3DData reconstruct_surface_structured_light(
    const StructuredLightModel& model,
    double max_ray_gap = 1e6);

}  // namespace mv::reconstruction
