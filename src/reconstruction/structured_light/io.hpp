#pragma once

#include <filesystem>

#include <reconstruction/structured_light/model.hpp>

namespace mv::reconstruction {

void write_structured_light_model(const std::filesystem::path& path,
                                  const StructuredLightModel& model);
[[nodiscard]] StructuredLightModel read_structured_light_model(
    const std::filesystem::path& path);

}  // namespace mv::reconstruction
