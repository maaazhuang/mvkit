#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>
#include <core/geometry_types.hpp>

namespace mv::calibration {

#ifndef MVKIT_CALIBRATION_CALPLATE_FINDER_DEFINED
#define MVKIT_CALIBRATION_CALPLATE_FINDER_DEFINED
struct CalplateFinder {
  std::int64_t column{};
  std::int64_t row{};
};
#endif

struct CalplateDescription {
  std::int64_t rows{};
  std::int64_t columns{};
  double z{};
  bool light_on_dark{true};
  std::vector<Point3d> points;
  std::vector<double> radii;
  std::vector<CalplateFinder> finders;
};

[[nodiscard]] std::filesystem::path resolve_calib_descr(std::string_view descr);
[[nodiscard]] CalplateDescription parse_calplate_cpd(const std::filesystem::path& path);

}  // namespace mv::calibration
