#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>
#include <core/geometry_types.hpp>

namespace mv::core {
class Image;
struct Point2d;
}

namespace mv::calibration {

struct CaltabDescription {
  std::int64_t rows{};
  std::int64_t columns{};
  double z{};
  bool light_on_dark{false};
  std::vector<Point3d> points;
  std::vector<double> radii;
  std::optional<std::array<double, 4>> rim;
  std::optional<std::array<double, 4>> frame;
  std::optional<std::array<double, 4>> triangle;
  std::optional<double> frame_width;
};

[[nodiscard]] CaltabDescription parse_caltab_descr(const std::filesystem::path& path);

[[nodiscard]] std::vector<core::Point2d> detect_caltab(
    const core::Image& image, const CaltabDescription& description);

}  // namespace mv::calibration
