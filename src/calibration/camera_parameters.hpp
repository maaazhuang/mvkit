#pragma once

#include <cstdint>
#include <variant>
#include <string>
#include <string_view>
#include <vector>

#include <core/tuple.hpp>

namespace mv::calibration {

struct CommonCameraParameters {
  double sx{};
  double sy{};
  double cx{};
  double cy{};
  std::int64_t image_width{};
  std::int64_t image_height{};
};

struct AreaScanDivisionParameters : CommonCameraParameters {
  double focus{};
  double kappa{};
};

struct AreaScanPolynomialParameters : CommonCameraParameters {
  double focus{};
  double k1{};
  double k2{};
  double k3{};
  double p1{};
  double p2{};
};

struct AreaScanTelecentricDivisionParameters : CommonCameraParameters {
  double magnification{};
  double kappa{};
};

struct AreaScanTelecentricPolynomialParameters : CommonCameraParameters {
  double magnification{};
  double k1{};
  double k2{};
  double k3{};
  double p1{};
  double p2{};
};

using CameraParameters = std::variant<
    AreaScanDivisionParameters,
    AreaScanPolynomialParameters,
    AreaScanTelecentricDivisionParameters,
    AreaScanTelecentricPolynomialParameters>;

[[nodiscard]] CameraParameters parse_camera_parameters(const core::Tuple& tuple);
[[nodiscard]] core::Tuple camera_parameters_to_tuple(const CameraParameters& params);
[[nodiscard]] bool is_perspective(const CameraParameters& params) noexcept;
[[nodiscard]] const CommonCameraParameters& common_parameters(const CameraParameters& params) noexcept;
[[nodiscard]] std::vector<std::string> optimizable_parameter_names(const CameraParameters& params);
[[nodiscard]] double camera_parameter_value(const CameraParameters& params, std::string_view name);
void set_camera_parameter_value(CameraParameters& params, std::string_view name, double value);

}  // namespace mv::calibration
