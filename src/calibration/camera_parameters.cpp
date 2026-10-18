#include <calibration/camera_parameters.hpp>

#include <cmath>
#include <string_view>

#include <core/exception.hpp>

namespace mv::calibration {
namespace {
double number_at(const core::Tuple& tuple, std::size_t index, std::string_view where) {
  try {
    const double value = tuple.as_real64(index);
    if (!std::isfinite(value)) {
      throw core::Exception(core::Status::InvalidArgument, where, "camera parameter must be finite");
    }
    return value;
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::InvalidArgument) throw;
    throw core::Exception(core::Status::InvalidArgument, where, "camera parameter must be numeric");
  }
}

std::int64_t integer_at(const core::Tuple& tuple, std::size_t index, std::string_view where) {
  try {
    return tuple.as_int64(index);
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, where, "image size must be an integer");
  }
}

CommonCameraParameters common(const core::Tuple& tuple, std::size_t sx_index) {
  CommonCameraParameters c;
  c.sx = number_at(tuple, sx_index, "parse_camera_parameters");
  c.sy = number_at(tuple, sx_index + 1, "parse_camera_parameters");
  c.cx = number_at(tuple, sx_index + 2, "parse_camera_parameters");
  c.cy = number_at(tuple, sx_index + 3, "parse_camera_parameters");
  c.image_width = integer_at(tuple, sx_index + 4, "parse_camera_parameters");
  c.image_height = integer_at(tuple, sx_index + 5, "parse_camera_parameters");
  if (c.sx <= 0.0 || c.sy <= 0.0 || c.image_width <= 0 || c.image_height <= 0) {
    throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "pixel size and image dimensions must be positive");
  }
  return c;
}

template <typename T>
void assign_common(T& dst, const CommonCameraParameters& src) {
  static_cast<CommonCameraParameters&>(dst) = src;
}

}  // namespace

CameraParameters parse_camera_parameters(const core::Tuple& tuple) {
  if (tuple.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "camera parameter tuple is empty");
  }
  std::string_view type;
  try {
    type = tuple.string_view(0);
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "camera type must be a string");
  }

  if (type == "area_scan_division") {
    if (tuple.size() != 9) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "area_scan_division expects 9 tuple elements");
    AreaScanDivisionParameters p;
    p.focus = number_at(tuple, 1, "parse_camera_parameters");
    p.kappa = number_at(tuple, 2, "parse_camera_parameters");
    assign_common(p, common(tuple, 3));
    if (p.focus <= 0.0) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "focus must be positive");
    return p;
  }
  if (type == "area_scan_polynomial") {
    if (tuple.size() != 13) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "area_scan_polynomial expects 13 tuple elements");
    AreaScanPolynomialParameters p;
    p.focus = number_at(tuple, 1, "parse_camera_parameters");
    p.k1 = number_at(tuple, 2, "parse_camera_parameters");
    p.k2 = number_at(tuple, 3, "parse_camera_parameters");
    p.k3 = number_at(tuple, 4, "parse_camera_parameters");
    p.p1 = number_at(tuple, 5, "parse_camera_parameters");
    p.p2 = number_at(tuple, 6, "parse_camera_parameters");
    assign_common(p, common(tuple, 7));
    if (p.focus <= 0.0) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "focus must be positive");
    return p;
  }
  if (type == "area_scan_telecentric_division") {
    if (tuple.size() != 9) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "area_scan_telecentric_division expects 9 tuple elements");
    AreaScanTelecentricDivisionParameters p;
    p.magnification = number_at(tuple, 1, "parse_camera_parameters");
    p.kappa = number_at(tuple, 2, "parse_camera_parameters");
    assign_common(p, common(tuple, 3));
    if (p.magnification <= 0.0) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "magnification must be positive");
    return p;
  }
  if (type == "area_scan_telecentric_polynomial") {
    if (tuple.size() != 13) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "area_scan_telecentric_polynomial expects 13 tuple elements");
    AreaScanTelecentricPolynomialParameters p;
    p.magnification = number_at(tuple, 1, "parse_camera_parameters");
    p.k1 = number_at(tuple, 2, "parse_camera_parameters");
    p.k2 = number_at(tuple, 3, "parse_camera_parameters");
    p.k3 = number_at(tuple, 4, "parse_camera_parameters");
    p.p1 = number_at(tuple, 5, "parse_camera_parameters");
    p.p2 = number_at(tuple, 6, "parse_camera_parameters");
    assign_common(p, common(tuple, 7));
    if (p.magnification <= 0.0) throw core::Exception(core::Status::InvalidArgument, "parse_camera_parameters", "magnification must be positive");
    return p;
  }
  throw core::Exception(core::Status::Unsupported, "parse_camera_parameters", "unsupported camera type");
}

core::Tuple camera_parameters_to_tuple(const CameraParameters& params) {
  return std::visit([](const auto& p) {
    using T = std::decay_t<decltype(p)>;
    constexpr bool perspective = std::is_same_v<T, AreaScanDivisionParameters> ||
                                 std::is_same_v<T, AreaScanPolynomialParameters>;
    const double scale = [&] {
      if constexpr (perspective) return p.focus;
      else return p.magnification;
    }();
    using core::Value;
    // 相机描述始终为定长混合元组；字段类型与顺序保持原样，一次分配完整载荷。
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters> ||
                  std::is_same_v<T, AreaScanTelecentricDivisionParameters>) {
      return core::Tuple{Value{perspective ? "area_scan_division" : "area_scan_telecentric_division"},
                         Value{scale}, Value{p.kappa}, Value{p.sx}, Value{p.sy}, Value{p.cx}, Value{p.cy},
                         Value{p.image_width}, Value{p.image_height}};
    } else {
      return core::Tuple{Value{perspective ? "area_scan_polynomial" : "area_scan_telecentric_polynomial"},
                         Value{scale}, Value{p.k1}, Value{p.k2}, Value{p.k3}, Value{p.p1}, Value{p.p2},
                         Value{p.sx}, Value{p.sy}, Value{p.cx}, Value{p.cy}, Value{p.image_width}, Value{p.image_height}};
    }
  }, params);
}

bool is_perspective(const CameraParameters& params) noexcept {
  return std::holds_alternative<AreaScanDivisionParameters>(params) ||
         std::holds_alternative<AreaScanPolynomialParameters>(params);
}

const CommonCameraParameters& common_parameters(const CameraParameters& params) noexcept {
  return std::visit([](const auto& p) -> const CommonCameraParameters& { return p; }, params);
}


std::vector<std::string> optimizable_parameter_names(const CameraParameters& params) {
  return std::visit([](const auto& p) {
    using T = std::decay_t<decltype(p)>;
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters>) return std::vector<std::string>{"focus","kappa","sx","sy","cx","cy"};
    if constexpr (std::is_same_v<T, AreaScanPolynomialParameters>) return std::vector<std::string>{"focus","k1","k2","k3","p1","p2","sx","sy","cx","cy"};
    if constexpr (std::is_same_v<T, AreaScanTelecentricDivisionParameters>) return std::vector<std::string>{"magnification","kappa","sx","sy","cx","cy"};
    return std::vector<std::string>{"magnification","k1","k2","k3","p1","p2","sx","sy","cx","cy"};
  }, params);
}

double camera_parameter_value(const CameraParameters& params, std::string_view name) {
  return std::visit([&](const auto& p) -> double {
    using T = std::decay_t<decltype(p)>;
    if (name == "sx") return p.sx;
    if (name == "sy") return p.sy;
    if (name == "cx") return p.cx;
    if (name == "cy") return p.cy;
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters>) { if (name=="focus") return p.focus; if(name=="kappa") return p.kappa; }
    if constexpr (std::is_same_v<T, AreaScanPolynomialParameters>) { if(name=="focus") return p.focus; if(name=="k1") return p.k1; if(name=="k2") return p.k2; if(name=="k3") return p.k3; if(name=="p1") return p.p1; if(name=="p2") return p.p2; }
    if constexpr (std::is_same_v<T, AreaScanTelecentricDivisionParameters>) { if(name=="magnification") return p.magnification; if(name=="kappa") return p.kappa; }
    if constexpr (std::is_same_v<T, AreaScanTelecentricPolynomialParameters>) { if(name=="magnification") return p.magnification; if(name=="k1") return p.k1; if(name=="k2") return p.k2; if(name=="k3") return p.k3; if(name=="p1") return p.p1; if(name=="p2") return p.p2; }
    throw core::Exception(core::Status::InvalidArgument, "camera_parameter_value", "parameter name is not valid for camera model");
  }, params);
}

void set_camera_parameter_value(CameraParameters& params, std::string_view name, double value) {
  if (!std::isfinite(value)) throw core::Exception(core::Status::InvalidArgument, "set_camera_parameter_value", "parameter must be finite");
  std::visit([&](auto& p) {
    using T = std::decay_t<decltype(p)>;
    if (name=="sx") { if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","sx must be positive"); p.sx=value; return; }
    if (name=="sy") { if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","sy must be positive"); p.sy=value; return; }
    if (name=="cx") { p.cx=value; return; }
    if (name=="cy") { p.cy=value; return; }
    if constexpr (std::is_same_v<T, AreaScanDivisionParameters>) { if(name=="focus"){if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","focus must be positive");p.focus=value;return;} if(name=="kappa"){p.kappa=value;return;} }
    if constexpr (std::is_same_v<T, AreaScanPolynomialParameters>) { if(name=="focus"){if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","focus must be positive");p.focus=value;return;} if(name=="k1"){p.k1=value;return;} if(name=="k2"){p.k2=value;return;} if(name=="k3"){p.k3=value;return;} if(name=="p1"){p.p1=value;return;} if(name=="p2"){p.p2=value;return;} }
    if constexpr (std::is_same_v<T, AreaScanTelecentricDivisionParameters>) { if(name=="magnification"){if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","magnification must be positive");p.magnification=value;return;} if(name=="kappa"){p.kappa=value;return;} }
    if constexpr (std::is_same_v<T, AreaScanTelecentricPolynomialParameters>) { if(name=="magnification"){if(value<=0) throw core::Exception(core::Status::InvalidArgument,"set_camera_parameter_value","magnification must be positive");p.magnification=value;return;} if(name=="k1"){p.k1=value;return;} if(name=="k2"){p.k2=value;return;} if(name=="k3"){p.k3=value;return;} if(name=="p1"){p.p1=value;return;} if(name=="p2"){p.p2=value;return;} }
    throw core::Exception(core::Status::InvalidArgument, "set_camera_parameter_value", "parameter name is not valid for camera model");
  }, params);
}

}  // namespace mv::calibration
