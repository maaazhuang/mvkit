#include <reconstruction/structured_light/model.hpp>
#include <calibration/camera_parameters.hpp>

#include <string>
#include <cmath>

#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

void require_single(const core::Tuple& value, const char* operation) {
  if (value.size() != 1) {
    throw core::Exception(core::Status::InvalidArgument, operation, "parameter value must contain exactly one item");
  }
}

bool power_of_two(std::int64_t value) {
  return value > 0 && (static_cast<std::uint64_t>(value) & (static_cast<std::uint64_t>(value) - 1U)) == 0U;
}

std::int64_t next_power_of_two(std::int64_t value) {
  std::uint64_t result = 1;
  while (result < static_cast<std::uint64_t>(value)) result <<= 1U;
  return static_cast<std::int64_t>(result);
}

std::int64_t default_single_stripe_width(const StructuredLightParameters& p) {
  const auto maximum = next_power_of_two((std::min)(p.pattern_width, p.pattern_height));
  std::int64_t result = p.min_stripe_width;
  std::int64_t remaining = maximum / p.min_stripe_width;
  while (remaining >= 4) {
    result *= 2;
    remaining /= 4;
  }
  return result;
}

bool parse_persistence(const core::Value& value) {
  if (value.kind() == core::ValueKind::Int64) {
    const auto integer = value.as_int64();
    if (integer == 0 || integer == 1) return integer != 0;
  }
  if (value.kind() == core::ValueKind::String) {
    const auto text = value.as_string().view();
    if (text == "true") return true;
    if (text == "false") return false;
  }
  throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param",
                        "persistence must be 'true', 'false', 1, or 0");
}

void validate_parameters(const StructuredLightParameters& p) {
  if (p.pattern_width < 0 || p.pattern_height < 0) {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "pattern dimensions must be nonnegative until configured");
  }
  if (p.min_stripe_width < 4 || !power_of_two(p.min_stripe_width)) {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "min_stripe_width must be a power of two not smaller than 4");
  }
  if (p.pattern_width <= p.min_stripe_width / 2 ||
      p.pattern_height <= p.min_stripe_width / 2 ||
      p.min_stripe_width > next_power_of_two(p.pattern_width) ||
      p.min_stripe_width > next_power_of_two(p.pattern_height)) {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param",
                          "min_stripe_width is incompatible with the pattern dimensions");
  }
  if (p.pattern_type == StructuredLightPatternType::SingleStripe &&
      (p.single_stripe_width < p.min_stripe_width ||
       !power_of_two(p.single_stripe_width) ||
       p.pattern_width <= p.single_stripe_width / 2 ||
       p.pattern_height <= p.single_stripe_width / 2 ||
       p.single_stripe_width > next_power_of_two(p.pattern_width) ||
       p.single_stripe_width > next_power_of_two(p.pattern_height))) {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param",
                          "single_stripe_width is incompatible with the pattern dimensions");
  }
  if (p.min_gray_difference < 0) {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "min_gray_difference must be nonnegative");
  }
}

const char* orientation_name(PatternOrientation orientation) {
  switch (orientation) {
    case PatternOrientation::Vertical: return "vertical";
    case PatternOrientation::Horizontal: return "horizontal";
    case PatternOrientation::Both: return "both";
  }
  return "vertical";
}

const char* type_name(StructuredLightPatternType type) {
  switch (type) {
    case StructuredLightPatternType::GrayCode: return "gray_code";
    case StructuredLightPatternType::GrayCodeAndPhaseShift: return "gray_code_and_phase_shift";
    case StructuredLightPatternType::SingleStripe: return "single_stripe";
  }
  return "gray_code_and_phase_shift";
}

}  // namespace

StructuredLightModel create_structured_light_model(std::string_view model_type) {
  if (model_type != "3d_reconstruction") {
    throw core::Exception(core::Status::InvalidArgument, "create_structured_light_model", "v1 supports only model_type=3d_reconstruction");
  }
  return StructuredLightModel{};
}

StructuredLightModel StructuredLightModel::clone() const {
  StructuredLightModel result;
  result.parameters_ = parameters_;
  result.generated_patterns_ = generated_patterns_;
  result.pattern_metadata_ = pattern_metadata_;
  result.decoded_correspondence_ = decoded_correspondence_;
  result.decoded_validity_ = decoded_validity_;
  result.decoded_contrast_ = decoded_contrast_;
  result.decoded_confidence_=decoded_confidence_;result.decoded_reason_=decoded_reason_;
  result.decoded_coordinate_stddev_=decoded_coordinate_stddev_;
  result.has_decoded_state_ = has_decoded_state_;
  result.calibration_observations_ = calibration_observations_;
  result.camera_setup_ = camera_setup_;
  return result;
}

void StructuredLightModel::set_camera_setup(std::optional<CameraSetupModel> setup) {
  if (setup) {
    if (setup->camera_count() != 2 || !setup->complete())
      throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue,
                            "set_structured_light_model_param", "camera_setup_model", -1,
                            "a complete two-device setup is required");
    if (!calibration::is_perspective(setup->camera(1)))
      throw core::Exception(core::Status::Unsupported, core::ErrorCode::UnsupportedValue,
                            "set_structured_light_model_param", "camera_setup_model", -1,
                            "projector must be perspective");
    if (calibration::common_parameters(setup->camera(1)).image_width != parameters_.pattern_width ||
        calibration::common_parameters(setup->camera(1)).image_height != parameters_.pattern_height)
      throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue,
                            "set_structured_light_model_param", "camera_setup_model", -1,
                            "projector dimensions must match pattern dimensions");
    // 官方按设备索引定义相机与逆相机，不要求公开的私有角色枚举。
    auto sensor = CameraSetupModel::create("camera_projector");
    sensor.set_camera(0, setup->camera(0), setup->camera_pose(0));
    sensor.set_camera(1, setup->camera(1), setup->camera_pose(1));
    setup = std::move(sensor);
  }
  camera_setup_ = std::move(setup);
  decoded_correspondence_ = {};
  decoded_validity_ = {};
  decoded_contrast_ = {};
  decoded_confidence_ = {};
  decoded_reason_ = {};
  decoded_coordinate_stddev_ = {};
  has_decoded_state_ = false;
}

void set_structured_light_model_param(StructuredLightModel& model,
                                      std::string_view name,
                                      const core::Tuple& value) {
  require_single(value, "set_structured_light_model_param");
  auto candidate = model.parameters_;
  if (name == "pattern_width") {
    candidate.pattern_width = value.at(0).as_int64();
    if (candidate.pattern_width <= 0) throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "pattern_width must be positive");
  } else if (name == "pattern_height") {
    candidate.pattern_height = value.at(0).as_int64();
    if (candidate.pattern_height <= 0) throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "pattern_height must be positive");
  } else if (name == "pattern_orientation") {
    const std::string x(value.string_view(0));
    if (x == "vertical") candidate.orientation = PatternOrientation::Vertical;
    else if (x == "horizontal") candidate.orientation = PatternOrientation::Horizontal;
    else if (x == "both") candidate.orientation = PatternOrientation::Both;
    else throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "invalid pattern_orientation");
  } else if (name == "pattern_type") {
    const std::string x(value.string_view(0));
    if (x == "gray_code") candidate.pattern_type = StructuredLightPatternType::GrayCode;
    else if (x == "gray_code_and_phase_shift") candidate.pattern_type = StructuredLightPatternType::GrayCodeAndPhaseShift;
    else if (x == "single_stripe") {
      candidate.pattern_type = StructuredLightPatternType::SingleStripe;
      candidate.single_stripe_width = default_single_stripe_width(candidate);
    }
    else throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "invalid pattern_type");
  } else if (name == "normalization") {
    const std::string x(value.string_view(0));
    if (x == "global") candidate.normalization = StructuredLightNormalization::Global;
    else if (x == "inverted_pattern") candidate.normalization = StructuredLightNormalization::InvertedPattern;
    else {
      throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "invalid normalization");
    }
  } else if (name == "min_stripe_width") {
    candidate.min_stripe_width = value.at(0).as_int64();
  } else if (name == "single_stripe_width") {
    if (candidate.pattern_type != StructuredLightPatternType::SingleStripe) {
      throw core::Exception(core::Status::InvalidState, "set_structured_light_model_param",
                            "single_stripe_width requires pattern_type=single_stripe");
    }
    candidate.single_stripe_width = value.at(0).as_int64();
  } else if (name == "min_gray_difference") {
    candidate.min_gray_difference = value.at(0).as_int64();
  } else if(name=="quality_output") {
    candidate.quality_output=parse_persistence(value.at(0));
  } else if(name=="saturation_margin") {
    const auto n=value.at(0).as_int64();
    if(n<0 || n>127)throw core::Exception(core::Status::InvalidArgument,"set_structured_light_model_param","invalid saturation_margin");
    candidate.saturation_margin=n;
  } else if(name=="hole_max_width") {
    const auto n=value.at(0).as_int64();
    if(n<0 || n>4096)throw core::Exception(core::Status::InvalidArgument,"set_structured_light_model_param","invalid hole_max_width");
    candidate.hole_max_width=n;
  } else if(name=="min_phase_amplitude" || name=="intensity_stddev" || name=="projector_coordinate_stddev" || name=="hole_max_depth_jump") {
    const double n=value.at(0).as_real64();
    if(!std::isfinite(n) || n<0)throw core::Exception(core::Status::InvalidArgument,"set_structured_light_model_param","invalid quality parameter");
    if(name=="min_phase_amplitude")candidate.min_phase_amplitude=n;
    else if(name=="intensity_stddev")candidate.intensity_stddev=n;
    else if(name=="projector_coordinate_stddev")candidate.projector_coordinate_stddev=n;
    else candidate.hole_max_depth_jump=n;
  } else if (name == "persistence") {
    candidate.persistence = parse_persistence(value.at(0));
  } else {
    throw core::Exception(core::Status::InvalidArgument, "set_structured_light_model_param", "unknown structured-light parameter");
  }
  validate_parameters(candidate);
  model.parameters_ = candidate;
  model.generated_patterns_ = {};
  model.pattern_metadata_.clear();
  model.decoded_correspondence_ = {};
  model.decoded_validity_ = {};
  model.decoded_contrast_ = {};
  model.decoded_confidence_={};model.decoded_reason_={};model.decoded_coordinate_stddev_={};
  model.has_decoded_state_ = false;
  model.calibration_observations_.clear();
}

void get_structured_light_model_param(const StructuredLightModel& model,
                                      std::string_view name,
                                      core::Tuple* value) {
  if (!value) throw core::Exception(core::Status::InvalidArgument, "get_structured_light_model_param", "output must not be null");
  core::Tuple out;
  if (name == "type") out.append("3d_reconstruction");
  else if (name == "pattern_width") out.append(model.parameters_.pattern_width);
  else if (name == "pattern_height") out.append(model.parameters_.pattern_height);
  else if (name == "pattern_orientation") out.append(orientation_name(model.parameters_.orientation));
  else if (name == "pattern_type") out.append(type_name(model.parameters_.pattern_type));
  else if (name == "normalization") {
    out.append(model.parameters_.normalization == StructuredLightNormalization::Global
                   ? "global" : "inverted_pattern");
  }
  else if (name == "min_stripe_width") out.append(model.parameters_.min_stripe_width);
  else if (name == "single_stripe_width") {
    if (model.parameters_.pattern_type != StructuredLightPatternType::SingleStripe) {
      throw core::Exception(core::Status::InvalidState, "get_structured_light_model_param",
                            "single_stripe_width requires pattern_type=single_stripe");
    }
    out.append(model.parameters_.single_stripe_width);
  }
  else if (name == "min_gray_difference") out.append(model.parameters_.min_gray_difference);
  else if(name=="quality_output")out.append(model.parameters_.quality_output?"true":"false");
  else if(name=="min_phase_amplitude")out.append(model.parameters_.min_phase_amplitude);
  else if(name=="intensity_stddev")out.append(model.parameters_.intensity_stddev);
  else if(name=="projector_coordinate_stddev")out.append(model.parameters_.projector_coordinate_stddev);
  else if(name=="saturation_margin")out.append(model.parameters_.saturation_margin);
  else if(name=="hole_max_width")out.append(model.parameters_.hole_max_width);
  else if(name=="hole_max_depth_jump")out.append(model.parameters_.hole_max_depth_jump);
  else if (name == "persistence") out.append(model.parameters_.persistence ? "true" : "false");
  else throw core::Exception(core::Status::InvalidArgument, "get_structured_light_model_param", "unknown structured-light parameter");
  *value = std::move(out);
}

}  // namespace mv::reconstruction
