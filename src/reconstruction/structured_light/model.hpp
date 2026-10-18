#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <map>
#include <string_view>
#include <vector>

#include <calibration/model.hpp>
#include <core/iconic_tuple.hpp>
#include <core/tuple.hpp>
#include <reconstruction/camera_setup.hpp>
#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {

enum class PatternOrientation { Vertical, Horizontal, Both };
enum class StructuredLightPatternType { GrayCode, GrayCodeAndPhaseShift, SingleStripe };
enum class StructuredLightNormalization { Global, InvertedPattern };
enum class PatternKind { SingleStripe, NormalizationBlack, NormalizationWhite, GrayBit, GrayBitInverted, Phase };

struct PatternMetadata {
  PatternOrientation orientation{PatternOrientation::Vertical};
  PatternKind kind{PatternKind::NormalizationBlack};
  std::int64_t bit_index{-1};
  std::int64_t phase_index{-1};
};

struct StructuredLightCalibrationObservation {
  std::int64_t calib_object_index{};
  std::int64_t pose_index{};
  std::vector<double> projector_rows;
  std::vector<double> projector_columns;
  std::vector<std::int64_t> indices;
};

struct StructuredLightParameters {
  std::int64_t pattern_width{1024};
  std::int64_t pattern_height{1024};
  PatternOrientation orientation{PatternOrientation::Both};
  StructuredLightPatternType pattern_type{StructuredLightPatternType::GrayCodeAndPhaseShift};
  StructuredLightNormalization normalization{StructuredLightNormalization::Global};
  std::int64_t min_stripe_width{32};
  std::int64_t single_stripe_width{128};
  std::int64_t min_gray_difference{30};
  bool persistence{false};
  bool quality_output{false}; std::int64_t saturation_margin{};
  double min_phase_amplitude{},intensity_stddev{},projector_coordinate_stddev{};
  std::int64_t hole_max_width{}; double hole_max_depth_jump{.01};
};

class StructuredLightModel {
 public:
  StructuredLightModel(const StructuredLightModel&) = delete;
  StructuredLightModel& operator=(const StructuredLightModel&) = delete;
  StructuredLightModel(StructuredLightModel&&) noexcept = default;
  StructuredLightModel& operator=(StructuredLightModel&&) noexcept = default;

  [[nodiscard]] StructuredLightModel clone() const;
  // 标准 camera_setup_model 参数保存独立副本，并清除旧解码结果。
  void set_camera_setup(std::optional<CameraSetupModel> setup);

  [[nodiscard]] const StructuredLightParameters& parameters() const noexcept { return parameters_; }
  [[nodiscard]] const std::vector<PatternMetadata>& pattern_metadata() const noexcept { return pattern_metadata_; }
  [[nodiscard]] const core::IconicTuple& generated_patterns() const noexcept { return generated_patterns_; }
  [[nodiscard]] const std::map<std::pair<std::int64_t,std::int64_t>, StructuredLightCalibrationObservation>& calibration_observations() const noexcept { return calibration_observations_; }
  [[nodiscard]] const std::optional<CameraSetupModel>& camera_setup() const noexcept { return camera_setup_; }

 private:
  StructuredLightModel() = default;
  friend StructuredLightModel create_structured_light_model(std::string_view);
  friend void set_structured_light_model_param(StructuredLightModel&, std::string_view, const core::Tuple&);
  friend void get_structured_light_model_param(const StructuredLightModel&, std::string_view, core::Tuple*);
  friend core::IconicTuple gen_structured_light_pattern(StructuredLightModel&);
  friend void decode_structured_light_pattern(const core::IconicTuple&, StructuredLightModel&);
  friend core::IconicTuple get_structured_light_object(const StructuredLightModel&, std::string_view);
  friend void add_structured_light_calibration_observation(StructuredLightModel&, std::int64_t, std::int64_t, const core::Tuple&, const core::Tuple&, const core::Tuple&);
  friend void sample_structured_light_calibration_observation(StructuredLightModel&, const calibration::CalibrationDataModel&, std::int64_t, std::int64_t);
  friend CameraSetupModel calibrate_structured_light_setup(StructuredLightModel&, const calibration::CalibrationDataModel&, calibration::CameraParameters, std::int64_t, double*);
  friend void set_structured_light_camera_setup(StructuredLightModel&, const CameraSetupModel&);
  friend ObjectModel3DData reconstruct_surface_structured_light(const StructuredLightModel&, double);

  StructuredLightParameters parameters_;
  core::IconicTuple generated_patterns_;
  std::vector<PatternMetadata> pattern_metadata_;
  core::IconicTuple decoded_correspondence_;
  core::Image decoded_validity_;
  core::Image decoded_contrast_;
  core::Image decoded_confidence_,decoded_reason_;
  core::IconicTuple decoded_coordinate_stddev_;
  bool has_decoded_state_{false};
  std::map<std::pair<std::int64_t, std::int64_t>, StructuredLightCalibrationObservation> calibration_observations_;
  std::optional<CameraSetupModel> camera_setup_;
};

[[nodiscard]] StructuredLightModel create_structured_light_model(std::string_view model_type);
void set_structured_light_model_param(StructuredLightModel& model, std::string_view name, const core::Tuple& value);
void get_structured_light_model_param(const StructuredLightModel& model, std::string_view name, core::Tuple* value);

}  // namespace mv::reconstruction
