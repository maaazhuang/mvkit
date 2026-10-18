#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <cstdint>
#include <limits>
#include <reconstruction/stereo/cost_semantics.hpp>
#include <reconstruction/stereo/disparity.hpp>
#include <reconstruction/stereo/disparity_protocol.hpp>
#include <string_view>
#include <vector>

namespace mv::reconstruction {
namespace {
constexpr char op[] = "binocular_disparity_ms";

[[noreturn]] void fail(core::ErrorCode code, const char* parameter, std::int64_t index,
                       std::string_view message, core::Status status = core::Status::InvalidArgument) {
  throw core::Exception(status, code, op, parameter, index, message);
}

std::int64_t integer(const core::Value& value, const char* parameter, std::int64_t index) {
  if (value.kind() != core::ValueKind::Int64)
    fail(core::ErrorCode::InvalidType, parameter, index, "integer value required");
  return value.as_int64();
}

std::int64_t scalar(const core::Tuple& tuple, const char* parameter, std::int64_t lower, std::int64_t upper) {
  if (tuple.size() != 1)
    fail(core::ErrorCode::InvalidCardinality, parameter, -1, "exactly one integer required");
  const auto value = integer(tuple.at(0), parameter, 0);
  if (value < lower || value > upper)
    fail(core::ErrorCode::InvalidValue, parameter, 0, "integer value is out of range");
  return value;
}

std::string_view text(const core::Value& value, const char* parameter, std::int64_t index) {
  if (value.kind() != core::ValueKind::String)
    fail(core::ErrorCode::InvalidType, parameter, index, "string value required");
  return value.as_string().view();
}

struct Controls {
  std::int64_t surface, edge;
  bool auto_minimum, auto_maximum;
  std::int64_t minimum{}, maximum{};
  core::Tuple names, values;
};

Controls controls(const core::Tuple& minimum, const core::Tuple& maximum, const core::Tuple& surface,
                  const core::Tuple& edge, const core::Tuple& names, const core::Tuple& values) {
  Controls result{scalar(surface, "SurfaceSmoothing", 0, INT32_MAX),
                  scalar(edge, "EdgeSmoothing", 0, INT32_MAX), minimum.empty(), maximum.empty()};
  if (!result.auto_minimum) result.minimum = scalar(minimum, "MinDisparity", -32768, 32768);
  if (!result.auto_maximum) result.maximum = scalar(maximum, "MaxDisparity", -32768, 32768);
  if (!result.auto_minimum && !result.auto_maximum && result.minimum > result.maximum)
    fail(core::ErrorCode::InvalidValue, "MaxDisparity", 0, "minimum exceeds maximum disparity");
  if (names.size() != values.size())
    fail(core::ErrorCode::InvalidCardinality, "GenParamValue", -1, "generic tuple lengths must match");
  std::string_view method = "accurate", similarity = "census_dense";
  std::string_view consistency = "true", subpixel = "true";
  std::int64_t levels = -1, offset = 3;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto name_value = names.at(i), value = values.at(i);
    const auto name = text(name_value, "GenParamName", i);
    if (name == "method") {
      const auto v = text(value, "GenParamValue", i);
      if (v != "accurate" && v != "fast" && v != "very_fast")
        fail(core::ErrorCode::InvalidValue, "GenParamValue", i, "unknown method");
      method = values.string_view(i);
    } else if (name == "similarity_measure") {
      const auto v = text(value, "GenParamValue", i);
      if (v != "census_dense" && v != "census_sparse")
        fail(core::ErrorCode::InvalidValue, "GenParamValue", i, "unknown similarity measure");
      similarity = values.string_view(i);
    } else if (name == "consistency_check" || name == "sub_disparity") {
      const auto v = text(value, "GenParamValue", i);
      if (v != "true" && v != "false")
        fail(core::ErrorCode::InvalidValue, "GenParamValue", i, "boolean string required");
      if (name == "consistency_check")
        consistency = values.string_view(i);
      else
        subpixel = values.string_view(i);
    } else if (name == "num_levels") {
      if (value.kind() == core::ValueKind::String) {
        if (text(value, "GenParamValue", i) != "auto")
          fail(core::ErrorCode::InvalidValue, "GenParamValue", i, "num_levels must be integer or auto");
        levels = -1;
      } else
        levels = integer(value, "GenParamValue", i);
    } else if (name == "disparity_offset") {
      offset = integer(value, "GenParamValue", i);
      if (offset < 0 || offset > INT32_MAX)
        fail(core::ErrorCode::InvalidValue, "GenParamValue", i, "disparity_offset is out of range");
    } else
      fail(core::ErrorCode::InvalidValue, "GenParamName", i, "unknown generic parameter");
  }
  // HALCON accurate 模式检查类型后忽略两个金字塔控制。
  if (method == "accurate") {
    levels = 1;
    offset = 3;
  } else if (levels != -1 && (levels < 1 || levels > 8))
    fail(core::ErrorCode::UnsupportedValue, "GenParamValue", -1,
         "pyramid level outside the implemented range", core::Status::Unsupported);
  for (const auto* name : {"method", "similarity_measure", "consistency_check", "sub_disparity", "num_levels",
                           "disparity_offset"})
    result.names.append(name);
  result.values.append(method).append(similarity).append(consistency).append(subpixel);
  if (levels == -1)
    result.values.append("auto");
  else
    result.values.append(levels);
  result.values.append(offset);
  return result;
}

void validate_image(const core::Image& image, const char* parameter, std::size_t index) {
  if (image.empty() || image.domain().empty())
    fail(core::ErrorCode::InvalidObjectState, parameter, index, "nonempty image domain required");
  if (image.pixel_type() != core::PixelType::UInt8)
    fail(core::ErrorCode::InvalidType, parameter, index, "first image channel must be byte");
  if (image.width() > INT32_MAX || image.height() > INT32_MAX)
    fail(core::ErrorCode::UnsupportedValue, parameter, index,
         "image dimensions exceed the implemented kernel range", core::Status::Unsupported);
}

void valid_domain(DisparityResult& result, const core::Image& left, const core::Image& right) {
  const auto ld = left.domain(), rd = right.domain();
  std::vector<core::RegionRun> runs;
  for (const auto& run : ld.runs()) {
    const auto* d = reinterpret_cast<const float*>(static_cast<const std::byte*>(result.disparity.data()) +
                                                   run.row * result.disparity.stride());
    const auto* s = reinterpret_cast<const float*>(static_cast<const std::byte*>(result.score.data()) +
                                                   run.row * result.score.stride());
    std::int64_t first = -1;
    for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
      const bool finite = std::isfinite(d[c]) && std::isfinite(s[c]);
      const auto rc = finite ? std::llround(double(c) + d[c]) : -1;
      const bool valid = finite && rd.contains(run.row, rc);
      if (valid && first == -1) first = c;
      if (!valid && first != -1) {
        runs.push_back({run.row, first, c - 1});
        first = -1;
      }
    }
    if (first != -1) runs.push_back({run.row, first, run.col_end});
  }
  auto domain = core::Region::from_runs(runs);
  result.disparity = result.disparity.reduce_domain(domain);
  result.score = result.score.reduce_domain(domain);
}
}  // namespace

StereoDisparityImages binocular_disparity_ms_protocol(const core::IconicTuple& left,
                                                      const core::IconicTuple& right,
                                                      const core::Tuple& minimum, const core::Tuple& maximum,
                                                      const core::Tuple& surface, const core::Tuple& edge,
                                                      const core::Tuple& names, const core::Tuple& values) {
  left.require_kind(core::IconicKind::Image, op);
  right.require_kind(core::IconicKind::Image, op);
  if (left.empty() || right.empty()) return {};
  if (left.size() != right.size())
    fail(core::ErrorCode::InvalidCardinality, "ImageRect2", -1, "image object array lengths must match");
  const auto p = controls(minimum, maximum, surface, edge, names, values);
  std::vector<core::Image> disparities, scores;
  disparities.reserve(left.size());
  scores.reserve(left.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    const auto& a = left.at(i).as_image();
    const auto& b = right.at(i).as_image();
    validate_image(a, "ImageRect1", i);
    validate_image(b, "ImageRect2", i);
    if (a.height() != b.height())
      fail(core::ErrorCode::InvalidValue, "ImageRect2", i, "image heights must match");
    const auto minimum_value = p.auto_minimum ? -(std::min<std::int64_t>)(32768, a.width() - 1) : p.minimum;
    const auto maximum_value = p.auto_maximum ? (std::min<std::int64_t>)(32768, b.width() - 1) : p.maximum;
    if (minimum_value > maximum_value)
      fail(core::ErrorCode::InvalidValue, "MaxDisparity", 0, "minimum exceeds maximum disparity");
    auto result =
        binocular_disparity_ms(a, b, minimum_value, maximum_value, p.surface, p.edge, p.names, p.values);
    valid_domain(result, a, b);
    disparities.push_back(std::move(result.disparity));
    scores.push_back(std::move(result.score));
  }
  return {core::make_image_tuple(std::move(disparities)), core::make_image_tuple(std::move(scores))};
}

StereoDisparityQualityImages binocular_disparity_ms_with_quality_protocol(
    const core::IconicTuple& left, const core::IconicTuple& right, const core::Tuple& minimum,
    const core::Tuple& maximum, const core::Tuple& surface, const core::Tuple& edge, const core::Tuple& names,
    const core::Tuple& values, const core::Tuple& min_confidence) {
  constexpr char quality_op[] = "binocular_disparity_ms_with_quality";
  try {
    left.require_kind(core::IconicKind::Image, quality_op);
    right.require_kind(core::IconicKind::Image, quality_op);
    if (left.empty() || right.empty()) return {};
    if (left.size() != right.size())
      fail(core::ErrorCode::InvalidCardinality, "ImageRect2", -1, "image object array lengths must match");
    auto p = controls(minimum, maximum, surface, edge, names, values);
    const auto quality_number = [](const core::Tuple& tuple, const char* parameter, double upper) {
      if (tuple.size() != 1)
        fail(core::ErrorCode::InvalidCardinality, parameter, -1, "exactly one numeric value required");
      const auto item = tuple.at(0);
      if (item.kind() != core::ValueKind::Int64 && item.kind() != core::ValueKind::Real64)
        fail(core::ErrorCode::InvalidType, parameter, 0, "numeric value required",
             core::Status::TypeMismatch);
      const double number = item.as_real64();
      if (!std::isfinite(number) || number < 0 || number > upper)
        fail(core::ErrorCode::InvalidValue, parameter, 0, "quality value is out of range");
      return number;
    };
    const auto confidence = quality_number(min_confidence, "MinConfidence", 1);
    // 只有扩展加入私有内核控制；正式 generic 已通过上方不变的六名称协议。
    p.names.append("quality_output").append("min_confidence");
    p.values.append("true").append(confidence);
    std::vector<core::Image> disparities, scores, confidences, validities, reasons;
    disparities.reserve(left.size());
    scores.reserve(left.size());
    confidences.reserve(left.size());
    validities.reserve(left.size());
    reasons.reserve(left.size());
    for (std::size_t i = 0; i < left.size(); ++i) {
      const auto& a = left.at(i).as_image();
      const auto& b = right.at(i).as_image();
      validate_image(a, "ImageRect1", i);
      validate_image(b, "ImageRect2", i);
      if (a.height() != b.height())
        fail(core::ErrorCode::InvalidValue, "ImageRect2", i, "image heights must match");
      const auto minimum_value = p.auto_minimum ? -(std::min<std::int64_t>)(32768, a.width() - 1) : p.minimum;
      const auto maximum_value = p.auto_maximum ? (std::min<std::int64_t>)(32768, b.width() - 1) : p.maximum;
      if (minimum_value > maximum_value)
        fail(core::ErrorCode::InvalidValue, "MaxDisparity", 0, "minimum exceeds maximum disparity");
      auto result =
          binocular_disparity_ms(a, b, minimum_value, maximum_value, p.surface, p.edge, p.names, p.values);
      if (result.quality_channels.size() != 3)
        fail(core::ErrorCode::InternalFailure, "Output", i,
             "quality kernel returned an invalid channel count", core::Status::InternalError);
      valid_domain(result, a, b);
      disparities.push_back(std::move(result.disparity));
      scores.push_back(std::move(result.score));
      confidences.push_back(std::move(result.quality_channels[0]));
      validities.push_back(std::move(result.quality_channels[1]));
      reasons.push_back(std::move(result.quality_channels[2]));
    }
    return {core::make_image_tuple(std::move(disparities)), core::make_image_tuple(std::move(scores)),
            core::make_image_tuple(std::move(confidences)), core::make_image_tuple(std::move(validities)),
            core::make_image_tuple(std::move(reasons))};
  } catch (const core::Exception& error) {
    throw core::Exception(error.status(), error.error_code(), quality_op, error.parameter(),
                          error.parameter_index(), error.message());
  }
}
}  // namespace mv::reconstruction
