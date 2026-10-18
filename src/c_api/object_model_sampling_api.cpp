#include <mvkit/c/mv_reconstruction3d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <reconstruction/normal_estimation.hpp>
#include <reconstruction/sampling.hpp>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "object_model_index.hpp"
#include "tuple_input.hpp"

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
std::shared_ptr<mv::c_api::ObjectModel3DObject> model_at(const TupleInput& models, std::size_t i,
                                                         const char* op) {
  try {
    return models.at(i).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& error) {
    throw mv::core::Exception(error.status(), error.error_code(), op, "ObjectModel3D", i, error.message());
  }
}
void append_model(mv::core::Tuple& result, mv::reconstruction::ObjectModel3DData value) {
  auto handle = std::make_unique<MvObjectModel3DImpl>();
  handle->value = std::move(value);
  result.append(mv::core::Handle::make<mv::c_api::ObjectModel3DObject>(handle.get()));
  mv::c_api::release_opaque(handle.release());
}
std::string_view method_name(const TupleInput& method) {
  if (method.size() != 1) method.fail(ErrorCode::InvalidCardinality, -1, "one Method string required");
  return method.string(0);
}
void parameter_lengths(const TupleInput& names, const TupleInput& values) {
  if (names.size() != values.size())
    values.fail(ErrorCode::InvalidCardinality, -1, "one value per generic parameter required");
}
void positive_count(const TupleInput& values, std::size_t i, std::size_t* output) {
  const auto count = values.number(i);
  if (!std::isfinite(count) || count < 1 || count >= double(std::numeric_limits<std::size_t>::max()))
    values.fail(ErrorCode::InvalidValue, i, "positive count required");
  *output = static_cast<std::size_t>(count);
}
void sample_lengths(const TupleInput& models, const TupleInput& distance) {
  if (distance.size() != 1 && distance.size() != models.size())
    distance.fail(ErrorCode::InvalidCardinality, -1, "one parameter or one per model required");
  for (std::size_t i = 0; i < distance.size(); ++i)
    if (!(distance.number(i) > 0) || !std::isfinite(distance.number(i)))
      distance.fail(ErrorCode::InvalidValue, i, "positive finite sampling parameter required");
}
}  // namespace

extern "C" ::MvStatus mv_sample_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Method,
                                              ::MvTuple SamplingParam, ::MvTuple GenParamName,
                                              ::MvTuple GenParamValue, ::MvTuple* SampledObjectModel3D) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_sample_object_model_3d";
    mv::c_api::ControlOutputs<1> output(op, {SampledObjectModel3D});
    const TupleInput models(ObjectModel3D, op, "ObjectModel3D"), method(Method, op, "Method"),
        distance(SamplingParam, op, "SamplingParam"), names(GenParamName, op, "GenParamName"),
        values(GenParamValue, op, "GenParamValue");
    const auto name = method_name(method);
    if (name != "fast" && name != "fast_compute_normals" && name != "accurate" &&
        name != "accurate_use_normals" && name != "xyz_mapping" && name != "xyz_mapping_compute_normals" &&
        name != "furthest_point" && name != "furthest_point_compute_normals")
      method.fail(ErrorCode::InvalidValue, 0, "unknown sampling method");
    parameter_lengths(names, values);
    sample_lengths(models, distance);
    mv::reconstruction::SamplingOptions options;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (key == "min_num_points") {
        const auto count = values.number(i);
        if (!std::isfinite(count) || count < 0 || count >= double(std::numeric_limits<std::size_t>::max()))
          values.fail(ErrorCode::InvalidValue, i, "nonnegative count required");
        options.minimum_points = std::max<std::size_t>(1, static_cast<std::size_t>(count));
      } else if (key == "max_angle_diff") {
        options.maximum_normal_angle = values.number(i);
        if (!std::isfinite(options.maximum_normal_angle) || options.maximum_normal_angle < 0)
          values.fail(ErrorCode::InvalidValue, i, "nonnegative finite angle required");
        if (name != "accurate_use_normals")
          options.maximum_normal_angle = std::min(180., options.maximum_normal_angle);
      } else
        names.fail(ErrorCode::InvalidValue, i, "unknown sampling parameter");
    }
    for (std::size_t i = 0; i < models.size(); ++i)
      append_model(output[0],
                   mv::reconstruction::sample_points(model_at(models, i, op)->handle()->value, name,
                                                     distance.number(distance.size() == 1 ? 0 : i), options));
    output.Commit();
  });
}
extern "C" ::MvStatus mv_surface_normals_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Method,
                                                       ::MvTuple GenParamName, ::MvTuple GenParamValue,
                                                       ::MvTuple* ObjectModel3DNormals) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_surface_normals_object_model_3d";
    mv::c_api::ControlOutputs<1> output(op, {ObjectModel3DNormals});
    const TupleInput models(ObjectModel3D, op, "ObjectModel3D"), method(Method, op, "Method"),
        names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    if (!models.size()) {
      output.Commit();
      return;
    }
    const auto name = method_name(method);
    parameter_lengths(names, values);
    mv::reconstruction::NormalEstimationOptions options;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (key == "mls_kNN")
        positive_count(values, i, &options.neighbors);
      else if (key == "mls_order") {
        const auto order = values.number(i);
        if (!std::isfinite(order)) values.fail(ErrorCode::InvalidValue, i, "finite MLS order required");
        if (order < 1 || order >= 4)
          values.fail(ErrorCode::InvalidValue, i, "MLS order must truncate to [1,3]");
        options.order = static_cast<int>(order);
      } else if (key == "mls_relative_sigma" || key == "mls_abs_sigma") {
        const auto sigma = values.number(i);
        if (!std::isfinite(sigma) || sigma <= 0)
          values.fail(ErrorCode::InvalidValue, i, "positive finite sigma required");
        (key == "mls_relative_sigma" ? options.relative_sigma : options.absolute_sigma) = sigma;
      } else if (key == "mls_force_inwards") {
        const auto value = values.string(i);
        if (value != "true" && value != "false")
          values.fail(ErrorCode::InvalidValue, i, "boolean string required");
        options.force_inwards = value == "true";
      } else
        names.fail(ErrorCode::InvalidValue, i, "unknown normal parameter");
    }
    for (std::size_t i = 0; i < models.size(); ++i)
      append_model(output[0], mv::reconstruction::estimate_normals(model_at(models, i, op)->handle()->value,
                                                                   name, options));
    output.Commit();
  });
}
