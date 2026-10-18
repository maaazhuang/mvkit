#include <mvkit/c/mv_reconstruction3d.h>

#include <cmath>
#include <map>
#include <reconstruction/pair_registration.hpp>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "object_model_index.hpp"
#include "tuple_input.hpp"
namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
std::shared_ptr<mv::c_api::ObjectModel3DObject> model(const TupleInput& models, const char* op,
                                                      const char* parameter) {
  if (models.size() != 1) models.fail(ErrorCode::InvalidCardinality, -1, "exactly one model required");
  try {
    return models.at(0).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& e) {
    throw mv::core::Exception(e.status(), e.error_code(), op, parameter, 0, e.message());
  }
}
void lengths(const TupleInput& names, const TupleInput& values) {
  if (names.size() != values.size())
    values.fail(ErrorCode::InvalidCardinality, -1, "one generic value per name required");
}
std::string_view method(const TupleInput& value) {
  if (value.size() != 1) value.fail(ErrorCode::InvalidCardinality, -1, "one Method required");
  return value.string(0);
}
bool unit_interval(double value) { return value <= 0 || value >= 1; }
}  // namespace
extern "C" ::MvStatus mv_register_object_model_3d_pair(::MvTuple ObjectModel3D1, ::MvTuple ObjectModel3D2,
                                                     ::MvTuple Method, ::MvTuple GenParamName,
                                                     ::MvTuple GenParamValue, ::MvTuple* Pose,
                                                     ::MvTuple* Score) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_register_object_model_3d_pair";
    mv::c_api::ControlOutputs<2> outputs(op, {Pose, Score});
    const TupleInput first(ObjectModel3D1, op, "ObjectModel3D1"),
        second(ObjectModel3D2, op, "ObjectModel3D2"), name(Method, op, "Method"),
        names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    const auto a = model(first, op, "ObjectModel3D1"), b = model(second, op, "ObjectModel3D2");
    const auto algorithm = method(name);
    if (algorithm != "icp" && algorithm != "matching")
      name.fail(ErrorCode::InvalidValue, 0, "icp or matching required");
    lengths(names, values);
    std::map<std::string_view, std::size_t> last;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (key != "default_parameters" && key != "rel_sampling_distance" &&
          key != "rel_sampling_distance_obj1" && key != "rel_sampling_distance_obj2" &&
          key != "key_point_fraction" && key != "key_point_fraction_obj1" &&
          key != "key_point_fraction_obj2" && key != "pose_ref_num_steps" && key != "pose_ref_sub_sampling" &&
          key != "pose_ref_dist_threshold_rel" && key != "pose_ref_dist_threshold_abs" &&
          key != "model_invert_normals")
        names.fail(ErrorCode::InvalidValue, i, "unknown registration parameter");
      last[key] = i;
    }
    mv::reconstruction::PairRegistrationOptions options;
    if (const auto it = last.find("default_parameters"); it != last.end()) {
      const auto value = values.string(it->second);
      if (value == "fast") {
        options.preset = mv::reconstruction::PairRegistrationPreset::Fast;
        options.refinement_steps = 2;
        options.sampling_distance[0] = options.sampling_distance[1] = .07;
      } else if (value == "robust") {
        options.preset = mv::reconstruction::PairRegistrationPreset::Robust;
        options.refinement_steps = 10;
        options.sampling_distance[0] = options.sampling_distance[1] = .03;
      } else if (value != "accurate")
        values.fail(ErrorCode::InvalidValue, it->second, "fast, accurate or robust required");
    }
    // 全局及逐模型覆盖保持输入顺序，每个键仅最后一次出现生效。
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (last[key] != i || key == "default_parameters") continue;
      if (key == "model_invert_normals") {
        const auto v = values.string(i);
        if (v != "true" && v != "false")
          values.fail(ErrorCode::InvalidValue, i, "true or false string required");
        options.invert_normals = v == "true";
        continue;
      }
      const auto v = values.number(i);
      if (key.starts_with("rel_sampling_distance") || key.starts_with("key_point_fraction")) {
        if (unit_interval(v)) values.fail(ErrorCode::InvalidValue, i, "value must lie in (0,1)");
        const bool sampling = key.starts_with("rel_sampling");
        auto* parameter = sampling ? options.sampling_distance : options.key_point_fraction;
        if (key.ends_with("obj1"))
          parameter[0] = v;
        else if (key.ends_with("obj2"))
          parameter[1] = v;
        else
          parameter[0] = parameter[1] = v;
      } else if (key == "pose_ref_num_steps") {
        if (v < 0 || (std::isfinite(v) && v > 1000000))
          values.fail(ErrorCode::InvalidValue, i, "nonnegative bounded refinement count required");
        options.refinement_steps = std::isfinite(v) ? static_cast<std::size_t>(v) : 0;
      } else if (key == "pose_ref_sub_sampling") {
        if (!std::isfinite(v) || v < 1 || v > 1000000)
          values.fail(ErrorCode::InvalidValue, i, "finite sub-sampling count of at least one required");
        options.sub_sampling = static_cast<std::size_t>(v);
      } else if (key == "pose_ref_dist_threshold_rel") {
        if (unit_interval(v)) values.fail(ErrorCode::InvalidValue, i, "relative threshold must lie in (0,1)");
        options.relative_distance_threshold = v;
      } else if (key == "pose_ref_dist_threshold_abs") {
        if (v <= 0) values.fail(ErrorCode::InvalidValue, i, "positive absolute threshold required");
        options.absolute_distance_threshold = v;
        options.absolute_threshold_set = true;
      }
    }
    auto result =
        mv::reconstruction::register_pair(a->handle()->value, b->handle()->value, algorithm, options);
    outputs[0] = std::move(result.pose);
    outputs[1] = std::move(result.score);
    outputs.Commit();
  });
}
