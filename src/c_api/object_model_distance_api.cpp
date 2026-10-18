#include <mvkit/c/mv_reconstruction3d.h>

#include <calibration/hom_mat3d.hpp>
#include <cmath>
#include <reconstruction/object_distance.hpp>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "object_model_index.hpp"
#include "tuple_input.hpp"
namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
auto model_at(const TupleInput& input, std::size_t i, const char* op, const char* parameter) {
  try {
    return input.at(i).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& e) {
    throw mv::core::Exception(e.status(), e.error_code(), op, parameter, i, e.message());
  }
}
bool boolean(const TupleInput& values, std::size_t i) {
  const auto v = values.string(i);
  if (v != "true" && v != "false") values.fail(ErrorCode::InvalidValue, i, "boolean string required");
  return v == "true";
}
}  // namespace
extern "C" ::MvStatus mv_distance_object_model_3d(::MvTuple ObjectModel3DFrom, ::MvTuple ObjectModel3DTo,
                                                ::MvTuple Pose, ::MvTuple MaxDistance,
                                                ::MvTuple GenParamName, ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_distance_object_model_3d";
    const TupleInput from(ObjectModel3DFrom, op, "ObjectModel3DFrom"),
        to(ObjectModel3DTo, op, "ObjectModel3DTo"), pose(Pose, op, "Pose"),
        maximum(MaxDistance, op, "MaxDistance"), names(GenParamName, op, "GenParamName"),
        values(GenParamValue, op, "GenParamValue");
    if (from.size() != 1) from.fail(ErrorCode::InvalidCardinality, -1, "exactly one source model required");
    if (to.size() != 1) to.fail(ErrorCode::InvalidCardinality, -1, "exactly one target model required");
    const auto source = model_at(from, 0, op, "ObjectModel3DFrom"),
               target = model_at(to, 0, op, "ObjectModel3DTo");
    if (names.size() != values.size())
      values.fail(ErrorCode::InvalidCardinality, -1, "one value per generic parameter required");
    mv::reconstruction::ObjectDistanceOptions options;
    if (maximum.size() != 1)
      maximum.fail(ErrorCode::InvalidCardinality, -1, "exactly one maximum value required");
    options.maximum = maximum.number(0);
    std::string output_attribute = "&distance";
    bool invert = false;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto name = names.string(i);
      if (name == "distance_to") {
        options.distance_to = values.string(i);
        if (options.distance_to != "auto" && options.distance_to != "points" &&
            options.distance_to != "triangles" && options.distance_to != "primitive")
          values.fail(ErrorCode::InvalidValue, i, "unknown target type");
      } else if (name == "method") {
        options.method = values.string(i);
        if (options.method != "auto" && options.method != "linear" && options.method != "kd-tree" &&
            options.method != "voxel")
          values.fail(ErrorCode::InvalidValue, i, "unknown distance method");
      } else if (name == "signed_distances")
        options.signed_distances = boolean(values, i);
      else if (name == "store_closest_index")
        options.store_closest_index = boolean(values, i);
      else if (name == "invert_pose")
        invert = boolean(values, i);
      else if (name == "output_attribute") {
        const auto attribute = values.string(i);
        if (attribute.size() < 2 || attribute[0] != '&' || attribute.find('\0') != std::string_view::npos)
          values.fail(ErrorCode::InvalidValue, i, "extended attribute name starting with & required");
        output_attribute = attribute;
      } else if (name == "sampling_dist_abs" || name == "sampling_dist_rel") {
        const auto distance = values.number(i);
        if (!std::isfinite(distance) || distance <= 0)
          values.fail(ErrorCode::InvalidValue, i, "positive finite sampling distance required");
      } else
        names.fail(ErrorCode::InvalidValue, i, "unknown distance parameter");
    }
    if (pose.size()) {
      auto matrix = mv::calibration::pose_to_hom_mat3d(pose.materialize());
      if (invert) matrix = mv::calibration::hom_mat3d_invert(matrix);
      options.pose = mv::calibration::parse_hom_mat3d(matrix, op);
    }
    const auto& geometry = target->handle()->value;
    std::shared_ptr<const mv::reconstruction::NeighborhoodIndex> index;
    const bool point_target =
        options.distance_to == "points" ||
        (options.distance_to == "auto" &&
         geometry.primitive_kind == mv::reconstruction::PrimitiveKind::None && geometry.triangles.empty());
    if (point_target &&
        (options.method == "kd-tree" || (options.method == "auto" && geometry.points.size() >= 100)))
      index = mv::c_api::object_model_index(target->handle());
    const auto result = [&] {
      try {
        return mv::reconstruction::object_distances(source->handle()->value, geometry, options, index.get());
      } catch (const mv::core::Exception& e) {
        throw mv::core::Exception(e.status(), e.error_code(), op, e.parameter(), e.parameter_index(),
                                  e.message());
      }
    }();
    const mv::reconstruction::PointAttributes::Update updates[]{{output_attribute, result.distances},
                                                                {"&closest_index", result.closest}};
    source->handle()->value.point_attributes.Replace(
        {updates, options.store_closest_index ? std::size_t{2} : std::size_t{1}});
  });
}
