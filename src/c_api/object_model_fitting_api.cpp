#include <mvkit/c/mv_reconstruction3d.h>

#include <memory>
#include <reconstruction/primitive_fitting.hpp>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
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
void append_model(mv::core::Tuple& output, mv::reconstruction::ObjectModel3DData value) {
  auto handle = std::make_unique<MvObjectModel3DImpl>();
  handle->value = std::move(value);
  output.append(mv::core::Handle::make<mv::c_api::ObjectModel3DObject>(handle.get()));
  mv::c_api::release_opaque(handle.release());
}
void lengths(const TupleInput& names, const TupleInput& values) {
  if (names.size() != values.size())
    values.fail(ErrorCode::InvalidCardinality, -1, "one value per generic parameter required");
}
bool boolean(const TupleInput& values, std::size_t i) {
  const auto text = values.string(i);
  if (text != "true" && text != "false")
    values.fail(ErrorCode::InvalidValue, i, "true or false string required");
  return text == "true";
}
}  // namespace
extern "C" ::MvStatus mv_fit_primitives_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple GenParamName,
                                                      ::MvTuple GenParamValue,
                                                      ::MvTuple* ObjectModel3DOut) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_fit_primitives_object_model_3d";
    mv::c_api::ControlOutputs<1> output(op, {ObjectModel3DOut});
    const TupleInput models(ObjectModel3D, op, "ObjectModel3D"), names(GenParamName, op, "GenParamName"),
        values(GenParamValue, op, "GenParamValue");
    if (!models.size()) {
      output.Commit();
      return;
    }
    lengths(names, values);
    mv::reconstruction::PrimitiveFitOptions options;
    bool has_type = false, copy_points = true, copy_mapping = false;
    std::int64_t radius_index = -1;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto name = names.string(i);
      if (name == "primitive_type") {
        const auto type = values.string(i);
        const auto bit = type == "plane"      ? 1u
                         : type == "sphere"   ? 2u
                         : type == "cylinder" ? 4u
                         : type == "all"      ? 7u
                                              : 0u;
        if (!bit) values.fail(ErrorCode::InvalidValue, i, "unknown primitive type");
        if (!has_type) {
          options.types = 0;
          has_type = true;
        }
        options.types |= bit;
      } else if (name == "fitting_algorithm") {
        const auto algorithm = values.string(i);
        if (algorithm == "least_squares")
          options.loss = mv::reconstruction::PrimitiveFitLoss::LeastSquares;
        else if (algorithm == "least_squares_huber")
          options.loss = mv::reconstruction::PrimitiveFitLoss::Huber;
        else if (algorithm == "least_squares_tukey")
          options.loss = mv::reconstruction::PrimitiveFitLoss::Tukey;
        else
          values.fail(ErrorCode::InvalidValue, i, "unknown fitting algorithm");
      } else if (name == "min_radius") {
        options.minimum_radius = values.number(i);
        radius_index = i;
      } else if (name == "max_radius") {
        options.maximum_radius = values.number(i);
        radius_index = i;
      } else if (name == "output_point_coord")
        copy_points = boolean(values, i);
      else if (name == "output_xyz_mapping")
        copy_mapping = boolean(values, i);
      else
        names.fail(ErrorCode::InvalidValue, i, "unknown fitting parameter");
    }
    if ((options.types & 6) &&
        (options.maximum_radius <= 0 || options.minimum_radius > options.maximum_radius))
      values.fail(ErrorCode::InvalidValue, radius_index, "invalid curved primitive radius range");
    for (std::size_t i = 0; i < models.size(); ++i)
      append_model(output[0],
                   mv::reconstruction::fit_primitive_model(model_at(models, i, op)->handle()->value, options,
                                                           copy_points, copy_mapping));
    output.Commit();
  });
}
