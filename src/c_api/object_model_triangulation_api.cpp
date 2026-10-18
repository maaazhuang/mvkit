#include <mvkit/c/mv_reconstruction3d.h>

#include <cmath>
#include <memory>
#include <reconstruction/mapped_triangulation.hpp>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "tuple_input.hpp"

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
constexpr const char* op = "mv_triangulate_object_model_3d";
std::shared_ptr<mv::c_api::ObjectModel3DObject> model_at(const TupleInput& models, std::size_t i) {
  try {
    return models.at(i).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& error) {
    throw mv::core::Exception(error.status(), error.error_code(), op, "ObjectModel3D", i, error.message());
  }
}
void append_model(mv::core::Tuple& output, mv::reconstruction::ObjectModel3DData model) {
  auto handle = std::make_unique<MvObjectModel3DImpl>();
  handle->value = std::move(model);
  output.append(mv::core::Handle::make<mv::c_api::ObjectModel3DObject>(handle.get()));
  mv::c_api::release_opaque(handle.release());
}
}  // namespace

extern "C" ::MvStatus mv_triangulate_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Method,
                                                   ::MvTuple GenParamName, ::MvTuple GenParamValue,
                                                   ::MvTuple* TriangulatedObjectModel3D,
                                                   ::MvTuple* Information) {
  return mv::c_api::translate([&] {
    mv::c_api::ControlOutputs<2> outputs(op, {TriangulatedObjectModel3D, Information});
    const TupleInput models(ObjectModel3D, op, "ObjectModel3D"), method(Method, op, "Method"),
        names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    if (!models.size()) {
      outputs.Commit();
      return;
    }
    if (method.size() != 1) method.fail(ErrorCode::InvalidCardinality, -1, "one method string required");
    const auto algorithm = method.string(0);
    if (algorithm != "xyz_mapping" && algorithm != "greedy" && algorithm != "implicit" &&
        algorithm != "polygon_triangulation")
      method.fail(ErrorCode::InvalidValue, 0, "unknown triangulation method");
    if (names.size() != values.size())
      values.fail(ErrorCode::InvalidCardinality, -1, "one generic value per name required");
    if (algorithm == "greedy" || algorithm == "implicit")
      throw mv::core::Exception(mv::core::Status::Unsupported, ErrorCode::UnsupportedValue, op, "Method", 0,
                                "greedy and implicit triangulation are not implemented");
    mv::reconstruction::MappedTriangulationOptions options;
    unsigned direction_mask = 0;
    std::int64_t direction_index = -1;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (algorithm == "polygon_triangulation")
        names.fail(ErrorCode::InvalidValue, i, "polygon triangulation has no generic parameters");
      // 每次出现都验证；HALCON xyz_mapping 不忽略此前重复项的非法值。
      if (key == "xyz_mapping_max_area_holes") {
        const auto value = values.number(i);
        if (std::isfinite(value) && value < 0)
          values.fail(ErrorCode::InvalidValue, i, "nonnegative hole area required");
        // 实测 HALCON 在面积非有限时禁用补洞。
        options.maximum_hole_area = std::isfinite(value) ? value : 0;
      } else if (key == "xyz_mapping_max_view_angle") {
        const auto value = values.number(i);
        if (value < 0 || value > 1.57079632679489661923)
          values.fail(ErrorCode::InvalidValue, i, "view angle in [0,pi/2] required");
        options.maximum_view_angle = value;
        options.filter_view_angle = true;
      } else if (key == "xyz_mapping_output_all_points") {
        const auto value = values.string(i);
        if (value != "true" && value != "false")
          values.fail(ErrorCode::InvalidValue, i, "true or false string required");
        options.output_all_points = value == "true";
      } else if (key == "xyz_mapping_max_view_dir_x" || key == "xyz_mapping_max_view_dir_y" ||
                 key == "xyz_mapping_max_view_dir_z") {
        const auto value = values.number(i);
        if (direction_index < 0) direction_index = static_cast<std::int64_t>(i);
        if (key.ends_with("_x")) {
          options.view_direction.x = value;
          direction_mask |= 1;
        }
        if (key.ends_with("_y")) {
          options.view_direction.y = value;
          direction_mask |= 2;
        }
        if (key.ends_with("_z")) {
          options.view_direction.z = value;
          direction_mask |= 4;
        }
      } else {
        names.fail(ErrorCode::InvalidValue, i, "unknown xyz_mapping parameter");
      }
    }
    if (direction_mask && (direction_mask != 7 || !options.filter_view_angle))
      names.fail(ErrorCode::InvalidValue, direction_index,
                 "all three view direction components and an explicit view angle required");
    if (direction_mask) {
      const auto v = options.view_direction;
      const auto norm2 = v.x * v.x + v.y * v.y + v.z * v.z;
      if (!(norm2 > 0) || !std::isfinite(norm2))
        values.fail(ErrorCode::InvalidValue, direction_index, "finite nonzero view direction required");
    }
    for (std::size_t i = 0; i < models.size(); ++i) {
      const auto model = model_at(models, i);
      mv::reconstruction::ObjectModel3DData mesh;
      try {
        if (algorithm == "xyz_mapping") {
          mesh = mv::reconstruction::triangulate_mapping(model->handle()->value, options);
        } else {
          const auto& input = model->handle()->value;
          if (!input.triangles_are_polygons || input.triangles.empty())
            throw mv::core::Exception(mv::core::Status::InvalidState, ErrorCode::InvalidObjectState, op,
                                      "ObjectModel3D", i, "polygon data required");
          // 当前存储仅支持三角 PLY polygon，不代表支持一般多边形。
          mesh = input;
          mesh.triangles_are_polygons = false;
        }
      } catch (const mv::core::Exception& error) {
        throw mv::core::Exception(error.status(), error.error_code(), op, "ObjectModel3D", i,
                                  error.message());
      }
      outputs[1].append(static_cast<std::int64_t>(mesh.triangles.size()));
      append_model(outputs[0], std::move(mesh));
    }
    outputs.Commit();
  });
}
