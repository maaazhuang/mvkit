#include <mvkit/c/mv_reconstruction3d.h>

#include <algorithm>
#include <array>
#include <calibration/hom_mat3d.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/coordinate_selection.hpp>
#include <string>
#include <vector>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "object_model_index.hpp"
#include "tuple_input.hpp"

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
using Model = mv::reconstruction::ObjectModel3DData;

std::shared_ptr<mv::c_api::ObjectModel3DObject> model_at(const TupleInput& models, std::size_t i,
                                                         const char* operation) {
  try {
    return models.at(i).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& error) {
    throw mv::core::Exception(error.status(), error.error_code(), operation, "ObjectModel3D", i,
                              error.message());
  }
}
void append_model(mv::core::Tuple& output, Model model) {
  auto handle = std::make_unique<MvObjectModel3DImpl>();
  handle->value = std::move(model);
  output.append(mv::core::Handle::make<mv::c_api::ObjectModel3DObject>(handle.get()));
  mv::c_api::release_opaque(handle.release());
}
bool present(const Model& model, std::size_t i) { return model.valid.empty() || model.valid[i] != 0; }
std::string text_scalar(const TupleInput& input) {
  if (input.size() != 1) input.fail(ErrorCode::InvalidCardinality, -1, "exactly one string required");
  const auto value = input.string(0);
  if (value.find('\0') != std::string_view::npos)
    input.fail(ErrorCode::InvalidValue, 0, "embedded NUL is not supported");
  return std::string(value);
}
struct IoOptions {
  bool invert_normals{false};
  std::string file_type;
};
IoOptions io_options(const TupleInput& names, const TupleInput& values, bool reading) {
  if (names.size() != values.size())
    names.fail(ErrorCode::InvalidCardinality, -1, "generic names and values must have equal length");
  IoOptions result;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto name = names.string(i);
    if (name == "invert_normals") {
      const auto text = values.string(i);
      if (text != "true" && text != "false")
        values.fail(ErrorCode::InvalidValue, i, "invert_normals must be true or false");
      result.invert_normals = text == "true";
    } else if (reading && name == "file_type") {
      const auto text = values.string(i);
      if (text != "ply" && text != "mvkit")
        values.fail(ErrorCode::InvalidValue, i, "only PLY and MVKit native readers are implemented");
      result.file_type = std::string(text);
    } else
      names.fail(ErrorCode::InvalidValue, i, "generic parameter is not implemented");
  }
  return result;
}
void invert_normals(Model& data) {
  for (auto& normal : data.normals) {
    normal.x = -normal.x;
    normal.y = -normal.y;
    normal.z = -normal.z;
  }
  for (auto& triangle : data.triangles) std::swap(triangle[1], triangle[2]);
}

Model compact(const Model& model, const std::vector<std::size_t>& indices) {
  return mv::reconstruction::compact_object_model_3d(model, indices);
}

double point_attribute(const Model& model, std::size_t i, std::string_view attribute, const TupleInput& input,
                       std::size_t parameter_index,
                       const mv::reconstruction::PointAttributes::Snapshot& extended) {
  if (attribute.starts_with("&")) {
    const auto values = mv::reconstruction::PointAttributes::Find(extended, attribute);
    if (!values) input.fail(ErrorCode::InvalidValue, parameter_index, "extended attribute is missing");
    return (*values)[i];
  }
  if (attribute == "point_coord_x") return model.points[i].x;
  if (attribute == "point_coord_y") return model.points[i].y;
  if (attribute == "point_coord_z") return model.points[i].z;
  if (attribute == "point_normal_x" || attribute == "point_normal_y" || attribute == "point_normal_z") {
    if (model.normals.empty()) input.fail(ErrorCode::InvalidValue, parameter_index, "model has no normals");
    return attribute == "point_normal_x"   ? model.normals[i].x
           : attribute == "point_normal_y" ? model.normals[i].y
                                           : model.normals[i].z;
  }
  if (attribute == "mapping_row" || attribute == "mapping_col") {
    const auto width = model.mapping_width ? model.mapping_width : model.width;
    if (!width) input.fail(ErrorCode::InvalidValue, parameter_index, "model has no XYZ mapping");
    const auto slot = model.source_indices.empty() ? i : model.source_indices[i];
    return static_cast<double>(attribute == "mapping_row" ? slot / width : slot % width);
  }
  if (attribute == "point_score" && !model.confidence.empty()) return model.confidence[i];
  if (attribute == "observed_valid" && !model.observed_valid.empty()) return model.observed_valid[i];
  if (attribute == "filled" && !model.filled.empty()) return model.filled[i];
  if (attribute == "quality_reason" && !model.quality_reason.empty()) return model.quality_reason[i];
  if (attribute == "depth_uncertainty" && !model.depth_uncertainty.empty()) return model.depth_uncertainty[i];
  input.fail(ErrorCode::InvalidValue, parameter_index, "point attribute is missing or unsupported");
}
void validate_attribute(const Model& model, std::string_view attribute, const TupleInput& input,
                        std::size_t parameter_index) {
  if (attribute == "point_coord_x" || attribute == "point_coord_y" || attribute == "point_coord_z") {
    if (model.points.empty()) input.fail(ErrorCode::InvalidValue, parameter_index, "model has no points");
  } else if (attribute == "point_normal_x" || attribute == "point_normal_y" ||
             attribute == "point_normal_z") {
    if (model.normals.empty()) input.fail(ErrorCode::InvalidValue, parameter_index, "model has no normals");
  } else if (attribute == "mapping_row" || attribute == "mapping_col") {
    if (!(model.mapping_width || model.width))
      input.fail(ErrorCode::InvalidValue, parameter_index, "model has no XYZ mapping");
  } else if ((attribute == "point_score" && !model.confidence.empty()) ||
             (attribute == "observed_valid" && !model.observed_valid.empty()) ||
             (attribute == "filled" && !model.filled.empty()) ||
             (attribute == "quality_reason" && !model.quality_reason.empty()) ||
             (attribute == "depth_uncertainty" && !model.depth_uncertainty.empty())) {
    return;
  } else
    input.fail(ErrorCode::InvalidValue, parameter_index, "point attribute is missing or unsupported");
}
}  // namespace

extern "C" ::MvStatus mv_select_points_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Attrib,
                                                     ::MvTuple MinValue, ::MvTuple MaxValue,
                                                     ::MvTuple* ObjectModel3DThresholded) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_select_points_object_model_3d";
    mv::c_api::ControlOutputs<1> output(operation, {ObjectModel3DThresholded});
    const TupleInput models(ObjectModel3D, operation, "ObjectModel3D"),
        attributes(Attrib, operation, "Attrib"), lower(MinValue, operation, "MinValue"),
        upper(MaxValue, operation, "MaxValue");
    if (!models.size()) {
      output.Commit();
      return;
    }
    const bool numeric = attributes.size() && attributes.at(0).kind() != mv::core::ValueKind::String;
    if (lower.size() != (numeric ? 1 : attributes.size()) || upper.size() != lower.size())
      lower.fail(ErrorCode::InvalidCardinality, -1,
                 "threshold lengths must match named attributes or be scalar for numeric attributes");
    // 单组阈值使用栈存储，多组一次分配成对区间；数值读取顺序保持下界、上界交错。
    std::array<std::pair<double, double>, 1> scalar_bounds;
    std::vector<std::pair<double, double>> many_bounds;
    if (lower.size() > 1) many_bounds.resize(lower.size());
    const auto bounds = lower.size() > 1 ? std::span{many_bounds} : std::span{scalar_bounds}.first(lower.size());
    for (std::size_t i = 0; i < lower.size(); ++i) {
      bounds[i] = {lower.number(i), upper.number(i)};
    }
    for (std::size_t i = 0; i < attributes.size(); ++i)
      if (numeric)
        (void)attributes.number(i);
      else
        (void)attributes.string(i);
    for (std::size_t m = 0; m < models.size(); ++m) {
      const auto wrapper = model_at(models, m, operation);
      const auto& data = wrapper->handle()->value;
      const auto extended = data.point_attributes.Read();
      mv::reconstruction::validate_object_model_3d(data);
      std::size_t count = 0;
      // 只有逐点数值属性需要基数检查；命名属性不再额外遍历全部点。
      if (numeric) {
        if (data.valid.empty()) count = data.points.size();
        else for (std::size_t i = 0; i < data.points.size(); ++i) count += present(data, i);
      }
      if (!numeric)
        for (std::size_t a = 0; a < attributes.size(); ++a)
          if (attributes.string(a).starts_with("&")) {
            if (!mv::reconstruction::PointAttributes::Find(extended, attributes.string(a)))
              attributes.fail(ErrorCode::InvalidValue, a, "extended attribute is missing");
          } else
            validate_attribute(data, attributes.string(a), attributes, a);
      if (numeric && attributes.size() != static_cast<std::size_t>(count))
        attributes.fail(ErrorCode::InvalidCardinality, -1,
                        "one numeric attribute per present point required");
      if(!numeric && attributes.size()==1) {
        const auto name=attributes.string(0);
        const std::size_t axis=name=="point_coord_x"?0:name=="point_coord_y"?1:name=="point_coord_z"?2:3;
        if(axis<3) {
          append_model(output[0],mv::reconstruction::compact_coordinate_points(data,axis,bounds[0].first,bounds[0].second,extended));continue;
        }
      }
      std::vector<std::size_t> selected;selected.reserve(data.points.size());
      std::size_t point = 0;
      for (std::size_t i = 0; i < data.points.size(); ++i) {
        if (!present(data, i)) continue;
        bool keep = true;
        for (std::size_t a = 0; a < (numeric ? 1 : attributes.size()); ++a) {
          const double value = numeric
                                   ? attributes.number(point)
                                   : point_attribute(data, i, attributes.string(a), attributes, a, extended);
          keep = keep && value >= bounds[a].first && value <= bounds[a].second;
        }
        ++point;
        if (keep) selected.push_back(i);
      }
      append_model(output[0], mv::reconstruction::compact_object_model_3d(data, selected, extended));
    }
    output.Commit();
  });
}

extern "C" ::MvStatus mv_connection_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Feature,
                                                  ::MvTuple Value, ::MvTuple* ObjectModel3DConnected) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_connection_object_model_3d";
    mv::c_api::ControlOutputs<1> output(operation, {ObjectModel3DConnected});
    const TupleInput models(ObjectModel3D, operation, "ObjectModel3D"),
        features(Feature, operation, "Feature"), values(Value, operation, "Value");
    if (!models.size()) {
      output.Commit();
      return;
    }
    if (values.size() != features.size() && values.size() != 1)
      values.fail(ErrorCode::InvalidCardinality, -1, "one value per feature or a scalar required");
    for (std::size_t m = 0; m < models.size(); ++m) {
      const auto wrapper = model_at(models, m, operation);
      for (std::size_t f = 0; f < features.size(); ++f) {
        const auto feature = features.string(f);
        const double threshold = std::abs(values.number(values.size() == 1 ? 0 : f));
        if (feature != "distance_3d")
          features.fail(ErrorCode::InvalidValue, f, "connection feature is not implemented");
        // 负距离取绝对值；实测 HALCON 数据中，零或 NaN 不分离有限点。
        auto clusters = mv::reconstruction::connection_object_model_3d(
            wrapper->handle()->value,
            !(threshold > 0.) ? std::numeric_limits<double>::max()
                              : std::min(threshold, std::numeric_limits<double>::max()),
            1, mv::c_api::object_model_index(wrapper->handle()).get());
        for (auto& cluster : clusters) append_model(output[0], std::move(cluster));
      }
    }
    output.Commit();
  });
}

extern "C" ::MvStatus mv_intersect_plane_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Plane,
                                                       ::MvTuple* ObjectModel3DIntersection) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_intersect_plane_object_model_3d";
    mv::c_api::ControlOutputs<1> output(operation, {ObjectModel3DIntersection});
    const TupleInput models(ObjectModel3D, operation, "ObjectModel3D"), planes(Plane, operation, "Plane");
    if (!models.size()) {
      output.Commit();
      return;
    }
    if (planes.size() % 7 || !planes.size())
      planes.fail(ErrorCode::InvalidCardinality, -1, "complete plane poses required");
    const auto count = planes.size() / 7;
    if (models.size() != count && models.size() != 1 && count != 1)
      planes.fail(ErrorCode::InvalidCardinality, -1,
                  "model/plane counts must match or one must be singleton");
    for (std::size_t i = 0; i < std::max(models.size(), count); ++i) {
      mv::core::Tuple pose;
      for (std::size_t j = 0; j < 7; ++j) pose.append(planes.at((count == 1 ? 0 : i) * 7 + j));
      const auto matrix =
          mv::calibration::parse_hom_mat3d(mv::calibration::pose_to_hom_mat3d(pose), operation);
      const auto wrapper = model_at(models, models.size() == 1 ? 0 : i, operation);
      append_model(output[0],
                   mv::reconstruction::intersect_plane_object_model_3d(
                       wrapper->handle()->value, matrix[2], matrix[6], matrix[10],
                       -(matrix[2] * matrix[3] + matrix[6] * matrix[7] + matrix[10] * matrix[11])));
    }
    output.Commit();
  });
}

extern "C" ::MvStatus mv_read_object_model_3d(::MvTuple FileName, ::MvTuple Scale,
                                            ::MvTuple GenParamName, ::MvTuple GenParamValue,
                                            ::MvTuple* ObjectModel3D, ::MvTuple* Status) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_read_object_model_3d";
    mv::c_api::ControlOutputs<2> output(operation, {ObjectModel3D, Status});
    const TupleInput filename(FileName, operation, "FileName"), scale(Scale, operation, "Scale"),
        names(GenParamName, operation, "GenParamName"), values(GenParamValue, operation, "GenParamValue");
    const auto path = text_scalar(filename);
    if (scale.size() != 1) scale.fail(ErrorCode::InvalidCardinality, -1, "one scale required");
    double factor = 1.;
    if (scale.at(0).kind() == mv::core::ValueKind::String) {
      const auto unit = scale.string(0);
      if (unit == "m")
        factor = 1.;
      else if (unit == "cm")
        factor = .01;
      else if (unit == "mm")
        factor = .001;
      else if (unit == "um")
        factor = .000001;
      else
        scale.fail(ErrorCode::InvalidValue, 0, "unsupported scale unit");
    } else
      factor = scale.number(0);
    if (!std::isfinite(factor)) scale.fail(ErrorCode::InvalidValue, 0, "finite scale required");
    const auto options = io_options(names, values, true);
    if (!options.file_type.empty()) {
      std::ifstream stream(std::filesystem::u8path(path));
      std::string magic;
      std::getline(stream, magic);
      if (stream && ((options.file_type == "ply") != (magic == "ply" || magic == "ply\r")))
        filename.fail(ErrorCode::InvalidValue, 0, "file does not have requested format");
    }
    auto data = mv::reconstruction::read_object_model_3d(path, 1.);
    // HALCON 将 PLY 面视为 polygon，三顶点面也如此。
    std::ifstream source(std::filesystem::u8path(path));
    std::string magic;
    std::getline(source, magic);
    if (magic == "ply" || magic == "ply\r") data.triangles_are_polygons = true;
    if (factor != 1.) {
      for (auto& point : data.points) {
        point.x *= factor;
        point.y *= factor;
        point.z *= factor;
      }
      data.plane[3] *= factor;
      data.primitive_rms *= std::abs(factor);
      if (data.primitive_kind == mv::reconstruction::PrimitiveKind::Sphere ||
          data.primitive_kind == mv::reconstruction::PrimitiveKind::Cylinder) {
        for (std::size_t i = 0; i < 3; ++i) data.primitive[i] *= factor;
        data.primitive[data.primitive_kind == mv::reconstruction::PrimitiveKind::Sphere ? 3 : 6] *=
            std::abs(factor);
      }
    }
    if (options.invert_normals) invert_normals(data);
    append_model(output[0], std::move(data));
    output[1].append("");
    output.Commit();
  });
}

extern "C" ::MvStatus mv_write_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple FileType,
                                             ::MvTuple FileName, ::MvTuple GenParamName,
                                             ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_write_object_model_3d";
    const TupleInput models(ObjectModel3D, operation, "ObjectModel3D"), type(FileType, operation, "FileType"),
        filename(FileName, operation, "FileName"), names(GenParamName, operation, "GenParamName"),
        values(GenParamValue, operation, "GenParamValue");
    if (models.size() != 1) models.fail(ErrorCode::InvalidCardinality, -1, "exactly one model required");
    const auto format = text_scalar(type), path = text_scalar(filename);
    const auto options = io_options(names, values, false);
    const auto wrapper = model_at(models, 0, operation);
    if (options.invert_normals) {
      auto data = wrapper->handle()->value;
      invert_normals(data);
      mv::reconstruction::write_object_model_3d(path, format, data);
    } else
      mv::reconstruction::write_object_model_3d(path, format, wrapper->handle()->value);
  });
}
