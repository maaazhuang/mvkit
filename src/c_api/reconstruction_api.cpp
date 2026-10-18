#include <mvkit/c/mv_reconstruction3d.h>
#include <mvkit/c/mv_stereo.h>
#include <mvkit/c/mv_structured_light.h>

#include <algorithm>
#include <atomic>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <cmath>
#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <reconstruction/neighborhood_index.hpp>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/primitive_fitting.hpp>
#include <reconstruction/stereo/disparity.hpp>
#include <reconstruction/stereo/reconstruct.hpp>
#include <reconstruction/stereo/rectification.hpp>
#include <reconstruction/stereo/setup_calibration.hpp>
#include <reconstruction/structured_light/calibration.hpp>
#include <reconstruction/structured_light/decode.hpp>
#include <reconstruction/structured_light/io.hpp>
#include <reconstruction/structured_light/pattern.hpp>
#include <reconstruction/structured_light/reconstruct.hpp>
#include <shared_mutex>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "object_model_index.hpp"
#include "opaque.hpp"
#include "structured_error.hpp"
#include "tuple_input.hpp"

namespace {

using mv::c_api::Exception;
using mv::core::Status;

std::string_view sv(::MvStringView value, const char* operation) {
  if (value.data == nullptr && value.size != 0U) {
    throw Exception(Status::InvalidArgument, operation, "invalid string view");
  }
  return {value.data != nullptr ? value.data : "", value.size};
}

void require_output(const void* output, const char* operation) {
  if (!output) {
    throw Exception(Status::InvalidArgument, operation, "output pointer must not be null");
  }
}

const mv::core::Tuple& tuple(::MvTuple handle, const char* operation) {
  if (!handle) {
    throw Exception(Status::InvalidArgument, operation, "tuple handle must not be null");
  }
  return handle->value;
}

const mv::core::Image& image(::MvObject handle, const char* operation) {
  return mv::c_api::require_single_image(handle, operation);
}

const mv::core::IconicTuple& image_objects(::MvObject handle, const char* operation) {
  if (!handle) {
    throw Exception(Status::InvalidArgument, operation, "image handle must not be null");
  }
  handle->value.require_kind(mv::core::IconicKind::Image, operation);
  return handle->value;
}

template <class Wrapper>
std::shared_ptr<Wrapper> tuple_control_wrapper(::MvTuple handles, std::size_t index, const char* operation,
                                               const char* parameter) {
  if (!handles) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, operation, parameter, -1,
                    "handle Tuple must not be null");
  }
  try {
    return handles->value.as_handle(index).as<Wrapper>();
  } catch (const mv::core::Exception& error) {
    throw Exception(error.status(), error.error_code(), operation, parameter,
                    static_cast<std::int64_t>(index), error.message());
  }
}

mv::reconstruction::ObjectModel3DData& object_model(::MvObjectModel3D handle, const char* operation) {
  if (!handle) {
    throw Exception(Status::InvalidArgument, operation, "object model handle must not be null");
  }
  return handle->value;
}

const mv::reconstruction::ObjectModel3DData& tuple_object_model(::MvTuple handles, std::size_t index,
                                                                const char* operation) {
  if (!handles) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, operation, "ObjectModel3D",
                    -1, "object model handle Tuple must not be null");
  }
  const auto wrapper =
      tuple_control_wrapper<mv::c_api::ObjectModel3DObject>(handles, index, operation, "ObjectModel3D");
  return object_model(wrapper->handle(), operation);
}

mv::reconstruction::StructuredLightModel& structured_model(::MvStructuredLightModel handle,
                                                           const char* operation);

MvStructuredLightModelImpl* tuple_structured_handle(::MvTuple handles, std::size_t index,
                                                    const char* operation) {
  if (!handles) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, operation,
                    "StructuredLightModel", -1, "structured-light model handle Tuple must not be null");
  }
  const auto wrapper = tuple_control_wrapper<mv::c_api::StructuredLightModelObject>(handles, index, operation,
                                                                                    "StructuredLightModel");
  auto* handle = wrapper->handle();
  (void)structured_model(handle, operation);
  return handle;
}

const mv::reconstruction::StructuredLightModel& tuple_structured_model(::MvTuple handles, std::size_t index,
                                                                       const char* operation) {
  return *tuple_structured_handle(handles, index, operation)->value;
}

mv::reconstruction::CameraSetupModel& camera_setup(::MvCameraSetupModel handle, const char* operation) {
  if (!handle || !handle->value) {
    throw Exception(Status::InvalidArgument, operation, "camera setup handle must not be null");
  }
  return *handle->value;
}

MvCameraSetupModelImpl* tuple_camera_setup(::MvTuple handles, std::size_t index, const char* operation) {
  if (!handles) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, operation, "CameraSetupModel",
                    -1, "camera setup handle Tuple must not be null");
  }
  const auto wrapper =
      tuple_control_wrapper<mv::c_api::CameraSetupModelObject>(handles, index, operation, "CameraSetupModel");
  auto* handle = wrapper->handle();
  (void)camera_setup(handle, operation);
  return handle;
}

mv::reconstruction::StructuredLightModel& structured_model(::MvStructuredLightModel handle,
                                                           const char* operation) {
  if (!handle || !handle->value) {
    throw Exception(Status::InvalidArgument, operation, "structured-light model must not be null");
  }
  return *handle->value;
}

mv::calibration::CalibrationDataModel& calibration_model(::MvCalibData handle, const char* operation) {
  if (!handle || !handle->value) {
    throw Exception(Status::InvalidArgument, operation, "calibration data must not be null");
  }
  return *handle->value;
}

const mv::calibration::CalibrationDataModel& tuple_calibration_model(::MvTuple handles, std::size_t index,
                                                                     const char* operation,
                                                                     const char* parameter) {
  if (!handles) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, operation, parameter, -1,
                    "calibration data handle Tuple must not be null");
  }
  const auto wrapper =
      tuple_control_wrapper<mv::c_api::CalibrationDataObject>(handles, index, operation, parameter);
  return calibration_model(wrapper->handle(), operation);
}

std::unique_ptr<MvTupleImpl> make_tuple(mv::core::Tuple value) {
  auto handle = std::make_unique<MvTupleImpl>();
  handle->value = std::move(value);
  return handle;
}

std::unique_ptr<MvObjectImpl> make_image(mv::core::Image value) {
  return mv::c_api::make_image_handle(std::move(value));
}

void append_object_model(mv::core::Tuple* result, mv::reconstruction::ObjectModel3DData value) {
  auto* handle = new MvObjectModel3DImpl();
  handle->value = std::move(value);
  result->append(mv::core::Handle::make<mv::c_api::ObjectModel3DObject>(handle));
  mv::c_api::release_opaque(handle);
}

void append_camera_setup(mv::core::Tuple* result, mv::reconstruction::CameraSetupModel value) {
  auto handle = std::make_unique<MvCameraSetupModelImpl>();
  handle->value = std::make_unique<mv::reconstruction::CameraSetupModel>(std::move(value));
  result->append(mv::core::Handle::make<mv::c_api::CameraSetupModelObject>(handle.get()));
  mv::c_api::release_opaque(handle.release());
}

void append_structured_model(mv::core::Tuple* result, mv::reconstruction::StructuredLightModel value) {
  auto* handle = new MvStructuredLightModelImpl();
  handle->value = std::make_unique<mv::reconstruction::StructuredLightModel>(std::move(value));
  result->append(mv::core::Handle::make<mv::c_api::StructuredLightModelObject>(handle));
  mv::c_api::release_opaque(handle);
}

std::unique_ptr<MvTupleImpl> make_object_model_tuple(mv::reconstruction::ObjectModel3DData value) {
  auto result = std::make_unique<MvTupleImpl>();
  append_object_model(&result->value, std::move(value));
  return result;
}

void require_empty_gen_params(::MvTuple names, ::MvTuple values, const char* operation) {
  const auto& n = tuple(names, operation);
  const auto& v = tuple(values, operation);
  if (n.size() != v.size()) {
    throw Exception(Status::InvalidArgument, operation, "generic parameter lengths must match");
  }
  if (!n.empty()) {
    throw Exception(Status::InvalidArgument, operation, "this operation has no generic parameters in v1");
  }
}

}  // namespace

namespace {
std::shared_ptr<mv::c_api::ObjectModel3DObject> model_control(const mv::c_api::TupleInput& models,
                                                              std::size_t index, const char* operation) {
  try {
    return models.at(index).as_handle().as<mv::c_api::ObjectModel3DObject>();
  } catch (const mv::core::Exception& error) {
    throw Exception(error.status(), error.error_code(), operation, "ObjectModel3D",
                    static_cast<std::int64_t>(index), error.message());
  }
}
bool model_point_present(const mv::reconstruction::ObjectModel3DData& model, std::size_t i) {
  // HALCON 排除 XYZ 映射无效槽，但非组织模型的显式坐标（含 NaN/Inf）仍可寻址。
  return model.valid.empty() || model.valid[i] != 0;
}
void transform_model_controls(::MvTuple input, ::MvTuple transformation, ::MvTuple* destination,
                              bool rigid) {
  const char* operation = rigid ? "mv_rigid_trans_object_model_3d" : "mv_affine_trans_object_model_3d";
  mv::c_api::ControlOutputs<1> outputs(operation, {destination});
  const mv::c_api::TupleInput models(input, operation, "ObjectModel3D");
  const mv::c_api::TupleInput transforms(transformation, operation, rigid ? "Pose" : "HomMat3D");
  const std::size_t width = rigid ? 7 : 12;
  if (transforms.size() % width)
    transforms.fail(mv::core::ErrorCode::InvalidCardinality, -1, "complete transformations required");
  const auto count = transforms.size() / width;
  if (models.size() != count && models.size() != 1 && count != 1)
    transforms.fail(mv::core::ErrorCode::InvalidCardinality, -1,
                    "model and transformation counts must agree or one must be singleton");
  // 空模型数组不产生输出时，HALCON 仍检查系数类型。
  for (std::size_t i = 0; i < transforms.size(); ++i) transforms.number(i);
  if (!models.size() || !count) {
    outputs.Commit();
    return;
  }
  std::vector<mv::core::Tuple> packed;
  // 单个拥有型变换在同步调用内只读借用；外部借用存储及批量变换仍完整物化。
  // 上方已按原顺序验证全部类型，内核继续执行有限值、位姿代码及模型校验。
  const auto* singleton = count == 1 ? transforms.owned_tuple() : nullptr;
  if (!singleton) {
    packed.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      mv::core::Tuple values;
      for (std::size_t j = 0; j < width; ++j) values.append(transforms.at(i * width + j));
      packed.push_back(std::move(values));
    }
  }
  for (std::size_t i = 0; i < std::max(models.size(), count); ++i) {
    const auto model = model_control(models, models.size() == 1 ? 0 : i, operation);
    const auto& data = object_model(model->handle(), operation);
    const auto& values = singleton ? *singleton : packed[count == 1 ? 0 : i];
    append_object_model(&outputs[0], rigid ? mv::reconstruction::rigid_trans_object_model_3d(data, values)
                                           : mv::reconstruction::affine_trans_object_model_3d(data, values));
  }
  outputs.Commit();
}
}  // namespace

extern "C" ::MvStatus mv_gen_object_model_3d_from_points(::MvTuple X, ::MvTuple Y, ::MvTuple Z,
                                                       ::MvTuple* ObjectModel3D) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_gen_object_model_3d_from_points";
    mv::c_api::ControlOutputs<1> outputs(operation, {ObjectModel3D});
    const mv::c_api::TupleInput x(X, operation, "X"), y(Y, operation, "Y"), z(Z, operation, "Z");
    if (x.size() != y.size() || y.size() != z.size())
      x.fail(mv::core::ErrorCode::InvalidCardinality, -1, "X, Y and Z must have equal length");
    mv::reconstruction::ObjectModel3DData data;
    data.points.resize(x.size());
    const auto* tx=x.owned_tuple();const auto* ty=y.owned_tuple();const auto* tz=z.owned_tuple();
    if(tx && ty && tz && tx->kind()==mv::core::TupleKind::Real64 &&
        ty->kind()==mv::core::TupleKind::Real64 && tz->kind()==mv::core::TupleKind::Real64) {
      const auto xx=tx->real64_span(),yy=ty->real64_span(),zz=tz->real64_span();
      for(std::size_t i=0;i<xx.size();++i)data.points[i]={xx[i],yy[i],zz[i]};
    } else for (std::size_t i = 0; i < x.size(); ++i) data.points[i] = {x.number(i), y.number(i), z.number(i)};
    append_object_model(&outputs[0], std::move(data));
    outputs.Commit();
  });
}

extern "C" ::MvStatus mv_rigid_trans_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple Pose,
                                                   ::MvTuple* ObjectModel3DRigidTrans) {
  return mv::c_api::translate(
      [&] { transform_model_controls(ObjectModel3D, Pose, ObjectModel3DRigidTrans, true); });
}
extern "C" ::MvStatus mv_affine_trans_object_model_3d(::MvTuple ObjectModel3D, ::MvTuple HomMat3D,
                                                    ::MvTuple* ObjectModel3DAffineTrans) {
  return mv::c_api::translate(
      [&] { transform_model_controls(ObjectModel3D, HomMat3D, ObjectModel3DAffineTrans, false); });
}

namespace {
mv::reconstruction::XyzImages stored_xyz_images(const mv::reconstruction::ObjectModel3DData& model) {
  constexpr auto op = "mv_object_model_3d_to_xyz";
  const auto width = model.mapping_width ? model.mapping_width : model.width;
  const auto height = model.mapping_height ? model.mapping_height : model.height;
  mv::reconstruction::validate_object_model_3d(model);
  if (!width || !height) throw Exception(Status::InvalidState, op, "model has no stored XYZ mapping");
  mv::reconstruction::XyzImages images;
  if (model.width == width && model.height == height) {
    images = mv::reconstruction::object_model_3d_to_xyz(model);
  } else {
    images = {mv::core::Image::create_uninitialized(width, height, std::array{mv::core::PixelType::Float32}),
              mv::core::Image::create_uninitialized(width, height, std::array{mv::core::PixelType::Float32}),
              mv::core::Image::create_uninitialized(width, height, std::array{mv::core::PixelType::Float32})};
    for (auto* image : {&images.x, &images.y, &images.z})
      for (std::size_t row = 0; row < height; ++row)
        std::fill_n(
            reinterpret_cast<float*>(static_cast<std::byte*>(image->mutable_data()) + row * image->stride()),
            width, std::numeric_limits<float>::quiet_NaN());
  }
  std::vector<mv::core::RegionRun> runs;
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    const auto& point = model.points[i];
    if ((!model.valid.empty() && !model.valid[i]) || !std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      continue;
    const auto slot = model.source_indices.empty() ? i : model.source_indices[i];
    if (slot / width >= height)
      throw Exception(Status::InvalidState, op, "stored mapping index outside image");
    const auto row = slot / width, column = slot % width;
    if (!runs.empty() && runs.back().row == static_cast<std::int64_t>(row) &&
        std::int64_t(runs.back().col_end) + 1 == static_cast<std::int64_t>(column))
      runs.back().col_end = static_cast<std::int64_t>(column);
    else
      runs.push_back({static_cast<std::int64_t>(row), static_cast<std::int64_t>(column),
                      static_cast<std::int64_t>(column)});
    if (model.width != width || model.height != height) {
      const float coords[]{static_cast<float>(point.x), static_cast<float>(point.y),
                           static_cast<float>(point.z)};
      std::size_t axis = 0;
      for (auto* image : {&images.x, &images.y, &images.z})
        reinterpret_cast<float*>(static_cast<std::byte*>(image->mutable_data()) +
                                 row * image->stride())[column] = coords[axis++];
    }
  }
  const auto domain = mv::core::Region::from_runs(runs);
  images.x = images.x.reduce_domain(domain);
  images.y = images.y.reduce_domain(domain);
  images.z = images.z.reduce_domain(domain);
  return images;
}
void export_stored_xyz(::MvObject* x, ::MvObject* y, ::MvObject* z, ::MvTuple input, bool batch) {
  constexpr auto op = "mv_object_model_3d_to_xyz";
  require_output(x, op);
  require_output(y, op);
  require_output(z, op);
  if (x == y || x == z || y == z)
    throw Exception(Status::InvalidArgument, op, "output pointers must be distinct");
  const mv::c_api::TupleInput models(input, op, "ObjectModel3D");
  if (!batch && models.size() != 1)
    models.fail(mv::core::ErrorCode::InvalidCardinality, -1, "from_xyz_map requires exactly one model");
  mv::core::IconicTuple xs, ys, zs;
  for (std::size_t i = 0; i < models.size(); ++i) {
    const auto control = model_control(models, i, op);
    auto images = stored_xyz_images(object_model(control->handle(), op));
    xs = xs.concat(mv::core::IconicTuple{mv::core::IconicValue::from_image(std::move(images.x))});
    ys = ys.concat(mv::core::IconicTuple{mv::core::IconicValue::from_image(std::move(images.y))});
    zs = zs.concat(mv::core::IconicTuple{mv::core::IconicValue::from_image(std::move(images.z))});
  }
  auto xh = mv::c_api::make_image_handle(std::move(xs), op);
  auto yh = mv::c_api::make_image_handle(std::move(ys), op);
  auto zh = mv::c_api::make_image_handle(std::move(zs), op);
  *x = xh.release();
  *y = yh.release();
  *z = zh.release();
}
}  // namespace
extern "C" ::MvStatus mv_object_model_3d_to_xyz(::MvObject* X, ::MvObject* Y, ::MvObject* Z,
                                              ::MvTuple ObjectModel3D, ::MvTuple Type,
                                              ::MvTuple CamParam, ::MvTuple Pose) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_object_model_3d_to_xyz";
    const mv::c_api::TupleInput type(Type, op, "Type");
    if (type.size() != 1) type.fail(mv::core::ErrorCode::InvalidCardinality, -1, "one Type string required");
    const auto mode = type.string(0);
    if (mode == "cartesian" || mode == "cartesian_faces" || mode == "cartesian_faces_no_opengl")
      throw Exception(Status::Unsupported, op, "cartesian point/face rendering is not implemented");
    if (mode != "from_xyz_map")
      type.fail(mv::core::ErrorCode::InvalidValue, 0, "unknown XYZ conversion Type");
    // HALCON 在 from_xyz_map 模式忽略这些控制的载荷。
    const mv::c_api::TupleInput camera(CamParam, op, "CamParam"), pose(Pose, op, "Pose");
    export_stored_xyz(X, Y, Z, ObjectModel3D, false);
  });
}

static ::MvStatus legacy_get_object_model_3d_params(::MvTuple model_handle, ::MvStringView name,
                                                  ::MvTuple* output) {
  return mv::c_api::translate([&] {
    require_output(output, "mv_get_object_model_3d_params");
    const auto key = sv(name, "mv_get_object_model_3d_params");
    mv::core::Tuple result;
    if (!model_handle) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_get_object_model_3d_params", "ObjectModel3D", -1,
                      "object model handle Tuple must not be null");
    }
    for (std::size_t item = 0; item < model_handle->value.size(); ++item) {
      const auto& value = tuple_object_model(model_handle, item, "mv_get_object_model_3d_params");
      if (key == "num_points") {
        result.append(static_cast<std::int64_t>(value.points.size()));
      } else if (key == "width") {
        result.append(static_cast<std::int64_t>(value.width));
      } else if (key == "height") {
        result.append(static_cast<std::int64_t>(value.height));
      } else if (key == "num_valid") {
        std::size_t count = value.points.size();
        if (!value.valid.empty())
          count = static_cast<std::size_t>(
              std::count_if(value.valid.begin(), value.valid.end(), [](std::uint8_t v) { return v != 0U; }));
        result.append(static_cast<std::int64_t>(count));
      } else if (key == "has_confidence") {
        result.append(!value.confidence.empty());
      } else if (key == "has_reconstruction_quality") {
        result.append(!value.observed_valid.empty());
      } else if (key == "observed_valid") {
        for (auto v : value.observed_valid) result.append(static_cast<std::int64_t>(v));
      } else if (key == "filled") {
        for (auto v : value.filled) result.append(static_cast<std::int64_t>(v));
      } else if (key == "quality_reason") {
        for (auto v : value.quality_reason) result.append(static_cast<std::int64_t>(v));
      } else if (key == "depth_uncertainty") {
        for (auto v : value.depth_uncertainty) result.append(static_cast<double>(v));
      } else if (key == "confidence") {
        for (float confidence : value.confidence) result.append(static_cast<double>(confidence));
      } else if (key == "has_normals") {
        result.append(static_cast<std::int64_t>(value.normals.empty() ? 0 : 1));
      } else if (key == "point_normal_x" || key == "point_normal_y" || key == "point_normal_z") {
        if (value.normals.empty()) {
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params",
                          "object model has no normals");
        }
        for (std::size_t i = 0; i < value.normals.size(); ++i) {
          if (!model_point_present(value, i)) continue;
          const auto& n = value.normals[i];
          result.append(key == "point_normal_x" ? n.x : key == "point_normal_y" ? n.y : n.z);
        }
      } else if (key == "num_triangles") {
        result.append(static_cast<std::int64_t>(value.triangles_are_polygons ? 0 : value.triangles.size()));
      } else if (key == "num_polygons") {
        result.append(static_cast<std::int64_t>(value.triangles_are_polygons ? value.triangles.size() : 0));
      } else if (key == "has_polygons") {
        result.append(value.triangles_are_polygons && !value.triangles.empty() ? "true" : "false");
      } else if (key == "polygons") {
        if (!value.triangles_are_polygons || value.triangles.empty())
          throw mv::core::Exception(mv::core::Status::InvalidState, mv::core::ErrorCode::InvalidValue,
                                    "mv_get_object_model_3d_params", "GenParamName", -1,
                                    "model has no polygons");
        for (const auto& polygon : value.triangles) {
          result.append(std::int64_t{3});
          for (const auto index : polygon) result.append(static_cast<std::int64_t>(index));
        }
      } else if (key == "num_lines") {
        result.append(static_cast<std::int64_t>(value.lines.size()));
      } else if (key == "has_lines") {
        result.append(value.lines.empty() ? "false" : "true");
      } else if (key == "lines") {
        if (value.lines.empty())
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "object model has no lines");
        for (const auto& line : value.lines) {
          result.append(std::int64_t{2});
          result.append(static_cast<std::int64_t>(line[0]));
          result.append(static_cast<std::int64_t>(line[1]));
        }
      } else if (key == "primitive_type") {
        using Kind = mv::reconstruction::PrimitiveKind;
        if (value.primitive_kind == Kind::None)
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no primitive");
        result.append(value.primitive_kind == Kind::Plane      ? "plane"
                      : value.primitive_kind == Kind::Sphere   ? "sphere"
                      : value.primitive_kind == Kind::Cylinder ? "cylinder"
                                                               : "none");
      } else if (key == "primitive_parameter") {
        if (value.primitive_kind == mv::reconstruction::PrimitiveKind::Plane) {
          for (size_t i = 0; i < 3; ++i) result.append(value.plane[i]);
          result.append(-value.plane[3]);
        } else if (value.primitive_kind != mv::reconstruction::PrimitiveKind::None) {
          const size_t count = value.primitive_kind == mv::reconstruction::PrimitiveKind::Sphere ? 4 : 7;
          for (size_t i = 0; i < count; ++i) result.append(value.primitive[i]);
        } else
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no primitive");
      } else if (key == "primitive_pose" || key == "primitive_parameter_pose") {
        if (value.primitive_kind == mv::reconstruction::PrimitiveKind::None)
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no primitive");
        const auto pose = mv::reconstruction::primitive_frame_pose(value);
        for (size_t i = 0; i < pose.size(); ++i) result.append(pose.at(i));
        if (key == "primitive_parameter_pose" &&
            value.primitive_kind != mv::reconstruction::PrimitiveKind::Plane)
          result.append(
              value.primitive[value.primitive_kind == mv::reconstruction::PrimitiveKind::Sphere ? 3 : 6]);
      } else if (key == "primitive_parameter_extension" || key == "num_primitive_parameter_extension") {
        const auto kind = value.primitive_kind;
        if (kind == mv::reconstruction::PrimitiveKind::None)
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no primitive");
        if (!value.primitive_support)
          throw Exception(Status::Unsupported, "mv_get_object_model_3d_params",
                          "primitive support metadata unavailable");
        const auto& support = *value.primitive_support;
        if (key == "num_primitive_parameter_extension") {
          result.append(static_cast<int64_t>(kind == mv::reconstruction::PrimitiveKind::Plane
                                                 ? 2 * (support.plane_boundary.size() + 1)
                                             : kind == mv::reconstruction::PrimitiveKind::Cylinder ? 2
                                                                                                   : 0));
        } else if (kind == mv::reconstruction::PrimitiveKind::Cylinder) {
          for (auto v : support.cylinder_extent) result.append(v);
        } else if (kind == mv::reconstruction::PrimitiveKind::Plane) {
          const auto n = support.plane_boundary.size();
          for (const auto axis : {support.axis_x, support.axis_y})
            for (size_t j = 0; j <= n; ++j) {
              const auto p = support.plane_boundary[(n - j) % n];
              result.append((p.x - support.origin.x) * axis.x + (p.y - support.origin.y) * axis.y +
                            (p.z - support.origin.z) * axis.z);
            }
        } else
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params",
                          "primitive has no extension");
      } else if (key == "source_indices") {
        for (size_t i = 0; i < value.points.size(); ++i)
          result.append(static_cast<int64_t>(value.source_indices.empty() ? i : value.source_indices[i]));
      } else if (key == "voxel_counts") {
        for (auto count : value.voxel_counts) result.append(static_cast<int64_t>(count));
      } else if (key == "input_to_sample") {
        for (auto index : value.input_to_sample) result.append(index);
      } else if (key == "fit_iterations") {
        result.append(static_cast<int64_t>(value.fit_iterations));
      } else if (key == "fit_converged") {
        result.append(static_cast<int64_t>(value.fit_converged));
      } else if (key == "information_ratio") {
        result.append(value.information_ratio);
      } else if (key == "triangles") {
        if (value.triangles_are_polygons && !value.triangles.empty())
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no triangles");
        for (const auto& triangle : value.triangles)
          for (auto index : triangle) result.append(static_cast<int64_t>(index));
      } else if (key == "primitive_rms") {
        if (value.primitive_kind == mv::reconstruction::PrimitiveKind::None)
          throw Exception(Status::InvalidState, "mv_get_object_model_3d_params", "model has no primitive");
        result.append(value.primitive_rms);
      } else if (key == "num_inliers") {
        result.append(static_cast<std::int64_t>(value.primitive_num_inliers));
      } else {
        throw Exception(Status::InvalidArgument, "mv_get_object_model_3d_params", "unknown parameter");
      }
    }
    *output = make_tuple(std::move(result)).release();
  });
}

extern "C" ::MvStatus mv_xyz_to_object_model_3d(::MvObject X, ::MvObject Y, ::MvObject Z,
                                              ::MvTuple* ObjectModel3D) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_xyz_to_object_model_3d";
    mv::c_api::ControlOutputs<1> outputs(operation, {ObjectModel3D});
    const auto& x = image_objects(X, operation);
    const auto& y = image_objects(Y, operation);
    const auto& z = image_objects(Z, operation);
    if (!x.empty() && !y.empty() && !z.empty())
      append_object_model(&outputs[0], mv::reconstruction::xyz_to_object_model_3d(
                                           x.at(0).as_image(), y.at(0).as_image(), z.at(0).as_image()));
    outputs.Commit();
  });
}

extern "C" ::MvStatus mv_get_object_model_3d_params(::MvTuple ObjectModel3D, ::MvTuple GenParamName,
                                                  ::MvTuple* GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto operation = "mv_get_object_model_3d_params";
    mv::c_api::ControlOutputs<1> outputs(operation, {GenParamValue});
    const mv::c_api::TupleInput models(ObjectModel3D, operation, "ObjectModel3D");
    const mv::c_api::TupleInput names(GenParamName, operation, "GenParamName");
    for (std::size_t i = 0; i < names.size(); ++i) (void)names.string(i);
    for (std::size_t i = 0; i < models.size(); ++i) {
      const auto wrapper = model_control(models, i, operation);
      const auto& data = object_model(wrapper->handle(), operation);
      const auto attributes = data.point_attributes.Read();
      for (std::size_t j = 0; j < names.size(); ++j) {
        const auto name = names.string(j);
        if (data.points.empty() &&
            (name == "point_coord_x" || name == "point_coord_y" || name == "point_coord_z"))
          names.fail(mv::core::ErrorCode::InvalidValue, j, "model has no point coordinate data");
        if (name == "num_points") {
          std::size_t count = 0;
          for (std::size_t p = 0; p < data.points.size(); ++p) count += model_point_present(data, p);
          outputs[0].append(static_cast<std::int64_t>(count));
        } else if (name == "point_coord_x" || name == "point_coord_y" || name == "point_coord_z") {
          if (data.points.empty())
            throw mv::core::Exception(Status::InvalidState, mv::core::ErrorCode::InvalidValue, operation,
                                      "GenParamName", j, "model has no point coordinates");
          for (std::size_t p = 0; p < data.points.size(); ++p)
            if (model_point_present(data, p)) {
              const auto& point = data.points[p];
              outputs[0].append(name == "point_coord_x"   ? point.x
                                : name == "point_coord_y" ? point.y
                                                          : point.z);
            }
        } else if (name == "has_primitive_data" || name == "has_primitive_rms") {
          const bool present =
              data.primitive_kind != mv::reconstruction::PrimitiveKind::None &&
              (name == "has_primitive_data" || data.fit_converged || data.primitive_num_inliers > 0);
          outputs[0].append(present ? "true" : "false");
        } else if (name == "has_points" || name == "has_point_normals" || name == "has_triangles" ||
                   name == "has_xyz_mapping") {
          const bool present = name == "has_points"          ? !data.points.empty()
                               : name == "has_point_normals" ? !data.normals.empty()
                               : name == "has_triangles"
                                   ? (!data.triangles_are_polygons && !data.triangles.empty())
                                   : (data.width != 0 || data.mapping_width != 0);
          outputs[0].append(present ? "true" : "false");
        } else if (name == "mapping_size" || name == "mapping_row" || name == "mapping_col") {
          const auto width = data.mapping_width ? data.mapping_width : data.width;
          const auto height = data.mapping_height ? data.mapping_height : data.height;
          if (!width) names.fail(mv::core::ErrorCode::InvalidValue, j, "model has no XYZ mapping");
          if (name == "mapping_size") {
            outputs[0].append(static_cast<std::int64_t>(width));
            outputs[0].append(static_cast<std::int64_t>(height));
          } else {
            for (std::size_t p = 0; p < data.points.size(); ++p)
              if (model_point_present(data, p)) {
                const auto slot = data.source_indices.empty() ? p : data.source_indices[p];
                outputs[0].append(
                    static_cast<std::int64_t>(name == "mapping_row" ? slot / width : slot % width));
              }
          }
        } else if (name == "extended_attribute_names") {
          if (attributes)
            for (const auto& [key, values] : attributes->values) outputs[0].append(std::string_view(key));
        } else if (name.starts_with("&") || name == "original_point_indices") {
          const auto values = mv::reconstruction::PointAttributes::Find(attributes, name);
          if (!values) names.fail(mv::core::ErrorCode::InvalidValue, j, "extended attribute is missing");
          for (std::size_t p = 0; p < data.points.size(); ++p)
            if (model_point_present(data, p)) outputs[0].append((*values)[p]);
        } else if (name == "has_normals") {
          names.fail(mv::core::ErrorCode::InvalidValue, j, "use has_point_normals");
        } else {
          MvTupleImpl model;
          model.value.append(models.at(i));
          ::MvTuple raw = nullptr;
          const ::MvStringView text{name.data(), name.size()};
          const auto status = legacy_get_object_model_3d_params(&model, text, &raw);
          if (status != MV_STATUS_OK) {
            const auto& info = mv::c_api::last_error_info();
            throw Exception(static_cast<Status>(status), static_cast<mv::core::ErrorCode>(info.code),
                            operation, "GenParamName", j, {info.message.data, info.message.size});
          }
          std::unique_ptr<MvTupleImpl> result(raw);
          for (std::size_t p = 0; p < result->value.size(); ++p) outputs[0].append(result->value.at(p));
        }
      }
    }
    outputs.Commit();
  });
}

static ::MvStatus legacy_set_structured_light_model_param(::MvTuple model, ::MvStringView name, ::MvTuple value) {
  return mv::c_api::translate([&] {
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_set_structured_light_model_param", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    const auto key = sv(name, "mv_set_structured_light_model_param");
    const auto& parameter = tuple(value, "mv_set_structured_light_model_param");
    std::unordered_map<MvStructuredLightModelImpl*, std::unique_ptr<mv::reconstruction::StructuredLightModel>>
        candidates;
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      auto* target = tuple_structured_handle(model, item, "mv_set_structured_light_model_param");
      auto [entry, inserted] = candidates.try_emplace(target);
      if (inserted) {
        entry->second = std::make_unique<mv::reconstruction::StructuredLightModel>(target->value->clone());
      }
      mv::reconstruction::set_structured_light_model_param(*entry->second, key, parameter);
    }
    for (auto& entry : candidates) *entry.first->value = std::move(*entry.second);
  });
}

static ::MvStatus legacy_get_structured_light_model_param(::MvTuple model, ::MvStringView name,
                                                        ::MvTuple* output) {
  return mv::c_api::translate([&] {
    require_output(output, "mv_get_structured_light_model_param");
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_get_structured_light_model_param", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    const auto key = sv(name, "mv_get_structured_light_model_param");
    mv::core::Tuple result;
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      mv::core::Tuple part;
      mv::reconstruction::get_structured_light_model_param(
          tuple_structured_model(model, item, "mv_get_structured_light_model_param"), key, &part);
      result = result.concat(part);
    }
    *output = make_tuple(std::move(result)).release();
  });
}

static ::MvStatus legacy_gen_structured_light_pattern(::MvTuple model, ::MvObject* pattern_images) {
  return mv::c_api::translate([&] {
    require_output(pattern_images, "mv_gen_structured_light_pattern");
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_gen_structured_light_pattern", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    std::unordered_map<MvStructuredLightModelImpl*, std::unique_ptr<mv::reconstruction::StructuredLightModel>>
        candidates;
    mv::core::IconicTuple result;
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      auto* target = tuple_structured_handle(model, item, "mv_gen_structured_light_pattern");
      auto [entry, inserted] = candidates.try_emplace(target);
      if (inserted) {
        entry->second = std::make_unique<mv::reconstruction::StructuredLightModel>(target->value->clone());
      }
      result = result.concat(mv::reconstruction::gen_structured_light_pattern(*entry->second));
    }
    auto output_handle = mv::c_api::make_image_handle(std::move(result), "mv_gen_structured_light_pattern");
    for (auto& entry : candidates) *entry.first->value = std::move(*entry.second);
    *pattern_images = output_handle.release();
  });
}

static ::MvStatus legacy_decode_structured_light_pattern(::MvObject camera_images, ::MvTuple model) {
  return mv::c_api::translate([&] {
    const auto& images = image_objects(camera_images, "mv_decode_structured_light_pattern");
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_decode_structured_light_pattern", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    std::unordered_map<MvStructuredLightModelImpl*, std::unique_ptr<mv::reconstruction::StructuredLightModel>>
        candidates;
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      auto* target = tuple_structured_handle(model, item, "mv_decode_structured_light_pattern");
      auto [entry, inserted] = candidates.try_emplace(target);
      if (inserted) {
        entry->second = std::make_unique<mv::reconstruction::StructuredLightModel>(target->value->clone());
      }
      mv::reconstruction::decode_structured_light_pattern(images, *entry->second);
    }
    for (auto& entry : candidates) *entry.first->value = std::move(*entry.second);
  });
}

static ::MvStatus legacy_get_structured_light_object(::MvTuple model, ::MvStringView object_name,
                                                   ::MvObject* images) {
  return mv::c_api::translate([&] {
    require_output(images, "mv_get_structured_light_object");
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_get_structured_light_object", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    const auto key = sv(object_name, "mv_get_structured_light_object");
    mv::core::IconicTuple result;
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      result = result.concat(mv::reconstruction::get_structured_light_object(
          tuple_structured_model(model, item, "mv_get_structured_light_object"), key));
    }
    *images = mv::c_api::make_iconic(std::move(result)).release();
  });
}

static ::MvStatus legacy_reconstruct_surface_structured_light(::MvTuple model, ::MvTuple* output) {
  return mv::c_api::translate([&] {
    require_output(output, "mv_reconstruct_surface_structured_light");
    if (!model) {
      throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument,
                      "mv_reconstruct_surface_structured_light", "StructuredLightModel", -1,
                      "structured-light model handle Tuple must not be null");
    }
    auto results = std::make_unique<MvTupleImpl>();
    for (std::size_t item = 0; item < model->value.size(); ++item) {
      append_object_model(&results->value,
                          mv::reconstruction::reconstruct_surface_structured_light(tuple_structured_model(
                              model, item, "mv_reconstruct_surface_structured_light")));
    }
    *output = results.release();
  });
}

namespace {
using mv::c_api::TupleInput;
class LightControlStorage {
 public:
  LightControlStorage(::MvTuple input, const char* op, const char* parameter) {
    TupleInput checked(input, op, parameter);
    if (input && input->refs.load(std::memory_order_relaxed)!=0)
      handle_ = input;
    else {
      storage_.value = checked.materialize();
      handle_ = &storage_;
    }
  }
  operator ::MvTuple() const noexcept { return handle_; }

 private:
  MvTupleImpl storage_;
  ::MvTuple handle_{};
};
::MvStringView light_text(::MvTuple input, const char* op, const char* parameter) {
  TupleInput value(input, op, parameter);
  if (value.size() != 1)
    value.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one string required");
  auto text = value.string(0);
  return {text.data(), text.size()};
}
MvStructuredLightModelImpl& light_model(::MvTuple input, const char* op) {
  TupleInput value(input, op, "StructuredLightModel");
  if (value.size() != 1)
    value.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one structured-light model required");
  try {
    auto wrapper = value.at(0).as_handle().as<mv::c_api::StructuredLightModelObject>();
    auto* model = wrapper->handle();
    (void)structured_model(model, op);
    return *model;
  } catch (const mv::core::Exception& e) {
    throw Exception(e.status(), e.error_code(), op, "StructuredLightModel", 0, e.message());
  }
}
std::filesystem::path light_path(::MvTuple input, const char* op) {
  auto text = light_text(input, op, "FileName");
  std::string_view value(text.data, text.size);
  if (value.find('\0') != std::string_view::npos)
    throw Exception(Status::InvalidArgument, op, "path must be UTF-8 without embedded NUL");
  if (value.empty()) return std::filesystem::path{".mvslm"};
  return std::filesystem::u8path(value.begin(), value.end());
}
}  // namespace
extern "C" ::MvStatus mv_create_structured_light_model(::MvTuple ModelType,
                                                     ::MvTuple* StructuredLightModel) {
  return mv::c_api::translate([&] {
    const char* op = "mv_create_structured_light_model";
    mv::c_api::ControlOutputs<1> out(op, {StructuredLightModel});
    auto type = light_text(ModelType, op, "ModelType");
    append_structured_model(&out[0],
                            mv::reconstruction::create_structured_light_model({type.data, type.size}));
    out.Commit();
  });
}
extern "C" ::MvStatus mv_write_structured_light_model(::MvTuple StructuredLightModel,
                                                    ::MvTuple FileName) {
  return mv::c_api::translate([&] {
    const char* op = "mv_write_structured_light_model";
    auto& model = light_model(StructuredLightModel, op);
    mv::reconstruction::write_structured_light_model(light_path(FileName, op), *model.value);
  });
}
extern "C" ::MvStatus mv_read_structured_light_model(::MvTuple FileName, ::MvTuple* StructuredLightModel) {
  return mv::c_api::translate([&] {
    const char* op = "mv_read_structured_light_model";
    mv::c_api::ControlOutputs<1> out(op, {StructuredLightModel});
    append_structured_model(&out[0],
                            mv::reconstruction::read_structured_light_model(light_path(FileName, op)));
    out.Commit();
  });
}
extern "C" ::MvStatus mv_set_structured_light_model_param(::MvTuple StructuredLightModel,
                                                        ::MvTuple GenParamName,
                                                        ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    const char* op = "mv_set_structured_light_model_param";
    auto& model = light_model(StructuredLightModel, op);
    TupleInput names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    if (!names.size())
      names.fail(mv::core::ErrorCode::InvalidCardinality, -1, "parameter names must not be empty");
    if (names.size() != values.size())
      values.fail(mv::core::ErrorCode::InvalidCardinality, -1, "parameter columns must have equal lengths");
    for (std::size_t i = 0; i < names.size(); ++i) (void)names.string(i);
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto key = names.string(i);
      if (key == "camera_setup_model") {
        const auto value = values.at(i);
        if (value.kind() == mv::core::ValueKind::Int64 && value.as_int64() == 0) {
          model.value->set_camera_setup(std::nullopt);
        } else {
          if (value.kind() != mv::core::ValueKind::Handle)
            values.fail(mv::core::ErrorCode::InvalidType, i, "camera setup handle required");
          try {
            const auto owner = value.as_handle().as<mv::c_api::CameraSetupModelObject>();
            if (!owner->handle()->valid.load() || !owner->handle()->value)
              values.fail(mv::core::ErrorCode::InvalidObjectState, i, "camera setup is cleared");
            model.value->set_camera_setup(*owner->handle()->value);
          } catch (const mv::core::Exception& error) {
            throw mv::core::Exception(error.status(), error.error_code(), op, "GenParamValue", i, error.message());
          }
        }
        continue;
      }
      if (key == "pattern_width" || key == "pattern_height" || key == "min_stripe_width" ||
          key == "single_stripe_width" || key == "min_gray_difference")
        (void)values.integer(i);
      if (key == "persistence" && values.at(i).kind() == mv::core::ValueKind::Real64)
        values.fail(mv::core::ErrorCode::InvalidType, i, "persistence requires string or integer");
      mv::core::Tuple single;
      single.append(values.at(i));
      // 每次内核更新独立提供强异常保证；HALCON 保留此前成功更新。
      mv::reconstruction::set_structured_light_model_param(*model.value, names.string(i), single);
    }
  });
}
extern "C" ::MvStatus mv_get_structured_light_model_param(::MvTuple StructuredLightModel,
                                                        ::MvTuple GenParamName, ::MvTuple* GenParamValue) {
  return mv::c_api::translate([&] {
    const char* op = "mv_get_structured_light_model_param";
    mv::c_api::ControlOutputs<1> out(op, {GenParamValue});
    auto& model = light_model(StructuredLightModel, op);
    TupleInput names(GenParamName, op, "GenParamName");
    if (!names.size()) names.fail(mv::core::ErrorCode::InvalidCardinality, -1, "query must not be empty");
    for (std::size_t i = 0; i < names.size(); ++i) {
      mv::core::Tuple part;
      if (names.string(i) == "camera_setup_model") {
        if (model.value->camera_setup()) append_camera_setup(&part, *model.value->camera_setup());
        else part.append(std::int64_t{0});
      } else {
        mv::reconstruction::get_structured_light_model_param(*model.value, names.string(i), &part);
      }
      out[0] = out[0].concat(part);
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_gen_structured_light_pattern(::MvObject* PatternImages,
                                                    ::MvTuple StructuredLightModel) {
  return mv::c_api::translate([&] {
    const char* op = "mv_gen_structured_light_pattern";
    require_output(PatternImages, op);
    auto& model = light_model(StructuredLightModel, op);
    auto candidate = model.value->clone();
    auto result =
        mv::c_api::make_image_handle(mv::reconstruction::gen_structured_light_pattern(candidate), op);
    *model.value = std::move(candidate);
    *PatternImages = result.release();
  });
}
extern "C" ::MvStatus mv_decode_structured_light_pattern(::MvObject CameraImages,
                                                       ::MvTuple StructuredLightModel) {
  return mv::c_api::translate([&] {
    const char* op = "mv_decode_structured_light_pattern";
    auto& model = light_model(StructuredLightModel, op);
    auto candidate = model.value->clone();
    mv::reconstruction::decode_structured_light_pattern(image_objects(CameraImages, op), candidate);
    *model.value = std::move(candidate);
  });
}
extern "C" ::MvStatus mv_get_structured_light_object(::MvObject* Object, ::MvTuple StructuredLightModel,
                                                   ::MvTuple ObjectName) {
  return mv::c_api::translate([&] {
    const char* op = "mv_get_structured_light_object";
    require_output(Object, op);
    auto& model = light_model(StructuredLightModel, op);
    auto key = light_text(ObjectName, op, "ObjectName");
    auto result = mv::reconstruction::get_structured_light_object(*model.value, {key.data, key.size});
    *Object = mv::c_api::make_iconic(std::move(result)).release();
  });
}
extern "C" ::MvStatus mv_reconstruct_surface_structured_light(::MvTuple StructuredLightModel,
                                                            ::MvTuple* ObjectModel3D) {
  return mv::c_api::translate([&] {
    const char* op = "mv_reconstruct_surface_structured_light";
    mv::c_api::ControlOutputs<1> out(op, {ObjectModel3D});
    auto& model = light_model(StructuredLightModel, op);
    append_object_model(&out[0], mv::reconstruction::reconstruct_surface_structured_light(*model.value));
    out.Commit();
  });
}

namespace {
using SetupInput = mv::c_api::TupleInput;
mv::c_api::CameraSetupModelObject* setup_owner(const SetupInput& input) {
  if (input.size() != 1)
    input.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one setup handle required");
  try {
    return &input.handle(0).object<mv::c_api::CameraSetupModelObject>();
  } catch (const mv::core::Exception& e) {
    input.fail(e.error_code(), 0, e.message());
  }
}
std::string setup_text(const SetupInput& input) {
  if (input.size() != 1)
    input.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one string required");
  auto value = input.string(0);
  if (value.find('\0') != std::string_view::npos)
    input.fail(mv::core::ErrorCode::InvalidValue, 0, "embedded NUL is not supported");
  return std::string(value);
}
std::int64_t setup_index(const SetupInput& input, const mv::reconstruction::CameraSetupModel& model) {
  const auto index = input.scalar_integer();
  if (index < 0 || static_cast<std::uint64_t>(index) >= model.camera_count())
    input.fail(mv::core::ErrorCode::IndexOutOfRange, 0, "camera index outside setup");
  return index;
}
}  // namespace
extern "C" ::MvStatus mv_create_camera_setup_model(::MvTuple NumCameras, ::MvTuple* CameraSetupModelID) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_create_camera_setup_model";
    mv::c_api::ControlOutputs<1> out(op, {CameraSetupModelID});
    const SetupInput count(NumCameras, op, "NumCameras");
    const auto n = count.scalar_integer();
    if (n <= 0) count.fail(mv::core::ErrorCode::InvalidValue, 0, "camera count must be positive");
    append_camera_setup(&out[0], mv::reconstruction::CameraSetupModel::create(n));
    out.Commit();
  });
}
extern "C" ::MvStatus mv_set_camera_setup_cam_param(::MvTuple CameraSetupModelID, ::MvTuple CameraIdx,
                                                  ::MvTuple CameraType, ::MvTuple CameraParam,
                                                  ::MvTuple CameraPose) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_camera_setup_cam_param";
    const SetupInput id(CameraSetupModelID, op, "CameraSetupModelID"), idx(CameraIdx, op, "CameraIdx"),
        type(CameraType, op, "CameraType"), params(CameraParam, op, "CameraParam"),
        pose(CameraPose, op, "CameraPose");
    auto owner = setup_owner(id);
    auto& model = *owner->handle()->value;
    const auto i = setup_index(idx, model);
    if (params.size() == 0)
      params.fail(mv::core::ErrorCode::InvalidCardinality, -1, "camera parameters must not be empty");
    if (type.size() != 0 && setup_text(type) != params.string(0))
      type.fail(mv::core::ErrorCode::InvalidValue, 0, "camera type must match parameter tuple");
    mv::calibration::CameraParameters parsed;
    try {
      if(const auto* owned=params.owned_tuple())parsed=mv::calibration::parse_camera_parameters(*owned);
      else parsed = mv::calibration::parse_camera_parameters(params.materialize());
    } catch (const mv::core::Exception& e) {
      params.fail(e.error_code(), -1, e.message());
    }
    if (pose.size() != 7)
      pose.fail(mv::core::ErrorCode::InvalidCardinality, -1, "pose requires seven values");
    pose.integer(6, true);
    for (std::size_t j = 0; j < 6; ++j) pose.number(j);
    try {
      if(const auto* owned=pose.owned_tuple())model.set_camera(i,std::move(parsed),*owned);
      else model.set_camera(i, std::move(parsed), pose.materialize());
    } catch (const mv::core::Exception& e) {
      pose.fail(e.error_code(), -1, e.message());
    }
  });
}
extern "C" ::MvStatus mv_get_camera_setup_param(::MvTuple CameraSetupModelID, ::MvTuple CameraIdx,
                                              ::MvTuple GenParamName, ::MvTuple* GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_get_camera_setup_param";
    mv::c_api::ControlOutputs<1> out(op, {GenParamValue});
    const SetupInput id(CameraSetupModelID, op, "CameraSetupModelID"), idx(CameraIdx, op, "CameraIdx"),
        name(GenParamName, op, "GenParamName");
    auto owner = setup_owner(id);
    const auto& model = *owner->handle()->value;
    const auto key = setup_text(name);
    if (idx.size() == 1 && idx.at(0).kind() == mv::core::ValueKind::String) {
      if (setup_text(idx) != "general") idx.fail(mv::core::ErrorCode::InvalidValue, 0, "expected 'general'");
      if (key == "num_cameras")
        out[0].append(static_cast<std::int64_t>(model.camera_count()));
      else if (key == "reference_camera")
        out[0].append(std::int64_t{-1});
      else if (key == "coord_transf_pose")
        out[0] = mv::calibration::pose_to_halcon_tuple(mv::Pose3d{});
      else if (key != "camera_calib_error")
        name.fail(mv::core::ErrorCode::InvalidValue, 0, "unknown general setup parameter");
    } else {
      const auto i = setup_index(idx, model);
      if (key != "type" && key != "params" && key != "pose" && key != "params_deviations" &&
          key != "params_covariances")
        name.fail(mv::core::ErrorCode::InvalidValue, 0, "unknown camera setup parameter");
      try {
        const auto parameters = mv::calibration::camera_parameters_to_tuple(model.camera(i));
        if (key == "type")
          out[0].append(parameters.at(0));
        else if (key == "params")
          out[0] = parameters;
        else if (key == "pose")
          out[0] = model.camera_pose(i);
      } catch (const mv::core::Exception& e) {
        idx.fail(e.error_code(), 0, e.message());
      }
    }
    out.Commit();
  });
}
