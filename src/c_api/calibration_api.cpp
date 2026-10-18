#include <mvkit/c/mv_calibration.h>

#include <array>
#include <calibration/model.hpp>
#include <calibration/camera_parameters.hpp>
#include <calibration/estimation.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/parameters.hpp>
#include <calibration/persist.hpp>
#include <calibration/pose_estimation.hpp>
#include <calibration/projection.hpp>
#include <calibration/projective_estimation.hpp>
#include <calibration/rectification.hpp>
#include <calibration/targets/find_calib_object.hpp>
#include <climits>
#include <cmath>
#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <core/iconic_tuple.hpp>
#include <imageproc/operators.hpp>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include "structured_error.hpp"
#include "tuple_input.hpp"
#include "xld_cache.hpp"
namespace {
using mv::c_api::Exception;
using mv::core::Status;
void out(const void* p, const char* w) {
  if (!p) throw Exception(Status::InvalidArgument, w, "output pointer must not be null");
}
mv::calibration::CalibrationDataModel& model(::MvCalibData h, const char* w) {
  if (!h || !h->value) throw Exception(Status::InvalidArgument, w, "calibration data must not be null");
  return *h->value;
}
const mv::core::Tuple& tup(::MvTuple h, const char* w) {
  if (!h) throw Exception(Status::InvalidArgument, w, "tuple handle must not be null");
  return h->value;
}
std::unique_ptr<MvTupleImpl> make_tuple(mv::core::Tuple value) {
  auto h = std::make_unique<MvTupleImpl>();
  h->value = std::move(value);
  return h;
}
std::unique_ptr<MvObjectImpl> make_xld(mv::core::XldContours value) {
  return std::unique_ptr<MvObjectImpl>{mv::c_api::make_xld_contours(std::move(value))};
}
void require_image(::MvObject image, const char* where) {
  if (!image)
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, where, "Image", -1,
                    "image handle must not be null");
  image->value.require_kind(mv::core::IconicKind::Image, where);
}
void require_xld(::MvObject contours, const char* where) {
  if (!contours)
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::NullArgument, where, "Contours", -1,
                    "contour handle must not be null");
  contours->value.require_kind(mv::core::IconicKind::XldContour, where);
}
void require_distinct(const void* a, const void* b, const char* where) {
  if (a == b)
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::InvalidValue, where, "outputs", -1,
                    "output pointers must be distinct");
}
void commit_maps(mv::core::Image row, mv::core::Image col, ::MvObject* row_map, ::MvObject* col_map) {
  auto rh = mv::c_api::make_image_handle(std::move(row));
  auto ch = mv::c_api::make_image_handle(std::move(col));
  *row_map = rh.release();
  *col_map = ch.release();
}
}  // namespace

extern "C" ::MvStatus mv_contour_to_world_plane_xld(::MvObject Contours, ::MvObject* ContoursTrans,
                                                  ::MvTuple CameraParam, ::MvTuple WorldPose,
                                                  ::MvTuple Scale) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_contour_to_world_plane_xld";
    require_xld(Contours, op);
    out(ContoursTrans, op);
    const mv::c_api::ControlTupleView camera(CameraParam, op, "CameraParam");
    const mv::c_api::ControlTupleView pose(WorldPose, op, "WorldPose");
    const mv::c_api::ControlTupleView scale(Scale, op, "Scale");
    // iconic 数组为空时仍验证 HALCON 正式控制。
    auto project = mv::calibration::WorldPlaneProjection::prepare(camera, pose, scale, true);
    auto result = mv::calibration::contour_to_world_plane_xld(mv::c_api::xld_value(Contours, op), project);
    auto handle = make_xld(std::move(result));
    *ContoursTrans = handle.release();
  });
}
extern "C" ::MvStatus mv_change_radial_distortion_contours_xld(::MvObject Contours,
                                                             ::MvObject* ContoursRectified,
                                                             ::MvTuple CamParamIn,
                                                             ::MvTuple CamParamOut) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_change_radial_distortion_contours_xld";
    require_xld(Contours, op);
    out(ContoursRectified, op);
    const mv::c_api::ControlTupleView input(CamParamIn, op, "CamParamIn");
    const mv::c_api::ControlTupleView output(CamParamOut, op, "CamParamOut");
    const auto camera_in = mv::calibration::parse_camera_parameters(input);
    const auto camera_out = mv::calibration::parse_camera_parameters(output);
    auto result = mv::calibration::change_radial_distortion_contours_xld(mv::c_api::xld_value(Contours, op),
                                                                         camera_in, camera_out);
    auto handle = make_xld(std::move(result));
    *ContoursRectified = handle.release();
  });
}
extern "C" ::MvStatus mv_vector_to_pose(::MvTuple WorldX, ::MvTuple WorldY, ::MvTuple WorldZ,
                                      ::MvTuple ImageRow, ::MvTuple ImageColumn,
                                      ::MvTuple CameraParam, ::MvTuple Method, ::MvTuple QualityType,
                                      ::MvTuple* Pose, ::MvTuple* Quality) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_vector_to_pose";
    mv::c_api::ControlOutputs<2> outputs(op, {Pose, Quality});
    const mv::c_api::TupleInput method(Method, op, "Method"), quality(QualityType, op, "QualityType");
    if (method.size() != 1)
      method.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one method required");
    const auto name = method.string(0);
    if (quality.size() != 1)
      quality.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one quality type required");
    if (quality.string(0) != "error")
      quality.fail(mv::core::ErrorCode::InvalidValue, 0, "only error quality is supported");
    const mv::c_api::ControlTupleView x(WorldX, op, "WorldX"), y(WorldY, op, "WorldY"),
        z(WorldZ, op, "WorldZ"), row(ImageRow, op, "ImageRow"), col(ImageColumn, op, "ImageColumn"),
        camera(CameraParam, op, "CameraParam");
    auto result = mv::calibration::estimate_pose(x, y, z, row, col, camera, name);
    outputs[0] = std::move(result.pose);
    outputs[1].append(result.error);
    outputs.Commit();
  });
}
extern "C" ::MvStatus mv_vector_to_proj_hom_mat2d(::MvTuple Px, ::MvTuple Py, ::MvTuple Qx,
                                                ::MvTuple Qy, ::MvTuple Method, ::MvTuple CovXX1,
                                                ::MvTuple CovYY1, ::MvTuple CovXY1, ::MvTuple CovXX2,
                                                ::MvTuple CovYY2, ::MvTuple CovXY2, ::MvTuple* HomMat2D,
                                                ::MvTuple* Covariance) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_vector_to_proj_hom_mat2d";
    mv::c_api::ControlOutputs<2> outputs(op, {HomMat2D, Covariance});
    const mv::c_api::TupleInput method(Method, op, "Method");
    if (method.size() != 1)
      method.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one method required");
    const auto name = method.string(0);
    if (name != "dlt" && name != "normalized_dlt" && name != "gold_standard")
      method.fail(mv::core::ErrorCode::InvalidValue, 0, "unknown estimation method");
    const mv::c_api::ControlTupleView x(Px, op, "Px"), y(Py, op, "Py"), u(Qx, op, "Qx"), v(Qy, op, "Qy");
    const std::array<::MvTuple, 6> controls{CovXX1, CovYY1, CovXY1, CovXX2, CovYY2, CovXY2};
    constexpr std::array<const char*, 6> labels{"CovXX1", "CovYY1", "CovXY1", "CovXX2", "CovYY2", "CovXY2"};
    std::array<mv::core::Tuple, 6> covariances;
    for (std::size_t k = 0; k < 6; ++k) {
      const mv::c_api::TupleInput input(controls[k], op, labels[k]);
      if (name == "gold_standard") covariances[k] = input.materialize();
    }
    auto result = mv::calibration::estimate_projective_transform(x, y, u, v, name, covariances);
    outputs[0] = std::move(result.homography);
    outputs[1] = std::move(result.covariance);
    outputs.Commit();
  });
}

namespace {
using RectInput = mv::c_api::TupleInput;
std::string rect_text(const RectInput& input) {
  if (input.size() != 1)
    input.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one string required");
  const auto v = input.string(0);
  if (v.find('\0') != std::string_view::npos)
    input.fail(mv::core::ErrorCode::InvalidValue, 0, "embedded NUL is not supported");
  return std::string(v);
}
std::size_t rect_dimension(const RectInput& input) {
  const auto v = input.scalar_number();
  if (v <= 0 || v > double(INT32_MAX))
    input.fail(mv::core::ErrorCode::InvalidValue, 0, "positive representable dimension required");
  const auto rounded = std::floor(v + .5);
  if (rounded < 1) input.fail(mv::core::ErrorCode::InvalidValue, 0, "rounded dimension must be positive");
  return static_cast<std::size_t>(rounded);
}
}  // namespace
extern "C" ::MvStatus mv_gen_radial_distortion_map(::MvObject* Map, ::MvTuple CamParamIn,
                                                 ::MvTuple CamParamOut, ::MvTuple MapType) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_gen_radial_distortion_map";
    mv::c_api::require_output(Map, op, "Map");
    const RectInput in(CamParamIn, op, "CamParamIn"), out(CamParamOut, op, "CamParamOut"),
        type(MapType, op, "MapType");
    auto result = mv::calibration::gen_radial_distortion_map_halcon(in.materialize(), out.materialize(),
                                                                    rect_text(type));
    auto owner = mv::c_api::make_image_handle(std::move(result));
    *Map = owner.release();
  });
}
extern "C" ::MvStatus mv_gen_image_to_world_plane_map(::MvObject* Map, ::MvTuple CameraParam,
                                                    ::MvTuple WorldPose, ::MvTuple WidthIn,
                                                    ::MvTuple HeightIn, ::MvTuple WidthMapped,
                                                    ::MvTuple HeightMapped, ::MvTuple Scale,
                                                    ::MvTuple MapType) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_gen_image_to_world_plane_map";
    mv::c_api::require_output(Map, op, "Map");
    const RectInput camera(CameraParam, op, "CameraParam"), pose(WorldPose, op, "WorldPose"),
        wi(WidthIn, op, "WidthIn"), hi(HeightIn, op, "HeightIn"), w(WidthMapped, op, "WidthMapped"),
        h(HeightMapped, op, "HeightMapped"), scale(Scale, op, "Scale"), type(MapType, op, "MapType");
    auto result = mv::calibration::gen_image_to_world_plane_map_halcon(
        camera.materialize(), pose.materialize(), rect_dimension(wi), rect_dimension(hi), rect_dimension(w),
        rect_dimension(h), scale.materialize(), rect_text(type));
    auto owner = mv::c_api::make_image_handle(std::move(result));
    *Map = owner.release();
  });
}
extern "C" ::MvStatus mv_change_radial_distortion_image(::MvObject Image, ::MvObject Region,
                                                      ::MvObject* ImageRectified, ::MvTuple CamParamIn,
                                                      ::MvTuple CamParamOut) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_change_radial_distortion_image";
    mv::c_api::require_output(ImageRectified, op, "ImageRectified");
    mv::c_api::require_iconic(Image, op, "Image");
    mv::c_api::require_iconic(Region, op, "Region");
    Image->value.require_kind(mv::core::IconicKind::Image, op);
    if (Image->value.empty()) {
      *ImageRectified = mv::c_api::make_iconic({}).release();
      return;
    }
    Region->value.require_kind(mv::core::IconicKind::Region, op);
    const RectInput in(CamParamIn, op, "CamParamIn"), out(CamParamOut, op, "CamParamOut");
    auto result = mv::calibration::change_radial_distortion_image_halcon(Image->value, Region->value,
                                                                         in.materialize(), out.materialize());
    auto owner = mv::c_api::make_image_handle(std::move(result), op);
    *ImageRectified = owner.release();
  });
}
extern "C" ::MvStatus mv_image_to_world_plane(::MvObject Image, ::MvObject* ImageWorld,
                                            ::MvTuple CameraParam, ::MvTuple WorldPose,
                                            ::MvTuple Width, ::MvTuple Height, ::MvTuple Scale,
                                            ::MvTuple Interpolation) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_image_to_world_plane";
    mv::c_api::require_output(ImageWorld, op, "ImageWorld");
    mv::c_api::require_iconic(Image, op, "Image");
    Image->value.require_kind(mv::core::IconicKind::Image, op);
    if (Image->value.empty()) {
      *ImageWorld = mv::c_api::make_iconic({}).release();
      return;
    }
    const RectInput camera(CameraParam, op, "CameraParam"), pose(WorldPose, op, "WorldPose"),
        w(Width, op, "Width"), h(Height, op, "Height"), scale(Scale, op, "Scale"),
        mode(Interpolation, op, "Interpolation");
    auto result = mv::calibration::image_to_world_plane_halcon(
        Image->value, camera.materialize(), pose.materialize(), rect_dimension(w), rect_dimension(h),
        scale.materialize(), rect_text(mode));
    auto owner = mv::c_api::make_image_handle(std::move(result), op);
    *ImageWorld = owner.release();
  });
}
