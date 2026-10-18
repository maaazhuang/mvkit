#include <mvkit/c/mv_calibration.h>

#include <array>
#include <calibration/model.hpp>
#include <calibration/camera_model.hpp>
#include <calibration/estimation.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/parameters.hpp>
#include <calibration/persist.hpp>
#include <calibration/projection.hpp>
#include <calibration/targets/find_calib_object.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <vector>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#include <runtime/cpu/cpu_features.hpp>
#define MVKIT_CALIBRATION_CONTROL_SSE2 1
#else
#define MVKIT_CALIBRATION_CONTROL_SSE2 0
#endif

#include "control_object.hpp"
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
#include "xld_cache.hpp"

namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;

mv::c_api::CalibrationDataObject* data(const TupleInput& input) {
  if (input.size() != 1)
    input.fail(ErrorCode::InvalidCardinality, -1, "exactly one calibration data handle required");
  try {
    return &input.handle(0).object<mv::c_api::CalibrationDataObject>();
  } catch (const mv::core::Exception& error) {
    input.fail(error.error_code(), 0, error.message());
  }
}
std::string text(const TupleInput& input) {
  if (input.size() != 1) input.fail(ErrorCode::InvalidCardinality, -1, "exactly one string required");
  const auto value = input.string(0);
  if (value.find('\0') != std::string_view::npos)
    input.fail(ErrorCode::InvalidValue, 0, "embedded NUL is not supported");
  return std::string(value);
}
std::int64_t index(const TupleInput& input, std::size_t count) {
  const auto value = input.scalar_integer();
  if (value < 0 || static_cast<std::uint64_t>(value) >= count)
    input.fail(ErrorCode::IndexOutOfRange, 0, "index outside calibration setup");
  return value;
}
mv::core::Tuple item_index(const TupleInput& input, const std::string& type) {
  if (input.size() == 1 && input.at(0).kind() == mv::core::ValueKind::String) {
    if (text(input) != "general") input.fail(ErrorCode::InvalidValue, 0, "expected 'general'");
    return input.materialize();
  }
  const std::size_t count = type == "calib_obj_pose" ? 2 : 1;
  if (input.size() != count) input.fail(ErrorCode::InvalidCardinality, -1, "invalid item index cardinality");
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (input.integer(i) < 0) input.fail(ErrorCode::IndexOutOfRange, i, "index must be nonnegative");
  }
  return input.materialize();
}
}  // namespace

extern "C" ::MvStatus mv_set_calib_data_observ_points(::MvTuple CalibDataID, ::MvTuple CameraIdx,
                                                    ::MvTuple CalibObjIdx, ::MvTuple CalibObjPoseIdx,
                                                    ::MvTuple Row, ::MvTuple Column, ::MvTuple Index,
                                                    ::MvTuple Pose) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_calib_data_observ_points";
    const TupleInput id(CalibDataID, op, "CalibDataID"), ci(CameraIdx, op, "CameraIdx"),
        oi(CalibObjIdx, op, "CalibObjIdx"), pi(CalibObjPoseIdx, op, "CalibObjPoseIdx"), row(Row, op, "Row"),
        col(Column, op, "Column"), indices(Index, op, "Index"), pose(Pose, op, "Pose");
    auto owner = data(id);
    auto& model = *owner->handle()->value;
    const auto camera = index(ci, model.num_cameras()), object = index(oi, model.num_calibration_objects());
    const auto view = pi.scalar_integer();
    if (view < 0) pi.fail(ErrorCode::IndexOutOfRange, 0, "pose index must be nonnegative");
    if (!model.camera(camera).initial_parameters)
      ci.fail(ErrorCode::InvalidObjectState, 0, "camera is not configured");
    const auto& target = model.target(object);
    if (target.kind == mv::calibration::TargetKind::Unconfigured)
      oi.fail(ErrorCode::InvalidObjectState, 0, "calibration object is not configured");
    if (row.size() != col.size())
      col.fail(ErrorCode::InvalidCardinality, -1, "row/column lengths must match");
    mv::calibration::CalibrationObservation candidate;
    candidate.rows.reserve(row.size());
    candidate.columns.reserve(col.size());
    for (std::size_t i = 0; i < row.size(); ++i) {
      candidate.rows.push_back(row.number(i));
      candidate.columns.push_back(col.number(i));
    }
    candidate.indices.reserve(row.size());
    const bool all = indices.size() == 1 && indices.at(0).kind() == mv::core::ValueKind::String;
    if (all) {
      if (text(indices) != "all") indices.fail(ErrorCode::InvalidValue, 0, "expected 'all'");
      if (row.size() != target.points.size())
        indices.fail(ErrorCode::InvalidCardinality, -1, "'all' requires all target points");
      for (std::size_t i = 0; i < row.size(); ++i) candidate.indices.push_back(static_cast<std::int64_t>(i));
    } else {
      if (indices.size() != row.size())
        indices.fail(ErrorCode::InvalidCardinality, -1, "index length must match row/column");
      std::set<std::int64_t> unique;
      for (std::size_t i = 0; i < indices.size(); ++i) {
        auto value = indices.integer(i);
        if (value < 0 || static_cast<std::uint64_t>(value) >= target.points.size() ||
            !unique.insert(value).second)
          indices.fail(ErrorCode::InvalidValue, i, "indices must be unique and in range");
        candidate.indices.push_back(value);
      }
    }
    if (row.size() == 0) return;
    if (pose.size() != 7) pose.fail(ErrorCode::InvalidCardinality, -1, "pose requires seven values");
    const auto code = pose.integer(6, true);
    if (code < 0 || code > 13 || (code & 6) == 6)
      pose.fail(ErrorCode::InvalidValue, 6, "invalid pose representation code");
    bool finite = true;
    for (std::size_t i = 0; i < 6; ++i) finite = std::isfinite(pose.number(i)) && finite;
    try {
      if (finite) {
        if (const auto* owned = pose.owned_tuple()) candidate.initial_pose = mv::calibration::pose_from_halcon_tuple(*owned);
        else candidate.initial_pose = mv::calibration::pose_from_halcon_tuple(pose.materialize());
      }
      if (code == 0 || code == 1) {
        std::array<double, 6> reported;
        for (std::size_t i = 0; i < 6; ++i) {
          auto value = pose.number(i);
          if (i >= 3) {
            value = std::fmod((code == 1 ? -value : value), 360.);
            if (value < 0) value += 360.;
          }
          reported[i] = value;
        }
        // 一次建立标准混合位姿，避免六个实数逐次扩容后再提升存储。
        candidate.reported_pose = mv::core::Tuple{
            mv::core::Value{reported[0]}, mv::core::Value{reported[1]}, mv::core::Value{reported[2]},
            mv::core::Value{reported[3]}, mv::core::Value{reported[4]}, mv::core::Value{reported[5]},
            mv::core::Value{std::int64_t{0}}};
      } else if (finite) {
        const auto standard = mv::calibration::pose_to_halcon_tuple(*candidate.initial_pose);
        for (std::size_t i = 0; i < 6; ++i) {
          auto value = standard.as_real64(i);
          if (i >= 3) {
            value = std::fmod(value, 360.);
            if (value < 0) value += 360.;
          }
          candidate.reported_pose.append(value);
        }
        candidate.reported_pose.append(std::int64_t{0});
      } else {
        pose.fail(ErrorCode::UnsupportedValue, -1,
                  "nonfinite nonstandard pose conversion is not implemented");
      }
    } catch (const mv::core::Exception& e) {
      pose.fail(e.error_code(), -1, e.message());
    }
    // 存储允许少量及非有限观测；求解时独立验证几何。在可能分配的 map 提交后使状态失效。
    model.replace_observation({camera, object, view}, std::move(candidate));
    model.clear_calibration_result();
    model.clear_hand_eye_result();
  });
}

extern "C" ::MvStatus mv_get_calib_data_observ_points(::MvTuple CalibDataID, ::MvTuple CameraIdx,
                                                    ::MvTuple CalibObjIdx, ::MvTuple CalibObjPoseIdx,
                                                    ::MvTuple* Row, ::MvTuple* Column, ::MvTuple* Index,
                                                    ::MvTuple* Pose) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_get_calib_data_observ_points";
    mv::c_api::ControlOutputs<4> out(op, {Row, Column, Index, Pose});
    const TupleInput id(CalibDataID, op, "CalibDataID"), ci(CameraIdx, op, "CameraIdx"),
        oi(CalibObjIdx, op, "CalibObjIdx"), pi(CalibObjPoseIdx, op, "CalibObjPoseIdx");
    auto owner = data(id);
    const auto& model = *owner->handle()->value;
    const auto camera = index(ci, model.num_cameras()), object = index(oi, model.num_calibration_objects());
    const auto view = pi.scalar_integer();
    if (view < 0) pi.fail(ErrorCode::IndexOutOfRange, 0, "pose index must be nonnegative");
    if (!model.camera(camera).initial_parameters)
      ci.fail(ErrorCode::InvalidObjectState, 0, "camera is not configured");
    try {
      const auto& obs = model.observation({camera, object, view});
      out[0] = mv::core::Tuple{obs.rows};
      out[1] = mv::core::Tuple{obs.columns};
      out[2] = mv::core::Tuple{obs.indices};
      if (!obs.reported_pose.empty()) {
        out[3] = obs.reported_pose;
      } else if (obs.initial_pose) {
        auto values = mv::calibration::pose_to_halcon_tuple(*obs.initial_pose);
        for (std::size_t i = 0; i < values.size(); ++i) {
          if (i >= 3 && i < 6) {
            auto angle = std::fmod(values.at(i).as_real64(), 360.);
            if (angle < 0) angle += 360.;
            out[3].append(angle);
          } else
            out[3].append(values.at(i));
        }
      }
    } catch (const mv::core::Exception& e) {
      throw mv::core::Exception(e.status(), e.error_code(), op, "CalibObjPoseIdx", 0, e.message());
    }
    out.Commit();
  });
}

extern "C" ::MvStatus mv_find_calib_object(::MvObject Image, ::MvTuple CalibDataID, ::MvTuple CameraIdx,
                                         ::MvTuple CalibObjIdx, ::MvTuple CalibObjPoseIdx,
                                         ::MvTuple GenParamName, ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_find_calib_object";
    const auto& image = mv::c_api::require_single_image(Image, op, "Image");
    const TupleInput id(CalibDataID, op, "CalibDataID"), ci(CameraIdx, op, "CameraIdx"),
        oi(CalibObjIdx, op, "CalibObjIdx"), pi(CalibObjPoseIdx, op, "CalibObjPoseIdx"),
        names(GenParamName, op, "GenParamName"), values(GenParamValue, op, "GenParamValue");
    auto owner = data(id);
    auto& model = *owner->handle()->value;
    auto camera = index(ci, model.num_cameras()), object = index(oi, model.num_calibration_objects());
    auto view = pi.scalar_integer();
    if (view < 0) pi.fail(ErrorCode::IndexOutOfRange, 0, "pose index must be nonnegative");
    if (!model.camera(camera).initial_parameters)
      ci.fail(ErrorCode::InvalidObjectState, 0, "camera is not configured");
    if (names.size() != values.size())
      values.fail(ErrorCode::InvalidCardinality, -1, "generic parameter lengths must match");
    if (names.size())
      names.fail(ErrorCode::UnsupportedValue, 0, "detector generic parameters are not implemented");
    try {
      mv::calibration::find_calib_object(image, model, camera, object, view, nullptr);
    } catch (const mv::core::Exception& e) {
      throw mv::core::Exception(e.status(), e.error_code(), op, "Image", -1, e.message());
    }
  });
}

extern "C" ::MvStatus mv_get_calib_data_observ_contours(::MvObject* Contours, ::MvTuple CalibDataID,
                                                      ::MvTuple ContourName, ::MvTuple CameraIdx,
                                                      ::MvTuple CalibObjIdx,
                                                      ::MvTuple CalibObjPoseIdx) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_get_calib_data_observ_contours";
    mv::c_api::require_output(Contours, op, "Contours");
    const TupleInput id(CalibDataID, op, "CalibDataID"), names(ContourName, op, "ContourName");
    auto owner = data(id);
    const auto& model = *owner->handle()->value;
    auto name = text(names);
    if (name != "marks" && name != "marks_with_hole" && name != "caltab" && name != "last_caltab")
      names.fail(ErrorCode::InvalidValue, 0, "invalid contour name");
    if (name == "last_caltab") {
      // 手动观测没有检测轮廓；当前检测器不保存自动检测几何，不能从标记框虚构轮廓。
      if (model.has_automatic_detection())
        names.fail(ErrorCode::UnsupportedValue, 0, "last finder contours are not retained by the detector");
      *Contours = mv::c_api::make_xld_contours(mv::core::XldContours{});
      return;
    }
    const TupleInput ci(CameraIdx, op, "CameraIdx"), oi(CalibObjIdx, op, "CalibObjIdx"),
        pi(CalibObjPoseIdx, op, "CalibObjPoseIdx");
    auto camera = index(ci, model.num_cameras()), object = index(oi, model.num_calibration_objects());
    auto view = pi.scalar_integer();
    if (view < 0) pi.fail(ErrorCode::IndexOutOfRange, 0, "pose index must be nonnegative");
    if (!model.camera(camera).initial_parameters)
      ci.fail(ErrorCode::InvalidObjectState, 0, "camera is not configured");
    try {
      const auto& observation = model.observation({camera, object, view});
      if (name != "marks" && !observation.mark_contours.empty())
        names.fail(ErrorCode::UnsupportedValue, 0, "finder/hole results are not retained by the detector");
      *Contours =
          mv::c_api::make_xld_contours(name == "marks" ? observation.mark_contours : mv::core::XldContours{});
    } catch (const mv::core::Exception& e) {
      throw mv::core::Exception(e.status(), e.error_code(), op, "CalibObjPoseIdx", 0, e.message());
    }
  });
}

extern "C" ::MvStatus mv_create_calib_data(::MvTuple CalibSetup, ::MvTuple NumCameras,
                                         ::MvTuple NumCalibObjects, ::MvTuple* CalibDataID) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_create_calib_data";
    mv::c_api::ControlOutputs<1> output(op, {CalibDataID});
    const TupleInput setup(CalibSetup, op, "CalibSetup"), cameras(NumCameras, op, "NumCameras"),
        objects(NumCalibObjects, op, "NumCalibObjects");
    const auto name = text(setup);
    const auto nc = cameras.scalar_integer(), no = objects.scalar_integer();
    if (nc < 0) cameras.fail(ErrorCode::InvalidValue, 0, "camera count must be nonnegative");
    if (no < 0) objects.fail(ErrorCode::InvalidValue, 0, "object count must be nonnegative");
    if (name == "calibration_object") {
      if (nc == 0)
        cameras.fail(ErrorCode::InvalidValue, 0, "camera calibration requires at least one camera");
      if (no == 0)
        objects.fail(ErrorCode::InvalidValue, 0, "camera calibration requires at least one object");
    }
    auto handle = std::make_unique<MvCalibDataImpl>();
    handle->value = std::make_unique<mv::calibration::CalibrationDataModel>(
        mv::calibration::CalibrationDataModel::create(name, nc, no));
    output[0].append(mv::core::Handle::make<mv::c_api::CalibrationDataObject>(handle.get()));
    mv::c_api::release_opaque(handle.release());
    output.Commit();
  });
}

extern "C" ::MvStatus mv_set_calib_data_cam_param(::MvTuple CalibDataID, ::MvTuple CameraIdx,
                                                ::MvTuple CameraType, ::MvTuple CameraParam) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_calib_data_cam_param";
    const TupleInput id(CalibDataID, op, "CalibDataID"), slot(CameraIdx, op, "CameraIdx"),
        type(CameraType, op, "CameraType"), parameters(CameraParam, op, "CameraParam");
    auto owner = data(id);
    auto& model = *owner->handle()->value;
    const auto camera = index(slot, model.num_cameras());
    if (parameters.size() == 0)
      parameters.fail(ErrorCode::InvalidCardinality, -1, "camera parameters must not be empty");
    if (type.size() != 0 && text(type) != parameters.string(0))
      type.fail(ErrorCode::InvalidValue, 0, "camera type must match the camera parameter tuple");
    mv::core::Tuple materialized;
    const auto* value = parameters.owned_tuple();
    if (!value) {
      materialized = parameters.materialize();
      value = &materialized;
    }
    if (model.setup_kind() != mv::calibration::SetupKind::CalibrationObject &&
        !mv::calibration::is_perspective(mv::calibration::parse_camera_parameters(*value)))
      parameters.fail(ErrorCode::InvalidValue, 0, "hand-eye setups require a perspective camera");
    // 拥有型元组仅借用到解析完成；核心 setter 构造完整候选后才提交。
    mv::calibration::set_camera_parameters(model, camera, *value);
  });
}

extern "C" ::MvStatus mv_set_calib_data_calib_object(::MvTuple CalibDataID, ::MvTuple CalibObjIdx,
                                                   ::MvTuple CalibObjDescr) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_calib_data_calib_object";
    const TupleInput id(CalibDataID, op, "CalibDataID"), slot(CalibObjIdx, op, "CalibObjIdx"),
        description(CalibObjDescr, op, "CalibObjDescr");
    auto owner = data(id);
    auto& model = *owner->handle()->value;
    const auto target = index(slot, model.num_calibration_objects());
    if (description.size() == 1 && description.at(0).kind() == mv::core::ValueKind::String) {
      // 文件解析先构造完整候选，再以不抛异常的移动提交。
      mv::calibration::set_calplate_target(model, target, text(description));
    } else {
      if (description.size() % 3 != 0)
        description.fail(ErrorCode::InvalidCardinality, -1,
                         "packed X/Y/Z coordinates must have equal length");
      const auto count = description.size() / 3;
      mv::calibration::CalibrationTarget candidate;
      candidate.kind = mv::calibration::TargetKind::CustomPoints;
      candidate.points.reserve(count);
      for (std::size_t i = 0; i < count; ++i) {
        candidate.points.push_back(
            {description.number(i), description.number(i + count), description.number(i + 2 * count)});
      }
      // HALCON 可存储空或非有限数值坐标；拟合独立执行，在优化前拒绝不可用几何。
      model.clear_calibration_result();
      model.target(target) = std::move(candidate);
    }
  });
}

extern "C" ::MvStatus mv_set_calib_data(::MvTuple CalibDataID, ::MvTuple ItemType, ::MvTuple ItemIdx,
                                      ::MvTuple DataName, ::MvTuple DataValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_set_calib_data";
    const TupleInput id(CalibDataID, op, "CalibDataID"), type(ItemType, op, "ItemType"),
        slot(ItemIdx, op, "ItemIdx"), name(DataName, op, "DataName"), value(DataValue, op, "DataValue");
    auto owner = data(id);
    const auto kind = text(type), attribute = text(name);
    const auto indices = item_index(slot, kind);
    if (kind == "model" && attribute == "optimization_method" &&
        owner->handle()->value->setup_kind() == mv::calibration::SetupKind::CalibrationObject)
      name.fail(ErrorCode::InvalidValue, 0, "optimization_method is a hand-eye setting");
    if (kind == "camera" && attribute == "params")
      name.fail(ErrorCode::UnsupportedValue, 0, "use set_calib_data_cam_param to configure a camera");
    // 每个核心 setter 在修改有效状态前构造候选；模型身份唯一，无需复制全部观测。
    try {
      if (const auto* owned = value.owned_tuple())
        mv::calibration::set_calib_data(*owner->handle()->value, kind, indices, attribute, *owned);
      else
        mv::calibration::set_calib_data(*owner->handle()->value, kind, indices, attribute, value.materialize());
    } catch (const mv::core::Exception& error) {
      throw mv::core::Exception(error.status(), error.error_code(), op,
                                error.status() == mv::core::Status::OutOfRange ? "ItemIdx" : "DataValue", -1,
                                error.message());
    }
  });
}

extern "C" ::MvStatus mv_get_calib_data(::MvTuple CalibDataID, ::MvTuple ItemType, ::MvTuple ItemIdx,
                                      ::MvTuple DataName, ::MvTuple* DataValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_get_calib_data";
    mv::c_api::ControlOutputs<1> output(op, {DataValue});
    const TupleInput id(CalibDataID, op, "CalibDataID"), type(ItemType, op, "ItemType"),
        slot(ItemIdx, op, "ItemIdx"), names(DataName, op, "DataName");
    auto owner = data(id);
    const auto kind = text(type);
    const auto indices = item_index(slot, kind);
    const auto& model = *owner->handle()->value;
    if (kind != "model" && names.size() != 1)
      names.fail(ErrorCode::InvalidCardinality, -1, "this item type requires exactly one data name");
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto attribute = names.string(i);
      if (attribute.find('\0') != std::string_view::npos)
        names.fail(ErrorCode::InvalidValue, i, "embedded NUL is not supported");
      try {
        if (kind == "model" && attribute == "optimization_method" &&
            model.setup_kind() == mv::calibration::SetupKind::CalibrationObject)
          names.fail(ErrorCode::InvalidValue, i, "optimization_method is a hand-eye setting");
        if (kind == "camera" && attribute == "params" && !model.calibration_result())
          names.fail(ErrorCode::InvalidObjectState, i, "calibrated camera parameters are not available");
        auto values = mv::calibration::get_calib_data(model, kind, indices, attribute);
        if(names.size()==1)output[0]=std::move(values);
        else for (std::size_t j = 0; j < values.size(); ++j) output[0].append(values.at(j));
      } catch (const mv::core::Exception& error) {
        throw mv::core::Exception(error.status(), error.error_code(), op,
                                  error.status() == mv::core::Status::OutOfRange ? "ItemIdx" : "DataName", i,
                                  error.message());
      }
    }
    output.Commit();
  });
}

namespace {
[[noreturn]] void forward_error(const mv::core::Exception& e, const char* op, const char* parameter) {
  throw mv::core::Exception(e.status(), e.error_code(), op, parameter, -1, e.message());
}
}  // namespace
extern "C" ::MvStatus mv_calibrate_cameras(::MvTuple CalibDataID, MvTuple* Error) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_calibrate_cameras";
    mv::c_api::ControlOutputs<1> out(op, {Error});
    const TupleInput id(CalibDataID, op, "CalibDataID");
    auto owner = data(id);
    auto candidate = std::make_unique<mv::calibration::CalibrationDataModel>(owner->handle()->value->clone());
    double error = 0;
    try {
      mv::calibration::calibrate_cameras(*candidate, &error);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "CalibDataID");
    }
    out[0].append(error);
    owner->handle()->value = std::move(candidate);
    out.Commit();
  });
}
extern "C" ::MvStatus mv_calibrate_hand_eye(::MvTuple CalibDataID, MvTuple* Errors) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_calibrate_hand_eye";
    mv::c_api::ControlOutputs<1> out(op, {Errors});
    const TupleInput id(CalibDataID, op, "CalibDataID");
    auto owner = data(id);
    auto candidate = std::make_unique<mv::calibration::CalibrationDataModel>(owner->handle()->value->clone());
    std::array<double, 4> errors{};
    try {
      mv::calibration::calibrate_hand_eye(*candidate, candidate->optimization_method(), &errors);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "CalibDataID");
    }
    // 私有求解器使用弧度，HALCON 公开旋转误差使用度。
    constexpr double degrees = 180. / 3.14159265358979323846;
    errors[1] *= degrees;
    errors[3] *= degrees;
    for (auto v : errors) out[0].append(v);
    owner->handle()->value = std::move(candidate);
    out.Commit();
  });
}
extern "C" ::MvStatus mv_remove_calib_data_observ(::MvTuple CalibDataID, ::MvTuple CameraIdx,
                                                ::MvTuple CalibObjIdx, ::MvTuple CalibObjPoseIdx) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_remove_calib_data_observ";
    const TupleInput id(CalibDataID, op, "CalibDataID"), ci(CameraIdx, op, "CameraIdx"),
        oi(CalibObjIdx, op, "CalibObjIdx"), pi(CalibObjPoseIdx, op, "CalibObjPoseIdx");
    auto owner = data(id);
    auto& model = *owner->handle()->value;
    auto camera = index(ci, model.num_cameras()), object = index(oi, model.num_calibration_objects()),
         view = pi.scalar_integer();
    if (view < 0) pi.fail(ErrorCode::IndexOutOfRange, 0, "pose index must be nonnegative");
    try {
      model.remove_observation({camera, object, view});
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "CalibObjPoseIdx");
    }
  });
}
extern "C" ::MvStatus mv_read_cam_par(::MvTuple CamParFile, MvTuple* CameraParam) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_read_cam_par";
    mv::c_api::ControlOutputs<1> out(op, {CameraParam});
    const TupleInput path(CamParFile, op, "CamParFile");
    auto file = text(path);
    try {
      out[0] = mv::calibration::read_cam_par(file);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "CamParFile");
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_write_cam_par(::MvTuple CameraParam, ::MvTuple CamParFile) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_write_cam_par";
    const TupleInput value(CameraParam, op, "CameraParam"), path(CamParFile, op, "CamParFile");
    auto file = text(path);
    auto parameters = value.materialize();
    try {
      mv::calibration::write_cam_par(file, parameters);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, e.status() == mv::core::Status::IoError ? "CamParFile" : "CameraParam");
    }
  });
}
extern "C" ::MvStatus mv_read_pose(::MvTuple PoseFile, MvTuple* Pose) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_read_pose";
    mv::c_api::ControlOutputs<1> out(op, {Pose});
    const TupleInput path(PoseFile, op, "PoseFile");
    auto file = text(path);
    try {
      out[0] = mv::calibration::read_pose(file);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "PoseFile");
    }
    out.Commit();
  });
}
extern "C" ::MvStatus mv_write_pose(::MvTuple Pose, ::MvTuple PoseFile) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_write_pose";
    const TupleInput value(Pose, op, "Pose"), path(PoseFile, op, "PoseFile");
    auto file = text(path);
    auto pose = value.materialize();
    try {
      mv::calibration::write_pose(file, pose);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, e.status() == mv::core::Status::IoError ? "PoseFile" : "Pose");
    }
  });
}
extern "C" ::MvStatus mv_write_calib_data(::MvTuple CalibDataID, ::MvTuple FileName) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_write_calib_data";
    const TupleInput id(CalibDataID, op, "CalibDataID"), path(FileName, op, "FileName");
    auto owner = data(id);
    auto file = text(path);
    try {
      mv::calibration::write_calib_data(file, *owner->handle()->value);
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "FileName");
    }
  });
}
extern "C" ::MvStatus mv_read_calib_data(::MvTuple FileName, MvTuple* CalibDataID) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_read_calib_data";
    mv::c_api::ControlOutputs<1> out(op, {CalibDataID});
    const TupleInput path(FileName, op, "FileName");
    auto file = text(path);
    auto handle = std::make_unique<MvCalibDataImpl>();
    try {
      handle->value =
          std::make_unique<mv::calibration::CalibrationDataModel>(mv::calibration::read_calib_data(file));
    } catch (const mv::core::Exception& e) {
      forward_error(e, op, "FileName");
    }
    out[0].append(mv::core::Handle::make<mv::c_api::CalibrationDataObject>(handle.get()));
    mv::c_api::release_opaque(handle.release());
    out.Commit();
  });
}

extern "C" ::MvStatus mv_project_3d_point(::MvTuple X, ::MvTuple Y, ::MvTuple Z,
                                        ::MvTuple CameraParam, ::MvTuple* Row, ::MvTuple* Column) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_project_3d_point";
    mv::c_api::ControlOutputs<2> output(op, {Row, Column});
    const TupleInput x(X, op, "X"), y(Y, op, "Y"), z(Z, op, "Z");
    const mv::c_api::ControlTupleView cp(CameraParam, op, "CameraParam");
    const auto camera = mv::calibration::parse_camera_parameters(cp);
    const auto zero_polynomial = std::visit(
        [](const auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, mv::calibration::AreaScanPolynomialParameters> ||
                        std::is_same_v<T, mv::calibration::AreaScanTelecentricPolynomialParameters>)
            return p.k1 == 0. && p.k2 == 0. && p.k3 == 0. && p.p1 == 0. && p.p2 == 0.;
          else
            return false;
        },
        camera);
    const auto direct_projection = std::visit(
        [&](const auto& p) {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, mv::calibration::AreaScanDivisionParameters> ||
                        std::is_same_v<T, mv::calibration::AreaScanTelecentricDivisionParameters>)
            return p.kappa == 0.;
          else
            return zero_polynomial;
        },
        camera);
    if (y.size() != x.size()) y.fail(ErrorCode::InvalidCardinality, -1, "coordinate lengths must match");
    if (z.size() != x.size()) z.fail(ErrorCode::InvalidCardinality, -1, "coordinate lengths must match");
    std::vector<double> rows(x.size()), cols(x.size());
    std::size_t i=0;
#if MVKIT_CALIBRATION_CONTROL_SSE2
    const auto* xt=x.owned_tuple();const auto* yt=y.owned_tuple();const auto* zt=z.owned_tuple();
    const auto& cpu=mv::runtime::cpu_features();
    if(direct_projection && xt && yt && zt && xt->kind()==mv::core::TupleKind::Real64 &&
       yt->kind()==mv::core::TupleKind::Real64 && zt->kind()==mv::core::TupleKind::Real64 &&
       mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),
                                   {true,false,false,false})) {
      const auto xs=xt->real64_span(),ys=yt->real64_span(),zs=zt->real64_span();
      const auto finite=[](__m128d value) {
        return _mm_cmple_pd(_mm_andnot_pd(_mm_set1_pd(-0.),value),_mm_set1_pd(std::numeric_limits<double>::max()));
      };
      std::visit([&](const auto& p) {
        using T=std::decay_t<decltype(p)>;
        constexpr bool perspective=std::is_same_v<T,mv::calibration::AreaScanDivisionParameters> ||
                                    std::is_same_v<T,mv::calibration::AreaScanPolynomialParameters>;
        const double factor=[&]{if constexpr(perspective)return p.focus;else return p.magnification;}();
        const auto scale=_mm_set1_pd(factor),sx=_mm_set1_pd(p.sx),sy=_mm_set1_pd(p.sy);
        const auto cx=_mm_set1_pd(p.cx),cy=_mm_set1_pd(p.cy);
        for(;x.size()-i>=2;i+=2) {
          const auto px=_mm_loadu_pd(xs.data()+i),py=_mm_loadu_pd(ys.data()+i),pz=_mm_loadu_pd(zs.data()+i);
          auto valid=finite(pz);
          if(!zero_polynomial)valid=_mm_and_pd(valid,_mm_and_pd(finite(px),finite(py)));
          if constexpr(perspective)valid=_mm_and_pd(valid,_mm_cmpneq_pd(pz,_mm_setzero_pd()));
          if(_mm_movemask_pd(valid)!=3)break; // 可疑点按原逐点顺序报告参数和索引。
          const auto u=_mm_mul_pd(px,scale),v=_mm_mul_pd(py,scale);
          if constexpr(perspective) {
            _mm_storeu_pd(rows.data()+i,_mm_add_pd(_mm_div_pd(v,_mm_mul_pd(pz,sy)),cy));
            _mm_storeu_pd(cols.data()+i,_mm_add_pd(_mm_div_pd(u,_mm_mul_pd(pz,sx)),cx));
          } else {
            _mm_storeu_pd(rows.data()+i,_mm_add_pd(_mm_div_pd(v,sy),cy));
            _mm_storeu_pd(cols.data()+i,_mm_add_pd(_mm_div_pd(u,sx),cx));
          }
        }
      },camera);
    }
#endif
    for (; i < x.size(); ++i) {
      const auto px = x.number(i), py = y.number(i), pz = z.number(i);
      if (!zero_polynomial && !std::isfinite(px))
        x.fail(ErrorCode::InvalidValue, i, "finite coordinate required");
      if (!zero_polynomial && !std::isfinite(py))
        y.fail(ErrorCode::InvalidValue, i, "finite coordinate required");
      if (!std::isfinite(pz)) z.fail(ErrorCode::InvalidValue, i, "finite coordinate required");
      mv::Point2d direct;
      const auto metric = std::visit(
          [&](const auto& p) -> mv::calibration::MetricPoint {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, mv::calibration::AreaScanDivisionParameters> ||
                          std::is_same_v<T, mv::calibration::AreaScanPolynomialParameters>) {
              if (pz == 0.) z.fail(ErrorCode::InvalidValue, i, "perspective projection requires nonzero Z");
              if (direct_projection)
                direct = {py * p.focus / (pz * p.sy) + p.cy, px * p.focus / (pz * p.sx) + p.cx};
              return {p.focus * px / pz, p.focus * py / pz};
            } else {
              if (direct_projection)
                direct = {p.magnification * py / p.sy + p.cy, p.magnification * px / p.sx + p.cx};
              return {p.magnification * px, p.magnification * py};
            }
          },
          camera);
      const auto point =
          direct_projection ? direct : mv::calibration::undistorted_metric_to_image_point(camera, metric);
      rows[i] = point.row;
      cols[i] = point.col;
    }
    output[0] = mv::core::Tuple{std::move(rows)};
    output[1] = mv::core::Tuple{std::move(cols)};
    output.Commit();
  });
}

extern "C" ::MvStatus mv_change_radial_distortion_points(::MvTuple Row, ::MvTuple Col,
                                                       ::MvTuple CamParamIn, ::MvTuple CamParamOut,
                                                       ::MvTuple* RowChanged, ::MvTuple* ColChanged) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_change_radial_distortion_points";
    mv::c_api::ControlOutputs<2> output(op, {RowChanged, ColChanged});
    const TupleInput rows(Row, op, "Row"), cols(Col, op, "Col");
    if (rows.size() != cols.size())
      cols.fail(ErrorCode::InvalidCardinality, -1, "row/column lengths must match");
    const mv::c_api::ControlTupleView r(Row, op, "Row"), c(Col, op, "Col"), in(CamParamIn, op, "CamParamIn"),
        out(CamParamOut, op, "CamParamOut");
    bool nonfinite = false;
    for (std::size_t i = 0; i < rows.size(); ++i)
      nonfinite |= !std::isfinite(rows.number(i)) || !std::isfinite(cols.number(i));
    if (!nonfinite)
      mv::calibration::change_radial_distortion_points(r, c, in, out, &output[0], &output[1]);
    else {
      const auto camera_in = mv::calibration::parse_camera_parameters(in);
      const auto camera_out = mv::calibration::parse_camera_parameters(out);
      std::vector<double> rr(rows.size()), cc(rows.size());
      for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto row = rows.number(i), col = cols.number(i);
        if (!std::isfinite(row) || !std::isfinite(col)) {
          rr[i] = cc[i] = std::numeric_limits<double>::quiet_NaN();
          continue;
        }
        const auto metric = mv::calibration::image_point_to_undistorted_metric(camera_in, {row, col});
        const auto point = mv::calibration::undistorted_metric_to_image_point(camera_out, metric);
        rr[i] = point.row;
        cc[i] = point.col;
      }
      output[0] = mv::core::Tuple{std::move(rr)};
      output[1] = mv::core::Tuple{std::move(cc)};
    }
    output.Commit();
  });
}
extern "C" ::MvStatus mv_image_points_to_world_plane(::MvTuple CameraParam, ::MvTuple WorldPose,
                                                   ::MvTuple Rows, ::MvTuple Cols, ::MvTuple Scale,
                                                   ::MvTuple* X, ::MvTuple* Y) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_image_points_to_world_plane";
    mv::c_api::ControlOutputs<2> output(op, {X, Y});
    const mv::c_api::ControlTupleView cp(CameraParam, op, "CameraParam"), pose(WorldPose, op, "WorldPose"),
        scale(Scale, op, "Scale");
    const auto project = mv::calibration::WorldPlaneProjection::prepare(cp, pose, scale, true);
    const TupleInput rows(Rows, op, "Rows"), cols(Cols, op, "Cols");
    if (rows.size() != cols.size())
      cols.fail(ErrorCode::InvalidCardinality, -1, "row/column lengths must match");
    const mv::c_api::ControlTupleView r(Rows, op, "Rows"), c(Cols, op, "Cols");
    project.map_tuples(r, c, &output[0], &output[1]);
    output.Commit();
  });
}

extern "C" ::MvStatus mv_change_radial_distortion_cam_par(::MvTuple Mode, ::MvTuple CamParamIn,
                                                        ::MvTuple DistortionCoeffs,
                                                        ::MvTuple* CamParamOut) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_change_radial_distortion_cam_par";
    mv::c_api::ControlOutputs<1> output(op, {CamParamOut});
    const TupleInput mode(Mode, op, "Mode"), coeff(DistortionCoeffs, op, "DistortionCoeffs");
    const auto name = text(mode);
    if (name != "fixed" && name != "adaptive" && name != "fullsize" && name != "preserve_resolution")
      mode.fail(ErrorCode::InvalidValue, 0, "unknown rectification mode");
    const mv::c_api::ControlTupleView cp(CamParamIn, op, "CamParamIn");
    const auto camera = mv::calibration::parse_camera_parameters(cp);
    const auto polynomial =
        std::holds_alternative<mv::calibration::AreaScanPolynomialParameters>(camera) ||
        std::holds_alternative<mv::calibration::AreaScanTelecentricPolynomialParameters>(camera);
    const std::size_t count = polynomial ? 5 : 1;
    if (coeff.size() != count)
      coeff.fail(ErrorCode::InvalidCardinality, -1, "wrong number of distortion coefficients");
    const mv::core::Tuple& input = cp;
    mv::core::Tuple result;
    bool unchanged = true;
    for (std::size_t i = 0; i < input.size(); ++i) {
      if (i >= 2 && i < 2 + count) {
        const auto value = coeff.number(i - 2);
        unchanged &= value == input.as_real64(i);
        result.append(value);
      } else
        result.append(input.at(i));
    }
    if (name != "fixed" && !unchanged)
      throw mv::core::Exception(
          mv::core::Status::Unsupported, ErrorCode::UnsupportedValue, op, "Mode", 0,
          "rectification extent optimization is not implemented for changed coefficients");
    output[0] = std::move(result);
    output.Commit();
  });
}
extern "C" ::MvStatus mv_vector_to_hom_mat2d(::MvTuple Px, ::MvTuple Py, ::MvTuple Qx, ::MvTuple Qy,
                                           ::MvTuple* HomMat2D) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_vector_to_hom_mat2d";
    mv::c_api::ControlOutputs<1> output(op, {HomMat2D});
    const TupleInput x(Px, op, "Px"), y(Py, op, "Py"), u(Qx, op, "Qx"), v(Qy, op, "Qy");
    const auto size=x.size();
    if (size < 3) x.fail(ErrorCode::InvalidCardinality, -1, "at least three point pairs required");
    for (const auto* c : {&y, &u, &v})
      if (c->size() != size) c->fail(ErrorCode::InvalidCardinality, -1, "coordinate lengths must match");
    const mv::c_api::ControlTupleView xx(Px, op, "Px"), yy(Py, op, "Py"), uu(Qx, op, "Qx"), vv(Qy, op, "Qy");
    bool bad_u = false, bad_v = false;
    const auto* owned_u=u.owned_tuple();const auto* owned_v=v.owned_tuple();
    const auto numeric=[](const mv::core::Tuple* value) {
      return value&&(value->kind()==mv::core::TupleKind::Real64||value->kind()==mv::core::TupleKind::Int64);
    };
    if(numeric(owned_u)&&numeric(owned_v)) {
      // 拥有型同质数组只读借用；Int64 转 Double 总是有限，Mixed/外部值保留原逐项诊断。
      const auto real_u=owned_u->kind()==mv::core::TupleKind::Real64?owned_u->real64_span():std::span<const double>{};
      const auto real_v=owned_v->kind()==mv::core::TupleKind::Real64?owned_v->real64_span():std::span<const double>{};
      for(std::size_t i=0;i<size;++i) {
        if(!real_u.empty())bad_u|=!std::isfinite(real_u[i]);
        if(!real_v.empty())bad_v|=!std::isfinite(real_v[i]);
      }
    } else {
      for (std::size_t i = 0; i < size; ++i) {
        bad_u |= !std::isfinite(u.number(i));
        bad_v |= !std::isfinite(v.number(i));
      }
    }
    if (!bad_u && !bad_v)
      mv::calibration::vector_to_hom_mat2d(xx, yy, uu, vv, &output[0], true);
    else {
      const mv::core::Tuple zero(std::vector<double>(size, 0.));
      const mv::core::Tuple& safe_u = bad_u ? zero : static_cast<const mv::core::Tuple&>(uu);
      const mv::core::Tuple& safe_v = bad_v ? zero : static_cast<const mv::core::Tuple&>(vv);
      mv::core::Tuple finite;
      mv::calibration::vector_to_hom_mat2d(xx, yy, safe_u, safe_v, &finite, true);
      for (std::size_t i = 0; i < 6; ++i)
        output[0].append(((i < 3 && bad_u) || (i >= 3 && bad_v)) ? std::numeric_limits<double>::quiet_NaN()
                                                                 : finite.as_real64(i));
    }
    output.Commit();
  });
}
