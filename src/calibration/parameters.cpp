#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/parameters.hpp>
#include <cmath>
#include <array>
#include <core/exception.hpp>
#include <set>
#include <string>
#include <vector>

namespace mv::calibration {
namespace {
std::int64_t single_index(const core::Tuple& tuple, std::string_view where) {
  if (tuple.size() != 1)
    throw core::Exception(core::Status::InvalidArgument, where,
                          "item index must contain exactly one integer");
  try {
    return tuple.at(0).as_int64();
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, where, "item index must be integer");
  }
}

bool general_index(const core::Tuple& tuple) {
  if(tuple.size()!=1)return false;
  const auto kind=tuple.kind();
  if(kind!=core::TupleKind::String &&
      (kind!=core::TupleKind::Mixed || tuple.at(0).kind()!=core::ValueKind::String))return false;
  // 数值索引是正常输入，不通过抛出并捕获类型异常来判断字符串哨兵。
  return tuple.string_view(0)=="general";
}

std::string single_string(const core::Tuple& tuple, std::string_view where) {
  if (tuple.size() != 1)
    throw core::Exception(core::Status::InvalidArgument, where, "expected one string value");
  try {
    return std::string(tuple.string_view(0));
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, where, "expected string value");
  }
}

double single_real(const core::Tuple& tuple, std::string_view where) {
  if (tuple.size() != 1)
    throw core::Exception(core::Status::InvalidArgument, where, "expected one numeric value");
  try {
    const double value = tuple.at(0).as_real64();
    if (!std::isfinite(value))
      throw core::Exception(core::Status::InvalidArgument, where, "numeric value must be finite");
    return value;
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::InvalidArgument) throw;
    throw core::Exception(core::Status::InvalidArgument, where, "expected numeric value");
  }
}

std::int64_t single_int(const core::Tuple& tuple, std::string_view where) {
  if (tuple.size() != 1)
    throw core::Exception(core::Status::InvalidArgument, where, "expected one integer value");
  try {
    return tuple.at(0).as_int64();
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, where, "expected integer value");
  }
}

const std::set<std::string>& allowed_settings(const CameraParameters& params) {
  // 合法名称仅由相机类型决定；四个不可变集合初始化一次，避免每次配置重建树节点。
  static const auto settings=[] {
    std::array<std::set<std::string>,4> result;
    for(std::size_t i=0;i<result.size();++i) {
      result[i]={"transx","transy","transz","alpha","beta","gamma","sx","sy","cx","cy"};
      result[i].insert(i<2?"focus":"magnification");
      if(i==0 || i==2)result[i].insert("kappa");
      else for(const auto* name:{"k1","k2","k3","p1","p2"})result[i].insert(name);
    }
    return result;
  }();
  const auto index=std::holds_alternative<AreaScanDivisionParameters>(params)?0u:
      std::holds_alternative<AreaScanPolynomialParameters>(params)?1u:
      std::holds_alternative<AreaScanTelecentricDivisionParameters>(params)?2u:3u;
  return settings[index];
}

std::set<std::string> parse_excluded(const core::Tuple& value, const CameraParameters& params) {
  const auto& allowed = allowed_settings(params);
  std::set<std::string> result;
  for (std::size_t i = 0; i < value.size(); ++i) {
    std::string name;
    try {
      name = std::string(value.string_view(i));
    } catch (const core::Exception&) {
      throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                            "excluded_settings must contain strings");
    }
    if (name == "all") {
      result.insert(allowed.begin(), allowed.end());
      continue;
    }
    if (name == "pose") {
      for (const auto* pose : {"transx", "transy", "transz", "alpha", "beta", "gamma"}) result.insert(pose);
      continue;
    }
    if (name == "params") {
      for (const auto& parameter : allowed)
        if (parameter != "transx" && parameter != "transy" && parameter != "transz" && parameter != "alpha" &&
            parameter != "beta" && parameter != "gamma")
          result.insert(parameter);
      continue;
    }
    if (name == "principal_point") {
      result.insert("cx");
      result.insert("cy");
      continue;
    }
    if (name == "poly" || name == "poly_tan_2") {
      for (const auto* parameter : {"k1", "k2", "k3", "p1", "p2"})
        if (allowed.count(parameter) && (name == "poly" || parameter[0] == 'p')) result.insert(parameter);
      continue;
    }
    if (!allowed.count(name)) {
      throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                            "excluded camera setting is not valid for this camera model");
    }
    result.insert(std::move(name));
  }
  return result;
}
core::Tuple camera_settings(const CameraSlot& camera, bool excluded) {
  if (!camera.parameters)
    throw core::Exception(core::Status::InvalidState, "get_calib_data",
                          "camera parameters are not configured");
  const auto& allowed = allowed_settings(*camera.parameters);
  core::Tuple result;
  for (const auto* name : {"transx", "transy", "transz", "alpha", "beta", "gamma", "focus", "magnification",
                           "kappa", "k1", "k2", "k3", "p1", "p2", "sx", "sy", "cx", "cy"})
    if (allowed.count(name) && (camera.excluded_settings.count(name) != 0) == excluded) result.append(name);
  return result;
}
}  // namespace

void set_calib_data(CalibrationDataModel& data, std::string_view item_type, const core::Tuple& item_index,
                    std::string_view data_name, const core::Tuple& data_value) {
  if (item_type == "camera") {
    if (data_name == "params") {
      set_camera_parameters(data, single_index(item_index, "set_calib_data"), data_value);
      return;
    }
    if (data_name == "excluded_settings" || data_name == "calib_settings") {
      std::vector<std::pair<std::int64_t, std::set<std::string>>> candidates;
      const bool general = general_index(item_index);
      const auto first = general ? 0 : single_index(item_index, "set_calib_data");
      if (!general) (void)data.camera(first);  // 计算 first+1 前检查边界。
      const auto last = general ? static_cast<std::int64_t>(data.num_cameras()) : first + 1;
      if(!general) {
        auto& camera=data.camera(first);
        if(!camera.parameters)
          throw core::Exception(core::Status::InvalidState,"set_calib_data","camera parameters are not configured");
        auto settings=parse_excluded(data_value,*camera.parameters);
        // 名称全部验证后转移已分配节点；固定 string 比较器不抛异常，提交不再分配。
        if(data_name=="excluded_settings")camera.excluded_settings.merge(settings);
        else for(const auto& setting:settings)camera.excluded_settings.erase(setting);
        data.clear_calibration_result();
        return;
      }
      for (auto i = first; i < last; ++i) {
        const auto& camera = data.camera(i);
        if (!camera.parameters)
          throw core::Exception(core::Status::InvalidState, "set_calib_data",
                                "camera parameters are not configured");
        const auto settings = parse_excluded(data_value, *camera.parameters);
        auto candidate = camera.excluded_settings;
        for (const auto& setting : settings) {
          if (data_name == "excluded_settings")
            candidate.insert(setting);
          else
            candidate.erase(setting);
        }
        candidates.emplace_back(i, std::move(candidate));
      }
      data.clear_calibration_result();
      for (auto& [i, settings] : candidates) data.camera(i).excluded_settings = std::move(settings);
      return;
    }
    throw core::Exception(core::Status::Unsupported, "set_calib_data", "unsupported camera data name");
  }

  if (item_type == "tool") {
    const auto index = single_index(item_index, "set_calib_data");
    if (data_name != "tool_in_base_pose") {
      throw core::Exception(core::Status::Unsupported, "set_calib_data", "unsupported tool data name");
    }
    const auto candidate = pose_from_halcon_tuple(data_value);
    data.set_tool_in_base_pose(index, candidate);
    return;
  }

  if (item_type == "model") {
    if (!general_index(item_index))
      throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                            "model item index must be 'general'");
    auto next = data.options();
    if (data_name == "max_iterations") {
      const auto v = single_int(data_value, "set_calib_data");
      if (v <= 0)
        throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                              "max_iterations must be positive");
      next.max_iterations = static_cast<std::size_t>(v);
    } else if (data_name == "function_tolerance") {
      const auto v = single_real(data_value, "set_calib_data");
      if (v < 0.0)
        throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                              "function_tolerance must be non-negative");
      next.function_tolerance = v;
    } else if (data_name == "gradient_tolerance") {
      const auto v = single_real(data_value, "set_calib_data");
      if (v < 0.0)
        throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                              "gradient_tolerance must be non-negative");
      next.gradient_tolerance = v;
    } else if (data_name == "parameter_tolerance") {
      const auto v = single_real(data_value, "set_calib_data");
      if (v < 0.0)
        throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                              "parameter_tolerance must be non-negative");
      next.parameter_tolerance = v;
    } else if (data_name == "initial_damping") {
      const auto v = single_real(data_value, "set_calib_data");
      if (v <= 0.0)
        throw core::Exception(core::Status::InvalidArgument, "set_calib_data",
                              "initial_damping must be positive");
      next.initial_damping = v;
    } else if (data_name == "optimization_method") {
      const auto method = single_string(data_value, "set_calib_data");
      if (data.setup_kind() == SetupKind::CalibrationObject) {
        if (method != "lm")
          throw core::Exception(core::Status::Unsupported, "set_calib_data",
                                "camera calibration v1 only supports lm");
      } else if (method != "linear" && method != "nonlinear") {
        throw core::Exception(core::Status::Unsupported, "set_calib_data",
                              "hand-eye v1 supports linear or nonlinear");
      }
      data.set_optimization_method(method);
      return;
    } else {
      throw core::Exception(core::Status::Unsupported, "set_calib_data", "unsupported model data name");
    }
    data.clear_calibration_result();
    data.options() = next;
    return;
  }
  throw core::Exception(core::Status::Unsupported, "set_calib_data", "unsupported item type");
}

core::Tuple get_calib_data(const CalibrationDataModel& data, std::string_view item_type,
                           const core::Tuple& item_index, std::string_view data_name) {
  if (item_type == "camera") {
    const auto index = single_index(item_index, "get_calib_data");
    const auto& camera = data.camera(index);
    if (data_name == "init_params") {
      if (!camera.initial_parameters)
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "initial camera parameters are not configured");
      return camera_parameters_to_tuple(*camera.initial_parameters);
    }
    if (data_name == "type") {
      if (!camera.initial_parameters)
        throw core::Exception(core::Status::InvalidState, "get_calib_data", "camera type is not configured");
      return camera_parameters_to_tuple(*camera.initial_parameters).select(core::Tuple{std::int64_t{0}});
    }
    if (data_name == "params") {
      if (!camera.parameters)
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera parameters are not configured");
      return camera_parameters_to_tuple(*camera.parameters);
    }
    if (data_name == "excluded_settings" || data_name == "calib_settings")
      return camera_settings(camera, data_name == "excluded_settings");
    if (data_name == "stddev" || data_name == "params_deviations") {
      if (!data.calibration_result())
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera calibration result is not available");
      if (!data.calibration_result()->covariance_available)
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera uncertainty is unavailable");
      core::Tuple result;
      for (double v : data.calibration_result()->camera_stddev) result.append(v);
      return result;
    }
    if (data_name == "covariance" || data_name == "params_covariances") {
      if (!data.calibration_result())
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera calibration result is not available");
      if (!data.calibration_result()->covariance_available)
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera covariance is unavailable");
      core::Tuple result;
      for (double v : data.calibration_result()->camera_covariance) result.append(v);
      return result;
    }
    if (data.hand_eye_result()) {
      if (data.setup_kind() == SetupKind::HandEyeMovingCamera && data_name == "camera_in_tool_pose")
        return pose_to_halcon_tuple(data.hand_eye_result()->first_pose);
      if (data.setup_kind() == SetupKind::HandEyeStationaryCamera && data_name == "base_in_camera_pose")
        return pose_to_halcon_tuple(data.hand_eye_result()->first_pose);
    }
    throw core::Exception(core::Status::Unsupported, "get_calib_data", "unsupported camera data name");
  }

  if (item_type == "tool") {
    const auto index = single_index(item_index, "get_calib_data");
    if (data_name != "tool_in_base_pose")
      throw core::Exception(core::Status::Unsupported, "get_calib_data", "unsupported tool data name");
    const auto it = data.tool_in_base_poses().find(index);
    if (it == data.tool_in_base_poses().end())
      throw core::Exception(core::Status::NotFound, "get_calib_data", "tool pose not found");
    return pose_to_halcon_tuple(it->second);
  }
  if (item_type == "calib_obj_pose") {
    if (item_index.size() != 2)
      throw core::Exception(core::Status::InvalidArgument, "get_calib_data",
                            "calib_obj_pose index must be {object,pose}");
    std::int64_t object_index{}, pose_index{};
    try {
      object_index = item_index.at(0).as_int64();
      pose_index = item_index.at(1).as_int64();
    } catch (const core::Exception&) {
      throw core::Exception(core::Status::InvalidArgument, "get_calib_data",
                            "calib_obj_pose indices must be integers");
    }
    const ObservationKey key{0, object_index, pose_index};
    if (data_name == "valid") {
      const auto& target = data.target(object_index);
      const auto it = data.observations().find(key);
      if (it == data.observations().end()) {
        throw core::Exception(core::Status::NotFound, "get_calib_data", "calibration observation not found");
      }
      std::vector<std::int64_t> flags(target.points.size(), 0);
      for (const auto index : it->second.indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < flags.size()) {
          flags[static_cast<std::size_t>(index)] = 1;
        }
      }
      core::Tuple result;
      for (auto flag : flags) result.append(flag);
      return result;
    }
    if (!data.calibration_result())
      throw core::Exception(core::Status::InvalidState, "get_calib_data",
                            "camera calibration result is not available");
    if (data_name == "pose") {
      const auto it = data.calibration_result()->poses.find(key);
      if (it == data.calibration_result()->poses.end())
        throw core::Exception(core::Status::NotFound, "get_calib_data", "calibration pose result not found");
      return pose_to_halcon_tuple(it->second);
    }
    if (data_name == "rmse") {
      const auto it = data.calibration_result()->per_view_rmse.find(key);
      if (it == data.calibration_result()->per_view_rmse.end())
        throw core::Exception(core::Status::NotFound, "get_calib_data", "calibration pose RMSE not found");
      return core::Tuple{it->second};
    }
    if (data_name == "errors") {
      const auto pose_it = data.calibration_result()->poses.find(key);
      if (pose_it == data.calibration_result()->poses.end()) {
        throw core::Exception(core::Status::NotFound, "get_calib_data", "calibration pose result not found");
      }
      const auto obs_it = data.observations().find(key);
      if (obs_it == data.observations().end()) {
        throw core::Exception(core::Status::NotFound, "get_calib_data", "calibration observation not found");
      }
      if (!data.camera(0).parameters) {
        throw core::Exception(core::Status::InvalidState, "get_calib_data",
                              "camera parameters are not configured");
      }
      const auto& target = data.target(object_index);
      const auto& observation = obs_it->second;
      core::Tuple result;
      for (std::size_t i = 0; i < observation.rows.size(); ++i) {
        const auto object_point = static_cast<std::size_t>(observation.indices[i]);
        const auto image_point = project_camera_point(
            *data.camera(0).parameters, transform_point(pose_it->second, target.points.at(object_point)));
        result.append(
            std::hypot(image_point.row - observation.rows[i], image_point.col - observation.columns[i]));
      }
      return result;
    }
    throw core::Exception(core::Status::Unsupported, "get_calib_data",
                          "unsupported calib_obj_pose data name");
  }
  if (item_type == "calib_obj") {
    const auto index = single_index(item_index, "get_calib_data");
    const auto& target = data.target(index);
    if (target.kind == TargetKind::Unconfigured)
      throw core::Exception(core::Status::InvalidState, "get_calib_data",
                            "calibration object is not configured");
    if (data_name == "num_marks") return core::Tuple{static_cast<std::int64_t>(target.points.size())};
    if (data_name == "x" || data_name == "y" || data_name == "z") {
      core::Tuple output;
      for (const auto& point : target.points)
        output.append(data_name == "x" ? point.x : data_name == "y" ? point.y : point.z);
      return output;
    }
    if (!data.hand_eye_result())
      throw core::Exception(core::Status::InvalidState, "get_calib_data", "hand-eye result is not available");
    if (data.setup_kind() == SetupKind::HandEyeMovingCamera && data_name == "object_in_base_pose")
      return pose_to_halcon_tuple(data.hand_eye_result()->second_pose);
    if (data.setup_kind() == SetupKind::HandEyeStationaryCamera && data_name == "object_in_tool_pose")
      return pose_to_halcon_tuple(data.hand_eye_result()->second_pose);
    throw core::Exception(core::Status::Unsupported, "get_calib_data", "unsupported calib_obj data name");
  }
  if (item_type == "model") {
    if (!general_index(item_index))
      throw core::Exception(core::Status::InvalidArgument, "get_calib_data",
                            "model item index must be 'general'");
    if (data_name == "type") {
      switch (data.setup_kind()) {
        case SetupKind::CalibrationObject:
          return core::Tuple{"calibration_object"};
        case SetupKind::HandEyeMovingCamera:
          return core::Tuple{"hand_eye_moving_cam"};
        case SetupKind::HandEyeStationaryCamera:
          return core::Tuple{"hand_eye_stationary_cam"};
        case SetupKind::HandEyeScaraMovingCamera:
          return core::Tuple{"hand_eye_scara_moving_cam"};
        case SetupKind::HandEyeScaraStationaryCamera:
          return core::Tuple{"hand_eye_scara_stationary_cam"};
      }
    }
    if (data_name == "num_cameras") return core::Tuple{static_cast<std::int64_t>(data.num_cameras())};
    if (data_name == "num_calib_objs")
      return core::Tuple{static_cast<std::int64_t>(data.num_calibration_objects())};
    if (data_name == "max_iterations")
      return core::Tuple{static_cast<std::int64_t>(data.options().max_iterations)};
    if (data_name == "function_tolerance") return core::Tuple{data.options().function_tolerance};
    if (data_name == "gradient_tolerance") return core::Tuple{data.options().gradient_tolerance};
    if (data_name == "parameter_tolerance") return core::Tuple{data.options().parameter_tolerance};
    if (data_name == "initial_damping") return core::Tuple{data.options().initial_damping};
    if (data_name == "optimization_method") {
      core::Tuple result;
      result.append(data.optimization_method());
      return result;
    }
    if (data.calibration_result()) {
      if (data_name == "rmse") return core::Tuple{data.calibration_result()->rmse};
      if (data_name == "mean_error") return core::Tuple{data.calibration_result()->mean_error};
      if (data_name == "max_error") return core::Tuple{data.calibration_result()->max_error};
      if (data_name == "num_points")
        return core::Tuple{static_cast<std::int64_t>(data.calibration_result()->num_points)};
      if (data_name == "solver_summary") {
        const auto& q = data.calibration_result()->solver_summary;
        core::Tuple result;
        result.append("termination").append(static_cast<std::int64_t>(q.termination));
        result.append("iterations").append(static_cast<std::int64_t>(q.iterations));
        result.append("initial_cost").append(q.initial_cost);
        result.append("final_cost").append(q.final_cost);
        result.append("message").append(std::string_view{q.message});
        return result;
      }
    }
    if (data.hand_eye_result()) {
      if (data_name == "translation_rmse") return core::Tuple{data.hand_eye_result()->translation_rmse};
      if (data_name == "rotation_rmse") return core::Tuple{data.hand_eye_result()->rotation_rmse};
      if (data_name == "translation_max") return core::Tuple{data.hand_eye_result()->translation_max};
      if (data_name == "rotation_max") return core::Tuple{data.hand_eye_result()->rotation_max};
    }
    throw core::Exception(core::Status::Unsupported, "get_calib_data", "unsupported model data name");
  }
  throw core::Exception(core::Status::Unsupported, "get_calib_data", "unsupported item type");
}

}  // namespace mv::calibration
