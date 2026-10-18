#pragma once
#include <memory>
#include <calibration/solver/problem.hpp>
#include <core/geometry_types.hpp>
namespace mv::calibration {
std::unique_ptr<solver::ResidualFunction> make_hand_eye_residual(bool moving,solver::ParameterBlockId first,solver::ParameterBlockId second,Pose3d tool_in_base,Pose3d object_in_camera);
}
