#include <mvkit/c/mv_reconstruction3d.h>

#include <mvkit/cpp/mv_reconstruction3d.hpp>

#include "api_bridge.hpp"
namespace mv {
void RegisterObjectModel3dPair(const MvTuple& ObjectModel3D1, const MvTuple& ObjectModel3D2,
                               const MvTuple& Method, const MvTuple& GenParamName,
                               const MvTuple& GenParamValue, MvTuple* Pose, MvTuple* Score) {
  cpp_api::TupleOutputs<2> output("RegisterObjectModel3dPair", {Pose, Score});
  cpp_api::check_status(
      mv_register_object_model_3d_pair(ObjectModel3D1.ControlInput(), ObjectModel3D2.ControlInput(),
                                       Method.ControlInput(), GenParamName.ControlInput(),
                                       GenParamValue.ControlInput(), output.Out(0), output.Out(1)),
      "RegisterObjectModel3dPair");
  output.Commit();
}

MvPose MvObjectModel3D::RegisterObjectModel3dPair(const MvObjectModel3D& target, const MvString& method,
                                                  const MvTuple& names, const MvTuple& values,
                                                  MvTuple* score) const {
  if (!score)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::NullArgument, "RegisterObjectModel3dPair", "Score",
                      -1, "Score must not be null");
  MvTuple pose, result;
  mv::RegisterObjectModel3dPair(MvTuple(*this), MvTuple(target), method, names, values, &pose, &result);
  MvPose output(pose);
  *score = std::move(result);
  return output;
}
MvPose MvObjectModel3D::RegisterObjectModel3dPair(const MvObjectModel3D& target, const MvString& method,
                                                  const MvString& name, double value, MvTuple* score) const {
  return RegisterObjectModel3dPair(target, method, MvTuple(name), MvTuple(value), score);
}
MvPose MvObjectModel3D::RegisterObjectModel3dPair(const MvObjectModel3D& target, const char* method,
                                                  const char* name, double value, MvTuple* score) const {
  return RegisterObjectModel3dPair(target, MvString(method), MvString(name), value, score);
}
MvPose MvObjectModel3D::RegisterObjectModel3dPair(const MvObjectModel3D& target, const wchar_t* method,
                                                  const wchar_t* name, double value, MvTuple* score) const {
  return RegisterObjectModel3dPair(target, MvString(method), MvString(name), value, score);
}
}  // namespace mv
