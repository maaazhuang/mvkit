#include <mvkit/c/mv_reconstruction3d.h>

#include <mvkit/cpp/mv_reconstruction3d.hpp>

#include "api_bridge.hpp"
namespace mv {
void DistanceObjectModel3d(const MvTuple& ObjectModel3DFrom, const MvTuple& ObjectModel3DTo,
                           const MvTuple& Pose, const MvTuple& MaxDistance, const MvTuple& GenParamName,
                           const MvTuple& GenParamValue) {
  cpp_api::check_status(
      mv_distance_object_model_3d(ObjectModel3DFrom.ControlInput(), ObjectModel3DTo.ControlInput(),
                                  Pose.ControlInput(), MaxDistance.ControlInput(),
                                  GenParamName.ControlInput(), GenParamValue.ControlInput()),
      "DistanceObjectModel3d");
}

void MvObjectModel3D::DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose,
                                            const MvTuple& MaxDistance, const MvTuple& GenParamName,
                                            const MvTuple& GenParamValue) const {
  mv::DistanceObjectModel3d(MvTuple(*this), MvTuple(ObjectModel3DTo), Pose, MaxDistance, GenParamName,
                            GenParamValue);
}
void MvObjectModel3D::DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose,
                                            double MaxDistance, const MvString& GenParamName,
                                            const MvString& GenParamValue) const {
  DistanceObjectModel3d(ObjectModel3DTo, Pose, MvTuple(MaxDistance), MvTuple(GenParamName),
                        MvTuple(GenParamValue));
}
void MvObjectModel3D::DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose,
                                            double MaxDistance, const char* GenParamName,
                                            const char* GenParamValue) const {
  DistanceObjectModel3d(ObjectModel3DTo, Pose, MaxDistance, MvString(GenParamName), MvString(GenParamValue));
}
void MvObjectModel3D::DistanceObjectModel3d(const MvObjectModel3D& ObjectModel3DTo, const MvPose& Pose,
                                            double MaxDistance, const wchar_t* GenParamName,
                                            const wchar_t* GenParamValue) const {
  DistanceObjectModel3d(ObjectModel3DTo, Pose, MaxDistance, MvString(GenParamName), MvString(GenParamValue));
}
}  // namespace mv
