#include "api_bridge.hpp"
#include <mvkit/c/mv_calibration.h>

#include <mvkit/cpp/mv_calibration.hpp>
#include <mvkit/cpp/mv_geometry.hpp>

namespace mv {
MvCalibData::MvCalibData() noexcept = default;
MvCalibData::MvCalibData(const MvCalibData&) = default;
MvCalibData::MvCalibData(MvCalibData&&) noexcept = default;
MvCalibData& MvCalibData::operator=(const MvCalibData&) = default;
MvCalibData& MvCalibData::operator=(MvCalibData&&) noexcept = default;
MvCalibData::~MvCalibData() noexcept = default;
MvCalibData::MvCalibData(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::CalibrationData)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvCalibData", "Handle", -1,
                      "control handle has wrong model type");
}
MvCamPar::MvCamPar() noexcept = default;
MvCamPar::MvCamPar(const MvTuple& value) : MvTuple(value) {}
MvCamPar::MvCamPar(const MvCamPar&) = default;
MvCamPar::MvCamPar(MvCamPar&&) noexcept = default;
MvCamPar& MvCamPar::operator=(const MvCamPar&) = default;
MvCamPar& MvCamPar::operator=(MvCamPar&&) noexcept = default;
MvCamPar::~MvCamPar() noexcept = default;

void CreateCalibData(const MvTuple& CalibSetup, const MvTuple& NumCameras, const MvTuple& NumCalibObjects,
                     MvTuple* CalibDataID) {
  cpp_api::TupleOutputs<1> output("CreateCalibData", {CalibDataID});
  cpp_api::check_status(mv_create_calib_data(CalibSetup.ControlInput(), NumCameras.ControlInput(),
                                                NumCalibObjects.ControlInput(), output.Out(0)),
                           "CreateCalibData");
  output.Commit();
}
void SetCalibDataCamParam(const MvTuple& CalibDataID, const MvTuple& CameraIdx, const MvTuple& CameraType,
                          const MvTuple& CameraParam) {
  cpp_api::check_status(mv_set_calib_data_cam_param(CalibDataID.ControlInput(), CameraIdx.ControlInput(),
                                                       CameraType.ControlInput(), CameraParam.ControlInput()),
                           "SetCalibDataCamParam");
}
void SetCalibDataCalibObject(const MvTuple& CalibDataID, const MvTuple& CalibObjIdx,
                             const MvTuple& CalibObjDescr) {
  cpp_api::check_status(
      mv_set_calib_data_calib_object(CalibDataID.ControlInput(), CalibObjIdx.ControlInput(),
                                     CalibObjDescr.ControlInput()),
      "SetCalibDataCalibObject");
}
void MvCalibData::CreateCalibData(const MvString& setup, std::int64_t nc, std::int64_t no) {
  MvTuple result;
  mv::CreateCalibData(MvTuple(setup), nc, no, &result);
  MvHandle::operator=(result.H());
}
void MvCalibData::CreateCalibData(const char* setup, std::int64_t nc, std::int64_t no) {
  CreateCalibData(MvString(setup), nc, no);
}
void MvCalibData::CreateCalibData(const wchar_t* setup, std::int64_t nc, std::int64_t no) {
  CreateCalibData(MvString(setup), nc, no);
}
void MvCalibData::SetCalibDataCamParam(const MvTuple& idx, const MvTuple& type,
                                       const MvCamPar& parameters) const {
  mv::SetCalibDataCamParam(MvTuple(*this), idx, type, parameters);
}
void MvCalibData::SetCalibDataCamParam(const MvTuple& idx, const MvString& type,
                                       const MvCamPar& parameters) const {
  SetCalibDataCamParam(idx, MvTuple(type), parameters);
}
void MvCalibData::SetCalibDataCamParam(const MvTuple& idx, const char* type,
                                       const MvCamPar& parameters) const {
  SetCalibDataCamParam(idx, MvString(type), parameters);
}
void MvCalibData::SetCalibDataCamParam(const MvTuple& idx, const wchar_t* type,
                                       const MvCamPar& parameters) const {
  SetCalibDataCamParam(idx, MvString(type), parameters);
}
void MvCalibData::SetCalibDataCalibObject(std::int64_t idx, const MvTuple& description) const {
  mv::SetCalibDataCalibObject(MvTuple(*this), idx, description);
}
void MvCalibData::SetCalibDataCalibObject(std::int64_t idx, double description) const {
  SetCalibDataCalibObject(idx, MvTuple(description));
}
void MvCamPar::SetCalibDataCamParam(const MvCalibData& data, const MvTuple& idx, const MvTuple& type) const {
  data.SetCalibDataCamParam(idx, type, *this);
}
void MvCamPar::SetCalibDataCamParam(const MvCalibData& data, const MvTuple& idx, const MvString& type) const {
  data.SetCalibDataCamParam(idx, type, *this);
}
void MvCamPar::SetCalibDataCamParam(const MvCalibData& data, const MvTuple& idx, const char* type) const {
  data.SetCalibDataCamParam(idx, type, *this);
}
void MvCamPar::SetCalibDataCamParam(const MvCalibData& data, const MvTuple& idx, const wchar_t* type) const {
  data.SetCalibDataCamParam(idx, type, *this);
}
void SetCalibData(const MvTuple& id, const MvTuple& type, const MvTuple& index, const MvTuple& name,
                  const MvTuple& value) {
  cpp_api::check_status(mv_set_calib_data(id.ControlInput(), type.ControlInput(), index.ControlInput(),
                                             name.ControlInput(), value.ControlInput()),
                           "SetCalibData");
}
void GetCalibData(const MvTuple& id, const MvTuple& type, const MvTuple& index, const MvTuple& name,
                  MvTuple* value) {
  cpp_api::TupleOutputs<1> output("GetCalibData", {value});
  cpp_api::check_status(mv_get_calib_data(id.ControlInput(), type.ControlInput(), index.ControlInput(),
                                             name.ControlInput(), output.Out(0)),
                           "GetCalibData");
  output.Commit();
}
void MvCalibData::SetCalibData(const MvString& type, const MvTuple& index, const MvString& name,
                               const MvTuple& value) const {
  mv::SetCalibData(MvTuple(*this), type, index, name, value);
}
void MvCalibData::SetCalibData(const MvString& type, std::int64_t index, const MvString& name,
                               const MvString& value) const {
  SetCalibData(type, MvTuple(index), name, MvTuple(value));
}
void MvCalibData::SetCalibData(const char* type, std::int64_t index, const char* name,
                               const char* value) const {
  SetCalibData(MvString(type), index, MvString(name), MvString(value));
}
void MvCalibData::SetCalibData(const wchar_t* type, std::int64_t index, const wchar_t* name,
                               const wchar_t* value) const {
  SetCalibData(MvString(type), index, MvString(name), MvString(value));
}
MvTuple MvCalibData::GetCalibData(const MvString& type, const MvTuple& index, const MvTuple& name) const {
  MvTuple value;
  mv::GetCalibData(MvTuple(*this), type, index, name, &value);
  return value;
}
MvTuple MvCalibData::GetCalibData(const MvString& type, std::int64_t index, const MvString& name) const {
  return GetCalibData(type, MvTuple(index), MvTuple(name));
}
MvTuple MvCalibData::GetCalibData(const char* type, std::int64_t index, const char* name) const {
  return GetCalibData(MvString(type), index, MvString(name));
}
MvTuple MvCalibData::GetCalibData(const wchar_t* type, std::int64_t index, const wchar_t* name) const {
  return GetCalibData(MvString(type), index, MvString(name));
}
void SetCalibDataObservPoints(const MvTuple& id, const MvTuple& camera, const MvTuple& object,
                              const MvTuple& view, const MvTuple& row, const MvTuple& column,
                              const MvTuple& index, const MvTuple& pose) {
  cpp_api::check_status(
      mv_set_calib_data_observ_points(id.ControlInput(), camera.ControlInput(), object.ControlInput(),
                                      view.ControlInput(), row.ControlInput(), column.ControlInput(),
                                      index.ControlInput(), pose.ControlInput()),
      "SetCalibDataObservPoints");
}
void GetCalibDataObservPoints(const MvTuple& id, const MvTuple& camera, const MvTuple& object,
                              const MvTuple& view, MvTuple* row, MvTuple* column, MvTuple* index,
                              MvTuple* pose) {
  cpp_api::TupleOutputs<4> output("GetCalibDataObservPoints", {row, column, index, pose});
  cpp_api::check_status(
      mv_get_calib_data_observ_points(id.ControlInput(), camera.ControlInput(), object.ControlInput(),
                                      view.ControlInput(), output.Out(0), output.Out(1), output.Out(2),
                                      output.Out(3)),
      "GetCalibDataObservPoints");
  output.Commit();
}
void FindCalibObject(const MvObject& image, const MvTuple& id, const MvTuple& camera, const MvTuple& object,
                     const MvTuple& view, const MvTuple& names, const MvTuple& values) {
  cpp_api::check_status(
      mv_find_calib_object(image.NativeHandle(), id.ControlInput(), camera.ControlInput(),
                           object.ControlInput(), view.ControlInput(), names.ControlInput(),
                           values.ControlInput()),
      "FindCalibObject");
}
void GetCalibDataObservContours(MvObject* contours, const MvTuple& id, const MvTuple& name,
                                const MvTuple& camera, const MvTuple& object, const MvTuple& view) {
  cpp_api::require_output(contours, "GetCalibDataObservContours");
  ::MvObject raw = nullptr;
  cpp_api::check_status(
      mv_get_calib_data_observ_contours(&raw, id.ControlInput(), name.ControlInput(), camera.ControlInput(),
                                        object.ControlInput(), view.ControlInput()),
      "GetCalibDataObservContours");
  auto value = detail::MvObjectAccess::Adopt(raw);
  detail::MvObjectAccess::CheckOutput(contours, value);
  detail::MvObjectAccess::Commit(contours, std::move(value));
}
void MvCalibData::SetCalibDataObservPoints(std::int64_t camera, std::int64_t object, std::int64_t view,
                                           const MvTuple& row, const MvTuple& column, const MvTuple& index,
                                           const MvTuple& pose) const {
  mv::SetCalibDataObservPoints(MvTuple(*this), camera, object, view, row, column, index, pose);
}
void MvCalibData::GetCalibDataObservPoints(std::int64_t camera, std::int64_t object, std::int64_t view,
                                           MvTuple* row, MvTuple* column, MvTuple* index,
                                           MvTuple* pose) const {
  mv::GetCalibDataObservPoints(MvTuple(*this), camera, object, view, row, column, index, pose);
}
void MvCalibData::FindCalibObject(const MvImage& image, std::int64_t camera, std::int64_t object,
                                  std::int64_t view, const MvTuple& names, const MvTuple& values) const {
  mv::FindCalibObject(image, MvTuple(*this), camera, object, view, names, values);
}
MvXLDCont MvCalibData::GetCalibDataObservContours(const MvString& name, std::int64_t camera,
                                                  std::int64_t object, std::int64_t view) const {
  MvObject output;
  mv::GetCalibDataObservContours(&output, MvTuple(*this), name, camera, object, view);
  return MvXLDCont(output);
}
MvXLDCont MvCalibData::GetCalibDataObservContours(const char* name, std::int64_t camera, std::int64_t object,
                                                  std::int64_t view) const {
  return GetCalibDataObservContours(MvString(name), camera, object, view);
}
MvXLDCont MvCalibData::GetCalibDataObservContours(const wchar_t* name, std::int64_t camera,
                                                  std::int64_t object, std::int64_t view) const {
  return GetCalibDataObservContours(MvString(name), camera, object, view);
}
}  // namespace mv

namespace mv {
void CalibrateCameras(const MvTuple& CalibDataID, MvTuple* Error) {
  cpp_api::TupleOutputs<1> out("CalibrateCameras", {Error});
  cpp_api::check_status(mv_calibrate_cameras(CalibDataID.ControlInput(), out.Out(0)), "CalibrateCameras");
  out.Commit();
}
void CalibrateHandEye(const MvTuple& CalibDataID, MvTuple* Errors) {
  cpp_api::TupleOutputs<1> out("CalibrateHandEye", {Errors});
  cpp_api::check_status(mv_calibrate_hand_eye(CalibDataID.ControlInput(), out.Out(0)), "CalibrateHandEye");
  out.Commit();
}
void RemoveCalibDataObserv(const MvTuple& CalibDataID, const MvTuple& CameraIdx, const MvTuple& CalibObjIdx,
                           const MvTuple& CalibObjPoseIdx) {
  cpp_api::check_status(
      mv_remove_calib_data_observ(CalibDataID.ControlInput(), CameraIdx.ControlInput(),
                                  CalibObjIdx.ControlInput(), CalibObjPoseIdx.ControlInput()),
      "RemoveCalibDataObserv");
}
void ReadCalibData(const MvTuple& FileName, MvTuple* CalibDataID) {
  cpp_api::TupleOutputs<1> out("ReadCalibData", {CalibDataID});
  cpp_api::check_status(mv_read_calib_data(FileName.ControlInput(), out.Out(0)), "ReadCalibData");
  out.Commit();
}
void WriteCalibData(const MvTuple& CalibDataID, const MvTuple& FileName) {
  cpp_api::check_status(mv_write_calib_data(CalibDataID.ControlInput(), FileName.ControlInput()),
                           "WriteCalibData");
}
void ReadCamPar(const MvTuple& CamParFile, MvTuple* CameraParam) {
  cpp_api::TupleOutputs<1> out("ReadCamPar", {CameraParam});
  cpp_api::check_status(mv_read_cam_par(CamParFile.ControlInput(), out.Out(0)), "ReadCamPar");
  out.Commit();
}
void WriteCamPar(const MvTuple& CameraParam, const MvTuple& CamParFile) {
  cpp_api::check_status(mv_write_cam_par(CameraParam.ControlInput(), CamParFile.ControlInput()),
                           "WriteCamPar");
}
void ReadPose(const MvTuple& PoseFile, MvTuple* Pose) {
  cpp_api::TupleOutputs<1> out("ReadPose", {Pose});
  cpp_api::check_status(mv_read_pose(PoseFile.ControlInput(), out.Out(0)), "ReadPose");
  out.Commit();
}
void WritePose(const MvTuple& Pose, const MvTuple& PoseFile) {
  cpp_api::check_status(mv_write_pose(Pose.ControlInput(), PoseFile.ControlInput()), "WritePose");
}
double MvCalibData::CalibrateCameras() const {
  MvTuple e;
  mv::CalibrateCameras(MvTuple(*this), &e);
  return e.D();
}
MvTuple MvCalibData::CalibrateHandEye() const {
  MvTuple e;
  mv::CalibrateHandEye(MvTuple(*this), &e);
  return e;
}
void MvCalibData::RemoveCalibDataObserv(std::int64_t ci, std::int64_t oi, std::int64_t pi) const {
  mv::RemoveCalibDataObserv(MvTuple(*this), ci, oi, pi);
}
void MvCalibData::ReadCalibData(const MvString& FileName) {
  MvTuple v;
  mv::ReadCalibData(MvTuple(FileName), &v);
  MvHandle::operator=(v.H());
}
void MvCalibData::WriteCalibData(const MvString& FileName) const {
  mv::WriteCalibData(MvTuple(*this), MvTuple(FileName));
}
void MvCalibData::ReadCalibData(const char* p) { ReadCalibData(MvString(p)); }
void MvCalibData::ReadCalibData(const wchar_t* p) { ReadCalibData(MvString(p)); }
void MvCalibData::WriteCalibData(const char* p) const { WriteCalibData(MvString(p)); }
void MvCalibData::WriteCalibData(const wchar_t* p) const { WriteCalibData(MvString(p)); }
void MvCamPar::ReadCamPar(const MvString& CamParFile) {
  MvTuple v;
  mv::ReadCamPar(MvTuple(CamParFile), &v);
  MvTuple::operator=(std::move(v));
}
void MvCamPar::WriteCamPar(const MvString& CamParFile) const {
  mv::WriteCamPar(MvTuple(*this), MvTuple(CamParFile));
}
void MvCamPar::ReadCamPar(const char* p) { ReadCamPar(MvString(p)); }
void MvCamPar::ReadCamPar(const wchar_t* p) { ReadCamPar(MvString(p)); }
void MvCamPar::WriteCamPar(const char* p) const { WriteCamPar(MvString(p)); }
void MvCamPar::WriteCamPar(const wchar_t* p) const { WriteCamPar(MvString(p)); }
void MvPose::ReadPose(const MvString& PoseFile) {
  MvTuple v;
  mv::ReadPose(MvTuple(PoseFile), &v);
  MvTuple::operator=(std::move(v));
}
void MvPose::WritePose(const MvString& PoseFile) const { mv::WritePose(MvTuple(*this), MvTuple(PoseFile)); }
void MvPose::ReadPose(const char* p) { ReadPose(MvString(p)); }
void MvPose::ReadPose(const wchar_t* p) { ReadPose(MvString(p)); }
void MvPose::WritePose(const char* p) const { WritePose(MvString(p)); }
void MvPose::WritePose(const wchar_t* p) const { WritePose(MvString(p)); }

void ContourToWorldPlaneXld(const MvObject& contours, MvObject* output, const MvTuple& camera,
                            const MvTuple& pose, const MvTuple& scale) {
  cpp_api::require_output(output, "ContourToWorldPlaneXld");
  ::MvObject raw = nullptr;
  cpp_api::check_status(mv_contour_to_world_plane_xld(contours.NativeHandle(), &raw, camera.ControlInput(),
                                                         pose.ControlInput(), scale.ControlInput()),
                           "ContourToWorldPlaneXld");
  auto value = detail::MvObjectAccess::Adopt(raw);
  detail::MvObjectAccess::CheckOutput(output, value);
  detail::MvObjectAccess::Commit(output, std::move(value));
}
void ChangeRadialDistortionContoursXld(const MvObject& contours, MvObject* output, const MvTuple& camera_in,
                                       const MvTuple& camera_out) {
  cpp_api::require_output(output, "ChangeRadialDistortionContoursXld");
  ::MvObject raw = nullptr;
  cpp_api::check_status(
      mv_change_radial_distortion_contours_xld(contours.NativeHandle(), &raw, camera_in.ControlInput(),
                                               camera_out.ControlInput()),
      "ChangeRadialDistortionContoursXld");
  auto value = detail::MvObjectAccess::Adopt(raw);
  detail::MvObjectAccess::CheckOutput(output, value);
  detail::MvObjectAccess::Commit(output, std::move(value));
}
MvXLDCont MvXLDCont::ContourToWorldPlaneXld(const MvTuple& camera, const MvPose& pose,
                                            const MvTuple& scale) const {
  MvXLDCont result;
  mv::ContourToWorldPlaneXld(*this, &result, camera, pose, scale);
  return result;
}
MvXLDCont MvPose::ContourToWorldPlaneXld(const MvXLDCont& contours, const MvTuple& camera,
                                         const MvTuple& scale) const {
  return contours.ContourToWorldPlaneXld(camera, *this, scale);
}
MvXLDCont MvXLDCont::ContourToWorldPlaneXld(const MvTuple& camera, const MvPose& pose,
                                            const MvString& scale) const {
  return ContourToWorldPlaneXld(camera, pose, MvTuple(scale));
}
MvXLDCont MvXLDCont::ContourToWorldPlaneXld(const MvTuple& camera, const MvPose& pose,
                                            const char* scale) const {
  return ContourToWorldPlaneXld(camera, pose, MvString(scale));
}
MvXLDCont MvXLDCont::ContourToWorldPlaneXld(const MvTuple& camera, const MvPose& pose,
                                            const wchar_t* scale) const {
  return ContourToWorldPlaneXld(camera, pose, MvString(scale));
}
MvXLDCont MvPose::ContourToWorldPlaneXld(const MvXLDCont& contours, const MvTuple& camera,
                                         const MvString& scale) const {
  return ContourToWorldPlaneXld(contours, camera, MvTuple(scale));
}
MvXLDCont MvPose::ContourToWorldPlaneXld(const MvXLDCont& contours, const MvTuple& camera,
                                         const char* scale) const {
  return ContourToWorldPlaneXld(contours, camera, MvString(scale));
}
MvXLDCont MvPose::ContourToWorldPlaneXld(const MvXLDCont& contours, const MvTuple& camera,
                                         const wchar_t* scale) const {
  return ContourToWorldPlaneXld(contours, camera, MvString(scale));
}
MvXLDCont MvXLDCont::ChangeRadialDistortionContoursXld(const MvCamPar& in, const MvCamPar& out) const {
  MvXLDCont result;
  mv::ChangeRadialDistortionContoursXld(*this, &result, in, out);
  return result;
}
MvXLDCont MvCamPar::ChangeRadialDistortionContoursXld(const MvXLDCont& contours, const MvCamPar& out) const {
  return contours.ChangeRadialDistortionContoursXld(*this, out);
}

void Project3dPoint(const MvTuple& X, const MvTuple& Y, const MvTuple& Z, const MvTuple& CameraParam,
                    MvTuple* Row, MvTuple* Column) {
  cpp_api::TupleOutputs<2> output("Project3dPoint", {Row, Column});
  cpp_api::check_status(mv_project_3d_point(X.ControlInput(), Y.ControlInput(), Z.ControlInput(),
                                               CameraParam.ControlInput(), output.Out(0), output.Out(1)),
                           "Project3dPoint");
  output.Commit();
}

void ChangeRadialDistortionPoints(const MvTuple& Row, const MvTuple& Col, const MvTuple& CamParamIn,
                                  const MvTuple& CamParamOut, MvTuple* RowChanged, MvTuple* ColChanged) {
  cpp_api::TupleOutputs<2> output("ChangeRadialDistortionPoints", {RowChanged, ColChanged});
  cpp_api::check_status(
      mv_change_radial_distortion_points(Row.ControlInput(), Col.ControlInput(), CamParamIn.ControlInput(),
                                         CamParamOut.ControlInput(), output.Out(0), output.Out(1)),
      "ChangeRadialDistortionPoints");
  output.Commit();
}
void ImagePointsToWorldPlane(const MvTuple& CameraParam, const MvTuple& WorldPose, const MvTuple& Rows,
                             const MvTuple& Cols, const MvTuple& Scale, MvTuple* X, MvTuple* Y) {
  cpp_api::TupleOutputs<2> output("ImagePointsToWorldPlane", {X, Y});
  cpp_api::check_status(mv_image_points_to_world_plane(
                               CameraParam.ControlInput(), WorldPose.ControlInput(), Rows.ControlInput(),
                               Cols.ControlInput(), Scale.ControlInput(), output.Out(0), output.Out(1)),
                           "ImagePointsToWorldPlane");
  output.Commit();
}
void MvCamPar::Project3dPoint(const MvTuple& X, const MvTuple& Y, const MvTuple& Z, MvTuple* Row,
                              MvTuple* Column) const {
  mv::Project3dPoint(X, Y, Z, *this, Row, Column);
}
void MvCamPar::ChangeRadialDistortionPoints(const MvTuple& Row, const MvTuple& Col, const MvCamPar& Out,
                                            MvTuple* RowChanged, MvTuple* ColChanged) const {
  mv::ChangeRadialDistortionPoints(Row, Col, *this, Out, RowChanged, ColChanged);
}
void MvCamPar::ImagePointsToWorldPlane(const MvPose& Pose, const MvTuple& Rows, const MvTuple& Cols,
                                       const MvTuple& Scale, MvTuple* X, MvTuple* Y) const {
  mv::ImagePointsToWorldPlane(*this, Pose, Rows, Cols, Scale, X, Y);
}
void MvCamPar::ImagePointsToWorldPlane(const MvPose& Pose, const MvTuple& Rows, const MvTuple& Cols,
                                       const MvString& Scale, MvTuple* X, MvTuple* Y) const {
  ImagePointsToWorldPlane(Pose, Rows, Cols, MvTuple(Scale), X, Y);
}
void MvCamPar::ImagePointsToWorldPlane(const MvPose& Pose, const MvTuple& Rows, const MvTuple& Cols,
                                       const char* Scale, MvTuple* X, MvTuple* Y) const {
  ImagePointsToWorldPlane(Pose, Rows, Cols, MvString(Scale), X, Y);
}
void MvCamPar::ImagePointsToWorldPlane(const MvPose& Pose, const MvTuple& Rows, const MvTuple& Cols,
                                       const wchar_t* Scale, MvTuple* X, MvTuple* Y) const {
  ImagePointsToWorldPlane(Pose, Rows, Cols, MvString(Scale), X, Y);
}
}  // namespace mv

namespace mv {
void ChangeRadialDistortionCamPar(const MvTuple& mode, const MvTuple& in, const MvTuple& coeff,
                                  MvTuple* out) {
  cpp_api::TupleOutputs<1> output("ChangeRadialDistortionCamPar", {out});
  cpp_api::check_status(mv_change_radial_distortion_cam_par(mode.ControlInput(), in.ControlInput(),
                                                               coeff.ControlInput(), output.Out(0)),
                           "ChangeRadialDistortionCamPar");
  output.Commit();
}
MvCamPar MvCamPar::ChangeRadialDistortionCamPar(const MvString& mode, const MvTuple& coeff) const {
  MvTuple result;
  mv::ChangeRadialDistortionCamPar(MvTuple(mode), *this, coeff, &result);
  return MvCamPar(result);
}
MvCamPar MvCamPar::ChangeRadialDistortionCamPar(const MvString& mode, double coeff) const {
  return ChangeRadialDistortionCamPar(mode, MvTuple(coeff));
}
MvCamPar MvCamPar::ChangeRadialDistortionCamPar(const char* mode, double coeff) const {
  return ChangeRadialDistortionCamPar(MvString(mode), coeff);
}
MvCamPar MvCamPar::ChangeRadialDistortionCamPar(const wchar_t* mode, double coeff) const {
  return ChangeRadialDistortionCamPar(MvString(mode), coeff);
}
void VectorToHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u, const MvTuple& v,
                      MvTuple* result) {
  cpp_api::TupleOutputs<1> output("VectorToHomMat2d", {result});
  cpp_api::check_status(mv_vector_to_hom_mat2d(x.ControlInput(), y.ControlInput(), u.ControlInput(),
                                                  v.ControlInput(), output.Out(0)),
                           "VectorToHomMat2d");
  output.Commit();
}
void MvHomMat2D::VectorToHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u, const MvTuple& v) {
  MvTuple result;
  mv::VectorToHomMat2d(x, y, u, v, &result);
  MvTuple::operator=(std::move(result));
}
}  // namespace mv

namespace mv {
namespace {
template <class Call>
void rect_output(MvObject* result, const char* op, Call&& call) {
  cpp_api::require_output(result, op);
  ::MvObject handle = nullptr;
  cpp_api::check_status(call(&handle), op);
  detail::MvObjectAccess::Commit(result, detail::MvObjectAccess::Adopt(handle));
}
}  // namespace
void GenRadialDistortionMap(MvObject* Map, const MvTuple& in, const MvTuple& out, const MvTuple& type) {
  rect_output(Map, "GenRadialDistortionMap", [&](::MvObject* h) {
    return mv_gen_radial_distortion_map(h, in.ControlInput(), out.ControlInput(), type.ControlInput());
  });
}
void GenImageToWorldPlaneMap(MvObject* Map, const MvTuple& camera, const MvTuple& pose, const MvTuple& wi,
                             const MvTuple& hi, const MvTuple& w, const MvTuple& h, const MvTuple& scale,
                             const MvTuple& type) {
  rect_output(Map, "GenImageToWorldPlaneMap", [&](::MvObject* result) {
    return mv_gen_image_to_world_plane_map(result, camera.ControlInput(), pose.ControlInput(),
                                           wi.ControlInput(), hi.ControlInput(), w.ControlInput(),
                                           h.ControlInput(), scale.ControlInput(), type.ControlInput());
  });
}
void ChangeRadialDistortionImage(const MvObject& image, const MvObject& region, MvObject* result,
                                 const MvTuple& in, const MvTuple& out) {
  rect_output(result, "ChangeRadialDistortionImage", [&](::MvObject* h) {
    return mv_change_radial_distortion_image(image.NativeHandle(), region.NativeHandle(), h,
                                             in.ControlInput(), out.ControlInput());
  });
}
void ImageToWorldPlane(const MvObject& image, MvObject* result, const MvTuple& camera, const MvTuple& pose,
                       const MvTuple& w, const MvTuple& h, const MvTuple& scale, const MvTuple& mode) {
  rect_output(result, "ImageToWorldPlane", [&](::MvObject* out) {
    return mv_image_to_world_plane(image.NativeHandle(), out, camera.ControlInput(), pose.ControlInput(),
                                   w.ControlInput(), h.ControlInput(), scale.ControlInput(),
                                   mode.ControlInput());
  });
}
MvImage MvCamPar::ChangeRadialDistortionImage(const MvImage& image, const MvRegion& region,
                                              const MvCamPar& out) const {
  MvObject result;
  mv::ChangeRadialDistortionImage(image, region, &result, *this, out);
  return MvImage(result);
}
MvImage MvImage::ChangeRadialDistortionImage(const MvRegion& region, const MvCamPar& in,
                                             const MvCamPar& out) const {
  return in.ChangeRadialDistortionImage(*this, region, out);
}
MvImage MvCamPar::GenRadialDistortionMap(const MvCamPar& out, const MvString& type) const {
  MvObject result;
  mv::GenRadialDistortionMap(&result, *this, out, MvTuple(type));
  return MvImage(result);
}
MvImage MvCamPar::GenRadialDistortionMap(const MvCamPar& out, const char* type) const {
  return GenRadialDistortionMap(out, MvString(type));
}
MvImage MvCamPar::GenRadialDistortionMap(const MvCamPar& out, const wchar_t* type) const {
  return GenRadialDistortionMap(out, MvString(type));
}
void MvImage::GenRadialDistortionMap(const MvCamPar& in, const MvCamPar& out, const MvString& type) {
  *this = in.GenRadialDistortionMap(out, type);
}
void MvImage::GenRadialDistortionMap(const MvCamPar& in, const MvCamPar& out, const char* type) {
  GenRadialDistortionMap(in, out, MvString(type));
}
void MvImage::GenRadialDistortionMap(const MvCamPar& in, const MvCamPar& out, const wchar_t* type) {
  GenRadialDistortionMap(in, out, MvString(type));
}
MvImage MvCamPar::GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn,
                                          std::int64_t HeightIn, std::int64_t WidthMapped,
                                          std::int64_t HeightMapped, const MvTuple& Scale,
                                          const MvString& MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, *this, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped, Scale,
                              MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvCamPar::GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn,
                                          std::int64_t HeightIn, std::int64_t WidthMapped,
                                          std::int64_t HeightMapped, const MvString& Scale,
                                          const MvString& MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, *this, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvCamPar::GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn,
                                          std::int64_t HeightIn, std::int64_t WidthMapped,
                                          std::int64_t HeightMapped, const char* Scale,
                                          const char* MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, *this, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvCamPar::GenImageToWorldPlaneMap(const MvPose& WorldPose, std::int64_t WidthIn,
                                          std::int64_t HeightIn, std::int64_t WidthMapped,
                                          std::int64_t HeightMapped, const wchar_t* Scale,
                                          const wchar_t* MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, *this, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
void MvImage::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose,
                                      std::int64_t WidthIn, std::int64_t HeightIn, std::int64_t WidthMapped,
                                      std::int64_t HeightMapped, const MvTuple& Scale,
                                      const MvString& MapType) {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              Scale, MvTuple(MvString(MapType)));
  *this = MvImage(result);
}
void MvImage::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose,
                                      std::int64_t WidthIn, std::int64_t HeightIn, std::int64_t WidthMapped,
                                      std::int64_t HeightMapped, const MvString& Scale,
                                      const MvString& MapType) {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  *this = MvImage(result);
}
void MvImage::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose,
                                      std::int64_t WidthIn, std::int64_t HeightIn, std::int64_t WidthMapped,
                                      std::int64_t HeightMapped, const char* Scale, const char* MapType) {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  *this = MvImage(result);
}
void MvImage::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, const MvPose& WorldPose,
                                      std::int64_t WidthIn, std::int64_t HeightIn, std::int64_t WidthMapped,
                                      std::int64_t HeightMapped, const wchar_t* Scale,
                                      const wchar_t* MapType) {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, WorldPose, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  *this = MvImage(result);
}
MvImage MvPose::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn,
                                        std::int64_t HeightIn, std::int64_t WidthMapped,
                                        std::int64_t HeightMapped, const MvTuple& Scale,
                                        const MvString& MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, *this, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              Scale, MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvPose::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn,
                                        std::int64_t HeightIn, std::int64_t WidthMapped,
                                        std::int64_t HeightMapped, const MvString& Scale,
                                        const MvString& MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, *this, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvPose::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn,
                                        std::int64_t HeightIn, std::int64_t WidthMapped,
                                        std::int64_t HeightMapped, const char* Scale,
                                        const char* MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, *this, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvPose::GenImageToWorldPlaneMap(const MvCamPar& CameraParam, std::int64_t WidthIn,
                                        std::int64_t HeightIn, std::int64_t WidthMapped,
                                        std::int64_t HeightMapped, const wchar_t* Scale,
                                        const wchar_t* MapType) const {
  MvObject result;
  mv::GenImageToWorldPlaneMap(&result, CameraParam, *this, WidthIn, HeightIn, WidthMapped, HeightMapped,
                              MvTuple(MvString(Scale)), MvTuple(MvString(MapType)));
  return MvImage(result);
}
MvImage MvCamPar::ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                                    std::int64_t Height, const MvTuple& Scale,
                                    const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, *this, WorldPose, Width, Height, Scale,
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvCamPar::ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                                    std::int64_t Height, const MvString& Scale,
                                    const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, *this, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvCamPar::ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                                    std::int64_t Height, const char* Scale, const char* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, *this, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvCamPar::ImageToWorldPlane(const MvImage& Image, const MvPose& WorldPose, std::int64_t Width,
                                    std::int64_t Height, const wchar_t* Scale,
                                    const wchar_t* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, *this, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvImage::ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                                   std::int64_t Height, const MvTuple& Scale,
                                   const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(*this, &result, CameraParam, WorldPose, Width, Height, Scale,
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvImage::ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                                   std::int64_t Height, const MvString& Scale,
                                   const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(*this, &result, CameraParam, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvImage::ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                                   std::int64_t Height, const char* Scale, const char* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(*this, &result, CameraParam, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvImage::ImageToWorldPlane(const MvCamPar& CameraParam, const MvPose& WorldPose, std::int64_t Width,
                                   std::int64_t Height, const wchar_t* Scale,
                                   const wchar_t* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(*this, &result, CameraParam, WorldPose, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvPose::ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                                  std::int64_t Height, const MvTuple& Scale,
                                  const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, CameraParam, *this, Width, Height, Scale,
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvPose::ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                                  std::int64_t Height, const MvString& Scale,
                                  const MvString& Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, CameraParam, *this, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvPose::ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                                  std::int64_t Height, const char* Scale, const char* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, CameraParam, *this, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
MvImage MvPose::ImageToWorldPlane(const MvImage& Image, const MvCamPar& CameraParam, std::int64_t Width,
                                  std::int64_t Height, const wchar_t* Scale,
                                  const wchar_t* Interpolation) const {
  MvObject result;
  mv::ImageToWorldPlane(Image, &result, CameraParam, *this, Width, Height, MvTuple(MvString(Scale)),
                        MvTuple(MvString(Interpolation)));
  return MvImage(result);
}
}  // namespace mv

namespace mv {

void VectorToProjHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u, const MvTuple& v,
                          const MvTuple& method, const MvTuple& xx1, const MvTuple& yy1, const MvTuple& xy1,
                          const MvTuple& xx2, const MvTuple& yy2, const MvTuple& xy2, MvTuple* h,
                          MvTuple* covariance) {
  cpp_api::TupleOutputs<2> outputs("VectorToProjHomMat2d", {h, covariance});
  cpp_api::check_status(
      mv_vector_to_proj_hom_mat2d(x.ControlInput(), y.ControlInput(), u.ControlInput(), v.ControlInput(),
                                  method.ControlInput(), xx1.ControlInput(), yy1.ControlInput(),
                                  xy1.ControlInput(), xx2.ControlInput(), yy2.ControlInput(),
                                  xy2.ControlInput(), outputs.Out(0), outputs.Out(1)),
      "VectorToProjHomMat2d");
  outputs.Commit();
}
MvTuple MvHomMat2D::VectorToProjHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u,
                                         const MvTuple& v, const MvString& method, const MvTuple& xx1,
                                         const MvTuple& yy1, const MvTuple& xy1, const MvTuple& xx2,
                                         const MvTuple& yy2, const MvTuple& xy2) {
  MvTuple result, covariance;
  mv::VectorToProjHomMat2d(x, y, u, v, method, xx1, yy1, xy1, xx2, yy2, xy2, &result, &covariance);
  MvTuple::operator=(std::move(result));
  return covariance;
}
MvTuple MvHomMat2D::VectorToProjHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u,
                                         const MvTuple& v, const char* method, const MvTuple& xx1,
                                         const MvTuple& yy1, const MvTuple& xy1, const MvTuple& xx2,
                                         const MvTuple& yy2, const MvTuple& xy2) {
  return VectorToProjHomMat2d(x, y, u, v, MvString(method), xx1, yy1, xy1, xx2, yy2, xy2);
}
MvTuple MvHomMat2D::VectorToProjHomMat2d(const MvTuple& x, const MvTuple& y, const MvTuple& u,
                                         const MvTuple& v, const wchar_t* method, const MvTuple& xx1,
                                         const MvTuple& yy1, const MvTuple& xy1, const MvTuple& xx2,
                                         const MvTuple& yy2, const MvTuple& xy2) {
  return VectorToProjHomMat2d(x, y, u, v, MvString(method), xx1, yy1, xy1, xx2, yy2, xy2);
}
}  // namespace mv

namespace mv {
void VectorToPose(const MvTuple& x, const MvTuple& y, const MvTuple& z, const MvTuple& row,
                  const MvTuple& col, const MvTuple& camera, const MvTuple& method,
                  const MvTuple& qualityType, MvTuple* pose, MvTuple* quality) {
  cpp_api::TupleOutputs<2> outputs("VectorToPose", {pose, quality});
  cpp_api::check_status(
      mv_vector_to_pose(x.ControlInput(), y.ControlInput(), z.ControlInput(), row.ControlInput(),
                        col.ControlInput(), camera.ControlInput(), method.ControlInput(),
                        qualityType.ControlInput(), outputs.Out(0), outputs.Out(1)),
      "VectorToPose");
  outputs.Commit();
}

MvPose MvCamPar::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                              const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvString& Method,
                              const MvTuple& QualityType, MvTuple* Quality) const {
  cpp_api::require_output(Quality, "VectorToPose");
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, *this, Method, QualityType, &pose,
                   &quality);
  MvPose result(pose);
  *Quality = std::move(quality);
  return result;
}
MvPose MvCamPar::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                              const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvString& Method,
                              const MvString& QualityType, double* Quality) const {
  cpp_api::require_output(Quality, "VectorToPose");
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, *this, Method, QualityType, &pose,
                   &quality);
  const auto value = quality.D();
  MvPose result(pose);
  *Quality = value;
  return result;
}
MvPose MvCamPar::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                              const MvTuple& ImageRow, const MvTuple& ImageColumn, const char* Method,
                              const char* QualityType, double* Quality) const {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, MvString(Method), MvString(QualityType),
                      Quality);
}
MvPose MvCamPar::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                              const MvTuple& ImageRow, const MvTuple& ImageColumn, const wchar_t* Method,
                              const wchar_t* QualityType, double* Quality) const {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, MvString(Method), MvString(QualityType),
                      Quality);
}
MvPose MvImage::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const MvString& Method, const MvTuple& QualityType, MvTuple* Quality) {
  cpp_api::require_output(Quality, "VectorToPose");
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, Method, QualityType, &pose,
                   &quality);
  MvPose result(pose);
  *Quality = std::move(quality);
  return result;
}
MvPose MvImage::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const MvString& Method, const MvString& QualityType, double* Quality) {
  cpp_api::require_output(Quality, "VectorToPose");
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, Method, QualityType, &pose,
                   &quality);
  const auto value = quality.D();
  MvPose result(pose);
  *Quality = value;
  return result;
}
MvPose MvImage::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const char* Method, const char* QualityType, double* Quality) {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, MvString(Method),
                      MvString(QualityType), Quality);
}
MvPose MvImage::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const wchar_t* Method, const wchar_t* QualityType, double* Quality) {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, MvString(Method),
                      MvString(QualityType), Quality);
}
MvTuple MvPose::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                             const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                             const MvString& Method, const MvTuple& QualityType) {
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, Method, QualityType, &pose,
                   &quality);
  MvPose result(pose);
  *this = std::move(result);
  return quality;
}
double MvPose::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                            const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                            const MvString& Method, const MvString& QualityType) {
  MvTuple pose, quality;
  mv::VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, Method, QualityType, &pose,
                   &quality);
  const auto value = quality.D();
  MvPose result(pose);
  *this = std::move(result);
  return value;
}
double MvPose::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                            const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                            const char* Method, const char* QualityType) {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, MvString(Method),
                      MvString(QualityType));
}
double MvPose::VectorToPose(const MvTuple& WorldX, const MvTuple& WorldY, const MvTuple& WorldZ,
                            const MvTuple& ImageRow, const MvTuple& ImageColumn, const MvCamPar& CameraParam,
                            const wchar_t* Method, const wchar_t* QualityType) {
  return VectorToPose(WorldX, WorldY, WorldZ, ImageRow, ImageColumn, CameraParam, MvString(Method),
                      MvString(QualityType));
}
}  // namespace mv
