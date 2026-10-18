#include <mvkit/c/mv_stereo.h>

#include <mvkit/cpp/mv_stereo.hpp>

#include "api_bridge.hpp"
namespace mv {
MvStereoModel::MvStereoModel() noexcept = default;
MvStereoModel::MvStereoModel(const MvStereoModel&) = default;
MvStereoModel::MvStereoModel(MvStereoModel&&) noexcept = default;
MvStereoModel& MvStereoModel::operator=(const MvStereoModel&) = default;
MvStereoModel& MvStereoModel::operator=(MvStereoModel&&) noexcept = default;
MvStereoModel::~MvStereoModel() noexcept = default;
MvStereoModel::MvStereoModel(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::StereoModel)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvStereoModel", "Handle", -1,
                      "wrong model type");
}
void CreateStereoModel(const MvTuple& setup, const MvTuple& method, const MvTuple& names,
                       const MvTuple& values, MvTuple* model) {
  cpp_api::TupleOutputs<1> out("CreateStereoModel", {model});
  cpp_api::check_status(mv_create_stereo_model(setup.ControlInput(), method.ControlInput(),
                                                  names.ControlInput(), values.ControlInput(), out.Out(0)),
                           "CreateStereoModel");
  out.Commit();
}
void SetStereoModelParam(const MvTuple& model, const MvTuple& names, const MvTuple& values) {
  cpp_api::check_status(
      mv_set_stereo_model_param(model.ControlInput(), names.ControlInput(), values.ControlInput()),
      "SetStereoModelParam");
}
void ClearStereoModel(const MvTuple& model) {
  cpp_api::check_status(mv_clear_stereo_model(model.ControlInput()), "ClearStereoModel");
}
void ReconstructPointsStereo(const MvTuple& model, const MvTuple& rows, const MvTuple& columns,
                             const MvTuple& covariance, const MvTuple& cameras, const MvTuple& points,
                             MvTuple* x, MvTuple* y, MvTuple* z, MvTuple* cov, MvTuple* indices) {
  cpp_api::TupleOutputs<5> out("ReconstructPointsStereo", {x, y, z, cov, indices});
  cpp_api::check_status(
      mv_reconstruct_points_stereo(model.ControlInput(), rows.ControlInput(), columns.ControlInput(),
                                   covariance.ControlInput(), cameras.ControlInput(), points.ControlInput(),
                                   out.Out(0), out.Out(1), out.Out(2), out.Out(3), out.Out(4)),
      "ReconstructPointsStereo");
  out.Commit();
}
void MvStereoModel::CreateStereoModel(const MvCameraSetupModel& setup, const MvString& method,
                                      const MvTuple& names, const MvTuple& values) {
  MvTuple out;
  mv::CreateStereoModel(MvTuple(setup), method, names, values, &out);
  MvHandle::operator=(out.H());
}
void MvStereoModel::CreateStereoModel(const MvCameraSetupModel& setup, const MvString& method,
                                      const MvString& names, const MvTuple& values) {
  CreateStereoModel(setup, method, MvTuple(names), values);
}
void MvStereoModel::CreateStereoModel(const MvCameraSetupModel& setup, const char* method, const char* names,
                                      const MvTuple& values) {
  CreateStereoModel(setup, MvString(method), MvTuple(names), values);
}
void MvStereoModel::CreateStereoModel(const MvCameraSetupModel& setup, const wchar_t* method,
                                      const wchar_t* names, const MvTuple& values) {
  CreateStereoModel(setup, MvString(method), MvTuple(MvString(names)), values);
}
MvStereoModel::MvStereoModel(const MvCameraSetupModel& setup, const MvString& method, const MvTuple& names,
                             const MvTuple& values) {
  CreateStereoModel(setup, method, names, values);
}
MvStereoModel::MvStereoModel(const MvCameraSetupModel& setup, const MvString& method, const MvString& names,
                             const MvTuple& values) {
  CreateStereoModel(setup, method, names, values);
}
MvStereoModel::MvStereoModel(const MvCameraSetupModel& setup, const char* method, const char* names,
                             const MvTuple& values) {
  CreateStereoModel(setup, method, names, values);
}
MvStereoModel::MvStereoModel(const MvCameraSetupModel& setup, const wchar_t* method, const wchar_t* names,
                             const MvTuple& values) {
  CreateStereoModel(setup, method, names, values);
}
void MvStereoModel::SetStereoModelParam(const MvTuple& names, const MvTuple& values) const {
  mv::SetStereoModelParam(MvTuple(*this), names, values);
}
void MvStereoModel::SetStereoModelParam(const MvString& names, const MvTuple& values) const {
  SetStereoModelParam(MvTuple(names), values);
}
void MvStereoModel::SetStereoModelParam(const char* names, const MvTuple& values) const {
  SetStereoModelParam(MvTuple(names), values);
}
void MvStereoModel::SetStereoModelParam(const wchar_t* names, const MvTuple& values) const {
  SetStereoModelParam(MvTuple(MvString(names)), values);
}
void MvStereoModel::ClearStereoModel() const { mv::ClearStereoModel(MvTuple(*this)); }
void MvStereoModel::ReconstructPointsStereo(const MvTuple& row, const MvTuple& column,
                                            const MvTuple& covariance, std::int64_t camera,
                                            std::int64_t point, MvTuple* x, MvTuple* y, MvTuple* z,
                                            MvTuple* cov, MvTuple* indices) const {
  mv::ReconstructPointsStereo(MvTuple(*this), row, column, covariance, camera, point, x, y, z, cov, indices);
}
void MvStereoModel::ReconstructPointsStereo(double row, double column, const MvTuple& covariance,
                                            std::int64_t camera, std::int64_t point, double* x, double* y,
                                            double* z, double* cov, std::int64_t* indices) const {
  constexpr char op[] = "ReconstructPointsStereo";
  const std::array<const void*, 5> destinations{{x, y, z, cov, indices}};
  for (std::size_t i = 0; i < destinations.size(); ++i) {
    cpp_api::require_output(destinations[i], op);
    const auto a = reinterpret_cast<std::uintptr_t>(destinations[i]);
    for (std::size_t j = 0; j < i; ++j) {
      const auto b = reinterpret_cast<std::uintptr_t>(destinations[j]);
      if ((a < b ? b - a : a - b) < sizeof(double))
        throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, op, "Output", -1,
                          "output ranges must be distinct");
    }
  }
  MvTuple xx, yy, zz, cc, ii;
  mv::ReconstructPointsStereo(MvTuple(*this), row, column, covariance, camera, point, &xx, &yy, &zz, &cc,
                              &ii);
  const auto vx = xx.D(), vy = yy.D(), vz = zz.D(), vc = cc.D();
  const auto vi = ii.I();
  *x = vx;
  *y = vy;
  *z = vz;
  *cov = vc;
  *indices = vi;
}
}  // namespace mv
