#include "api_bridge.hpp"
#include <mvkit/c/mv_reconstruction3d.h>

#include <atomic>
#include <cstring>
#include <limits>
#include <memory>
#include <mvkit/cpp/mv_calibration.hpp>
#include <mvkit/cpp/mv_reconstruction3d.hpp>
#include <utility>
#include <vector>

namespace mv {
void GenObjectModel3dFromPoints(const MvTuple& X, const MvTuple& Y, const MvTuple& Z,
                                MvTuple* ObjectModel3D) {
  cpp_api::TupleOutputs<1> output("GenObjectModel3dFromPoints", {ObjectModel3D});
  cpp_api::check_status(
      mv_gen_object_model_3d_from_points(X.ControlInput(), Y.ControlInput(), Z.ControlInput(), output.Out(0)),
      "GenObjectModel3dFromPoints");
  output.Commit();
}
void XyzToObjectModel3d(const MvObject& X, const MvObject& Y, const MvObject& Z, MvTuple* ObjectModel3D) {
  cpp_api::TupleOutputs<1> output("XyzToObjectModel3d", {ObjectModel3D});
  cpp_api::check_status(
      mv_xyz_to_object_model_3d(X.NativeHandle(), Y.NativeHandle(), Z.NativeHandle(), output.Out(0)),
      "XyzToObjectModel3d");
  output.Commit();
}

void GetObjectModel3dParams(const MvTuple& ObjectModel3D, const MvTuple& GenParamName,
                            MvTuple* GenParamValue) {
  cpp_api::TupleOutputs<1> output("GetObjectModel3dParams", {GenParamValue});
  cpp_api::check_status(
      mv_get_object_model_3d_params(ObjectModel3D.ControlInput(), GenParamName.ControlInput(), output.Out(0)),
      "GetObjectModel3dParams");
  output.Commit();
}
void RigidTransObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Pose,
                             MvTuple* ObjectModel3DRigidTrans) {
  cpp_api::TupleOutputs<1> output("RigidTransObjectModel3d", {ObjectModel3DRigidTrans});
  cpp_api::check_status(
      mv_rigid_trans_object_model_3d(ObjectModel3D.ControlInput(), Pose.ControlInput(), output.Out(0)),
      "RigidTransObjectModel3d");
  output.Commit();
}
void AffineTransObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& HomMat3D,
                              MvTuple* ObjectModel3DAffineTrans) {
  cpp_api::TupleOutputs<1> output("AffineTransObjectModel3d", {ObjectModel3DAffineTrans});
  cpp_api::check_status(
      mv_affine_trans_object_model_3d(ObjectModel3D.ControlInput(), HomMat3D.ControlInput(), output.Out(0)),
      "AffineTransObjectModel3d");
  output.Commit();
}
namespace {
MvObjectModel3D singleton_model(const MvTuple& values, const char* operation) {
  if (values.Length() != 1)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidCardinality,
                      MvStringView{operation, std::strlen(operation)}, "ObjectModel3D", -1,
                      "exactly one result model required");
  return MvObjectModel3D(values.H());
}
}  // namespace
void MvObjectModel3D::GenObjectModel3dFromPoints(const MvTuple& X, const MvTuple& Y, const MvTuple& Z) {
  MvTuple result;
  mv::GenObjectModel3dFromPoints(X, Y, Z, &result);
  *this = singleton_model(result, "GenObjectModel3dFromPoints");
}
void MvObjectModel3D::GenObjectModel3dFromPoints(double X, double Y, double Z) {
  GenObjectModel3dFromPoints(MvTuple(X), MvTuple(Y), MvTuple(Z));
}
void MvObjectModel3D::XyzToObjectModel3d(const MvImage& X, const MvImage& Y, const MvImage& Z) {
  MvTuple result;
  mv::XyzToObjectModel3d(X, Y, Z, &result);
  *this = singleton_model(result, "XyzToObjectModel3d");
}
MvTuple MvObjectModel3D::GetObjectModel3dParams(const MvTuple& GenParamName) const {
  MvTuple result;
  mv::GetObjectModel3dParams(MvTuple(*this), GenParamName, &result);
  return result;
}
MvTuple MvObjectModel3D::GetObjectModel3dParams(const MvObjectModel3DArray& models,
                                                const MvTuple& GenParamName) {
  MvTuple result;
  mv::GetObjectModel3dParams(models.ConvertToTuple(), GenParamName, &result);
  return result;
}
MvObjectModel3D MvObjectModel3D::RigidTransObjectModel3d(const MvPose& Pose) const {
  MvTuple result;
  mv::RigidTransObjectModel3d(MvTuple(*this), Pose, &result);
  return singleton_model(result, "RigidTransObjectModel3d");
}
MvObjectModel3DArray MvObjectModel3D::RigidTransObjectModel3d(const MvObjectModel3DArray& models,
                                                              const MvPoseArray& Pose) {
  MvTuple result;
  mv::RigidTransObjectModel3d(models.ConvertToTuple(), Pose.ConvertToTuple(), &result);
  MvObjectModel3DArray array;
  array.SetFromTuple(result);
  return array;
}
MvObjectModel3D MvObjectModel3D::AffineTransObjectModel3d(const MvHomMat3D& HomMat3D) const {
  MvTuple result;
  mv::AffineTransObjectModel3d(MvTuple(*this), HomMat3D, &result);
  return singleton_model(result, "AffineTransObjectModel3d");
}
MvObjectModel3DArray MvObjectModel3D::AffineTransObjectModel3d(const MvObjectModel3DArray& models,
                                                               const MvHomMat3DArray& HomMat3D) {
  MvTuple result;
  mv::AffineTransObjectModel3d(models.ConvertToTuple(), HomMat3D.ConvertToTuple(), &result);
  MvObjectModel3DArray array;
  array.SetFromTuple(result);
  return array;
}
}  // namespace mv

namespace mv {
struct MvObjectModel3DArrayData {
  std::atomic<std::size_t> refs{1};
  std::vector<MvObjectModel3D> values;
};
MvObjectModel3DArray::MvObjectModel3DArray() noexcept = default;
MvObjectModel3DArray::MvObjectModel3DArray(const MvObjectModel3D* values, std::int64_t length) {
  if (length < 0 ||
      std::uint64_t(length) >
          std::uint64_t(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(MvObjectModel3D) ||
      (length && !values))
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, "MvObjectModel3DArray", "values", -1,
                      "nonnegative length and valid array required");
  if (length) {
    auto next = std::make_unique<MvObjectModel3DArrayData>();
    next->values.assign(values, values + length);
    data_ = next.release();
  }
}
MvObjectModel3DArray::MvObjectModel3DArray(const MvObjectModel3DArray& other) : data_(other.data_) {
  if (data_) data_->refs.fetch_add(1, std::memory_order_relaxed);
}
MvObjectModel3DArray::MvObjectModel3DArray(MvObjectModel3DArray&& other) noexcept
    : data_(std::exchange(other.data_, nullptr)) {}
MvObjectModel3DArray& MvObjectModel3DArray::operator=(const MvObjectModel3DArray& other) {
  if (this != &other) {
    auto next = other.data_;
    if (next) next->refs.fetch_add(1, std::memory_order_relaxed);
    Clear();
    data_ = next;
  }
  return *this;
}
MvObjectModel3DArray& MvObjectModel3DArray::operator=(MvObjectModel3DArray&& other) noexcept {
  if (this != &other) {
    Clear();
    data_ = std::exchange(other.data_, nullptr);
  }
  return *this;
}
MvObjectModel3DArray::~MvObjectModel3DArray() noexcept { Clear(); }
void MvObjectModel3DArray::Clear() noexcept {
  auto old = std::exchange(data_, nullptr);
  if (old && old->refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete old;
}
const MvObjectModel3D* MvObjectModel3DArray::Tools() const noexcept {
  return data_ ? data_->values.data() : nullptr;
}
std::int64_t MvObjectModel3DArray::Length() const noexcept {
  return data_ ? std::int64_t(data_->values.size()) : 0;
}
void MvObjectModel3DArray::SetFromTuple(const MvTuple& values) {
  if (!values.Length()) {
    Clear();
    return;
  }
  auto next = std::make_unique<MvObjectModel3DArrayData>();
  next->values.reserve(values.Length());
  for (std::size_t i = 0; i < values.Length(); ++i) next->values.emplace_back(values.H(i));
  Clear();
  data_ = next.release();
}
MvTuple MvObjectModel3DArray::ConvertToTuple() const {
  MvTuple result;
  if (data_)
    for (const auto& model : data_->values) result.Append(model);
  return result;
}
struct MvPoseArrayData {
  std::atomic<std::size_t> refs{1};
  std::vector<MvPose> values;
};
MvPoseArray::MvPoseArray() noexcept = default;
MvPoseArray::MvPoseArray(const MvPose* values, std::int64_t length) {
  if (length < 0 ||
      std::uint64_t(length) > std::uint64_t(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(MvPose) ||
      (length && !values))
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, "MvPoseArray", "values", -1,
                      "nonnegative length and valid array required");
  if (length) {
    auto next = std::make_unique<MvPoseArrayData>();
    next->values.assign(values, values + length);
    data_ = next.release();
  }
}
MvPoseArray::MvPoseArray(const MvPoseArray& other) : data_(other.data_) {
  if (data_) data_->refs.fetch_add(1, std::memory_order_relaxed);
}
MvPoseArray::MvPoseArray(MvPoseArray&& other) noexcept : data_(std::exchange(other.data_, nullptr)) {}
MvPoseArray& MvPoseArray::operator=(const MvPoseArray& other) {
  if (this != &other) {
    auto next = other.data_;
    if (next) next->refs.fetch_add(1, std::memory_order_relaxed);
    Clear();
    data_ = next;
  }
  return *this;
}
MvPoseArray& MvPoseArray::operator=(MvPoseArray&& other) noexcept {
  if (this != &other) {
    Clear();
    data_ = std::exchange(other.data_, nullptr);
  }
  return *this;
}
MvPoseArray::~MvPoseArray() noexcept { Clear(); }
void MvPoseArray::Clear() noexcept {
  auto old = std::exchange(data_, nullptr);
  if (old && old->refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete old;
}
const MvPose* MvPoseArray::Tools() const noexcept { return data_ ? data_->values.data() : nullptr; }
std::int64_t MvPoseArray::Length() const noexcept { return data_ ? std::int64_t(data_->values.size()) : 0; }
void MvPoseArray::SetFromTuple(const MvTuple& values) {
  if (!values.Length()) {
    Clear();
    return;
  }
  auto next = std::make_unique<MvPoseArrayData>();
  if (values.Length() % 7)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidCardinality, "MvPoseArray::SetFromTuple",
                      "values", -1, "complete numeric groups required");
  next->values.reserve(values.Length() / 7);
  for (std::size_t i = 0; i < values.Length(); i += 7)
    next->values.emplace_back(values.TupleSelectRange(std::int64_t(i), std::int64_t(i + 7 - 1)));
  Clear();
  data_ = next.release();
}
MvTuple MvPoseArray::ConvertToTuple() const {
  MvTuple result;
  if (data_)
    for (const auto& value : data_->values)
      for (std::size_t i = 0; i < value.Length(); ++i) result.Append(value.D(i));
  return result;
}
struct MvHomMat3DArrayData {
  std::atomic<std::size_t> refs{1};
  std::vector<MvHomMat3D> values;
};
MvHomMat3DArray::MvHomMat3DArray() noexcept = default;
MvHomMat3DArray::MvHomMat3DArray(const MvHomMat3D* values, std::int64_t length) {
  if (length < 0 ||
      std::uint64_t(length) >
          std::uint64_t(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(MvHomMat3D) ||
      (length && !values))
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, "MvHomMat3DArray", "values", -1,
                      "nonnegative length and valid array required");
  if (length) {
    auto next = std::make_unique<MvHomMat3DArrayData>();
    next->values.assign(values, values + length);
    data_ = next.release();
  }
}
MvHomMat3DArray::MvHomMat3DArray(const MvHomMat3DArray& other) : data_(other.data_) {
  if (data_) data_->refs.fetch_add(1, std::memory_order_relaxed);
}
MvHomMat3DArray::MvHomMat3DArray(MvHomMat3DArray&& other) noexcept
    : data_(std::exchange(other.data_, nullptr)) {}
MvHomMat3DArray& MvHomMat3DArray::operator=(const MvHomMat3DArray& other) {
  if (this != &other) {
    auto next = other.data_;
    if (next) next->refs.fetch_add(1, std::memory_order_relaxed);
    Clear();
    data_ = next;
  }
  return *this;
}
MvHomMat3DArray& MvHomMat3DArray::operator=(MvHomMat3DArray&& other) noexcept {
  if (this != &other) {
    Clear();
    data_ = std::exchange(other.data_, nullptr);
  }
  return *this;
}
MvHomMat3DArray::~MvHomMat3DArray() noexcept { Clear(); }
void MvHomMat3DArray::Clear() noexcept {
  auto old = std::exchange(data_, nullptr);
  if (old && old->refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete old;
}
const MvHomMat3D* MvHomMat3DArray::Tools() const noexcept { return data_ ? data_->values.data() : nullptr; }
std::int64_t MvHomMat3DArray::Length() const noexcept {
  return data_ ? std::int64_t(data_->values.size()) : 0;
}
void MvHomMat3DArray::SetFromTuple(const MvTuple& values) {
  if (!values.Length()) {
    Clear();
    return;
  }
  auto next = std::make_unique<MvHomMat3DArrayData>();
  if (values.Length() % 12)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidCardinality, "MvHomMat3DArray::SetFromTuple",
                      "values", -1, "complete numeric groups required");
  next->values.reserve(values.Length() / 12);
  for (std::size_t i = 0; i < values.Length(); i += 12)
    next->values.emplace_back(values.TupleSelectRange(std::int64_t(i), std::int64_t(i + 12 - 1)));
  Clear();
  data_ = next.release();
}
MvTuple MvHomMat3DArray::ConvertToTuple() const {
  MvTuple result;
  if (data_)
    for (const auto& value : data_->values)
      for (std::size_t i = 0; i < value.Length(); ++i) result.Append(value.D(i));
  return result;
}
}  // namespace mv

namespace mv {
void SelectPointsObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Attrib, const MvTuple& MinValue,
                               const MvTuple& MaxValue, MvTuple* ObjectModel3DThresholded) {
  cpp_api::TupleOutputs<1> result("SelectPointsObjectModel3d", {ObjectModel3DThresholded});
  cpp_api::check_status(
      mv_select_points_object_model_3d(ObjectModel3D.ControlInput(), Attrib.ControlInput(),
                                       MinValue.ControlInput(), MaxValue.ControlInput(), result.Out(0)),
      "SelectPointsObjectModel3d");
  result.Commit();
}
void ConnectionObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Feature, const MvTuple& Value,
                             MvTuple* ObjectModel3DConnected) {
  cpp_api::TupleOutputs<1> result("ConnectionObjectModel3d", {ObjectModel3DConnected});
  cpp_api::check_status(mv_connection_object_model_3d(ObjectModel3D.ControlInput(), Feature.ControlInput(),
                                                         Value.ControlInput(), result.Out(0)),
                           "ConnectionObjectModel3d");
  result.Commit();
}
void IntersectPlaneObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Plane,
                                 MvTuple* ObjectModel3DIntersection) {
  cpp_api::TupleOutputs<1> result("IntersectPlaneObjectModel3d", {ObjectModel3DIntersection});
  cpp_api::check_status(
      mv_intersect_plane_object_model_3d(ObjectModel3D.ControlInput(), Plane.ControlInput(), result.Out(0)),
      "IntersectPlaneObjectModel3d");
  result.Commit();
}
void ReadObjectModel3d(const MvTuple& FileName, const MvTuple& Scale, const MvTuple& GenParamName,
                       const MvTuple& GenParamValue, MvTuple* ObjectModel3D, MvTuple* MvStatus) {
  cpp_api::TupleOutputs<2> result("ReadObjectModel3d", {ObjectModel3D, MvStatus});
  cpp_api::check_status(
      mv_read_object_model_3d(FileName.ControlInput(), Scale.ControlInput(), GenParamName.ControlInput(),
                              GenParamValue.ControlInput(), result.Out(0), result.Out(1)),
      "ReadObjectModel3d");
  result.Commit();
}
void WriteObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& FileType, const MvTuple& FileName,
                        const MvTuple& GenParamName, const MvTuple& GenParamValue) {
  cpp_api::check_status(
      mv_write_object_model_3d(ObjectModel3D.ControlInput(), FileType.ControlInput(), FileName.ControlInput(),
                               GenParamName.ControlInput(), GenParamValue.ControlInput()),
      "WriteObjectModel3d");
}
namespace {
MvObjectModel3DArray model_array(const MvTuple& handles) {
  MvObjectModel3DArray result;
  result.SetFromTuple(handles);
  return result;
}
}  // namespace
MvObjectModel3DArray MvObjectModel3D::SelectPointsObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                                const MvTuple& Attrib,
                                                                const MvTuple& MinValue,
                                                                const MvTuple& MaxValue) {
  MvTuple result;
  mv::SelectPointsObjectModel3d(ObjectModel3D.ConvertToTuple(), Attrib, MinValue, MaxValue, &result);
  return model_array(result);
}
MvObjectModel3DArray MvObjectModel3D::ConnectionObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                              const MvTuple& Feature, const MvTuple& Value) {
  MvTuple result;
  mv::ConnectionObjectModel3d(ObjectModel3D.ConvertToTuple(), Feature, Value, &result);
  return model_array(result);
}
MvObjectModel3DArray MvObjectModel3D::IntersectPlaneObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                                  const MvPoseArray& Plane) {
  MvTuple result;
  mv::IntersectPlaneObjectModel3d(ObjectModel3D.ConvertToTuple(), Plane.ConvertToTuple(), &result);
  return model_array(result);
}
MvObjectModel3D MvObjectModel3D::IntersectPlaneObjectModel3d(const MvPose& Plane) const {
  MvTuple result;
  mv::IntersectPlaneObjectModel3d(MvTuple(*this), Plane, &result);
  return singleton_model(result, "IntersectPlaneObjectModel3d");
}
MvTuple MvObjectModel3D::ReadObjectModel3d(const MvString& FileName, const MvTuple& Scale,
                                           const MvTuple& GenParamName, const MvTuple& GenParamValue) {
  MvTuple result, status;
  mv::ReadObjectModel3d(FileName, Scale, GenParamName, GenParamValue, &result, &status);
  auto replacement = singleton_model(result, "ReadObjectModel3d");
  *this = std::move(replacement);
  return status;
}
MvTuple MvObjectModel3D::ReadObjectModel3d(const MvString& FileName, const MvTuple& Scale,
                                           const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                           MvObjectModel3DArray* ObjectModel3D) {
  cpp_api::require_output(ObjectModel3D, "ReadObjectModel3d");
  MvTuple result, status;
  mv::ReadObjectModel3d(FileName, Scale, GenParamName, GenParamValue, &result, &status);
  auto replacement = model_array(result);
  *ObjectModel3D = std::move(replacement);
  return status;
}
}  // namespace mv

namespace mv {
MvObjectModel3D MvObjectModel3D::SelectPointsObjectModel3d(const MvString& Attrib, double MinValue,
                                                           double MaxValue) const {
  MvTuple result;
  mv::SelectPointsObjectModel3d(MvTuple(*this), MvString(Attrib), MinValue, MaxValue, &result);
  return singleton_model(result, "SelectPointsObjectModel3d");
}
MvObjectModel3DArray MvObjectModel3D::ConnectionObjectModel3d(const MvString& Feature, double Value) const {
  MvTuple result;
  mv::ConnectionObjectModel3d(MvTuple(*this), MvString(Feature), Value, &result);
  return model_array(result);
}
MvString MvObjectModel3D::ReadObjectModel3d(const MvString& FileName, const MvString& Scale,
                                            const MvString& GenParamName, const MvString& GenParamValue) {
  const auto status = ReadObjectModel3d(MvString(FileName), MvTuple(MvString(Scale)),
                                        MvTuple(MvString(GenParamName)), MvTuple(MvString(GenParamValue)));
  return status.S();
}
void MvObjectModel3D::WriteObjectModel3d(const MvString& FileType, const MvString& FileName,
                                         const MvString& GenParamName, const MvString& GenParamValue) const {
  mv::WriteObjectModel3d(MvTuple(*this), MvString(FileType), MvString(FileName), MvString(GenParamName),
                         MvString(GenParamValue));
}
}  // namespace mv

namespace mv {
MvObjectModel3D MvObjectModel3D::SelectPointsObjectModel3d(const char* Attrib, double MinValue,
                                                           double MaxValue) const {
  MvTuple result;
  mv::SelectPointsObjectModel3d(MvTuple(*this), MvString(Attrib), MinValue, MaxValue, &result);
  return singleton_model(result, "SelectPointsObjectModel3d");
}
MvObjectModel3DArray MvObjectModel3D::ConnectionObjectModel3d(const char* Feature, double Value) const {
  MvTuple result;
  mv::ConnectionObjectModel3d(MvTuple(*this), MvString(Feature), Value, &result);
  return model_array(result);
}
MvString MvObjectModel3D::ReadObjectModel3d(const char* FileName, const char* Scale, const char* GenParamName,
                                            const char* GenParamValue) {
  const auto status = ReadObjectModel3d(MvString(FileName), MvTuple(MvString(Scale)),
                                        MvTuple(MvString(GenParamName)), MvTuple(MvString(GenParamValue)));
  return status.S();
}
void MvObjectModel3D::WriteObjectModel3d(const char* FileType, const char* FileName, const char* GenParamName,
                                         const char* GenParamValue) const {
  mv::WriteObjectModel3d(MvTuple(*this), MvString(FileType), MvString(FileName), MvString(GenParamName),
                         MvString(GenParamValue));
}
}  // namespace mv

namespace mv {
MvObjectModel3D MvObjectModel3D::SelectPointsObjectModel3d(const wchar_t* Attrib, double MinValue,
                                                           double MaxValue) const {
  MvTuple result;
  mv::SelectPointsObjectModel3d(MvTuple(*this), MvString(Attrib), MinValue, MaxValue, &result);
  return singleton_model(result, "SelectPointsObjectModel3d");
}
MvObjectModel3DArray MvObjectModel3D::ConnectionObjectModel3d(const wchar_t* Feature, double Value) const {
  MvTuple result;
  mv::ConnectionObjectModel3d(MvTuple(*this), MvString(Feature), Value, &result);
  return model_array(result);
}
MvString MvObjectModel3D::ReadObjectModel3d(const wchar_t* FileName, const wchar_t* Scale,
                                            const wchar_t* GenParamName, const wchar_t* GenParamValue) {
  const auto status = ReadObjectModel3d(MvString(FileName), MvTuple(MvString(Scale)),
                                        MvTuple(MvString(GenParamName)), MvTuple(MvString(GenParamValue)));
  return status.S();
}
void MvObjectModel3D::WriteObjectModel3d(const wchar_t* FileType, const wchar_t* FileName,
                                         const wchar_t* GenParamName, const wchar_t* GenParamValue) const {
  mv::WriteObjectModel3d(MvTuple(*this), MvString(FileType), MvString(FileName), MvString(GenParamName),
                         MvString(GenParamValue));
}
}  // namespace mv

namespace mv {
void MvObjectModel3D::WriteObjectModel3d(const MvString& FileType, const MvString& FileName,
                                         const MvTuple& GenParamName, const MvTuple& GenParamValue) const {
  mv::WriteObjectModel3d(MvTuple(*this), FileType, FileName, GenParamName, GenParamValue);
}
}  // namespace mv

namespace mv {
MvCameraSetupModel::MvCameraSetupModel() noexcept = default;
MvCameraSetupModel::MvCameraSetupModel(const MvCameraSetupModel&) = default;
MvCameraSetupModel::MvCameraSetupModel(MvCameraSetupModel&&) noexcept = default;
MvCameraSetupModel& MvCameraSetupModel::operator=(const MvCameraSetupModel&) = default;
MvCameraSetupModel& MvCameraSetupModel::operator=(MvCameraSetupModel&&) noexcept = default;
MvCameraSetupModel::~MvCameraSetupModel() noexcept = default;
MvCameraSetupModel::MvCameraSetupModel(const MvHandle& handle) : MvHandle(handle) {
  if (!IsInitialized() || Type() != MvObjectType::CameraSetupModel)
    throw MvException(MvStatus::TypeMismatch, MvErrorCode::InvalidType, "MvCameraSetupModel", "Handle", -1,
                      "wrong model type");
}
void CreateCameraSetupModel(const MvTuple& count, MvTuple* result) {
  cpp_api::TupleOutputs<1> out("CreateCameraSetupModel", {result});
  cpp_api::check_status(mv_create_camera_setup_model(count.ControlInput(), out.Out(0)),
                           "CreateCameraSetupModel");
  out.Commit();
}
void SetCameraSetupCamParam(const MvTuple& id, const MvTuple& index, const MvTuple& type,
                            const MvTuple& params, const MvTuple& pose) {
  cpp_api::check_status(
      mv_set_camera_setup_cam_param(id.ControlInput(), index.ControlInput(), type.ControlInput(),
                                    params.ControlInput(), pose.ControlInput()),
      "SetCameraSetupCamParam");
}
void GetCameraSetupParam(const MvTuple& id, const MvTuple& index, const MvTuple& name, MvTuple* result) {
  cpp_api::TupleOutputs<1> out("GetCameraSetupParam", {result});
  cpp_api::check_status(
      mv_get_camera_setup_param(id.ControlInput(), index.ControlInput(), name.ControlInput(), out.Out(0)),
      "GetCameraSetupParam");
  out.Commit();
}
void MvCameraSetupModel::CreateCameraSetupModel(std::int64_t count) {
  MvTuple result;
  mv::CreateCameraSetupModel(count, &result);
  MvHandle::operator=(result.H());
}
MvTuple MvCameraSetupModel::GetCameraSetupParam(const MvTuple& index, const MvString& name) const {
  MvTuple result;
  mv::GetCameraSetupParam(MvTuple(*this), index, MvTuple(name), &result);
  return result;
}
MvTuple MvCameraSetupModel::GetCameraSetupParam(std::int64_t index, const MvString& name) const {
  return GetCameraSetupParam(MvTuple(index), name);
}
MvTuple MvCameraSetupModel::GetCameraSetupParam(std::int64_t index, const char* name) const {
  return GetCameraSetupParam(index, MvString(name));
}
MvTuple MvCameraSetupModel::GetCameraSetupParam(std::int64_t index, const wchar_t* name) const {
  return GetCameraSetupParam(index, MvString(name));
}
void MvCameraSetupModel::SetCameraSetupCamParam(const MvTuple& index, const MvTuple& type,
                                                const MvCamPar& params, const MvTuple& pose) const {
  mv::SetCameraSetupCamParam(MvTuple(*this), index, type, params, pose);
}
void MvCameraSetupModel::SetCameraSetupCamParam(const MvTuple& index, const MvString& type,
                                                const MvCamPar& params, const MvTuple& pose) const {
  SetCameraSetupCamParam(index, MvTuple(MvString(type)), params, pose);
}
void MvCameraSetupModel::SetCameraSetupCamParam(const MvTuple& index, const char* type,
                                                const MvCamPar& params, const MvTuple& pose) const {
  SetCameraSetupCamParam(index, MvTuple(MvString(type)), params, pose);
}
void MvCameraSetupModel::SetCameraSetupCamParam(const MvTuple& index, const wchar_t* type,
                                                const MvCamPar& params, const MvTuple& pose) const {
  SetCameraSetupCamParam(index, MvTuple(MvString(type)), params, pose);
}
void MvCamPar::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                      const MvTuple& type, const MvTuple& pose) const {
  model.SetCameraSetupCamParam(index, type, *this, pose);
}
void MvPose::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                    const MvTuple& type, const MvCamPar& params, const MvTuple& pose) {
  model.SetCameraSetupCamParam(index, type, params, pose);
}
void MvCamPar::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                      const MvString& type, const MvTuple& pose) const {
  model.SetCameraSetupCamParam(index, type, *this, pose);
}
void MvPose::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                    const MvString& type, const MvCamPar& params, const MvTuple& pose) {
  model.SetCameraSetupCamParam(index, type, params, pose);
}
void MvCamPar::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index, const char* type,
                                      const MvTuple& pose) const {
  model.SetCameraSetupCamParam(index, type, *this, pose);
}
void MvPose::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index, const char* type,
                                    const MvCamPar& params, const MvTuple& pose) {
  model.SetCameraSetupCamParam(index, type, params, pose);
}
void MvCamPar::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                      const wchar_t* type, const MvTuple& pose) const {
  model.SetCameraSetupCamParam(index, type, *this, pose);
}
void MvPose::SetCameraSetupCamParam(const MvCameraSetupModel& model, const MvTuple& index,
                                    const wchar_t* type, const MvCamPar& params, const MvTuple& pose) {
  model.SetCameraSetupCamParam(index, type, params, pose);
}
}  // namespace mv

namespace mv {
namespace {
template <class Call>
void xyz_outputs(MvObject* x, MvObject* y, MvObject* z, Call&& call) {
  constexpr auto op = "ObjectModel3dToXyz";
  cpp_api::require_output(x, op);
  cpp_api::require_output(y, op);
  cpp_api::require_output(z, op);
  if (x == y || x == z || y == z)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, MvStringView{op, std::strlen(op)},
                      "Output", -1, "output pointers must be distinct");
  struct Owners {
    ::MvObject values[3]{};
    ~Owners() {
      for (auto value : values)
        if (value) (void)mv_clear_obj(value);
    }
  } owners;
  cpp_api::check_status(call(&owners.values[0], &owners.values[1], &owners.values[2]), op);
  auto sx = detail::MvObjectAccess::Adopt(std::exchange(owners.values[0], nullptr));
  auto sy = detail::MvObjectAccess::Adopt(std::exchange(owners.values[1], nullptr));
  auto sz = detail::MvObjectAccess::Adopt(std::exchange(owners.values[2], nullptr));
  detail::MvObjectAccess::CheckOutput(x, sx);
  detail::MvObjectAccess::CheckOutput(y, sy);
  detail::MvObjectAccess::CheckOutput(z, sz);
  detail::MvObjectAccess::Commit(x, std::move(sx));
  detail::MvObjectAccess::Commit(y, std::move(sy));
  detail::MvObjectAccess::Commit(z, std::move(sz));
}
MvImage xyz_member(MvImage* y, MvImage* z, const MvTuple& models, const MvString& type,
                   const MvCamPar& camera, const MvPose& pose) {
  MvImage x;
  mv::ObjectModel3dToXyz(&x, y, z, models, MvTuple(type), camera, pose);
  return x;
}
}  // namespace
void ObjectModel3dToXyz(MvObject* X, MvObject* Y, MvObject* Z, const MvTuple& models, const MvTuple& type,
                        const MvTuple& camera, const MvTuple& pose) {
  xyz_outputs(X, Y, Z, [&](::MvObject* x, ::MvObject* y, ::MvObject* z) {
    return mv_object_model_3d_to_xyz(x, y, z, models.ControlInput(), type.ControlInput(),
                                     camera.ControlInput(), pose.ControlInput());
  });
}

MvImage MvObjectModel3D::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3DArray& models,
                                            const MvString& type, const MvCamPar& camera,
                                            const MvPose& pose) {
  return xyz_member(Y, Z, models.ConvertToTuple(), type, camera, pose);
}
MvImage MvObjectModel3D::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvString& type,
                                            const MvCamPar& camera, const MvPose& pose) const {
  return xyz_member(Y, Z, MvTuple(*this), type, camera, pose);
}
MvImage MvObjectModel3D::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const char* type, const MvCamPar& camera,
                                            const MvPose& pose) const {
  return ObjectModel3dToXyz(Y, Z, MvString(type), camera, pose);
}
MvImage MvObjectModel3D::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const wchar_t* type,
                                            const MvCamPar& camera, const MvPose& pose) const {
  return ObjectModel3dToXyz(Y, Z, MvString(type), camera, pose);
}
MvImage MvCamPar::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3DArray& models,
                                     const MvString& type, const MvPose& pose) const {
  return xyz_member(Y, Z, models.ConvertToTuple(), type, *this, pose);
}
MvImage MvCamPar::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& model,
                                     const MvString& type, const MvPose& pose) const {
  return xyz_member(Y, Z, MvTuple(model), type, *this, pose);
}
MvImage MvCamPar::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& model, const char* type,
                                     const MvPose& pose) const {
  return ObjectModel3dToXyz(Y, Z, model, MvString(type), pose);
}
MvImage MvCamPar::ObjectModel3dToXyz(MvImage* Y, MvImage* Z, const MvObjectModel3D& model,
                                     const wchar_t* type, const MvPose& pose) const {
  return ObjectModel3dToXyz(Y, Z, model, MvString(type), pose);
}
}  // namespace mv
