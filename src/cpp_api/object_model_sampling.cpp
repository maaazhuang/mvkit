#include <mvkit/c/mv_reconstruction3d.h>

#include <cstring>
#include <mvkit/cpp/mv_reconstruction3d.hpp>

#include "api_bridge.hpp"

namespace mv {
void SampleObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method, const MvTuple& SamplingParam,
                         const MvTuple& GenParamName, const MvTuple& GenParamValue,
                         MvTuple* SampledObjectModel3D) {
  cpp_api::TupleOutputs<1> output("SampleObjectModel3d", {SampledObjectModel3D});
  cpp_api::check_status(
      mv_sample_object_model_3d(ObjectModel3D.ControlInput(), Method.ControlInput(),
                                SamplingParam.ControlInput(), GenParamName.ControlInput(),
                                GenParamValue.ControlInput(), output.Out(0)),
      "SampleObjectModel3d");
  output.Commit();
}
void SurfaceNormalsObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method,
                                 const MvTuple& GenParamName, const MvTuple& GenParamValue,
                                 MvTuple* ObjectModel3DNormals) {
  cpp_api::TupleOutputs<1> output("SurfaceNormalsObjectModel3d", {ObjectModel3DNormals});
  cpp_api::check_status(mv_surface_normals_object_model_3d(
                               ObjectModel3D.ControlInput(), Method.ControlInput(),
                               GenParamName.ControlInput(), GenParamValue.ControlInput(), output.Out(0)),
                           "SurfaceNormalsObjectModel3d");
  output.Commit();
}

namespace {
MvObjectModel3D single_model(const MvTuple& values, const char* op) {
  if (values.Length() != 1)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidCardinality, {op, std::strlen(op)},
                      "ObjectModel3D", -1, "exactly one result model required");
  return MvObjectModel3D(values.H());
}
MvObjectModel3DArray model_array(const MvTuple& values) {
  MvObjectModel3DArray array;
  array.SetFromTuple(values);
  return array;
}
}  // namespace
MvObjectModel3DArray MvObjectModel3D::SampleObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                          const MvString& Method,
                                                          const MvTuple& SamplingParam,
                                                          const MvTuple& GenParamName,
                                                          const MvTuple& GenParamValue) {
  MvTuple output;
  mv::SampleObjectModel3d(ObjectModel3D.ConvertToTuple(), Method, SamplingParam, GenParamName, GenParamValue,
                          &output);
  return model_array(output);
}
MvObjectModel3D MvObjectModel3D::SampleObjectModel3d(const MvString& Method, double SamplingParam,
                                                     const MvTuple& GenParamName,
                                                     const MvTuple& GenParamValue) const {
  MvTuple output;
  mv::SampleObjectModel3d(MvTuple(*this), Method, SamplingParam, GenParamName, GenParamValue, &output);
  return single_model(output, "SampleObjectModel3d");
}
MvObjectModel3D MvObjectModel3D::SampleObjectModel3d(const char* Method, double SamplingParam,
                                                     const MvTuple& GenParamName,
                                                     const MvTuple& GenParamValue) const {
  return SampleObjectModel3d(MvString(Method), SamplingParam, GenParamName, GenParamValue);
}
MvObjectModel3D MvObjectModel3D::SampleObjectModel3d(const wchar_t* Method, double SamplingParam,
                                                     const MvTuple& GenParamName,
                                                     const MvTuple& GenParamValue) const {
  return SampleObjectModel3d(MvString(Method), SamplingParam, GenParamName, GenParamValue);
}
MvObjectModel3DArray MvObjectModel3D::SurfaceNormalsObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                                  const MvString& Method,
                                                                  const MvTuple& GenParamName,
                                                                  const MvTuple& GenParamValue) {
  MvTuple output;
  mv::SurfaceNormalsObjectModel3d(ObjectModel3D.ConvertToTuple(), Method, GenParamName, GenParamValue,
                                  &output);
  return model_array(output);
}
MvObjectModel3D MvObjectModel3D::SurfaceNormalsObjectModel3d(const MvString& Method,
                                                             const MvTuple& GenParamName,
                                                             const MvTuple& GenParamValue) const {
  MvTuple output;
  mv::SurfaceNormalsObjectModel3d(MvTuple(*this), Method, GenParamName, GenParamValue, &output);
  return single_model(output, "SurfaceNormalsObjectModel3d");
}
MvObjectModel3D MvObjectModel3D::SurfaceNormalsObjectModel3d(const char* Method, const MvTuple& GenParamName,
                                                             const MvTuple& GenParamValue) const {
  return SurfaceNormalsObjectModel3d(MvString(Method), GenParamName, GenParamValue);
}
MvObjectModel3D MvObjectModel3D::SurfaceNormalsObjectModel3d(const wchar_t* Method,
                                                             const MvTuple& GenParamName,
                                                             const MvTuple& GenParamValue) const {
  return SurfaceNormalsObjectModel3d(MvString(Method), GenParamName, GenParamValue);
}
}  // namespace mv
