#include <mvkit/c/mv_reconstruction3d.h>

#include <mvkit/cpp/mv_reconstruction3d.hpp>

#include "api_bridge.hpp"
namespace mv {
void TriangulateObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& Method,
                              const MvTuple& GenParamName, const MvTuple& GenParamValue,
                              MvTuple* TriangulatedObjectModel3D, MvTuple* Information) {
  cpp_api::TupleOutputs<2> outputs("TriangulateObjectModel3d", {TriangulatedObjectModel3D, Information});
  cpp_api::check_status(
      mv_triangulate_object_model_3d(ObjectModel3D.ControlInput(), Method.ControlInput(),
                                     GenParamName.ControlInput(), GenParamValue.ControlInput(),
                                     outputs.Out(0), outputs.Out(1)),
      "TriangulateObjectModel3d");
  outputs.Commit();
}
MvObjectModel3DArray MvObjectModel3D::TriangulateObjectModel3d(const MvObjectModel3DArray& models,
                                                               const MvString& method, const MvTuple& names,
                                                               const MvTuple& values, MvTuple* information) {
  if (!information)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::NullArgument, "TriangulateObjectModel3d",
                      "Information", -1, "Information must not be null");
  MvTuple output, info;
  mv::TriangulateObjectModel3d(models.ConvertToTuple(), method, names, values, &output, &info);
  MvObjectModel3DArray result;
  result.SetFromTuple(output);
  *information = std::move(info);
  return result;
}
MvObjectModel3D MvObjectModel3D::TriangulateObjectModel3d(const MvString& method, const MvTuple& names,
                                                          const MvTuple& values,
                                                          std::int64_t* information) const {
  if (!information)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::NullArgument, "TriangulateObjectModel3d",
                      "Information", -1, "Information must not be null");
  MvTuple output, info;
  mv::TriangulateObjectModel3d(MvTuple(*this), method, names, values, &output, &info);
  MvObjectModel3D result(output.H());
  const auto count = info.I();
  *information = count;
  return result;
}
MvObjectModel3D MvObjectModel3D::TriangulateObjectModel3d(const char* method, const MvTuple& names,
                                                          const MvTuple& values,
                                                          std::int64_t* information) const {
  return TriangulateObjectModel3d(MvString(method), names, values, information);
}
MvObjectModel3D MvObjectModel3D::TriangulateObjectModel3d(const wchar_t* method, const MvTuple& names,
                                                          const MvTuple& values,
                                                          std::int64_t* information) const {
  return TriangulateObjectModel3d(MvString(method), names, values, information);
}
}  // namespace mv
