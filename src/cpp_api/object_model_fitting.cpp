#include <mvkit/c/mv_reconstruction3d.h>

#include <mvkit/cpp/mv_reconstruction3d.hpp>

#include "api_bridge.hpp"
namespace mv {
void FitPrimitivesObjectModel3d(const MvTuple& ObjectModel3D, const MvTuple& GenParamName,
                                const MvTuple& GenParamValue, MvTuple* ObjectModel3DOut) {
  cpp_api::TupleOutputs<1> output("FitPrimitivesObjectModel3d", {ObjectModel3DOut});
  cpp_api::check_status(
      mv_fit_primitives_object_model_3d(ObjectModel3D.ControlInput(), GenParamName.ControlInput(),
                                        GenParamValue.ControlInput(), output.Out(0)),
      "FitPrimitivesObjectModel3d");
  output.Commit();
}

MvObjectModel3DArray MvObjectModel3D::FitPrimitivesObjectModel3d(const MvObjectModel3DArray& ObjectModel3D,
                                                                 const MvTuple& GenParamName,
                                                                 const MvTuple& GenParamValue) {
  MvTuple output;
  mv::FitPrimitivesObjectModel3d(ObjectModel3D.ConvertToTuple(), GenParamName, GenParamValue, &output);
  MvObjectModel3DArray result;
  result.SetFromTuple(output);
  return result;
}
MvObjectModel3D MvObjectModel3D::FitPrimitivesObjectModel3d(const MvTuple& GenParamName,
                                                            const MvTuple& GenParamValue) const {
  MvTuple output;
  mv::FitPrimitivesObjectModel3d(MvTuple(*this), GenParamName, GenParamValue, &output);
  return MvObjectModel3D(output.H());
}
}  // namespace mv
