#include "api_bridge.hpp"
#include <mvkit/c/mv_stereo.h>

#include <array>
#include <mvkit/cpp/mv_calibration.hpp>
#include <mvkit/cpp/mv_geometry.hpp>
#include <utility>

namespace mv {
void GenBinocularRectificationMap(MvObject* Map1, MvObject* Map2, const MvTuple& CamParam1,
                                  const MvTuple& CamParam2, const MvTuple& RelPose,
                                  const MvTuple& SubSampling, const MvTuple& Method, const MvTuple& MapType,
                                  MvTuple* CamParamRect1, MvTuple* CamParamRect2, MvTuple* CamPoseRect1,
                                  MvTuple* CamPoseRect2, MvTuple* RelPoseRect) {
  constexpr char op[] = "GenBinocularRectificationMap";
  const std::array<const void*, 7> destinations{Map1,         Map2,         CamParamRect1, CamParamRect2,
                                                CamPoseRect1, CamPoseRect2, RelPoseRect};
  for (std::size_t i = 0; i < destinations.size(); ++i) {
    cpp_api::require_output(destinations[i], op);
    for (std::size_t j = 0; j < i; ++j)
      if (destinations[i] == destinations[j])
        throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, op, "Output", i,
                          "output pointers must be distinct");
  }
  cpp_api::TupleOutputs<5> controls(op,
                                    {CamParamRect1, CamParamRect2, CamPoseRect1, CamPoseRect2, RelPoseRect});
  struct Owners {
    ::MvObject values[2]{};
    ~Owners() {
      for (auto value : values)
        if (value) (void)mv_clear_obj(value);
    }
  } owners;
  cpp_api::check_status(
      mv_gen_binocular_rectification_map(
          &owners.values[0], &owners.values[1], CamParam1.ControlInput(), CamParam2.ControlInput(),
          RelPose.ControlInput(), SubSampling.ControlInput(), Method.ControlInput(), MapType.ControlInput(),
          controls.Out(0), controls.Out(1), controls.Out(2), controls.Out(3), controls.Out(4)),
      op);
  auto first = detail::MvObjectAccess::Adopt(std::exchange(owners.values[0], nullptr)),
       second = detail::MvObjectAccess::Adopt(std::exchange(owners.values[1], nullptr));
  detail::MvObjectAccess::CheckOutput(Map1, first);
  detail::MvObjectAccess::CheckOutput(Map2, second);
  controls.Commit();
  detail::MvObjectAccess::Commit(Map1, std::move(first));
  detail::MvObjectAccess::Commit(Map2, std::move(second));
}
}  // namespace mv

namespace mv {
MvImage MvImage::GenBinocularRectificationMap(const MvCamPar& a, const MvCamPar& b, const MvPose& p,
                                              double sampling, const MvString& method, const MvString& type,
                                              MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                              MvPose* relative) {
  MvImage second;
  mv::GenBinocularRectificationMap(this, &second, a, b, p, sampling, method, type, ra, rb, pa, pb, relative);
  return second;
}
MvImage MvImage::GenBinocularRectificationMap(const MvCamPar& a, const MvCamPar& b, const MvPose& p,
                                              double sampling, const char* method, const char* type,
                                              MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                              MvPose* relative) {
  return GenBinocularRectificationMap(a, b, p, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
MvImage MvImage::GenBinocularRectificationMap(const MvCamPar& a, const MvCamPar& b, const MvPose& p,
                                              double sampling, const wchar_t* method, const wchar_t* type,
                                              MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                              MvPose* relative) {
  return GenBinocularRectificationMap(a, b, p, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
MvImage MvCamPar::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& b, const MvPose& p,
                                               double sampling, const MvString& method, const MvString& type,
                                               MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                               MvPose* relative) const {
  MvImage first;
  mv::GenBinocularRectificationMap(&first, Map2, *this, b, p, sampling, method, type, ra, rb, pa, pb,
                                   relative);
  return first;
}
MvImage MvCamPar::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& b, const MvPose& p,
                                               double sampling, const char* method, const char* type,
                                               MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                               MvPose* relative) const {
  return GenBinocularRectificationMap(Map2, b, p, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
MvImage MvCamPar::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& b, const MvPose& p,
                                               double sampling, const wchar_t* method, const wchar_t* type,
                                               MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                               MvPose* relative) const {
  return GenBinocularRectificationMap(Map2, b, p, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
MvImage MvPose::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& a, const MvCamPar& b,
                                             double sampling, const MvString& method, const MvString& type,
                                             MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                             MvPose* relative) const {
  MvImage first;
  mv::GenBinocularRectificationMap(&first, Map2, a, b, *this, sampling, method, type, ra, rb, pa, pb,
                                   relative);
  return first;
}
MvImage MvPose::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& a, const MvCamPar& b,
                                             double sampling, const char* method, const char* type,
                                             MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                             MvPose* relative) const {
  return GenBinocularRectificationMap(Map2, a, b, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
MvImage MvPose::GenBinocularRectificationMap(MvImage* Map2, const MvCamPar& a, const MvCamPar& b,
                                             double sampling, const wchar_t* method, const wchar_t* type,
                                             MvCamPar* ra, MvCamPar* rb, MvPose* pa, MvPose* pb,
                                             MvPose* relative) const {
  return GenBinocularRectificationMap(Map2, a, b, sampling, MvString(method), MvString(type), ra, rb, pa, pb,
                                      relative);
}
}  // namespace mv
