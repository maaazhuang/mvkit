#include "api_bridge.hpp"
#include <mvkit/c/mv_stereo.h>

#include <mvkit/cpp/mv_calibration.hpp>
#include <mvkit/cpp/mv_geometry.hpp>
#include <utility>

namespace mv {
void DisparityImageToXyz(const MvObject& Disparity, MvObject* X, MvObject* Y, MvObject* Z,
                         const MvTuple& CamParamRect1, const MvTuple& CamParamRect2,
                         const MvTuple& RelPoseRect) {
  constexpr char op[] = "DisparityImageToXyz";
  for (auto p : {X, Y, Z}) cpp_api::require_output(p, op);
  if (X == Y || X == Z || Y == Z)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, op, "Output", -1,
                      "output pointers must be distinct");
  struct Owners {
    ::MvObject values[3]{};
    ~Owners() {
      for (auto p : values)
        if (p) (void)mv_clear_obj(p);
    }
  } owners;
  cpp_api::check_status(
      mv_disparity_image_to_xyz(Disparity.NativeHandle(), &owners.values[0], &owners.values[1],
                                &owners.values[2], CamParamRect1.ControlInput(), CamParamRect2.ControlInput(),
                                RelPoseRect.ControlInput()),
      op);
  auto x = detail::MvObjectAccess::Adopt(std::exchange(owners.values[0], nullptr)),
       y = detail::MvObjectAccess::Adopt(std::exchange(owners.values[1], nullptr)),
       z = detail::MvObjectAccess::Adopt(std::exchange(owners.values[2], nullptr));
  detail::MvObjectAccess::CheckOutput(X, x);
  detail::MvObjectAccess::CheckOutput(Y, y);
  detail::MvObjectAccess::CheckOutput(Z, z);
  detail::MvObjectAccess::Commit(X, std::move(x));
  detail::MvObjectAccess::Commit(Y, std::move(y));
  detail::MvObjectAccess::Commit(Z, std::move(z));
}
MvImage MvImage::DisparityImageToXyz(MvImage* Y, MvImage* Z, const MvCamPar& a, const MvCamPar& b,
                                     const MvPose& p) const {
  MvImage x;
  mv::DisparityImageToXyz(*this, &x, Y, Z, a, b, p);
  return x;
}
MvImage MvCamPar::DisparityImageToXyz(const MvImage& d, MvImage* Y, MvImage* Z, const MvCamPar& b,
                                      const MvPose& p) const {
  return d.DisparityImageToXyz(Y, Z, *this, b, p);
}
MvImage MvPose::DisparityImageToXyz(const MvImage& d, MvImage* Y, MvImage* Z, const MvCamPar& a,
                                    const MvCamPar& b) const {
  return d.DisparityImageToXyz(Y, Z, a, b, *this);
}
}  // namespace mv
