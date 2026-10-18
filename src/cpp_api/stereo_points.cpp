#include <mvkit/c/mv_stereo.h>

#include <mvkit/cpp/mv_calibration.hpp>
#include <mvkit/cpp/mv_geometry.hpp>

#include "api_bridge.hpp"
namespace mv {
void DisparityToPoint3d(const MvTuple& a, const MvTuple& b, const MvTuple& p, const MvTuple& row,
                        const MvTuple& col, const MvTuple& d, MvTuple* x, MvTuple* y, MvTuple* z) {
  cpp_api::TupleOutputs<3> out("DisparityToPoint3d", {x, y, z});
  cpp_api::check_status(
      mv_disparity_to_point_3d(a.ControlInput(), b.ControlInput(), p.ControlInput(), row.ControlInput(),
                               col.ControlInput(), d.ControlInput(), out.Out(0), out.Out(1), out.Out(2)),
      "DisparityToPoint3d");
  out.Commit();
}
namespace {
void scalar(const MvCamPar& a, const MvCamPar& b, const MvPose& p, double row, double col, double d,
            double* x, double* y, double* z) {
  for (auto v : {x, y, z}) cpp_api::require_output(v, "DisparityToPoint3d");
  if (x == y || x == z || y == z)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, "DisparityToPoint3d", "Output", -1,
                      "output pointers must be distinct");
  MvTuple xx, yy, zz;
  mv::DisparityToPoint3d(a, b, p, row, col, d, &xx, &yy, &zz);
  const auto vx = xx.D(), vy = yy.D(), vz = zz.D();
  *x = vx;
  *y = vy;
  *z = vz;
}
}  // namespace
void MvCamPar::DisparityToPoint3d(const MvCamPar& b, const MvPose& p, const MvTuple& row, const MvTuple& col,
                                  const MvTuple& d, MvTuple* x, MvTuple* y, MvTuple* z) const {
  mv::DisparityToPoint3d(*this, b, p, row, col, d, x, y, z);
}
void MvCamPar::DisparityToPoint3d(const MvCamPar& b, const MvPose& p, double row, double col, double d,
                                  double* x, double* y, double* z) const {
  scalar(*this, b, p, row, col, d, x, y, z);
}
void MvPose::DisparityToPoint3d(const MvCamPar& a, const MvCamPar& b, const MvTuple& row, const MvTuple& col,
                                const MvTuple& d, MvTuple* x, MvTuple* y, MvTuple* z) const {
  mv::DisparityToPoint3d(a, b, *this, row, col, d, x, y, z);
}
void MvPose::DisparityToPoint3d(const MvCamPar& a, const MvCamPar& b, double row, double col, double d,
                                double* x, double* y, double* z) const {
  scalar(a, b, *this, row, col, d, x, y, z);
}
}  // namespace mv
