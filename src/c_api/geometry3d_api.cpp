#include "tuple_input.hpp"
#include <mvkit/c/mv_geometry.h>

#include <memory>

#include "error_state.hpp"
#include "opaque.hpp"
#include "structured_error.hpp"

#include <calibration/hom_mat3d.hpp>
#include <core/exception.hpp>

namespace {
using mv::c_api::Exception;
using mv::core::Status;

void out(const void* p, const char* where) {
  if (p == nullptr) throw Exception(Status::InvalidArgument, where, "output pointer must not be null");
}

mv::c_api::ControlTupleView tup(::MvTuple input,const char* operation,const char* parameter) {
  return mv::c_api::ControlTupleView(input,operation,parameter);
}

void require_distinct(const void* a, const void* b, const char* where) {
  if (a == b) {
    throw Exception(Status::InvalidArgument, mv::core::ErrorCode::InvalidValue, where, "outputs", -1,
                    "parallel Tuple output pointers must be distinct");
  }
}

std::unique_ptr<MvTupleImpl> make_tuple(mv::core::Tuple value) {
  auto handle = std::make_unique<MvTupleImpl>();
  handle->value = std::move(value);
  return handle;
}

void commit_one(mv::core::Tuple value, ::MvTuple* output) {
  auto handle = make_tuple(std::move(value));
  *output = handle.release();
}

}  // namespace

extern "C" ::MvStatus mv_pose_to_hom_mat3d(::MvTuple pose,::MvTuple* hom_mat3d) {
  return mv::c_api::translate([&] {

    out(hom_mat3d, "pose_to_hom_mat3d");
    commit_one(mv::calibration::pose_to_hom_mat3d(tup(pose,"pose_to_hom_mat3d","Pose")), hom_mat3d);
  });
}

extern "C" ::MvStatus mv_hom_mat3d_to_pose(::MvTuple hom_mat3d,::MvTuple* pose) {
  return mv::c_api::translate([&] {

    out(pose, "hom_mat3d_to_pose");
    commit_one(mv::calibration::hom_mat3d_to_pose(tup(hom_mat3d,"hom_mat3d_to_pose","HomMat3D")), pose);
  });
}

extern "C" ::MvStatus mv_hom_mat3d_compose(::MvTuple left,::MvTuple right,::MvTuple* hom_mat3d) {
  return mv::c_api::translate([&] {

    out(hom_mat3d, "hom_mat3d_compose");
    commit_one(mv::calibration::hom_mat3d_compose(tup(left,"hom_mat3d_compose","HomMat3DLeft"),
                                                 tup(right,"hom_mat3d_compose","HomMat3DRight")),
               hom_mat3d);
  });
}

extern "C" ::MvStatus mv_hom_mat3d_invert(::MvTuple hom_mat3d,::MvTuple* hom_mat3d_out) {
  return mv::c_api::translate([&] {

    out(hom_mat3d_out, "hom_mat3d_invert");
    commit_one(mv::calibration::hom_mat3d_invert(tup(hom_mat3d,"hom_mat3d_invert","HomMat3D")),
               hom_mat3d_out);
  });
}

extern "C" ::MvStatus mv_affine_trans_point_3d(::MvTuple hom_mat3d,::MvTuple px,::MvTuple py,::MvTuple pz,::MvTuple* qx,::MvTuple* qy,::MvTuple* qz) {
  return mv::c_api::translate([&] {

    constexpr auto op = "affine_trans_point_3d";
    out(qx, op);
    out(qy, op);
    out(qz, op);
    require_distinct(qx, qy, op);
    require_distinct(qx, qz, op);
    require_distinct(qy, qz, op);
    const auto count=mv::c_api::TupleInput(px,op,"Px").size();
    if(count!=mv::c_api::TupleInput(py,op,"Py").size())
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Py",-1,"coordinate arrays must have equal lengths");
    if(count!=mv::c_api::TupleInput(pz,op,"Pz").size())
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Pz",-1,"coordinate arrays must have equal lengths");
    mv::core::Tuple ox;
    mv::core::Tuple oy;
    mv::core::Tuple oz;
    if(count==0)(void)mv::calibration::parse_hom_mat3d(tup(hom_mat3d,op,"HomMat3D"),op);
    else mv::calibration::affine_trans_point_3d(tup(hom_mat3d,op,"HomMat3D"), tup(px,op,"Px"), tup(py,op,"Py"), tup(pz,op,"Pz"), &ox, &oy, &oz);
    auto xh = make_tuple(std::move(ox));
    auto yh = make_tuple(std::move(oy));
    auto zh = make_tuple(std::move(oz));
    *qx = xh.release();
    *qy = yh.release();
    *qz = zh.release();
  });
}
