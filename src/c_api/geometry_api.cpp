#include "tuple_input.hpp"
#include <mvkit/c/mv_geometry.h>

#include <memory>

#include "error_state.hpp"
#include "opaque.hpp"
#include "structured_error.hpp"

#include <core/exception.hpp>
#include <geometry/analytic2d.hpp>
#include <geometry/hom_mat2d.hpp>

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

extern "C" ::MvStatus mv_hom_mat2d_identity(::MvTuple* hom_mat2d) {
  return mv::c_api::translate([&] {

    out(hom_mat2d, "hom_mat2d_identity");
    commit_one(mv::geometry::hom_mat2d_identity(), hom_mat2d);
  });
}

extern "C" ::MvStatus mv_hom_mat2d_translate(::MvTuple hom_mat2d,::MvTuple tx_input,::MvTuple ty_input,::MvTuple* hom_mat2d_out) {
  return mv::c_api::translate([&] {
    const auto tx=mv::c_api::TupleInput(tx_input,"hom_mat2d_translate","Tx").scalar_number();
    const auto ty=mv::c_api::TupleInput(ty_input,"hom_mat2d_translate","Ty").scalar_number();
    out(hom_mat2d_out, "hom_mat2d_translate");
    commit_one(mv::geometry::hom_mat2d_translate(tup(hom_mat2d,"hom_mat2d_translate","HomMat2D"), tx, ty),
               hom_mat2d_out);
  });
}

extern "C" ::MvStatus mv_hom_mat2d_rotate(::MvTuple hom_mat2d,::MvTuple phi_input,::MvTuple px_input,::MvTuple py_input,::MvTuple* hom_mat2d_out) {
  return mv::c_api::translate([&] {
    const auto phi=mv::c_api::TupleInput(phi_input,"hom_mat2d_rotate","Phi").scalar_number();
    const auto px=mv::c_api::TupleInput(px_input,"hom_mat2d_rotate","Px").scalar_number();
    const auto py=mv::c_api::TupleInput(py_input,"hom_mat2d_rotate","Py").scalar_number();
    out(hom_mat2d_out, "hom_mat2d_rotate");
    commit_one(mv::geometry::hom_mat2d_rotate(tup(hom_mat2d,"hom_mat2d_rotate","HomMat2D"), phi, px, py),
               hom_mat2d_out);
  });
}

extern "C" ::MvStatus mv_hom_mat2d_scale(::MvTuple hom_mat2d,::MvTuple sx_input,::MvTuple sy_input,::MvTuple px_input,::MvTuple py_input,::MvTuple* hom_mat2d_out) {
  return mv::c_api::translate([&] {
    const auto sx=mv::c_api::TupleInput(sx_input,"hom_mat2d_scale","Sx").scalar_number();
    const auto sy=mv::c_api::TupleInput(sy_input,"hom_mat2d_scale","Sy").scalar_number();
    const auto px=mv::c_api::TupleInput(px_input,"hom_mat2d_scale","Px").scalar_number();
    const auto py=mv::c_api::TupleInput(py_input,"hom_mat2d_scale","Py").scalar_number();
    out(hom_mat2d_out, "hom_mat2d_scale");
    commit_one(mv::geometry::hom_mat2d_scale(tup(hom_mat2d,"hom_mat2d_scale","HomMat2D"), sx, sy, px, py),
               hom_mat2d_out);
  });
}

extern "C" ::MvStatus mv_hom_mat2d_compose(::MvTuple left,::MvTuple right,::MvTuple* hom_mat2d) {
  return mv::c_api::translate([&] {

    out(hom_mat2d, "hom_mat2d_compose");
    commit_one(mv::geometry::hom_mat2d_compose(tup(left,"hom_mat2d_compose","HomMat2DLeft"),
                                              tup(right,"hom_mat2d_compose","HomMat2DRight")),
               hom_mat2d);
  });
}

extern "C" ::MvStatus mv_hom_mat2d_invert(::MvTuple hom_mat2d,::MvTuple* hom_mat2d_out) {
  return mv::c_api::translate([&] {

    out(hom_mat2d_out, "hom_mat2d_invert");
    commit_one(mv::geometry::hom_mat2d_invert(tup(hom_mat2d,"hom_mat2d_invert","HomMat2D")), hom_mat2d_out);
  });
}

extern "C" ::MvStatus mv_affine_trans_point_2d(::MvTuple hom_mat2d,::MvTuple px,::MvTuple py,::MvTuple* qx,::MvTuple* qy) {
  return mv::c_api::translate([&] {

    constexpr auto op = "affine_trans_point_2d";
    out(qx, op);
    out(qy, op);
    require_distinct(qx, qy, op);
    if(mv::c_api::TupleInput(px,op,"Px").size()!=mv::c_api::TupleInput(py,op,"Py").size())
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Py",-1,"coordinate arrays must have equal lengths");
    mv::core::Tuple ox;
    mv::core::Tuple oy;
    if(mv::c_api::TupleInput(px,op,"Px").size()==0)
      (void)mv::geometry::parse_hom_mat2d(tup(hom_mat2d,op,"HomMat2D"),op);
    else mv::geometry::affine_trans_point_2d(tup(hom_mat2d,op,"HomMat2D"), tup(px,op,"Px"), tup(py,op,"Py"), &ox, &oy);
    auto xh = make_tuple(std::move(ox));
    auto yh = make_tuple(std::move(oy));
    *qx = xh.release();
    *qy = yh.release();
  });
}

extern "C" ::MvStatus mv_vector_angle_to_rigid(::MvTuple row1_input,::MvTuple column1_input,::MvTuple angle1_input,::MvTuple row2_input,::MvTuple column2_input,::MvTuple angle2_input,::MvTuple* hom_mat2d) {
  return mv::c_api::translate([&] {
    const auto row1=mv::c_api::TupleInput(row1_input,"vector_angle_to_rigid","Row1").scalar_number();
    const auto column1=mv::c_api::TupleInput(column1_input,"vector_angle_to_rigid","Column1").scalar_number();
    const auto angle1=mv::c_api::TupleInput(angle1_input,"vector_angle_to_rigid","Angle1").scalar_number();
    const auto row2=mv::c_api::TupleInput(row2_input,"vector_angle_to_rigid","Row2").scalar_number();
    const auto column2=mv::c_api::TupleInput(column2_input,"vector_angle_to_rigid","Column2").scalar_number();
    const auto angle2=mv::c_api::TupleInput(angle2_input,"vector_angle_to_rigid","Angle2").scalar_number();
    out(hom_mat2d, "vector_angle_to_rigid");
    // HALCON 齐次图像坐标以 row 为 x、column 为 y。
    commit_one(mv::geometry::vector_angle_to_rigid(column1, row1, angle1, column2, row2, angle2),
               hom_mat2d);
  });
}

extern "C" ::MvStatus mv_distance_pp(::MvTuple row1,::MvTuple column1,::MvTuple row2,::MvTuple column2,::MvTuple* distance) {
  return mv::c_api::translate([&] {

    constexpr auto op = "distance_pp";
    out(distance, op);
    mv::core::Tuple out_d;
    mv::geometry::distance_pp(tup(row1,op,"Row1"), tup(column1,op,"Column1"), tup(row2,op,"Row2"), tup(column2,op,"Column2"),
                              &out_d);
    commit_one(std::move(out_d), distance);
  });
}

extern "C" ::MvStatus mv_distance_pl(::MvTuple row,::MvTuple column,::MvTuple row1,::MvTuple column1,::MvTuple row2,::MvTuple column2,::MvTuple* distance) {
  return mv::c_api::translate([&] {

    constexpr auto op = "distance_pl";
    out(distance, op);
    mv::core::Tuple out_d;
    mv::geometry::distance_pl(tup(row,op,"Row"), tup(column,op,"Column"), tup(row1,op,"Row1"), tup(column1,op,"Column1"),
                              tup(row2,op,"Row2"), tup(column2,op,"Column2"), &out_d);
    commit_one(std::move(out_d), distance);
  });
}

extern "C" ::MvStatus mv_intersection_lines(::MvTuple row_a1,::MvTuple column_a1,::MvTuple row_a2,::MvTuple column_a2,::MvTuple row_b1,::MvTuple column_b1,::MvTuple row_b2,::MvTuple column_b2,::MvTuple* row,::MvTuple* column,::MvTuple* is_overlapping) {
  return mv::c_api::translate([&] {

    constexpr auto op = "intersection_lines";
    out(row, op);
    out(column, op);
    out(is_overlapping, op);
    require_distinct(row, column, op);
    require_distinct(row, is_overlapping, op);
    require_distinct(column, is_overlapping, op);
    mv::core::Tuple orow;
    mv::core::Tuple ocol;
    mv::core::Tuple overlap;
    mv::geometry::intersection_lines(tup(row_a1,op,"Line1Row1"), tup(column_a1,op,"Line1Column1"), tup(row_a2,op,"Line1Row2"),
                                     tup(column_a2,op,"Line1Column2"), tup(row_b1,op,"Line2Row1"), tup(column_b1,op,"Line2Column1"),
                                     tup(row_b2,op,"Line2Row2"), tup(column_b2,op,"Line2Column2"), &orow, &ocol, &overlap);
    auto rh = make_tuple(std::move(orow));
    auto ch = make_tuple(std::move(ocol));
    auto oh = make_tuple(std::move(overlap));
    *row = rh.release();
    *column = ch.release();
    *is_overlapping = oh.release();
  });
}

extern "C" ::MvStatus mv_angle_ll(::MvTuple row_a1,::MvTuple column_a1,::MvTuple row_a2,::MvTuple column_a2,::MvTuple row_b1,::MvTuple column_b1,::MvTuple row_b2,::MvTuple column_b2,::MvTuple* angle) {
  return mv::c_api::translate([&] {

    constexpr auto op = "angle_ll";
    out(angle, op);
    mv::core::Tuple out_a;
    mv::geometry::angle_ll(tup(row_a1,op,"RowA1"), tup(column_a1,op,"ColumnA1"), tup(row_a2,op,"RowA2"), tup(column_a2,op,"ColumnA2"),
                           tup(row_b1,op,"RowB1"), tup(column_b1,op,"ColumnB1"), tup(row_b2,op,"RowB2"), tup(column_b2,op,"ColumnB2"),
                           &out_a);
    commit_one(std::move(out_a), angle);
  });
}
