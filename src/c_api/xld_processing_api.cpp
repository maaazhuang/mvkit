#include <mvkit/c/mv_xld.h>
#include "error_state.hpp"
#include "xld_fit_controls.hpp"
#include "control_operator_outputs.hpp"

extern "C" ::MvStatus mv_fit_line_contour_xld(
    ::MvObject Contours, ::MvTuple Algorithm, ::MvTuple MaxNumPoints, ::MvTuple ClippingEndPoints,
    ::MvTuple Iterations, ::MvTuple ClippingFactor, ::MvTuple* RowBegin, ::MvTuple* ColBegin,
    ::MvTuple* RowEnd, ::MvTuple* ColEnd, ::MvTuple* Nr, ::MvTuple* Nc, ::MvTuple* Dist) {
  return mv::c_api::translate([&]{
    constexpr auto op="fit_line_contour_xld";
    mv::c_api::ControlOutputs<7> output(op,{
      RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist
    });
    auto contours=mv::c_api::xld_fit::contours(Contours,op);
    if(!contours.contour_count()){
      output.Commit();
      return;
    }
    const auto settings=mv::c_api::xld_fit::line(Algorithm,MaxNumPoints,ClippingEndPoints,Iterations,ClippingFactor,op);
    const auto values=mv::c_api::xld_fit::algorithm(op,[&]{
      return mv::xld::fit_line_contour_xld(contours,settings);
    });
    for(const auto& fitted:values){
      auto v=fitted;
      if(v.distance<0){
        v.normal_row=-v.normal_row;
        v.normal_col=-v.normal_col;
        v.distance=-v.distance;
      }
      output[0].append(v.begin.row);
      output[1].append(v.begin.col);
      output[2].append(v.end.row);
      output[3].append(v.end.col);
      output[4].append(v.normal_row);
      output[5].append(v.normal_col);
      output[6].append(v.distance);
    }
    output.Commit();
  });
}

extern "C" ::MvStatus mv_fit_circle_contour_xld(
    ::MvObject Contours, ::MvTuple Algorithm, ::MvTuple MaxNumPoints, ::MvTuple MaxClosureDist,
    ::MvTuple ClippingEndPoints, ::MvTuple Iterations, ::MvTuple ClippingFactor, ::MvTuple* Row,
    ::MvTuple* Column, ::MvTuple* Radius, ::MvTuple* StartPhi, ::MvTuple* EndPhi,
    ::MvTuple* PointOrder) {
  return mv::c_api::translate([&]{
    constexpr auto op="fit_circle_contour_xld";
    mv::c_api::ControlOutputs<6> output(op,{
      Row,Column,Radius,StartPhi,EndPhi,PointOrder
    });
    auto contours=mv::c_api::xld_fit::contours(Contours,op);
    if(!contours.contour_count()){
      output.Commit();
      return;
    }
    const auto settings=mv::c_api::xld_fit::circle(Algorithm,MaxNumPoints,MaxClosureDist,ClippingEndPoints,Iterations,ClippingFactor,op);
    const auto values=mv::c_api::xld_fit::algorithm(op,[&]{
      return mv::xld::fit_circle_contour_xld(contours,settings);
    });
    for(const auto& v:values){
      output[0].append(v.row);
      output[1].append(v.column);
      output[2].append(v.radius);
      output[3].append(v.start_phi);
      output[4].append(v.end_phi);
      output[5].append(v.point_order==mv::xld::PointOrder::Positive?"positive":"negative");
    }
    output.Commit();
  });
}

extern "C" ::MvStatus mv_fit_ellipse_contour_xld(
    ::MvObject Contours, ::MvTuple Algorithm, ::MvTuple MaxNumPoints, ::MvTuple MaxClosureDist,
    ::MvTuple ClippingEndPoints, ::MvTuple VossTabSize, ::MvTuple Iterations,
    ::MvTuple ClippingFactor, ::MvTuple* Row, ::MvTuple* Column, ::MvTuple* Phi, ::MvTuple* Radius1,
    ::MvTuple* Radius2, ::MvTuple* StartPhi, ::MvTuple* EndPhi, ::MvTuple* PointOrder) {
  return mv::c_api::translate([&]{
    constexpr auto op="fit_ellipse_contour_xld";
    mv::c_api::ControlOutputs<8> output(op,{
      Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder
    });
    auto contours=mv::c_api::xld_fit::contours(Contours,op);
    if(!contours.contour_count()){
      output.Commit();
      return;
    }
    const auto settings=mv::c_api::xld_fit::ellipse(Algorithm,MaxNumPoints,MaxClosureDist,ClippingEndPoints,VossTabSize,Iterations,ClippingFactor,op);
    const auto values=mv::c_api::xld_fit::algorithm(op,[&]{
      return mv::xld::fit_ellipse_contour_xld(contours,settings);
    });
    for(const auto& v:values){
      output[0].append(v.row);
      output[1].append(v.column);
      output[2].append(v.phi);
      output[3].append(v.radius1);
      output[4].append(v.radius2);
      output[5].append(v.start_phi);
      output[6].append(v.end_phi);
      output[7].append(v.point_order==mv::xld::PointOrder::Positive?"positive":"negative");
    }
    output.Commit();
  });
}
