#include <mvkit/c/mv_xld.h>
#include <c_api/storage_access.hpp>

#include <atomic>
#include <limits>
#include <vector>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "structured_error.hpp"
#include "control_operator_outputs.hpp"
#include "tuple_input.hpp"
#include <core/exception.hpp>




namespace {
using mv::c_api::Exception;
using mv::core::Status;
}

namespace {
const mv::core::XldContour* single_contour(::MvObject contour,const char* op) {
  mv::c_api::require_iconic(contour,op,"Contour");
  contour->value.require_kind(mv::core::IconicKind::XldContour,op);
  if(contour->value.size()>1)
    throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Contour",-1,
                    "one contour or an empty object array required");
  return contour->value.empty()?nullptr:&contour->value.at(0).as_xld_contour();
}
[[noreturn]] void missing_attribute(const char* op,std::int64_t index) {
  throw Exception(Status::NotFound,mv::core::ErrorCode::MissingItem,op,"Name",index,
                  "requested contour attribute is not defined");
}
}

extern "C" ::MvStatus mv_get_contour_xld(::MvObject Contour,::MvTuple* Row,::MvTuple* Col) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_contour_xld";
    mv::c_api::ControlOutputs<2> result(op,{Row,Col});
    if(const auto* contour=single_contour(Contour,op)) {
      const auto points=contour->points();
      std::vector<double> rows(points.size()), columns(points.size());
      for (std::size_t i=0; i<points.size(); ++i) {
        rows[i]=points[i].row; columns[i]=points[i].col;
      }
      result[0]=mv::core::Tuple{std::move(rows)};
      result[1]=mv::core::Tuple{std::move(columns)};
    }
    result.Commit();
  });
}
extern "C" ::MvStatus mv_test_closed_xld(::MvObject XLD,::MvTuple* IsClosed) {
  return mv::c_api::translate([&] {
    constexpr auto op="test_closed_xld";
    mv::c_api::ControlOutputs<1> result(op,{IsClosed});
    mv::c_api::require_iconic(XLD,op,"XLD");
    XLD->value.require_kind(mv::core::IconicKind::XldContour,op);
    for(const auto& object:XLD->value.values()) {
      const auto points=object.as_xld_contour().points();
      result[0].append(int64_t(!points.empty()&&points.front()==points.back()));
    }
    result.Commit();
  });
}
extern "C" ::MvStatus mv_get_contour_attrib_xld(::MvObject Contour,::MvTuple Name,::MvTuple* Attrib) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_contour_attrib_xld";
    mv::c_api::ControlOutputs<1> result(op,{Attrib});
    const auto* contour=single_contour(Contour,op);
    if(!contour){result.Commit();return;}
    const mv::c_api::TupleInput name(Name,op,"Name");
    if(name.size()!=1)name.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one attribute name required");
    const auto text=name.string(0);
    mv::core::XldPointAttribute attribute;
    if(text=="response")attribute=mv::core::XldPointAttribute::Response;
    else if(text=="edge_direction")attribute=mv::core::XldPointAttribute::EdgeDirection;
    else if(text=="angle")attribute=mv::core::XldPointAttribute::NormalAngle;
    else missing_attribute(op,0);
    if(!contour->has_point_attribute(attribute))missing_attribute(op,0);
    for(const auto value:contour->point_attribute(attribute))result[0].append(value);
    result.Commit();
  });
}
extern "C" ::MvStatus mv_get_contour_global_attrib_xld(::MvObject Contour,::MvTuple Name,::MvTuple* Attrib) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_contour_global_attrib_xld";
    mv::c_api::ControlOutputs<1> result(op,{Attrib});
    const auto* contour=single_contour(Contour,op);
    if(!contour){result.Commit();return;}
    const mv::c_api::TupleInput names(Name,op,"Name");
    if(!names.size())names.fail(mv::core::ErrorCode::InvalidCardinality,-1,"at least one attribute name required");
    for(std::size_t i=0;i<names.size();++i) {
      if(names.string(i)!="cont_approx")missing_attribute(op,static_cast<std::int64_t>(i));
      double value;
      switch(contour->approximation()) {
        case mv::core::ContourApproximation::Line:value=-1;break;
        case mv::core::ContourApproximation::Ellipse:value=0;break;
        case mv::core::ContourApproximation::Circle:value=1;break;
        default:missing_attribute(op,static_cast<std::int64_t>(i));
      }
      result[0].append(value);
    }
    result.Commit();
  });
}
