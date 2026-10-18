#include <mvkit/c/mv_xld.h>
#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
#include "regions_cache.hpp"
#include "xld_cache.hpp"
#include <xld/operators.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
namespace {
using mv::c_api::TupleInput;
using mv::core::ErrorCode;
using mv::core::XldContour;
using mv::core::XldContours;
using mv::core::Tuple;
XldContours contours(::MvObject value,const char* op,const char* parameter="Contours") {
 mv::c_api::require_iconic(value,op,parameter);value->value.require_kind(mv::core::IconicKind::XldContour,op);return XldContours{value->value};
}
template<class F> decltype(auto) algorithm(const char* op,F&& f) {
 try{return std::forward<F>(f)();}catch(const mv::core::Exception& e){throw mv::core::Exception(e.status(),e.error_code(),op,e.parameter(),e.parameter_index(),e.message());}
}
std::string_view text(const TupleInput& v){if(v.size()!=1)v.fail(ErrorCode::InvalidCardinality,-1,"one string required");return v.string(0);}
std::size_t broadcast(std::initializer_list<const TupleInput*> values) {
 std::size_t count=0;for(const auto* v:values)count=std::max(count,v->size());
 for(const auto* v:values)if(v->size()!=1&&v->size()!=count){if(v->size()==0&&count<=1)return 0;v->fail(ErrorCode::InvalidCardinality,-1,"length one or common length required");}
 return count;
}
double numeric(const TupleInput& v,std::size_t i){const auto index=v.size()==1?0:i;auto value=v.number(index);if(!std::isfinite(value))v.fail(ErrorCode::InvalidValue,index,"finite number required");return value;}
mv::xld::PointOrder point_order(const TupleInput& v,std::size_t i){const auto index=v.size()==1?0:i;const auto t=v.string(index);if(t=="positive")return mv::xld::PointOrder::Positive;if(t=="negative")return mv::xld::PointOrder::Negative;v.fail(ErrorCode::UnsupportedValue,index,"positive or negative required");}
// HALCON iconic 矩阵作用于 (row,column) 像素角点，私有内核使用 (column,row) 像素中心；共轭转换，不改变内核。
Tuple matrix(::MvTuple input,const char* op,bool affine) {
 const TupleInput v(input,op,"HomMat2D");if(v.size()!=6&&(affine||v.size()!=9))v.fail(ErrorCode::InvalidCardinality,-1,"six affine or nine projective coefficients required");
 std::array<double,9> h{0,0,0,0,0,0,0,0,1};for(std::size_t i=0;i<v.size();++i)h[i]=numeric(v,i);
 std::array<double,9> shifted=h;
 for(int r=0;r<3;++r)shifted[3*r+2]=h[3*r+2]+.5*(h[3*r]+h[3*r+1]);
 for(int c=0;c<3;++c){shifted[c]-=.5*shifted[6+c];shifted[3+c]-=.5*shifted[6+c];}
 const int permutation[]{1,0,2};Tuple result;
 for(int r=0;r<(affine?2:3);++r)for(int c=0;c<3;++c)result.append(shifted[3*permutation[r]+permutation[c]]);
 return result;
}
XldContour explicit_closure(const XldContour& c) {
 const auto points=c.points();if(points.empty()||((points.front()==points.back())&&c.closed()))return c;
 std::vector<mv::core::Point2d> p(points.begin(),points.end());const bool append=points.front()!=points.back();if(append)p.push_back(p.front());
 const auto copy=[&](mv::core::XldPointAttribute attr){const auto a=c.point_attribute(attr);std::vector<double> out(a.begin(),a.end());if(append&&!out.empty())out.push_back(out.front());return out;};
 const auto response=copy(mv::core::XldPointAttribute::Response),edge=copy(mv::core::XldPointAttribute::EdgeDirection),normal=copy(mv::core::XldPointAttribute::NormalAngle);
 return XldContour::from_points_and_attributes(p,true,c.approximation(),response,edge,normal);
}
void append_generated(XldContours& result,const XldContours& value){for(std::size_t i=0;i<value.contour_count();++i){const auto& c=value.contour(i);result.append(c.closed()?explicit_closure(c):c);}}
}
extern "C" ::MvStatus mv_gen_contour_polygon_xld(::MvObject* Contour,::MvTuple Row,::MvTuple Col) {
 return mv::c_api::translate([&]{constexpr auto op="gen_contour_polygon_xld";mv::c_api::require_output(Contour,op);const TupleInput r(Row,op,"Row"),c(Col,op,"Col");if(!r.size())r.fail(ErrorCode::InvalidCardinality,-1,"nonempty coordinates required");if(c.size()!=r.size())c.fail(ErrorCode::InvalidCardinality,-1,"equal coordinate lengths required");std::vector<mv::core::Point2d> points;points.reserve(r.size());for(std::size_t i=0;i<r.size();++i)points.push_back({numeric(r,i),numeric(c,i)});const bool closed=points.size()>1&&points.front()==points.back();*Contour=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::gen_contour_polygon_xld(points,closed);}));});
}
extern "C" ::MvStatus mv_gen_circle_contour_xld(::MvObject* ContCircle,::MvTuple Row,::MvTuple Column,::MvTuple Radius,::MvTuple StartPhi,::MvTuple EndPhi,::MvTuple PointOrder,::MvTuple Resolution) {
 return mv::c_api::translate([&]{constexpr auto op="gen_circle_contour_xld";mv::c_api::require_output(ContCircle,op);const TupleInput r(Row,op,"Row"),c(Column,op,"Column"),radius(Radius,op,"Radius"),start(StartPhi,op,"StartPhi"),end(EndPhi,op,"EndPhi"),order(PointOrder,op,"PointOrder"),res(Resolution,op,"Resolution");const auto n=broadcast({&r,&c,&radius,&start,&end,&order,&res});XldContours result;for(std::size_t i=0;i<n;++i){const auto rr=numeric(radius,i),step=numeric(res,i);if(rr<=0)radius.fail(ErrorCode::InvalidValue,i,"positive radius required");if(step<=0)res.fail(ErrorCode::InvalidValue,i,"positive resolution required");append_generated(result,algorithm(op,[&]{return mv::xld::gen_circle_contour_xld(numeric(r,i),numeric(c,i),rr,numeric(start,i),numeric(end,i),point_order(order,i),step);}));}*ContCircle=mv::c_api::make_xld_contours(std::move(result));});
}
extern "C" ::MvStatus mv_gen_ellipse_contour_xld(::MvObject* ContEllipse,::MvTuple Row,::MvTuple Column,::MvTuple Phi,::MvTuple Radius1,::MvTuple Radius2,::MvTuple StartPhi,::MvTuple EndPhi,::MvTuple PointOrder,::MvTuple Resolution) {
 return mv::c_api::translate([&]{constexpr auto op="gen_ellipse_contour_xld";mv::c_api::require_output(ContEllipse,op);const TupleInput r(Row,op,"Row"),c(Column,op,"Column"),phi(Phi,op,"Phi"),a(Radius1,op,"Radius1"),b(Radius2,op,"Radius2"),start(StartPhi,op,"StartPhi"),end(EndPhi,op,"EndPhi"),order(PointOrder,op,"PointOrder"),res(Resolution,op,"Resolution");const auto n=broadcast({&r,&c,&phi,&a,&b,&start,&end,&order,&res});XldContours result;for(std::size_t i=0;i<n;++i)append_generated(result,algorithm(op,[&]{return mv::xld::gen_ellipse_contour_xld(numeric(r,i),numeric(c,i),numeric(phi,i),numeric(a,i),numeric(b,i),numeric(start,i),numeric(end,i),point_order(order,i),numeric(res,i));}));*ContEllipse=mv::c_api::make_xld_contours(std::move(result));});
}
extern "C" ::MvStatus mv_gen_contour_region_xld(::MvObject Regions,::MvObject* Contours,::MvTuple Mode) {
 return mv::c_api::translate([&]{constexpr auto op="gen_contour_region_xld";mv::c_api::require_output(Contours,op);mv::c_api::require_iconic(Regions,op,"Regions");Regions->value.require_kind(mv::core::IconicKind::Region,op);const TupleInput mode(Mode,op,"Mode");const auto value=text(mode);mv::xld::RegionContourMode kind;if(value=="center")kind=mv::xld::RegionContourMode::Center;else if(value=="border")kind=mv::xld::RegionContourMode::Border;else if(value=="border_holes")kind=mv::xld::RegionContourMode::BorderHoles;else mode.fail(ErrorCode::UnsupportedValue,0,"unsupported contour mode");*Contours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::gen_contour_region_xld(mv::core::Regions{Regions->value},kind);}));});
}
extern "C" ::MvStatus mv_gen_region_contour_xld(::MvObject Contour,::MvObject* Region,::MvTuple Mode) {
 return mv::c_api::translate([&]{constexpr auto op="gen_region_contour_xld";mv::c_api::require_output(Region,op);const auto c=contours(Contour,op,"Contour");const TupleInput mode(Mode,op,"Mode");const auto value=text(mode);mv::xld::ContourRegionMode kind;if(value=="filled")kind=mv::xld::ContourRegionMode::Filled;else if(value=="margin")kind=mv::xld::ContourRegionMode::Margin;else mode.fail(ErrorCode::UnsupportedValue,0,"filled or margin required");*Region=mv::c_api::make_regions(algorithm(op,[&]{return mv::xld::gen_region_contour_xld(c,kind);}));});
}
extern "C" ::MvStatus mv_length_xld(::MvObject XLD,::MvTuple* Length) {
 return mv::c_api::translate([&]{constexpr auto op="length_xld";mv::c_api::ControlOutputs<1> output(op,{Length});const auto value=contours(XLD,op,"XLD");output[0]=Tuple(mv::xld::length_xld(value));output.Commit();});
}
extern "C" ::MvStatus mv_area_center_xld(::MvObject XLD,::MvTuple* Area,::MvTuple* Row,::MvTuple* Column,::MvTuple* PointOrder) {
 return mv::c_api::translate([&]{constexpr auto op="area_center_xld";mv::c_api::ControlOutputs<4> output(op,{Area,Row,Column,PointOrder});auto v=algorithm(op,[&]{return mv::xld::area_center_xld(contours(XLD,op,"XLD"));});output[0]=std::move(v.area);output[1]=std::move(v.row);output[2]=std::move(v.column);output[3]=std::move(v.point_order);output.Commit();});
}
extern "C" ::MvStatus mv_affine_trans_contour_xld(::MvObject Contours,::MvObject* ContoursAffineTrans,::MvTuple HomMat2D) {
 return mv::c_api::translate([&]{constexpr auto op="affine_trans_contour_xld";mv::c_api::require_output(ContoursAffineTrans,op);const auto c=contours(Contours,op);const auto m=matrix(HomMat2D,op,true);*ContoursAffineTrans=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::affine_trans_contour_xld(c,m);}));});
}
extern "C" ::MvStatus mv_projective_trans_contour_xld(::MvObject Contours,::MvObject* ContoursProjTrans,::MvTuple HomMat2D) {
 return mv::c_api::translate([&]{constexpr auto op="projective_trans_contour_xld";mv::c_api::require_output(ContoursProjTrans,op);const auto c=contours(Contours,op);const auto m=matrix(HomMat2D,op,false);*ContoursProjTrans=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::projective_trans_contour_xld(c,m);}));});
}
extern "C" ::MvStatus mv_close_contours_xld(::MvObject Contours,::MvObject* ClosedContours) {
 return mv::c_api::translate([&]{constexpr auto op="close_contours_xld";mv::c_api::require_output(ClosedContours,op);const auto c=contours(Contours,op);XldContours result;for(std::size_t i=0;i<c.contour_count();++i)result.append(explicit_closure(c.contour(i)));*ClosedContours=mv::c_api::make_xld_contours(std::move(result));});
}
extern "C" ::MvStatus mv_smooth_contours_xld(::MvObject Contours,::MvObject* SmoothedContours,::MvTuple NumRegrPoints) {
 return mv::c_api::translate([&]{constexpr auto op="smooth_contours_xld";mv::c_api::require_output(SmoothedContours,op);const auto c=contours(Contours,op);const TupleInput n(NumRegrPoints,op,"NumRegrPoints");const auto v=n.scalar_integer(true);if(v<3||v>std::numeric_limits<int>::max()||v%2==0)n.fail(ErrorCode::InvalidValue,0,"odd regression window of at least three required");*SmoothedContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::smooth_contours_xld(c,static_cast<int>(v));}));});
}
extern "C" ::MvStatus mv_distance_pc(::MvObject Contour,::MvTuple Row,::MvTuple Column,::MvTuple* DistanceMin,::MvTuple* DistanceMax) {
 return mv::c_api::translate([&]{constexpr auto op="distance_pc";mv::c_api::ControlOutputs<2> output(op,{DistanceMin,DistanceMax});const auto c=contours(Contour,op,"Contour");const TupleInput r(Row,op,"Row"),col(Column,op,"Column");
if(!c.contour_count()||!r.size()){output.Commit();return;}
if(col.size()!=r.size())col.fail(ErrorCode::InvalidCardinality,-1,"equal coordinate lengths required");
XldContours first;first.append(c.contour(0));
const mv::c_api::ControlTupleView rv(Row,op,"Row"),cv(Column,op,"Column");
auto v=algorithm(op,[&]{return mv::xld::distance_pc(first,rv,cv);});output[0]=std::move(v.minimum);output[1]=std::move(v.maximum);output.Commit();});
}
extern "C" ::MvStatus mv_distance_cc(::MvObject Contour1,::MvObject Contour2,::MvTuple Mode,::MvTuple* DistanceMin,::MvTuple* DistanceMax) {
 return mv::c_api::translate([&]{constexpr auto op="distance_cc";mv::c_api::ControlOutputs<2> output(op,{DistanceMin,DistanceMax});const auto a=contours(Contour1,op,"Contour1"),b=contours(Contour2,op,"Contour2");const TupleInput modes(Mode,op,"Mode");const auto n=std::max({a.contour_count(),b.contour_count(),modes.size()});const auto check=[&](std::size_t count,const char* name){if(count!=n&&count!=1)throw mv::core::Exception(mv::core::Status::InvalidArgument,ErrorCode::InvalidCardinality,op,name,-1,"one or common number of operands required");};if(n>1){check(a.contour_count(),"Contour1");check(b.contour_count(),"Contour2");check(modes.size(),"Mode");}if(!a.contour_count()||!b.contour_count()||!modes.size()){output.Commit();return;}for(std::size_t i=0;i<n;++i){const auto at=modes.size()==1?0:i;const auto t=modes.string(at);mv::xld::ContourDistanceMode mode;if(t=="point_to_point")mode=mv::xld::ContourDistanceMode::PointToPoint;else if(t=="point_to_segment")mode=mv::xld::ContourDistanceMode::PointToSegment;else modes.fail(ErrorCode::UnsupportedValue,at,"unsupported distance mode");XldContours x,y;x.append(a.contour(a.contour_count()==1?0:i));y.append(b.contour(b.contour_count()==1?0:i));auto v=algorithm(op,[&]{return mv::xld::distance_cc(x,y,mode);});output[0].append(v.minimum.at(0));output[1].append(v.maximum.at(0));}output.Commit();});
}
extern "C" ::MvStatus mv_intersection_contours_xld(::MvObject Contour1,::MvObject Contour2,::MvTuple IntersectionType,::MvTuple* Row,::MvTuple* Column,::MvTuple* IsOverlapping) {
 return mv::c_api::translate([&]{constexpr auto op="intersection_contours_xld";mv::c_api::ControlOutputs<3> output(op,{Row,Column,IsOverlapping});const auto a=contours(Contour1,op,"Contour1"),b=contours(Contour2,op,"Contour2");const TupleInput type(IntersectionType,op,"IntersectionType");if(!a.contour_count()||!b.contour_count()){output.Commit();return;}const auto t=text(type);mv::xld::IntersectionMode mode;if(t=="all")mode=mv::xld::IntersectionMode::All;else if(t=="mutual")mode=mv::xld::IntersectionMode::Mutual;else if(t=="self")mode=mv::xld::IntersectionMode::Self;else type.fail(ErrorCode::UnsupportedValue,0,"unsupported intersection mode");auto v=algorithm(op,[&]{return mv::xld::intersection_contours_xld(a,b,mode);});output[0]=std::move(v.row);output[1]=std::move(v.column);output[2]=std::move(v.overlapping);output.Commit();});
}

namespace {
bool boolean_text(const TupleInput& value) {
 const auto t=text(value);if(t=="true")return true;if(t=="false")return false;
 value.fail(ErrorCode::UnsupportedValue,0,"true or false required");
}
mv::xld::XldAttributeMode attributes(const TupleInput& value) {
 const auto t=text(value);if(t=="attr_keep")return mv::xld::XldAttributeMode::Keep;
 if(t=="attr_forget")return mv::xld::XldAttributeMode::Forget;
 value.fail(ErrorCode::UnsupportedValue,0,"attr_keep or attr_forget required");
}
bool merge_eligible(const XldContours& contours) {
 for(std::size_t i=0;i<contours.contour_count();++i){const auto& c=contours.contour(i);if(c.size()>1&&!c.closed()&&c.points().front()!=c.points().back())return true;}
 return false;
}
}
extern "C" ::MvStatus mv_select_contours_xld(::MvObject Contours,::MvObject* SelectedContours,::MvTuple Feature,::MvTuple Min1,::MvTuple Max1,::MvTuple Min2,::MvTuple Max2) {
 return mv::c_api::translate([&]{constexpr auto op="select_contours_xld";mv::c_api::require_output(SelectedContours,op);const auto c=contours(Contours,op);
 const TupleInput f(Feature,op,"Feature"),a(Min1,op,"Min1"),b(Max1,op,"Max1"),d(Min2,op,"Min2"),e(Max2,op,"Max2");
 const auto name=text(f);mv::xld::XldSelectionFeature kind;
 if(name=="contour_length")kind=mv::xld::XldSelectionFeature::ContourLength;
 else if(name=="maximum_extent")kind=mv::xld::XldSelectionFeature::MaximumExtent;
 else if(name=="direction")kind=mv::xld::XldSelectionFeature::Direction;
 else if(name=="curvature")kind=mv::xld::XldSelectionFeature::Curvature;
 else if(name=="closed")kind=mv::xld::XldSelectionFeature::Closed;
 else if(name=="open")kind=mv::xld::XldSelectionFeature::Open;
 else f.fail(ErrorCode::UnsupportedValue,0,"unsupported contour feature");
 const double lo=a.scalar_number(),hi=b.scalar_number(),lo2=d.scalar_number(),hi2=e.scalar_number();
 if(kind!=mv::xld::XldSelectionFeature::Closed&&kind!=mv::xld::XldSelectionFeature::Open){if(lo>hi)b.fail(ErrorCode::InvalidValue,0,"ordered selection range required");if(lo2>hi2)e.fail(ErrorCode::InvalidValue,0,"ordered selection range required");}
 *SelectedContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::select_contours_xld(c,kind,lo,hi,lo2,hi2);}));});
}
extern "C" ::MvStatus mv_select_shape_xld(::MvObject XLD,::MvObject* SelectedXLD,::MvTuple Features,::MvTuple Operation,::MvTuple Min,::MvTuple Max) {
 return mv::c_api::translate([&]{constexpr auto op="select_shape_xld";mv::c_api::require_output(SelectedXLD,op);const auto c=contours(XLD,op,"XLD");
 const TupleInput f(Features,op,"Features"),mode(Operation,op,"Operation"),lo(Min,op,"Min"),hi(Max,op,"Max");
 if(!c.contour_count()){*SelectedXLD=mv::c_api::make_xld_contours(XldContours{});return;}
 if(lo.size()!=f.size())lo.fail(ErrorCode::InvalidCardinality,-1,"one bound per feature required");
 if(hi.size()!=f.size())hi.fail(ErrorCode::InvalidCardinality,-1,"one bound per feature required");
 if(!f.size()){*SelectedXLD=mv::c_api::make_xld_contours(XldContours{});return;}
 const auto m=text(mode);if(m!="and"&&m!="or")mode.fail(ErrorCode::UnsupportedValue,0,"and or or required");
 Tuple feature,lower,upper;
 for(std::size_t i=0;i<f.size();++i){const auto t=f.string(i);
  if(t!="area"&&t!="contlength"&&t!="row"&&t!="column"&&t!="width"&&t!="height"&&t!="ratio"&&t!="circularity")f.fail(ErrorCode::UnsupportedValue,i,"unsupported HALCON XLD shape feature");
  feature.append(mv::core::String{t});
  const auto bound=[&](const TupleInput& v,bool high){const auto value=v.at(i);if(value.kind()==mv::core::ValueKind::String){if(v.string(i)==(high?"max":"min"))return high?std::numeric_limits<double>::max():std::numeric_limits<double>::lowest();v.fail(ErrorCode::UnsupportedValue,i,"min or max required for open bound");}return numeric(v,i);};
  lower.append(bound(lo,false));upper.append(bound(hi,true));
 }
 *SelectedXLD=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::select_shape_xld(c,feature,m=="and",lower,upper);}));});
}
extern "C" ::MvStatus mv_crop_contours_xld(::MvObject Contours,::MvObject* CroppedContours,::MvTuple Row1,::MvTuple Col1,::MvTuple Row2,::MvTuple Col2,::MvTuple CloseContours) {
 return mv::c_api::translate([&]{constexpr auto op="crop_contours_xld";mv::c_api::require_output(CroppedContours,op);const auto c=contours(Contours,op);
 const TupleInput r1(Row1,op,"Row1"),c1(Col1,op,"Col1"),r2(Row2,op,"Row2"),c2(Col2,op,"Col2"),close(CloseContours,op,"CloseContours");
 const double a=r1.scalar_number(),b=c1.scalar_number(),d=r2.scalar_number(),e=c2.scalar_number();const bool closed=boolean_text(close);
 auto result=algorithm(op,[&]{return mv::xld::crop_contours_xld(c,a,b,d,e,closed);});XldContours out;append_generated(out,result);*CroppedContours=mv::c_api::make_xld_contours(std::move(out));});
}
extern "C" ::MvStatus mv_sort_contours_xld(::MvObject Contours,::MvObject* SortedContours,::MvTuple SortMode,::MvTuple Order,::MvTuple RowOrCol) {
 return mv::c_api::translate([&]{constexpr auto op="sort_contours_xld";mv::c_api::require_output(SortedContours,op);const auto c=contours(Contours,op);
 const TupleInput mode(SortMode,op,"SortMode"),order(Order,op,"Order"),axis(RowOrCol,op,"RowOrCol");if(!c.contour_count()){*SortedContours=mv::c_api::make_xld_contours(XldContours{});return;}
 const auto t=text(mode);mv::xld::XldSortMode kind;
 if(t=="upper_left")kind=mv::xld::XldSortMode::UpperLeft;else if(t=="upper_right")kind=mv::xld::XldSortMode::UpperRight;
 else if(t=="lower_left")kind=mv::xld::XldSortMode::LowerLeft;else if(t=="lower_right")kind=mv::xld::XldSortMode::LowerRight;
 else if(t=="character")kind=mv::xld::XldSortMode::Character;else mode.fail(ErrorCode::UnsupportedValue,0,"unsupported sort mode");
 const bool asc=boolean_text(order);const auto a=text(axis);if(a!="row"&&a!="column")axis.fail(ErrorCode::UnsupportedValue,0,"row or column required");
 *SortedContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::sort_contours_xld(c,kind,asc?mv::xld::XldSortOrder::Ascending:mv::xld::XldSortOrder::Descending,a=="row"?mv::xld::XldSortAxis::Row:mv::xld::XldSortAxis::Column);}));});
}
extern "C" ::MvStatus mv_union_adjacent_contours_xld(::MvObject Contours,::MvObject* UnionContours,::MvTuple MaxDistAbs,::MvTuple MaxDistRel,::MvTuple Mode) {
 return mv::c_api::translate([&]{constexpr auto op="union_adjacent_contours_xld";mv::c_api::require_output(UnionContours,op);const auto c=contours(Contours,op);
 const TupleInput a(MaxDistAbs,op,"MaxDistAbs"),b(MaxDistRel,op,"MaxDistRel"),mode(Mode,op,"Mode");const double abs=a.scalar_number(),rel=b.scalar_number();const auto m=merge_eligible(c)?attributes(mode):mv::xld::XldAttributeMode::Keep;
 *UnionContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::union_adjacent_contours_xld(c,abs,rel,m);}));});
}
extern "C" ::MvStatus mv_union_collinear_contours_xld(::MvObject Contours,::MvObject* UnionContours,::MvTuple MaxDistAbs,::MvTuple MaxDistRel,::MvTuple MaxShift,::MvTuple MaxAngle,::MvTuple Mode) {
 return mv::c_api::translate([&]{constexpr auto op="union_collinear_contours_xld";mv::c_api::require_output(UnionContours,op);const auto c=contours(Contours,op);
 const TupleInput a(MaxDistAbs,op,"MaxDistAbs"),b(MaxDistRel,op,"MaxDistRel"),shift(MaxShift,op,"MaxShift"),angle(MaxAngle,op,"MaxAngle"),mode(Mode,op,"Mode");
 const double abs=a.scalar_number(),rel=b.scalar_number(),s=shift.scalar_number(),t=angle.scalar_number();
 const auto m=merge_eligible(c)?attributes(mode):mv::xld::XldAttributeMode::Keep;
 *UnionContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::union_collinear_contours_xld(c,abs,rel,s,t,m);}));});
}
extern "C" ::MvStatus mv_union_cocircular_contours_xld(::MvObject Contours,::MvObject* UnionContours,::MvTuple MaxArcAngleDiff,::MvTuple MaxArcOverlap,::MvTuple MaxTangentAngle,::MvTuple MaxDist,::MvTuple MaxRadiusDiff,::MvTuple MaxCenterDist,::MvTuple MergeSmallContours,::MvTuple Iterations) {
 return mv::c_api::translate([&]{constexpr auto op="union_cocircular_contours_xld";mv::c_api::require_output(UnionContours,op);const auto c=contours(Contours,op);
 const TupleInput gap(MaxArcAngleDiff,op,"MaxArcAngleDiff"),overlap(MaxArcOverlap,op,"MaxArcOverlap"),tangent(MaxTangentAngle,op,"MaxTangentAngle"),dist(MaxDist,op,"MaxDist"),radius(MaxRadiusDiff,op,"MaxRadiusDiff"),center(MaxCenterDist,op,"MaxCenterDist"),small(MergeSmallContours,op,"MergeSmallContours"),iters(Iterations,op,"Iterations");
 if(!c.contour_count()){*UnionContours=mv::c_api::make_xld_contours(XldContours{});return;}
 mv::xld::CocircularOptions options{gap.scalar_number(),overlap.scalar_number(),tangent.scalar_number(),dist.scalar_number(),radius.scalar_number(),center.scalar_number()};
 options.merge_small_contours=boolean_text(small);const auto n=iters.scalar_integer(true);if(n<0||n>std::numeric_limits<int>::max())iters.fail(ErrorCode::InvalidValue,0,"nonnegative iteration count required");options.iterations=std::max(1,static_cast<int>(n));options.min_num_points=3;
 *UnionContours=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::union_cocircular_contours_xld(c,options,mv::xld::XldAttributeMode::Keep);}));});
}

namespace {
const mv::core::IconicTuple& images(::MvObject value,const char* op) {
 mv::c_api::require_iconic(value,op,"Image");value->value.require_kind(mv::core::IconicKind::Image,op);return value->value;
}
XldContour ordered_image_contour(const XldContour& contour) {
 const auto p=contour.points();if(contour.closed()||p.size()<2||p.front().row>p.back().row||(p.front().row==p.back().row&&p.front().col>=p.back().col))return contour;
 std::vector<mv::core::Point2d> points(p.rbegin(),p.rend());
 const auto reversed=[&](mv::core::XldPointAttribute attr){const auto a=contour.point_attribute(attr);return std::vector<double>(a.rbegin(),a.rend());};
 const auto response=reversed(mv::core::XldPointAttribute::Response),edge=reversed(mv::core::XldPointAttribute::EdgeDirection),normal=reversed(mv::core::XldPointAttribute::NormalAngle);
 return XldContour::from_points_and_attributes(points,false,contour.approximation(),response,edge,normal);
}
template<class F> XldContours image_contours(const mv::core::IconicTuple& images,F&& f) {
 XldContours result;for(std::size_t i=0;i<images.size();++i){const auto value=f(images.at(i).as_image());for(std::size_t k=0;k<value.contour_count();++k)result.append(ordered_image_contour(value.contour(k)));}return result;
}
}
extern "C" ::MvStatus mv_threshold_sub_pix(::MvObject Image,::MvObject* Border,::MvTuple Threshold) {
 return mv::c_api::translate([&]{constexpr auto op="threshold_sub_pix";mv::c_api::require_output(Border,op);const auto& input=images(Image,op);
 if(input.empty()){*Border=mv::c_api::make_xld_contours(XldContours{});return;}
 const TupleInput threshold(Threshold,op,"Threshold");const double t=threshold.scalar_number();
 *Border=mv::c_api::make_xld_contours(algorithm(op,[&]{return image_contours(input,[&](const auto& image){return mv::xld::threshold_sub_pix(image,(image.pixel_type()==mv::core::PixelType::Float32||image.pixel_type()==mv::core::PixelType::Int32)?t:std::round(t));});}));});
}
extern "C" ::MvStatus mv_edges_sub_pix(::MvObject Image,::MvObject* Edges,::MvTuple Filter,::MvTuple Alpha,::MvTuple Low,::MvTuple High) {
 return mv::c_api::translate([&]{constexpr auto op="edges_sub_pix";mv::c_api::require_output(Edges,op);const auto& input=images(Image,op);
 if(input.empty()){*Edges=mv::c_api::make_xld_contours(XldContours{});return;}
 const TupleInput filter(Filter,op,"Filter"),alpha(Alpha,op,"Alpha"),low(Low,op,"Low"),high(High,op,"High");const auto f=text(filter);mv::xld::EdgeFilter kind;
 if(f=="canny")kind=mv::xld::EdgeFilter::Canny;else if(f=="sobel")kind=mv::xld::EdgeFilter::Sobel;else filter.fail(ErrorCode::UnsupportedValue,0,"canny or sobel supported; recursive filters and junction completion are not implemented");
 const double a=alpha.scalar_number(),l=low.scalar_number(),h=high.scalar_number();
 *Edges=mv::c_api::make_xld_contours(algorithm(op,[&]{return image_contours(input,[&](const auto& image){return mv::xld::edges_sub_pix(image,kind,a,l,h);});}));});
}
extern "C" ::MvStatus mv_lines_gauss(::MvObject Image,::MvObject* Lines,::MvTuple Sigma,::MvTuple Low,::MvTuple High,::MvTuple LightDark,::MvTuple ExtractWidth,::MvTuple LineModel,::MvTuple CompleteJunctions) {
 return mv::c_api::translate([&]{constexpr auto op="lines_gauss";mv::c_api::require_output(Lines,op);const auto& input=images(Image,op);
 if(input.empty()){*Lines=mv::c_api::make_xld_contours(XldContours{});return;}
 const TupleInput sigma(Sigma,op,"Sigma"),low(Low,op,"Low"),high(High,op,"High"),polarity(LightDark,op,"LightDark"),width(ExtractWidth,op,"ExtractWidth"),model(LineModel,op,"LineModel"),junctions(CompleteJunctions,op,"CompleteJunctions");
 const double s=sigma.scalar_number(),l=low.scalar_number(),h=high.scalar_number();const auto light=text(polarity);if(light!="light"&&light!="dark")polarity.fail(ErrorCode::UnsupportedValue,0,"light or dark required");
 const bool extract=boolean_text(width),complete=boolean_text(junctions);(void)text(model);
 if(extract)width.fail(ErrorCode::UnsupportedValue,0,"line width extraction and asymmetric correction are not implemented");
 if(complete)junctions.fail(ErrorCode::UnsupportedValue,0,"junction completion is not implemented");
 *Lines=mv::c_api::make_xld_contours(algorithm(op,[&]{return image_contours(input,[&](const auto& image){return mv::xld::lines_gauss(image,s,l,h,light=="light"?mv::xld::LinePolarity::Light:mv::xld::LinePolarity::Dark);});}));});
}
extern "C" ::MvStatus mv_segment_contours_xld(::MvObject Contours,::MvObject* ContoursSplit,::MvTuple Mode,::MvTuple SmoothCont,::MvTuple MaxLineDist1,::MvTuple MaxLineDist2) {
 return mv::c_api::translate([&]{constexpr auto op="segment_contours_xld";mv::c_api::require_output(ContoursSplit,op);const auto input=contours(Contours,op);
 if(!input.contour_count()){*ContoursSplit=mv::c_api::make_xld_contours(XldContours{});return;}
 const TupleInput mode(Mode,op,"Mode"),smooth(SmoothCont,op,"SmoothCont"),first(MaxLineDist1,op,"MaxLineDist1"),second(MaxLineDist2,op,"MaxLineDist2");const auto m=text(mode);mv::xld::ContourSegmentationMode kind;
 if(m=="lines")kind=mv::xld::ContourSegmentationMode::Lines;else if(m=="lines_circles")kind=mv::xld::ContourSegmentationMode::LinesCircles;else if(m=="lines_ellipses")kind=mv::xld::ContourSegmentationMode::LinesEllipses;else mode.fail(ErrorCode::UnsupportedValue,0,"unsupported segmentation mode");
 const auto n=smooth.scalar_integer(true);if(n<0||n>std::numeric_limits<int>::max()-1)smooth.fail(ErrorCode::InvalidValue,0,"nonnegative bounded smoothing window required");const double a=first.scalar_number(),b=second.scalar_number();
 *ContoursSplit=mv::c_api::make_xld_contours(algorithm(op,[&]{return mv::xld::segment_contours_xld(input,kind,static_cast<int>(n),a,b);}));});
}
