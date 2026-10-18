#include "api_bridge.hpp"
#include <mvkit/cpp/mv_xld.hpp>
#include <mvkit/cpp/mv_geometry.hpp>
#include <mvkit/c/mv_xld.h>
#include <array>
#include <cstring>
namespace mv {
void TestClosedXld(const MvObject& XLD,MvTuple* IsClosed) {
  cpp_api::TupleOutputs<1> output("TestClosedXld",{IsClosed});
  cpp_api::check_status(mv_test_closed_xld(XLD.NativeHandle(),output.Out(0)),"TestClosedXld");output.Commit();
}
MvTuple MvXLD::TestClosedXld() const {MvTuple result;mv::TestClosedXld(*this,&result);return result;}
void GetContourXld(const MvObject& Contour,MvTuple* Row,MvTuple* Col) {
  cpp_api::TupleOutputs<2> result("GetContourXld",{Row,Col});
  cpp_api::check_status(mv_get_contour_xld(Contour.NativeHandle(),result.Out(0),result.Out(1)),"GetContourXld");
  result.Commit();
}
void GetContourAttribXld(const MvObject& Contour,const MvTuple& Name,MvTuple* Attrib) {
  cpp_api::TupleOutputs<1> result("GetContourAttribXld",{Attrib});
  cpp_api::check_status(mv_get_contour_attrib_xld(Contour.NativeHandle(),Name.ControlInput(),result.Out(0)),"GetContourAttribXld");
  result.Commit();
}
void GetContourGlobalAttribXld(const MvObject& Contour,const MvTuple& Name,MvTuple* Attrib) {
  cpp_api::TupleOutputs<1> result("GetContourGlobalAttribXld",{Attrib});
  cpp_api::check_status(mv_get_contour_global_attrib_xld(Contour.NativeHandle(),Name.ControlInput(),result.Out(0)),"GetContourGlobalAttribXld");
  result.Commit();
}
void MvXLDCont::GetContourXld(MvTuple* Row,MvTuple* Col) const {mv::GetContourXld(*this,Row,Col);}
MvTuple MvXLDCont::GetContourAttribXld(const MvString& Name) const {
  MvTuple result;mv::GetContourAttribXld(*this,MvTuple(Name.Text()),&result);return result;
}
MvTuple MvXLDCont::GetContourAttribXld(const char* Name) const {return GetContourAttribXld(MvString(Name));}
MvTuple MvXLDCont::GetContourAttribXld(const wchar_t* Name) const {return GetContourAttribXld(MvString(Name));}
MvTuple MvXLDCont::GetContourGlobalAttribXld(const MvTuple& Name) const {
  MvTuple result;mv::GetContourGlobalAttribXld(*this,Name,&result);return result;
}
MvTuple MvXLDCont::GetContourGlobalAttribXld(const MvString& Name) const {return GetContourGlobalAttribXld(MvTuple(Name.Text()));}
MvTuple MvXLDCont::GetContourGlobalAttribXld(const char* Name) const {return GetContourGlobalAttribXld(MvString(Name));}
MvTuple MvXLDCont::GetContourGlobalAttribXld(const wchar_t* Name) const {return GetContourGlobalAttribXld(MvString(Name));}
namespace {
struct ScalarDestination { const void* pointer;std::size_t bytes;template<class T>ScalarDestination(T* value):pointer(value),bytes(sizeof(T)){} };
void scalar_outputs(const char* op,std::initializer_list<ScalarDestination> values){std::size_t i=0;for(const auto v:values){cpp_api::require_output(v.pointer,op);std::size_t k=0;for(const auto w:values){if(k++>=i)break;const auto a=reinterpret_cast<std::uintptr_t>(v.pointer),b=reinterpret_cast<std::uintptr_t>(w.pointer);if(a<=b?b-a<v.bytes:a-b<w.bytes)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,{op,std::strlen(op)},"Output",-1,"distinct scalar output byte ranges required");}++i;}}
}
void GenContourPolygonXld(MvObject* Contour, const MvTuple& Row, const MvTuple& Col) {
  cpp_api::require_output(Contour,"GenContourPolygonXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_contour_polygon_xld(&raw,Row.ControlInput(),Col.ControlInput()),"GenContourPolygonXld");
  detail::MvObjectAccess::Commit(Contour,detail::MvObjectAccess::Adopt(raw));
}
void MvXLDCont::GenContourPolygonXld(const MvTuple& Row, const MvTuple& Col) {
  mv::GenContourPolygonXld(this,Row,Col);
}
void GenCircleContourXld(MvObject* ContCircle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, const MvTuple& Resolution) {
  cpp_api::require_output(ContCircle,"GenCircleContourXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_circle_contour_xld(&raw,Row.ControlInput(),Column.ControlInput(),Radius.ControlInput(),StartPhi.ControlInput(),EndPhi.ControlInput(),PointOrder.ControlInput(),Resolution.ControlInput()),"GenCircleContourXld");
  detail::MvObjectAccess::Commit(ContCircle,detail::MvObjectAccess::Adopt(raw));
}
void MvXLDCont::GenCircleContourXld(const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, double Resolution) {
  mv::GenCircleContourXld(this,Row,Column,Radius,StartPhi,EndPhi,PointOrder,Resolution);
}
void MvXLDCont::GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi, const MvString& PointOrder, double Resolution) {
  mv::GenCircleContourXld(this,Row,Column,Radius,StartPhi,EndPhi,MvTuple(PointOrder.Text()),Resolution);
}
void MvXLDCont::GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi, const char* PointOrder, double Resolution) {
  mv::GenCircleContourXld(this,Row,Column,Radius,StartPhi,EndPhi,MvTuple(MvString(PointOrder).Text()),Resolution);
}
void MvXLDCont::GenCircleContourXld(double Row, double Column, double Radius, double StartPhi, double EndPhi, const wchar_t* PointOrder, double Resolution) {
  mv::GenCircleContourXld(this,Row,Column,Radius,StartPhi,EndPhi,MvTuple(MvString(PointOrder).Text()),Resolution);
}
void GenEllipseContourXld(MvObject* ContEllipse, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, const MvTuple& Resolution) {
  cpp_api::require_output(ContEllipse,"GenEllipseContourXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_ellipse_contour_xld(&raw,Row.ControlInput(),Column.ControlInput(),Phi.ControlInput(),Radius1.ControlInput(),Radius2.ControlInput(),StartPhi.ControlInput(),EndPhi.ControlInput(),PointOrder.ControlInput(),Resolution.ControlInput()),"GenEllipseContourXld");
  detail::MvObjectAccess::Commit(ContEllipse,detail::MvObjectAccess::Adopt(raw));
}
void MvXLDCont::GenEllipseContourXld(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2, const MvTuple& StartPhi, const MvTuple& EndPhi, const MvTuple& PointOrder, double Resolution) {
  mv::GenEllipseContourXld(this,Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder,Resolution);
}
void MvXLDCont::GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2, double StartPhi, double EndPhi, const MvString& PointOrder, double Resolution) {
  mv::GenEllipseContourXld(this,Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,MvTuple(PointOrder.Text()),Resolution);
}
void MvXLDCont::GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2, double StartPhi, double EndPhi, const char* PointOrder, double Resolution) {
  mv::GenEllipseContourXld(this,Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,MvTuple(MvString(PointOrder).Text()),Resolution);
}
void MvXLDCont::GenEllipseContourXld(double Row, double Column, double Phi, double Radius1, double Radius2, double StartPhi, double EndPhi, const wchar_t* PointOrder, double Resolution) {
  mv::GenEllipseContourXld(this,Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,MvTuple(MvString(PointOrder).Text()),Resolution);
}
void GenContourRegionXld(const MvObject& Regions, MvObject* Contours, const MvTuple& Mode) {
  cpp_api::require_output(Contours,"GenContourRegionXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_contour_region_xld(Regions.NativeHandle(),&raw,Mode.ControlInput()),"GenContourRegionXld");
  detail::MvObjectAccess::Commit(Contours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvRegion::GenContourRegionXld(const MvString& Mode) const {
  MvXLDCont result;
  mv::GenContourRegionXld(*this,&result,MvTuple(Mode.Text()));
  return result;
}
MvXLDCont MvRegion::GenContourRegionXld(const char* Mode) const {
  MvXLDCont result;
  mv::GenContourRegionXld(*this,&result,MvTuple(MvString(Mode).Text()));
  return result;
}
MvXLDCont MvRegion::GenContourRegionXld(const wchar_t* Mode) const {
  MvXLDCont result;
  mv::GenContourRegionXld(*this,&result,MvTuple(MvString(Mode).Text()));
  return result;
}
void GenRegionContourXld(const MvObject& Contour, MvObject* Region, const MvTuple& Mode) {
  cpp_api::require_output(Region,"GenRegionContourXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_region_contour_xld(Contour.NativeHandle(),&raw,Mode.ControlInput()),"GenRegionContourXld");
  detail::MvObjectAccess::Commit(Region,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvXLDCont::GenRegionContourXld(const MvString& Mode) const {
  MvRegion result;
  mv::GenRegionContourXld(*this,&result,MvTuple(Mode.Text()));
  return result;
}
MvRegion MvXLDCont::GenRegionContourXld(const char* Mode) const {
  MvRegion result;
  mv::GenRegionContourXld(*this,&result,MvTuple(MvString(Mode).Text()));
  return result;
}
MvRegion MvXLDCont::GenRegionContourXld(const wchar_t* Mode) const {
  MvRegion result;
  mv::GenRegionContourXld(*this,&result,MvTuple(MvString(Mode).Text()));
  return result;
}
void LengthXld(const MvObject& XLD, MvTuple* Length) {
  cpp_api::TupleOutputs<1> output("LengthXld",{Length});
  cpp_api::check_status(mv_length_xld(XLD.NativeHandle(),output.Out(0)),"LengthXld");
  output.Commit();
}
MvTuple MvXLD::LengthXld() const {
  MvTuple result;
  mv::LengthXld(*this,&result);
  return result;
}
void AreaCenterXld(const MvObject& XLD, MvTuple* Area, MvTuple* Row, MvTuple* Column, MvTuple* PointOrder) {
  cpp_api::TupleOutputs<4> output("AreaCenterXld",{Area,Row,Column,PointOrder});
  cpp_api::check_status(mv_area_center_xld(XLD.NativeHandle(),output.Out(0),output.Out(1),output.Out(2),output.Out(3)),"AreaCenterXld");
  output.Commit();
}
MvTuple MvXLD::AreaCenterXld(MvTuple* Row, MvTuple* Column, MvTuple* PointOrder) const {
  MvTuple result;
  mv::AreaCenterXld(*this,&result,Row,Column,PointOrder);
  return result;
}
double MvXLD::AreaCenterXld(double* Row, double* Column, MvString* PointOrder) const {
  scalar_outputs("AreaCenterXld",{Row,Column,PointOrder});
  MvTuple tuple_Area;
  MvTuple tuple_Row;
  MvTuple tuple_Column;
  MvTuple tuple_PointOrder;
  mv::AreaCenterXld(*this,&tuple_Area,&tuple_Row,&tuple_Column,&tuple_PointOrder);
  if(tuple_Area.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"AreaCenterXld","Contours",-1,"nonempty iconic array required for a scalar result");
  const double staged_Row=tuple_Row.D();
  const double staged_Column=tuple_Column.D();
  const MvString staged_PointOrder=tuple_PointOrder.S();
  const double result=tuple_Area.D();
  *Row=staged_Row;
  *Column=staged_Column;
  *PointOrder=staged_PointOrder;
  return result;
}
void AffineTransContourXld(const MvObject& Contours, MvObject* ContoursAffineTrans, const MvTuple& HomMat2D) {
  cpp_api::require_output(ContoursAffineTrans,"AffineTransContourXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_affine_trans_contour_xld(Contours.NativeHandle(),&raw,HomMat2D.ControlInput()),"AffineTransContourXld");
  detail::MvObjectAccess::Commit(ContoursAffineTrans,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvHomMat2D::AffineTransContourXld(const MvXLDCont& Contours) const {
  MvXLDCont result;
  mv::AffineTransContourXld(Contours,&result,*this);
  return result;
}
MvXLDCont MvXLDCont::AffineTransContourXld(const MvHomMat2D& HomMat2D) const {
  MvXLDCont result;
  mv::AffineTransContourXld(*this,&result,HomMat2D);
  return result;
}
void ProjectiveTransContourXld(const MvObject& Contours, MvObject* ContoursProjTrans, const MvTuple& HomMat2D) {
  cpp_api::require_output(ContoursProjTrans,"ProjectiveTransContourXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_projective_trans_contour_xld(Contours.NativeHandle(),&raw,HomMat2D.ControlInput()),"ProjectiveTransContourXld");
  detail::MvObjectAccess::Commit(ContoursProjTrans,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvHomMat2D::ProjectiveTransContourXld(const MvXLDCont& Contours) const {
  MvXLDCont result;
  mv::ProjectiveTransContourXld(Contours,&result,*this);
  return result;
}
MvXLDCont MvXLDCont::ProjectiveTransContourXld(const MvHomMat2D& HomMat2D) const {
  MvXLDCont result;
  mv::ProjectiveTransContourXld(*this,&result,HomMat2D);
  return result;
}
void CloseContoursXld(const MvObject& Contours, MvObject* ClosedContours) {
  cpp_api::require_output(ClosedContours,"CloseContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_close_contours_xld(Contours.NativeHandle(),&raw),"CloseContoursXld");
  detail::MvObjectAccess::Commit(ClosedContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::CloseContoursXld() const {
  MvXLDCont result;
  mv::CloseContoursXld(*this,&result);
  return result;
}
void DistancePc(const MvObject& Contour, const MvTuple& Row, const MvTuple& Column, MvTuple* DistanceMin, MvTuple* DistanceMax) {
  cpp_api::TupleOutputs<2> output("DistancePc",{DistanceMin,DistanceMax});
  cpp_api::check_status(mv_distance_pc(Contour.NativeHandle(),Row.ControlInput(),Column.ControlInput(),output.Out(0),output.Out(1)),"DistancePc");
  output.Commit();
}
void MvXLDCont::DistancePc(const MvTuple& Row, const MvTuple& Column, MvTuple* DistanceMin, MvTuple* DistanceMax) const {
  mv::DistancePc(*this,Row,Column,DistanceMin,DistanceMax);
}
void MvXLDCont::DistancePc(double Row, double Column, double* DistanceMin, double* DistanceMax) const {
  scalar_outputs("DistancePc",{DistanceMin,DistanceMax});
  MvTuple tuple_DistanceMin;
  MvTuple tuple_DistanceMax;
  mv::DistancePc(*this,Row,Column,&tuple_DistanceMin,&tuple_DistanceMax);
  if(tuple_DistanceMin.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"DistancePc","Contours",-1,"nonempty iconic array required for a scalar result");
  const double staged_DistanceMin=tuple_DistanceMin.D();
  const double staged_DistanceMax=tuple_DistanceMax.D();
  *DistanceMin=staged_DistanceMin;
  *DistanceMax=staged_DistanceMax;
}
void DistanceCc(const MvObject& Contour1, const MvObject& Contour2, const MvTuple& Mode, MvTuple* DistanceMin, MvTuple* DistanceMax) {
  cpp_api::TupleOutputs<2> output("DistanceCc",{DistanceMin,DistanceMax});
  cpp_api::check_status(mv_distance_cc(Contour1.NativeHandle(),Contour2.NativeHandle(),Mode.ControlInput(),output.Out(0),output.Out(1)),"DistanceCc");
  output.Commit();
}
void MvXLDCont::DistanceCc(const MvXLDCont& Contour2, const MvTuple& Mode, MvTuple* DistanceMin, MvTuple* DistanceMax) const {
  mv::DistanceCc(*this,Contour2,Mode,DistanceMin,DistanceMax);
}
void MvXLDCont::DistanceCc(const MvXLDCont& Contour2, const MvString& Mode, double* DistanceMin, double* DistanceMax) const {
  scalar_outputs("DistanceCc",{DistanceMin,DistanceMax});
  MvTuple tuple_DistanceMin;
  MvTuple tuple_DistanceMax;
  mv::DistanceCc(*this,Contour2,MvTuple(Mode.Text()),&tuple_DistanceMin,&tuple_DistanceMax);
  if(tuple_DistanceMin.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"DistanceCc","Contours",-1,"nonempty iconic array required for a scalar result");
  const double staged_DistanceMin=tuple_DistanceMin.D();
  const double staged_DistanceMax=tuple_DistanceMax.D();
  *DistanceMin=staged_DistanceMin;
  *DistanceMax=staged_DistanceMax;
}
void MvXLDCont::DistanceCc(const MvXLDCont& Contour2, const char* Mode, double* DistanceMin, double* DistanceMax) const {
  scalar_outputs("DistanceCc",{DistanceMin,DistanceMax});
  MvTuple tuple_DistanceMin;
  MvTuple tuple_DistanceMax;
  mv::DistanceCc(*this,Contour2,MvTuple(MvString(Mode).Text()),&tuple_DistanceMin,&tuple_DistanceMax);
  if(tuple_DistanceMin.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"DistanceCc","Contours",-1,"nonempty iconic array required for a scalar result");
  const double staged_DistanceMin=tuple_DistanceMin.D();
  const double staged_DistanceMax=tuple_DistanceMax.D();
  *DistanceMin=staged_DistanceMin;
  *DistanceMax=staged_DistanceMax;
}
void MvXLDCont::DistanceCc(const MvXLDCont& Contour2, const wchar_t* Mode, double* DistanceMin, double* DistanceMax) const {
  scalar_outputs("DistanceCc",{DistanceMin,DistanceMax});
  MvTuple tuple_DistanceMin;
  MvTuple tuple_DistanceMax;
  mv::DistanceCc(*this,Contour2,MvTuple(MvString(Mode).Text()),&tuple_DistanceMin,&tuple_DistanceMax);
  if(tuple_DistanceMin.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"DistanceCc","Contours",-1,"nonempty iconic array required for a scalar result");
  const double staged_DistanceMin=tuple_DistanceMin.D();
  const double staged_DistanceMax=tuple_DistanceMax.D();
  *DistanceMin=staged_DistanceMin;
  *DistanceMax=staged_DistanceMax;
}
void IntersectionContoursXld(const MvObject& Contour1, const MvObject& Contour2, const MvTuple& IntersectionType, MvTuple* Row, MvTuple* Column, MvTuple* IsOverlapping) {
  cpp_api::TupleOutputs<3> output("IntersectionContoursXld",{Row,Column,IsOverlapping});
  cpp_api::check_status(mv_intersection_contours_xld(Contour1.NativeHandle(),Contour2.NativeHandle(),IntersectionType.ControlInput(),output.Out(0),output.Out(1),output.Out(2)),"IntersectionContoursXld");
  output.Commit();
}
void SmoothContoursXld(const MvObject& Contours, MvObject* SmoothedContours, const MvTuple& NumRegrPoints) {
  cpp_api::require_output(SmoothedContours,"SmoothContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_smooth_contours_xld(Contours.NativeHandle(),&raw,NumRegrPoints.ControlInput()),"SmoothContoursXld");
  detail::MvObjectAccess::Commit(SmoothedContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::SmoothContoursXld(std::int64_t NumRegrPoints) const {
  MvXLDCont result;
  mv::SmoothContoursXld(*this,&result,NumRegrPoints);
  return result;
}

void SelectContoursXld(const MvObject& Contours, MvObject* SelectedContours, const MvTuple& Feature, const MvTuple& Min1, const MvTuple& Max1, const MvTuple& Min2, const MvTuple& Max2) {
  cpp_api::require_output(SelectedContours,"SelectContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_select_contours_xld(Contours.NativeHandle(),&raw,Feature.ControlInput(),Min1.ControlInput(),Max1.ControlInput(),Min2.ControlInput(),Max2.ControlInput()),"SelectContoursXld");
  detail::MvObjectAccess::Commit(SelectedContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::SelectContoursXld(const MvString& Feature, double Min1, double Max1, double Min2, double Max2) const {
  MvXLDCont result;
  mv::SelectContoursXld(*this,&result,MvTuple(Feature.Text()),Min1,Max1,Min2,Max2);
  return result;
}
MvXLDCont MvXLDCont::SelectContoursXld(const char* Feature, double Min1, double Max1, double Min2, double Max2) const {
  MvXLDCont result;
  mv::SelectContoursXld(*this,&result,MvTuple(MvString(Feature).Text()),Min1,Max1,Min2,Max2);
  return result;
}
MvXLDCont MvXLDCont::SelectContoursXld(const wchar_t* Feature, double Min1, double Max1, double Min2, double Max2) const {
  MvXLDCont result;
  mv::SelectContoursXld(*this,&result,MvTuple(MvString(Feature).Text()),Min1,Max1,Min2,Max2);
  return result;
}
void SelectShapeXld(const MvObject& XLD, MvObject* SelectedXLD, const MvTuple& Features, const MvTuple& Operation, const MvTuple& Min, const MvTuple& Max) {
  cpp_api::require_output(SelectedXLD,"SelectShapeXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_select_shape_xld(XLD.NativeHandle(),&raw,Features.ControlInput(),Operation.ControlInput(),Min.ControlInput(),Max.ControlInput()),"SelectShapeXld");
  detail::MvObjectAccess::Commit(SelectedXLD,detail::MvObjectAccess::Adopt(raw));
}
MvXLD MvXLD::SelectShapeXld(const MvTuple& Features, const MvString& Operation, const MvTuple& Min, const MvTuple& Max) const {
  MvXLD result;
  mv::SelectShapeXld(*this,&result,Features,MvTuple(Operation.Text()),Min,Max);
  return result;
}
MvXLD MvXLD::SelectShapeXld(const MvString& Features, const MvString& Operation, double Min, double Max) const {
  MvXLD result;
  mv::SelectShapeXld(*this,&result,MvTuple(Features.Text()),MvTuple(Operation.Text()),Min,Max);
  return result;
}
MvXLD MvXLD::SelectShapeXld(const char* Features, const char* Operation, double Min, double Max) const {
  MvXLD result;
  mv::SelectShapeXld(*this,&result,MvTuple(MvString(Features).Text()),MvTuple(MvString(Operation).Text()),Min,Max);
  return result;
}
MvXLD MvXLD::SelectShapeXld(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const {
  MvXLD result;
  mv::SelectShapeXld(*this,&result,MvTuple(MvString(Features).Text()),MvTuple(MvString(Operation).Text()),Min,Max);
  return result;
}
MvXLDCont MvXLDCont::SelectShapeXld(const MvTuple& Features, const MvString& Operation, const MvTuple& Min, const MvTuple& Max) const {
  MvXLDCont result;
  mv::SelectShapeXld(*this,&result,Features,MvTuple(Operation.Text()),Min,Max);
  return result;
}
MvXLDCont MvXLDCont::SelectShapeXld(const MvString& Features, const MvString& Operation, double Min, double Max) const {
  MvXLDCont result;
  mv::SelectShapeXld(*this,&result,MvTuple(Features.Text()),MvTuple(Operation.Text()),Min,Max);
  return result;
}
MvXLDCont MvXLDCont::SelectShapeXld(const char* Features, const char* Operation, double Min, double Max) const {
  MvXLDCont result;
  mv::SelectShapeXld(*this,&result,MvTuple(MvString(Features).Text()),MvTuple(MvString(Operation).Text()),Min,Max);
  return result;
}
MvXLDCont MvXLDCont::SelectShapeXld(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const {
  MvXLDCont result;
  mv::SelectShapeXld(*this,&result,MvTuple(MvString(Features).Text()),MvTuple(MvString(Operation).Text()),Min,Max);
  return result;
}
void CropContoursXld(const MvObject& Contours, MvObject* CroppedContours, const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Row2, const MvTuple& Col2, const MvTuple& CloseContours) {
  cpp_api::require_output(CroppedContours,"CropContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_crop_contours_xld(Contours.NativeHandle(),&raw,Row1.ControlInput(),Col1.ControlInput(),Row2.ControlInput(),Col2.ControlInput(),CloseContours.ControlInput()),"CropContoursXld");
  detail::MvObjectAccess::Commit(CroppedContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::CropContoursXld(const MvTuple& Row1, const MvTuple& Col1, const MvTuple& Row2, const MvTuple& Col2, const MvString& CloseContours) const {
  MvXLDCont result;
  mv::CropContoursXld(*this,&result,Row1,Col1,Row2,Col2,MvTuple(CloseContours.Text()));
  return result;
}
MvXLDCont MvXLDCont::CropContoursXld(double Row1, double Col1, double Row2, double Col2, const MvString& CloseContours) const {
  MvXLDCont result;
  mv::CropContoursXld(*this,&result,Row1,Col1,Row2,Col2,MvTuple(CloseContours.Text()));
  return result;
}
MvXLDCont MvXLDCont::CropContoursXld(double Row1, double Col1, double Row2, double Col2, const char* CloseContours) const {
  MvXLDCont result;
  mv::CropContoursXld(*this,&result,Row1,Col1,Row2,Col2,MvTuple(MvString(CloseContours).Text()));
  return result;
}
MvXLDCont MvXLDCont::CropContoursXld(double Row1, double Col1, double Row2, double Col2, const wchar_t* CloseContours) const {
  MvXLDCont result;
  mv::CropContoursXld(*this,&result,Row1,Col1,Row2,Col2,MvTuple(MvString(CloseContours).Text()));
  return result;
}
void SortContoursXld(const MvObject& Contours, MvObject* SortedContours, const MvTuple& SortMode, const MvTuple& Order, const MvTuple& RowOrCol) {
  cpp_api::require_output(SortedContours,"SortContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_sort_contours_xld(Contours.NativeHandle(),&raw,SortMode.ControlInput(),Order.ControlInput(),RowOrCol.ControlInput()),"SortContoursXld");
  detail::MvObjectAccess::Commit(SortedContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::SortContoursXld(const MvString& SortMode, const MvString& Order, const MvString& RowOrCol) const {
  MvXLDCont result;
  mv::SortContoursXld(*this,&result,MvTuple(SortMode.Text()),MvTuple(Order.Text()),MvTuple(RowOrCol.Text()));
  return result;
}
MvXLDCont MvXLDCont::SortContoursXld(const char* SortMode, const char* Order, const char* RowOrCol) const {
  MvXLDCont result;
  mv::SortContoursXld(*this,&result,MvTuple(MvString(SortMode).Text()),MvTuple(MvString(Order).Text()),MvTuple(MvString(RowOrCol).Text()));
  return result;
}
MvXLDCont MvXLDCont::SortContoursXld(const wchar_t* SortMode, const wchar_t* Order, const wchar_t* RowOrCol) const {
  MvXLDCont result;
  mv::SortContoursXld(*this,&result,MvTuple(MvString(SortMode).Text()),MvTuple(MvString(Order).Text()),MvTuple(MvString(RowOrCol).Text()));
  return result;
}
void UnionAdjacentContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxDistAbs, const MvTuple& MaxDistRel, const MvTuple& Mode) {
  cpp_api::require_output(UnionContours,"UnionAdjacentContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_union_adjacent_contours_xld(Contours.NativeHandle(),&raw,MaxDistAbs.ControlInput(),MaxDistRel.ControlInput(),Mode.ControlInput()),"UnionAdjacentContoursXld");
  detail::MvObjectAccess::Commit(UnionContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const MvString& Mode) const {
  MvXLDCont result;
  mv::UnionAdjacentContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MvTuple(Mode.Text()));
  return result;
}
MvXLDCont MvXLDCont::UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const char* Mode) const {
  MvXLDCont result;
  mv::UnionAdjacentContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MvTuple(MvString(Mode).Text()));
  return result;
}
MvXLDCont MvXLDCont::UnionAdjacentContoursXld(double MaxDistAbs, double MaxDistRel, const wchar_t* Mode) const {
  MvXLDCont result;
  mv::UnionAdjacentContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MvTuple(MvString(Mode).Text()));
  return result;
}
void UnionCollinearContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxDistAbs, const MvTuple& MaxDistRel, const MvTuple& MaxShift, const MvTuple& MaxAngle, const MvTuple& Mode) {
  cpp_api::require_output(UnionContours,"UnionCollinearContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_union_collinear_contours_xld(Contours.NativeHandle(),&raw,MaxDistAbs.ControlInput(),MaxDistRel.ControlInput(),MaxShift.ControlInput(),MaxAngle.ControlInput(),Mode.ControlInput()),"UnionCollinearContoursXld");
  detail::MvObjectAccess::Commit(UnionContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle, const MvString& Mode) const {
  MvXLDCont result;
  mv::UnionCollinearContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MaxShift,MaxAngle,MvTuple(Mode.Text()));
  return result;
}
MvXLDCont MvXLDCont::UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle, const char* Mode) const {
  MvXLDCont result;
  mv::UnionCollinearContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MaxShift,MaxAngle,MvTuple(MvString(Mode).Text()));
  return result;
}
MvXLDCont MvXLDCont::UnionCollinearContoursXld(double MaxDistAbs, double MaxDistRel, double MaxShift, double MaxAngle, const wchar_t* Mode) const {
  MvXLDCont result;
  mv::UnionCollinearContoursXld(*this,&result,MaxDistAbs,MaxDistRel,MaxShift,MaxAngle,MvTuple(MvString(Mode).Text()));
  return result;
}
void UnionCocircularContoursXld(const MvObject& Contours, MvObject* UnionContours, const MvTuple& MaxArcAngleDiff, const MvTuple& MaxArcOverlap, const MvTuple& MaxTangentAngle, const MvTuple& MaxDist, const MvTuple& MaxRadiusDiff, const MvTuple& MaxCenterDist, const MvTuple& MergeSmallContours, const MvTuple& Iterations) {
  cpp_api::require_output(UnionContours,"UnionCocircularContoursXld");
  ::MvObject raw=nullptr;
  cpp_api::check_status(mv_union_cocircular_contours_xld(Contours.NativeHandle(),&raw,MaxArcAngleDiff.ControlInput(),MaxArcOverlap.ControlInput(),MaxTangentAngle.ControlInput(),MaxDist.ControlInput(),MaxRadiusDiff.ControlInput(),MaxCenterDist.ControlInput(),MergeSmallContours.ControlInput(),Iterations.ControlInput()),"UnionCocircularContoursXld");
  detail::MvObjectAccess::Commit(UnionContours,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::UnionCocircularContoursXld(const MvTuple& MaxArcAngleDiff, const MvTuple& MaxArcOverlap, const MvTuple& MaxTangentAngle, const MvTuple& MaxDist, const MvTuple& MaxRadiusDiff, const MvTuple& MaxCenterDist, const MvString& MergeSmallContours, std::int64_t Iterations) const {
  MvXLDCont result;
  mv::UnionCocircularContoursXld(*this,&result,MaxArcAngleDiff,MaxArcOverlap,MaxTangentAngle,MaxDist,MaxRadiusDiff,MaxCenterDist,MvTuple(MergeSmallContours.Text()),Iterations);
  return result;
}
MvXLDCont MvXLDCont::UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle, double MaxDist, double MaxRadiusDiff, double MaxCenterDist, const MvString& MergeSmallContours, std::int64_t Iterations) const {
  MvXLDCont result;
  mv::UnionCocircularContoursXld(*this,&result,MaxArcAngleDiff,MaxArcOverlap,MaxTangentAngle,MaxDist,MaxRadiusDiff,MaxCenterDist,MvTuple(MergeSmallContours.Text()),Iterations);
  return result;
}
MvXLDCont MvXLDCont::UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle, double MaxDist, double MaxRadiusDiff, double MaxCenterDist, const char* MergeSmallContours, std::int64_t Iterations) const {
  MvXLDCont result;
  mv::UnionCocircularContoursXld(*this,&result,MaxArcAngleDiff,MaxArcOverlap,MaxTangentAngle,MaxDist,MaxRadiusDiff,MaxCenterDist,MvTuple(MvString(MergeSmallContours).Text()),Iterations);
  return result;
}
MvXLDCont MvXLDCont::UnionCocircularContoursXld(double MaxArcAngleDiff, double MaxArcOverlap, double MaxTangentAngle, double MaxDist, double MaxRadiusDiff, double MaxCenterDist, const wchar_t* MergeSmallContours, std::int64_t Iterations) const {
  MvXLDCont result;
  mv::UnionCocircularContoursXld(*this,&result,MaxArcAngleDiff,MaxArcOverlap,MaxTangentAngle,MaxDist,MaxRadiusDiff,MaxCenterDist,MvTuple(MvString(MergeSmallContours).Text()),Iterations);
  return result;
}

void SegmentContoursXld(const MvObject& Contours, MvObject* ContoursSplit, const MvTuple& Mode, const MvTuple& SmoothCont, const MvTuple& MaxLineDist1, const MvTuple& MaxLineDist2) {
 cpp_api::require_output(ContoursSplit,"SegmentContoursXld");::MvObject raw=nullptr;
 cpp_api::check_status(mv_segment_contours_xld(Contours.NativeHandle(),&raw,Mode.ControlInput(),SmoothCont.ControlInput(),MaxLineDist1.ControlInput(),MaxLineDist2.ControlInput()),"SegmentContoursXld");
 detail::MvObjectAccess::Commit(ContoursSplit,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvXLDCont::SegmentContoursXld(const MvString& Mode, std::int64_t SmoothCont, double MaxLineDist1, double MaxLineDist2) const {
 MvXLDCont result;mv::SegmentContoursXld(*this,&result,MvTuple(MvString(Mode).Text()),SmoothCont,MaxLineDist1,MaxLineDist2);return result;
}
MvXLDCont MvXLDCont::SegmentContoursXld(const char* Mode, std::int64_t SmoothCont, double MaxLineDist1, double MaxLineDist2) const {
 MvXLDCont result;mv::SegmentContoursXld(*this,&result,MvTuple(MvString(Mode).Text()),SmoothCont,MaxLineDist1,MaxLineDist2);return result;
}
MvXLDCont MvXLDCont::SegmentContoursXld(const wchar_t* Mode, std::int64_t SmoothCont, double MaxLineDist1, double MaxLineDist2) const {
 MvXLDCont result;mv::SegmentContoursXld(*this,&result,MvTuple(MvString(Mode).Text()),SmoothCont,MaxLineDist1,MaxLineDist2);return result;
}
void EdgesSubPix(const MvObject& Image, MvObject* Edges, const MvTuple& Filter, const MvTuple& Alpha, const MvTuple& Low, const MvTuple& High) {
 cpp_api::require_output(Edges,"EdgesSubPix");::MvObject raw=nullptr;
 cpp_api::check_status(mv_edges_sub_pix(Image.NativeHandle(),&raw,Filter.ControlInput(),Alpha.ControlInput(),Low.ControlInput(),High.ControlInput()),"EdgesSubPix");
 detail::MvObjectAccess::Commit(Edges,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvImage::EdgesSubPix(const MvString& Filter, double Alpha, const MvTuple& Low, const MvTuple& High) const {
 MvXLDCont result;mv::EdgesSubPix(*this,&result,MvTuple(MvString(Filter).Text()),Alpha,Low,High);return result;
}
MvXLDCont MvImage::EdgesSubPix(const MvString& Filter, double Alpha, std::int64_t Low, std::int64_t High) const {
 MvXLDCont result;mv::EdgesSubPix(*this,&result,MvTuple(MvString(Filter).Text()),Alpha,Low,High);return result;
}
MvXLDCont MvImage::EdgesSubPix(const char* Filter, double Alpha, std::int64_t Low, std::int64_t High) const {
 MvXLDCont result;mv::EdgesSubPix(*this,&result,MvTuple(MvString(Filter).Text()),Alpha,Low,High);return result;
}
MvXLDCont MvImage::EdgesSubPix(const wchar_t* Filter, double Alpha, std::int64_t Low, std::int64_t High) const {
 MvXLDCont result;mv::EdgesSubPix(*this,&result,MvTuple(MvString(Filter).Text()),Alpha,Low,High);return result;
}
void ThresholdSubPix(const MvObject& Image, MvObject* Border, const MvTuple& Threshold) {
 cpp_api::require_output(Border,"ThresholdSubPix");::MvObject raw=nullptr;
 cpp_api::check_status(mv_threshold_sub_pix(Image.NativeHandle(),&raw,Threshold.ControlInput()),"ThresholdSubPix");
 detail::MvObjectAccess::Commit(Border,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvImage::ThresholdSubPix(const MvTuple& Threshold) const {
 MvXLDCont result;mv::ThresholdSubPix(*this,&result,Threshold);return result;
}
MvXLDCont MvImage::ThresholdSubPix(double Threshold) const {
 MvXLDCont result;mv::ThresholdSubPix(*this,&result,Threshold);return result;
}
void LinesGauss(const MvObject& Image, MvObject* Lines, const MvTuple& Sigma, const MvTuple& Low, const MvTuple& High, const MvTuple& LightDark, const MvTuple& ExtractWidth, const MvTuple& LineModel, const MvTuple& CompleteJunctions) {
 cpp_api::require_output(Lines,"LinesGauss");::MvObject raw=nullptr;
 cpp_api::check_status(mv_lines_gauss(Image.NativeHandle(),&raw,Sigma.ControlInput(),Low.ControlInput(),High.ControlInput(),LightDark.ControlInput(),ExtractWidth.ControlInput(),LineModel.ControlInput(),CompleteJunctions.ControlInput()),"LinesGauss");
 detail::MvObjectAccess::Commit(Lines,detail::MvObjectAccess::Adopt(raw));
}
MvXLDCont MvImage::LinesGauss(const MvTuple& Sigma, const MvTuple& Low, const MvTuple& High, const MvString& LightDark, const MvString& ExtractWidth, const MvString& LineModel, const MvString& CompleteJunctions) const {
 MvXLDCont result;mv::LinesGauss(*this,&result,Sigma,Low,High,MvTuple(MvString(LightDark).Text()),MvTuple(MvString(ExtractWidth).Text()),MvTuple(MvString(LineModel).Text()),MvTuple(MvString(CompleteJunctions).Text()));return result;
}
MvXLDCont MvImage::LinesGauss(double Sigma, double Low, double High, const MvString& LightDark, const MvString& ExtractWidth, const MvString& LineModel, const MvString& CompleteJunctions) const {
 MvXLDCont result;mv::LinesGauss(*this,&result,Sigma,Low,High,MvTuple(MvString(LightDark).Text()),MvTuple(MvString(ExtractWidth).Text()),MvTuple(MvString(LineModel).Text()),MvTuple(MvString(CompleteJunctions).Text()));return result;
}
MvXLDCont MvImage::LinesGauss(double Sigma, double Low, double High, const char* LightDark, const char* ExtractWidth, const char* LineModel, const char* CompleteJunctions) const {
 MvXLDCont result;mv::LinesGauss(*this,&result,Sigma,Low,High,MvTuple(MvString(LightDark).Text()),MvTuple(MvString(ExtractWidth).Text()),MvTuple(MvString(LineModel).Text()),MvTuple(MvString(CompleteJunctions).Text()));return result;
}
MvXLDCont MvImage::LinesGauss(double Sigma, double Low, double High, const wchar_t* LightDark, const wchar_t* ExtractWidth, const wchar_t* LineModel, const wchar_t* CompleteJunctions) const {
 MvXLDCont result;mv::LinesGauss(*this,&result,Sigma,Low,High,MvTuple(MvString(LightDark).Text()),MvTuple(MvString(ExtractWidth).Text()),MvTuple(MvString(LineModel).Text()),MvTuple(MvString(CompleteJunctions).Text()));return result;
}

void FitLineContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& ClippingEndPoints, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* RowBegin, MvTuple* ColBegin, MvTuple* RowEnd, MvTuple* ColEnd, MvTuple* Nr, MvTuple* Nc, MvTuple* Dist){cpp_api::TupleOutputs<7> output("FitLineContourXld",{RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist});cpp_api::check_status(mv_fit_line_contour_xld(Contours.NativeHandle(),Algorithm.ControlInput(),MaxNumPoints.ControlInput(),ClippingEndPoints.ControlInput(),Iterations.ControlInput(),ClippingFactor.ControlInput(),output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4),output.Out(5),output.Out(6)),"FitLineContourXld");output.Commit();}

void MvXLDCont::FitLineContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, MvTuple* RowBegin, MvTuple* ColBegin, MvTuple* RowEnd, MvTuple* ColEnd, MvTuple* Nr, MvTuple* Nc, MvTuple* Dist) const{mv::FitLineContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,ClippingEndPoints,Iterations,ClippingFactor,RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist);}

void MvXLDCont::FitLineContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin, double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const{scalar_outputs("FitLineContourXld",{RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist});std::array<MvTuple,7> values;mv::FitLineContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitLineContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();*RowBegin=numbers[0];*ColBegin=numbers[1];*RowEnd=numbers[2];*ColEnd=numbers[3];*Nr=numbers[4];*Nc=numbers[5];*Dist=numbers[6];}

void MvXLDCont::FitLineContourXld(const char* Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin, double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const{scalar_outputs("FitLineContourXld",{RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist});std::array<MvTuple,7> values;mv::FitLineContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitLineContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();*RowBegin=numbers[0];*ColBegin=numbers[1];*RowEnd=numbers[2];*ColEnd=numbers[3];*Nr=numbers[4];*Nc=numbers[5];*Dist=numbers[6];}

void MvXLDCont::FitLineContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* RowBegin, double* ColBegin, double* RowEnd, double* ColEnd, double* Nr, double* Nc, double* Dist) const{scalar_outputs("FitLineContourXld",{RowBegin,ColBegin,RowEnd,ColEnd,Nr,Nc,Dist});std::array<MvTuple,7> values;mv::FitLineContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitLineContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();*RowBegin=numbers[0];*ColBegin=numbers[1];*RowEnd=numbers[2];*ColEnd=numbers[3];*Nr=numbers[4];*Nc=numbers[5];*Dist=numbers[6];}

void FitCircleContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& MaxClosureDist, const MvTuple& ClippingEndPoints, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Radius, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder){cpp_api::TupleOutputs<6> output("FitCircleContourXld",{Row,Column,Radius,StartPhi,EndPhi,PointOrder});cpp_api::check_status(mv_fit_circle_contour_xld(Contours.NativeHandle(),Algorithm.ControlInput(),MaxNumPoints.ControlInput(),MaxClosureDist.ControlInput(),ClippingEndPoints.ControlInput(),Iterations.ControlInput(),ClippingFactor.ControlInput(),output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4),output.Out(5)),"FitCircleContourXld");output.Commit();}

void MvXLDCont::FitCircleContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Radius, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder) const{mv::FitCircleContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,Iterations,ClippingFactor,Row,Column,Radius,StartPhi,EndPhi,PointOrder);}

void MvXLDCont::FitCircleContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitCircleContourXld",{Row,Column,Radius,StartPhi,EndPhi,PointOrder});std::array<MvTuple,6> values;mv::FitCircleContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitCircleContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,5> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();MvString order=values[5].S();*Row=numbers[0];*Column=numbers[1];*Radius=numbers[2];*StartPhi=numbers[3];*EndPhi=numbers[4];*PointOrder=order;}

void MvXLDCont::FitCircleContourXld(const char* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitCircleContourXld",{Row,Column,Radius,StartPhi,EndPhi,PointOrder});std::array<MvTuple,6> values;mv::FitCircleContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitCircleContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,5> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();MvString order=values[5].S();*Row=numbers[0];*Column=numbers[1];*Radius=numbers[2];*StartPhi=numbers[3];*EndPhi=numbers[4];*PointOrder=order;}

void MvXLDCont::FitCircleContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Radius, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitCircleContourXld",{Row,Column,Radius,StartPhi,EndPhi,PointOrder});std::array<MvTuple,6> values;mv::FitCircleContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitCircleContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,5> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();MvString order=values[5].S();*Row=numbers[0];*Column=numbers[1];*Radius=numbers[2];*StartPhi=numbers[3];*EndPhi=numbers[4];*PointOrder=order;}

void FitEllipseContourXld(const MvObject& Contours, const MvTuple& Algorithm, const MvTuple& MaxNumPoints, const MvTuple& MaxClosureDist, const MvTuple& ClippingEndPoints, const MvTuple& VossTabSize, const MvTuple& Iterations, const MvTuple& ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Radius1, MvTuple* Radius2, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder){cpp_api::TupleOutputs<8> output("FitEllipseContourXld",{Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder});cpp_api::check_status(mv_fit_ellipse_contour_xld(Contours.NativeHandle(),Algorithm.ControlInput(),MaxNumPoints.ControlInput(),MaxClosureDist.ControlInput(),ClippingEndPoints.ControlInput(),VossTabSize.ControlInput(),Iterations.ControlInput(),ClippingFactor.ControlInput(),output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4),output.Out(5),output.Out(6),output.Out(7)),"FitEllipseContourXld");output.Commit();}

void MvXLDCont::FitEllipseContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations, double ClippingFactor, MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Radius1, MvTuple* Radius2, MvTuple* StartPhi, MvTuple* EndPhi, MvTuple* PointOrder) const{mv::FitEllipseContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,VossTabSize,Iterations,ClippingFactor,Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder);}

void MvXLDCont::FitEllipseContourXld(const MvString& Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1, double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitEllipseContourXld",{Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder});std::array<MvTuple,8> values;mv::FitEllipseContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,VossTabSize,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6],&values[7]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitEllipseContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();MvString order=values[7].S();*Row=numbers[0];*Column=numbers[1];*Phi=numbers[2];*Radius1=numbers[3];*Radius2=numbers[4];*StartPhi=numbers[5];*EndPhi=numbers[6];*PointOrder=order;}

void MvXLDCont::FitEllipseContourXld(const char* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1, double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitEllipseContourXld",{Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder});std::array<MvTuple,8> values;mv::FitEllipseContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,VossTabSize,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6],&values[7]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitEllipseContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();MvString order=values[7].S();*Row=numbers[0];*Column=numbers[1];*Phi=numbers[2];*Radius1=numbers[3];*Radius2=numbers[4];*StartPhi=numbers[5];*EndPhi=numbers[6];*PointOrder=order;}

void MvXLDCont::FitEllipseContourXld(const wchar_t* Algorithm, std::int64_t MaxNumPoints, double MaxClosureDist, std::int64_t ClippingEndPoints, std::int64_t VossTabSize, std::int64_t Iterations, double ClippingFactor, double* Row, double* Column, double* Phi, double* Radius1, double* Radius2, double* StartPhi, double* EndPhi, MvString* PointOrder) const{scalar_outputs("FitEllipseContourXld",{Row,Column,Phi,Radius1,Radius2,StartPhi,EndPhi,PointOrder});std::array<MvTuple,8> values;mv::FitEllipseContourXld(*this,MvTuple(MvString(Algorithm).Text()),MaxNumPoints,MaxClosureDist,ClippingEndPoints,VossTabSize,Iterations,ClippingFactor,&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6],&values[7]);for(const auto& v:values)if(v.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"FitEllipseContourXld","Contours",-1,"scalar fitting outputs require a nonempty result");std::array<double,7> numbers;numbers[0]=values[0].D();numbers[1]=values[1].D();numbers[2]=values[2].D();numbers[3]=values[3].D();numbers[4]=values[4].D();numbers[5]=values[5].D();numbers[6]=values[6].D();MvString order=values[7].S();*Row=numbers[0];*Column=numbers[1];*Phi=numbers[2];*Radius1=numbers[3];*Radius2=numbers[4];*StartPhi=numbers[5];*EndPhi=numbers[6];*PointOrder=order;}

namespace Extensions {

}
}
