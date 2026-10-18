#include "api_bridge.hpp"
#include <mvkit/cpp/mv_region.hpp>
#include <mvkit/cpp/mv_geometry.hpp>
#include <mvkit/c/mv_region.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <utility>
namespace mv {
void GenEmptyRegion(MvObject* EmptyRegion) {
  cpp_api::require_output(EmptyRegion,"GenEmptyRegion");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_empty_region(&raw),"GenEmptyRegion");
  detail::MvObjectAccess::Commit(EmptyRegion,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenEmptyRegion() {mv::GenEmptyRegion(this);}
void GenRegionRuns(MvObject* Region,const MvTuple& Row,const MvTuple& ColumnBegin,const MvTuple& ColumnEnd) {
  cpp_api::require_output(Region,"GenRegionRuns");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_region_runs(&raw,Row.ControlInput(),ColumnBegin.ControlInput(),ColumnEnd.ControlInput()),"GenRegionRuns");
  detail::MvObjectAccess::Commit(Region,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenRegionRuns(const MvTuple& Row,const MvTuple& ColumnBegin,const MvTuple& ColumnEnd) {mv::GenRegionRuns(this,Row,ColumnBegin,ColumnEnd);}
void MvRegion::GenRegionRuns(std::int64_t Row,std::int64_t ColumnBegin,std::int64_t ColumnEnd) {GenRegionRuns(MvTuple(Row),MvTuple(ColumnBegin),MvTuple(ColumnEnd));}
void GetRegionRuns(const MvObject& Region,MvTuple* Row,MvTuple* ColumnBegin,MvTuple* ColumnEnd) {
  cpp_api::TupleOutputs<3> output("GetRegionRuns",{Row,ColumnBegin,ColumnEnd});
  cpp_api::check_status(mv_get_region_runs(Region.NativeHandle(),output.Out(0),output.Out(1),output.Out(2)),"GetRegionRuns");output.Commit();
}
void MvRegion::GetRegionRuns(MvTuple* Row,MvTuple* ColumnBegin,MvTuple* ColumnEnd) const {mv::GetRegionRuns(*this,Row,ColumnBegin,ColumnEnd);}
namespace {
template<class T,std::size_t N> void scalar_outputs(const char* op,const std::array<T*,N>& outputs) {
  for(std::size_t i=0;i<N;++i) {
    cpp_api::require_output(outputs[i],op);
    const auto a=reinterpret_cast<std::uintptr_t>(outputs[i]);
    for(std::size_t j=0;j<i;++j) {
      const auto b=reinterpret_cast<std::uintptr_t>(outputs[j]);
      if((a<=b?b-a:a-b)<sizeof(T))throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,{op,std::strlen(op)},"Output",-1,"output byte ranges must be distinct");
    }
  }
}
void scalar_cardinality(const MvTuple& tuple,const char* op) {
  if(tuple.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,{op,std::strlen(op)},"Region",-1,"scalar query requires a nonempty region array");
}
}
void Union1(const MvObject& Region,MvObject* RegionUnion) {
  cpp_api::require_output(RegionUnion,"Union1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_union1(Region.NativeHandle(),&raw),"Union1");
  detail::MvObjectAccess::Commit(RegionUnion,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Union1() const {
  MvRegion result;mv::Union1(*this,&result);return result;
}
void Union2(const MvObject& Region1,const MvObject& Region2,MvObject* RegionUnion) {
  cpp_api::require_output(RegionUnion,"Union2");::MvObject raw=nullptr;
  cpp_api::check_status(mv_union2(Region1.NativeHandle(),Region2.NativeHandle(),&raw),"Union2");
  detail::MvObjectAccess::Commit(RegionUnion,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Union2(const MvRegion& Region2) const {
  MvRegion result;mv::Union2(*this,Region2,&result);return result;
}
void Intersection(const MvObject& Region1,const MvObject& Region2,MvObject* RegionIntersection) {
  cpp_api::require_output(RegionIntersection,"Intersection");::MvObject raw=nullptr;
  cpp_api::check_status(mv_intersection(Region1.NativeHandle(),Region2.NativeHandle(),&raw),"Intersection");
  detail::MvObjectAccess::Commit(RegionIntersection,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Intersection(const MvRegion& Region2) const {
  MvRegion result;mv::Intersection(*this,Region2,&result);return result;
}
void Difference(const MvObject& Region,const MvObject& Sub,MvObject* RegionDifference) {
  cpp_api::require_output(RegionDifference,"Difference");::MvObject raw=nullptr;
  cpp_api::check_status(mv_difference(Region.NativeHandle(),Sub.NativeHandle(),&raw),"Difference");
  detail::MvObjectAccess::Commit(RegionDifference,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Difference(const MvRegion& Sub) const {
  MvRegion result;mv::Difference(*this,Sub,&result);return result;
}
void Connection(const MvObject& Region,MvObject* ConnectedRegions) {
  cpp_api::require_output(ConnectedRegions,"Connection");::MvObject raw=nullptr;
  cpp_api::check_status(mv_connection(Region.NativeHandle(),&raw),"Connection");
  detail::MvObjectAccess::Commit(ConnectedRegions,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Connection() const {
  MvRegion result;mv::Connection(*this,&result);return result;
}
void FillUp(const MvObject& Region,MvObject* RegionFillUp) {
  cpp_api::require_output(RegionFillUp,"FillUp");::MvObject raw=nullptr;
  cpp_api::check_status(mv_fill_up(Region.NativeHandle(),&raw),"FillUp");
  detail::MvObjectAccess::Commit(RegionFillUp,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::FillUp() const {
  MvRegion result;mv::FillUp(*this,&result);return result;
}
void AreaCenter(const MvObject& Regions,MvTuple* Area,MvTuple* Row,MvTuple* Column) {
  cpp_api::TupleOutputs<3> output("AreaCenter",{Area,Row,Column});
  cpp_api::check_status(mv_area_center(Regions.NativeHandle(),output.Out(0),output.Out(1),output.Out(2)),"AreaCenter");output.Commit();
}
MvTuple MvRegion::AreaCenter(MvTuple* Row, MvTuple* Column) const {
  MvTuple result;mv::AreaCenter(*this,&result,Row,Column);return result;
}
std::int64_t MvRegion::AreaCenter(double* Row, double* Column) const {
  scalar_outputs<double,2>("AreaCenter",{Row,Column});
  MvTuple tuple_Area,tuple_Row,tuple_Column;
  mv::AreaCenter(*this,&tuple_Area,&tuple_Row,&tuple_Column);
  scalar_cardinality(tuple_Area,"AreaCenter");
  const std::array<double,2> staged{tuple_Row.D(),tuple_Column.D()};
  const auto result=tuple_Area.I();
  *Row=staged[0];*Column=staged[1];return result;
}
void SmallestRectangle1(const MvObject& Regions,MvTuple* Row1,MvTuple* Column1,MvTuple* Row2,MvTuple* Column2) {
  cpp_api::TupleOutputs<4> output("SmallestRectangle1",{Row1,Column1,Row2,Column2});
  cpp_api::check_status(mv_smallest_rectangle1(Regions.NativeHandle(),output.Out(0),output.Out(1),output.Out(2),output.Out(3)),"SmallestRectangle1");output.Commit();
}
void MvRegion::SmallestRectangle1(MvTuple* Row1, MvTuple* Column1, MvTuple* Row2, MvTuple* Column2) const {
  mv::SmallestRectangle1(*this,Row1,Column1,Row2,Column2);
}
void MvRegion::SmallestRectangle1(std::int64_t* Row1, std::int64_t* Column1, std::int64_t* Row2, std::int64_t* Column2) const {
  scalar_outputs<std::int64_t,4>("SmallestRectangle1",{Row1,Column1,Row2,Column2});
  MvTuple tuple_Row1,tuple_Column1,tuple_Row2,tuple_Column2;
  mv::SmallestRectangle1(*this,&tuple_Row1,&tuple_Column1,&tuple_Row2,&tuple_Column2);
  scalar_cardinality(tuple_Row1,"SmallestRectangle1");
  const std::array<std::int64_t,4> staged{tuple_Row1.I(),tuple_Column1.I(),tuple_Row2.I(),tuple_Column2.I()};
  *Row1=staged[0];*Column1=staged[1];*Row2=staged[2];*Column2=staged[3];
}
void SmallestRectangle2(const MvObject& Regions,MvTuple* Row,MvTuple* Column,MvTuple* Phi,MvTuple* Length1,MvTuple* Length2) {
  cpp_api::TupleOutputs<5> output("SmallestRectangle2",{Row,Column,Phi,Length1,Length2});
  cpp_api::check_status(mv_smallest_rectangle2(Regions.NativeHandle(),output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4)),"SmallestRectangle2");output.Commit();
}
void MvRegion::SmallestRectangle2(MvTuple* Row, MvTuple* Column, MvTuple* Phi, MvTuple* Length1, MvTuple* Length2) const {
  mv::SmallestRectangle2(*this,Row,Column,Phi,Length1,Length2);
}
void MvRegion::SmallestRectangle2(double* Row, double* Column, double* Phi, double* Length1, double* Length2) const {
  scalar_outputs<double,5>("SmallestRectangle2",{Row,Column,Phi,Length1,Length2});
  MvTuple tuple_Row,tuple_Column,tuple_Phi,tuple_Length1,tuple_Length2;
  mv::SmallestRectangle2(*this,&tuple_Row,&tuple_Column,&tuple_Phi,&tuple_Length1,&tuple_Length2);
  scalar_cardinality(tuple_Row,"SmallestRectangle2");
  const std::array<double,5> staged{tuple_Row.D(),tuple_Column.D(),tuple_Phi.D(),tuple_Length1.D(),tuple_Length2.D()};
  *Row=staged[0];*Column=staged[1];*Phi=staged[2];*Length1=staged[3];*Length2=staged[4];
}
void SmallestCircle(const MvObject& Regions,MvTuple* Row,MvTuple* Column,MvTuple* Radius) {
  cpp_api::TupleOutputs<3> output("SmallestCircle",{Row,Column,Radius});
  cpp_api::check_status(mv_smallest_circle(Regions.NativeHandle(),output.Out(0),output.Out(1),output.Out(2)),"SmallestCircle");output.Commit();
}
void MvRegion::SmallestCircle(MvTuple* Row, MvTuple* Column, MvTuple* Radius) const {
  mv::SmallestCircle(*this,Row,Column,Radius);
}
void MvRegion::SmallestCircle(double* Row, double* Column, double* Radius) const {
  scalar_outputs<double,3>("SmallestCircle",{Row,Column,Radius});
  MvTuple tuple_Row,tuple_Column,tuple_Radius;
  mv::SmallestCircle(*this,&tuple_Row,&tuple_Column,&tuple_Radius);
  scalar_cardinality(tuple_Row,"SmallestCircle");
  const std::array<double,3> staged{tuple_Row.D(),tuple_Column.D(),tuple_Radius.D()};
  *Row=staged[0];*Column=staged[1];*Radius=staged[2];
}
void Circularity(const MvObject& Regions,MvTuple* Circularity) {
  cpp_api::TupleOutputs<1> output("Circularity",{Circularity});
  cpp_api::check_status(mv_circularity(Regions.NativeHandle(),output.Out(0)),"Circularity");output.Commit();
}
MvTuple MvRegion::Circularity() const {
  MvTuple result;mv::Circularity(*this,&result);return result;
}
void OrientationRegion(const MvObject& Regions,MvTuple* Phi) {
  cpp_api::TupleOutputs<1> output("OrientationRegion",{Phi});
  cpp_api::check_status(mv_orientation_region(Regions.NativeHandle(),output.Out(0)),"OrientationRegion");output.Commit();
}
MvTuple MvRegion::OrientationRegion() const {
  MvTuple result;mv::OrientationRegion(*this,&result);return result;
}
void Eccentricity(const MvObject& Regions,MvTuple* Anisometry,MvTuple* Bulkiness,MvTuple* StructureFactor) {
  cpp_api::TupleOutputs<3> output("Eccentricity",{Anisometry,Bulkiness,StructureFactor});
  cpp_api::check_status(mv_eccentricity(Regions.NativeHandle(),output.Out(0),output.Out(1),output.Out(2)),"Eccentricity");output.Commit();
}
MvTuple MvRegion::Eccentricity(MvTuple* Bulkiness, MvTuple* StructureFactor) const {
  MvTuple result;mv::Eccentricity(*this,&result,Bulkiness,StructureFactor);return result;
}
double MvRegion::Eccentricity(double* Bulkiness, double* StructureFactor) const {
  scalar_outputs<double,2>("Eccentricity",{Bulkiness,StructureFactor});
  MvTuple tuple_Anisometry,tuple_Bulkiness,tuple_StructureFactor;
  mv::Eccentricity(*this,&tuple_Anisometry,&tuple_Bulkiness,&tuple_StructureFactor);
  scalar_cardinality(tuple_Anisometry,"Eccentricity");
  const std::array<double,2> staged{tuple_Bulkiness.D(),tuple_StructureFactor.D()};
  const auto result=tuple_Anisometry.D();
  *Bulkiness=staged[0];*StructureFactor=staged[1];return result;
}
void GenRectangle1(MvObject* Rectangle, const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) {
  cpp_api::require_output(Rectangle,"GenRectangle1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_rectangle1(&raw, Row1.ControlInput(), Column1.ControlInput(), Row2.ControlInput(), Column2.ControlInput()),"GenRectangle1");
  detail::MvObjectAccess::Commit(Rectangle,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenRectangle1(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) { mv::GenRectangle1(this,Row1, Column1, Row2, Column2); }
void MvRegion::GenRectangle1(double Row1, double Column1, double Row2, double Column2) { mv::GenRectangle1(this,MvTuple(Row1), MvTuple(Column1), MvTuple(Row2), MvTuple(Column2)); }
void GenRectangle2(MvObject* Rectangle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Length1, const MvTuple& Length2) {
  cpp_api::require_output(Rectangle,"GenRectangle2");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_rectangle2(&raw, Row.ControlInput(), Column.ControlInput(), Phi.ControlInput(), Length1.ControlInput(), Length2.ControlInput()),"GenRectangle2");
  detail::MvObjectAccess::Commit(Rectangle,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenRectangle2(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Length1, const MvTuple& Length2) { mv::GenRectangle2(this,Row, Column, Phi, Length1, Length2); }
void MvRegion::GenRectangle2(double Row, double Column, double Phi, double Length1, double Length2) { mv::GenRectangle2(this,MvTuple(Row), MvTuple(Column), MvTuple(Phi), MvTuple(Length1), MvTuple(Length2)); }
void GenCircle(MvObject* Circle, const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius) {
  cpp_api::require_output(Circle,"GenCircle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_circle(&raw, Row.ControlInput(), Column.ControlInput(), Radius.ControlInput()),"GenCircle");
  detail::MvObjectAccess::Commit(Circle,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenCircle(const MvTuple& Row, const MvTuple& Column, const MvTuple& Radius) { mv::GenCircle(this,Row, Column, Radius); }
void MvRegion::GenCircle(double Row, double Column, double Radius) { mv::GenCircle(this,MvTuple(Row), MvTuple(Column), MvTuple(Radius)); }
void DilationCircle(const MvObject& Region, MvObject* RegionDilation, const MvTuple& Radius) {
  cpp_api::require_output(RegionDilation,"DilationCircle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_dilation_circle(Region.NativeHandle(), &raw, Radius.ControlInput()),"DilationCircle");
  detail::MvObjectAccess::Commit(RegionDilation,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::DilationCircle(const MvTuple& Radius) const { MvRegion result;mv::DilationCircle(*this,&result,Radius);return result; }
MvRegion MvRegion::DilationCircle(double Radius) const { MvRegion result;mv::DilationCircle(*this,&result,MvTuple(Radius));return result; }
void ErosionCircle(const MvObject& Region, MvObject* RegionErosion, const MvTuple& Radius) {
  cpp_api::require_output(RegionErosion,"ErosionCircle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_erosion_circle(Region.NativeHandle(), &raw, Radius.ControlInput()),"ErosionCircle");
  detail::MvObjectAccess::Commit(RegionErosion,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::ErosionCircle(const MvTuple& Radius) const { MvRegion result;mv::ErosionCircle(*this,&result,Radius);return result; }
MvRegion MvRegion::ErosionCircle(double Radius) const { MvRegion result;mv::ErosionCircle(*this,&result,MvTuple(Radius));return result; }
void OpeningCircle(const MvObject& Region, MvObject* RegionOpening, const MvTuple& Radius) {
  cpp_api::require_output(RegionOpening,"OpeningCircle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_opening_circle(Region.NativeHandle(), &raw, Radius.ControlInput()),"OpeningCircle");
  detail::MvObjectAccess::Commit(RegionOpening,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::OpeningCircle(const MvTuple& Radius) const { MvRegion result;mv::OpeningCircle(*this,&result,Radius);return result; }
MvRegion MvRegion::OpeningCircle(double Radius) const { MvRegion result;mv::OpeningCircle(*this,&result,MvTuple(Radius));return result; }
void ClosingCircle(const MvObject& Region, MvObject* RegionClosing, const MvTuple& Radius) {
  cpp_api::require_output(RegionClosing,"ClosingCircle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_closing_circle(Region.NativeHandle(), &raw, Radius.ControlInput()),"ClosingCircle");
  detail::MvObjectAccess::Commit(RegionClosing,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::ClosingCircle(const MvTuple& Radius) const { MvRegion result;mv::ClosingCircle(*this,&result,Radius);return result; }
MvRegion MvRegion::ClosingCircle(double Radius) const { MvRegion result;mv::ClosingCircle(*this,&result,MvTuple(Radius));return result; }
void DilationRectangle1(const MvObject& Region, MvObject* RegionDilation, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(RegionDilation,"DilationRectangle1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_dilation_rectangle1(Region.NativeHandle(), &raw, Width.ControlInput(), Height.ControlInput()),"DilationRectangle1");
  detail::MvObjectAccess::Commit(RegionDilation,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::DilationRectangle1(std::int64_t Width, std::int64_t Height) const { MvRegion result;mv::DilationRectangle1(*this,&result,MvTuple(Width), MvTuple(Height));return result; }
void ErosionRectangle1(const MvObject& Region, MvObject* RegionErosion, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(RegionErosion,"ErosionRectangle1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_erosion_rectangle1(Region.NativeHandle(), &raw, Width.ControlInput(), Height.ControlInput()),"ErosionRectangle1");
  detail::MvObjectAccess::Commit(RegionErosion,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::ErosionRectangle1(std::int64_t Width, std::int64_t Height) const { MvRegion result;mv::ErosionRectangle1(*this,&result,MvTuple(Width), MvTuple(Height));return result; }
void OpeningRectangle1(const MvObject& Region, MvObject* RegionOpening, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(RegionOpening,"OpeningRectangle1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_opening_rectangle1(Region.NativeHandle(), &raw, Width.ControlInput(), Height.ControlInput()),"OpeningRectangle1");
  detail::MvObjectAccess::Commit(RegionOpening,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::OpeningRectangle1(std::int64_t Width, std::int64_t Height) const { MvRegion result;mv::OpeningRectangle1(*this,&result,MvTuple(Width), MvTuple(Height));return result; }
void ClosingRectangle1(const MvObject& Region, MvObject* RegionClosing, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(RegionClosing,"ClosingRectangle1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_closing_rectangle1(Region.NativeHandle(), &raw, Width.ControlInput(), Height.ControlInput()),"ClosingRectangle1");
  detail::MvObjectAccess::Commit(RegionClosing,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::ClosingRectangle1(std::int64_t Width, std::int64_t Height) const { MvRegion result;mv::ClosingRectangle1(*this,&result,MvTuple(Width), MvTuple(Height));return result; }
void MoveRegion(const MvObject& Region, MvObject* RegionMoved, const MvTuple& Row, const MvTuple& Column) {
  cpp_api::require_output(RegionMoved,"MoveRegion");::MvObject raw=nullptr;
  cpp_api::check_status(mv_move_region(Region.NativeHandle(), &raw, Row.ControlInput(), Column.ControlInput()),"MoveRegion");
  detail::MvObjectAccess::Commit(RegionMoved,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::MoveRegion(std::int64_t Row, std::int64_t Column) const { MvRegion result;mv::MoveRegion(*this,&result,MvTuple(Row), MvTuple(Column));return result; }

void SymmDifference(const MvObject& Region1, const MvObject& Region2, MvObject* RegionDifference) {
  cpp_api::require_output(RegionDifference,"SymmDifference");::MvObject raw=nullptr;
  cpp_api::check_status(mv_symm_difference(Region1.NativeHandle(),Region2.NativeHandle(),&raw),"SymmDifference");
  detail::MvObjectAccess::Commit(RegionDifference,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::SymmDifference(const MvRegion& Region2) const { MvRegion value;mv::SymmDifference(*this,Region2, &value);return value; }
void TestEqualRegion(const MvObject& Regions1, const MvObject& Regions2, MvTuple* IsEqual) {
  cpp_api::TupleOutputs<1> output("TestEqualRegion",{IsEqual});
  cpp_api::check_status(mv_test_equal_region(Regions1.NativeHandle(),Regions2.NativeHandle(),output.Out(0)),"TestEqualRegion");
  output.Commit();
}
std::int64_t MvRegion::TestEqualRegion(const MvRegion& Regions2) const { MvTuple value;mv::TestEqualRegion(*this,Regions2,&value);scalar_cardinality(value,"TestEqualRegion");return value.I(); }
void TestRegionPoint(const MvObject& Regions, const MvTuple& Row, const MvTuple& Column, MvTuple* IsInside) {
  cpp_api::TupleOutputs<1> output("TestRegionPoint",{IsInside});
  cpp_api::check_status(mv_test_region_point(Regions.NativeHandle(),Row.ControlInput(),Column.ControlInput(),output.Out(0)),"TestRegionPoint");
  output.Commit();
}
std::int64_t MvRegion::TestRegionPoint(const MvTuple& Row, const MvTuple& Column) const { MvTuple value;mv::TestRegionPoint(*this,Row,Column,&value);scalar_cardinality(value,"TestRegionPoint");return value.I(); }
std::int64_t MvRegion::TestRegionPoint(std::int64_t Row, std::int64_t Column) const { MvTuple value;mv::TestRegionPoint(*this,MvTuple(Row),MvTuple(Column),&value);scalar_cardinality(value,"TestRegionPoint");return value.I(); }
void RegionFeatures(const MvObject& Regions, const MvTuple& Features, MvTuple* Value) {
  cpp_api::TupleOutputs<1> output("RegionFeatures",{Value});
  cpp_api::check_status(mv_region_features(Regions.NativeHandle(),Features.ControlInput(),output.Out(0)),"RegionFeatures");
  output.Commit();
}
MvTuple MvRegion::RegionFeatures(const MvTuple& Features) const { MvTuple value;mv::RegionFeatures(*this,Features,&value);return value; }
double MvRegion::RegionFeatures(const MvString& Features) const { auto value=RegionFeatures(MvTuple(MvString(Features)));scalar_cardinality(value,"RegionFeatures");return value.D(); }
double MvRegion::RegionFeatures(const char* Features) const { auto value=RegionFeatures(MvTuple(MvString(Features)));scalar_cardinality(value,"RegionFeatures");return value.D(); }
double MvRegion::RegionFeatures(const wchar_t* Features) const { auto value=RegionFeatures(MvTuple(MvString(Features)));scalar_cardinality(value,"RegionFeatures");return value.D(); }
void SelectShape(const MvObject& Regions, MvObject* SelectedRegions, const MvTuple& Features, const MvTuple& Operation, const MvTuple& Min, const MvTuple& Max) {
  cpp_api::require_output(SelectedRegions,"SelectShape");::MvObject raw=nullptr;
  cpp_api::check_status(mv_select_shape(Regions.NativeHandle(),&raw,Features.ControlInput(),Operation.ControlInput(),Min.ControlInput(),Max.ControlInput()),"SelectShape");
  detail::MvObjectAccess::Commit(SelectedRegions,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::SelectShape(const MvTuple& Features, const MvString& Operation, const MvTuple& Min, const MvTuple& Max) const { MvRegion value;mv::SelectShape(*this,&value,Features,MvTuple(Operation),Min,Max);return value; }
MvRegion MvRegion::SelectShape(const MvString& Features, const MvString& Operation, double Min, double Max) const { MvRegion value;mv::SelectShape(*this,&value,MvTuple(Features),MvTuple(Operation),MvTuple(Min),MvTuple(Max));return value; }
MvRegion MvRegion::SelectShape(const char* Features, const char* Operation, double Min, double Max) const { MvRegion value;mv::SelectShape(*this,&value,MvTuple(MvString(Features)),MvTuple(MvString(Operation)),MvTuple(Min),MvTuple(Max));return value; }
MvRegion MvRegion::SelectShape(const wchar_t* Features, const wchar_t* Operation, double Min, double Max) const { MvRegion value;mv::SelectShape(*this,&value,MvTuple(MvString(Features)),MvTuple(MvString(Operation)),MvTuple(Min),MvTuple(Max));return value; }
void SortRegion(const MvObject& Regions, MvObject* SortedRegions, const MvTuple& SortMode, const MvTuple& Order, const MvTuple& RowOrCol) {
  cpp_api::require_output(SortedRegions,"SortRegion");::MvObject raw=nullptr;
  cpp_api::check_status(mv_sort_region(Regions.NativeHandle(),&raw,SortMode.ControlInput(),Order.ControlInput(),RowOrCol.ControlInput()),"SortRegion");
  detail::MvObjectAccess::Commit(SortedRegions,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::SortRegion(const MvTuple& SortMode, const MvString& Order, const MvString& RowOrCol) const { MvRegion value;mv::SortRegion(*this,&value,SortMode,MvTuple(Order),MvTuple(RowOrCol));return value; }
MvRegion MvRegion::SortRegion(const MvTuple& SortMode, const char* Order, const char* RowOrCol) const { MvRegion value;mv::SortRegion(*this,&value,SortMode,MvTuple(MvString(Order)),MvTuple(MvString(RowOrCol)));return value; }
MvRegion MvRegion::SortRegion(const MvTuple& SortMode, const wchar_t* Order, const wchar_t* RowOrCol) const { MvRegion value;mv::SortRegion(*this,&value,SortMode,MvTuple(MvString(Order)),MvTuple(MvString(RowOrCol)));return value; }
void SelectShapeStd(const MvObject& Regions, MvObject* SelectedRegions, const MvTuple& Shape, const MvTuple& Percent) {
  cpp_api::require_output(SelectedRegions,"SelectShapeStd");::MvObject raw=nullptr;
  cpp_api::check_status(mv_select_shape_std(Regions.NativeHandle(),&raw,Shape.ControlInput(),Percent.ControlInput()),"SelectShapeStd");
  detail::MvObjectAccess::Commit(SelectedRegions,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::SelectShapeStd(const MvString& Shape, double Percent) const { MvRegion value;mv::SelectShapeStd(*this,&value,MvTuple(Shape),MvTuple(Percent));return value; }
MvRegion MvRegion::SelectShapeStd(const char* Shape, double Percent) const { MvRegion value;mv::SelectShapeStd(*this,&value,MvTuple(MvString(Shape)),MvTuple(Percent));return value; }
MvRegion MvRegion::SelectShapeStd(const wchar_t* Shape, double Percent) const { MvRegion value;mv::SelectShapeStd(*this,&value,MvTuple(MvString(Shape)),MvTuple(Percent));return value; }
void ShapeTrans(const MvObject& Region, MvObject* RegionTrans, const MvTuple& Type) {
  cpp_api::require_output(RegionTrans,"ShapeTrans");::MvObject raw=nullptr;
  cpp_api::check_status(mv_shape_trans(Region.NativeHandle(),&raw,Type.ControlInput()),"ShapeTrans");
  detail::MvObjectAccess::Commit(RegionTrans,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::ShapeTrans(const MvString& Type) const { MvRegion value;mv::ShapeTrans(*this,&value,MvTuple(Type));return value; }
MvRegion MvRegion::ShapeTrans(const char* Type) const { MvRegion value;mv::ShapeTrans(*this,&value,MvTuple(MvString(Type)));return value; }
MvRegion MvRegion::ShapeTrans(const wchar_t* Type) const { MvRegion value;mv::ShapeTrans(*this,&value,MvTuple(MvString(Type)));return value; }
void GenRegionPolygonFilled(MvObject* Region, const MvTuple& Rows, const MvTuple& Columns) {
  cpp_api::require_output(Region,"GenRegionPolygonFilled");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_region_polygon_filled(&raw,Rows.ControlInput(),Columns.ControlInput()),"GenRegionPolygonFilled");
  detail::MvObjectAccess::Commit(Region,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenRegionPolygonFilled(const MvTuple& Rows,const MvTuple& Columns) {mv::GenRegionPolygonFilled(this,Rows,Columns);}
void Skeleton(const MvObject& Region, MvObject* Skeleton) {
  cpp_api::require_output(Skeleton,"Skeleton");::MvObject raw=nullptr;
  cpp_api::check_status(mv_skeleton(Region.NativeHandle(),&raw),"Skeleton");
  detail::MvObjectAccess::Commit(Skeleton,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Skeleton() const {MvRegion result;mv::Skeleton(*this,&result);return result;}
void PartitionRectangle(const MvObject& Region, MvObject* Partitioned, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(Partitioned,"PartitionRectangle");::MvObject raw=nullptr;
  cpp_api::check_status(mv_partition_rectangle(Region.NativeHandle(),&raw,Width.ControlInput(),Height.ControlInput()),"PartitionRectangle");
  detail::MvObjectAccess::Commit(Partitioned,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::PartitionRectangle(double Width,double Height) const {MvRegion result;mv::PartitionRectangle(*this,&result,MvTuple(Width),MvTuple(Height));return result;}
void PartitionDynamic(const MvObject& Region, MvObject* Partitioned, const MvTuple& Distance, const MvTuple& Percent) {
  cpp_api::require_output(Partitioned,"PartitionDynamic");::MvObject raw=nullptr;
  cpp_api::check_status(mv_partition_dynamic(Region.NativeHandle(),&raw,Distance.ControlInput(),Percent.ControlInput()),"PartitionDynamic");
  detail::MvObjectAccess::Commit(Partitioned,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::PartitionDynamic(double Distance,double Percent) const {MvRegion result;mv::PartitionDynamic(*this,&result,MvTuple(Distance),MvTuple(Percent));return result;}
void GenEllipse(MvObject* Ellipse, const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2) {
  cpp_api::require_output(Ellipse,"GenEllipse");::MvObject raw=nullptr;
  cpp_api::check_status(mv_gen_ellipse(&raw,Row.ControlInput(),Column.ControlInput(),Phi.ControlInput(),Radius1.ControlInput(),Radius2.ControlInput()),"GenEllipse");
  detail::MvObjectAccess::Commit(Ellipse,detail::MvObjectAccess::Adopt(raw));
}
void MvRegion::GenEllipse(const MvTuple& Row, const MvTuple& Column, const MvTuple& Phi, const MvTuple& Radius1, const MvTuple& Radius2) {mv::GenEllipse(this,Row,Column,Phi,Radius1,Radius2);}
void MvRegion::GenEllipse(double Row, double Column, double Phi, double Radius1, double Radius2) {mv::GenEllipse(this,MvTuple(Row),MvTuple(Column),MvTuple(Phi),MvTuple(Radius1),MvTuple(Radius2));}
void AffineTransRegion(const MvObject& Region, MvObject* RegionAffineTrans, const MvTuple& HomMat2D, const MvTuple& Interpolate) {
  cpp_api::require_output(RegionAffineTrans,"AffineTransRegion");::MvObject raw=nullptr;
  cpp_api::check_status(mv_affine_trans_region(Region.NativeHandle(),&raw,HomMat2D.ControlInput(),Interpolate.ControlInput()),"AffineTransRegion");
  detail::MvObjectAccess::Commit(RegionAffineTrans,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvHomMat2D::AffineTransRegion(const MvRegion& Region, const MvString& Interpolate) const {MvRegion result;mv::AffineTransRegion(Region,&result,*this,MvTuple(MvString(Interpolate)));return result;}
MvRegion MvHomMat2D::AffineTransRegion(const MvRegion& Region, const char* Interpolate) const {MvRegion result;mv::AffineTransRegion(Region,&result,*this,MvTuple(MvString(Interpolate)));return result;}
MvRegion MvHomMat2D::AffineTransRegion(const MvRegion& Region, const wchar_t* Interpolate) const {MvRegion result;mv::AffineTransRegion(Region,&result,*this,MvTuple(MvString(Interpolate)));return result;}
MvRegion MvRegion::AffineTransRegion(const MvHomMat2D& HomMat2D, const MvString& Interpolate) const {MvRegion result;mv::AffineTransRegion(*this,&result,HomMat2D,MvTuple(MvString(Interpolate)));return result;}
MvRegion MvRegion::AffineTransRegion(const MvHomMat2D& HomMat2D, const char* Interpolate) const {MvRegion result;mv::AffineTransRegion(*this,&result,HomMat2D,MvTuple(MvString(Interpolate)));return result;}
MvRegion MvRegion::AffineTransRegion(const MvHomMat2D& HomMat2D, const wchar_t* Interpolate) const {MvRegion result;mv::AffineTransRegion(*this,&result,HomMat2D,MvTuple(MvString(Interpolate)));return result;}
void Boundary(const MvObject& Region, MvObject* RegionBorder, const MvTuple& BoundaryType) {
  cpp_api::require_output(RegionBorder,"Boundary");::MvObject raw=nullptr;
  cpp_api::check_status(mv_boundary(Region.NativeHandle(),&raw,BoundaryType.ControlInput()),"Boundary");
  detail::MvObjectAccess::Commit(RegionBorder,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Boundary(const MvString& BoundaryType) const {MvRegion result;mv::Boundary(*this,&result,MvTuple(MvString(BoundaryType)));return result;}
MvRegion MvRegion::Boundary(const char* BoundaryType) const {MvRegion result;mv::Boundary(*this,&result,MvTuple(MvString(BoundaryType)));return result;}
MvRegion MvRegion::Boundary(const wchar_t* BoundaryType) const {MvRegion result;mv::Boundary(*this,&result,MvTuple(MvString(BoundaryType)));return result;}
void FillUpShape(const MvObject& Region, MvObject* RegionFillUp, const MvTuple& Feature, const MvTuple& Min, const MvTuple& Max) {
  cpp_api::require_output(RegionFillUp,"FillUpShape");::MvObject raw=nullptr;
  cpp_api::check_status(mv_fill_up_shape(Region.NativeHandle(),&raw,Feature.ControlInput(),Min.ControlInput(),Max.ControlInput()),"FillUpShape");
  detail::MvObjectAccess::Commit(RegionFillUp,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::FillUpShape(const MvString& Feature, const MvTuple& Min, const MvTuple& Max) const {MvRegion result;mv::FillUpShape(*this,&result,MvTuple(MvString(Feature)),Min,Max);return result;}
MvRegion MvRegion::FillUpShape(const MvString& Feature, double Min, double Max) const {MvRegion result;mv::FillUpShape(*this,&result,MvTuple(MvString(Feature)),MvTuple(Min),MvTuple(Max));return result;}
MvRegion MvRegion::FillUpShape(const char* Feature, double Min, double Max) const {MvRegion result;mv::FillUpShape(*this,&result,MvTuple(MvString(Feature)),MvTuple(Min),MvTuple(Max));return result;}
MvRegion MvRegion::FillUpShape(const wchar_t* Feature, double Min, double Max) const {MvRegion result;mv::FillUpShape(*this,&result,MvTuple(MvString(Feature)),MvTuple(Min),MvTuple(Max));return result;}
void DistanceTransform(const MvObject& Region, MvObject* DistanceImage, const MvTuple& Metric, const MvTuple& Foreground, const MvTuple& Width, const MvTuple& Height) {
  cpp_api::require_output(DistanceImage,"DistanceTransform");::MvObject raw=nullptr;
  cpp_api::check_status(mv_distance_transform(Region.NativeHandle(),&raw,Metric.ControlInput(),Foreground.ControlInput(),Width.ControlInput(),Height.ControlInput()),"DistanceTransform");
  detail::MvObjectAccess::Commit(DistanceImage,detail::MvObjectAccess::Adopt(raw));
}
MvImage MvRegion::DistanceTransform(const MvString& Metric, const MvString& Foreground, std::int64_t Width, std::int64_t Height) const {MvImage result;mv::DistanceTransform(*this,&result,MvTuple(MvString(Metric)),MvTuple(MvString(Foreground)),MvTuple(Width),MvTuple(Height));return result;}
MvImage MvRegion::DistanceTransform(const char* Metric, const char* Foreground, std::int64_t Width, std::int64_t Height) const {MvImage result;mv::DistanceTransform(*this,&result,MvTuple(MvString(Metric)),MvTuple(MvString(Foreground)),MvTuple(Width),MvTuple(Height));return result;}
MvImage MvRegion::DistanceTransform(const wchar_t* Metric, const wchar_t* Foreground, std::int64_t Width, std::int64_t Height) const {MvImage result;mv::DistanceTransform(*this,&result,MvTuple(MvString(Metric)),MvTuple(MvString(Foreground)),MvTuple(Width),MvTuple(Height));return result;}
void Dilation1(const MvObject& Region, const MvObject& StructElement, MvObject* RegionDilation, const MvTuple& Iterations) {
  cpp_api::require_output(RegionDilation,"Dilation1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_dilation1(Region.NativeHandle(),StructElement.NativeHandle(),&raw,Iterations.ControlInput()),"Dilation1");
  detail::MvObjectAccess::Commit(RegionDilation,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Dilation1(const MvRegion& StructElement, std::int64_t Iterations) const {MvRegion result;mv::Dilation1(*this,StructElement,&result,MvTuple(Iterations));return result;}
void Erosion1(const MvObject& Region, const MvObject& StructElement, MvObject* RegionErosion, const MvTuple& Iterations) {
  cpp_api::require_output(RegionErosion,"Erosion1");::MvObject raw=nullptr;
  cpp_api::check_status(mv_erosion1(Region.NativeHandle(),StructElement.NativeHandle(),&raw,Iterations.ControlInput()),"Erosion1");
  detail::MvObjectAccess::Commit(RegionErosion,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Erosion1(const MvRegion& StructElement, std::int64_t Iterations) const {MvRegion result;mv::Erosion1(*this,StructElement,&result,MvTuple(Iterations));return result;}
void Opening(const MvObject& Region, const MvObject& StructElement, MvObject* RegionOpening) {
  cpp_api::require_output(RegionOpening,"Opening");::MvObject raw=nullptr;
  cpp_api::check_status(mv_opening(Region.NativeHandle(),StructElement.NativeHandle(),&raw),"Opening");
  detail::MvObjectAccess::Commit(RegionOpening,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Opening(const MvRegion& StructElement) const {MvRegion result;mv::Opening(*this,StructElement,&result);return result;}
void Closing(const MvObject& Region, const MvObject& StructElement, MvObject* RegionClosing) {
  cpp_api::require_output(RegionClosing,"Closing");::MvObject raw=nullptr;
  cpp_api::check_status(mv_closing(Region.NativeHandle(),StructElement.NativeHandle(),&raw),"Closing");
  detail::MvObjectAccess::Commit(RegionClosing,detail::MvObjectAccess::Adopt(raw));
}
MvRegion MvRegion::Closing(const MvRegion& StructElement) const {MvRegion result;mv::Closing(*this,StructElement,&result);return result;}
void TopHat(const MvObject& Region,const MvObject& StructElement,MvObject* RegionTopHat) {
  cpp_api::require_output(RegionTopHat,"TopHat");::MvObject raw=nullptr;
  cpp_api::check_status(mv_top_hat(Region.NativeHandle(),StructElement.NativeHandle(),&raw),"TopHat");
  detail::MvObjectAccess::Commit(RegionTopHat,detail::MvObjectAccess::Adopt(raw));
}

void BottomHat(const MvObject& Region,const MvObject& StructElement,MvObject* RegionBottomHat) {
  cpp_api::require_output(RegionBottomHat,"BottomHat");::MvObject raw=nullptr;
  cpp_api::check_status(mv_bottom_hat(Region.NativeHandle(),StructElement.NativeHandle(),&raw),"BottomHat");
  detail::MvObjectAccess::Commit(RegionBottomHat,detail::MvObjectAccess::Adopt(raw));
}

MvRegion MvRegion::TopHat(const MvRegion& StructElement) const {MvRegion result;mv::TopHat(*this,StructElement,&result);return result;}
MvRegion MvRegion::BottomHat(const MvRegion& StructElement) const {MvRegion result;mv::BottomHat(*this,StructElement,&result);return result;}
}
