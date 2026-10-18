#include <mvkit/cpp/mv_geometry.hpp>
#include <mvkit/c/mv_geometry.h>
#include "api_bridge.hpp"
namespace mv {
void HomMat2dIdentity(MvTuple* HomMat2DIdentity) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dIdentity",{{HomMat2DIdentity}});
  cpp_api::check_status(mv_hom_mat2d_identity(outputs.Out(0)),"HomMat2dIdentity");
  outputs.Commit();
}
void HomMat2dTranslate(const MvTuple& HomMat2D,const MvTuple& Tx,const MvTuple& Ty,MvTuple* HomMat2DTranslate) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dTranslate",{{HomMat2DTranslate}});
  cpp_api::check_status(mv_hom_mat2d_translate(HomMat2D.ControlInput(),Tx.ControlInput(),Ty.ControlInput(),outputs.Out(0)),"HomMat2dTranslate");
  outputs.Commit();
}
void HomMat2dRotate(const MvTuple& HomMat2D,const MvTuple& Phi,const MvTuple& Px,const MvTuple& Py,MvTuple* HomMat2DRotate) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dRotate",{{HomMat2DRotate}});
  cpp_api::check_status(mv_hom_mat2d_rotate(HomMat2D.ControlInput(),Phi.ControlInput(),Px.ControlInput(),Py.ControlInput(),outputs.Out(0)),"HomMat2dRotate");
  outputs.Commit();
}
void HomMat2dScale(const MvTuple& HomMat2D,const MvTuple& Sx,const MvTuple& Sy,const MvTuple& Px,const MvTuple& Py,MvTuple* HomMat2DScale) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dScale",{{HomMat2DScale}});
  cpp_api::check_status(mv_hom_mat2d_scale(HomMat2D.ControlInput(),Sx.ControlInput(),Sy.ControlInput(),Px.ControlInput(),Py.ControlInput(),outputs.Out(0)),"HomMat2dScale");
  outputs.Commit();
}
void HomMat2dCompose(const MvTuple& HomMat2DLeft,const MvTuple& HomMat2DRight,MvTuple* HomMat2DCompose) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dCompose",{{HomMat2DCompose}});
  cpp_api::check_status(mv_hom_mat2d_compose(HomMat2DLeft.ControlInput(),HomMat2DRight.ControlInput(),outputs.Out(0)),"HomMat2dCompose");
  outputs.Commit();
}
void HomMat2dInvert(const MvTuple& HomMat2D,MvTuple* HomMat2DInvert) {
  cpp_api::TupleOutputs<1> outputs("HomMat2dInvert",{{HomMat2DInvert}});
  cpp_api::check_status(mv_hom_mat2d_invert(HomMat2D.ControlInput(),outputs.Out(0)),"HomMat2dInvert");
  outputs.Commit();
}
void AffineTransPoint2d(const MvTuple& HomMat2D,const MvTuple& Px,const MvTuple& Py,MvTuple* Qx,MvTuple* Qy) {
  cpp_api::TupleOutputs<2> outputs("AffineTransPoint2d",{{Qx,Qy}});
  cpp_api::check_status(mv_affine_trans_point_2d(HomMat2D.ControlInput(),Px.ControlInput(),Py.ControlInput(),outputs.Out(0),outputs.Out(1)),"AffineTransPoint2d");
  outputs.Commit();
}
void VectorAngleToRigid(const MvTuple& Row1,const MvTuple& Column1,const MvTuple& Angle1,const MvTuple& Row2,const MvTuple& Column2,const MvTuple& Angle2,MvTuple* HomMat2D) {
  cpp_api::TupleOutputs<1> outputs("VectorAngleToRigid",{{HomMat2D}});
  cpp_api::check_status(mv_vector_angle_to_rigid(Row1.ControlInput(),Column1.ControlInput(),Angle1.ControlInput(),Row2.ControlInput(),Column2.ControlInput(),Angle2.ControlInput(),outputs.Out(0)),"VectorAngleToRigid");
  outputs.Commit();
}
void DistancePp(const MvTuple& Row1,const MvTuple& Column1,const MvTuple& Row2,const MvTuple& Column2,MvTuple* Distance) {
  cpp_api::TupleOutputs<1> outputs("DistancePp",{{Distance}});
  cpp_api::check_status(mv_distance_pp(Row1.ControlInput(),Column1.ControlInput(),Row2.ControlInput(),Column2.ControlInput(),outputs.Out(0)),"DistancePp");
  outputs.Commit();
}
void DistancePl(const MvTuple& Row,const MvTuple& Column,const MvTuple& Row1,const MvTuple& Column1,const MvTuple& Row2,const MvTuple& Column2,MvTuple* Distance) {
  cpp_api::TupleOutputs<1> outputs("DistancePl",{{Distance}});
  cpp_api::check_status(mv_distance_pl(Row.ControlInput(),Column.ControlInput(),Row1.ControlInput(),Column1.ControlInput(),Row2.ControlInput(),Column2.ControlInput(),outputs.Out(0)),"DistancePl");
  outputs.Commit();
}
void IntersectionLines(const MvTuple& Line1Row1,const MvTuple& Line1Column1,const MvTuple& Line1Row2,const MvTuple& Line1Column2,const MvTuple& Line2Row1,const MvTuple& Line2Column1,const MvTuple& Line2Row2,const MvTuple& Line2Column2,MvTuple* Row,MvTuple* Column,MvTuple* IsOverlapping) {
  cpp_api::TupleOutputs<3> outputs("IntersectionLines",{{Row,Column,IsOverlapping}});
  cpp_api::check_status(mv_intersection_lines(Line1Row1.ControlInput(),Line1Column1.ControlInput(),Line1Row2.ControlInput(),Line1Column2.ControlInput(),Line2Row1.ControlInput(),Line2Column1.ControlInput(),Line2Row2.ControlInput(),Line2Column2.ControlInput(),outputs.Out(0),outputs.Out(1),outputs.Out(2)),"IntersectionLines");
  outputs.Commit();
}
void AngleLl(const MvTuple& RowA1,const MvTuple& ColumnA1,const MvTuple& RowA2,const MvTuple& ColumnA2,const MvTuple& RowB1,const MvTuple& ColumnB1,const MvTuple& RowB2,const MvTuple& ColumnB2,MvTuple* Angle) {
  cpp_api::TupleOutputs<1> outputs("AngleLl",{{Angle}});
  cpp_api::check_status(mv_angle_ll(RowA1.ControlInput(),ColumnA1.ControlInput(),RowA2.ControlInput(),ColumnA2.ControlInput(),RowB1.ControlInput(),ColumnB1.ControlInput(),RowB2.ControlInput(),ColumnB2.ControlInput(),outputs.Out(0)),"AngleLl");
  outputs.Commit();
}
void PoseToHomMat3d(const MvTuple& Pose,MvTuple* HomMat3D) {
  cpp_api::TupleOutputs<1> outputs("PoseToHomMat3d",{{HomMat3D}});
  cpp_api::check_status(mv_pose_to_hom_mat3d(Pose.ControlInput(),outputs.Out(0)),"PoseToHomMat3d");
  outputs.Commit();
}
void HomMat3dToPose(const MvTuple& HomMat3D,MvTuple* Pose) {
  cpp_api::TupleOutputs<1> outputs("HomMat3dToPose",{{Pose}});
  cpp_api::check_status(mv_hom_mat3d_to_pose(HomMat3D.ControlInput(),outputs.Out(0)),"HomMat3dToPose");
  outputs.Commit();
}
void HomMat3dCompose(const MvTuple& HomMat3DLeft,const MvTuple& HomMat3DRight,MvTuple* HomMat3DCompose) {
  cpp_api::TupleOutputs<1> outputs("HomMat3dCompose",{{HomMat3DCompose}});
  cpp_api::check_status(mv_hom_mat3d_compose(HomMat3DLeft.ControlInput(),HomMat3DRight.ControlInput(),outputs.Out(0)),"HomMat3dCompose");
  outputs.Commit();
}
void HomMat3dInvert(const MvTuple& HomMat3D,MvTuple* HomMat3DInvert) {
  cpp_api::TupleOutputs<1> outputs("HomMat3dInvert",{{HomMat3DInvert}});
  cpp_api::check_status(mv_hom_mat3d_invert(HomMat3D.ControlInput(),outputs.Out(0)),"HomMat3dInvert");
  outputs.Commit();
}
void AffineTransPoint3d(const MvTuple& HomMat3D,const MvTuple& Px,const MvTuple& Py,const MvTuple& Pz,MvTuple* Qx,MvTuple* Qy,MvTuple* Qz) {
  cpp_api::TupleOutputs<3> outputs("AffineTransPoint3d",{{Qx,Qy,Qz}});
  cpp_api::check_status(mv_affine_trans_point_3d(HomMat3D.ControlInput(),Px.ControlInput(),Py.ControlInput(),Pz.ControlInput(),outputs.Out(0),outputs.Out(1),outputs.Out(2)),"AffineTransPoint3d");
  outputs.Commit();
}
MvHomMat2D::MvHomMat2D()=default;
MvHomMat2D::MvHomMat2D(const MvTuple& values):MvTuple(values){}
MvHomMat2D::MvHomMat2D(const MvHomMat2D&)=default;
MvHomMat2D::MvHomMat2D(MvHomMat2D&&) noexcept=default;
MvHomMat2D& MvHomMat2D::operator=(const MvHomMat2D&)=default;
MvHomMat2D& MvHomMat2D::operator=(MvHomMat2D&&) noexcept=default;
MvHomMat2D::~MvHomMat2D() noexcept=default;
void MvHomMat2D::HomMat2dIdentity() {
  mv::HomMat2dIdentity(this);
}
MvHomMat2D MvHomMat2D::HomMat2dTranslate(const MvTuple& Tx, const MvTuple& Ty) const {
  MvTuple result;
  mv::HomMat2dTranslate(*this,Tx,Ty,&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dTranslate(double Tx, double Ty) const {
  MvTuple result;
  mv::HomMat2dTranslate(*this,MvTuple(Tx),MvTuple(Ty),&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dRotate(const MvTuple& Phi, const MvTuple& Px, const MvTuple& Py) const {
  MvTuple result;
  mv::HomMat2dRotate(*this,Phi,Px,Py,&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dRotate(double Phi, double Px, double Py) const {
  MvTuple result;
  mv::HomMat2dRotate(*this,MvTuple(Phi),MvTuple(Px),MvTuple(Py),&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dScale(const MvTuple& Sx, const MvTuple& Sy, const MvTuple& Px, const MvTuple& Py) const {
  MvTuple result;
  mv::HomMat2dScale(*this,Sx,Sy,Px,Py,&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dScale(double Sx, double Sy, double Px, double Py) const {
  MvTuple result;
  mv::HomMat2dScale(*this,MvTuple(Sx),MvTuple(Sy),MvTuple(Px),MvTuple(Py),&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dCompose(const MvHomMat2D& HomMat2DRight) const {
  MvTuple result;
  mv::HomMat2dCompose(*this,HomMat2DRight,&result);
  return MvHomMat2D(result);
}
MvHomMat2D MvHomMat2D::HomMat2dInvert() const {
  MvTuple result;
  mv::HomMat2dInvert(*this,&result);
  return MvHomMat2D(result);
}
MvTuple MvHomMat2D::AffineTransPoint2d(const MvTuple& Px, const MvTuple& Py, MvTuple* Qy) const {
  MvTuple result;
  mv::AffineTransPoint2d(*this,Px,Py,&result,Qy);
  return result;
}
double MvHomMat2D::AffineTransPoint2d(double Px, double Py, double* Qy) const {
  MvTuple result;
  cpp_api::require_output(Qy,"AffineTransPoint2d");MvTuple tuple_Qy;
  mv::AffineTransPoint2d(*this,MvTuple(Px),MvTuple(Py),&result,&tuple_Qy);
  const auto value_Qy=tuple_Qy.D();
  const auto value=result.D();
  *Qy=value_Qy;
  return value;
}
void MvHomMat2D::VectorAngleToRigid(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Angle1, const MvTuple& Row2, const MvTuple& Column2, const MvTuple& Angle2) {
  mv::VectorAngleToRigid(Row1,Column1,Angle1,Row2,Column2,Angle2,this);
}
void MvHomMat2D::VectorAngleToRigid(double Row1, double Column1, double Angle1, double Row2, double Column2, double Angle2) {
  mv::VectorAngleToRigid(MvTuple(Row1),MvTuple(Column1),MvTuple(Angle1),MvTuple(Row2),MvTuple(Column2),MvTuple(Angle2),this);
}
MvHomMat3D::MvHomMat3D()=default;
MvHomMat3D::MvHomMat3D(const MvTuple& values):MvTuple(values){}
MvHomMat3D::MvHomMat3D(const MvHomMat3D&)=default;
MvHomMat3D::MvHomMat3D(MvHomMat3D&&) noexcept=default;
MvHomMat3D& MvHomMat3D::operator=(const MvHomMat3D&)=default;
MvHomMat3D& MvHomMat3D::operator=(MvHomMat3D&&) noexcept=default;
MvHomMat3D::~MvHomMat3D() noexcept=default;
MvPose MvHomMat3D::HomMat3dToPose() const {
  MvTuple result;
  mv::HomMat3dToPose(*this,&result);
  return MvPose(result);
}
MvHomMat3D MvHomMat3D::HomMat3dCompose(const MvHomMat3D& HomMat3DRight) const {
  MvTuple result;
  mv::HomMat3dCompose(*this,HomMat3DRight,&result);
  return MvHomMat3D(result);
}
MvHomMat3D MvHomMat3D::HomMat3dInvert() const {
  MvTuple result;
  mv::HomMat3dInvert(*this,&result);
  return MvHomMat3D(result);
}
MvTuple MvHomMat3D::AffineTransPoint3d(const MvTuple& Px, const MvTuple& Py, const MvTuple& Pz, MvTuple* Qy, MvTuple* Qz) const {
  MvTuple result;
  mv::AffineTransPoint3d(*this,Px,Py,Pz,&result,Qy,Qz);
  return result;
}
double MvHomMat3D::AffineTransPoint3d(double Px, double Py, double Pz, double* Qy, double* Qz) const {
  MvTuple result;
  cpp_api::require_output(Qy,"AffineTransPoint3d");MvTuple tuple_Qy;
  cpp_api::require_output(Qz,"AffineTransPoint3d");MvTuple tuple_Qz;
  mv::AffineTransPoint3d(*this,MvTuple(Px),MvTuple(Py),MvTuple(Pz),&result,&tuple_Qy,&tuple_Qz);
  const auto value_Qy=tuple_Qy.D();
  const auto value_Qz=tuple_Qz.D();
  const auto value=result.D();
  *Qy=value_Qy;
  *Qz=value_Qz;
  return value;
}
MvPose::MvPose()=default;
MvPose::MvPose(const MvTuple& values):MvTuple(values){}
MvPose::MvPose(const MvPose&)=default;
MvPose::MvPose(MvPose&&) noexcept=default;
MvPose& MvPose::operator=(const MvPose&)=default;
MvPose& MvPose::operator=(MvPose&&) noexcept=default;
MvPose::~MvPose() noexcept=default;
MvHomMat3D MvPose::PoseToHomMat3d() const {
  MvTuple result;
  mv::PoseToHomMat3d(*this,&result);
  return MvHomMat3D(result);
}
MvTuple MvMisc::DistancePp(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) {
  MvTuple result;
  mv::DistancePp(Row1,Column1,Row2,Column2,&result);
  return result;
}
double MvMisc::DistancePp(double Row1, double Column1, double Row2, double Column2) {
  MvTuple result;
  mv::DistancePp(MvTuple(Row1),MvTuple(Column1),MvTuple(Row2),MvTuple(Column2),&result);
  const auto value=result.D();
  return value;
}
MvTuple MvMisc::DistancePl(const MvTuple& Row, const MvTuple& Column, const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) {
  MvTuple result;
  mv::DistancePl(Row,Column,Row1,Column1,Row2,Column2,&result);
  return result;
}
double MvMisc::DistancePl(double Row, double Column, double Row1, double Column1, double Row2, double Column2) {
  MvTuple result;
  mv::DistancePl(MvTuple(Row),MvTuple(Column),MvTuple(Row1),MvTuple(Column1),MvTuple(Row2),MvTuple(Column2),&result);
  const auto value=result.D();
  return value;
}
MvTuple MvMisc::AngleLl(const MvTuple& RowA1, const MvTuple& ColumnA1, const MvTuple& RowA2, const MvTuple& ColumnA2, const MvTuple& RowB1, const MvTuple& ColumnB1, const MvTuple& RowB2, const MvTuple& ColumnB2) {
  MvTuple result;
  mv::AngleLl(RowA1,ColumnA1,RowA2,ColumnA2,RowB1,ColumnB1,RowB2,ColumnB2,&result);
  return result;
}
double MvMisc::AngleLl(double RowA1, double ColumnA1, double RowA2, double ColumnA2, double RowB1, double ColumnB1, double RowB2, double ColumnB2) {
  MvTuple result;
  mv::AngleLl(MvTuple(RowA1),MvTuple(ColumnA1),MvTuple(RowA2),MvTuple(ColumnA2),MvTuple(RowB1),MvTuple(ColumnB1),MvTuple(RowB2),MvTuple(ColumnB2),&result);
  const auto value=result.D();
  return value;
}
}
