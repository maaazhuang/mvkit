#include <calibration/hand_eye/residual.hpp>
#include <array>
#include <cmath>
#include <span>
#include <vector>
#include <calibration/geometry3d.hpp>
#include <calibration/solver/manifold.hpp>
namespace mv::calibration { namespace {
Pose3d from_values(std::span<const double>v){return {{v[0],v[1],v[2]},normalize_quaternion({v[3],v[4],v[5],v[6]})};}
std::array<double,6> log_pose(const Pose3d&e){auto q=normalize_quaternion(e.rotation);double s=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z),scale=s<1e-12?2.0:2.0*std::atan2(s,q.w)/s;return {e.translation.x,e.translation.y,e.translation.z,q.x*scale,q.y*scale,q.z*scale};}
std::array<double,6> residual(bool moving,const Pose3d&a,const Pose3d&b,const Pose3d&t,const Pose3d&c){Pose3d err;if(moving){auto pred=compose_pose(t,compose_pose(a,c));err=compose_pose(inverse_pose(b),pred);}else{auto pred=compose_pose(a,compose_pose(t,b));err=compose_pose(inverse_pose(c),pred);}return log_pose(err);}
class Fn final:public solver::ResidualFunction{public:Fn(bool m,solver::ParameterBlockId a,solver::ParameterBlockId b,Pose3d t,Pose3d c):moving_(m),ids_{a,b},tool_(t),cam_(c){}std::size_t residual_size()const noexcept override{return 6;}std::span<const solver::ParameterBlockId>parameter_blocks()const noexcept override{return ids_;}core::Status evaluate(std::span<const solver::ParameterBlockView>p,std::span<double>r,std::span<solver::JacobianView>j)const override{if(p.size()!=2||r.size()!=6)return core::Status::InvalidArgument;auto a=from_values(p[0].values),b=from_values(p[1].values);auto base=residual(moving_,a,b,tool_,cam_);for(int k=0;k<6;++k)r[k]=base[k];if(!j.empty()){if(j.size()!=2)return core::Status::InvalidArgument;for(int block=0;block<2;++block){for(int c=0;c<6;++c){solver::ParameterBlock tmp;tmp.id=p[block].id;tmp.values.assign(p[block].values.begin(),p[block].values.end());tmp.tangent_size=6;tmp.manifold=solver::ManifoldKind::PoseSE3;std::array<double,6>d{};d[c]=1e-7;if(solver::apply_local_delta(tmp,d)!=core::Status::Ok)return core::Status::InvalidState;auto aa=block==0?from_values(tmp.values):a,bb=block==1?from_values(tmp.values):b;auto rp=residual(moving_,aa,bb,tool_,cam_);for(int rr=0;rr<6;++rr)j[block](rr,c)=(rp[rr]-base[rr])/1e-7;}}}return core::Status::Ok;}private:bool moving_;std::array<solver::ParameterBlockId,2>ids_;Pose3d tool_,cam_;};
}
std::unique_ptr<solver::ResidualFunction> make_hand_eye_residual(bool moving,solver::ParameterBlockId first,solver::ParameterBlockId second,Pose3d t,Pose3d c){return std::make_unique<Fn>(moving,first,second,t,c);}
}
