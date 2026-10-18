#include <mvkit/cpp/mv_ncc_matching.hpp>
#include <mvkit/c/mv_ncc_matching.h>
#include "api_bridge.hpp"
#include <utility>
#include <cstdint>
#include <atomic>
#include <limits>
#include <vector>
#include <memory>
namespace mv {
void CreateNccModel(const MvObject& image,const MvTuple& levels,const MvTuple& start,
    const MvTuple& extent,const MvTuple& step,const MvTuple& metric,MvTuple* id) {
  cpp_api::TupleOutputs<1> output("CreateNccModel",{id});
  cpp_api::check_status(mv_create_ncc_model(image.NativeHandle(),levels.ControlInput(),
      start.ControlInput(),extent.ControlInput(),step.ControlInput(),metric.ControlInput(),output.Out(0)),"CreateNccModel");
  output.Commit();
}
void GetNccModelParams(const MvTuple& id,MvTuple* levels,MvTuple* start,MvTuple* extent,MvTuple* step,MvTuple* metric) {
  cpp_api::TupleOutputs<5> output("GetNccModelParams",{levels,start,extent,step,metric});
  cpp_api::check_status(mv_get_ncc_model_params(id.ControlInput(),output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4)),"GetNccModelParams");
  output.Commit();
}
void ReadNccModel(const MvTuple& filename,MvTuple* id) {
  cpp_api::TupleOutputs<1> output("ReadNccModel",{id});
  cpp_api::check_status(mv_read_ncc_model(filename.ControlInput(),output.Out(0)),"ReadNccModel");output.Commit();
}
void WriteNccModel(const MvTuple& id,const MvTuple& filename) {
  cpp_api::check_status(mv_write_ncc_model(id.ControlInput(),filename.ControlInput()),"WriteNccModel");
}
MvNCCModel::MvNCCModel() noexcept=default;
MvNCCModel::MvNCCModel(const MvNCCModel&)=default;
MvNCCModel::MvNCCModel(MvNCCModel&&) noexcept=default;
MvNCCModel& MvNCCModel::operator=(const MvNCCModel&)=default;
MvNCCModel& MvNCCModel::operator=(MvNCCModel&&) noexcept=default;
MvNCCModel::~MvNCCModel() noexcept=default;
MvNCCModel::MvNCCModel(const MvHandle& handle):MvHandle(handle) {
  if(!IsInitialized()||Type()!=MvObjectType::NccModel)throw MvException(MvStatus::TypeMismatch,MvErrorCode::InvalidType,
      "MvNCCModel","ModelID",-1,"control handle is not an NCC model");
}
void MvNCCModel::CreateNccModel(const MvImage& image,const MvTuple& levels,double start,double extent,const MvTuple& step,const MvString& metric) {
  MvTuple id;mv::CreateNccModel(image,levels,start,extent,step,metric,&id);MvHandle::operator=(id.H());
}
void MvNCCModel::CreateNccModel(const MvImage& image,const MvTuple& levels,double start,double extent,const MvTuple& step,const char* metric) {
  CreateNccModel(image,levels,start,extent,step,MvString(metric));
}
void MvNCCModel::CreateNccModel(const MvImage& image,std::int64_t levels,double start,double extent,double step,const MvString& metric) {
  CreateNccModel(image,MvTuple(levels),start,extent,MvTuple(step),metric);
}
void MvNCCModel::CreateNccModel(const MvImage& image,std::int64_t levels,double start,double extent,double step,const char* metric) {
  CreateNccModel(image,MvTuple(levels),start,extent,MvTuple(step),MvString(metric));
}
std::int64_t MvNCCModel::GetNccModelParams(double* start,double* extent,double* step,MvString* metric) const {
  cpp_api::require_output(start,"GetNccModelParams");cpp_api::require_output(extent,"GetNccModelParams");
  cpp_api::require_output(step,"GetNccModelParams");cpp_api::require_output(metric,"GetNccModelParams");
  const std::uintptr_t addresses[]={reinterpret_cast<std::uintptr_t>(start),reinterpret_cast<std::uintptr_t>(extent),reinterpret_cast<std::uintptr_t>(step),reinterpret_cast<std::uintptr_t>(metric)};
  const std::size_t sizes[]={sizeof(double),sizeof(double),sizeof(double),sizeof(MvString)};
  for(std::size_t i=0;i<4;++i)for(std::size_t j=0;j<i;++j)
    if(addresses[i]<=addresses[j] ? addresses[j]-addresses[i]<sizes[i] : addresses[i]-addresses[j]<sizes[j])
      throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,"GetNccModelParams","Output",-1,"output ranges must be distinct");
  MvTuple levels,a,b,c,d;mv::GetNccModelParams(MvTuple(*this),&levels,&a,&b,&c,&d);
  const auto count=levels.I();const auto av=a.D(),bv=b.D(),cv=c.D();auto dv=d.S();
  *metric=std::move(dv);*start=av;*extent=bv;*step=cv;return count;
}
void MvNCCModel::ReadNccModel(const MvString& filename) {MvTuple id;mv::ReadNccModel(filename,&id);MvHandle::operator=(id.H());}
void MvNCCModel::ReadNccModel(const char* filename) {ReadNccModel(MvString(filename));}
void MvNCCModel::WriteNccModel(const MvString& filename) const {mv::WriteNccModel(MvTuple(*this),filename);}
void MvNCCModel::WriteNccModel(const char* filename) const {WriteNccModel(MvString(filename));}
MvNCCModel MvImage::CreateNccModel(const MvTuple& levels,double start,double extent,const MvTuple& step,const MvString& metric) const {
  MvNCCModel result;result.CreateNccModel(*this,levels,start,extent,step,metric);return result;
}
MvNCCModel MvImage::CreateNccModel(const MvTuple& levels,double start,double extent,const MvTuple& step,const char* metric) const {
  return CreateNccModel(levels,start,extent,step,MvString(metric));
}
MvNCCModel MvImage::CreateNccModel(std::int64_t levels,double start,double extent,double step,const MvString& metric) const {
  return CreateNccModel(MvTuple(levels),start,extent,MvTuple(step),metric);
}
MvNCCModel MvImage::CreateNccModel(std::int64_t levels,double start,double extent,double step,const char* metric) const {
  return CreateNccModel(MvTuple(levels),start,extent,MvTuple(step),MvString(metric));
}
}

namespace mv {
void FindNccModel(const MvObject& image,const MvTuple& id,const MvTuple& start,const MvTuple& extent,
    const MvTuple& score,const MvTuple& matches,const MvTuple& overlap,const MvTuple& subpixel,const MvTuple& levels,
    MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) {
  cpp_api::TupleOutputs<4> output("FindNccModel",{row,column,angle,quality});
  cpp_api::check_status(mv_find_ncc_model(image.NativeHandle(),id.ControlInput(),start.ControlInput(),extent.ControlInput(),
      score.ControlInput(),matches.ControlInput(),overlap.ControlInput(),subpixel.ControlInput(),levels.ControlInput(),
      output.Out(0),output.Out(1),output.Out(2),output.Out(3)),"FindNccModel");output.Commit();
}
void FindNccModels(const MvObject& image,const MvTuple& id,const MvTuple& start,const MvTuple& extent,
    const MvTuple& score,const MvTuple& matches,const MvTuple& overlap,const MvTuple& subpixel,const MvTuple& levels,
    MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) {
  cpp_api::TupleOutputs<5> output("FindNccModels",{row,column,angle,quality,model});
  cpp_api::check_status(mv_find_ncc_models(image.NativeHandle(),id.ControlInput(),start.ControlInput(),extent.ControlInput(),
      score.ControlInput(),matches.ControlInput(),overlap.ControlInput(),subpixel.ControlInput(),levels.ControlInput(),
      output.Out(0),output.Out(1),output.Out(2),output.Out(3),output.Out(4)),"FindNccModels");output.Commit();
}
struct MvNCCModelArrayData {std::atomic<std::size_t> refs{1};std::vector<MvNCCModel> values;};
MvNCCModelArray::MvNCCModelArray() noexcept=default;
MvNCCModelArray::MvNCCModelArray(const MvNCCModel* models,std::int64_t length) {
  if(length<0 || std::uint64_t(length)>std::uint64_t(std::numeric_limits<std::ptrdiff_t>::max())/sizeof(MvNCCModel) || (length && !models))throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,"MvNCCModelArray","models",-1,"nonnegative length and valid model array required");
  if(length){auto next=std::make_unique<MvNCCModelArrayData>();next->values.assign(models,models+length);data_=next.release();}
}
MvNCCModelArray::MvNCCModelArray(const MvNCCModelArray& other):data_(other.data_){if(data_)data_->refs.fetch_add(1,std::memory_order_relaxed);}
MvNCCModelArray::MvNCCModelArray(MvNCCModelArray&& other) noexcept:data_(std::exchange(other.data_,nullptr)){}
MvNCCModelArray& MvNCCModelArray::operator=(const MvNCCModelArray& other){if(this!=&other){auto next=other.data_;if(next)next->refs.fetch_add(1,std::memory_order_relaxed);Clear();data_=next;}return *this;}
MvNCCModelArray& MvNCCModelArray::operator=(MvNCCModelArray&& other) noexcept {if(this!=&other){Clear();data_=std::exchange(other.data_,nullptr);}return *this;}
MvNCCModelArray::~MvNCCModelArray() noexcept{Clear();}
void MvNCCModelArray::Clear() noexcept{auto old=std::exchange(data_,nullptr);if(old&&old->refs.fetch_sub(1,std::memory_order_acq_rel)==1)delete old;}
const MvNCCModel* MvNCCModelArray::Tools() const noexcept{return data_?data_->values.data():nullptr;}
std::int64_t MvNCCModelArray::Length() const noexcept{return data_?std::int64_t(data_->values.size()):0;}
void MvNCCModelArray::SetFromTuple(const MvTuple& handles) {
  if(!handles.Length()){Clear();return;}auto next=std::make_unique<MvNCCModelArrayData>();next->values.reserve(handles.Length());
  for(std::size_t i=0;i<handles.Length();++i)next->values.emplace_back(handles.H(i));Clear();data_=next.release();
}
MvTuple MvNCCModelArray::ConvertToTuple() const {MvTuple result;if(data_)for(const auto& model:data_->values)result.Append(model);return result;}
void MvNCCModel::FindNccModel(const MvImage& image,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,const MvTuple& levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  mv::FindNccModel(image,MvTuple(*this),start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality);
}
void MvNCCModel::FindNccModel(const MvImage& image,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  FindNccModel(image,start,extent,score,matches,overlap,subpixel,MvTuple(levels),row,column,angle,quality);
}
void MvNCCModel::FindNccModel(const MvImage& image,double start,double extent,double score,std::int64_t matches,double overlap,const char* subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  FindNccModel(image,start,extent,score,matches,overlap,MvString(subpixel),MvTuple(levels),row,column,angle,quality);
}
void MvNCCModel::FindNccModels(const MvImage& image,const MvNCCModelArray& ids,const MvTuple& start,const MvTuple& extent,const MvTuple& score,const MvTuple& matches,const MvTuple& overlap,const MvTuple& subpixel,const MvTuple& levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) {
  mv::FindNccModels(image,ids.ConvertToTuple(),start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality,model);
}
void MvNCCModel::FindNccModels(const MvImage& image,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) const {
  mv::FindNccModels(image,MvTuple(*this),start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality,model);
}
void MvNCCModel::FindNccModels(const MvImage& image,double start,double extent,double score,std::int64_t matches,double overlap,const char* subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) const {
  FindNccModels(image,start,extent,score,matches,overlap,MvString(subpixel),levels,row,column,angle,quality,model);
}
void MvImage::FindNccModel(const MvNCCModel& id,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,const MvTuple& levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  id.FindNccModel(*this,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality);
}
void MvImage::FindNccModel(const MvNCCModel& id,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  id.FindNccModel(*this,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality);
}
void MvImage::FindNccModel(const MvNCCModel& id,double start,double extent,double score,std::int64_t matches,double overlap,const char* subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality) const {
  id.FindNccModel(*this,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality);
}
void MvImage::FindNccModels(const MvNCCModelArray& ids,const MvTuple& start,const MvTuple& extent,const MvTuple& score,const MvTuple& matches,const MvTuple& overlap,const MvTuple& subpixel,const MvTuple& levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) const {
  MvNCCModel::FindNccModels(*this,ids,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality,model);
}
void MvImage::FindNccModels(const MvNCCModel& id,double start,double extent,double score,std::int64_t matches,double overlap,const MvString& subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) const {
  id.FindNccModels(*this,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality,model);
}
void MvImage::FindNccModels(const MvNCCModel& id,double start,double extent,double score,std::int64_t matches,double overlap,const char* subpixel,std::int64_t levels,MvTuple* row,MvTuple* column,MvTuple* angle,MvTuple* quality,MvTuple* model) const {
  id.FindNccModels(*this,start,extent,score,matches,overlap,subpixel,levels,row,column,angle,quality,model);
}
}
