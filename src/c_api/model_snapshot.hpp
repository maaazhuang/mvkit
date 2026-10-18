#pragma once

#include <ncc_matching/model.hpp>
#include <shape_matching/model.hpp>
#include <shape_matching/operators.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

namespace mv::c_api {
// 快照只保存经原编解码器验证的不可变状态。模型句柄、序列化项各自拥有生命周期；
// 句柄恢复始终复制状态，不共享清除标志或可变参数。文件仍逐次读取并比较完整字节。
struct ModelSnapshot {
  std::string bytes;
  bool canonical=false;
  std::variant<shape_matching::ShapeModel,ncc_matching::Model> model;
  template<class Model> ModelSnapshot(std::string text,Model value)
      :bytes(std::move(text)),model(std::move(value)) {}
};
inline shape_matching::ShapeModel clone_model(const shape_matching::ShapeModel& model) {
  auto result=model;
  // 搜索工作缓存不属于持久化状态，快照不延长已有搜索索引的生命周期。
  result.set_model_cache_enabled(model.model_cache_enabled());return result;
}
inline ncc_matching::Model clone_model(const ncc_matching::Model& model) {return ncc_matching::clone(model);}

// 单项缓存限制为 4 MiB 文本及 16 MiB 准备数组；大模型继续使用原编解码路径。
inline bool snapshot_budget(const shape_matching::ShapeModel& model) {
  std::size_t bytes=model.refinement_points().size()*sizeof(shape_matching::ShapeModelPoint);
  for(const auto& level:model.levels()) {
    bytes+=level.points.size()*sizeof(shape_matching::ShapeModelPoint);
    for(std::size_t i=0;i<level.contours.contour_count();++i)
      bytes+=level.contours.contour(i).size()*(sizeof(core::Point2d)+3*sizeof(double));
  }
  return bytes<=16u*1024u*1024u;
}
inline bool snapshot_budget(const ncc_matching::Model& model) {
  std::size_t bytes=0;
  const auto patch_bytes=[&](const ncc_matching::Patch& patch) {
    bytes+=patch.samples.size()*sizeof(ncc_matching::Sample)+
      (patch.weights.size()+patch.relative_rows.size()+patch.relative_columns.size())*sizeof(double)+
      patch.runs.size()*sizeof(ncc_matching::SampleRun)+patch.weight_runs.size()*sizeof(ncc_matching::WeightRun);
  };
  patch_bytes(model.original);
  for(const auto& level:model.levels) {
    bytes+=level.gray.size()*sizeof(double)+level.mask.size();
    for(const auto& patch:level.patches)patch_bytes(patch);
  }
  return bytes<=16u*1024u*1024u;
}
template<class Model> std::string encode_model(const Model& model) {
  if constexpr(std::is_same_v<Model,shape_matching::ShapeModel>)return shape_matching::serialize_model(model);
  else return ncc_matching::serialize(model);
}
template<class Opaque> std::shared_ptr<const ModelSnapshot> encoded_snapshot(Opaque& handle,std::string& uncached) {
  std::lock_guard lock(handle.snapshot_mutex);
  if(handle.snapshot && handle.snapshot->canonical)return handle.snapshot;
  auto bytes=encode_model(handle.value);
  if(bytes.size()>4u*1024u*1024u || !snapshot_budget(handle.value)) {uncached=std::move(bytes);return {};}
  auto result=std::make_shared<ModelSnapshot>(std::move(bytes),clone_model(handle.value));
  result->canonical=true;
  handle.snapshot=result;
  return result;
}
template<class Model,class Decode>
Model read_snapshot(std::string bytes,Decode decode,std::shared_ptr<const ModelSnapshot>& owner) {
  // 每线程、每种模型仅保留最后一个弱引用；最后一个使用者释放后不保留文件或模型。
  static thread_local std::weak_ptr<const ModelSnapshot> previous;
  if(auto cached=previous.lock();cached && cached->bytes==bytes) {
    auto result=clone_model(std::get<Model>(cached->model));owner=std::move(cached);return result;
  }
  auto result=decode(bytes);
  if(bytes.size()<=4u*1024u*1024u && snapshot_budget(result)) {
    auto snapshot=std::make_shared<ModelSnapshot>(std::move(bytes),clone_model(result));
    previous=snapshot;owner=std::move(snapshot);
  }
  return result;
}
} // namespace mv::c_api
