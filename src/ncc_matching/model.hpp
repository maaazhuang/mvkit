#pragma once

#include <core/image.hpp>
#include <matching/multi.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <string_view>
#include <string>

namespace mv::ncc_matching {

struct CreateOptions {
  int levels=0;
  bool ignore_global_polarity=false;
  double angle_start=0,angle_extent=0,angle_step=0;
};
struct FindOptions {
  double angle_start=0,angle_extent=0,min_score=.8,max_overlap=.5;
  std::size_t num_matches=1,max_candidates=10000;
  bool subpixel=true;
  // HALCON 金字塔控制为 1-based；highest=0 使用全部训练层。
  int highest_level=0,lowest_level=1;
};
struct Match { double row=0,column=0,angle=0,score=0; };
struct Sample { std::int32_t row=0,column=0;double weight=0; };
struct SampleRun { std::int32_t row=0,begin=0,end=0; };
struct WeightRun {std::int32_t row,begin,end;double weight;};
struct Patch {
  double angle=0,origin_row=0,origin_column=0;
  std::int32_t row_min=0,row_max=0,column_min=0,column_max=0;
  bool bounds_prepared=false;
  std::pmr::vector<Sample> samples{runtime::persist_resource()};
  // SIMD 权重保留原样本顺序及 Double 值。
  std::pmr::vector<double> weights{runtime::persist_resource()};
  std::pmr::vector<double> relative_rows{runtime::persist_resource()},relative_columns{runtime::persist_resource()};
  std::pmr::vector<SampleRun> runs{runtime::persist_resource()};double weight_sum=0;
  std::pmr::vector<WeightRun> weight_runs{runtime::persist_resource()};
  explicit Patch(std::pmr::memory_resource* memory=runtime::persist_resource()):samples(memory),weights(memory),relative_rows(memory),relative_columns(memory),runs(memory),weight_runs(memory) {}
  Patch(const Patch&)=delete;Patch& operator=(const Patch&)=delete;
  Patch(Patch&&) noexcept=default;Patch& operator=(Patch&&) noexcept=default;
};
struct Level {
  std::size_t width=0,height=0;
  double origin_row=0,origin_column=0;
  std::pmr::vector<double> gray{runtime::persist_resource()};
  std::pmr::vector<std::uint8_t> mask{runtime::persist_resource()};
  std::pmr::vector<Patch> patches{runtime::persist_resource()};
  Level()=default;
  Level(const Level&)=delete;Level& operator=(const Level&)=delete;
  Level(Level&&) noexcept=default;Level& operator=(Level&&) noexcept=default;
};
struct Model {
  CreateOptions options;
  std::size_t sample_count=0;
  double origin_row=0,origin_column=0;
  Patch original;
  std::pmr::vector<Level> levels{runtime::persist_resource()};
  Model()=default;
  Model(const Model&)=delete;Model& operator=(const Model&)=delete;
  Model(Model&&) noexcept=default;Model& operator=(Model&&) noexcept=default;
};
using Matches=std::pmr::vector<Match>;

Model create(const core::Image&,const CreateOptions&);
// 持久化从验证后的精确 Double 快照重建准备数据。
Model rebuild(Level,const CreateOptions&,double origin_row,double origin_column);
// 独立句柄复制已验证的准备数据，避免从文本重复训练；所有数组仍使用持久分配器。
Model clone(const Model&);
Patch make_patch(const Level&,double angle,std::pmr::memory_resource*);
Matches find(const core::Image&,const Model&,const FindOptions&);
std::vector<matching::Match<Match>> find_models(const core::Image&,std::span<const Model* const>,
    std::span<const FindOptions>,matching::Selection,bool per_model_quota_before_overlap=false);
// 缓慢且独立于金字塔的公开诊断原语，供数学测试使用。
double normalized_correlation(std::span<const double>,std::span<const double>,bool absolute=false);
void write(const Model&,std::string_view path);
Model read(std::string_view path);
// 保存训练状态，恢复时重建可重复生成的金字塔与缓存。
std::string serialize(const Model& model);
Model deserialize(std::string_view bytes);

} // namespace mv::ncc_matching
