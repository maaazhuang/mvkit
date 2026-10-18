#pragma once

#include <runtime/memory/overwrite_buffer.hpp>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory_resource>
#include <new>
#include <unordered_map>

namespace mv::ncc_matching {

// 原四 lane 样本组按索引递增；末尾不足一组只在存储中补零，归约仍走标量尾部。
struct RefinementGradientGroup {
  std::size_t index;
  std::array<double,12> values; // 梯度布局：gy[4]、gx[4]、g_angle[4]。
};

// 拟合首次导数的可选查询局部存储，此头不含 ISA 专用代码。矩可用 BilinearMoments，
// 加权 dot/wg 重放由独立编译 AVX2 内核提供。模型共享前须证明有序采样坐标、尺寸、掩码和原点一致。
// 缓存销毁前像素不可变，数据地址须存活且不复用。键包含裁剪视图首像素的全局坐标；
// 全局位姿相同不保证不同裁剪视图的采样或浮点相减一致。
template<class Moments>
class RefinementCache {
  static constexpr std::size_t byte_budget=32*1024*1024;
  // 计累计分配而非存活字节；arena 可保留已释放桶，缓存插入仍不得超过 32 MiB。
  class BoundedResource final : public std::pmr::memory_resource {
    std::pmr::memory_resource* upstream_;
    std::size_t used_=0;
    void* do_allocate(std::size_t bytes,std::size_t alignment) override {
      if(bytes>byte_budget-used_)throw std::bad_alloc();
      auto* result=upstream_->allocate(bytes,alignment);used_+=bytes;return result;
    }
    void do_deallocate(void* pointer,std::size_t bytes,std::size_t alignment) override {
      upstream_->deallocate(pointer,bytes,alignment);
    }
    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override {
      return this==&other;
    }
  public:
    explicit BoundedResource(std::pmr::memory_resource* memory):upstream_(memory) {}
    std::size_t used() const noexcept {return used_;}
  };
public:
  struct Key {
    const double* pixels=nullptr;
    std::size_t width=0,height=0;
    std::int64_t origin_row=0,origin_column=0;
    double row=0,column=0,angle=0;
    bool rotate=false;
    bool operator==(const Key&) const=default;
  };
  struct Record {
    runtime::OverwriteBuffer<double> values;
    runtime::OverwriteBuffer<RefinementGradientGroup> gradient_groups;
    Moments unweighted{};
    std::size_t count=0;
    bool ready=false,compressed=false;
    Record(std::size_t sample_count,std::pmr::memory_resource* memory)
      :values(4*sample_count,memory),gradient_groups(0,memory),count(sample_count) {}
    Record(std::size_t sample_count,std::size_t groups,std::pmr::memory_resource* memory)
      :values(sample_count,memory),gradient_groups(groups,memory),count(sample_count),compressed(true) {}
    // q 使用原内核同一首样本偏移中心化；梯度按 gy[N]/gx[N]/g_angle[N] 排列，
    // rotate=false 时角度梯度为零，这些数组不含模板权重。
    double* samples() noexcept {return values.data();}
    const double* samples() const noexcept {return values.data();}
    double* gradients() noexcept {return compressed?nullptr:values.data()+count;}
    const double* gradients() const noexcept {return compressed?nullptr:values.data()+count;}
    RefinementGradientGroup* groups() noexcept {return gradient_groups.data();}
    const RefinementGradientGroup* groups() const noexcept {return gradient_groups.data();}
    std::size_t group_count() const noexcept {return gradient_groups.size();}
  };
private:
  struct Hash {
    std::size_t operator()(const Key& key) const noexcept {
      std::size_t result=std::hash<const double*>{}(key.pixels);
      const auto mix=[&](std::size_t value){result^=value+std::size_t(0x9e3779b9)+(result<<6)+(result>>2);};
      mix(std::hash<std::size_t>{}(key.width));mix(std::hash<std::size_t>{}(key.height));
      mix(std::hash<std::int64_t>{}(key.origin_row));mix(std::hash<std::int64_t>{}(key.origin_column));
      mix(std::hash<double>{}(key.row));mix(std::hash<double>{}(key.column));mix(std::hash<double>{}(key.angle));
      mix(std::size_t(key.rotate));return result;
    }
  };
  BoundedResource resource_;
  std::pmr::unordered_map<Key,Record,Hash> records_;
public:
  explicit RefinementCache(std::pmr::memory_resource* memory):resource_(memory),records_(&resource_) {}
  RefinementCache(const RefinementCache&)=delete;
  RefinementCache& operator=(const RefinementCache&)=delete;
  bool can_store(std::size_t sample_count) const noexcept {
    // 为最坏稠密载荷预留余量；缓存满后未命中使用普通采样，不生成无法保留的记录工作区再扫描。
    return sample_count && records_.size()<4096 && sample_count<=byte_budget/(4*sizeof(double)) &&
        resource_.used()<=byte_budget-4*sample_count*sizeof(double);
  }
  const Record* find(const Key& key,std::size_t sample_count) const {
    const auto found=records_.find(key);
    return found!=records_.end() && found->second.ready && found->second.count==sample_count?&found->second:nullptr;
  }
  // 采样前预留可选输出，分配失败仅禁用优化并继续普通采样。
  // 边界检查失败留下 unready 记录，后续相同键可重试，不能作为有效记录重放。
  Record* reserve(const Key& key,std::size_t sample_count) {
    const auto found=records_.find(key);
    if(found!=records_.end())return found->second.count==sample_count && !found->second.compressed?&found->second:nullptr;
    if(!sample_count || sample_count>byte_budget/(4*sizeof(double)) || records_.size()>=4096)return nullptr;
    try {
      return &records_.try_emplace(key,sample_count,&resource_).first->second;
    }catch(const std::bad_alloc&) {return nullptr;}
  }
  // 先采到复用工作区，再存精确 q 和稠密梯度或非零四 lane 组；NaN/Inf 视为非零并保留。
  // 压缩反增载荷时稠密纹理用原布局；仅改变存储，不近似无权统计或样本数。
  const Record* store(const Key& key,const double* samples,const double* gradients,
      std::size_t sample_count,const Moments& moments) {
    if(!samples || !gradients || !sample_count || sample_count>byte_budget/sizeof(double))return nullptr;
    if(const auto found=records_.find(key);found!=records_.end()) {
      auto& record=found->second;
      if(record.count!=sample_count)return nullptr;
      if(record.ready)return &record;
      if(record.compressed)return nullptr;
      std::memcpy(record.samples(),samples,sample_count*sizeof(double));
      std::memcpy(record.gradients(),gradients,3*sample_count*sizeof(double));
      finish(record,moments);return &record;
    }
    if(records_.size()>=4096)return nullptr;
    const auto nonzero=[&](std::size_t index) {
      const auto lanes=std::min(std::size_t(4),sample_count-index);
      for(std::size_t axis=0;axis<3;++axis)for(std::size_t lane=0;lane<lanes;++lane)
        if(gradients[axis*sample_count+index+lane]!=0)return true;
      return false;
    };
    std::size_t groups=0;
    for(std::size_t index=0;index<sample_count;index+=4)groups+=nonzero(index);
    const bool compressed=sample_count*sizeof(double)+groups*sizeof(RefinementGradientGroup)<4*sample_count*sizeof(double);
    if(!compressed && sample_count>byte_budget/(4*sizeof(double)))return nullptr;
    try {
      Record* record=compressed?&records_.try_emplace(key,sample_count,groups,&resource_).first->second
          :&records_.try_emplace(key,sample_count,&resource_).first->second;
      std::memcpy(record->samples(),samples,sample_count*sizeof(double));
      if(compressed) {
        std::size_t output=0;
        for(std::size_t index=0;index<sample_count;index+=4)if(nonzero(index)) {
          auto& group=record->groups()[output++];group.index=index;group.values.fill(0);
          const auto lanes=std::min(std::size_t(4),sample_count-index);
          for(std::size_t axis=0;axis<3;++axis)for(std::size_t lane=0;lane<lanes;++lane)
            group.values[4*axis+lane]=gradients[axis*sample_count+index+lane];
        }
      }else std::memcpy(record->gradients(),gradients,3*sample_count*sizeof(double));
      finish(*record,moments);return record;
    }catch(const std::bad_alloc&) {return nullptr;}
  }
  static void finish(Record& record,const Moments& moments) {
    record.unweighted=moments;
    record.unweighted.dot=0;
    for(auto& value:record.unweighted.wg)value=0;
    record.ready=true;
  }
  std::size_t allocation_bytes() const noexcept {return resource_.used();}
  std::size_t size() const noexcept {return records_.size();}
  std::size_t dense_gradient_groups() const noexcept {
    std::size_t count=0;for(const auto& item:records_)if(item.second.ready)count+=(item.second.count+3)/4;return count;
  }
  std::size_t stored_gradient_groups() const noexcept {
    std::size_t count=0;
    for(const auto& item:records_)if(item.second.ready)
      count+=item.second.compressed?item.second.group_count():(item.second.count+3)/4;
    return count;
  }
  std::size_t compressed_records() const noexcept {
    std::size_t count=0;for(const auto& item:records_)count+=item.second.ready && item.second.compressed;return count;
  }
};

} // namespace mv::ncc_matching
