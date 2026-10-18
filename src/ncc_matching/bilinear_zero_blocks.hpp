#pragma once

#include "model.hpp"
#include <runtime/memory/overwrite_buffer.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <new>
#include <vector>

namespace mv::ncc_matching {

// 查询局部证明存储像素全为精确正零；内核借用时源图像须存活且不变。
// 负零、NaN、Inf 及其他值均视为占用。
class BilinearZeroImage {
  const double* image_=nullptr;
  std::size_t width_=0,height_=0;
  runtime::OverwriteBuffer<std::uint32_t> prefix_;
public:
  BilinearZeroImage(const double* image,std::size_t width,std::size_t height,
      std::pmr::memory_resource* memory):prefix_(0,memory) {
    if(!image || !width || !height || width>std::size_t(std::numeric_limits<int>::max()) ||
        height>std::size_t(std::numeric_limits<int>::max()) || height>32000000/width ||
        height+1>std::numeric_limits<std::size_t>::max()/(width+1) ||
        (height+1)*(width+1)>std::numeric_limits<std::size_t>::max()/sizeof(std::uint32_t))return;
    try {prefix_.reset_for_overwrite((width+1)*(height+1));}catch(const std::bad_alloc&) {return;}
    std::fill_n(prefix_.data(),width+1,0u);
    for(std::size_t row=0;row<height;++row) {
      auto* output=prefix_.data()+(row+1)*(width+1);
      const auto* previous=prefix_.data()+row*(width+1);output[0]=0;
      std::uint32_t occupied=0;
      for(std::size_t column=0;column<width;++column) {
        occupied+=std::bit_cast<std::uint64_t>(image[row*width+column])!=0;
        output[column+1]=previous[column+1]+occupied;
      }
    }
    image_=image;width_=width;height_=height;
  }
  bool valid() const noexcept {return image_!=nullptr;}
  std::size_t allocation_bytes() const noexcept {return prefix_.size()*sizeof(std::uint32_t);}
  const double* image() const noexcept {return image_;}
  std::size_t width() const noexcept {return width_;}
  std::size_t height() const noexcept {return height_;}
  bool rectangle_is_zero(std::size_t row0,std::size_t column0,std::size_t row1,std::size_t column1) const noexcept {
    if(!valid() || row0>row1 || column0>column1 || row1>height_ || column1>width_)return false;
    const auto stride=width_+1;
    // 差值为非负条带计数，精确占用证明无需有符号转换或无符号中间回绕。
    const auto bottom=prefix_.data()[row1*stride+column1]-prefix_.data()[row1*stride+column0];
    const auto top=prefix_.data()[row0*stride+column1]-prefix_.data()[row0*stride+column0];
    return bottom==top;
  }
};

struct BilinearZeroBlock {
  std::size_t begin;
  double row_min,row_max,column_min,column_max;
};

// 从实际有序 SIMD 坐标数组准备，不从栅格块框推断；不可变 Patch 及样本/权重须保持身份。
// 末尾不足四项用普通四 lane/标量路径，无需元数据。空间证明只共享四坐标处于同一 8x8 tile 的组，
// 其他组独立包围盒；可选分配失败仍保留块证明。
class BilinearZeroSamples {
  const Patch* patch_=nullptr;
  std::size_t block_size_=32;
  double row_radius_=0,column_radius_=0;
  runtime::OverwriteBuffer<BilinearZeroBlock> blocks_;
  runtime::OverwriteBuffer<BilinearZeroBlock> tiles_;
  runtime::OverwriteBuffer<std::uint32_t> group_tiles_;
  std::size_t discarded_tile_bytes_=0;
  bool prepare_tiles(const Patch& patch) {
    const auto groups=patch.samples.size()/4;
    if(groups>std::numeric_limits<std::uint32_t>::max() || groups>std::numeric_limits<std::size_t>::max()/sizeof(BilinearZeroBlock))return false;
    struct Key {double row,column;std::uint32_t group;bool common;};
    static_assert(sizeof(Key)==24);
    // 浮点键避免异常有限几何的整数转换；全部有限四 lane 组按实际坐标定界，跨 8x8 tile 的组独立证明。
    try {
      // 普通临时缓冲返回时释放，调用方使用 monotonic arena 也如此；每四 lane 组峰值 64 字节，
      // 不保留哈希节点或桶扩容分配。
      std::vector<Key> keys;keys.reserve(groups);
      std::vector<BilinearZeroBlock> bounds;bounds.reserve(groups);
      for(std::size_t group=0;group<groups;++group) {
        const auto begin=4*group;
        BilinearZeroBlock box{begin,patch.relative_rows[begin],patch.relative_rows[begin],patch.relative_columns[begin],patch.relative_columns[begin]};
        Key key{std::floor(box.row_min/8),std::floor(box.column_min/8),std::uint32_t(group),true};
        for(std::size_t lane=1;lane<4;++lane) {
          const auto row=patch.relative_rows[begin+lane],column=patch.relative_columns[begin+lane];
          key.common=key.common && key.row==std::floor(row/8) && key.column==std::floor(column/8);
          box.row_min=std::min(box.row_min,row);box.row_max=std::max(box.row_max,row);
          box.column_min=std::min(box.column_min,column);box.column_max=std::max(box.column_max,column);
        }
        bounds.push_back(box);keys.push_back(key);
      }
      std::sort(keys.begin(),keys.end(),[](const Key& first,const Key& second) {
        if(first.common!=second.common)return first.common;
        if(first.common) {
          if(first.row!=second.row)return first.row<second.row;
          if(first.column!=second.column)return first.column<second.column;
        }
        return first.group<second.group;
      });
      const auto same_tile=[](const Key& first,const Key& second) {
        return first.common && second.common && first.row==second.row && first.column==second.column;
      };
      std::size_t tile_count=1;
      for(std::size_t index=1;index<groups;++index)tile_count+=!same_tile(keys[index-1],keys[index]);
      group_tiles_.reset_for_overwrite(groups);tiles_.reset_for_overwrite(tile_count);
      std::uint32_t tile_index=0;
      for(std::size_t index=0;index<groups;++index) {
        const auto& key=keys[index];const auto& box=bounds[key.group];
        if(index==0 || !same_tile(keys[index-1],key)) {
          if(index)++tile_index;
          tiles_[tile_index]=box;
        }else {
          auto& tile=tiles_[tile_index];
          tile.row_min=std::min(tile.row_min,box.row_min);tile.row_max=std::max(tile.row_max,box.row_max);
          tile.column_min=std::min(tile.column_min,box.column_min);tile.column_max=std::max(tile.column_max,box.column_max);
        }
        group_tiles_[key.group]=tile_index;
      }
      return true;
    }catch(const std::bad_alloc&) {
      // monotonic 资源在后续可选分配失败时仍保留先前成功分配，须计入累计查询预算。
      discarded_tile_bytes_+=tiles_.size()*sizeof(BilinearZeroBlock)+group_tiles_.size()*sizeof(std::uint32_t);
      tiles_.reset_for_overwrite(0);group_tiles_.reset_for_overwrite(0);return false;
    }
  }
public:
  BilinearZeroSamples(const Patch& patch,std::pmr::memory_resource* memory,std::size_t block_size=32,bool spatial_tiles=false):blocks_(0,memory),tiles_(0,memory),group_tiles_(0,memory) {
    const auto count=patch.samples.size();
    if((block_size!=16 && block_size!=32) || count<block_size || patch.relative_rows.size()!=count || patch.relative_columns.size()!=count || patch.weights.size()!=count)return;
    for(std::size_t index=0;index<count;++index) {
      if(!std::isfinite(patch.relative_rows[index]) || !std::isfinite(patch.relative_columns[index]) ||
          !std::isfinite(patch.weights[index]) || !std::isfinite(patch.samples[index].weight))return;
      row_radius_=std::max(row_radius_,std::abs(patch.relative_rows[index]));
      column_radius_=std::max(column_radius_,std::abs(patch.relative_columns[index]));
    }
    const auto groups=count/block_size;
    if(groups>std::numeric_limits<std::size_t>::max()/sizeof(BilinearZeroBlock))return;
    try {blocks_.reset_for_overwrite(groups);}catch(const std::bad_alloc&) {return;}
    for(std::size_t index=0;index<groups;++index) {
      auto& block=blocks_.data()[index];block.begin=index*block_size;
      block.row_min=block.row_max=patch.relative_rows[block.begin];
      block.column_min=block.column_max=patch.relative_columns[block.begin];
      for(std::size_t lane=1;lane<block_size;++lane) {
        const auto sample=block.begin+lane;
        block.row_min=std::min(block.row_min,patch.relative_rows[sample]);block.row_max=std::max(block.row_max,patch.relative_rows[sample]);
        block.column_min=std::min(block.column_min,patch.relative_columns[sample]);block.column_max=std::max(block.column_max,patch.relative_columns[sample]);
      }
    }
    patch_=&patch;block_size_=block_size;
    if(spatial_tiles)prepare_tiles(patch);
  }
  bool valid() const noexcept {return patch_!=nullptr;}
  std::size_t allocation_bytes() const noexcept {return (blocks_.size()+tiles_.size())*sizeof(BilinearZeroBlock)+group_tiles_.size()*sizeof(std::uint32_t)+discarded_tile_bytes_;}
  const Patch* patch() const noexcept {return patch_;}
  const BilinearZeroBlock* data() const noexcept {return blocks_.data();}
  std::size_t size() const noexcept {return blocks_.size();}
  std::size_t block_size() const noexcept {return block_size_;}
  double maximum_coordinate_radius() const noexcept {return row_radius_+column_radius_;}
  bool spatial_tiles() const noexcept {return tiles_.size()!=0;}
  std::size_t tile_count() const noexcept {return tiles_.size();}
  const BilinearZeroBlock* tiles() const noexcept {return tiles_.data();}
  const std::uint32_t* group_tiles() const noexcept {return group_tiles_.data();}
};

// 仅缓存最后位姿的 tile 证明；ready 状态下借用 Image/Patch/flag 存储须不可变且独占。
// 修改/复用前失效，身份不是内容哈希；源改变时图像/样本证明也须重建。
// 查询前仍检查 interior、有限偏移、RN/FTZ/DAZ；评分/导数可共享 flags，不共享样本矩。
class BilinearZeroProofCache {
  std::uint64_t row_=0,column_=0,angle_=0,cosine_=0,sine_=0;
  const BilinearZeroImage* image_=nullptr;
  const BilinearZeroSamples* samples_=nullptr;
  const double* pixels_=nullptr;
  const Patch* patch_=nullptr;
  std::uint8_t* flags_=nullptr;
  std::size_t width_=0,height_=0,capacity_=0;
  bool aligned_=false,ready_=false;
public:
  void invalidate() noexcept {ready_=false;}
  bool matches(const BilinearZeroImage& image,const BilinearZeroSamples& samples,const Match& pose,
      double ca,double sa,bool aligned,std::uint8_t* flags,std::size_t capacity)const noexcept {
    return ready_ && row_==std::bit_cast<std::uint64_t>(pose.row) && column_==std::bit_cast<std::uint64_t>(pose.column) &&
        angle_==std::bit_cast<std::uint64_t>(pose.angle) && cosine_==std::bit_cast<std::uint64_t>(ca) && sine_==std::bit_cast<std::uint64_t>(sa) &&
        image_==&image && samples_==&samples && pixels_==image.image() && patch_==samples.patch() &&
        width_==image.width() && height_==image.height() && flags_==flags && capacity_==capacity && aligned_==aligned;
  }
  void remember(const BilinearZeroImage& image,const BilinearZeroSamples& samples,const Match& pose,
      double ca,double sa,bool aligned,std::uint8_t* flags,std::size_t capacity)noexcept {
    row_=std::bit_cast<std::uint64_t>(pose.row);column_=std::bit_cast<std::uint64_t>(pose.column);angle_=std::bit_cast<std::uint64_t>(pose.angle);
    cosine_=std::bit_cast<std::uint64_t>(ca);sine_=std::bit_cast<std::uint64_t>(sa);image_=&image;samples_=&samples;
    pixels_=image.image();patch_=samples.patch();width_=image.width();height_=image.height();flags_=flags;capacity_=capacity;aligned_=aligned;ready_=true;
  }
};

struct BilinearZeroView {
  const BilinearZeroImage* image=nullptr;
  const BilinearZeroSamples* samples=nullptr;
  // 空间模式计数唯一 tile 证明，而非样本组。
  std::size_t* tested_blocks=nullptr;
  std::size_t* skipped_blocks=nullptr;
  // 调用方拥有且每次调用独占；读取前重新证明或精确位姿复用，无缓存时每次完整覆盖。
  std::uint8_t* tile_flags=nullptr;
  std::size_t tile_flags_capacity=0;
  BilinearZeroProofCache* proof_cache=nullptr;
};

} // namespace mv::ncc_matching
