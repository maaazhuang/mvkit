#pragma once
#include "opaque.hpp"
#include <reconstruction/neighborhood_index.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <mutex>
#include <shared_mutex>
#include <cmath>
#include <limits>
#include <algorithm>
namespace mv::c_api {
struct ObjectModelCoordinateColumns {
  std::array<std::once_flag,3> ready;
  std::array<std::vector<double,runtime::PlainPayloadAllocator<double>>,3> columns;
  std::array<std::vector<double,runtime::PlainPayloadAllocator<double>>,3> block_ranges;
};
// 只缓存不可变坐标，不缓存筛选谓词或结果。单个查询轴占 8*N 字节，首次使用承担准备与发布成本。
inline std::span<const double> object_model_coordinate_column(MvObjectModel3DImpl* model,
                                                              std::string_view attribute) {
  if(model->value.points.size()<65536)return {};
  const unsigned axis=attribute=="point_coord_x" ? 0 : attribute=="point_coord_y" ? 1 :
                      attribute=="point_coord_z" ? 2 : 3;
  if(axis==3)return {};
  std::call_once(model->coordinate_once,[&] {
    model->coordinate_columns=std::allocate_shared<ObjectModelCoordinateColumns>(
        std::pmr::polymorphic_allocator<ObjectModelCoordinateColumns>{runtime::persist_resource()});
  });
  auto& cache=*model->coordinate_columns;
  std::call_once(cache.ready[axis],[&] {
    decltype(cache.columns)::value_type values{runtime::PlainPayloadAllocator<double>{runtime::persist_resource(),true}};
    values.resize(model->value.points.size());
    for(std::size_t i=0;i<values.size();++i) {
      const auto& point=model->value.points[i];
      values[i]=axis==0 ? point.x : axis==1 ? point.y : point.z;
    }
    decltype(cache.columns)::value_type ranges{runtime::PlainPayloadAllocator<double>{runtime::persist_resource(),true}};
    ranges.resize(model->value.valid.empty()?values.size()/16*2:0);
    for(std::size_t first=0;first/16*2<ranges.size();first+=16) {
      double lower=values[first],upper=lower;
      bool finite=std::isfinite(lower);
      for(std::size_t i=first+1;i<first+16;++i) {
        finite=finite && std::isfinite(values[i]);
        lower=std::min(lower,values[i]);upper=std::max(upper,values[i]);
      }
      // 即使边界扩展到无穷，NaN 摘要也要求完整比较。
      ranges[first/16*2]=finite?lower:std::numeric_limits<double>::quiet_NaN();
      ranges[first/16*2+1]=finite?upper:std::numeric_limits<double>::quiet_NaN();
    }
    cache.block_ranges[axis]=std::move(ranges);
    cache.columns[axis]=std::move(values);
  });
  return cache.columns[axis];
}
inline std::span<const double> object_model_coordinate_ranges(MvObjectModel3DImpl* model,
                                                              std::string_view attribute) {
  // 仅在同一轴坐标列经 call_once 发布后调用。
  if(!model->coordinate_columns)return {};
  const unsigned axis=attribute=="point_coord_x"?0:attribute=="point_coord_y"?1:attribute=="point_coord_z"?2:3;
  return axis<3 ? std::span<const double>{model->coordinate_columns->block_ranges[axis]} : std::span<const double>{};
}
// ObjectModel3D 载荷不可变，索引引用载荷并共享句柄生命周期；变换创建新句柄及新缓存。
inline std::shared_ptr<const reconstruction::NeighborhoodIndex>
object_model_index(MvObjectModel3DImpl* model) {
  { std::shared_lock lock(model->nearest_mutex); if(model->nearest_index)return model->nearest_index; }
  std::unique_lock lock(model->nearest_mutex);
  if(!model->nearest_index)model->nearest_index=std::make_shared<reconstruction::NeighborhoodIndex>(model->value,runtime::persist_resource());
  return model->nearest_index;
}
} // namespace mv::c_api
