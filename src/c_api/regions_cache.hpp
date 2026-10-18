#pragma once
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include <utility>
#include <type_traits>
#include <cstddef>
namespace mv::c_api{
void ensure_centroid_cache(MvObjectImpl&);
inline void prepare_regions_cache(MvObjectImpl& h) {
  // 不可变批次一次固定描述符数组；各导出缓冲成功发布后不移动、不修改。
  std::call_once(h.ExportCache().run_cache_once,[&] {
    h.ExportCache().run_cache=std::vector<MVRegionRunExport>(mv::core::Regions{h.value}.size());
  });
}
inline void ensure_region_cache(MvObjectImpl& h,std::size_t index) {
  auto& exported=h.ExportCache().run_cache[index];
  std::call_once(exported.once,[&] {
    // prepare_regions_cache 已对不可变批次完成一次验证。
    const auto& region=h.value.at(index).as_region();
    static_assert(std::is_trivially_copyable_v<::MvRegionRun> && std::is_trivially_copyable_v<core::RegionRun>);
    static_assert(std::is_standard_layout_v<::MvRegionRun> && std::is_standard_layout_v<core::RegionRun>);
    static_assert(sizeof(::MvRegionRun)==sizeof(core::RegionRun));
    static_assert(std::is_same_v<decltype(::MvRegionRun::row),decltype(core::RegionRun::row)>);
    static_assert(std::is_same_v<decltype(::MvRegionRun::col_begin),decltype(core::RegionRun::col_begin)>);
    static_assert(std::is_same_v<decltype(::MvRegionRun::col_end),decltype(core::RegionRun::col_end)>);
    static_assert(offsetof(::MvRegionRun,row)==offsetof(core::RegionRun,row));
    static_assert(offsetof(::MvRegionRun,col_begin)==offsetof(core::RegionRun,col_begin));
    static_assert(offsetof(::MvRegionRun,col_end)==offsetof(core::RegionRun,col_end));
    const auto count=region.run_count();decltype(exported.values) values;
    if(count) {
      if(count*sizeof(::MvRegionRun)>=8192) {
        auto* memory=mv::runtime::persist_resource();
        auto* buffer=std::pmr::polymorphic_allocator<::MvRegionRun>{memory}.allocate(count);
        values={buffer,MVRegionRunDeleter{memory,count}};
        std::uninitialized_default_construct_n(buffer,count);
      } else values={new ::MvRegionRun[count],MVRegionRunDeleter{nullptr,count}};
      region.copy_run_bytes({reinterpret_cast<std::byte*>(values.get()),count*sizeof(::MvRegionRun)});
    }
    // 分配/复制异常时保留旧指针和数量；call_once 可安全重试，其他独立导出保持有效。
    exported.values=std::move(values);exported.count=count;
  });
}
inline ::MvObject make_regions(mv::core::IconicTuple value){value.require_kind(mv::core::IconicKind::Region,"make_regions");auto out=std::make_unique<MvObjectImpl>();out->value=std::move(value);return out.release();}
inline ::MvObject make_regions(mv::core::Regions value){return make_regions(value.iconic_tuple());}
inline ::MvObject make_regions(mv::core::Region value){return make_regions(mv::core::IconicTuple{mv::core::IconicValue::from_region(std::move(value))});}
}
