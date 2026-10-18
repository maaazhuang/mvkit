#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include <core/exception.hpp>
#include <core/region.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::regionproc::detail {

using BandInterval = core::RegionInterval;
using BandProfile = std::vector<BandInterval>;
struct RowBand {
  std::int64_t first, last;
  BandProfile profile;
};
using RowBands = std::vector<RowBand>;

inline std::int64_t band_add(std::int64_t value, std::int64_t delta) {
  if ((delta > 0 && value > std::numeric_limits<std::int64_t>::max()-delta) ||
      (delta < 0 && value < std::numeric_limits<std::int64_t>::min()-delta))
    throw core::Exception(core::Status::OutOfRange,"region morphology","coordinate overflow");
  return value+delta;
}

inline bool band_sub_safe(std::int64_t value, std::int64_t delta) {
  return !((delta > 0 && value < std::numeric_limits<std::int64_t>::min()+delta) ||
           (delta < 0 && value > std::numeric_limits<std::int64_t>::max()+delta));
}

// 仅压缩连续行的相同 profile，行间空隙保持；非规则输入用普通 RLE，
// 避免整次形态学为 profile 支付分配/复制成本。
inline std::optional<RowBands> compress_bands(const core::Region& region) {
  const auto runs=region.runs();
  if (runs.size()<128) return std::nullopt;
  RowBands bands;
  std::size_t stored=0;
  for (std::size_t first=0;first<runs.size();) {
    if(!bands.empty() && bands.back().last!=std::numeric_limits<std::int64_t>::max() &&
       bands.back().last+1==runs[first].row) {
      const auto& profile=bands.back().profile;
      const auto count=profile.size();
      // 规范行序使两边界足以证明行长度；重复行仅比较列，不重复扫描同一行边界。
      bool same=count<=runs.size()-first && runs[first+count-1].row==runs[first].row &&
          (first+count==runs.size() || runs[first+count].row!=runs[first].row);
      if(same) {
        // 区间差累加不逐项分支，长重复行 profile 可向量归约。
        const auto difference=[&](std::size_t j) {
          return (std::uint64_t(profile[j].first)^std::uint64_t(runs[first+j].col_begin)) |
                 (std::uint64_t(profile[j].last)^std::uint64_t(runs[first+j].col_end));
        };
        std::uint64_t d0=0,d1=0,d2=0,d3=0;
        std::size_t j=0;
        for(;j+4<=count;j+=4) {
          d0|=difference(j);d1|=difference(j+1);
          d2|=difference(j+2);d3|=difference(j+3);
        }
        for(;j<count;++j)d0|=difference(j);
        same=(d0|d1|d2|d3)==0;
      }
      if(same) {bands.back().last=runs[first].row;first+=count;continue;}
    }
    auto last=first+1;
    while(last<runs.size() && runs[last].row==runs[first].row)++last;
    stored+=last-first;
    if(stored>runs.size()/4)return std::nullopt;
    BandProfile profile;
    profile.reserve(last-first);
    for(auto i=first;i<last;++i)profile.push_back({runs[i].col_begin,runs[i].col_end});
    bands.push_back({runs[first].row,runs[first].row,std::move(profile)});
    first=last;
  }
  return bands;
}

inline void append_interval(BandProfile& out,BandInterval value) {
  if(!out.empty() && (value.first<=out.back().last ||
      (out.back().last!=std::numeric_limits<std::int64_t>::max() && value.first==out.back().last+1)))
    out.back().last=std::max(out.back().last,value.last);
  else out.push_back(value);
}

inline BandProfile combine_profiles(const BandProfile& a,const BandProfile& b,bool dilation) {
  BandProfile out;
  out.reserve(dilation?a.size()+b.size():std::min(a.size(),b.size()));
  std::size_t i=0,j=0;
  if(dilation) {
    while(i<a.size() || j<b.size())
      append_interval(out,j==b.size() || (i<a.size() && a[i].first<=b[j].first)?a[i++]:b[j++]);
  } else {
    while(i<a.size() && j<b.size()) {
      const auto first=std::max(a[i].first,b[j].first),last=std::min(a[i].last,b[j].last);
      if(first<=last)out.push_back({first,last});
      if(a[i].last<b[j].last)++i;else ++j;
    }
  }
  return out;
}

inline void append_band(RowBands& out,std::int64_t first,std::int64_t last,BandProfile profile) {
  if(profile.empty())return;
  if(!out.empty() && out.back().last!=std::numeric_limits<std::int64_t>::max() &&
      out.back().last+1==first && out.back().profile==profile)
    out.back().last=last;
  else out.push_back({first,last,std::move(profile)});
}

// 扫描两条有序 band 流，仅 profile 变化时切分，整个重叠范围只算一次区间并/交。
inline RowBands combine_bands(const RowBands& a,const RowBands& b,bool dilation) {
  RowBands out;
  out.reserve(a.size()+b.size());
  std::size_t i=0,j=0;
  auto ar=a.empty()?0:a[0].first,br=b.empty()?0:b[0].first;
  while(i<a.size() && j<b.size()) {
    if(a[i].last<br) {
      if(dilation)append_band(out,ar,a[i].last,a[i].profile);
      if(++i<a.size())ar=a[i].first;
    } else if(b[j].last<ar) {
      if(dilation)append_band(out,br,b[j].last,b[j].profile);
      if(++j<b.size())br=b[j].first;
    } else {
      if(dilation && ar<br)append_band(out,ar,br-1,a[i].profile);
      if(dilation && br<ar)append_band(out,br,ar-1,b[j].profile);
      const auto first=std::max(ar,br),last=std::min(a[i].last,b[j].last);
      append_band(out,first,last,combine_profiles(a[i].profile,b[j].profile,dilation));
      if(a[i].last==last) {if(++i<a.size())ar=a[i].first;} else ar=last+1;
      if(b[j].last==last) {if(++j<b.size())br=b[j].first;} else br=last+1;
    }
  }
  if(dilation) {
    while(i<a.size()) {append_band(out,ar,a[i].last,a[i].profile);if(++i<a.size())ar=a[i].first;}
    while(j<b.size()) {append_band(out,br,b[j].last,b[j].profile);if(++j<b.size())br=b[j].first;}
  }
  return out;
}

inline core::Region expand_bands(const RowBands& bands) {
  std::vector<core::RegionBandView> views;
  views.reserve(bands.size());
  for(const auto& band:bands)views.push_back({band.first,band.last,band.profile});
  return core::Region::from_row_bands(views);
}

inline std::optional<core::Region> banded_morphology(const core::Region& region,
                                                   const core::Region& element,bool dilation) {
  auto bands=compress_bands(region);
  if(!bands)return std::nullopt;
  const auto bounds=*region.bounding_box(),eb=*element.bounding_box();
  if(dilation) {
    (void)band_add(bounds.row_min,eb.row_min);(void)band_add(bounds.row_max,eb.row_max);
    (void)band_add(bounds.col_min,eb.col_min);(void)band_add(bounds.col_max,eb.col_max);
  } else if(!band_sub_safe(bounds.row_min,eb.row_max) || !band_sub_safe(bounds.row_max,eb.row_min) ||
            !band_sub_safe(bounds.col_min,eb.col_max) || !band_sub_safe(bounds.col_max,eb.col_min))
    return std::nullopt; // 保留既有中间值检查行为。
  RowBands result;
  bool first=true;
  for(const auto& se:element.runs()) {
    RowBands shifted;
    shifted.reserve(bands->size());
    for(const auto& band:*bands) {
      BandProfile profile;
      profile.reserve(band.profile.size());
      for(const auto& interval:band.profile) {
        if(dilation)append_interval(profile,{interval.first+se.col_begin,interval.last+se.col_end});
        else {
          const auto begin=interval.first-se.col_begin,end=interval.last-se.col_end;
          if(begin<=end)profile.push_back({begin,end});
        }
      }
      append_band(shifted,dilation?band.first+se.row:band.first-se.row,
                  dilation?band.last+se.row:band.last-se.row,std::move(profile));
    }
    result=first?std::move(shifted):combine_bands(result,shifted,dilation);
    first=false;
    if(result.empty())break;
  }
  return expand_bands(result);
}

inline std::optional<core::Region> banded_rectangle(const core::Region& region,
                                                   std::int64_t w,std::int64_t h,bool dilation) {
  auto bands=compress_bands(region);
  if(!bands)return std::nullopt;
  RowBands result;
  for(const auto& band:*bands) {
    BandProfile profile;
    profile.reserve(band.profile.size());
    for(const auto& interval:band.profile) {
      if(!dilation && static_cast<std::uint64_t>(interval.last)-static_cast<std::uint64_t>(interval.first)<static_cast<std::uint64_t>(w-1))continue;
      append_interval(profile,{band_add(interval.first,dilation?-(w-1)/2:(w-1)/2),
                               band_add(interval.last,dilation?w/2:-w/2)});
    }
    if(profile.empty())continue;
    const auto delta=dilation?-(h-1)/2:-h/2;
    append_band(result,band_add(band.first,delta),band_add(band.last,delta),std::move(profile));
  }
  for(std::int64_t covered=1;covered<h && !result.empty();) {
    const auto step=std::min(covered,h-covered);
    auto shifted=result;
    for(auto& band:shifted) {band.first=band_add(band.first,step);band.last=band_add(band.last,step);}
    result=combine_bands(result,shifted,dilation);
    covered+=step;
  }
  return expand_bands(result);
}

} // namespace mv::regionproc::detail
