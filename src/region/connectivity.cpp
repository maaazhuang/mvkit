#include <region/operators.hpp>
#include <region/banded_morphology.hpp>

#include <algorithm>
#include <limits>
#include <memory_resource>
#include <vector>

#include <runtime/memory/scratch_arena.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::regionproc {
namespace {
class DisjointSet {
 public:
  DisjointSet(std::size_t n, std::pmr::memory_resource* memory)
      : parent_(memory), rank_(memory) {
    parent_.reserve(n);rank_.reserve(n);
  }
  std::size_t add() {
    const auto label=parent_.size();parent_.push_back(label);rank_.push_back(0);
    return label;
  }
  std::size_t size() const {return parent_.size();}
  std::size_t find(std::size_t x) {
    while(parent_[x]!=x){parent_[x]=parent_[parent_[x]];x=parent_[x];}
    return x;
  }
  void unite(std::size_t a, std::size_t b) {
    a = find(a); b = find(b); if (a == b) return;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent_[b] = a;
    if (rank_[a] == rank_[b]) ++rank_[a];
  }
 private:
  std::pmr::vector<std::size_t> parent_;
  std::pmr::vector<unsigned char> rank_;
};

bool overlap4(const mv::core::RegionRun& a, const mv::core::RegionRun& b) noexcept {
  return !(a.col_end < b.col_begin || b.col_end < a.col_begin);
}

bool overlap8(const mv::core::RegionRun& a, const mv::core::RegionRun& b) noexcept {
  if (overlap4(a, b)) return true;
  if (a.col_end < b.col_begin) {
    return a.col_end != std::numeric_limits<std::int64_t>::max() && std::int64_t(a.col_end) + 1 >= b.col_begin;
  }
  return b.col_end != std::numeric_limits<std::int64_t>::max() && std::int64_t(b.col_end) + 1 >= a.col_begin;
}

void connect_runs(std::span<const mv::core::RegionRun> runs,
                  Connectivity connectivity, DisjointSet& dsu,
                  std::span<std::size_t> labels) {
  // 两行均为有序不相交区间，前行游标单调推进，仅访问可能重叠当前游程的区间。
  std::size_t previous_begin = 0, previous_end = 0, begin = 0;
  while (begin < runs.size()) {
    auto end = begin + 1;
    while (end < runs.size() && runs[end].row == runs[begin].row) ++end;
    if (previous_end > previous_begin &&
        runs[previous_begin].row != std::numeric_limits<std::int64_t>::max() &&
        std::int64_t(runs[previous_begin].row) + 1 == runs[begin].row) {
      auto cursor = previous_begin;
      for (auto i = begin; i < end; ++i) {
        auto label=std::numeric_limits<std::size_t>::max();
        const auto connected = [&](std::size_t j) {
          return connectivity == Connectivity::Four ? overlap4(runs[i], runs[j])
                                                     : overlap8(runs[i], runs[j]);
        };
        while (cursor < previous_end && runs[cursor].col_end < runs[i].col_begin && !connected(cursor)) ++cursor;
        for (auto j = cursor; j < previous_end; ++j) {
          if (connected(j)) {
            if(label==std::numeric_limits<std::size_t>::max())label=labels[j];
            else if(label!=labels[j])dsu.unite(label,labels[j]);
          }
          else if (runs[j].col_begin > runs[i].col_end) break;
        }
        labels[i]=label==std::numeric_limits<std::size_t>::max()?dsu.add():label;
      }
    } else for(auto i=begin;i<end;++i)labels[i]=dsu.add();
    previous_begin = begin; previous_end = end; begin = end;
  }
}

template<class Keep>
std::pmr::vector<mv::core::RegionRun> complement_runs(
    const mv::core::RectI64& bounds, std::span<const mv::core::RegionRun> runs, Keep keep) {
  std::pmr::vector<mv::core::RegionRun> result{mv::runtime::persist_resource()};
  result.reserve(runs.size());
  std::size_t index = 0;
  for (auto row = bounds.row_min;; ++row) {
    auto column = bounds.col_min;
    bool finished = false;
    while (index < runs.size() && runs[index].row == row) {
      const auto& run = runs[index];
      if (keep(index) && !finished) {
        if (column < run.col_begin) result.push_back({row, column, std::int64_t(run.col_begin) - 1});
        if (run.col_end == bounds.col_max) finished = true;
        else column = std::int64_t(run.col_end) + 1;
      }
      ++index;
    }
    if (!finished) result.push_back({row, column, bounds.col_max});
    if (row == bounds.row_max) break;
  }
  return result;
}

std::optional<mv::core::Region> fill_bands(const mv::core::Region& region,
                                          Connectivity connectivity) {
  auto foreground = detail::compress_bands(region);
  if (!foreground) return std::nullopt;
  const auto bounds = *region.bounding_box();
  detail::RowBands background;
  for (std::size_t i = 0; i < foreground->size(); ++i) {
    const auto& band = (*foreground)[i];
    if (i && (*foreground)[i-1].last + 1 < band.first)
      detail::append_band(background, (*foreground)[i-1].last + 1, band.first - 1,
                          {{bounds.col_min, bounds.col_max}});
    detail::BandProfile profile;
    auto column = bounds.col_min;
    bool finished = false;
    for (const auto& interval : band.profile) {
      if (column < interval.first) profile.push_back({column, interval.first - 1});
      if (interval.last == bounds.col_max) { finished = true; break; }
      column = interval.last + 1;
    }
    if (!finished) profile.push_back({column, bounds.col_max});
    detail::append_band(background, band.first, band.last, std::move(profile));
  }
  if (background.empty()) return region;
  mv::runtime::ScratchArena scratch;
  auto* memory = scratch.resource();
  std::pmr::vector<mv::core::RegionRun> representatives{memory};
  std::int64_t row = 0;
  for (std::size_t i = 0; i < background.size(); ++i) {
    if (i) row += background[i-1].last + 1 == background[i].first ? 1 : 2;
    for (const auto& interval : background[i].profile)
      representatives.push_back({row, interval.first, interval.last});
  }
  DisjointSet dsu(representatives.size(), memory);
  std::pmr::vector<std::size_t> labels(representatives.size(), memory);
  connect_runs(representatives, connectivity, dsu, labels);
  std::pmr::vector<unsigned char> exterior(dsu.size(), 0, memory);
  std::size_t index = 0;
  for (const auto& band : background) for (const auto& interval : band.profile) {
    labels[index] = dsu.find(labels[index]);
    if (band.first == bounds.row_min || band.last == bounds.row_max ||
        interval.first == bounds.col_min || interval.last == bounds.col_max)
      exterior[labels[index]] = 1;
    ++index;
  }
  detail::RowBands holes;
  index = 0;
  for (const auto& band : background) {
    detail::BandProfile profile;
    for (const auto& interval : band.profile)
      if (!exterior[labels[index++]]) profile.push_back(interval);
    detail::append_band(holes, band.first, band.last, std::move(profile));
  }
  if (holes.empty()) return region;
  return detail::expand_bands(detail::combine_bands(*foreground, holes, true));
}

// 只读紧凑 profile 仅本调用借用不可变源；分区输出独立拥有复制的游程。
struct ConnectivityBand {
  std::int64_t first,last;
  std::span<const mv::core::RegionRun> profile;
};
std::optional<std::pmr::vector<ConnectivityBand>> compress_connectivity_bands(
    const mv::core::Region& region,std::pmr::memory_resource* memory) {
  const auto runs=region.runs();if(runs.size()<128)return std::nullopt;
  std::pmr::vector<ConnectivityBand> bands{memory};std::size_t stored=0;
  for(std::size_t first=0;first<runs.size();) {
    if(!bands.empty() && bands.back().last!=std::numeric_limits<std::int64_t>::max() &&
       bands.back().last+1==runs[first].row) {
      const auto previous=bands.back().profile;const auto count=previous.size();
      bool same=count<=runs.size()-first && runs[first+count-1].row==runs[first].row &&
          (first+count==runs.size() || runs[first+count].row!=runs[first].row);
      if(same) {
        const auto difference=[&](std::size_t j) {
          return (std::uint64_t(previous[j].col_begin)^std::uint64_t(runs[first+j].col_begin)) |
                 (std::uint64_t(previous[j].col_end)^std::uint64_t(runs[first+j].col_end));
        };
        std::uint64_t d0=0,d1=0,d2=0,d3=0;std::size_t j=0;
        for(;j+4<=count;j+=4){d0|=difference(j);d1|=difference(j+1);d2|=difference(j+2);d3|=difference(j+3);}
        for(;j<count;++j)d0|=difference(j);same=(d0|d1|d2|d3)==0;
      }
      if(same){bands.back().last=runs[first].row;first+=count;continue;}
    }
    auto last=first+1;while(last<runs.size() && runs[last].row==runs[first].row)++last;
    stored+=last-first;if(stored>runs.size()/4)return std::nullopt;
    bands.push_back({runs[first].row,runs[first].row,runs.subspan(first,last-first)});first=last;
  }
  return bands;
}

void label_region(const mv::core::Region& region,Connectivity connectivity,
                  DisjointSet& dsu,std::pmr::vector<std::size_t>& labels) {
  auto* memory=labels.get_allocator().resource();
  auto bands=compress_connectivity_bands(region,memory);
  if(!bands) {connect_runs(region.runs(),connectivity,dsu,labels);return;}
  // 相邻同 profile 包含纵向相连矩形，各区间为一个节点；合成行保持邻接及实际空隙。
  std::pmr::vector<mv::core::RegionRun> representatives{memory};
  std::int64_t row=0;
  for(std::size_t i=0;i<bands->size();++i) {
    const auto& band=(*bands)[i];
    if(i)row+=(*bands)[i-1].last!=std::numeric_limits<std::int64_t>::max() &&
                 (*bands)[i-1].last+1==band.first?1:2;
    for(const auto& interval:band.profile)representatives.push_back({row,interval.col_begin,interval.col_end});
  }
  std::pmr::vector<std::size_t> compact_labels(representatives.size(),memory);
  connect_runs(representatives,connectivity,dsu,compact_labels);
  std::size_t source=0,destination=0;
  for(const auto& band:*bands) {
    const auto size=band.profile.size();
    for(auto r=band.first;;++r) {
      std::copy_n(compact_labels.data()+source,size,labels.data()+destination);
      destination+=size;
      if(r==band.last)break;
    }
    source+=size;
  }
}

std::size_t component_labels(const mv::core::Region& region,Connectivity connectivity,
                      std::span<std::size_t>& labels,
                      std::pmr::vector<std::size_t>& counts,
                      std::pmr::vector<mv::core::RegionPartitionBand>& partitions) {
  auto* memory=counts.get_allocator().resource();
  auto bands=compress_connectivity_bands(region,memory);
  std::pmr::vector<mv::core::RegionRun> representatives{memory};
  if(bands) {
    std::size_t compact_count=0;
    for(const auto& band:*bands)compact_count+=band.profile.size();
    representatives.reserve(compact_count);
    partitions.reserve(bands->size());
    std::int64_t row=0;
    for(std::size_t i=0;i<bands->size();++i) {
      const auto& band=(*bands)[i];
      if(i)row+=(*bands)[i-1].last!=std::numeric_limits<std::int64_t>::max() &&
                   (*bands)[i-1].last+1==band.first?1:2;
      for(const auto& interval:band.profile)representatives.push_back({row,interval.col_begin,interval.col_end});
    }
  }
  const auto nodes=bands?std::span<const mv::core::RegionRun>{representatives}:region.runs();
  DisjointSet dsu(nodes.size(),memory);
  // 按压缩后数量分配工作区，不按展开源；标签使用前写完，原始存储归 arena 所有。
  labels={std::pmr::polymorphic_allocator<std::size_t>{memory}.allocate(nodes.size()),nodes.size()};
  auto node_labels=labels;
  connect_runs(nodes,connectivity,dsu,node_labels);
  counts.reserve(dsu.size());
  const auto none=nodes.size();
  std::pmr::vector<std::size_t> root_group(dsu.size(),none,memory);
  const auto assign=[&](std::size_t& label,std::size_t repetitions) {
    auto& group=root_group[dsu.find(label)];
    if(group==none){group=counts.size();counts.push_back(0);}
    counts[group]+=repetitions;label=group;
  };
  if(!bands) {for(auto& label:labels)assign(label,1);return labels.size();}
  std::size_t source=0;
  for(const auto& band:*bands) {
    const auto size=band.profile.size();
    const auto rows=static_cast<std::size_t>(std::uint64_t(band.last)-std::uint64_t(band.first)+1);
    for(std::size_t j=0;j<size;++j)assign(node_labels[source+j],rows);
    partitions.push_back({size,rows});
    source+=size;
  }
  return nodes.size();
}

}

mv::core::Regions connection(const mv::core::Region& region, Connectivity connectivity) {
  // 构造时已经严格证明四邻接连通的区域，同样满足八邻接；无需重新打标签或复制游程。
  if(region.known_connected4() || (connectivity==Connectivity::Eight && region.known_connected8()))
    return mv::core::Regions{{region}};
  const auto runs = region.runs();
  if (runs.empty()) return {};
  bool separated_rows=true;
  for(std::size_t i=1;i<runs.size();++i)if(runs[i].row!=runs[i-1].row &&
      runs[i-1].row!=std::numeric_limits<std::int64_t>::max() &&
      runs[i].row==std::int64_t(runs[i-1].row)+1) {separated_rows=false;break;}
  if(separated_rows) {
    auto parts=mv::core::Region::partition_runs(region);
    for(const auto& part:parts)part.record_connected_component(true);
    return mv::core::Regions{std::move(parts)};
  }

  mv::runtime::ScratchArena scratch;
  auto* memory = scratch.resource();
  std::span<std::size_t> labels;
  std::pmr::vector<std::size_t> counts{memory};
  std::pmr::vector<mv::core::RegionPartitionBand> partitions{memory};
  const auto compact_size=component_labels(region,connectivity,labels,counts,partitions);
  auto parts=mv::core::Region::partition_runs(region,labels.first(compact_size),counts,partitions);
  for(const auto& part:parts)part.record_connected_component(connectivity==Connectivity::Four);
  return mv::core::Regions{std::move(parts)};
}

mv::core::Regions connection(const mv::core::Regions& regions, Connectivity connectivity) {
  bool proven=true;
  for(std::size_t i=0;i<regions.size();++i)
    if(!regions[i].known_connected4() && !(connectivity==Connectivity::Eight && regions[i].known_connected8())) {
      proven=false;break;
    }
  if(proven)return regions;
  if(regions.size()==1)return connection(regions[0],connectivity);
  std::vector<mv::core::IconicValue> out;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const auto parts = connection(regions[index], connectivity);
    const auto values=parts.iconic_tuple().values();
    out.insert(out.end(),values.begin(),values.end());
  }
  return mv::core::Regions{mv::core::IconicTuple{std::move(out)}};
}

mv::core::Regions connection(const mv::core::Regions& regions,
                             const mv::core::RuntimeContext& context) {
  const auto connectivity = context.neighborhood == mv::core::Neighborhood::Four
                                ? Connectivity::Four
                                : Connectivity::Eight;
  if(context.max_connection==0)return connection(regions,connectivity);
  std::vector<mv::core::IconicValue> out;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const auto parts = connection(regions[index], connectivity);
    for (std::size_t part = 0; part < parts.size(); ++part) {
      if (context.max_connection != 0 && out.size() >= context.max_connection) {
        return mv::core::Regions{mv::core::IconicTuple{std::move(out)}};
      }
      out.push_back(parts.iconic_tuple().at(part));
    }
  }
  return mv::core::Regions{mv::core::IconicTuple{std::move(out)}};
}

mv::core::Region fill_up(const mv::core::Region& region, Connectivity connectivity) {
  const auto bounds = region.bounding_box();
  if (!bounds) return region;
  // 有界背景分量须上、侧、下均有前景；若没有三条连续占用行，
  // 所有背景像素均可经缺失行逃逸，4/8 连通均如此。
  auto previous = region.runs().front().row;
  unsigned consecutive = 1;
  for (const auto& run : region.runs()) {
    if (run.row == previous) continue;
    consecutive = previous != std::numeric_limits<std::int64_t>::max() &&
                          std::int64_t(previous) + 1 == run.row ? consecutive + 1 : 1;
    previous = run.row;
    if (consecutive == 3) break;
  }
  if (consecutive < 3) return region;
  if (auto filled = fill_bands(region, connectivity)) return *filled;
  const auto background = mv::core::Region::from_canonical_runs(
      complement_runs(*bounds, region.runs(), [](std::size_t) { return true; }));
  if (background.empty()) return region;
  mv::runtime::ScratchArena scratch;
  const auto runs = background.runs();
  DisjointSet dsu(runs.size(), scratch.resource());
  std::pmr::vector<std::size_t> labels(runs.size(),scratch.resource());
  label_region(background, connectivity, dsu,labels);
  std::pmr::vector<unsigned char> exterior(dsu.size(), 0, scratch.resource());
  for (std::size_t i = 0; i < runs.size(); ++i) {
    const auto& r = runs[i];
    labels[i]=dsu.find(labels[i]);
    if (r.row == bounds->row_min || r.row == bounds->row_max ||
        r.col_begin == bounds->col_min || r.col_end == bounds->col_max)
      exterior[labels[i]] = 1;
  }
  if (std::none_of(labels.begin(), labels.end(), [&](auto label) { return !exterior[label]; }))
    return region;
  return mv::core::Region::from_canonical_runs(complement_runs(
      *bounds, runs, [&](std::size_t i) { return exterior[labels[i]] != 0; }));
}

mv::core::Regions fill_up(const mv::core::Regions& regions, Connectivity connectivity) {
  std::vector<mv::core::IconicValue> out;
  out.reserve(regions.size());
  for (std::size_t index = 0; index < regions.size(); ++index) {
    auto filled=fill_up(regions[index], connectivity);
    // 填充产生超集，面积相等即证明对象未变。
    out.push_back(filled.area()==regions[index].area()?regions.iconic_tuple().at(index):
                  mv::core::IconicValue::from_region(std::move(filled)));
  }
  return mv::core::Regions{mv::core::IconicTuple{std::move(out)}};
}

mv::core::Regions fill_up(const mv::core::Regions& regions,
                          const mv::core::RuntimeContext& context) {
  const auto connectivity = context.neighborhood == mv::core::Neighborhood::Four
                                ? Connectivity::Four
                                : Connectivity::Eight;
  return fill_up(regions, connectivity);
}

}  // namespace mv::regionproc
