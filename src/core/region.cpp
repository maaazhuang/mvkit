#include <core/region.hpp>
#include <core/run_copy_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>

#include <algorithm>
#include <bit>
#include <atomic>
#include <cstring>
#include <limits>
#include <iterator>
#include <type_traits>
#include <memory_resource>
#include <mutex>

#include <core/exception.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <runtime/memory/scratch_arena.hpp>

namespace mv::core {

struct Region::Storage {
  std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};
  std::shared_ptr<const void> owner;
  std::span<const RegionRun> shared_runs;
  std::uint64_t area = 0;
  std::uint64_t max_run_length = 0;
  // 已发布存储始终非空，空状态由 Region 句柄表达。
  RectI64 bounds;
  bool owns_runs=false;
  struct Segments {
    std::shared_ptr<const Storage> owner, middle_owner;
    std::pmr::vector<std::span<const RegionRun>> pieces{mv::runtime::persist_resource()};
    mutable std::once_flag continuous_once;
  };
  bool segmented=false; // 使用既有尾部 padding，平面元数据尺寸不变。
  mutable std::atomic<std::uint8_t> connected{0}; // padding 内的派生事实：位 0 为四邻接，位 1 为八邻接。
  const Segments* segment_data() const noexcept { return segmented ? static_cast<const Segments*>(owner.get()) : nullptr; }

  void materialize() const {
    const auto* fragments=segment_data();if(!fragments)return;
    std::call_once(fragments->continuous_once,[this,fragments] {
      std::size_t count=0;for(const auto piece:fragments->pieces)count+=piece.size();
      auto* output=const_cast<Storage*>(this)->allocate_runs(count,"Region::runs");
      for(const auto piece:fragments->pieces){std::memcpy(output,piece.data(),piece.size_bytes());output+=piece.size();}
    });
  }

  ~Storage() {
    if(owns_runs)runs.get_allocator().resource()->deallocate(
        const_cast<RegionRun*>(shared_runs.data()),shared_runs.size_bytes(),alignof(RegionRun));
  }

  RegionRun* allocate_runs(std::size_t count,const char* operation) {
    static_assert(std::is_trivially_destructible_v<RegionRun>);
    if(count>runs.max_size())
      throw Exception(Status::OutOfRange,operation,"run count exceeds allocation range");
    auto* memory=runs.get_allocator().resource();
    auto* buffer=static_cast<RegionRun*>(memory->allocate(count*sizeof(RegionRun),alignof(RegionRun)));
    shared_runs={buffer,count};owns_runs=true;
    return buffer;
  }
};

Region Region::from_runs(std::span<const RegionRun> input) {
  if (input.empty()) return {};
  const auto less=[](const RegionRun& a,const RegionRun& b) {
    if(a.row!=b.row)return a.row<b.row;
    if(a.col_begin!=b.col_begin)return a.col_begin<b.col_begin;
    return a.col_end<b.col_end;
  };
  bool sorted=true;
  for(std::size_t i=0;i<input.size();++i) {
    if(input[i].col_begin>input[i].col_end)
      throw Exception(Status::InvalidArgument,"Region::from_runs","run begin column must not exceed end column");
    if(i && less(input[i],input[i-1]))sorted=false;
  }
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<RegionRun> buffer{scratch.resource()};
  auto normalized=input;
  if(!sorted) {
    buffer.assign(input.begin(),input.end());
    std::sort(buffer.begin(),buffer.end(),less);
    normalized=buffer;
  }
  auto storage=std::make_shared<Storage>();
  storage->runs.reserve(normalized.size());
  RectI64 bounds{normalized.front().row,normalized.front().col_begin,
                 normalized.front().row,normalized.front().col_end};
  std::uint64_t area=0;
  // 每个合并游程仅完成一次；复制及元数据共用规范化遍历，不再遍历全部输出计算面积/边界。
  const auto emit=[&](const RegionRun& run) {
    constexpr std::uint64_t sign=std::uint64_t{1}<<63;
    const auto distance=(static_cast<std::uint64_t>(run.col_end)^sign)-
                        (static_cast<std::uint64_t>(run.col_begin)^sign);
    if(distance==std::numeric_limits<std::uint64_t>::max())
      throw Exception(Status::OutOfRange,"Region::from_runs","run length exceeds uint64 range");
    const auto length=distance+1;
    if(area>std::numeric_limits<std::uint64_t>::max()-length)
      throw Exception(Status::OutOfRange,"Region::from_runs","region area exceeds uint64 range");
    area+=length;
    storage->max_run_length=std::max(storage->max_run_length,length);
    bounds.row_max=run.row;
    bounds.col_min=std::min<std::int64_t>(bounds.col_min,run.col_begin);
    bounds.col_max=std::max<std::int64_t>(bounds.col_max,run.col_end);
    storage->runs.push_back(run);
  };
  auto current=normalized.front();
  for(std::size_t i=1;i<normalized.size();++i) {
    const auto& run=normalized[i];
    const bool adjacent=current.col_end!=std::numeric_limits<std::int64_t>::max() &&
                        run.col_begin==std::int64_t(current.col_end)+1;
    if(current.row==run.row && (run.col_begin<=current.col_end || adjacent))
      current.col_end=std::max<std::int64_t>(current.col_end,run.col_end);
    else { emit(current);current=run; }
  }
  emit(current);
  storage->area=area;
  storage->bounds=bounds;
  return Region{std::move(storage)};
}

Region Region::from_canonical_runs(std::pmr::vector<RegionRun>&& input) {
  if(input.empty())return {};
  const auto height=std::int64_t(input.back().row)-input.front().row+1;
  if(height>0 && std::uint64_t(height)==input.size() && input.size()<UINT64_C(4294967296)) {
    // 每行恰一条游程时独立验证行号；仍检查全部端点，归约不依赖前一条游程。
    // 行数 <2^32 且每行长度 <=2^32，合法输入的面积之和必可用 UInt64 表示。
    bool invalid=false;
    std::uint64_t area=0,max_length=0;
    std::int32_t first=INT32_MAX,last=INT32_MIN;
    for(std::size_t i=0;i<input.size();++i) {
      const auto& run=input[i];
      invalid|=std::int64_t(run.row)!=std::int64_t(input.front().row)+std::int64_t(i) || run.col_begin>run.col_end;
      const auto length=static_cast<std::uint64_t>(std::int64_t(run.col_end)-run.col_begin+1);
      area+=length;max_length=std::max(max_length,length);
      first=std::min(first,run.col_begin);last=std::max(last,run.col_end);
    }
    if(!invalid) {
      const RectI64 box{input.front().row,first,input.back().row,last};
      auto storage=std::make_shared<Storage>();storage->runs=std::move(input);
      storage->area=area;storage->max_run_length=max_length;storage->bounds=box;
      return Region{std::move(storage)};
    }
    // 非法候选保留通用验证器的错误优先级。
  }
  RectI64 bounds{input.front().row,input.front().col_begin,input.back().row,input.front().col_end};
  std::uint64_t area=0,max_length=0;
  bool overflow=false;
  auto previous_row=input.front().row,previous_end=input.front().col_end;
  for(std::size_t i=0;i<input.size();++i) {
    const auto& run=input[i];
    if(run.col_begin>run.col_end || (i &&
        (run.row<previous_row || (run.row==previous_row &&
         (run.col_begin<=previous_end ||
          (previous_end!=std::numeric_limits<std::int64_t>::max() &&
           run.col_begin==std::int64_t(previous_end)+1))))))
      throw Exception(Status::InvalidArgument,"Region::from_canonical_runs","runs must be canonical");
    const auto difference=static_cast<std::uint64_t>(run.col_end)-static_cast<std::uint64_t>(run.col_begin);
    const auto length=difference+1;
    overflow|=length==0 || area>std::numeric_limits<std::uint64_t>::max()-length;
    area+=length;
    max_length=std::max(max_length,length);
    // 每行规范列递增，只有首个 begin 和最后 end 能扩展全局列边界。
    if(i && run.row!=previous_row) {
      bounds.col_min=std::min<std::int64_t>(bounds.col_min,run.col_begin);
      bounds.col_max=std::max<std::int64_t>(bounds.col_max,previous_end);
    }
    previous_row=run.row;previous_end=run.col_end;
  }
  bounds.col_max=std::max<std::int64_t>(bounds.col_max,previous_end);
  // 此前合法游程已溢出时，仍保留非法输入的错误优先级。
  if(overflow)throw Exception(Status::OutOfRange,"Region::from_canonical_runs","region area exceeds uint64 range");
  auto storage=std::make_shared<Storage>();
  storage->runs=std::move(input);
  storage->area=area;storage->max_run_length=max_length;storage->bounds=bounds;
  return Region{std::move(storage)};
}


Region Region::from_shared_canonical_runs(
    std::shared_ptr<const std::pmr::vector<RegionRun>> owner,
    std::size_t offset, std::size_t count) {
  if (!owner || owner->get_allocator().resource()!=mv::runtime::persist_resource() ||
      offset>owner->size() || count>owner->size()-offset)
    throw Exception(Status::InvalidArgument,"Region::from_shared_canonical_runs","invalid storage slice");
  if (!count) return {};
  const auto input=std::span<const RegionRun>{*owner}.subspan(offset,count);
  RectI64 bounds{input.front().row,input.front().col_begin,input.back().row,input.front().col_end};
  std::uint64_t area=0,max_length=0;
  bool overflow=false;
  for(std::size_t i=0;i<input.size();++i) {
    const auto& run=input[i];
    if(run.col_begin>run.col_end || (i &&
        (run.row<input[i-1].row || (run.row==input[i-1].row &&
         (run.col_begin<=input[i-1].col_end ||
          (input[i-1].col_end!=std::numeric_limits<std::int64_t>::max() &&
           run.col_begin==std::int64_t(input[i-1].col_end)+1))))))
      throw Exception(Status::InvalidArgument,"Region::from_shared_canonical_runs","runs must be canonical");
    const auto length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
    overflow|=!length || area>std::numeric_limits<std::uint64_t>::max()-length;
    area+=length;max_length=std::max(max_length,length);
    bounds.col_min=std::min<std::int64_t>(bounds.col_min,run.col_begin);
    bounds.col_max=std::max<std::int64_t>(bounds.col_max,run.col_end);
  }
  if(overflow)throw Exception(Status::OutOfRange,"Region::from_shared_canonical_runs","region area exceeds uint64 range");
  auto storage=std::make_shared<Storage>();
  storage->owner=std::move(owner);storage->shared_runs=input;
  storage->area=area;storage->max_run_length=max_length;storage->bounds=bounds;
  return Region{std::move(storage)};
}

Region Region::from_row_bands(std::span<const RegionBandView> bands) {
  if(bands.empty())return {};
  auto storage=std::make_shared<Storage>();
  std::size_t count=0;
  std::uint64_t area=0,max_length=0;
  RectI64 bounds{bands.front().first,std::numeric_limits<std::int64_t>::max(),
                 bands.back().last,std::numeric_limits<std::int64_t>::min()};
  for(std::size_t i=0;i<bands.size();++i) {
    const auto& band=bands[i];
    (void)region_coordinate(band.first, "Region::from_row_bands");
    (void)region_coordinate(band.last, "Region::from_row_bands");
    if(band.first>band.last || band.profile.empty() || (i && bands[i-1].last>=band.first))
      throw Exception(Status::InvalidArgument,"Region::from_row_bands","invalid row bands");
    const auto rows=std::uint64_t(band.last)-std::uint64_t(band.first)+1;
    if(!rows || rows>(storage->runs.max_size()-count)/band.profile.size())
      throw Exception(Status::OutOfRange,"Region::from_row_bands","result size overflow");
    count+=static_cast<std::size_t>(rows)*band.profile.size();
    for(std::size_t j=0;j<band.profile.size();++j) {
      const auto& interval=band.profile[j];
      (void)region_coordinate(interval.first, "Region::from_row_bands");
      (void)region_coordinate(interval.last, "Region::from_row_bands");
      if(interval.first>interval.last || (j &&
          (interval.first<=band.profile[j-1].last ||
           (band.profile[j-1].last!=std::numeric_limits<std::int64_t>::max() &&
            interval.first==band.profile[j-1].last+1))))
        throw Exception(Status::InvalidArgument,"Region::from_row_bands","invalid column profile");
      const auto length=std::uint64_t(interval.last)-std::uint64_t(interval.first)+1;
      if(!length || length>(std::numeric_limits<std::uint64_t>::max()-area)/rows)
        throw Exception(Status::OutOfRange,"Region::from_row_bands","region area overflow");
      area+=length*rows;max_length=std::max(max_length,length);
      bounds.col_min=std::min(bounds.col_min,interval.first);
      bounds.col_max=std::max(bounds.col_max,interval.last);
    }
  }
  auto* buffer=storage->allocate_runs(count,"Region::from_row_bands");
  std::size_t written=0;
  for(const auto& band:bands)for(auto row=band.first;;++row) {
    for(const auto& interval:band.profile) {
      // 紧凑输入已逐项验证为 int32；展开时不再为每个重复行检查相同边界。
      RegionRun run;
      run.row=static_cast<std::int32_t>(row);
      run.col_begin=static_cast<std::int32_t>(interval.first);
      run.col_end=static_cast<std::int32_t>(interval.last);
      std::construct_at(buffer+written++,run);
    }
    if(row==band.last)break;
  }
  storage->area=area;storage->max_run_length=max_length;storage->bounds=bounds;
  storage->connected.store(bands.size()==1 && bands.front().profile.size()==1?3:0,std::memory_order_relaxed);
  return Region{std::move(storage)};
}

Region Region::from_row_masks(const Region& domain, std::span<const std::uint32_t> masks) {
  const auto invalid=[] {
    throw Exception(Status::InvalidArgument,"Region::from_row_masks","mask count or tail padding does not match domain");
  };
  auto storage=std::make_shared<Storage>();
  std::size_t index=0,count=0,previous_index=0,previous_count=0;
  const RegionRun* previous_run=nullptr;
  for(const auto& run:domain.runs()) {
    const auto length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
    const auto blocks=(length-1)/32+1;
    if(blocks>masks.size()-index)invalid();
    const auto tail=length%32;
    if(tail && (masks[index+static_cast<std::size_t>(blocks)-1]>>tail))invalid();
    const auto start=index, before=count;
    const bool repeated=previous_run && previous_run->col_begin==run.col_begin &&
        previous_run->col_end==run.col_end &&
        std::equal(masks.begin()+index,masks.begin()+index+blocks,masks.begin()+previous_index);
    if(repeated) {
      if(count>std::numeric_limits<std::size_t>::max()-previous_count)
        throw Exception(Status::OutOfRange,"Region::from_row_masks","run count exceeds size_t range");
      count+=previous_count;index+=static_cast<std::size_t>(blocks);
    } else {
    bool active=false;
    for(std::uint64_t b=0;b<blocks;b+=2) {
      std::uint64_t mask=masks[index++];
      if(b+1<blocks)mask|=std::uint64_t(masks[index++])<<32;
      const auto starts=std::popcount(mask&~((mask<<1)|std::uint64_t(active)));
      if(count>std::numeric_limits<std::size_t>::max()-starts)
        throw Exception(Status::OutOfRange,"Region::from_row_masks","run count exceeds size_t range");
      count+=starts;active=(mask>>63)!=0;
    }
    }
    previous_index=start;previous_count=count-before;previous_run=&run;
  }
  if(index!=masks.size())invalid();
  if(!count)return {};
  // 计数遍历证明精确输出容量；分配原始 persist 存储，每个平凡游程只构造一次，避免清零及逐区间容量分支。
  auto* buffer=storage->allocate_runs(count,"Region::from_row_masks");
  std::size_t written=0;
  index=0;
  std::uint64_t area=0,max_length=0;
  std::int64_t min_column=std::numeric_limits<std::int64_t>::max();
  std::int64_t max_column=std::numeric_limits<std::int64_t>::min();
  previous_run=nullptr;
  std::size_t previous_output=0;
  std::uint64_t previous_area=0;
  for(const auto& run:domain.runs()) {
    const auto length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
    const auto blocks=(length-1)/32+1;
    const auto start=index, output_start=written;
    const auto area_before=area;
    if(previous_run && previous_run->col_begin==run.col_begin &&
       previous_run->col_end==run.col_end &&
       std::equal(masks.begin()+index,masks.begin()+index+blocks,masks.begin()+previous_index)) {
      // 输出容量已精确计数，可直接复制前一游程的不可变区间并使用新行坐标。
      for(auto i=previous_output;i<output_start;++i) {
        const auto& prior=buffer[i];
        std::construct_at(buffer+written++,RegionRun{run.row,prior.col_begin,prior.col_end});
      }
      area+=previous_area;
      index+=static_cast<std::size_t>(blocks);
    } else {
    bool active=false;auto begin=std::int64_t(run.col_begin),c=begin;
    const auto emit=[&](std::int64_t end) {
      // mask 尾位已验证且输出为域内子区间，端点已由原始 int32 游程约束。
      RegionRun generated;
      generated.row=run.row;generated.col_begin=static_cast<std::int32_t>(begin);
      generated.col_end=static_cast<std::int32_t>(end);
      std::construct_at(buffer+written++,generated);
      const auto size=std::uint64_t(end)-std::uint64_t(begin)+1;
      // 输出是已验证定义域的子集，面积/长度已有上界；追加游程时元数据保持局部。
      area+=size;max_length=std::max(max_length,size);
    };
    for(std::uint64_t b=0;b<blocks;b+=2) {
      std::uint64_t mask=masks[index++];
      if(b+1<blocks)mask|=std::uint64_t(masks[index++])<<32;
      const auto previous=(mask<<1)|std::uint64_t(active);
      auto starts=mask&~previous,ends=~mask&previous;
      if(active && ends){emit(c+(std::countr_zero(ends)-1));ends&=ends-1;active=false;}
      while(starts) {
        begin=c+std::countr_zero(starts);starts&=starts-1;
        if(ends){emit(c+(std::countr_zero(ends)-1));ends&=ends-1;}
        else active=true;
      }
      if(b+2<blocks)c+=64;
    }
    if(active)emit(run.col_end);
    // 同一定义域区间内游程有序，只有首尾能扩展水平边界，重复行保持相同边界。
    if(written!=output_start) {
      min_column=std::min<std::int64_t>(min_column,buffer[output_start].col_begin);
      max_column=std::max<std::int64_t>(max_column,buffer[written-1].col_end);
    }
    }
    previous_run=&run;previous_index=start;previous_output=output_start;
    previous_area=area-area_before;
  }
  // 规范性来自定义域及位转换，同轮遍历已收集元数据，避免第三次 RLE 遍历。
  storage->area=area;storage->max_run_length=max_length;
  storage->bounds=RectI64{buffer[0].row,min_column,buffer[count-1].row,max_column};
  return Region{std::move(storage)};
}

std::vector<Region> Region::partition_runs(const Region& source,
    std::span<const std::size_t> labels,std::span<const std::size_t> counts,
    std::span<const RegionPartitionBand> bands) {
  const auto input=source.runs();
  const bool single=labels.empty() && counts.empty() && bands.empty();
  const auto invalid=[] {
    throw Exception(Status::InvalidArgument,"Region::partition_runs","invalid partition labels or counts");
  };
  if(!single) {
    if(counts.empty())invalid();
    if(bands.empty()) {if(labels.size()!=input.size())invalid();}
    else {
      std::size_t expanded=0,compact=0;
      for(const auto& band:bands) {
        if(!band.runs_per_row || !band.row_count ||
           band.runs_per_row>labels.size()-compact ||
           band.row_count>(input.size()-expanded)/band.runs_per_row)invalid();
        compact+=band.runs_per_row;expanded+=band.runs_per_row*band.row_count;
      }
      if(compact!=labels.size() || expanded!=input.size())invalid();
    }
    std::size_t sum=0;
    for(auto count:counts) {
      if(!count || count>input.size()-sum)invalid();
      sum+=count;
    }
    if(sum!=input.size())invalid();
  }
  const auto groups=single?input.size():counts.size();
  const auto count_of=[&](std::size_t i){return single?std::size_t{1}:counts[i];};
  struct Batch {
    std::unique_ptr<Storage[]> owned_objects;
    Storage* objects=nullptr;
    std::pmr::memory_resource* memory=mv::runtime::persist_resource();
    RegionRun* runs;
    std::size_t count;
    Batch(std::size_t n,std::size_t size):owned_objects(n?std::make_unique<Storage[]>(n):nullptr),objects(owned_objects.get()),count(size) {
      runs=static_cast<RegionRun*>(memory->allocate(count*sizeof(RegionRun),alignof(RegionRun)));
    }
    ~Batch(){memory->deallocate(runs,count*sizeof(RegionRun),alignof(RegionRun));}
  };
  std::vector<Region> output;
  output.reserve(groups);
  constexpr auto batch_run_limit=(32*1024)/sizeof(RegionRun);
  for(std::size_t first=0;first<groups;) {
    auto last=first+1;
    auto size=count_of(first);
    while(last<groups && last-first<128 && size<=batch_run_limit && count_of(last)<=batch_run_limit-size)
      size+=count_of(last++);
    // 元数据及共享控制块一次分配；容量保持有界，小尾块不保留完整 128 项。
    const auto allocate=[&]<std::size_t Capacity>() -> std::shared_ptr<Batch> {
      struct InlineBatch:Batch {
        Storage slots[Capacity];
        explicit InlineBatch(std::size_t n):Batch(0,n) {this->objects=slots;}
      };
      return std::make_shared<InlineBatch>(size);
    };
    const auto n=last-first;
    auto batch=!single?std::make_shared<Batch>(n,size):n<=1?allocate.template operator()<1>():
        n<=8?allocate.template operator()<8>():
        n<=32?allocate.template operator()<32>():allocate.template operator()<128>();
    std::size_t offset=0;
    for(auto i=first;i<last;++i) {
      auto& storage=batch->objects[i-first];
      storage.shared_runs={batch->runs+offset,count_of(i)};
      if(single) {
        const auto run=input[i];
        std::construct_at(batch->runs+offset,run);
        storage.area=storage.max_run_length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
        storage.bounds=RectI64{run.row,run.col_begin,run.row,run.col_end};
      }
      output.push_back(Region{std::shared_ptr<const Storage>{batch,&storage}});
      offset+=count_of(i);
    }
    first=last;
  }
  if(single)return output;
  mv::runtime::ScratchArena scratch;
  // 散布紧凑源索引，再顺序写出各输出并归约元数据，避免随机 24 字节写入后重新遍历全部游程。
  const auto gather=[&]<class Index>() {
  auto* indices=std::pmr::polymorphic_allocator<Index>{scratch.resource()}.allocate(input.size());
  struct Destination {Index* next;Index* end;};
  std::pmr::vector<Destination> destinations{scratch.resource()};
  destinations.reserve(groups);
  std::size_t offset=0;
  for(auto count:counts) {
    destinations.push_back({indices+offset,indices+offset+count});
    offset+=count;
  }
  if(bands.empty())for(std::size_t i=0;i<input.size();++i) {
    const auto group=labels[i];
    if(group>=groups)invalid();
    auto& destination=destinations[group];
    if(destination.next==destination.end)invalid();
    *destination.next++=static_cast<Index>(i);
  }
  else {
    struct BandGroup {std::size_t width=0,used=0;Index* begin=nullptr;};
    std::pmr::vector<BandGroup> band_groups(groups,scratch.resource());
    std::size_t compact=0,expanded=0;
    for(const auto& band:bands) {
      for(std::size_t j=0;j<band.runs_per_row;++j) {
        const auto group=labels[compact+j];
        if(group>=groups)invalid();
        ++band_groups[group].width;
      }
      for(std::size_t j=0;j<band.runs_per_row;++j) {
        const auto group=labels[compact+j];
        auto& part=band_groups[group];
        if(!part.used) {
          auto& destination=destinations[group];
          const auto available=static_cast<std::size_t>(destination.end-destination.next);
          if(band.row_count>available/part.width)invalid();
          part.begin=destination.next;
          destination.next+=band.row_count*part.width;
        }
        // 每个 band 预留连续组切片，即使多个区间共享分量，列仍按行优先排列。
        auto* target=part.begin+part.used++;
        auto source_index=expanded+j;
        for(std::size_t row=0;row<band.row_count;++row) {
          *target=static_cast<Index>(source_index);
          source_index+=band.runs_per_row;
          if(row+1<band.row_count)target+=part.width;
        }
      }
      for(std::size_t j=0;j<band.runs_per_row;++j) {
        auto& part=band_groups[labels[compact+j]];part.width=0;part.used=0;
      }
      compact+=band.runs_per_row;expanded+=band.runs_per_row*band.row_count;
    }
  }
  offset=0;
  for(auto& region:output) {
    auto& storage=*const_cast<Storage*>(region.storage_.get());
    const auto runs=storage.shared_runs;
    auto* target=const_cast<RegionRun*>(runs.data());
    std::uint64_t area=0,max_length=0;
    auto first=std::numeric_limits<std::int64_t>::max();
    auto last=std::numeric_limits<std::int64_t>::min();
    for(std::size_t j=0;j<runs.size();++j) {
      const auto run=input[indices[offset+j]];
      std::construct_at(target+j,run);
      const auto length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
      area+=length;max_length=std::max(max_length,length);
      first=std::min<std::int64_t>(first,run.col_begin);last=std::max<std::int64_t>(last,run.col_end);
    }
    offset+=runs.size();
    storage.area=area;storage.max_run_length=max_length;
    storage.bounds=RectI64{runs.front().row,first,runs.back().row,last};
  }
  };
  if(input.size()<=std::numeric_limits<std::uint32_t>::max())gather.template operator()<std::uint32_t>();
  else gather.template operator()<std::size_t>();
  // 总计数等于 input.size() 且各组不超自身计数，因此全部输出跨度均完整初始化。
  return output;
}

bool Region::shares_storage_owner(const Region& other) const noexcept {
  return !storage_.owner_before(other.storage_) && !other.storage_.owner_before(storage_);
}

std::uint64_t Region::max_run_length() const noexcept { return storage_ ? storage_->max_run_length : 0; }

bool Region::is_rectangle() const noexcept {
  if(!storage_)return false;
  const auto& b=storage_->bounds;
  const auto width=std::uint64_t(b.col_max-b.col_min)+1;
  const auto height=std::uint64_t(b.row_max-b.row_min)+1;
  return storage_->area/width==height && storage_->area%width==0;
}

bool Region::known_connected4() const noexcept {return storage_ && (storage_->connected.load(std::memory_order_relaxed)&1);}
bool Region::known_connected8() const noexcept {return storage_ && (storage_->connected.load(std::memory_order_relaxed)&2);}
void Region::record_connected_component(bool four_connected) const noexcept {
  if(storage_)storage_->connected.fetch_or(four_connected?3:2,std::memory_order_relaxed);
}

Region Region::rectangle(RectI64 rect) {
  (void)region_coordinate(rect.row_min, "Region::rectangle");
  (void)region_coordinate(rect.row_max, "Region::rectangle");
  (void)region_coordinate(rect.col_min, "Region::rectangle");
  (void)region_coordinate(rect.col_max, "Region::rectangle");
  if (rect.row_min > rect.row_max || rect.col_min > rect.col_max) return {};
  // 先检查计数与面积；正常矩形无需构造 4 KiB scratch 工作区。
  const std::pmr::vector<RegionRun> capacity_check{mv::runtime::persist_resource()};
  const auto row_count_u = static_cast<std::uint64_t>(rect.row_max) - static_cast<std::uint64_t>(rect.row_min) + 1u;
  if (row_count_u > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    throw Exception(Status::OutOfRange, "Region::rectangle", "row count exceeds size_t");
  }
  const auto width = static_cast<std::uint64_t>(rect.col_max) -
                     static_cast<std::uint64_t>(rect.col_min) + 1u;
  // 不确定时保留原 reserve/area 错误；通过检查后直接构造矩形，不重复验证行带。
  if (row_count_u && width && row_count_u <= capacity_check.max_size() &&
      row_count_u <= std::numeric_limits<std::uint64_t>::max() / width) {
    // 四个端点已验证，矩形天然规范；直接拥有同样的 persist 游程及精确元数据。
    auto storage=std::make_shared<Storage>();
    const auto count=static_cast<std::size_t>(row_count_u);
    auto* buffer=storage->allocate_runs(count,"Region::from_row_bands");
    const auto first_row=static_cast<std::int32_t>(rect.row_min);
    const auto first_column=static_cast<std::int32_t>(rect.col_min);
    const auto last_column=static_cast<std::int32_t>(rect.col_max);
    for(std::size_t i=0;i<count;++i) {
      RegionRun run;
      run.row=static_cast<std::int32_t>(std::int64_t(first_row)+std::int64_t(i));
      run.col_begin=first_column;run.col_end=last_column;
      std::construct_at(buffer+i,run);
    }
    storage->area=row_count_u*width;storage->max_run_length=width;storage->bounds=rect;
    storage->connected.store(3,std::memory_order_relaxed);
    return Region{std::move(storage)};
  }
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<RegionRun> runs{scratch.resource()};
  runs.reserve(static_cast<std::size_t>(row_count_u));
  for (std::int64_t row = rect.row_min;; ++row) {
    runs.push_back({row, rect.col_min, rect.col_max});
    if (row == rect.row_max) break;
  }
  return from_runs(runs);
}

std::vector<Region> Region::rectangles(std::span<const RectI64> boxes) {
  constexpr std::size_t run_limit = (32 * 1024) / sizeof(RegionRun);
  const auto count_of = [](const RectI64& box) -> std::uint64_t {
    (void)region_coordinate(box.row_min, "Region::rectangle");
    (void)region_coordinate(box.row_max, "Region::rectangle");
    (void)region_coordinate(box.col_min, "Region::rectangle");
    (void)region_coordinate(box.col_max, "Region::rectangle");
    if (box.row_min > box.row_max || box.col_min > box.col_max) return 0;
    return static_cast<std::uint64_t>(box.row_max - box.row_min) + 1;
  };
  std::vector<Region> output;
  output.reserve(boxes.size());
  // 与连通分量相同的别名所有权：保留任意单元不会延长整幅分区的生命周期。
  struct Batch {
    std::pmr::memory_resource* memory = mv::runtime::persist_resource();
    RegionRun* runs;
    std::size_t count;
    Storage* objects = nullptr;
    explicit Batch(std::size_t size) : count(size) {
      runs = static_cast<RegionRun*>(memory->allocate(count * sizeof(RegionRun), alignof(RegionRun)));
    }
    ~Batch() { memory->deallocate(runs, count * sizeof(RegionRun), alignof(RegionRun)); }
  };
  for (std::size_t first = 0; first < boxes.size();) {
    auto size = count_of(boxes[first]);
    if (!size || size > run_limit) {
      output.push_back(rectangle(boxes[first++]));
      continue;
    }
    auto last = first + 1;
    while (last < boxes.size() && last - first < 128) {
      const auto next = count_of(boxes[last]);
      if (!next || next > run_limit - size) break;
      size += next;
      ++last;
    }
    // 小尾块缩小元数据容量；游程只分配实际数量，且每块面积计算不可能溢出 UInt64。
    const auto allocate = [&]<std::size_t Capacity>() -> std::shared_ptr<Batch> {
      struct InlineBatch : Batch {
        Storage slots[Capacity];
        explicit InlineBatch(std::size_t count) : Batch(count) { this->objects = slots; }
      };
      return std::make_shared<InlineBatch>(static_cast<std::size_t>(size));
    };
    const auto n = last - first;
    auto batch = n <= 1 ? allocate.template operator()<1>() :
                 n <= 8 ? allocate.template operator()<8>() :
                 n <= 32 ? allocate.template operator()<32>() : allocate.template operator()<128>();
    std::size_t offset = 0;
    for (auto i = first; i < last; ++i) {
      const auto box = boxes[i];
      const auto rows = static_cast<std::size_t>(box.row_max - box.row_min + 1);
      const auto width = static_cast<std::uint64_t>(box.col_max - box.col_min) + 1;
      auto& storage = batch->objects[i - first];
      storage.shared_runs = {batch->runs + offset, rows};
      storage.area = rows * width;
      storage.max_run_length = width;
      storage.bounds = box;
      storage.connected.store(3, std::memory_order_relaxed);
      for (std::size_t row = 0; row < rows; ++row) {
        RegionRun run;
        run.row = static_cast<std::int32_t>(box.row_min + static_cast<std::int64_t>(row));
        run.col_begin = static_cast<std::int32_t>(box.col_min);
        run.col_end = static_cast<std::int32_t>(box.col_max);
        std::construct_at(batch->runs + offset + row, run);
      }
      output.push_back(Region{std::shared_ptr<const Storage>{batch, &storage}});
      offset += rows;
    }
    first = last;
  }
  return output;
}

namespace {
// 规范游程按行及每行的 begin/end 递增；定位首个可能重叠项时避免坐标算术溢出。
std::size_t seek_run_overlap(std::span<const RegionRun> runs, std::size_t cursor,
                             const RegionRun& query) {
  const auto before=[](const RegionRun& run,const RegionRun& key) {
    return run.row<key.row || (run.row==key.row && run.col_end<key.col_begin);
  };
  if (cursor==runs.size() || !before(runs[cursor],query)) return cursor;
  return static_cast<std::size_t>(std::lower_bound(runs.begin()+cursor,runs.end(),
      query,before)-runs.begin());
}
}

Region Region::intersect(const Region& other) const {
  if (empty() || other.empty()) return {};
  if (storage_ == other.storage_) return *this;
  const auto& ab=storage_->bounds;const auto& bb=other.storage_->bounds;
  const auto covers=[](const RectI64& a,const RectI64& b) {
    return a.row_min<=b.row_min && a.row_max>=b.row_max && a.col_min<=b.col_min && a.col_max>=b.col_max;
  };
  // 面积与包围盒精确证明矩形，覆盖关系直接保留不可变源，部分相交仅生成一次矩形。
  const bool ar=is_rectangle(),br=other.is_rectangle();
  if(ar && covers(ab,bb))return other;
  if(br && covers(bb,ab))return *this;
  if(ar && br)return rectangle({std::max(ab.row_min,bb.row_min),std::max(ab.col_min,bb.col_min),
                                std::min(ab.row_max,bb.row_max),std::min(ab.col_max,bb.col_max)});
  if (area() == other.area() && bounding_box() == other.bounding_box() &&
      runs().size() == other.runs().size() &&
      std::equal(runs().begin(), runs().end(), other.runs().begin())) return *this;
  std::pmr::vector<RegionRun> result{mv::runtime::persist_resource()};
  const auto a = runs();
  const auto b = other.runs();
  result.reserve(std::min(a.size(),b.size()));
  std::uint64_t result_area=0,result_max_length=0;
  std::int32_t min_column=INT32_MAX,max_column=INT32_MIN;
  std::size_t i = 0;
  std::size_t j = 0;
  const bool seek_b=a.size()<b.size()/16;
  const bool seek_a=b.size()<a.size()/16;
  while (i < a.size() && j < b.size()) {
    if (seek_b) j=seek_run_overlap(b,j,a[i]);
    else if (seek_a) i=seek_run_overlap(a,i,b[j]);
    if (i==a.size() || j==b.size()) break;
    if (a[i].row < b[j].row) { ++i; continue; }
    if (b[j].row < a[i].row) { ++j; continue; }
    const auto begin = std::max(a[i].col_begin, b[j].col_begin);
    const auto end = std::min(a[i].col_end, b[j].col_end);
    if (begin <= end) {
      result.push_back({a[i].row, begin, end});
      const auto length=std::uint64_t(std::int64_t(end)-begin)+1;
      result_area+=length;result_max_length=std::max(result_max_length,length);
      min_column=std::min(min_column,begin);max_column=std::max(max_column,end);
    }
    if (a[i].col_end < b[j].col_end) ++i;
    else ++j;
  }
  if(result.empty())return {};
  // 规范输入的交集仍规范且面积不超过任一输入；生成时同步汇总，无需二次验证扫描。
  auto storage=std::make_shared<Storage>();
  storage->bounds={result.front().row,min_column,result.back().row,max_column};
  storage->area=result_area;storage->max_run_length=result_max_length;
  storage->runs=std::move(result);
  return Region{std::move(storage)};
}


Region Region::united(const Region& other) const {
  struct FlatLayout { std::pmr::vector<RegionRun> runs;std::shared_ptr<const void> owner;
    std::span<const RegionRun> shared_runs;std::uint64_t area,max_run_length;RectI64 bounds;bool owns_runs; };
  static_assert(sizeof(Storage)==sizeof(FlatLayout),"segmented unions must not grow flat region records");

  if (empty()) return other;
  if (other.empty()) return *this;
  const Region* base = this;
  const Region* addition = &other;
  if (base->runs().size() < addition->runs().size()) std::swap(base, addition);
  if(!base->storage_->segmented && base->runs().size()>=256 && addition->runs().size()<=128 &&
      addition->runs().size()<base->runs().size()/16) {
    const auto input=base->runs(),added=addition->runs();
    bool one_per_row=true;for(std::size_t i=1;i<added.size();++i)one_per_row=one_per_row && added[i-1].row!=added[i].row;
    if(one_per_row) {
      auto fragments=std::make_shared<Storage::Segments>();fragments->owner=base->storage_;
      fragments->pieces.reserve(2*added.size()+1);
      std::pmr::vector<RegionRun> middle{mv::runtime::persist_resource()};middle.reserve(added.size());
      std::size_t cursor=0;std::uint64_t removed_area=0;
      for(const auto& run:added) {
        const auto first=std::lower_bound(input.begin()+cursor,input.end(),run,[](const auto& old,const auto& value) {
          return old.row<value.row || (old.row==value.row && old.col_end<value.col_begin && std::int64_t(old.col_end)+1<value.col_begin);
        });
        auto last=first;RegionRun merged=run;
        while(last!=input.end() && last->row==run.row &&
              (last->col_begin<=run.col_end || (run.col_end!=INT64_MAX && last->col_begin==std::int64_t(run.col_end)+1))) {
          merged.col_begin=std::min<std::int64_t>(merged.col_begin,last->col_begin);merged.col_end=std::max<std::int64_t>(merged.col_end,last->col_end);
          removed_area+=std::uint64_t(last->col_end)-std::uint64_t(last->col_begin)+1;++last;
        }
        const auto index=std::size_t(first-input.begin());
        if(index>cursor)fragments->pieces.push_back(input.subspan(cursor,index-cursor));
        fragments->pieces.push_back({}); // 建立不可变中段所有权后填充。
        middle.push_back(merged);cursor=std::size_t(last-input.begin());
      }
      if(cursor<input.size())fragments->pieces.push_back(input.subspan(cursor));
      const auto replacement=Region::from_canonical_runs(std::move(middle));
      fragments->middle_owner=replacement.storage_;const auto replacement_runs=replacement.runs();
      std::size_t index=0;for(auto& piece:fragments->pieces)if(piece.empty())piece=replacement_runs.subspan(index++,1);
      const auto retained=base->area()-removed_area;
      if(replacement.area()>UINT64_MAX-retained)throw Exception(Status::OutOfRange,"Region::united","region area exceeds uint64 range");
      auto storage=std::make_shared<Storage>();storage->area=retained+replacement.area();
      storage->max_run_length=std::max(base->max_run_length(),replacement.max_run_length());
      const auto a=*base->bounding_box(),b=*addition->bounding_box();
      storage->bounds={std::min(a.row_min,b.row_min),std::min(a.col_min,b.col_min),std::max(a.row_max,b.row_max),std::max(a.col_max,b.col_max)};
      storage->owner=std::move(fragments);storage->segmented=true;return Region{std::move(storage)};
    }
  }
  if (base->runs().size() >= 256 && addition->runs().size() < base->runs().size() / 16) {
    const auto input = base->runs();
    const auto box = *addition->bounding_box();
    const auto first = std::lower_bound(input.begin(), input.end(), box.row_min,
        [](const RegionRun& run, std::int64_t row) { return run.row < row; });
    const auto last = std::upper_bound(first, input.end(), box.row_max,
        [](std::int64_t row, const RegionRun& run) { return row < run.row; });
    // 仅合并较小输入涉及的行并更新元数据，行范围覆盖整个基底时避免递归特化。
    if (first != input.begin() || last != input.end()) {
      std::pmr::vector<RegionRun> affected(first, last, mv::runtime::persist_resource());
      const auto previous = Region::from_canonical_runs(std::move(affected));
      const auto merged = previous.united(*addition);
      const auto middle = merged.runs();
      const auto prefix = static_cast<std::size_t>(first - input.begin());
      const auto suffix = static_cast<std::size_t>(input.end() - last);
      auto storage = std::make_shared<Storage>();
      const auto retained_area = base->area() - previous.area();
      if (merged.area() > std::numeric_limits<std::uint64_t>::max() - retained_area)
        throw Exception(Status::OutOfRange, "Region::united", "region area exceeds uint64 range");
      if (middle.size() > storage->runs.max_size() - prefix - suffix)
        throw Exception(Status::OutOfRange, "Region::united", "run count exceeds allocation range");
      // 并集已完整计算且规范，保留不可变未变片段，避免逐结果复制大型公共输入；连续游程查询只安全物化一次。
      if(!base->storage_->segmented) {
        auto fragments=std::make_shared<Storage::Segments>();
        fragments->owner=base->storage_;fragments->middle_owner=merged.storage_;
        if(prefix)fragments->pieces.push_back(input.first(prefix));
        if(!middle.empty())fragments->pieces.push_back(middle);
        if(suffix)fragments->pieces.push_back(input.last(suffix));
        storage->owner=std::move(fragments);storage->segmented=true;
      } else {
        // 限制重复并集的 persist 所有权深度；此前分段输入已由上方 runs() 物化。
        auto* output=storage->allocate_runs(prefix+middle.size()+suffix,"Region::united");
        if(prefix)std::memcpy(output,input.data(),prefix*sizeof(RegionRun));
        if(!middle.empty())std::memcpy(output+prefix,middle.data(),middle.size_bytes());
        if(suffix)std::memcpy(output+prefix+middle.size(),input.data()+input.size()-suffix,suffix*sizeof(RegionRun));
      }
      storage->area = retained_area + merged.area();
      storage->max_run_length = std::max(base->max_run_length(), merged.max_run_length());
      storage->bounds = *base->bounding_box();
      storage->bounds.row_min = std::min(storage->bounds.row_min, box.row_min);
      storage->bounds.row_max = std::max(storage->bounds.row_max, box.row_max);
      storage->bounds.col_min = std::min(storage->bounds.col_min, box.col_min);
      storage->bounds.col_max = std::max(storage->bounds.col_max, box.col_max);
      return Region{std::move(storage)};
    }
  }
  std::pmr::vector<RegionRun> result{mv::runtime::persist_resource()};
  result.reserve(runs().size() + other.runs().size());
  const auto a=runs(), b=other.runs();
  std::size_t i=0,j=0;
  while(i<a.size() || j<b.size()) {
    const auto& run = j==b.size() || (i<a.size() &&
        (a[i].row<b[j].row || (a[i].row==b[j].row && a[i].col_begin<=b[j].col_begin)))
        ? a[i++] : b[j++];
    if(!result.empty() && result.back().row==run.row &&
       (run.col_begin<=result.back().col_end ||
        (result.back().col_end!=std::numeric_limits<std::int64_t>::max() &&
         run.col_begin==std::int64_t(result.back().col_end)+1)))
      result.back().col_end=std::max<std::int64_t>(result.back().col_end,run.col_end);
    else result.push_back(run);
  }
  return from_canonical_runs(std::move(result));
}

Region Region::difference(const Region& other) const {
  if (empty() || other.empty()) return *this;
  if(is_rectangle() && other.is_rectangle()) {
    const auto& a=storage_->bounds;const auto& b=other.storage_->bounds;
    const auto top=std::max(a.row_min,b.row_min),bottom=std::min(a.row_max,b.row_max);
    const auto left=std::max(a.col_min,b.col_min),right=std::min(a.col_max,b.col_max);
    if(top>bottom || left>right)return *this;
    RegionBandView bands[3];std::size_t count=0;
    const RegionInterval full{a.col_min,a.col_max};
    RegionInterval middle[2];std::size_t columns=0;
    if(a.col_min<left)middle[columns++]={a.col_min,left-1};
    if(right<a.col_max)middle[columns++]={right+1,a.col_max};
    const bool has_top=a.row_min<top,has_bottom=bottom<a.row_max;
    if(has_top)bands[count++]={a.row_min,top-1,{&full,1}};
    if(columns)bands[count++]={top,bottom,{middle,columns}};
    if(has_bottom)bands[count++]={bottom+1,a.row_max,{&full,1}};
    auto result=from_row_bands({bands,count});
    if(!result.empty()) {
      // 两条侧边由顶部或底部横带连通；无侧边时上下两块不可误判为一块。
      const bool connected=columns==2 ? has_top || has_bottom : !(columns==0 && has_top && has_bottom);
      if(connected)result.record_connected_component(true);
    }
    return result;
  }
  std::pmr::vector<RegionRun> result{mv::runtime::persist_resource()};
  const auto a = runs(), b = other.runs();
  result.reserve(a.size());
  // 生产者输出已验证 Region 的有序不相交子集，长度及总和由 this->area() 约束，
  // UInt64 元数据不会溢出；公开规范输入验证保持原契约。
  std::uint64_t result_area=0,result_max_length=0;
  std::int64_t col_min=std::numeric_limits<std::int64_t>::max();
  std::int64_t col_max=std::numeric_limits<std::int64_t>::min();
  const auto emit=[&](const RegionRun& run) {
    const auto length=std::uint64_t(run.col_end)-std::uint64_t(run.col_begin)+1;
    result_area+=length;result_max_length=std::max(result_max_length,length);
    col_min=std::min<std::int64_t>(col_min,run.col_begin);col_max=std::max<std::int64_t>(col_max,run.col_end);
    result.push_back(run);
  };
  std::size_t j = 0;
  const bool seek_b=a.size()<b.size()/16;
  for (const auto& ar : a) {
    if (seek_b) j=seek_run_overlap(b,j,ar);
    while (j < b.size() && b[j].row < ar.row) ++j;
    auto current = std::int64_t(ar.col_begin);
    bool exhausted = false;
    auto k = j;
    for (; k < b.size() && b[k].row == ar.row; ++k) {
      const auto& br = b[k];
      if (br.col_end < current) continue;
      if (br.col_begin > ar.col_end) break;
      if (br.col_begin > current)
        emit({ar.row, current, std::int64_t(br.col_begin) - 1});
      if (br.col_end >= ar.col_end) { exhausted = true; break; }
      current = std::int64_t(br.col_end) + 1;
    }
    // k 为首个未来区间或仍覆盖当前查询终点的区间，保留以供下一次规范查询使用。
    j = k;
    if (!exhausted) emit({ar.row, current, ar.col_end});
  }
  if(result.empty())return {};
  auto storage=std::make_shared<Storage>();
  storage->bounds={result.front().row,col_min,result.back().row,col_max};
  storage->area=result_area;storage->max_run_length=result_max_length;
  storage->runs=std::move(result);
  return Region{std::move(storage)};
}

Region Region::shifted(std::int64_t row_delta, std::int64_t col_delta) const {
  (void)region_coordinate(row_delta, "Region::shifted");
  (void)region_coordinate(col_delta, "Region::shifted");
  if (empty()) return {};
  auto checked_add = [](std::int64_t value, std::int64_t delta, const char*) {
    return region_coordinate(value + delta, "Region::shifted");
  };
  // 平移保持规范次序及全部尺寸元数据；极值检查一次即可证明后续加法安全，包含负偏移。
  const auto bounds=storage_->bounds;
  const RectI64 shifted_bounds{
      checked_add(bounds.row_min,row_delta,"row shift exceeds int32 range"),
      checked_add(bounds.col_min,col_delta,"column shift exceeds int32 range"),
      checked_add(bounds.row_max,row_delta,"row shift exceeds int32 range"),
      checked_add(bounds.col_max,col_delta,"column shift exceeds int32 range")};
  auto storage=std::make_shared<Storage>();
  const auto input=runs();
  auto* output=storage->allocate_runs(input.size(),"Region::shifted");
  for (const auto& run : input) {
    // 边界已证明每个坐标可表示，精确容量省去逐项范围检查和 vector 扩容分支。
    RegionRun shifted;
    shifted.row=static_cast<std::int32_t>(std::int64_t(run.row)+row_delta);
    shifted.col_begin=static_cast<std::int32_t>(std::int64_t(run.col_begin)+col_delta);
    shifted.col_end=static_cast<std::int32_t>(std::int64_t(run.col_end)+col_delta);
    std::construct_at(output++,shifted);
  }
  storage->area=storage_->area;
  storage->connected.store(storage_->connected.load(std::memory_order_relaxed),std::memory_order_relaxed);
  storage->max_run_length=storage_->max_run_length;
  storage->bounds=shifted_bounds;
  return Region{std::move(storage)};
}

bool Region::contains(std::int64_t row, std::int64_t column) const noexcept {
  const auto contains_run = [&](std::span<const RegionRun> values) {
    // 每行规范不相交游程的 col_end 递增，单次 (row,col_end) 字典序搜索定位唯一可能包含点的游程，无需 row+1。
    const auto hit = std::lower_bound(
        values.begin(), values.end(), row, [column](const RegionRun& run, std::int64_t target_row) {
          return run.row < target_row || (run.row == target_row && run.col_end < column);
        });
    return hit != values.end() && hit->row == row && hit->col_begin <= column;
  };
  if (!storage_) return false;
  if (const auto* fragments = storage_->segment_data()) {
    for (const auto piece : fragments->pieces)
      if (row >= piece.front().row && row <= piece.back().row && contains_run(piece)) return true;
    return false;
  }
  return contains_run(storage_->shared_runs.empty() ? std::span<const RegionRun>{storage_->runs}
                                                    : storage_->shared_runs);
}

Region Region::clipped(RectI64 rect) const {
  (void)region_coordinate(rect.row_min, "Region::clipped");
  (void)region_coordinate(rect.row_max, "Region::clipped");
  (void)region_coordinate(rect.col_min, "Region::clipped");
  (void)region_coordinate(rect.col_max, "Region::clipped");
  if (rect.row_min > rect.row_max || rect.col_min > rect.col_max) return {};
  const auto rows = std::uint64_t(rect.row_max) - std::uint64_t(rect.row_min) + 1;
  const auto width = std::uint64_t(rect.col_max) - std::uint64_t(rect.col_min) + 1;
  std::pmr::vector<RegionRun> result{mv::runtime::persist_resource()};
  // 保留 rectangle 的计数/面积错误，源为空也如此。
  if (!rows || !width || rows > result.max_size() || rows > UINT64_MAX / width)
    return intersect(rectangle(rect));
  if (empty()) return {};
  const auto& box = storage_->bounds;
  if (rect.row_min <= box.row_min && rect.row_max >= box.row_max &&
      rect.col_min <= box.col_min && rect.col_max >= box.col_max) return *this;
  if (rect.row_max < box.row_min || rect.row_min > box.row_max ||
      rect.col_max < box.col_min || rect.col_min > box.col_max) return {};
  const auto input = runs();
  const auto first = std::lower_bound(input.begin(),input.end(),rect.row_min,
      [](const RegionRun& run,std::int64_t row) {return run.row < row;});
  const auto last = std::upper_bound(first,input.end(),rect.row_max,
      [](std::int64_t row,const RegionRun& run) {return row < run.row;});
  // 列片段较多时使用既有规范定位，避免为每个窄分块扫描全部片段。
  if (rows < input.size()/16) {
    result.reserve(std::min(static_cast<std::size_t>(last-first),static_cast<std::size_t>(rows)));
    auto cursor = static_cast<std::size_t>(first-input.begin());
    const auto limit = static_cast<std::size_t>(last-input.begin());
    while (cursor < limit) {
      const auto row = input[cursor].row;
      cursor = seek_run_overlap(input,cursor,{row,rect.col_min,rect.col_max});
      if (cursor >= limit) break;
      if (input[cursor].row != row) continue;
      while (cursor < limit && input[cursor].row == row &&
             input[cursor].col_begin <= rect.col_max) {
        const auto& run = input[cursor++];
        result.push_back({row,std::max<std::int64_t>(run.col_begin,rect.col_min),
                             std::min<std::int64_t>(run.col_end,rect.col_max)});
      }
      cursor = static_cast<std::size_t>(std::upper_bound(input.begin()+cursor,
          input.begin()+limit,row,[](std::int64_t key,const RegionRun& run) {
            return key < run.row;
          })-input.begin());
    }
    return from_canonical_runs(std::move(result));
  }
  result.reserve(static_cast<std::size_t>(last-first));
  for (auto it = first; it != last; ++it) {
    const auto begin = std::max<std::int64_t>(it->col_begin,rect.col_min);
    const auto end = std::min<std::int64_t>(it->col_end,rect.col_max);
    if (begin <= end) result.push_back({it->row,begin,end});
  }
  return from_canonical_runs(std::move(result));
}

bool Region::empty() const noexcept {
  return !storage_ || storage_->area==0;
}

std::span<const RegionRun> Region::runs() const {
  if (!storage_) return {};
  storage_->materialize();
  if (!storage_->shared_runs.empty()) return storage_->shared_runs;
  return storage_->runs;
}

std::size_t Region::run_count() const noexcept {
  if(!storage_)return 0;
  if(const auto* fragments=storage_->segment_data()) {
    std::size_t count=0;for(const auto piece:fragments->pieces)count+=piece.size();return count;
  }
  return storage_->shared_runs.empty() ? storage_->runs.size() : storage_->shared_runs.size();
}

void Region::copy_run_bytes(std::span<std::byte> destination) const {
  const auto count=run_count();
  if(count>SIZE_MAX/sizeof(RegionRun) || destination.size()!=count*sizeof(RegionRun))
    throw Exception(Status::InvalidArgument,"Region::copy_run_bytes","destination must fit all run records");
  if(!count)return;
  bool streaming=false;
#if MVKIT_CORE_HAS_AVX2
  if(destination.size()>=65536) {
    const auto& cpu=runtime::cpu_features();
    streaming=runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,true,false,false});
  }
#endif
  const auto copy=[&](std::byte* output,std::span<const RegionRun> input) {
#if MVKIT_CORE_HAS_AVX2
    if(streaming && input.size_bytes()>=4096) {
      stream_run_bytes_avx2(output,reinterpret_cast<const std::byte*>(input.data()),input.size_bytes());return;
    }
#endif
    std::memcpy(output,input.data(),input.size_bytes());
  };
  if(const auto* fragments=storage_->segment_data()) {
    auto* output=destination.data();
    for(const auto piece:fragments->pieces){copy(output,piece);output+=piece.size_bytes();}
  } else {
    copy(destination.data(),runs());
  }
}

std::uint64_t Region::area() const noexcept {
  return storage_ ? storage_->area : 0;
}

std::optional<RectI64> Region::bounding_box() const noexcept {
  if(!storage_)return std::nullopt;
  return storage_->bounds;
}

bool Region::payload_uses_persist_resource() const noexcept {
  return !storage_ ||
         storage_->runs.get_allocator().resource() ==
             mv::runtime::persist_resource();
}

}  // namespace mv::core
