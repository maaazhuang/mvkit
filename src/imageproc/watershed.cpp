#include <imageproc/advanced.hpp>
#include <core/iconic_tuple.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <numeric>
#include <cassert>
#include <type_traits>

namespace mv::imageproc {
namespace {
using namespace mv::core;
constexpr auto operation = "watersheds_marker";
constexpr std::uint32_t none = UINT32_MAX;
// 节点高度 minimax 代价为 max(parent_cost,gray)，后出队父节点代价不会更小，因此首次发现即最终值。
// 每像素只入队一次：整数地形使用 FIFO 桶，real/int32 使用稳定 FIFO 堆。
// 未见 gray 原位改为精确最终代价，整数 FIFO 链接同时标记发现。链接为有效索引或 UINT32_MAX；
// UINT32_MAX-1 专用于未见像素，不可能是预算内像素索引。堆代价/时间戳入队后不变，时间戳在 UInt32 内。
template<class Cost> class MonotoneFloodQueue {
 public:
  static constexpr bool bucket_queue = std::is_integral_v<Cost> && sizeof(Cost) <= 2;
  using State = std::conditional_t<bucket_queue, std::uint32_t, std::uint8_t>;
  static constexpr State unseen = bucket_queue ? UINT32_MAX - 1 : 1;
  static constexpr State outside = bucket_queue ? UINT32_MAX : 2;
  MonotoneFloodQueue(std::pmr::vector<Cost>& levels, std::pmr::vector<State>& positions,
             std::size_t buckets, double offset, std::pmr::memory_resource* memory)
      : levels_(levels), positions_(positions), offset_(offset), heap_(memory),
        order_(memory), heads_(memory), tails_(memory) {
    if constexpr (bucket_queue) {
      heads_.assign(buckets, none); tails_.assign(buckets, none);
      current_ = buckets;
    } else { heap_.reserve(levels.size()); order_.resize(levels.size()); }
  }
  bool empty() const { return size_ == 0; }
  void push(std::uint32_t pixel, Cost level) {
    assert(positions_[pixel] == unseen);
    levels_[pixel] = level;
    if constexpr (bucket_queue) {
      const auto bucket = static_cast<std::size_t>(level - offset_);
      positions_[pixel] = none;
      if (tails_[bucket] != none) positions_[tails_[bucket]] = pixel;
      else heads_[bucket] = pixel;
      tails_[bucket] = pixel;
      current_ = std::min(current_, bucket);
    } else {
      positions_[pixel] = 0;
      order_[pixel] = sequence_++;
      heap_.push_back(pixel);
      auto i = heap_.size() - 1;
      while (i && less(heap_[i], heap_[(i - 1) / 2])) {
        std::swap(heap_[i], heap_[(i - 1) / 2]);
        i = (i - 1) / 2;
      }
    }
    ++size_;
  }
  std::uint32_t pop() {
    std::uint32_t pixel;
    if constexpr (bucket_queue) {
      while (heads_[current_] == none) ++current_;
      pixel = heads_[current_];
      heads_[current_] = positions_[pixel];
      if (heads_[current_] == none) tails_[current_] = none;
    } else {
      pixel = heap_.front();
      heap_.front() = heap_.back();
      heap_.pop_back();
      if (!heap_.empty()) down(0);
    }
    --size_; return pixel;
  }
 private:
  bool less(std::uint32_t a, std::uint32_t b) const {
    return levels_[a] < levels_[b] || (levels_[a] == levels_[b] && order_[a] < order_[b]);
  }
  void down(std::size_t i) {
    for (;;) {
      auto child = i * 2 + 1;
      if (child >= heap_.size()) break;
      if (child + 1 < heap_.size() && less(heap_[child + 1], heap_[child])) ++child;
      if (!less(heap_[child], heap_[i])) break;
      std::swap(heap_[child], heap_[i]); i = child;
    }
  }
  std::pmr::vector<Cost>& levels_;
  std::pmr::vector<State>& positions_;
  double offset_;
  std::size_t current_{}, size_{};
  std::uint32_t sequence_{};
  std::pmr::vector<std::uint32_t> heap_;
  // 带 padding 的定义域预算保证首次发现数量小于 2^24。
  std::pmr::vector<std::uint32_t> order_;
  std::pmr::vector<std::uint32_t> heads_, tails_;
};
template<class T> Regions flood_t(const Image& image,const Regions& markers,int connectivity) {
  advanced::check(!image.empty() && image.channel_count()==1,operation,"Image","one channel required");
  advanced::check(markers.size()<=65535,operation,"Markers","at most 65535 marker objects");
  const auto domain=image.domain();const auto box=domain.bounding_box();
  if(!box || markers.empty())return Regions{std::vector<Region>(markers.size())};
  const auto width=static_cast<std::size_t>(box->col_max-box->col_min+1);
  const auto height=static_cast<std::size_t>(box->row_max-box->row_min+1);
  advanced::check(width<=16777216/height,operation,"Domain","domain bounding box budget is 16777216 pixels");
  const auto pitch=width+2;
  advanced::check(pitch<=16777216/(height+2),operation,"Domain","padded domain bounding box exceeds pixel budget");
  const auto count=pitch*(height+2);
  runtime::ScratchArena scratch;auto* memory=scratch.resource();
  // minimax 代价就是输入灰度；原标量类型精确保留值，减少整数工作集带宽，不量化。
  std::pmr::vector<T> gray(count,T{},memory);
  using Queue = MonotoneFloodQueue<T>;
  std::pmr::vector<typename Queue::State> position(count,Queue::outside,memory);
  std::pmr::vector<std::uint32_t> label(count,0,memory);
  const auto index=[&](std::int64_t r,std::int64_t c){return static_cast<std::uint32_t>((r-box->row_min+1)*pitch+c-box->col_min+1);};
    for(const auto& run:domain.runs()){
      const auto* row=static_cast<const std::byte*>(image.data())+std::int64_t(run.row)*image.stride();
      for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c){T value;std::memcpy(&value,row+c*sizeof(T),sizeof(T));
        const auto i=index(run.row,c);gray[i]=value;position[i]=Queue::unseen;
        advanced::check(std::isfinite(double(gray[i])),operation,"Image","finite domain pixels required");}
    }
  for(std::size_t m=0;m<markers.size();++m){
    const auto clipped=markers[m].intersect(domain);
    for(const auto& run:clipped.runs())for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c)label[index(run.row,c)]=static_cast<std::uint32_t>(m+1);
  }
  const auto type=image.pixel_type();
  const std::size_t buckets=type==PixelType::UInt8?256:
    type==PixelType::UInt16 || type==PixelType::Int16?65536:0;
  MonotoneFloodQueue<T> queue(gray,position,buckets,type==PixelType::Int16?-32768:0,memory);
  for(const auto& run:domain.runs())for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c){const auto i=index(run.row,c);if(label[i])queue.push(i,gray[i]);}
  const std::int64_t step[]{-static_cast<std::int64_t>(pitch),-1,1,static_cast<std::int64_t>(pitch),
    -static_cast<std::int64_t>(pitch)-1,-static_cast<std::int64_t>(pitch)+1,
    static_cast<std::int64_t>(pitch)-1,static_cast<std::int64_t>(pitch)+1};
  while(!queue.empty()){
    const auto i=queue.pop();
    for(int k=0;k<connectivity;++k){
      const auto j=static_cast<std::uint32_t>(static_cast<std::int64_t>(i)+step[k]);
      // 已入队邻居不可能获得更低 minimax 代价；保持发现及邻居顺序，使等代价归属一致。
      if(position[j]!=Queue::unseen)continue;
      const auto candidate=std::max(gray[i],gray[j]);
      label[j]=label[i];queue.push(j,candidate);
    }
  }
  std::vector<std::pmr::vector<RegionRun>> runs; runs.reserve(markers.size());
  for(std::size_t m=0;m<markers.size();++m)runs.emplace_back(runtime::persist_resource());
  for(const auto& run:domain.runs()){
    auto begin=std::int64_t(run.col_begin);auto id=label[index(run.row,begin)];
    for(auto c=begin+1;c<=std::int64_t(run.col_end)+1;++c){const auto next=c<=run.col_end?label[index(run.row,c)]:0;
      if(next!=id || c==std::int64_t(run.col_end)+1){if(id)runs[id-1].push_back({run.row,begin,c-1});begin=c;id=next;}}
  }
  std::vector<Region> output;output.reserve(markers.size());
  for(auto& r:runs)output.push_back(Region::from_canonical_runs(std::move(r)));
  return Regions{std::move(output)};
}
Regions flood(const Image& image,const Regions& markers,int connectivity) {
  advanced::check(!image.empty(),operation,"Image","image required");
  return dispatch_scalar(image.pixel_type(),[&](auto tag){return flood_t<typename decltype(tag)::type>(image,markers,connectivity);});
}
}
Regions watersheds_marker(const IconicTuple& images,const Regions& markers,int connectivity) {
  advanced::check(connectivity==4 || connectivity==8,operation,"Connectivity","connectivity must be 4 or 8");
  images.require_kind(IconicKind::Image,operation);
  std::vector<Region> output;
  for(const auto& value:images.values()){
    const auto result=flood(value.as_image(),markers,connectivity);
    for(std::size_t i=0;i<result.size();++i)output.push_back(result[i]);
  }
  return Regions{std::move(output)};
}
} // namespace mv::imageproc
