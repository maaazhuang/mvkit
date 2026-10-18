#include "model.hpp"
#include "refinement_cache.hpp"
#include <core/exception.hpp>
#include <imageproc/gray_pyramid.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/execution/execution_context.hpp>
#include <runtime/memory/overwrite_buffer.hpp>
#if MVKIT_NCC_HAS_AVX2
#include "kernels_avx2.hpp"
#include "bilinear_zero_blocks.hpp"
#include "prefix_moments_cache.hpp"
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <cfenv>
#include <cstring>
#include <limits>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace mv::ncc_matching {
namespace {
constexpr double missing=-std::numeric_limits<double>::infinity();
struct Gray {
  std::size_t width=0,height=0;
  runtime::OverwriteBuffer<double> values;
  const std::uint8_t* bytes=nullptr;std::size_t stride=0;double dc=0;
  const double* shared_values=nullptr;
  std::size_t storage_width=0,storage_height=0,storage_row=0,storage_column=0;
  Gray(std::size_t w,std::size_t h,std::pmr::memory_resource* memory,bool borrowed=false):width(w),height(h),values(borrowed?0:w*h,memory),storage_width(w),storage_height(h) {}
  double at(std::size_t r,std::size_t c) const {
    if(bytes)return double(bytes[r*stride+c])-dc;
    return data()[(r-storage_row)*storage_width+c-storage_column];
  }
  const double* data() const {return shared_values?shared_values:values.data();}
};
struct PrefixStatistics {
  runtime::OverwriteBuffer<double> prefix_sum,prefix_squared;
  explicit PrefixStatistics(std::pmr::memory_resource* memory):prefix_sum(0,memory),prefix_squared(0,memory) {}
};
struct QueryLevel : PrefixStatistics {
  std::pmr::vector<Patch> endpoints;
  std::pmr::vector<const Patch*> patches;
  explicit QueryLevel(std::pmr::memory_resource* memory):PrefixStatistics(memory),endpoints(memory),patches(memory) {}
};
// 逐像素评分缓存避免重复评分起始列不同但重叠的 SIMD 窗口；
// tile 仅访问时分配，累计预算含已丢弃 scratch 分配。
struct ScorePlane {
  static constexpr std::size_t limit=64*1024*1024;
  static constexpr int tile_edge=16;
  using TileValues=std::array<double,tile_edge*tile_edge>;
  struct FreeTile {
    std::pmr::memory_resource* memory=nullptr;
    void operator()(TileValues* p)const{memory->deallocate(p,sizeof(*p),alignof(TileValues));}
  };
  using Tile=std::unique_ptr<TileValues,FreeTile>;
  int row0=0,column0=0,width=0,height=0;
  std::vector<Tile> tiles;
  std::pmr::memory_resource* memory=nullptr;
  std::size_t* allocated=nullptr;
  ScorePlane(int r,int c,int w,int h,std::pmr::memory_resource* resource,std::size_t* budget):
      row0(r),column0(c),width(w),height(h),memory(resource),allocated(budget) {
    const auto count=std::size_t((width+tile_edge-1)/tile_edge)*((height+tile_edge-1)/tile_edge);
    const auto metadata=count*sizeof(Tile);
    if(allocated && (metadata>limit || *allocated>limit-metadata)){width=height=0;return;}
    tiles.resize(count);if(allocated)*allocated+=metadata;
  }
  double* at(int r,int c,bool create=false) {
    r-=row0;c-=column0;if(r<0 || c<0 || r>=height || c>=width)return nullptr;
    auto& tile=tiles[std::size_t(r/tile_edge)*((width+tile_edge-1)/tile_edge)+c/tile_edge];
    if(!tile && create) {
      if(allocated && *allocated>limit-sizeof(TileValues))return nullptr;
      tile=Tile(static_cast<TileValues*>(memory->allocate(sizeof(TileValues),alignof(TileValues))),FreeTile{memory});
      tile->fill(std::numeric_limits<double>::quiet_NaN());
      if(allocated)*allocated+=sizeof(TileValues);
    }
    return tile?&(*tile)[(r%tile_edge)*tile_edge+c%tile_edge]:nullptr;
  }
};
[[noreturn]] void bad(const char* parameter,const char* message,core::Status status=core::Status::InvalidArgument) {
  throw core::Exception(status,core::default_error_code(status),"ncc_model_find",parameter,-1,message);
}
double discrete(const Gray& image,const Patch& patch,int row,int column,bool absolute,bool vectorize=false) {
  if(row+patch.row_min<0 || column+patch.column_min<0 ||
     row+patch.row_max>=std::int64_t(image.height) || column+patch.column_max>=std::int64_t(image.width))return missing;
  if(row+patch.row_min<std::int64_t(image.storage_row) || column+patch.column_min<std::int64_t(image.storage_column) ||
     row+patch.row_max>=std::int64_t(image.storage_row+image.storage_height) || column+patch.column_max>=std::int64_t(image.storage_column+image.storage_width))return missing;
#if MVKIT_NCC_HAS_AVX2
  if(vectorize && !image.bytes)return correlation_avx2(image.data(),image.storage_width,patch,row-int(image.storage_row),column-int(image.storage_column),absolute);
#endif
  const auto& first=patch.samples.front();
  const double offset=image.at(row+first.row,column+first.column);
  double sum=0,squared=0,dot=0;
  for(const auto& sample:patch.samples) {
    const double value=image.at(row+sample.row,column+sample.column)-offset;
    sum+=value;squared+=value*value;dot+=sample.weight*value;
  }
  const double variance=squared-sum*sum/double(patch.samples.size());
  if(!(variance>0))return missing;
  const double result=std::clamp(dot/std::sqrt(variance),-1.,1.);
  return absolute?std::abs(result):result;
}
double sample(const Gray& image,double row,double column) {
  if(row<0 || column<0 || row>image.height-1 || column>image.width-1)return missing;
  if(!image.bytes && (row<image.storage_row || column<image.storage_column || row>image.storage_row+image.storage_height-1 || column>image.storage_column+image.storage_width-1))return missing;
  const auto r=std::size_t(row),c=std::size_t(column),r1=std::min(r+1,image.height-1),c1=std::min(c+1,image.width-1);
  const auto fr=row-r,fc=column-c;
  return (1-fr)*((1-fc)*image.at(r,c)+fc*image.at(r,c1))+
      fr*((1-fc)*image.at(r1,c)+fc*image.at(r1,c1));
}
struct SampleCache {
  struct Key {double row,column,angle;bool operator==(const Key&)const=default;};
  struct Hash {std::size_t operator()(const Key& k)const{return std::hash<double>{}(k.row)^(std::hash<double>{}(k.column)<<1)^(std::hash<double>{}(k.angle)<<2);}};
  struct Values {runtime::OverwriteBuffer<double> samples{0,runtime::scratch_resource()};double sum,squared;};
  std::unordered_map<Key,Values,Hash> values;std::size_t bytes=0;
};
double continuous(const Gray& image,const Level& level,const Patch& original,const Match& pose,bool absolute,bool vectorize,SampleCache* cache=nullptr,double crop_row=0,double crop_column=0
#if MVKIT_NCC_HAS_AVX2
    ,const BilinearZeroView* zero_blocks=nullptr
#endif
    ) {
#if MVKIT_NCC_HAS_AVX2
  if(vectorize && !image.bytes) {
    auto local=pose;local.row-=image.storage_row;local.column-=image.storage_column;BilinearMoments m;
    const SampleCache::Key key{pose.row+crop_row,pose.column+crop_column,pose.angle};
    auto found=cache?cache->values.find(key):decltype(cache->values.find(key)){};
    if(cache && found!=cache->values.end()) {
      const auto& saved=found->second;m.sum=saved.sum;m.squared=saved.squared;m.dot=centered_dot_avx2(saved.samples.data(),original);
    }else {
      SampleCache::Values saved;
      const bool save=cache && cache->bytes+original.samples.size()*sizeof(double)<=32*1024*1024;
      if(save)saved.samples.reset_for_overwrite(original.samples.size());
      if(!bilinear_moments_avx2(image.data(),image.storage_width,image.storage_height,level,original,local,false,false,m,save?saved.samples.data():nullptr,nullptr,nullptr,nullptr,zero_blocks))return missing;
      if(save){saved.sum=m.sum;saved.squared=m.squared;cache->bytes+=original.samples.size()*sizeof(double);cache->values.emplace(key,std::move(saved));}
    }
    const auto variance=m.squared-m.sum*m.sum/double(original.samples.size());if(!(variance>0))return missing;
    const auto score=std::clamp(m.dot/std::sqrt(variance),-1.,1.);return absolute?std::abs(score):score;
  }
#endif
  const double ca=std::cos(pose.angle),sa=std::sin(pose.angle);
  double sum=0,squared=0,dot=0,offset=0;bool first=true;
  for(const auto& p:original.samples) {
    const auto r=p.row-level.origin_row,c=p.column-level.origin_column;
    auto value=sample(image,pose.row+ca*r-sa*c,pose.column+sa*r+ca*c);
    if(!std::isfinite(value))return missing;
    if(first){offset=value;first=false;}value-=offset;
    sum+=value;squared+=value*value;dot+=p.weight*value;
  }
  const auto variance=squared-sum*sum/double(original.samples.size());
  if(!(variance>0))return missing;
  const auto score=std::clamp(dot/std::sqrt(variance),-1.,1.);
  return absolute?std::abs(score):score;
}
// 对同一双线性、中心化、单位能量样本计算解析导数，用 Gauss-Newton 步替代多次数值全模板重扫。
#if MVKIT_NCC_HAS_AVX2
using DerivativeCache=RefinementCache<BilinearMoments>;
#endif
struct RefinementMemo {
#if MVKIT_NCC_HAS_AVX2
  DerivativeCache* cache=nullptr;
#endif
  double crop_row=0,crop_column=0;
  std::size_t* requests=nullptr;
  std::size_t* hits=nullptr;
};
bool refinement_step(const Gray& image,const Level& level,const Patch& original,const Match& pose,
                     bool absolute,bool rotate,double* delta,bool vectorize,double* output_score=nullptr,RefinementMemo* memo=nullptr,double* derivative_workspace=nullptr,const double* minimum_score=nullptr
#if MVKIT_NCC_HAS_AVX2
                     ,const BilinearZeroView* zero_blocks=nullptr
#endif
                     ) {
  const auto ca=std::cos(pose.angle),sa=std::sin(pose.angle);
  double sum=0,energy=0,dot=0,gs[3]{},qg[3]{},wg[3]{},gg[3][3]{},offset=0;bool first=true;
#if MVKIT_NCC_HAS_AVX2
  if(vectorize && !image.bytes) {
    auto local=pose;local.row-=image.storage_row;local.column-=image.storage_column;BilinearMoments m;
    const DerivativeCache::Key key{image.data(),image.storage_width,image.storage_height,
      std::int64_t((memo?memo->crop_row:0)+image.storage_row),std::int64_t((memo?memo->crop_column:0)+image.storage_column),
      pose.row+(memo?memo->crop_row:0),pose.column+(memo?memo->crop_column:0),pose.angle,rotate};
    const auto* saved=memo && memo->cache?memo->cache->find(key,original.samples.size()):nullptr;
    if(memo && memo->cache && memo->requests)++*memo->requests;
    if(saved) {
      m=saved->unweighted;
      if(saved->compressed)weighted_moments_compressed_avx2(saved->samples(),saved->groups(),saved->group_count(),original,rotate,m);
      else weighted_moments_avx2(saved->samples(),saved->gradients(),original,rotate,m);
      if(memo->hits)++*memo->hits;
    }else {
      const bool save=memo && memo->cache && derivative_workspace && memo->cache->can_store(original.samples.size());
      BilinearScoreGate gate{minimum_score?*minimum_score:missing,absolute};
      if(!bilinear_moments_avx2(image.data(),image.storage_width,image.storage_height,level,original,local,true,rotate,m,
          save?derivative_workspace:nullptr,save?derivative_workspace+original.samples.size():nullptr,nullptr,minimum_score?&gate:nullptr,zero_blocks))return false;
      if(gate.skipped) {
        const auto variance=m.squared-m.sum*m.sum/double(original.samples.size());
        const double sign=absolute && m.dot<0?-1:1;
        if(output_score)*output_score=std::clamp(sign*m.dot/std::sqrt(variance),-1.,1.);
        return false;
      }
      if(save)memo->cache->store(key,derivative_workspace,derivative_workspace+original.samples.size(),original.samples.size(),m);
    }
    sum=m.sum;energy=m.squared;dot=m.dot;
    for(int i=0;i<3;++i){gs[i]=m.gs[i];qg[i]=m.qg[i];wg[i]=m.wg[i];for(int j=0;j<3;++j)gg[i][j]=m.gg[i][j];}
  }else
#endif
  for(const auto& p:original.samples) {
    const auto dr=p.row-level.origin_row,dc=p.column-level.origin_column;
    const auto y=pose.row+ca*dr-sa*dc,x=pose.column+sa*dr+ca*dc;
    if(y<image.storage_row || x<image.storage_column || y>=image.storage_row+image.storage_height-1 || x>=image.storage_column+image.storage_width-1)return false;
    const auto r=std::size_t(y),c=std::size_t(x);const auto fy=y-r,fx=x-c;
    const auto a=image.at(r,c),b=image.at(r,c+1),d=image.at(r+1,c),e=image.at(r+1,c+1);
    auto q=(1-fy)*((1-fx)*a+fx*b)+fy*((1-fx)*d+fx*e);
    if(first){offset=q;first=false;}q-=offset;
    const auto gy=(1-fx)*(d-a)+fx*(e-b),gx=(1-fy)*(b-a)+fy*(e-d);
    const double g[3]={gy,gx,gy*(-sa*dr-ca*dc)+gx*(ca*dr-sa*dc)};
    sum+=q;energy+=q*q;dot+=q*p.weight;
    for(int i=0;i<(rotate?3:2);++i){gs[i]+=g[i];qg[i]+=q*g[i];wg[i]+=p.weight*g[i];for(int j=0;j<=i;++j)gg[i][j]+=g[i]*g[j];}
  }
  const auto n=double(original.samples.size());energy-=sum*sum/n;if(!(energy>0))return false;
  const double sign=absolute && dot<0?-1:1,score=sign*dot/std::sqrt(energy);
  if(output_score)*output_score=std::clamp(score,-1.,1.);
  if(minimum_score && std::clamp(score,-1.,1.)<=*minimum_score)return false;
  const int axes=rotate?3:2;double matrix[3][4]{};
  for(int i=0;i<axes;++i)qg[i]-=sum*gs[i]/n;
  for(int i=0;i<axes;++i) {
    for(int j=0;j<axes;++j)matrix[i][j]=(gg[std::max(i,j)][std::min(i,j)]-gs[i]*gs[j]/n-qg[i]*qg[j]/energy)/energy;
    matrix[i][axes]=sign*wg[i]/std::sqrt(energy)-score*qg[i]/energy;
  }
  for(int i=0;i<axes;++i) {
    int pivot=i;for(int j=i+1;j<axes;++j)if(std::abs(matrix[j][i])>std::abs(matrix[pivot][i]))pivot=j;
    if(std::abs(matrix[pivot][i])<1e-14)return false;
    for(int k=i;k<=axes;++k)std::swap(matrix[i][k],matrix[pivot][k]);
    const auto divisor=matrix[i][i];for(int k=i;k<=axes;++k)matrix[i][k]/=divisor;
    for(int j=0;j<axes;++j)if(j!=i){const auto factor=matrix[j][i];for(int k=i;k<=axes;++k)matrix[j][k]-=factor*matrix[i][k];}
  }
  for(int i=0;i<axes;++i)delta[i]=matrix[i][axes];if(!rotate)delta[2]=0;
  return std::isfinite(delta[0]) && std::isfinite(delta[1]) && std::isfinite(delta[2]);
}
void merge_identical(std::pmr::vector<Match>& candidates) {
  std::sort(candidates.begin(),candidates.end(),[](const Match& a,const Match& b) {
    if(a.row!=b.row)return a.row<b.row;if(a.column!=b.column)return a.column<b.column;return a.angle<b.angle;
  });
  candidates.erase(std::unique(candidates.begin(),candidates.end(),[](const Match& a,const Match& b) {
    return a.row==b.row && a.column==b.column && a.angle==b.angle;
  }),candidates.end());
}
bool domain_contains(const core::Region& domain,const Match& match) {
  return domain.contains(std::int64_t(std::floor(match.row+.5)),std::int64_t(std::floor(match.column+.5)));
}
double overlap(const Match& a,const Match& b,const Level& level) {
  // 求两个旋转模板包围矩形交集；多边形裁剪仅为有界元数据，不替代掩码评分。
  struct Point { double x,y; };
  const auto corners=[&](const Match& m) {
    std::array<Point,4> out;const auto ca=std::cos(m.angle),sa=std::sin(m.angle);
    const double left=-level.origin_column-.5,right=level.width-level.origin_column-.5;
    const double top=-level.origin_row-.5,bottom=level.height-level.origin_row-.5;
    const std::array<Point,4> input={Point{left,top},Point{right,top},Point{right,bottom},Point{left,bottom}};
    for(int i=0;i<4;++i)out[i]={m.column-a.column+ca*input[i].x+sa*input[i].y,m.row-a.row-sa*input[i].x+ca*input[i].y};
    return out;
  };
  const auto clip=corners(b);std::array<Point,16> polygon{},next{};
  const auto source=corners(a);std::copy(source.begin(),source.end(),polygon.begin());std::size_t count=4;
  const auto cross=[](Point a,Point b,Point c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);};
  for(int edge=0;edge<4 && count;++edge) {
    const auto p=clip[edge],q=clip[(edge+1)%4];std::size_t out=0;
    for(std::size_t i=0;i<count;++i) {
      const auto u=polygon[i],v=polygon[(i+1)%count];const auto du=cross(p,q,u),dv=cross(p,q,v);
      if(du>=0)next[out++]=u;
      if((du>=0)!=(dv>=0)) {const auto t=du/(du-dv);next[out++]={u.x+t*(v.x-u.x),u.y+t*(v.y-u.y)};}
    }
    polygon=next;count=out;
  }
  double area=0;for(std::size_t i=0;i<count;++i)area+=polygon[i].x*polygon[(i+1)%count].y-polygon[(i+1)%count].x*polygon[i].y;
  return std::abs(area)*.5/(double(level.width)*level.height);
}
}

namespace {
struct QueryCache {
  runtime::ExecutionContext context;
  std::pmr::vector<Gray> pyramid{context.scratch_resource()};
  std::int64_t row=-1,column=-1,bottom=-1,right=-1;
  SampleCache samples;const Model* sample_geometry=nullptr;bool cache_samples=false;
  std::unordered_map<std::uint64_t,std::vector<core::RegionInterval>> roi_rows;
  std::vector<std::unique_ptr<PrefixStatistics>> statistics;
  Gray* full_fine=nullptr;std::vector<std::unique_ptr<Gray>> fine_storage;
  std::unordered_map<const double*,std::shared_ptr<PrefixStatistics>> fine_statistics;
  std::size_t score_tile_bytes=0;
#if MVKIT_NCC_HAS_AVX2
  DerivativeCache derivatives{context.scratch_resource()};
  PrefixMomentsCache shared_prefix_moments{context.scratch_resource()};
  std::unordered_map<const double*,std::unique_ptr<BilinearZeroImage>> zero_images;
  std::size_t zero_image_bytes=0,zero_sample_bytes=0,zero_tested_blocks=0,zero_skipped_blocks=0;
#endif
  bool cache_derivatives=false;
  std::size_t model_count=0;
  bool unlimited_selection=false;
  std::size_t derivative_requests=0,derivative_hits=0;
};
}
namespace {
struct PreparedQuery {
  runtime::ExecutionContext context;
  runtime::OverwriteBuffer<double> derivative_workspace{0,context.scratch_resource()};
  std::pmr::vector<QueryLevel> query_levels{context.scratch_resource()};
  Matches candidates{context.scratch_resource()};
  std::unique_ptr<Gray> fine;
  bool fit_trace=false;
  std::size_t level_count=0,finest_level=0;
  std::unique_ptr<Patch> coarse_original;
  std::size_t model_index=0;
  const Model* model=nullptr;
  FindOptions options;
  core::Region domain;
  std::int64_t crop_row=0,crop_column=0;
  SampleCache* samples=nullptr;
  struct FitKey {
    double row,column,angle,score;
    bool operator==(const FitKey&) const=default;
  };
  struct FitHash {
    std::size_t operator()(const FitKey& k)const {
      return std::hash<double>{}(k.row)^(std::hash<double>{}(k.column)<<1)^(std::hash<double>{}(k.angle)<<2)^(std::hash<double>{}(k.score)<<3);
    }
  };
  std::unordered_map<const Patch*,ScorePlane> raster_scores;
  std::pmr::unordered_map<FitKey,Match,FitHash> fitted{context.scratch_resource()};
  bool memoize_refinement=false;
  bool default_background_guard=false;
  bool exact_byte_statistics=false;
  QueryCache* cache=nullptr;
  std::size_t refine_calls=0,fit_calls=0,fit_hits=0,raster_hits=0,derivative_calls=0,proposal_calls=0,proposal_rejected=0;
#if MVKIT_NCC_HAS_AVX2
  std::unique_ptr<BilinearZeroSamples> zero_samples;
  runtime::OverwriteBuffer<std::uint8_t> zero_tile_flags{0,context.scratch_resource()};
  BilinearZeroProofCache zero_proof_cache;
  BilinearZeroView zero_view{};
#endif
};
}
static std::unique_ptr<PreparedQuery> prepare(const core::Image& image,const Model& model,const FindOptions& requested,
    QueryCache* cache=nullptr) {
  FindOptions options=requested;
  auto prepared=std::make_unique<PreparedQuery>();prepared->model=&model;prepared->options=options;prepared->cache=cache;
  prepared->exact_byte_statistics=image.pixel_type()==core::PixelType::UInt8;
  const auto tick=std::chrono::steady_clock::now();
  if(image.empty() || image.channel_count()!=1)bad("image","one nonempty single-channel image required");
  if(model.levels.empty())bad("model","model is empty",core::Status::InvalidState);
  if(options.highest_level<0 || options.lowest_level<1)bad("NumLevels","invalid pyramid levels");
  const auto level_count=options.highest_level ? std::min(model.levels.size(),std::size_t(options.highest_level)) : model.levels.size();
  options.lowest_level=int(std::min(std::size_t(options.lowest_level),level_count));
  const auto finest=std::size_t(options.lowest_level-1);
  prepared->level_count=level_count;prepared->finest_level=finest;
  if(finest) {
    auto original=make_patch(model.levels[finest],0,prepared->context.scratch_resource());
    original.relative_rows.reserve(original.samples.size());original.relative_columns.reserve(original.samples.size());
    for(const auto& sample:original.samples){original.relative_rows.push_back(sample.row-model.levels[finest].origin_row);original.relative_columns.push_back(sample.column-model.levels[finest].origin_column);}
    prepared->coarse_original=std::make_unique<Patch>(std::move(original));
  }
  if(!std::isfinite(options.angle_start) || !std::isfinite(options.angle_extent) || options.angle_extent<0 ||
     !std::isfinite(options.min_score) || options.min_score<0 || options.min_score>1 ||
     !std::isfinite(options.max_overlap) || options.max_overlap<0 || options.max_overlap>1 || !options.max_candidates)
    bad("options","invalid query range, score, overlap or candidate budget");
  const auto angle_end=options.angle_start+options.angle_extent;
  if(!std::isfinite(angle_end))bad("angle_extent","query angle range must be finite");
  const auto clipped_start=std::max(options.angle_start,model.options.angle_start);
  const auto clipped_end=std::min(angle_end,model.options.angle_start+model.options.angle_extent);
  if(clipped_start>clipped_end+1e-12)return prepared;
  options.angle_start=clipped_start;options.angle_extent=std::max(0.,clipped_end-clipped_start);
  prepared->options=options;
  if(!image.width() || !image.height() || image.width()>std::size_t(std::numeric_limits<int>::max()) || image.height()>std::size_t(std::numeric_limits<int>::max()) ||
     image.height()>32000000/image.width())bad("image","search image exceeds pixel budget",core::Status::LimitExceeded);
  const auto domain=image.domain();prepared->domain=domain;
  if(!finest && cache && cache->cache_samples && image.pixel_type()==core::PixelType::UInt8) {
    if(!cache->sample_geometry)cache->sample_geometry=&model;
    const auto& a=cache->sample_geometry->levels.front();const auto& b=model.levels.front();
    if(a.width==b.width && a.height==b.height && a.origin_row==b.origin_row && a.origin_column==b.origin_column && a.mask==b.mask)
      prepared->samples=&cache->samples;
  }
  if(domain.empty())return prepared;
  const auto search_box=domain.bounding_box();
  const auto& base_model=model.levels.front();
  const auto radius=std::hypot(std::max(base_model.origin_row,double(base_model.height-1)-base_model.origin_row),
      std::max(base_model.origin_column,double(base_model.width-1)-base_model.origin_column));
  const auto lattice=std::int64_t(std::size_t{1}<<(level_count-1));
  const auto halo=std::int64_t(std::ceil(radius))+4*lattice;
  // 保留原全局金字塔晶格；仅裁剪不可能影响允许位姿及其插值/金字塔 halo 的像素。
  const auto crop_row=std::max<std::int64_t>(0,search_box->row_min-halo)/lattice*lattice;
  const auto crop_column=std::max<std::int64_t>(0,search_box->col_min-halo)/lattice*lattice;
  const auto crop_bottom=std::min<std::int64_t>(image.height(),((search_box->row_max+halo)/lattice+1)*lattice);
  const auto crop_right=std::min<std::int64_t>(image.width(),((search_box->col_max+halo)/lattice+1)*lattice);
  auto& context=prepared->context;auto* scratch=context.scratch_resource();
  const bool vectorize=context.cpu().selection().avx2;
  std::pmr::vector<Gray> local_pyramid{scratch};
  auto& pyramid=cache?cache->pyramid:local_pyramid;
  auto* pyramid_memory=cache?cache->context.scratch_resource():scratch;
  const bool reuse=cache && cache->row==crop_row && cache->column==crop_column &&
      cache->bottom==crop_bottom && cache->right==crop_right && pyramid.size()>=level_count;
  if(!reuse) {
  if(cache){cache->statistics.clear();cache->full_fine=nullptr;}
#if MVKIT_NCC_HAS_AVX2
  if(cache)cache->shared_prefix_moments.invalidate();
#endif
  pyramid.clear();
  Gray first{std::size_t(crop_right-crop_column),std::size_t(crop_bottom-crop_row),pyramid_memory,image.pixel_type()==core::PixelType::UInt8};
  imageproc::dispatch_scalar(image.pixel_type(),[&](auto tag) {
    using T=typename decltype(tag)::type;const auto* base=static_cast<const std::byte*>(image.data());
    T first_value;std::memcpy(&first_value,base+crop_row*image.stride()+crop_column*sizeof(T),sizeof(T));const double offset=double(first_value);
    if constexpr(std::is_same_v<T,std::uint8_t>) {
      first.bytes=reinterpret_cast<const std::uint8_t*>(base+crop_row*image.stride()+crop_column);first.stride=image.stride();first.dc=offset;
      return;
    }
    for(std::size_t r=0;r<first.height;++r) {
     std::size_t c=0;
#if MVKIT_NCC_HAS_AVX2
     if constexpr(std::is_same_v<T,std::uint8_t>)if(vectorize)c=gray_u8_avx2(reinterpret_cast<const std::uint8_t*>(base+(r+crop_row)*image.stride()+crop_column),first.values.data()+r*first.width,first.width,offset);
     if(vectorize) {
      const auto* row=base+(r+crop_row)*image.stride()+crop_column*sizeof(T);
      auto* output=first.values.data()+r*first.width;
      if constexpr(std::is_same_v<T,std::uint16_t>)c=gray_u16_avx2(reinterpret_cast<const T*>(row),output,first.width,offset);
      else if constexpr(std::is_same_v<T,std::int16_t>)c=gray_i16_avx2(reinterpret_cast<const T*>(row),output,first.width,offset);
      else if constexpr(std::is_same_v<T,std::int32_t>)c=gray_i32_avx2(reinterpret_cast<const T*>(row),output,first.width,offset);
      else if constexpr(std::is_same_v<T,float>)c=gray_f32_avx2(reinterpret_cast<const T*>(row),output,first.width,offset);
     }
#endif
     for(;c<first.width;++c) {
      T value;std::memcpy(&value,base+(r+crop_row)*image.stride()+(c+crop_column)*sizeof(T),sizeof(T));
      if(!std::isfinite(double(value)))bad("image","search samples must be finite");
      first.values[r*first.width+c]=double(value)-offset;
     }
    }
  });
  pyramid.push_back(std::move(first));
  for(std::size_t l=1;l<level_count;++l) {
    const auto& previous=pyramid.back();Gray next{std::max<std::size_t>(1,previous.width/2),std::max<std::size_t>(1,previous.height/2),pyramid_memory};
    for(std::size_t r=0;r<next.height;++r) {
      std::size_t c=0;
#if MVKIT_NCC_HAS_AVX2
      if(vectorize) {
        const auto r0=2*r,r1=std::min(r0+1,previous.height-1);
        if(previous.bytes)c=box2_u8_avx2(previous.bytes+r0*previous.stride,previous.bytes+r1*previous.stride,next.values.data()+r*next.width,previous.width,previous.dc);
        else c=box2_double_avx2(previous.data()+r0*previous.width,previous.data()+r1*previous.width,next.values.data()+r*next.width,previous.width);
      }
#endif
      for(;c<next.width;++c) {
      if(previous.bytes){const auto r0=2*r,r1=std::min(r0+1,previous.height-1),c0=2*c,c1=std::min(c0+1,previous.width-1);next.values[r*next.width+c]=.25*(previous.at(r0,c0)+previous.at(r0,c1)+previous.at(r1,c0)+previous.at(r1,c1));}
      else next.values[r*next.width+c]=imageproc::box2_gray(previous.data()+2*r*previous.width,
          previous.data()+std::min(2*r+1,previous.height-1)*previous.width,c,previous.width);
      }
    }
    pyramid.push_back(std::move(next));
  }
  if(cache){cache->row=crop_row;cache->column=crop_column;cache->bottom=crop_bottom;cache->right=crop_right;}
  } else if(image.pixel_type()==core::PixelType::UInt8) {
    // 零层候选局部转换由各模型私有持有。
    auto& fine=pyramid.front();
    fine.bytes=static_cast<const std::uint8_t*>(image.data())+crop_row*image.stride()+crop_column;
    fine.stride=image.stride();fine.storage_row=0;fine.storage_column=0;
    fine.storage_width=fine.width;fine.storage_height=fine.height;
  }
  auto& query_levels=prepared->query_levels;query_levels.reserve(level_count);
  for(std::size_t level_index=0;level_index<level_count;++level_index) {
    const auto& level=model.levels[level_index];
    QueryLevel query{scratch};query.endpoints.reserve(2);
    for(const auto& patch:level.patches)if(patch.angle>=options.angle_start-1e-12 && patch.angle<=angle_end+1e-12)query.patches.push_back(&patch);
    for(const double angle : {options.angle_start,angle_end}) {
      bool found=false;for(const auto* patch:query.patches)if(std::abs(patch->angle-angle)<1e-12)found=true;
      if(found)continue;
      query.endpoints.push_back(make_patch(level,angle,scratch));
      if(!query.endpoints.back().samples.empty())query.patches.push_back(&query.endpoints.back());
    }
    query_levels.push_back(std::move(query));
  }
  const auto top=level_count-1;const auto& coarse=pyramid[top];
  const auto coarse_radius=.5*std::hypot(double(model.levels[top].width),double(model.levels[top].height));
  const auto angle_stride=std::max<std::size_t>(1,std::size_t(std::min(double(query_levels[top].patches.size()),std::ceil(.5/std::max(1e-20,coarse_radius*model.options.angle_step)))));
  auto& coarse_angles=query_levels[top].patches;
  std::sort(coarse_angles.begin(),coarse_angles.end(),[](const Patch* a,const Patch* b){return a->angle<b->angle;});
  std::size_t kept=0;for(std::size_t i=0;i<coarse_angles.size();++i)if(i%angle_stride==0 || i+1==coarse_angles.size())coarse_angles[kept++]=coarse_angles[i];
  coarse_angles.resize(kept);
  if(cache) {
    cache->statistics.resize(pyramid.size());
    for(auto& item:cache->statistics)if(!item)item=std::make_unique<PrefixStatistics>(cache->context.scratch_resource());
  }
  auto& top_statistics=cache?*cache->statistics[top]:static_cast<PrefixStatistics&>(query_levels[top]);
  auto& prefix_sum=top_statistics.prefix_sum;auto& prefix_squared=top_statistics.prefix_squared;
  if(!prefix_sum.size() && image.pixel_type()==core::PixelType::UInt8 && !coarse.bytes) {
  prefix_sum.reset_for_overwrite((coarse.width+1)*coarse.height);prefix_squared.reset_for_overwrite(prefix_sum.size());
  for(std::size_t r=0;r<coarse.height;++r) {
    const auto begin=r*(coarse.width+1);double sum=0,sq=0;prefix_sum[begin]=0;prefix_squared[begin]=0;
    for(std::size_t c=0;c<coarse.width;++c){const auto q=coarse.values[r*coarse.width+c];sum+=q;sq+=q*q;prefix_sum[begin+c+1]=sum;prefix_squared[begin+c+1]=sq;}
  }
  }
  // 精确零支撑前缀：仅含共同 DC 的矩形不能匹配任意非常量掩码/旋转模板；
  // 拒绝不使用评分阈值或对比 epsilon。
  std::pmr::vector<std::uint32_t> support((coarse.width+1)*(coarse.height+1),0,scratch);
  for(std::size_t r=0;r<coarse.height;++r){std::uint32_t row_sum=0;for(std::size_t c=0;c<coarse.width;++c){row_sum+=coarse.at(r,c)!=0;support[(r+1)*(coarse.width+1)+c+1]=support[r*(coarse.width+1)+c+1]+row_sum;}}
  const auto has_support=[&](const Patch& p,int r,int c,int extra=0) {
    const auto pitch=coarse.width+1;const auto r0=r+p.row_min,r1=r+p.row_max+1,c0=c+p.column_min,c1=c+p.column_max+1+extra;
    return support[r1*pitch+c1]-support[r1*pitch+c0]-support[r0*pitch+c1]+support[r0*pitch+c0]!=0;
  };
  const double scale=std::ldexp(1.,int(top)),phase=(scale-1)*.5;
  const bool sparse_domain=image.has_reduced_domain() && double(domain.area())<.7*double(search_box->row_max-search_box->row_min+1)*double(search_box->col_max-search_box->col_min+1);
  const auto domain_runs=domain.runs();
  const auto may_contain=[&](const Match& pose,double spacing,double column_span=0) {
    if(spacing==1 && column_span==0)return domain_contains(domain,pose);
    // 保留细层后代能抵达实际 ROI 游程的全部质心；检查完整 SIMD 块，避免遗漏 lane 之间的 ROI。
    const auto margin=spacing==1?.5:2*spacing;
    if(pose.row<search_box->row_min-margin || pose.row>search_box->row_max+margin ||
       pose.column+column_span<search_box->col_min-margin || pose.column>search_box->col_max+margin)return false;
    if(!sparse_domain)return true;
    if(cache) {
      const auto first=std::int64_t(std::ceil(pose.row-margin)),last=std::int64_t(std::floor(pose.row+margin));
      const auto key=(std::uint64_t(std::uint32_t(first))<<32)|std::uint32_t(last);
      auto [entry,inserted]=cache->roi_rows.try_emplace(key);
      if(inserted) {
        auto it=std::lower_bound(domain_runs.begin(),domain_runs.end(),first,[](const auto& run,std::int64_t row){return run.row<row;});
        auto& intervals=entry->second;
        for(;it!=domain_runs.end() && it->row<=last;++it)intervals.push_back({it->col_begin,it->col_end});
        std::sort(intervals.begin(),intervals.end(),[](const auto& a,const auto& b){return a.first<b.first || (a.first==b.first && a.last<b.last);});
        std::size_t count=0;for(const auto interval:intervals){if(count && interval.first<=intervals[count-1].last)intervals[count-1].last=std::max(intervals[count-1].last,interval.last);else intervals[count++]=interval;}
        intervals.resize(count);
      }
      for(const auto interval:entry->second)if(interval.first<=pose.column+column_span+margin && interval.last>=pose.column-margin)return true;
      return false;
    }
    auto it=std::lower_bound(domain_runs.begin(),domain_runs.end(),pose.row-margin,
        [](const auto& run,double row){return run.row<row;});
    for(;it!=domain_runs.end() && it->row<=pose.row+margin;++it)
      if(it->col_begin<=pose.column+column_span+margin && it->col_end>=pose.column-margin)return true;
    return false;
  };
  auto& candidates=prepared->candidates;std::pmr::vector<double> scores(coarse.width*coarse.height,missing,scratch);
  const auto* shared_prefix_setting=std::getenv("MVKIT_NCC_SHARED_PREFIX_MOMENTS");
  // ROI 搜索会让多模型反复访问同一稀疏栅格；其他工作负载除非显式指定，保留原路径。
  const bool default_shared_prefix=cache && cache->model_count>=16 && cache->cache_derivatives && image.has_reduced_domain();
  const bool share_all_prefix=shared_prefix_setting?!std::strcmp(shared_prefix_setting,"all"):default_shared_prefix;
  const bool share_prefix=cache && vectorize && prefix_sum.size() && image.pixel_type()==core::PixelType::UInt8 &&
      (shared_prefix_setting?(!std::strcmp(shared_prefix_setting,"1") || share_all_prefix):default_shared_prefix);
  for(const auto* planned_patch:query_levels[top].patches) {
    const auto& patch=*planned_patch;
#if MVKIT_NCC_HAS_AVX2
    auto* shared_moments=share_prefix?cache->shared_prefix_moments.get(coarse.data(),coarse.width,coarse.height,patch):nullptr;
#endif
    std::fill(scores.begin(),scores.end(),missing);
    // 定义域归属将参考点取最近像素；单像素 ROI 包含距该点 0.5 px 内的小数质心。
    const auto margin=scale==1?.5:2*scale;
    const int r_begin=std::max({0,-patch.row_min,int(std::ceil((search_box->row_min-margin-crop_row-phase)/scale-patch.origin_row))});
    const int r_end=std::min(int(coarse.height)-patch.row_max,int(std::floor((search_box->row_max+margin-crop_row-phase)/scale-patch.origin_row))+1);
    const int c_begin=std::max({0,-patch.column_min,int(std::ceil((search_box->col_min-margin-crop_column-phase)/scale-patch.origin_column))});
    const int c_end=std::min(int(coarse.width)-patch.column_max,int(std::floor((search_box->col_max+margin-crop_column-phase)/scale-patch.origin_column))+1);
    for(int r=r_begin;r<r_end;++r) {
      int c=c_begin;
#if MVKIT_NCC_HAS_AVX2
      if(vectorize && prefix_sum.size())for(;c+15<c_end;c+=16) {
        Match pose{crop_row+scale*(r+patch.origin_row)+phase,crop_column+scale*(c+patch.origin_column)+phase,patch.angle,0};
        auto end_pose=pose;end_pose.column+=15*scale;
        if(!may_contain(pose,scale,15*scale) || !has_support(patch,r,c,15))continue;
        correlation_sixteen_prefix_avx2(coarse.data(),prefix_sum.data(),prefix_squared.data(),coarse.width,patch,r,c,model.options.ignore_global_polarity,scores.data()+r*coarse.width+c,prepared->exact_byte_statistics && top<=2,
            shared_moments?shared_moments->sum.data()+r*coarse.width+c:nullptr,shared_moments?shared_moments->squared.data()+r*coarse.width+c:nullptr);
        for(int lane=0;lane<16;++lane){auto p=pose;p.column+=lane*scale;if(!may_contain(p,scale))scores[r*coarse.width+c+lane]=missing;}
      }
      if(vectorize && !coarse.bytes)for(;c+3<c_end;c+=4) {
        Match pose{crop_row+scale*(r+patch.origin_row)+phase,crop_column+scale*(c+patch.origin_column)+phase,patch.angle,0};
        auto end_pose=pose;end_pose.column+=3*scale;
        if(!may_contain(pose,scale,3*scale) || !has_support(patch,r,c,3))continue;
        if(prefix_sum.size())correlation_four_prefix_avx2(coarse.data(),prefix_sum.data(),prefix_squared.data(),coarse.width,patch,r,c,model.options.ignore_global_polarity,scores.data()+r*coarse.width+c,prepared->exact_byte_statistics && top<=2,
            shared_moments?shared_moments->sum.data()+r*coarse.width+c:nullptr,shared_moments?shared_moments->squared.data()+r*coarse.width+c:nullptr);
        else correlation_four_avx2(coarse.data(),coarse.width,patch,r,c,model.options.ignore_global_polarity,scores.data()+r*coarse.width+c);
        for(int lane=0;lane<4;++lane){auto p=pose;p.column+=lane*scale;if(!may_contain(p,scale))scores[r*coarse.width+c+lane]=missing;}
      }
#endif
      for(;c<c_end;++c) {
        Match pose{crop_row+scale*(r+patch.origin_row)+phase,crop_column+scale*(c+patch.origin_column)+phase,patch.angle,0};
        if(may_contain(pose,scale) && has_support(patch,r,c))scores[r*coarse.width+c]=discrete(coarse,patch,r,c,model.options.ignore_global_polarity);
      }
    }
    // 保留高于文档粗阈值的全部局部峰，不将候选束静默截到 num_matches；溢出明确报错。
    for(int r=r_begin;r<r_end;++r)for(int c=c_begin;c<c_end;++c) {
      const auto value=scores[r*coarse.width+c];if(value<options.min_score*.4 || !std::isfinite(value))continue;
      bool maximum=true;
      for(int dr=-1;dr<=1;++dr)for(int dc=-1;dc<=1;++dc) {
        const auto rr=std::int64_t(r)+dr,cc=std::int64_t(c)+dc;
        if(rr<0 || cc<0 || rr>=std::int64_t(coarse.height) || cc>=std::int64_t(coarse.width) || (!dr && !dc))continue;
        const auto other=scores[rr*coarse.width+cc];
        if(other>value || (other==value && (dr<0 || (!dr && dc<0))))maximum=false;
      }
      if(!maximum)continue;
      if(candidates.size()==options.max_candidates)bad("max_candidates","coarse candidate budget exceeded",core::Status::LimitExceeded);
      candidates.push_back({crop_row+scale*(r+patch.origin_row)+phase,crop_column+scale*(c+patch.origin_column)+phase,patch.angle,value});
    }
  }
  const auto coarse_done=std::chrono::steady_clock::now();
  if(!finest && cache && cache->cache_samples && pyramid.front().bytes && !candidates.empty()) {
    auto& source=pyramid.front();
    if(!cache->full_fine) {
      auto full=std::make_unique<Gray>(source.width,source.height,cache->context.scratch_resource());
      for(std::size_t r=0;r<source.height;++r) {
        const auto* input=source.bytes+r*source.stride;auto* output=full->values.data()+r*source.width;std::size_t c=0;
#if MVKIT_NCC_HAS_AVX2
        if(vectorize)c=gray_u8_avx2(input,output,source.width,source.dc);
#endif
        for(;c<source.width;++c)output[c]=double(input[c])-source.dc;
      }
      cache->full_fine=full.get();cache->fine_storage.push_back(std::move(full));
    }
    source.shared_values=cache->full_fine->values.data();source.bytes=nullptr;
    source.storage_row=0;source.storage_column=0;source.storage_width=source.width;source.storage_height=source.height;
  }
  if(!finest && pyramid.front().bytes && !candidates.empty()) {
    auto& fine=pyramid.front();double rmin=fine.height,cmin=fine.width,rmax=0,cmax=0;
    for(const auto& p:candidates){rmin=std::min(rmin,p.row-crop_row);rmax=std::max(rmax,p.row-crop_row);cmin=std::min(cmin,p.column-crop_column);cmax=std::max(cmax,p.column-crop_column);}
    // 各细层后代最多移动两个晶格格，另加有界连续拟合位移；包围范围包含旋转支撑。
    const auto margin=radius+4*scale+8;
    fine.storage_row=std::size_t(std::max(0.,std::floor(rmin-margin)));fine.storage_column=std::size_t(std::max(0.,std::floor(cmin-margin)));
    fine.storage_height=std::size_t(std::min(double(fine.height),std::ceil(rmax+margin)+1))-fine.storage_row;
    fine.storage_width=std::size_t(std::min(double(fine.width),std::ceil(cmax+margin)+1))-fine.storage_column;
    fine.values.reset_for_overwrite(fine.storage_width*fine.storage_height);
    for(std::size_t r=0;r<fine.storage_height;++r) {
      const auto* input=fine.bytes+(r+fine.storage_row)*fine.stride+fine.storage_column;auto* output=fine.values.data()+r*fine.storage_width;std::size_t c=0;
#if MVKIT_NCC_HAS_AVX2
      if(vectorize)c=gray_u8_avx2(input,output,fine.storage_width,fine.dc);
#endif
      for(;c<fine.storage_width;++c)output[c]=double(input[c])-fine.dc;
    }
    fine.bytes=nullptr;
  }
  for(std::size_t l=top;l>finest;--l) {
   if(l==1 && options.subpixel) {
     // 最终连续拟合基于原样本联合求位置/角度，不为生成初值额外枚举完整栅格模板库。
     for(auto& p:candidates)if(!domain_contains(domain,p)) {
       double best=std::numeric_limits<double>::infinity();Match nearest=p;
       const auto& runs=domain.runs();
       auto first=std::lower_bound(runs.begin(),runs.end(),p.row-4,[](const auto& run,double row){return run.row<row;});
       for(auto it=first;it!=runs.end() && it->row<=p.row+4;++it) {const auto& run=*it;
         const auto c=std::clamp(std::round(p.column),double(run.col_begin),double(run.col_end));
         if(std::abs(c-p.column)>4)continue;const auto distance=std::hypot(double(run.row)-p.row,c-p.column);
         if(distance<best){best=distance;nearest.row=double(run.row);nearest.column=c;}
       }
       if(std::isfinite(best))p=nearest;else p.score=missing;
     }
     merge_identical(candidates);break;
   }
   auto& statistics=cache?*cache->statistics[l-1]:static_cast<PrefixStatistics&>(query_levels[l-1]);const auto& target=pyramid[l-1];
   if(!statistics.prefix_sum.size() && vectorize && image.pixel_type()==core::PixelType::UInt8 && !target.bytes && target.storage_width==target.width && target.storage_height==target.height && candidates.size()>=4) {
     statistics.prefix_sum.reset_for_overwrite((target.width+1)*target.height);statistics.prefix_squared.reset_for_overwrite(statistics.prefix_sum.size());
     for(std::size_t r=0;r<target.height;++r) {
       const auto begin=r*(target.width+1);double sum=0,sq=0;statistics.prefix_sum[begin]=0;statistics.prefix_squared[begin]=0;
       for(std::size_t c=0;c<target.width;++c){const auto q=target.data()[r*target.width+c];sum+=q;sq+=q*q;statistics.prefix_sum[begin+c+1]=sum;statistics.prefix_squared[begin+c+1]=sq;}
     }
   }
   // 相邻粗角假设常访问同一细晶格；缓存相同栅格评分，连续拟合不变。
   const bool memoize=cache && (image.has_reduced_domain() || candidates.size()>=32);
   std::vector<ScorePlane> memo;
   if(memoize)for(const auto* patch:query_levels[l-1].patches) {
     const int width=std::max(0,int(target.width)-patch->column_max+patch->column_min);
     const int height=std::max(0,int(target.height)-patch->row_max+patch->row_min);
     memo.emplace_back(-patch->row_min,-patch->column_min,width,height,scratch,&cache->score_tile_bytes);
   }
#if MVKIT_NCC_HAS_AVX2
   std::vector<PrefixMomentsCache::Entry*> shared_finer_moments;
   if(share_prefix && share_all_prefix && statistics.prefix_sum.size()) {
     shared_finer_moments.reserve(query_levels[l-1].patches.size());
     for(const auto* patch:query_levels[l-1].patches)
       shared_finer_moments.push_back(cache->shared_prefix_moments.get(target.data(),target.width,target.height,*patch));
   }
#endif
   for(auto& candidate:candidates) {
    const auto& level=model.levels[l-1];const auto& target=pyramid[l-1];const auto step=std::ldexp(1.,int(l-1)),offset=(step-1)*.5;
    Match best=candidate;best.score=missing;
    std::size_t patch_index=0;
    for(const auto* planned_patch:query_levels[l-1].patches) {
#if MVKIT_NCC_HAS_AVX2
      auto* shared_moments=shared_finer_moments.empty()?nullptr:shared_finer_moments[patch_index];
#endif
      auto* saved=memoize?&memo[patch_index]:nullptr;++patch_index;
      const auto& patch=*planned_patch;
      if(std::abs(patch.angle-candidate.angle)>model.options.angle_step*(l==top?angle_stride:1)*1.01)continue;
      const auto row=int(std::round((candidate.row-crop_row-offset)/step-patch.origin_row)),column=int(std::round((candidate.column-crop_column-offset)/step-patch.origin_column));
      for(int dr=-2;dr<=2;++dr) {
       if(saved && image.has_reduced_domain()) {
           for(int dc=-2;dc<=2;++dc){
             auto* value=saved->at(row+dr,column+dc,true);
             double score=missing;
             Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column+dc+patch.origin_column)+offset,patch.angle,score};
             if(!value || std::isnan(*value)) {
               double computed=missing;
               if(may_contain(pose,step)) {
#if MVKIT_NCC_HAS_AVX2
                 if(vectorize && statistics.prefix_sum.size() && row+dr+patch.row_min>=0 && row+dr+patch.row_max<int(target.height) && column+dc+patch.column_min>=0 && column+dc+patch.column_max<int(target.width)) {
                   auto* moments=shared_moments && row+dr>=0 && row+dr<int(target.height) && column+dc>=0 && column+dc<int(target.width)?shared_moments:nullptr;
                   const auto index=(row+dr)*target.width+column+dc;
                   computed=correlation_one_prefix_avx2(target.data(),statistics.prefix_sum.data(),statistics.prefix_squared.data(),target.width,patch,row+dr,column+dc,model.options.ignore_global_polarity,prepared->exact_byte_statistics && l<=3,
                       moments?moments->sum.data()+index:nullptr,moments?moments->squared.data()+index:nullptr);
                 }
                 else
#endif
                 computed=discrete(target,patch,row+dr,column+dc,model.options.ignore_global_polarity,vectorize);
               }
               if(value)*value=computed;pose.score=computed;
             }else pose.score=*value;
             if(pose.score>best.score)best=pose;
           }
           continue;
       }
       if(saved) {
         bool complete=true;std::array<double,5> cached{};
         for(int lane=0;lane<5;++lane){auto* q=saved->at(row+dr,column-2+lane);if(!q || std::isnan(*q)){complete=false;break;}cached[lane]=*q;}
         if(complete){for(int lane=0;lane<5;++lane){Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column-2+lane+patch.origin_column)+offset,patch.angle,cached[lane]};if(may_contain(pose,step) && pose.score>best.score)best=pose;}continue;}
       }
       int dc=-2;
#if MVKIT_NCC_HAS_AVX2
       if(vectorize && statistics.prefix_sum.size() && row+dr+patch.row_min>=0 && row+dr+patch.row_max<int(target.height) &&
          column-2+patch.column_min>=0 && column+5+patch.column_max<int(target.width)) {
         double values[8];
         auto* moments=shared_moments && row+dr>=0 && row+dr<int(target.height) && column>=2 && column+5<int(target.width)?shared_moments:nullptr;
         const auto index=(row+dr)*target.width+column-2;
         correlation_eight_prefix_avx2(target.data(),statistics.prefix_sum.data(),statistics.prefix_squared.data(),target.width,patch,row+dr,column-2,model.options.ignore_global_polarity,values,prepared->exact_byte_statistics && l<=3,
             moments?moments->sum.data()+index:nullptr,moments?moments->squared.data()+index:nullptr);
         if(saved)for(int lane=0;lane<8;++lane)if(auto* q=saved->at(row+dr,column-2+lane,true)) {
           Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column-2+lane+patch.origin_column)+offset,patch.angle,values[lane]};
           if(std::isnan(*q))*q=may_contain(pose,step)?values[lane]:missing;
           values[lane]=*q;
         }
         for(int lane=0;lane<5;++lane){Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column-2+lane+patch.origin_column)+offset,patch.angle,values[lane]};if(may_contain(pose,step) && pose.score>best.score)best=pose;}
         dc=3;
       }else if(vectorize && !target.bytes && row+dr+patch.row_min>=int(target.storage_row) && row+dr+patch.row_max<int(target.storage_row+target.storage_height) &&
          column-2+patch.column_min>=int(target.storage_column) && column+1+patch.column_max<int(target.storage_column+target.storage_width)) {
         double values[4];
         auto* moments=shared_moments && row+dr>=0 && row+dr<int(target.height) && column>=2 && column+1<int(target.width)?shared_moments:nullptr;
         const auto index=(row+dr)*target.width+column-2;
         if(statistics.prefix_sum.size())correlation_four_prefix_avx2(target.data(),statistics.prefix_sum.data(),statistics.prefix_squared.data(),target.width,patch,row+dr,column-2,model.options.ignore_global_polarity,values,prepared->exact_byte_statistics && l<=3,
             moments?moments->sum.data()+index:nullptr,moments?moments->squared.data()+index:nullptr);
         else correlation_four_avx2(target.data(),target.storage_width,patch,row+dr-int(target.storage_row),column-2-int(target.storage_column),model.options.ignore_global_polarity,values);
         if(saved)for(int lane=0;lane<4;++lane)if(auto* q=saved->at(row+dr,column-2+lane,true)) {
           Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column-2+lane+patch.origin_column)+offset,patch.angle,values[lane]};
           if(std::isnan(*q))*q=may_contain(pose,step)?values[lane]:missing;
           values[lane]=*q;
         }
         for(int lane=0;lane<4;++lane){Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column-2+lane+patch.origin_column)+offset,patch.angle,values[lane]};if(may_contain(pose,step) && pose.score>best.score)best=pose;}
         dc=2;
       }
#endif
       for(;dc<=2;++dc) {
        Match pose{crop_row+step*(row+dr+patch.origin_row)+offset,crop_column+step*(column+dc+patch.origin_column)+offset,patch.angle,0};
        if(!may_contain(pose,step))continue;
        auto* q=saved?saved->at(row+dr,column+dc,true):nullptr;
        if(q && !std::isnan(*q))pose.score=*q;
        else {pose.score=discrete(target,patch,row+dr,column+dc,model.options.ignore_global_polarity,vectorize);if(q)*q=pose.score;}
        if(pose.score>best.score)best=pose;
       }
      }
    }
    candidate=best;
   }
   merge_identical(candidates);
   // 各层过渡保留同一宽松金字塔阈值；细层失去相关性的假设不能耗尽后续全分辨率拟合，最终 min_score 不变。
   candidates.erase(std::remove_if(candidates.begin(),candidates.end(),[&](const Match& p){return !std::isfinite(p.score) || p.score<options.min_score*.4;}),candidates.end());
  }
  const auto finer_done=std::chrono::steady_clock::now();
  prepared->memoize_refinement=cache && candidates.size()>=32;
  const auto& original=finest?*prepared->coarse_original:model.original;
#if MVKIT_NCC_HAS_AVX2
  // 坐标计算前证明完整 32 样本块为常量；前缀归不可变查询像素，
  // 块框归此精确有序原 Patch，预算耗尽时回退。
  const auto* zero_setting=std::getenv("MVKIT_NCC_ZERO_BLOCKS");
  const auto* zero_block_setting=std::getenv("MVKIT_NCC_ZERO_BLOCK_SIZE");
  const auto* zero_mode=std::getenv("MVKIT_NCC_ZERO_BLOCK_MODE");
  const bool spatial_zero=zero_mode?!std::strcmp(zero_mode,"tiles"):cache && cache->model_count>=4;
  const std::size_t zero_block_size=zero_block_setting && !std::strcmp(zero_block_setting,"16")?16:32;
  auto& zero_source=pyramid.front();
  constexpr std::size_t zero_image_limit=64*1024*1024,zero_sample_limit=32*1024*1024;
  if(!finest && cache && vectorize && prepared->exact_byte_statistics && prepared->memoize_refinement &&
      options.subpixel && model.levels.size()>1 && original.samples.size()<=32768 &&
      zero_source.shared_values && !zero_source.bytes && (!zero_setting || std::strcmp(zero_setting,"0")) &&
      (!image.has_reduced_domain() || zero_setting)) {
    const auto pixels=zero_source.data();
    auto found=cache->zero_images.find(pixels);
    const auto prefix_bytes=4*(zero_source.storage_width+1)*(zero_source.storage_height+1);
    if(found==cache->zero_images.end() && cache->zero_images.size()<64 &&
        prefix_bytes+512<=zero_image_limit && cache->zero_image_bytes<=zero_image_limit-prefix_bytes-512) {
      auto built=std::make_unique<BilinearZeroImage>(pixels,zero_source.storage_width,zero_source.storage_height,cache->context.scratch_resource());
      cache->zero_image_bytes+=built->allocation_bytes()+512;
      found=cache->zero_images.emplace(pixels,std::move(built)).first;
    }
    // 每块最多四个边界及有界元数据，预算同时为样本视图拥有者和载荷留余量。
    const auto sample_estimate=spatial_zero?original.samples.size()*32+2048:(original.samples.size()/16)*64+512;
    if(found!=cache->zero_images.end() && found->second->valid() &&
        sample_estimate<=zero_sample_limit && cache->zero_sample_bytes<=zero_sample_limit-sample_estimate) {
      prepared->zero_samples=std::make_unique<BilinearZeroSamples>(original,cache->context.scratch_resource(),zero_block_size,spatial_zero);
      if(prepared->zero_samples->spatial_tiles()) {
        try {prepared->zero_tile_flags.reset_for_overwrite(prepared->zero_samples->tile_count());}
        catch(const std::bad_alloc&) {prepared->zero_tile_flags.reset_for_overwrite(0);}
      }
      cache->zero_sample_bytes+=prepared->zero_samples->allocation_bytes()+512;
      cache->zero_sample_bytes+=prepared->zero_tile_flags.size();
      if(prepared->zero_samples->valid()) {
        prepared->zero_view.image=found->second.get();prepared->zero_view.samples=prepared->zero_samples.get();
        prepared->zero_view.tile_flags=prepared->zero_tile_flags.data();
        prepared->zero_view.tile_flags_capacity=prepared->zero_tile_flags.size();
        const auto* proof_setting=std::getenv("MVKIT_NCC_ZERO_PROOF_CACHE");
        if(!proof_setting || std::strcmp(proof_setting,"0"))prepared->zero_view.proof_cache=&prepared->zero_proof_cache;
        if(std::getenv("MVKIT_NCC_DIAGNOSTICS")) {
          prepared->zero_view.tested_blocks=&cache->zero_tested_blocks;
          prepared->zero_view.skipped_blocks=&cache->zero_skipped_blocks;
        }
      }
    }
  }
#endif
  const auto full_score=[&](Match pose) {
    const auto scale=std::ldexp(1.,int(finest)),phase=(scale-1)*.5;
    pose.row=(pose.row-crop_row-phase)/scale;pose.column=(pose.column-crop_column-phase)/scale;
    return continuous(pyramid[finest],model.levels[finest],original,pose,model.options.ignore_global_polarity,vectorize,finest?nullptr:prepared->samples,crop_row,crop_column
#if MVKIT_NCC_HAS_AVX2
        ,&prepared->zero_view
#endif
        );
  };
  for(auto& candidate:candidates)if(std::isfinite(candidate.score))candidate.score=full_score(candidate);
  if(std::getenv("MVKIT_NCC_DIAGNOSTICS"))std::fprintf(stderr,"ncc stages coarse=%.3f finer=%.3f full=%.3f candidates=%zu\n",std::chrono::duration<double,std::micro>(coarse_done-tick).count(),std::chrono::duration<double,std::micro>(finer_done-coarse_done).count(),std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-finer_done).count(),candidates.size());
  std::sort(candidates.begin(),candidates.end(),[](const Match& a,const Match& b) {
    if(a.score!=b.score)return a.score>b.score;
    if(a.row!=b.row)return a.row<b.row;if(a.column!=b.column)return a.column<b.column;return a.angle<b.angle;
  });
  prepared->crop_row=crop_row;prepared->crop_column=crop_column;
  const auto& source=pyramid[finest];
  prepared->fine=std::make_unique<Gray>(source.width,source.height,scratch,true);
  auto& fine=*prepared->fine;fine.bytes=source.bytes;fine.stride=source.stride;fine.dc=source.dc;
  fine.shared_values=source.shared_values;
  fine.storage_row=source.storage_row;fine.storage_column=source.storage_column;
  fine.storage_width=source.storage_width;fine.storage_height=source.storage_height;
  if(!source.shared_values){fine.values.reset_for_overwrite(source.values.size());
  if(source.values.size())std::memcpy(fine.values.data(),source.values.data(),source.values.size()*sizeof(double));}
  // 这是经验验证的稠密搜索策略，不是相关上界。密度与缓存独立；
  // 别名模型即使候选少也可启用缓存，不支持的搜索保留六步。
  prepared->default_background_guard=!finest && cache && cache->cache_derivatives && cache->unlimited_selection &&
      candidates.size()>=32 && prepared->exact_byte_statistics && vectorize && options.subpixel &&
      !image.has_reduced_domain() &&
      !options.num_matches && options.min_score>=.75 && options.min_score<=.85 &&
      level_count==3 && !model.options.ignore_global_polarity &&
      model.options.angle_step<=.02 && options.angle_extent<=.4 &&
      std::fegetround()==FE_TONEAREST &&
      original.samples.size()<=32768 && fine.shared_values && !fine.bytes;
  if(cache && prepared->memoize_refinement && options.subpixel && model.levels.size()>1) {
    prepared->fit_trace=std::getenv("MVKIT_NCC_FIT_TRACE")!=nullptr;
  }
  return prepared;
}
// 更粗最低层使用自己的样本几何与评分，不评估全分辨率模板或使用零层缓存。
static bool refine_coarse_candidate(PreparedQuery& query,Match& candidate) {
  const auto& model=*query.model;const auto& options=query.options;
  const auto& level=model.levels[query.finest_level];const auto& original=*query.coarse_original;
  const auto scale=std::ldexp(1.,int(query.finest_level)),phase=(scale-1)*.5;
  const bool vectorize=query.context.cpu().selection().avx2;
  const auto local=[&](Match pose){pose.row=(pose.row-query.crop_row-phase)/scale;pose.column=(pose.column-query.crop_column-phase)/scale;return pose;};
  const auto score=[&](const Match& pose){return continuous(*query.fine,level,original,local(pose),model.options.ignore_global_polarity,vectorize);};
  if(options.subpixel)for(int iteration=0;iteration<6;++iteration) {
    double delta[3]{};
    if(!refinement_step(*query.fine,level,original,local(candidate),model.options.ignore_global_polarity,options.angle_extent>0,delta,vectorize))break;
    const auto factor=std::min({1.,.75/std::max(1e-20,std::abs(delta[0])),.75/std::max(1e-20,std::abs(delta[1])),
        options.angle_extent>0?model.options.angle_step/std::max(1e-20,std::abs(delta[2])):1.});
    bool accepted=false;
    for(int backtrack=0;backtrack<4;++backtrack) {
      const auto step=std::ldexp(factor,-backtrack);auto next=candidate;
      next.row+=scale*step*delta[0];next.column+=scale*step*delta[1];
      next.angle=std::clamp(next.angle+step*delta[2],options.angle_start,options.angle_start+options.angle_extent);
      if(!domain_contains(query.domain,next))continue;
      next.score=score(next);
      if(next.score>candidate.score){candidate=next;accepted=true;break;}
    }
    if(!accepted)break;
  }
  return candidate.score>=options.min_score && domain_contains(query.domain,candidate);
}
static bool refine_candidate(PreparedQuery& query,Match& candidate) {
  ++query.refine_calls;
  if(query.finest_level)return refine_coarse_candidate(query,candidate);
  const auto& model=*query.model;const auto& options=query.options;const auto& domain=query.domain;
  const auto crop_row=query.crop_row,crop_column=query.crop_column;const auto top=query.level_count-1;
  const auto angle_end=options.angle_start+options.angle_extent;
  const bool vectorize=query.context.cpu().selection().avx2;
  auto& query_levels=query.query_levels;const auto& original=model.original;
  double* workspace=nullptr;
#if MVKIT_NCC_HAS_AVX2
  if(vectorize && original.samples.size()<=32768 && options.subpixel && query.cache &&
      query.cache->cache_derivatives && query.samples && query.memoize_refinement && !options.num_matches) {
    const auto size=4*original.samples.size();
    if(query.derivative_workspace.size()!=size)query.derivative_workspace.reset_for_overwrite(size);
    workspace=query.derivative_workspace.data();
  }
#endif
  const auto full_score=[&](Match pose) {
    pose.row-=crop_row;pose.column-=crop_column;
    return continuous(*query.fine,model.levels.front(),original,pose,model.options.ignore_global_polarity,vectorize,nullptr,0,0
#if MVKIT_NCC_HAS_AVX2
        ,&query.zero_view
#endif
        );
  };
  const auto fit=[&](Match& candidate) {
    ++query.fit_calls;
    const PreparedQuery::FitKey key{candidate.row,candidate.column,candidate.angle,candidate.score};
    if(query.memoize_refinement) {
      const auto saved=query.fitted.find(key);
      if(saved!=query.fitted.end()){++query.fit_hits;candidate=saved->second;return;}
    }
    const auto fit_seed=candidate;
    constexpr int fine_iterations=6;
    double background_guard=-1;
    double first_background_guard=-1;
    const auto* guard_setting=std::getenv("MVKIT_NCC_BACKGROUND_GUARD");
    if(query.memoize_refinement && guard_setting) {
      const auto* setting=guard_setting;
      if(!std::strcmp(setting,"0.2"))background_guard=.2;
      else if(!std::strcmp(setting,"0.1"))background_guard=.1;
      else if(!std::strcmp(setting,"staged")){first_background_guard=.3;background_guard=.2;}
      else if(!std::strcmp(setting,"staged01")){first_background_guard=.3;background_guard=.1;}
      else if(!std::strcmp(setting,"staged21")){first_background_guard=.2;background_guard=.1;}
      else if(!std::strcmp(setting,"staged11") && query.default_background_guard) {
        first_background_guard=options.min_score>=.8 ? .1 : .2;background_guard=.1;
      }
    }else if(!guard_setting && query.default_background_guard) {
      first_background_guard=options.min_score>=.8 ? .1 : .2;background_guard=.1;
    }
    bool cached=false;double next_delta[3]{};
    RefinementMemo memo;memo.crop_row=crop_row;memo.crop_column=crop_column;
#if MVKIT_NCC_HAS_AVX2
    if(workspace) {
      memo.cache=&query.cache->derivatives;memo.requests=&query.cache->derivative_requests;memo.hits=&query.cache->derivative_hits;
    }
#endif
    if(options.subpixel)for(int iteration=0;iteration<fine_iterations;++iteration) {
      auto local=candidate;local.row-=crop_row;local.column-=crop_column;double delta[3]{};
      if(cached)std::copy(next_delta,next_delta+3,delta);
      else {
        ++query.derivative_calls;
        if(!refinement_step(*query.fine,model.levels.front(),original,local,model.options.ignore_global_polarity,options.angle_extent>0,delta,vectorize,nullptr,iteration==0?&memo:nullptr,workspace,nullptr
#if MVKIT_NCC_HAS_AVX2
            ,&query.zero_view
#endif
            ))break;
      }
      cached=false;
      const auto factor=std::min({1.,.75/std::max(1e-20,std::abs(delta[0])),.75/std::max(1e-20,std::abs(delta[1])),
          options.angle_extent>0?model.options.angle_step/std::max(1e-20,std::abs(delta[2])):1.});
      bool accepted=false;
      for(int backtrack=0;backtrack<4;++backtrack) {
        const auto step=std::ldexp(factor,-backtrack);auto improved=candidate;
        improved.row+=step*delta[0];improved.column+=step*delta[1];
        improved.angle=std::clamp(improved.angle+step*delta[2],options.angle_start,angle_end);
        if(!domain_contains(domain,improved))continue;
        bool next_valid=false;
        // 守卫只需精确评分；继续的候选下轮再求导数，终止的候选不算无用 Hessian。
        // 接受时评分严格递增，已达本轮守卫阈值则守卫不能终止拟合，保留原评分/导数联合提案。
        const bool guard_score_only=(iteration==0 && first_background_guard>=0 &&
            candidate.score<options.min_score-first_background_guard) ||
            (iteration==1 && background_guard>=0 && candidate.score<options.min_score-background_guard);
        if(backtrack==0 && iteration+1<fine_iterations &&
            !guard_score_only) {
          ++query.derivative_calls;++query.proposal_calls;
          auto next_local=improved;next_local.row-=crop_row;next_local.column-=crop_column;improved.score=missing;
          next_valid=refinement_step(*query.fine,model.levels.front(),original,next_local,model.options.ignore_global_polarity,options.angle_extent>0,next_delta,vectorize,&improved.score,nullptr,workspace,&candidate.score
#if MVKIT_NCC_HAS_AVX2
              ,&query.zero_view
#endif
              );
          if(!std::isfinite(improved.score))improved.score=full_score(improved);
          if(!(improved.score>candidate.score))++query.proposal_rejected;
        }else improved.score=full_score(improved);
        if(improved.score>candidate.score){candidate=improved;accepted=true;cached=next_valid;break;}
      }
      if(query.fit_trace)std::fprintf(stderr,"ncc_fit,%zu,%.17g,%.17g,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g,%d\n",query.model_index,
          fit_seed.row,fit_seed.column,fit_seed.angle,fit_seed.score,iteration+1,candidate.row,candidate.column,candidate.angle,candidate.score,int(accepted));
      if(iteration==1 && background_guard>=0 && candidate.score<options.min_score-background_guard)break;
      if(iteration==0 && first_background_guard>=0 && candidate.score<options.min_score-first_background_guard)break;
      if(!accepted || (std::hypot(delta[0],delta[1])<1e-4 && std::abs(delta[2])<1e-5))break;
    }
    if(query.memoize_refinement && query.fitted.size()<16384)query.fitted.emplace(key,candidate);
  };
    const auto seed=candidate;
    const bool recover_first=query.cache && options.subpixel && top>0 && candidate.score<options.min_score;
    if(!recover_first)fit(candidate);
    if(options.subpixel && top>0 && candidate.score<options.min_score) {
      // 锐边或裁剪支撑可能使局部线性初值无效，此时恢复离散最细层邻域。
      const auto& target=*query.fine;Match best=candidate;
      const PrefixStatistics* statistics=nullptr;
#if MVKIT_NCC_HAS_AVX2
      if(query.cache && vectorize && target.shared_values && target.storage_width==target.width && target.storage_height==target.height) {
        auto& saved=query.cache->fine_statistics[target.data()];
        if(!saved) {
          auto built=std::make_shared<PrefixStatistics>(query.cache->context.scratch_resource());
          built->prefix_sum.reset_for_overwrite((target.width+1)*target.height);built->prefix_squared.reset_for_overwrite(built->prefix_sum.size());
          for(std::size_t r=0;r<target.height;++r) {
            const auto begin=r*(target.width+1);double sum=0,sq=0;built->prefix_sum[begin]=0;built->prefix_squared[begin]=0;
            for(std::size_t c=0;c<target.width;++c){const auto q=target.data()[r*target.width+c];sum+=q;sq+=q*q;built->prefix_sum[begin+c+1]=sum;built->prefix_squared[begin+c+1]=sq;}
          }
          saved=std::move(built);
        }
        statistics=saved.get();
      }
#endif
      for(const auto* patch:query_levels.front().patches) {
        if(std::abs(patch->angle-seed.angle)>model.options.angle_step*1.01)continue;
        ScorePlane* plane=nullptr;
        if(query.memoize_refinement) {
          const auto width=std::max(0,int(target.storage_width)-patch->column_max+patch->column_min);
          const auto height=std::max(0,int(target.storage_height)-patch->row_max+patch->row_min);
          auto saved=query.raster_scores.try_emplace(patch,int(target.storage_row)-patch->row_min,int(target.storage_column)-patch->column_min,width,height,query.context.scratch_resource(),&query.cache->score_tile_bytes);
          plane=&saved.first->second;
        }
        const auto r=int(std::round(seed.row-crop_row-patch->origin_row)),c=int(std::round(seed.column-crop_column-patch->origin_column));
        for(int dr=-2;dr<=2;++dr) {
         int dc=-2;
#if MVKIT_NCC_HAS_AVX2
         if(vectorize && !target.bytes && r+dr+patch->row_min>=int(target.storage_row) && r+dr+patch->row_max<int(target.storage_row+target.storage_height) &&
            c-2+patch->column_min>=int(target.storage_column) && c+1+patch->column_max<int(target.storage_column+target.storage_width)) {
           std::array<double,4> values;std::array<double*,4> saved{};bool complete=plane!=nullptr;
           for(int lane=0;lane<4;++lane) {
             saved[lane]=plane?plane->at(r+dr,c-2+lane,true):nullptr;
             if(saved[lane] && !std::isnan(*saved[lane])){values[lane]=*saved[lane];++query.raster_hits;}
             else complete=false;
           }
           if(!complete) {
             std::array<double,4> computed;
             if(statistics)correlation_four_prefix_avx2(target.data(),statistics->prefix_sum.data(),statistics->prefix_squared.data(),target.storage_width,*patch,r+dr-int(target.storage_row),c-2-int(target.storage_column),model.options.ignore_global_polarity,computed.data(),query.exact_byte_statistics);
             else correlation_four_avx2(target.data(),target.storage_width,*patch,r+dr-int(target.storage_row),c-2-int(target.storage_column),model.options.ignore_global_polarity,computed.data());
             for(int lane=0;lane<4;++lane) {
               if(!saved[lane] || std::isnan(*saved[lane])){values[lane]=computed[lane];if(saved[lane])*saved[lane]=values[lane];}
             }
           }
           for(int lane=0;lane<4;++lane){Match p{crop_row+r+dr+patch->origin_row,crop_column+c-2+lane+patch->origin_column,patch->angle,values[lane]};if(domain_contains(domain,p) && p.score>best.score)best=p;}
           dc=2;
         }
#endif
         for(;dc<=2;++dc) {
          Match p{crop_row+r+dr+patch->origin_row,crop_column+c+dc+patch->origin_column,patch->angle,0};if(!domain_contains(domain,p))continue;
          auto* saved=plane?plane->at(r+dr,c+dc,true):nullptr;
          if(saved && !std::isnan(*saved)){++query.raster_hits;p.score=*saved;}
          else {
#if MVKIT_NCC_HAS_AVX2
            if(statistics && r+dr+patch->row_min>=int(target.storage_row) && r+dr+patch->row_max<int(target.storage_row+target.storage_height) &&
                c+dc+patch->column_min>=int(target.storage_column) && c+dc+patch->column_max<int(target.storage_column+target.storage_width))
              p.score=correlation_one_prefix_avx2(target.data(),statistics->prefix_sum.data(),statistics->prefix_squared.data(),target.storage_width,*patch,r+dr-int(target.storage_row),c+dc-int(target.storage_column),model.options.ignore_global_polarity,query.exact_byte_statistics);
            else
#endif
              p.score=discrete(target,*patch,r+dr,c+dc,model.options.ignore_global_polarity,vectorize);
            if(saved)*saved=p.score;
          }
          if(p.score>best.score)best=p;
         }
        }
      }
      if(best.row!=candidate.row || best.column!=candidate.column || best.angle!=candidate.angle) {
        best.score=full_score(best);fit(best);if(best.score>candidate.score)candidate=best;
      }else if(recover_first)fit(candidate);
    }
  return candidate.score>=options.min_score && domain_contains(domain,candidate);
}

Matches find(const core::Image& image,const Model& model,const FindOptions& options) {
  auto query=prepare(image,model,options);Matches result{runtime::persist_resource()};
  for(auto candidate:query->candidates) {
    if(!std::isfinite(candidate.score))continue;
    bool duplicate=false;for(const auto& p:result)if(overlap(candidate,p,model.levels.front())>options.max_overlap+1e-12){duplicate=true;break;}
    if(duplicate || !refine_candidate(*query,candidate))continue;
    duplicate=false;for(const auto& p:result)if(overlap(candidate,p,model.levels.front())>options.max_overlap+1e-12){duplicate=true;break;}
    if(duplicate)continue;result.push_back(candidate);if(options.num_matches && result.size()==options.num_matches)break;
  }
  std::sort(result.begin(),result.end(),[](const Match& a,const Match& b){
    if(a.score!=b.score)return a.score>b.score;if(a.row!=b.row)return a.row<b.row;
    if(a.column!=b.column)return a.column<b.column;return a.angle<b.angle;
  });return result;
}
std::vector<matching::Match<Match>> find_models(const core::Image& image,
    std::span<const Model* const> models,std::span<const FindOptions> options,matching::Selection selection,bool per_model_quota_before_overlap) {
  QueryCache cache;cache.cache_samples=models.size()>1;std::vector<matching::Match<Match>> candidates;
  cache.model_count=models.size();cache.unlimited_selection=selection.num_matches==0;
  cache.cache_derivatives=std::any_of(models.begin(),models.end(),[&](const Model* p){return p!=models.front();});
  std::vector<std::unique_ptr<PreparedQuery>> queries;queries.reserve(models.size());
  std::vector<std::size_t> owners;owners.reserve(models.size());
  const auto same_options=[](const FindOptions& a,const FindOptions& b) {
    return a.angle_start==b.angle_start && a.angle_extent==b.angle_extent && a.min_score==b.min_score &&
        a.max_overlap==b.max_overlap && a.num_matches==b.num_matches && a.max_candidates==b.max_candidates && a.subpixel==b.subpixel && a.highest_level==b.highest_level && a.lowest_level==b.lowest_level;
  };
  for(std::size_t i=0;i<models.size();++i) {
    std::size_t owner=i;
    for(std::size_t j=0;j<i;++j)if(models[i]==models[j] && same_options(options[i],options[j])){owner=owners[j];break;}
    owners.push_back(owner);
    if(owner!=i) {
      auto& query=*queries[owner];query.memoize_refinement=true;
      for(const auto& pose:query.candidates)if(std::isfinite(pose.score))candidates.push_back({pose,i});
      queries.push_back(nullptr);continue;
    }
    auto query=prepare(image,*models[i],options[i],&cache);
    query->model_index=i;
    for(const auto& pose:query->candidates)if(std::isfinite(pose.score))candidates.push_back({pose,i});
    queries.push_back(std::move(query));
  }
  const auto make_box=[&](const auto& candidate){
    const auto& l=models[candidate.model_index]->levels.front();
    return matching::box(matching::Bounds{-l.origin_row-.5,double(l.height)-l.origin_row-.5,
        -l.origin_column-.5,double(l.width)-l.origin_column-.5},candidate.pose);
  };
  const auto refine=[&](auto& candidate){return refine_candidate(*queries[owners[candidate.model_index]],candidate.pose);};
  // HALCON 先限制独立模型结果，再跨模型抑制；被抑制匹配不触发配额补齐。
  const bool independent_quota=per_model_quota_before_overlap && selection.num_matches<0 && selection.max_overlap>=0 &&
      std::any_of(options.begin(),options.end(),[](const auto& q){return q.num_matches>0;});
  auto result=matching::select_refined(std::move(candidates),options,independent_quota?matching::Selection{-1,-1}:selection,make_box,refine);
  if(independent_quota)result=matching::select(std::move(result),options,selection,make_box);
  if(std::getenv("MVKIT_NCC_DIAGNOSTICS"))for(std::size_t i=0;i<queries.size();++i)if(queries[i]) {
    const auto& q=*queries[i];std::fprintf(stderr,"ncc refinement model=%zu calls=%zu fits=%zu fit_hits=%zu score_planes=%zu raster_hits=%zu score_tile_bytes=%zu\n",i,q.refine_calls,q.fit_calls,q.fit_hits,q.raster_scores.size(),q.raster_hits,cache.score_tile_bytes);
    std::fprintf(stderr,"ncc derivatives model=%zu calls=%zu proposals=%zu rejected=%zu workspace=%zu\n",i,q.derivative_calls,q.proposal_calls,q.proposal_rejected,q.derivative_workspace.size()*sizeof(double));
  }
#if MVKIT_NCC_HAS_AVX2
  if(std::getenv("MVKIT_NCC_DIAGNOSTICS"))std::fprintf(stderr,"ncc prefix_moments requests=%zu geometry_hits=%zu ready_hits=%zu planes=%zu bytes=%zu\n",
      cache.shared_prefix_moments.requests(),cache.shared_prefix_moments.hits(),cache.shared_prefix_moments.ready_hits(),cache.shared_prefix_moments.materializations(),cache.shared_prefix_moments.allocation_bytes());
  if(std::getenv("MVKIT_NCC_DIAGNOSTICS"))std::fprintf(stderr,"ncc derivative_cache requests=%zu hits=%zu entries=%zu bytes=%zu dense_groups=%zu stored_groups=%zu compressed=%zu\n",cache.derivative_requests,cache.derivative_hits,cache.derivatives.size(),cache.derivatives.allocation_bytes(),cache.derivatives.dense_gradient_groups(),cache.derivatives.stored_gradient_groups(),cache.derivatives.compressed_records());
  if(std::getenv("MVKIT_NCC_DIAGNOSTICS"))std::fprintf(stderr,"ncc zero_blocks checked=%zu skipped=%zu image_bytes=%zu sample_bytes=%zu\n",cache.zero_tested_blocks,cache.zero_skipped_blocks,cache.zero_image_bytes,cache.zero_sample_bytes);
#endif
  return result;
}

} // namespace mv::ncc_matching
