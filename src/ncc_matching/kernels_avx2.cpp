#include "kernels_avx2.hpp"
#include "refinement_cache.hpp"
#include "bilinear_zero_blocks.hpp"
#include <immintrin.h>
#include <algorithm>
#include <cmath>
#include <limits>
namespace mv::ncc_matching {
namespace {
struct PrefixMoments {__m256d sum,squared;};
#if defined(_MSC_VER)
__forceinline
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((always_inline)) inline
#else
inline
#endif
PrefixMoments ordered_prefix_moments(const double* sums,const double* squares,std::size_t width,
    const Patch& patch,int row,int column,double* cached_sum,double* cached_squared) {
  if(cached_sum && cached_squared) {
    const auto sum=_mm256_loadu_pd(cached_sum),sq=_mm256_loadu_pd(cached_squared);
    if(_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(sum,sum,_CMP_ORD_Q),_mm256_cmp_pd(sq,sq,_CMP_ORD_Q)))==15)
      return {sum,sq};
  }
  auto sum=_mm256_setzero_pd(),sq=sum;const auto pitch=width+1;
  for(const auto& p:patch.runs) {
    const auto start=(row+p.row)*pitch+column+p.begin,end=(row+p.row)*pitch+column+p.end+1;
    sum=_mm256_add_pd(sum,_mm256_sub_pd(_mm256_loadu_pd(sums+end),_mm256_loadu_pd(sums+start)));
    sq=_mm256_add_pd(sq,_mm256_sub_pd(_mm256_loadu_pd(squares+end),_mm256_loadu_pd(squares+start)));
  }
  if(cached_sum)_mm256_storeu_pd(cached_sum,sum);
  if(cached_squared)_mm256_storeu_pd(cached_squared,sq);
  return {sum,sq};
}
__m256d prefix_scores(__m256d variance,__m256d dot,bool absolute) {
  auto score=_mm256_div_pd(dot,_mm256_sqrt_pd(variance));
  // 第二操作数保留无序评分及带符号零，行为同 std::clamp。
  score=_mm256_min_pd(_mm256_set1_pd(1.),_mm256_max_pd(_mm256_set1_pd(-1.),score));
  return absolute?_mm256_andnot_pd(_mm256_set1_pd(-0.),score):score;
}
}

std::size_t box2_u8_avx2(const std::uint8_t* top,const std::uint8_t* bottom,double* target,std::size_t width,double dc) {
  std::size_t c=0;const auto ones=_mm_set1_epi16(1);const auto quarter=_mm256_set1_pd(.25),offset=_mm256_set1_pd(dc);
  // byte 和及整数 DC 在 Double 中精确，等同标量从左到右的中心化 box 滤波，无需中间图像复制。
  for(;2*c+8<=width;c+=4) {
    const auto a=_mm_cvtepu8_epi16(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(top+2*c)));
    const auto b=_mm_cvtepu8_epi16(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(bottom+2*c)));
    const auto sum=_mm_add_epi32(_mm_madd_epi16(a,ones),_mm_madd_epi16(b,ones));
    _mm256_storeu_pd(target+c,_mm256_sub_pd(_mm256_mul_pd(_mm256_cvtepi32_pd(sum),quarter),offset));
  }return c;
}
std::size_t box2_double_avx2(const double* top,const double* bottom,double* target,std::size_t width) {
  std::size_t c=0;const auto quarter=_mm256_set1_pd(.25);
  for(;2*c+8<=width;c+=4) {
    const auto a=_mm256_loadu_pd(top+2*c),b=_mm256_loadu_pd(top+2*c+4),d=_mm256_loadu_pd(bottom+2*c),e=_mm256_loadu_pd(bottom+2*c+4);
    const auto even_top=_mm256_permute4x64_pd(_mm256_unpacklo_pd(a,b),0xd8),odd_top=_mm256_permute4x64_pd(_mm256_unpackhi_pd(a,b),0xd8);
    const auto even_bottom=_mm256_permute4x64_pd(_mm256_unpacklo_pd(d,e),0xd8),odd_bottom=_mm256_permute4x64_pd(_mm256_unpackhi_pd(d,e),0xd8);
    _mm256_storeu_pd(target+c,_mm256_mul_pd(_mm256_add_pd(_mm256_add_pd(_mm256_add_pd(even_top,odd_top),even_bottom),odd_bottom),quarter));
  }return c;
}
std::size_t gray_u16_avx2(const std::uint16_t* source,double* target,std::size_t count,double offset) {
  std::size_t c=0;const auto dc=_mm256_set1_pd(offset);
  // 先转换再相减，保持原 Double 标量操作；非有限块返回起点以按原序标量检查。
  for(;c+4<=count;c+=4) {
    const auto source_bits=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source+c));
    const auto a=_mm256_cvtepi32_pd(_mm_cvtepu16_epi32(source_bits));
    _mm256_storeu_pd(target+c,_mm256_sub_pd(a,dc));
  }return c;
}
std::size_t gray_i16_avx2(const std::int16_t* source,double* target,std::size_t count,double offset) {
  std::size_t c=0;const auto dc=_mm256_set1_pd(offset);
  // 先转换再相减，保持原 Double 标量操作；非有限块返回起点以按原序标量检查。
  for(;c+4<=count;c+=4) {
    const auto source_bits=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source+c));
    const auto a=_mm256_cvtepi32_pd(_mm_cvtepi16_epi32(source_bits));
    _mm256_storeu_pd(target+c,_mm256_sub_pd(a,dc));
  }return c;
}
std::size_t gray_i32_avx2(const std::int32_t* source,double* target,std::size_t count,double offset) {
  std::size_t c=0;const auto dc=_mm256_set1_pd(offset);
  // 先转换再相减，保持原 Double 标量操作；非有限块返回起点以按原序标量检查。
  for(;c+4<=count;c+=4) {
    const auto a=_mm256_cvtepi32_pd(_mm_loadu_si128(reinterpret_cast<const __m128i*>(source+c)));
    _mm256_storeu_pd(target+c,_mm256_sub_pd(a,dc));
  }return c;
}
std::size_t gray_f32_avx2(const float* source,double* target,std::size_t count,double offset) {
  std::size_t c=0;const auto dc=_mm256_set1_pd(offset);const auto limit=_mm256_set1_pd(std::numeric_limits<double>::max());
  // 先转换再相减，保持原 Double 标量操作；非有限块返回起点以按原序标量检查。
  for(;c+4<=count;c+=4) {
    const auto a=_mm256_cvtps_pd(_mm_loadu_ps(source+c));
    if(_mm256_movemask_pd(_mm256_cmp_pd(_mm256_andnot_pd(_mm256_set1_pd(-0.),a),limit,_CMP_LE_OQ))!=15)break;
    _mm256_storeu_pd(target+c,_mm256_sub_pd(a,dc));
  }return c;
}
std::size_t gray_u8_avx2(const std::uint8_t* source,double* target,std::size_t count,double offset) {
  std::size_t c=0;const auto dc=_mm256_set1_pd(offset);
  for(;c+8<=count;c+=8) {
    const auto v=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source+c)));
    _mm256_storeu_pd(target+c,_mm256_sub_pd(_mm256_cvtepi32_pd(_mm256_castsi256_si128(v)),dc));
    _mm256_storeu_pd(target+c+4,_mm256_sub_pd(_mm256_cvtepi32_pd(_mm256_extracti128_si256(v,1)),dc));
  }return c;
}
void correlation_four_avx2(const double* image,std::size_t width,const Patch& patch,int row,int column,bool absolute,double* scores) {
  const auto& p0=patch.samples.front();
  const auto offset=_mm256_loadu_pd(image+(row+p0.row)*width+column+p0.column);
  auto sum=_mm256_setzero_pd(),squared=sum,dot=sum;
  for(const auto& p:patch.samples) {
    const auto q=_mm256_sub_pd(_mm256_loadu_pd(image+(row+p.row)*width+column+p.column),offset);
    sum=_mm256_add_pd(sum,q);squared=_mm256_add_pd(squared,_mm256_mul_pd(q,q));
    dot=_mm256_add_pd(dot,_mm256_mul_pd(q,_mm256_set1_pd(p.weight)));
  }
  const auto variance=_mm256_sub_pd(squared,_mm256_div_pd(_mm256_mul_pd(sum,sum),_mm256_set1_pd(double(patch.samples.size()))));
  double vs[4],ds[4];_mm256_storeu_pd(vs,variance);_mm256_storeu_pd(ds,dot);
  for(int i=0;i<4;++i){const auto score=vs[i]>0?std::clamp(ds[i]/std::sqrt(vs[i]),-1.,1.):-std::numeric_limits<double>::infinity();scores[i]=absolute && std::isfinite(score)?std::abs(score):score;}
}
double correlation_one_prefix_avx2(const double* image,const double* sums,const double* squares,std::size_t width,const Patch& patch,int row,int column,bool absolute,bool exact_byte_lattice,double* cached_sum,double* cached_squared) {
  double sum=0,sq=0;const auto pitch=width+1;
  if(cached_sum && cached_squared && !std::isnan(*cached_sum) && !std::isnan(*cached_squared)) {
    sum=*cached_sum;sq=*cached_squared;
  }else {
    for(const auto& run:patch.runs){const auto a=(row+run.row)*pitch+column+run.begin,b=(row+run.row)*pitch+column+run.end+1;sum+=sums[b]-sums[a];sq+=squares[b]-squares[a];}
    if(cached_sum)*cached_sum=sum;
    if(cached_squared)*cached_squared=sq;
  }
  const auto count=double(patch.samples.size()),variance=sq-sum*sum/count;
  if(variance<=128*std::numeric_limits<double>::epsilon()*sq)return exact_byte_lattice && width<=32000000 && patch.samples.size()<=1048576 && !patch.runs.empty()?
      -std::numeric_limits<double>::infinity():correlation_avx2(image,width,patch,row,column,absolute);
  if(!patch.weight_runs.empty() && patch.weight_runs.size()*8<patch.samples.size()) {
    double dot=0;
    for(const auto& run:patch.weight_runs){const auto a=(row+run.row)*pitch+column+run.begin,b=(row+run.row)*pitch+column+run.end+1;dot+=(sums[b]-sums[a])*run.weight;}
    const auto score=std::clamp((dot-sum/count*patch.weight_sum)/std::sqrt(variance),-1.,1.);return absolute?std::abs(score):score;
  }
  auto dot=_mm256_setzero_pd();double tail=0;std::size_t index=0;
  for(const auto& run:patch.runs) {
    const auto* values=image+(row+run.row)*width+column+run.begin;const auto length=std::size_t(run.end-run.begin+1);std::size_t i=0;
    for(;i+4<=length;i+=4)dot=_mm256_add_pd(dot,_mm256_mul_pd(_mm256_loadu_pd(values+i),_mm256_loadu_pd(patch.weights.data()+index+i)));
    for(;i<length;++i)tail+=values[i]*patch.weights[index+i];index+=length;
  }
  double parts[4];_mm256_storeu_pd(parts,dot);
  const auto numerator=(parts[0]+parts[1])+(parts[2]+parts[3])+tail-sum/count*patch.weight_sum;
  const auto score=std::clamp(numerator/std::sqrt(variance),-1.,1.);return absolute?std::abs(score):score;
}
void correlation_four_prefix_avx2(const double* image,const double* sums,const double* squares,std::size_t width,const Patch& patch,int row,int column,bool absolute,double* scores,bool exact_byte_lattice,double* cached_sum,double* cached_squared) {
  const auto moments=ordered_prefix_moments(sums,squares,width,patch,row,column,cached_sum,cached_squared);
  const auto sum=moments.sum,sq=moments.squared;auto dot=_mm256_setzero_pd();const auto pitch=width+1;
  auto dot1=dot,dot2=dot,dot3=dot;std::size_t i=0;
  const auto accumulate=[&](std::size_t index,__m256d& acc){const auto& p=patch.samples[index];acc=_mm256_add_pd(acc,_mm256_mul_pd(_mm256_loadu_pd(image+(row+p.row)*width+column+p.column),_mm256_set1_pd(p.weight)));};
  if(!patch.weight_runs.empty() && patch.weight_runs.size()*2<patch.samples.size()) {
    for(const auto& run:patch.weight_runs){const auto a=(row+run.row)*pitch+column+run.begin,b=(row+run.row)*pitch+column+run.end+1;dot=_mm256_add_pd(dot,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b),_mm256_loadu_pd(sums+a)),_mm256_set1_pd(run.weight)));}i=patch.samples.size();
  }
  for(;i+4<=patch.samples.size();i+=4){accumulate(i,dot);accumulate(i+1,dot1);accumulate(i+2,dot2);accumulate(i+3,dot3);}
  dot=_mm256_add_pd(_mm256_add_pd(dot,dot1),_mm256_add_pd(dot2,dot3));
  for(;i<patch.samples.size();++i)accumulate(i,dot);
  const auto count=_mm256_set1_pd(double(patch.samples.size()));
  const auto variance=_mm256_sub_pd(sq,_mm256_div_pd(_mm256_mul_pd(sum,sum),count));
  dot=_mm256_sub_pd(dot,_mm256_mul_pd(_mm256_div_pd(sum,count),_mm256_set1_pd(patch.weight_sum)));
  const auto cutoff=_mm256_mul_pd(_mm256_set1_pd(128*std::numeric_limits<double>::epsilon()),sq);
  if(_mm256_movemask_pd(_mm256_cmp_pd(variance,cutoff,_CMP_GT_OQ))==15) {
    _mm256_storeu_pd(scores,prefix_scores(variance,dot,absolute));
  }else {
  double vs[4],ds[4],energy[4];_mm256_storeu_pd(vs,variance);_mm256_storeu_pd(ds,dot);_mm256_storeu_pd(energy,sq);
  for(int i=0;i<4;++i) {
    // 病态矩重新计算，不以舍入抹掉真实低对比 byte 窗口。
    if(vs[i]<=128*std::numeric_limits<double>::epsilon()*energy[i]){scores[i]=exact_byte_lattice && width<=32000000 && patch.samples.size()<=1048576 && !patch.runs.empty()?
        -std::numeric_limits<double>::infinity():correlation_avx2(image,width,patch,row,column+i,absolute);continue;}
    const auto score=std::clamp(ds[i]/std::sqrt(vs[i]),-1.,1.);scores[i]=absolute?std::abs(score):score;
  }
  }
}
void correlation_sixteen_prefix_avx2(const double* image,const double* sums,const double* squares,std::size_t width,const Patch& patch,int row,int column,bool absolute,double* scores,bool exact_byte_lattice,double* cached_sum,double* cached_squared) {
  auto d0=_mm256_setzero_pd(),d1=d0,d2=d0,d3=d0;
  // 十六相邻窗口复用各模板地址/权重；独立位置 lane 隐藏点积延迟，不减少样本、角度覆盖或候选阈值。
  if(!patch.weight_runs.empty() && patch.weight_runs.size()*2<patch.samples.size()) {
    for(const auto& run:patch.weight_runs) {
      const auto a=(row+run.row)*(width+1)+column+run.begin,b=(row+run.row)*(width+1)+column+run.end+1;const auto w=_mm256_set1_pd(run.weight);
      d0=_mm256_add_pd(d0,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b),_mm256_loadu_pd(sums+a)),w));
      d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b+4),_mm256_loadu_pd(sums+a+4)),w));
      d2=_mm256_add_pd(d2,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b+8),_mm256_loadu_pd(sums+a+8)),w));
      d3=_mm256_add_pd(d3,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b+12),_mm256_loadu_pd(sums+a+12)),w));
    }
  }else for(const auto& p:patch.samples) {
    const auto* q=image+(row+p.row)*width+column+p.column;const auto w=_mm256_set1_pd(p.weight);
    d0=_mm256_add_pd(d0,_mm256_mul_pd(_mm256_loadu_pd(q),w));d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_loadu_pd(q+4),w));
    d2=_mm256_add_pd(d2,_mm256_mul_pd(_mm256_loadu_pd(q+8),w));d3=_mm256_add_pd(d3,_mm256_mul_pd(_mm256_loadu_pd(q+12),w));
  }
  const __m256d dots[4]={d0,d1,d2,d3};const auto count=_mm256_set1_pd(double(patch.samples.size()));
  for(int block=0;block<4;++block) {
    const auto moments=ordered_prefix_moments(sums,squares,width,patch,row,column+4*block,
        cached_sum?cached_sum+4*block:nullptr,cached_squared?cached_squared+4*block:nullptr);
    const auto sum=moments.sum,sq=moments.squared;
    const auto variance=_mm256_sub_pd(sq,_mm256_div_pd(_mm256_mul_pd(sum,sum),count));
    const auto dot=_mm256_sub_pd(dots[block],_mm256_mul_pd(_mm256_div_pd(sum,count),_mm256_set1_pd(patch.weight_sum)));
      const auto cutoff=_mm256_mul_pd(_mm256_set1_pd(128*std::numeric_limits<double>::epsilon()),sq);
      if(_mm256_movemask_pd(_mm256_cmp_pd(variance,cutoff,_CMP_GT_OQ))==15) {
        _mm256_storeu_pd(scores+4*block,prefix_scores(variance,dot,absolute));
      }else {
    double vs[4],ds[4],energy[4];_mm256_storeu_pd(vs,variance);_mm256_storeu_pd(ds,dot);_mm256_storeu_pd(energy,sq);
    for(int lane=0;lane<4;++lane) {
      const auto index=4*block+lane;
      if(vs[lane]<=128*std::numeric_limits<double>::epsilon()*energy[lane]){scores[index]=exact_byte_lattice && width<=32000000 && patch.samples.size()<=1048576 && !patch.runs.empty()?
          -std::numeric_limits<double>::infinity():correlation_avx2(image,width,patch,row,column+index,absolute);continue;}
      const auto score=std::clamp(ds[lane]/std::sqrt(vs[lane]),-1.,1.);scores[index]=absolute?std::abs(score):score;
    }
      }
  }
}
void correlation_eight_prefix_avx2(const double* image,const double* sums,const double* squares,std::size_t width,const Patch& patch,int row,int column,bool absolute,double* scores,bool exact_byte_lattice,double* cached_sum,double* cached_squared) {
  auto d0=_mm256_setzero_pd(),d1=d0;
  if(!patch.weight_runs.empty() && patch.weight_runs.size()*2<patch.samples.size()) {
    for(const auto& run:patch.weight_runs){const auto a=(row+run.row)*(width+1)+column+run.begin,b=(row+run.row)*(width+1)+column+run.end+1;const auto w=_mm256_set1_pd(run.weight);
      d0=_mm256_add_pd(d0,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b),_mm256_loadu_pd(sums+a)),w));d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_sub_pd(_mm256_loadu_pd(sums+b+4),_mm256_loadu_pd(sums+a+4)),w));}
  }else for(const auto& p:patch.samples) {
    const auto* q=image+(row+p.row)*width+column+p.column;const auto w=_mm256_set1_pd(p.weight);
    d0=_mm256_add_pd(d0,_mm256_mul_pd(_mm256_loadu_pd(q),w));d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_loadu_pd(q+4),w));
  }
  const __m256d dots[2]={d0,d1};const auto count=_mm256_set1_pd(double(patch.samples.size()));
  for(int block=0;block<2;++block) {
    const auto moments=ordered_prefix_moments(sums,squares,width,patch,row,column+4*block,
        cached_sum?cached_sum+4*block:nullptr,cached_squared?cached_squared+4*block:nullptr);
    const auto sum=moments.sum,sq=moments.squared;
    const auto variance=_mm256_sub_pd(sq,_mm256_div_pd(_mm256_mul_pd(sum,sum),count));
    const auto dot=_mm256_sub_pd(dots[block],_mm256_mul_pd(_mm256_div_pd(sum,count),_mm256_set1_pd(patch.weight_sum)));
      const auto cutoff=_mm256_mul_pd(_mm256_set1_pd(128*std::numeric_limits<double>::epsilon()),sq);
      if(_mm256_movemask_pd(_mm256_cmp_pd(variance,cutoff,_CMP_GT_OQ))==15) {
        _mm256_storeu_pd(scores+4*block,prefix_scores(variance,dot,absolute));
      }else {
    double vs[4],ds[4],energy[4];_mm256_storeu_pd(vs,variance);_mm256_storeu_pd(ds,dot);_mm256_storeu_pd(energy,sq);
    for(int lane=0;lane<4;++lane) {
      const auto index=4*block+lane;
      if(vs[lane]<=128*std::numeric_limits<double>::epsilon()*energy[lane]){scores[index]=exact_byte_lattice && width<=32000000 && patch.samples.size()<=1048576 && !patch.runs.empty()?
          -std::numeric_limits<double>::infinity():correlation_avx2(image,width,patch,row,column+index,absolute);continue;}
      const auto score=std::clamp(ds[lane]/std::sqrt(vs[lane]),-1.,1.);scores[index]=absolute?std::abs(score):score;
    }
      }
  }
}
double correlation_avx2(const double* image,std::size_t width,const Patch& patch,int row,int column,bool absolute) {
  const auto& p0=patch.samples.front();const auto dc=image[(row+p0.row)*width+column+p0.column];
  const auto offset=_mm256_set1_pd(dc);auto sum=_mm256_setzero_pd(),squared=sum,dot=sum;std::size_t i=0;
  for(;i+4<=patch.samples.size();i+=4) {
    const auto* p=patch.samples.data()+i;
    const auto indices=_mm256_set_epi64x((row+p[3].row)*width+column+p[3].column,(row+p[2].row)*width+column+p[2].column,
        (row+p[1].row)*width+column+p[1].column,(row+p[0].row)*width+column+p[0].column);
    const bool contiguous=p[0].row==p[3].row && p[1].row==p[0].row && p[2].row==p[0].row &&
        p[1].column==p[0].column+1 && p[2].column==p[0].column+2 && p[3].column==p[0].column+3;
    const auto input=contiguous?_mm256_loadu_pd(image+(row+p[0].row)*width+column+p[0].column):_mm256_i64gather_pd(image,indices,8);
    const auto q=_mm256_sub_pd(input,offset);
    const auto weights=_mm256_loadu_pd(patch.weights.data()+i);
    sum=_mm256_add_pd(sum,q);squared=_mm256_add_pd(squared,_mm256_mul_pd(q,q));dot=_mm256_add_pd(dot,_mm256_mul_pd(q,weights));
  }
  double sums[4],squares[4],dots[4];_mm256_storeu_pd(sums,sum);_mm256_storeu_pd(squares,squared);_mm256_storeu_pd(dots,dot);
  double s=(sums[0]+sums[1])+(sums[2]+sums[3]),sq=(squares[0]+squares[1])+(squares[2]+squares[3]),d=(dots[0]+dots[1])+(dots[2]+dots[3]);
  for(;i<patch.samples.size();++i){const auto& p=patch.samples[i];const auto q=image[(row+p.row)*width+column+p.column]-dc;s+=q;sq+=q*q;d+=q*p.weight;}
  const auto variance=sq-s*s/double(patch.samples.size());if(!(variance>0))return -std::numeric_limits<double>::infinity();
  const auto score=std::clamp(d/std::sqrt(variance),-1.,1.);return absolute?std::abs(score):score;
}
namespace {
double total(__m256d v){double a[4];_mm256_storeu_pd(a,v);return (a[0]+a[1])+(a[2]+a[3]);}
bool strictly_inside(std::size_t width,std::size_t height,const Level& level,const Patch& patch,
    const Match& pose,double ca,double sa) {
  // 旧准备模型样本没有栅格边界；点包围盒不能表达至少四个不同样本的 NCC 模板，
  // 边界准备前保留逐样本检查。
  if(!patch.bounds_prepared || (patch.row_min==patch.row_max && patch.column_min==patch.column_max))return false;
  const double dr[2]={patch.row_min-level.origin_row,patch.row_max-level.origin_row};
  const double dc[2]={patch.column_min-level.origin_column,patch.column_max-level.origin_column};
  double ymin=std::numeric_limits<double>::infinity(),xmin=ymin,ymax=-ymin,xmax=-ymin;
  for(double r:dr)for(double c:dc) {
    const auto y=pose.row+ca*r-sa*c,x=pose.column+sa*r+ca*c;
    ymin=std::min(ymin,y);ymax=std::max(ymax,y);xmin=std::min(xmin,x);xmax=std::max(xmax,x);
  }
  // 样本矩形的仿射像包含全部样本；保留保守浮点余量，晶格边界位姿仍逐样本检查并裁剪邻居。
  const auto margin=64*std::numeric_limits<double>::epsilon()*(1+std::abs(pose.row)+std::abs(pose.column)+
      std::abs(dr[0])+std::abs(dr[1])+std::abs(dc[0])+std::abs(dc[1])+double(width)+double(height));
  return ymin>margin && xmin>margin && ymax<double(height-1)-margin && xmax<double(width-1)-margin;
}
bool zero_gradients(__m256d gy,__m256d gx,__m256d q) {
  return _mm256_movemask_pd(_mm256_cmp_pd(_mm256_or_pd(gy,gx),_mm256_setzero_pd(),_CMP_EQ_OQ))==15 &&
      _mm256_movemask_pd(_mm256_cmp_pd(_mm256_andnot_pd(_mm256_set1_pd(-0.),q),
        _mm256_set1_pd(std::numeric_limits<double>::max()),_CMP_LE_OQ))==15;
}
template<bool Aligned> bool supported_zero_block(const BilinearZeroImage& image,const BilinearZeroBlock& block,
    const Match& pose,double ca,double sa,bool positive_cos,bool positive_sin,double margin) {
  double ymin,ymax,xmin,xmax;
  if constexpr(Aligned) {
    ymin=pose.row+block.row_min;ymax=pose.row+block.row_max;
    xmin=pose.column+block.column_min;xmax=pose.column+block.column_max;
  }else {
    // 直接选择仿射极值，用八次乘法替代十六次角点乘法及 min/max/finite 循环。
    ymin=(pose.row+ca*(positive_cos?block.row_min:block.row_max))-sa*(positive_sin?block.column_max:block.column_min);
    ymax=(pose.row+ca*(positive_cos?block.row_max:block.row_min))-sa*(positive_sin?block.column_min:block.column_max);
    xmin=(pose.column+sa*(positive_sin?block.row_min:block.row_max))+ca*(positive_cos?block.column_min:block.column_max);
    xmax=(pose.column+sa*(positive_sin?block.row_max:block.row_min))+ca*(positive_cos?block.column_max:block.column_min);
  }
  ymin-=margin;xmin-=margin;ymax+=margin;xmax+=margin;
  if(!(ymin>=0 && xmin>=0 && ymax<double(image.height()-1) && xmax<double(image.width()-1)))return false;
  // 包含 floor 坐标单元及双线性右/下邻居。
  return image.rectangle_is_zero(std::size_t(ymin),std::size_t(xmin),std::size_t(ymax)+2,std::size_t(xmax)+2);
}
template<bool Rotate> void derivative_statistics(const double* values,const double* gradients,const Patch& patch,BilinearMoments& out) {
  const auto zero=_mm256_setzero_pd();const auto count=patch.samples.size();constexpr int axes=Rotate?3:2;
  // 九个累加器与样本/权重可留在寄存器；六个 Hessian 累加器另遍归约，保持各 lane 原加法序。
  __m256d gs[3]={zero,zero,zero},qg[3]={zero,zero,zero},wg[3]={zero,zero,zero};std::size_t i=0;
  for(;i+4<=count;i+=4) {
    const auto q=_mm256_loadu_pd(values+i);__m256d g[3];
    g[0]=_mm256_loadu_pd(gradients+i);g[1]=_mm256_loadu_pd(gradients+count+i);
    if(zero_gradients(g[0],g[1],q))continue;
    if constexpr(Rotate)g[2]=_mm256_loadu_pd(gradients+2*count+i);
    const auto w=_mm256_loadu_pd(patch.weights.data()+i);
    for(int j=0;j<axes;++j){gs[j]=_mm256_add_pd(gs[j],g[j]);qg[j]=_mm256_add_pd(qg[j],_mm256_mul_pd(q,g[j]));wg[j]=_mm256_add_pd(wg[j],_mm256_mul_pd(w,g[j]));}
  }
  for(int j=0;j<axes;++j){out.gs[j]=total(gs[j]);out.qg[j]=total(qg[j]);out.wg[j]=total(wg[j]);}
  for(;i<count;++i)for(int j=0;j<axes;++j){const auto g=gradients[j*count+i];out.gs[j]+=g;out.qg[j]+=values[i]*g;out.wg[j]+=patch.samples[i].weight*g;}
  __m256d gg[3][3]{};for(auto& row:gg)for(auto& v:row)v=zero;
  i=0;
  for(;i+4<=count;i+=4) {
    __m256d g[3];g[0]=_mm256_loadu_pd(gradients+i);g[1]=_mm256_loadu_pd(gradients+count+i);
    if constexpr(Rotate)g[2]=_mm256_loadu_pd(gradients+2*count+i);
    auto any=_mm256_or_pd(g[0],g[1]);if constexpr(Rotate)any=_mm256_or_pd(any,g[2]);
    if(_mm256_movemask_pd(_mm256_cmp_pd(any,zero,_CMP_EQ_OQ))==15)continue;
    for(int j=0;j<axes;++j)for(int k=0;k<=j;++k)gg[j][k]=_mm256_add_pd(gg[j][k],_mm256_mul_pd(g[j],g[k]));
  }
  for(int j=0;j<axes;++j)for(int k=0;k<=j;++k)out.gg[j][k]=total(gg[j][k]);
  for(;i<count;++i)for(int j=0;j<axes;++j)for(int k=0;k<=j;++k)out.gg[j][k]+=gradients[j*count+i]*gradients[k*count+i];
}
template<bool Derivative,bool Rotate,bool Aligned=false,bool Staged=false> bool bilinear(const double* image,std::size_t width,std::size_t height,const Level& level,
    const Patch& patch,const Match& pose,BilinearMoments& out,double ca,double sa,bool interior,double* centered_samples=nullptr,double* gradient_samples=nullptr,BilinearScoreGate* score_gate=nullptr,const BilinearZeroView* zero_blocks=nullptr) {
  const auto ca4=_mm256_set1_pd(ca),sa4=_mm256_set1_pd(sa),one=_mm256_set1_pd(1.),zero=_mm256_setzero_pd();
  const auto scalar=[&](const Sample& p,double& q,double* g) {
    const auto dr=p.row-level.origin_row,dc=p.column-level.origin_column;
    const auto y=pose.row+ca*dr-sa*dc,x=pose.column+sa*dr+ca*dc;
    if(y<0 || x<0 || y>height-1 || x>width-1 || (Derivative && (y>=height-1 || x>=width-1)))return false;
    const auto r=std::size_t(y),c=std::size_t(x),r1=std::min(r+1,height-1),c1=std::min(c+1,width-1);const auto fy=y-r,fx=x-c;
    const auto a=image[r*width+c],b=image[r*width+c1],d=image[r1*width+c],e=image[r1*width+c1];
    q=(1-fy)*((1-fx)*a+fx*b)+fy*((1-fx)*d+fx*e);
    if constexpr(Derivative){g[0]=(1-fx)*(d-a)+fx*(e-b);g[1]=(1-fy)*(b-a)+fy*(e-d);g[2]=g[0]*(-sa*dr-ca*dc)+g[1]*(ca*dr-sa*dc);}
    return true;
  };
  double offset=0,unused[3];if(!scalar(patch.samples.front(),offset,unused))return false;
  // 正零支撑保持精确原 q=+0-offset；其他模式或非有限偏移保留原采样及矩计算。
  if(!interior || !std::isfinite(offset) || (_mm_getcsr()&0xE040))zero_blocks=nullptr;
  // 公共界覆盖每个准备块的中间相消；坐标不可变，半径只准备一次。
  const auto zero_margin=zero_blocks?64*std::numeric_limits<double>::epsilon()*(1+std::abs(pose.row)+std::abs(pose.column)+
      (std::abs(ca)+std::abs(sa))*zero_blocks->samples->maximum_coordinate_radius()+double(width)+double(height)):0.;
  const bool positive_cos=ca>=0,positive_sin=sa>=0;
  const auto* group_tiles=zero_blocks && zero_blocks->samples->spatial_tiles() && zero_blocks->tile_flags &&
      zero_blocks->tile_flags_capacity>=zero_blocks->samples->tile_count()?zero_blocks->samples->group_tiles():nullptr;
  if(group_tiles) {
    const auto* image_proof=zero_blocks->image;const auto* sample_proof=zero_blocks->samples;
    auto* cache=zero_blocks->proof_cache;
    if(!cache || !cache->matches(*image_proof,*sample_proof,pose,ca,sa,Aligned,zero_blocks->tile_flags,zero_blocks->tile_flags_capacity)) {
      for(std::size_t tile=0;tile<sample_proof->tile_count();++tile) {
        const bool supported=supported_zero_block<Aligned>(*image_proof,sample_proof->tiles()[tile],pose,ca,sa,positive_cos,positive_sin,zero_margin);
        zero_blocks->tile_flags[tile]=std::uint8_t(supported);
        if(zero_blocks->tested_blocks)++*zero_blocks->tested_blocks;
        if(supported && zero_blocks->skipped_blocks)++*zero_blocks->skipped_blocks;
      }
      if(cache)cache->remember(*image_proof,*sample_proof,pose,ca,sa,Aligned,zero_blocks->tile_flags,zero_blocks->tile_flags_capacity);
    }
  }
  const auto shift=_mm256_set1_pd(offset),row0=_mm256_set1_pd(pose.row),col0=_mm256_set1_pd(pose.column),
      ro=_mm256_set1_pd(level.origin_row),co=_mm256_set1_pd(level.origin_column);
  const auto xmax=_mm256_set1_pd(double(width-1)),ymax=_mm256_set1_pd(double(height-1));
  const auto pitch=_mm_set1_epi32(int(width)),xi_max=_mm_set1_epi32(int(width-1)),yi_max=_mm_set1_epi32(int(height-1)),ones=_mm_set1_epi32(1);
  auto sum=zero,squared=zero,dot=zero;__m256d gs[3]={zero,zero,zero},qg[3]={zero,zero,zero},wg[3]={zero,zero,zero},gg[3][3]{};
  if constexpr(Derivative && !Staged)for(auto& row:gg)for(auto& v:row)v=zero;
  std::size_t i=0,zero_block_index=0,next_zero_block=0;constexpr int axes=Rotate?3:2;
  for(;i+4<=patch.samples.size();i+=4) {
    std::size_t zero_count=0;
    if(group_tiles) {
      if(zero_blocks->tile_flags[group_tiles[i/4]]) {
        zero_count=4;
        const auto vector_end=patch.samples.size()-patch.samples.size()%4;
        // 合并连续已证明组但保持统计次序，缓存输出可一次填完整游程。
        while(i+zero_count<vector_end && zero_blocks->tile_flags[group_tiles[(i+zero_count)/4]])zero_count+=4;
      }
    }else if(zero_blocks && i==next_zero_block && zero_block_index<zero_blocks->samples->size()) {
      const auto& block=zero_blocks->samples->data()[zero_block_index++];
      const auto block_size=zero_blocks->samples->block_size();next_zero_block+=block_size;
      if(zero_blocks->tested_blocks)++*zero_blocks->tested_blocks;
      if(supported_zero_block<Aligned>(*zero_blocks->image,block,pose,ca,sa,positive_cos,positive_sin,zero_margin)) {
        if(zero_blocks->skipped_blocks)++*zero_blocks->skipped_blocks;
        zero_count=block_size;
      }
    }
    if(zero_count) {
      if(offset==0) {
        if(centered_samples)std::fill_n(centered_samples+i,zero_count,0.);
      }else {
        const auto q=_mm256_sub_pd(zero,shift),q_squared=_mm256_mul_pd(q,q);
        // 保留各原四 lane 灰度更新的次序及权重乘法，不读取坐标。
        for(std::size_t sample=i;sample<i+zero_count;sample+=4) {
          const auto w=_mm256_loadu_pd(patch.weights.data()+sample);
          if(centered_samples)_mm256_storeu_pd(centered_samples+sample,q);
          sum=_mm256_add_pd(sum,q);squared=_mm256_add_pd(squared,q_squared);dot=_mm256_add_pd(dot,_mm256_mul_pd(w,q));
        }
      }
      if constexpr(Derivative)if(gradient_samples)for(int axis=0;axis<3;++axis)
        std::fill_n(gradient_samples+axis*patch.samples.size()+i,zero_count,0.);
      i+=zero_count-4;continue;
    }
    const auto dr=_mm256_loadu_pd(patch.relative_rows.data()+i),dc=_mm256_loadu_pd(patch.relative_columns.data()+i);
    __m256d y,x;
    if constexpr(Aligned){y=_mm256_add_pd(row0,dr);x=_mm256_add_pd(col0,dc);}
    else {y=_mm256_sub_pd(_mm256_add_pd(row0,_mm256_mul_pd(ca4,dr)),_mm256_mul_pd(sa4,dc));
      x=_mm256_add_pd(_mm256_add_pd(col0,_mm256_mul_pd(sa4,dr)),_mm256_mul_pd(ca4,dc));}
    if(!interior) {
      auto valid=_mm256_and_pd(_mm256_cmp_pd(y,zero,_CMP_GE_OQ),_mm256_cmp_pd(x,zero,_CMP_GE_OQ));
      valid=_mm256_and_pd(valid,_mm256_and_pd(_mm256_cmp_pd(y,ymax,Derivative?_CMP_LT_OQ:_CMP_LE_OQ),_mm256_cmp_pd(x,xmax,Derivative?_CMP_LT_OQ:_CMP_LE_OQ)));
      if(_mm256_movemask_pd(valid)!=15)return false;
    }
    const auto yr=_mm256_floor_pd(y),xc=_mm256_floor_pd(x),fy=_mm256_sub_pd(y,yr),fx=_mm256_sub_pd(x,xc);
    const auto r=_mm256_cvttpd_epi32(yr),c=_mm256_cvttpd_epi32(xc);
    __m256d a,b,d,e;
    // 小旋转也常产生连续图像地址；检查实际整数晶格，不要求 angle==0。
    const bool contiguous=
        (interior || (_mm_cvtsi128_si32(c)+4<int(width) && _mm_cvtsi128_si32(r)+1<int(height))) &&
        _mm_movemask_epi8(_mm_cmpeq_epi32(r,_mm_set1_epi32(_mm_cvtsi128_si32(r))))==0xffff &&
        _mm_movemask_epi8(_mm_cmpeq_epi32(c,_mm_add_epi32(_mm_set1_epi32(_mm_cvtsi128_si32(c)),_mm_set_epi32(3,2,1,0))))==0xffff;
    if(contiguous) {
      const auto index=std::size_t(_mm_cvtsi128_si32(r))*width+_mm_cvtsi128_si32(c);
      a=_mm256_loadu_pd(image+index);b=_mm256_loadu_pd(image+index+1);d=_mm256_loadu_pd(image+index+width);e=_mm256_loadu_pd(image+index+width+1);
    }else {
      const auto r1=interior?_mm_add_epi32(r,ones):_mm_min_epi32(_mm_add_epi32(r,ones),yi_max);
      const auto c1=interior?_mm_add_epi32(c,ones):_mm_min_epi32(_mm_add_epi32(c,ones),xi_max);
      const auto rows=_mm_mullo_epi32(r,pitch),rows1=_mm_mullo_epi32(r1,pitch);
      a=_mm256_i32gather_pd(image,_mm_add_epi32(rows,c),8);b=_mm256_i32gather_pd(image,_mm_add_epi32(rows,c1),8);
      d=_mm256_i32gather_pd(image,_mm_add_epi32(rows1,c),8);e=_mm256_i32gather_pd(image,_mm_add_epi32(rows1,c1),8);
    }
    const auto ux=_mm256_sub_pd(one,fx),uy=_mm256_sub_pd(one,fy);
    const auto q=_mm256_sub_pd(_mm256_add_pd(_mm256_mul_pd(uy,_mm256_add_pd(_mm256_mul_pd(ux,a),_mm256_mul_pd(fx,b))),
        _mm256_mul_pd(fy,_mm256_add_pd(_mm256_mul_pd(ux,d),_mm256_mul_pd(fx,e)))),shift);
    const auto w=_mm256_loadu_pd(patch.weights.data()+i);
    if(centered_samples)_mm256_storeu_pd(centered_samples+i,q);
    sum=_mm256_add_pd(sum,q);squared=_mm256_add_pd(squared,_mm256_mul_pd(q,q));dot=_mm256_add_pd(dot,_mm256_mul_pd(w,q));
    if constexpr(Derivative) {
      __m256d g[3];g[0]=_mm256_add_pd(_mm256_mul_pd(ux,_mm256_sub_pd(d,a)),_mm256_mul_pd(fx,_mm256_sub_pd(e,b)));
      g[1]=_mm256_add_pd(_mm256_mul_pd(uy,_mm256_sub_pd(b,a)),_mm256_mul_pd(fy,_mm256_sub_pd(e,d)));
      if(gradient_samples) {
        _mm256_storeu_pd(gradient_samples+i,g[0]);
        _mm256_storeu_pd(gradient_samples+patch.samples.size()+i,g[1]);
      }
      // 平坦块无导数矩；保留原插值及灰度矩，仅省角度导数和十五项累加（含寄存器溢出存取）。
      // 非有限中心化值保留原 0*Inf/NaN 路径。
      if(zero_gradients(g[0],g[1],q)) {
        if(gradient_samples)_mm256_storeu_pd(gradient_samples+2*patch.samples.size()+i,zero);
        continue;
      }
      if constexpr(Rotate)g[2]=_mm256_add_pd(_mm256_mul_pd(g[0],_mm256_sub_pd(_mm256_sub_pd(zero,_mm256_mul_pd(sa4,dr)),_mm256_mul_pd(ca4,dc))),
          _mm256_mul_pd(g[1],_mm256_sub_pd(_mm256_mul_pd(ca4,dr),_mm256_mul_pd(sa4,dc))));
      if(gradient_samples)_mm256_storeu_pd(gradient_samples+2*patch.samples.size()+i,Rotate?g[2]:zero);
      if constexpr(!Staged)for(int j=0;j<axes;++j){gs[j]=_mm256_add_pd(gs[j],g[j]);qg[j]=_mm256_add_pd(qg[j],_mm256_mul_pd(q,g[j]));wg[j]=_mm256_add_pd(wg[j],_mm256_mul_pd(w,g[j]));for(int k=0;k<=j;++k)gg[j][k]=_mm256_add_pd(gg[j][k],_mm256_mul_pd(g[j],g[k]));}
    }
  }
  out.sum=total(sum);out.squared=total(squared);out.dot=total(dot);
  if constexpr(Derivative && !Staged)for(int j=0;j<axes;++j){out.gs[j]=total(gs[j]);out.qg[j]=total(qg[j]);out.wg[j]=total(wg[j]);for(int k=0;k<=j;++k)out.gg[j][k]=total(gg[j][k]);}
  for(;i<patch.samples.size();++i){const auto& p=patch.samples[i];double q,g[3];if(!scalar(p,q,g))return false;q-=offset;
    if(centered_samples)centered_samples[i]=q;
    if constexpr(Derivative)if(gradient_samples){gradient_samples[i]=g[0];gradient_samples[patch.samples.size()+i]=g[1];gradient_samples[2*patch.samples.size()+i]=Rotate?g[2]:0.;}
    out.sum+=q;out.squared+=q*q;out.dot+=q*p.weight;
    if constexpr(Derivative && !Staged)for(int j=0;j<axes;++j){out.gs[j]+=g[j];out.qg[j]+=q*g[j];out.wg[j]+=p.weight*g[j];for(int k=0;k<=j;++k)out.gg[j][k]+=g[j]*g[k];}
  }
  if constexpr(Staged) {
    if(score_gate) {
      const auto energy=out.squared-out.sum*out.sum/double(patch.samples.size());
      if(energy>0) {
        // 这是完整且有序的细化评分，不是界或部分模板近似；拒绝的试探位姿无需求解。
        const double sign=score_gate->absolute && out.dot<0?-1:1;
        const auto score=std::clamp(sign*out.dot/std::sqrt(energy),-1.,1.);
        if(score<=score_gate->minimum){score_gate->skipped=true;return true;}
      }
    }
    derivative_statistics<Rotate>(centered_samples,gradient_samples,patch,out);
  }
  return true;
}
}
double centered_dot_avx2(const double* values,const Patch& patch) {
  auto dot=_mm256_setzero_pd();std::size_t i=0;
  for(;i+4<=patch.samples.size();i+=4)dot=_mm256_add_pd(dot,_mm256_mul_pd(_mm256_loadu_pd(values+i),_mm256_loadu_pd(patch.weights.data()+i)));
  double result=total(dot);for(;i<patch.samples.size();++i)result+=values[i]*patch.samples[i].weight;return result;
}
void weighted_moments_avx2(const double* centered_samples,const double* gradient_samples,const Patch& patch,bool rotate,BilinearMoments& out) {
  const auto zero=_mm256_setzero_pd();auto dot=zero;__m256d wg[3]={zero,zero,zero};const int axes=rotate?3:2;
  std::size_t i=0;
  for(;i+4<=patch.samples.size();i+=4) {
    const auto w=_mm256_loadu_pd(patch.weights.data()+i),q=_mm256_loadu_pd(centered_samples+i);
    dot=_mm256_add_pd(dot,_mm256_mul_pd(w,q));
    for(int j=0;j<axes;++j)wg[j]=_mm256_add_pd(wg[j],_mm256_mul_pd(w,_mm256_loadu_pd(gradient_samples+j*patch.samples.size()+i)));
  }
  out.dot=total(dot);for(int j=0;j<axes;++j)out.wg[j]=total(wg[j]);
  for(;i<patch.samples.size();++i) {
    out.dot+=centered_samples[i]*patch.samples[i].weight;
    for(int j=0;j<axes;++j)out.wg[j]+=patch.samples[i].weight*gradient_samples[j*patch.samples.size()+i];
  }
}
void weighted_moments_compressed_avx2(const double* centered_samples,const RefinementGradientGroup* groups,std::size_t group_count,const Patch& patch,bool rotate,BilinearMoments& out) {
  const auto zero=_mm256_setzero_pd();auto dot=zero;__m256d wg[3]={zero,zero,zero};const auto count=patch.samples.size();const int axes=rotate?3:2;
  // 灰度样本保持稠密，点积使用全部原四 lane 组及标量尾部，梯度全零也如此。
  std::size_t i=0;
  for(;i+4<=count;i+=4)dot=_mm256_add_pd(dot,_mm256_mul_pd(_mm256_loadu_pd(patch.weights.data()+i),_mm256_loadu_pd(centered_samples+i)));
  out.dot=total(dot);for(;i<count;++i)out.dot+=centered_samples[i]*patch.samples[i].weight;
  const RefinementGradientGroup* tail=nullptr;
  // 已验证 NCC 权重有限，省略零梯度块贡献精确零；保留组仍按原递增 lane 索引排列。
  for(std::size_t index=0;index<group_count;++index) {
    const auto& group=groups[index];
    if(group.index+4>count){tail=&group;break;}
    const auto w=_mm256_loadu_pd(patch.weights.data()+group.index);
    for(int j=0;j<axes;++j)wg[j]=_mm256_add_pd(wg[j],_mm256_mul_pd(w,_mm256_loadu_pd(group.values.data()+4*j)));
  }
  for(int j=0;j<axes;++j)out.wg[j]=total(wg[j]);
  // padding 仅用于存储，末尾不足一组不能参与 SIMD 归约，以免改变原标量尾部舍入序。
  if(tail)for(i=tail->index;i<count;++i)for(int j=0;j<axes;++j)
    out.wg[j]+=patch.samples[i].weight*tail->values[4*j+i-tail->index];
}
bool bilinear_moments_avx2(const double* image,std::size_t width,std::size_t height,const Level& level,const Patch& patch,const Match& pose,bool derivatives,bool rotate,BilinearMoments& out,double* centered_samples,double* gradient_samples,double* derivative_workspace,BilinearScoreGate* score_gate,const BilinearZeroView* zero_blocks) {
  if(score_gate)score_gate->skipped=false;
  if(zero_blocks && (!zero_blocks->image || !zero_blocks->samples || !zero_blocks->image->valid() || !zero_blocks->samples->valid() ||
      zero_blocks->image->image()!=image || zero_blocks->image->width()!=width || zero_blocks->image->height()!=height ||
      zero_blocks->samples->patch()!=&patch || zero_blocks->samples->size()!=patch.samples.size()/zero_blocks->samples->block_size()))zero_blocks=nullptr;
  const auto ca=std::cos(pose.angle),sa=std::sin(pose.angle);
  const bool interior=strictly_inside(width,height,level,patch,pose,ca,sa);
  const bool staged=derivatives && interior && patch.samples.size()<=32768 &&
      ((centered_samples && gradient_samples) || derivative_workspace);
  if(staged) {
    if(!centered_samples)centered_samples=derivative_workspace;
    if(!gradient_samples)gradient_samples=derivative_workspace+patch.samples.size();
    if(pose.angle==0)return rotate?
        bilinear<true,true,true,true>(image,width,height,level,patch,pose,out,ca,sa,true,centered_samples,gradient_samples,score_gate,zero_blocks):
        bilinear<true,false,true,true>(image,width,height,level,patch,pose,out,ca,sa,true,centered_samples,gradient_samples,score_gate,zero_blocks);
    return rotate?bilinear<true,true,false,true>(image,width,height,level,patch,pose,out,ca,sa,true,centered_samples,gradient_samples,score_gate,zero_blocks):
        bilinear<true,false,false,true>(image,width,height,level,patch,pose,out,ca,sa,true,centered_samples,gradient_samples,score_gate,zero_blocks);
  }
  if(pose.angle==0) {
    if(!derivatives)return bilinear<false,false,true>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,nullptr,nullptr,zero_blocks);
    return rotate?bilinear<true,true,true>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,gradient_samples,nullptr,zero_blocks):bilinear<true,false,true>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,gradient_samples,nullptr,zero_blocks);
  }
  if(!derivatives)return bilinear<false,false>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,nullptr,nullptr,zero_blocks);
  return rotate?bilinear<true,true>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,gradient_samples,nullptr,zero_blocks):bilinear<true,false>(image,width,height,level,patch,pose,out,ca,sa,interior,centered_samples,gradient_samples,nullptr,zero_blocks);
}
}
