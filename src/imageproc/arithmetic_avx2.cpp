#include "arithmetic_avx2.hpp"
#include <cstring>
#include <cmath>
#include <core/numeric_predicates.hpp>
#include <algorithm>
#include <limits>
#include <bit>
#include <cfenv>
#include <immintrin.h>
#include <core/region.hpp>
namespace mv::imageproc {
std::size_t emphasize_u8_row_avx2(const std::uint8_t* source,const float* mean,
    std::uint8_t* output,std::size_t size,double factor) {
  if(std::fegetround()!=FE_TONEAREST)return 0;
  std::size_t c=0;
  if(factor<=64 && double(float(factor))==factor) {
    const auto scale=_mm256_set1_ps(float(factor)),zero=_mm256_setzero_ps(),high=_mm256_set1_ps(255.f);
    const auto half=_mm256_set1_ps(.5f),sign=_mm256_set1_ps(-0.f);
    // 三次 Float32 运算的绝对误差界留有余量；只对可能跨越半值的 lane 回退。
    const auto error=_mm256_set1_ps(0x1p-20f*(255.f+float(factor)*511.f));
    for(;c+8<=size;c+=8) {
      const auto value=_mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source+c))));
      auto result=_mm256_add_ps(value,_mm256_mul_ps(scale,_mm256_sub_ps(value,_mm256_loadu_ps(mean+c))));
      result=_mm256_min_ps(high,_mm256_max_ps(zero,result));
      const auto whole=_mm256_cvttps_epi32(result);
      const auto fraction=_mm256_sub_ps(result,_mm256_cvtepi32_ps(whole));
      const auto rounded=_mm256_sub_epi32(whole,_mm256_castps_si256(_mm256_cmp_ps(fraction,half,_CMP_GE_OQ)));
      const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
      _mm_storel_epi64(reinterpret_cast<__m128i*>(output+c),_mm_packus_epi16(words,words));
      unsigned uncertain=static_cast<unsigned>(_mm256_movemask_ps(_mm256_cmp_ps(
          _mm256_andnot_ps(sign,_mm256_sub_ps(fraction,half)),error,_CMP_LE_OQ)));
      while(uncertain) {
        const auto lane=std::countr_zero(uncertain);
        const double v=source[c+lane];
        const auto exact=std::clamp(v+factor*(v-double(mean[c+lane])),0.,255.);
        const auto integral=static_cast<unsigned>(exact);
        output[c+lane]=static_cast<std::uint8_t>(integral+(exact-integral>=.5));
        uncertain&=uncertain-1;
      }
    }
  } else {
    const auto scale=_mm256_set1_pd(factor),zero=_mm256_setzero_pd(),high=_mm256_set1_pd(255.);
    const auto half=_mm256_set1_pd(.5),one=_mm256_set1_pd(1.);
    for(;c+4<=size;c+=4) {
      std::uint32_t bytes;std::memcpy(&bytes,source+c,sizeof(bytes));
      const auto value=_mm256_cvtepi32_pd(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(static_cast<int>(bytes))));
      auto result=_mm256_add_pd(value,_mm256_mul_pd(scale,_mm256_sub_pd(value,_mm256_cvtps_pd(_mm_loadu_ps(mean+c)))));
      result=_mm256_min_pd(high,_mm256_max_pd(zero,result));
      const auto whole=_mm256_cvttpd_epi32(result);
      const auto fraction=_mm256_sub_pd(result,_mm256_cvtepi32_pd(whole));
      const auto increment=_mm256_cvttpd_epi32(_mm256_blendv_pd(zero,one,_mm256_cmp_pd(fraction,half,_CMP_GE_OQ)));
      const auto words=_mm_packus_epi32(_mm_add_epi32(whole,increment),_mm_setzero_si128());
      const auto packed=_mm_cvtsi128_si32(_mm_packus_epi16(words,words));
      std::memcpy(output+c,&packed,sizeof(packed));
    }
  }
  return c;
}
void box_mean_columns_row_avx2(const std::uint32_t* columns,const std::size_t* leaving,
    const std::size_t* entering,float* output,std::size_t width,std::size_t radius,
    std::uint32_t area,std::uint32_t sum,bool byte) {
  const auto denominator=_mm256_set1_ps(float(area));
  const auto real_denominator=_mm256_set1_pd(double(area));
  const auto scalar=[&](std::uint32_t n){return byte?float(n)/float(area):static_cast<float>(double(n)/area);};
  output[0]=scalar(sum);
  std::size_t c=1;
  for(;c<width && (c<radius+1 || width-c<8+radius);++c) {
    sum=sum-columns[leaving[c-1]]+columns[entering[c-1]];output[c]=scalar(sum);
  }
  if(c==width)return;
  auto carry=_mm256_set1_epi32(static_cast<int>(sum));
  for(;c<width && width-c>=8+radius;c+=8) {
    auto delta=_mm256_sub_epi32(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(columns+c+radius)),
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(columns+c-radius-1)));
    delta=_mm256_add_epi32(delta,_mm256_slli_si256(delta,4));
    delta=_mm256_add_epi32(delta,_mm256_slli_si256(delta,8));
    const auto bridge=_mm256_inserti128_si256(_mm256_setzero_si256(),_mm_shuffle_epi32(_mm256_castsi256_si128(delta),0xff),1);
    const auto totals=_mm256_add_epi32(_mm256_add_epi32(delta,bridge),carry);
    if(byte)_mm256_storeu_ps(output+c,_mm256_div_ps(_mm256_cvtepi32_ps(totals),denominator));
    else {
      _mm_storeu_ps(output+c,_mm256_cvtpd_ps(_mm256_div_pd(_mm256_cvtepi32_pd(_mm256_castsi256_si128(totals)),real_denominator)));
      _mm_storeu_ps(output+c+4,_mm256_cvtpd_ps(_mm256_div_pd(_mm256_cvtepi32_pd(_mm256_extracti128_si256(totals,1)),real_denominator)));
    }
    carry=_mm256_permutevar8x32_epi32(totals,_mm256_set1_epi32(7));
  }
  sum=static_cast<std::uint32_t>(_mm_cvtsi128_si32(_mm256_castsi256_si128(carry)));
  for(;c<width;++c) {sum=sum-columns[leaving[c-1]]+columns[entering[c-1]];output[c]=scalar(sum);}
}
std::size_t rgb_to_hsv_u8_row_avx2(const std::uint8_t* red, const std::uint8_t* green,
    const std::uint8_t* blue, float* hue, float* saturation, float* value,
    std::size_t begin, std::size_t end) {
  if(std::fegetround()!=FE_TONEAREST)return begin;
  const auto one=_mm256_set1_epi32(1),zero=_mm256_setzero_si256();
  const auto scale=_mm256_set1_ps(255.f);
  const auto read=[](const std::uint8_t* source) {
    return _mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source)));
  };
  auto c=begin;
  for(;c+8<=end;c+=8) {
    const auto r=read(red+c),g=read(green+c),b=read(blue+c);
    const auto maximum=_mm256_max_epi32(_mm256_max_epi32(r,g),b);
    const auto minimum=_mm256_min_epi32(_mm256_min_epi32(r,g),b);
    const auto delta=_mm256_sub_epi32(maximum,minimum);
    const auto twice=_mm256_slli_epi32(delta,1),four=_mm256_slli_epi32(delta,2);
    const auto six=_mm256_add_epi32(twice,four);
    const auto red_value=_mm256_sub_epi32(g,b);
    const auto green_value=_mm256_add_epi32(twice,_mm256_sub_epi32(b,r));
    const auto blue_value=_mm256_add_epi32(four,_mm256_sub_epi32(r,g));
    auto numerator=_mm256_blendv_epi8(blue_value,green_value,_mm256_cmpeq_epi32(maximum,g));
    numerator=_mm256_blendv_epi8(numerator,red_value,_mm256_cmpeq_epi32(maximum,r));
    numerator=_mm256_add_epi32(numerator,_mm256_and_si256(six,_mm256_cmpgt_epi32(zero,numerator)));
    // byte HSV 的分子/分母均为小整数，最后一次 Float32 除法正确舍入。
    // 消除规范化后相减的误差；与旧 Double 表达式的全部 256^3 输出逐位对照。
    _mm256_storeu_ps(hue+c,_mm256_div_ps(_mm256_cvtepi32_ps(numerator),
        _mm256_cvtepi32_ps(_mm256_max_epi32(six,one))));
    _mm256_storeu_ps(saturation+c,_mm256_div_ps(_mm256_cvtepi32_ps(delta),
        _mm256_cvtepi32_ps(_mm256_max_epi32(maximum,one))));
    _mm256_storeu_ps(value+c,_mm256_div_ps(_mm256_cvtepi32_ps(maximum),scale));
  }
  return c;
}

std::size_t split_byte_interleaved_row_avx2(const std::uint8_t* source,std::uint8_t* red,
    std::uint8_t* green,std::uint8_t* blue,std::size_t count,std::size_t components,bool bgr) {
  const std::array outputs{bgr?blue:red,green,bgr?red:blue};
  const auto split=[&]<std::size_t Components>() {
    // 每批只读取实际 16 个像素的 Components*16 字节，裁剪尾部不越界读取。
    static constexpr auto masks=[] {
      std::array<std::array<std::array<std::int8_t,16>,Components>,3> result{};
      for(std::size_t ch=0;ch<3;++ch)for(std::size_t part=0;part<Components;++part)
        for(std::size_t pixel=0;pixel<16;++pixel) {
          const auto index=pixel*Components+ch;
          result[ch][part][pixel]=index/16==part?static_cast<std::int8_t>(index%16):std::int8_t{-128};
        }
      return result;
    }();
    std::size_t i=0;
    for(;count-i>=16;i+=16) {
      __m128i loaded[Components];
      for(std::size_t part=0;part<Components;++part)
        loaded[part]=_mm_loadu_si128(reinterpret_cast<const __m128i*>(source+i*Components+part*16));
      for(std::size_t ch=0;ch<3;++ch) {
        auto channel=_mm_setzero_si128();
        for(std::size_t part=0;part<Components;++part)
          channel=_mm_or_si128(channel,_mm_shuffle_epi8(loaded[part],
              _mm_loadu_si128(reinterpret_cast<const __m128i*>(masks[ch][part].data()))));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(outputs[ch]+i),channel);
      }
    }
    return i;
  };
  return components==3?split.template operator()<3>():split.template operator()<4>();
}

std::size_t transform_byte_planes_row_avx2(const std::uint8_t* a,const std::uint8_t* b,
    const std::uint8_t* d,std::uint8_t* x,std::uint8_t* y,std::uint8_t* z,
    std::size_t begin,std::size_t end,bool inverse) {
  if(std::fegetround()!=FE_TONEAREST)return begin;
  auto c=begin;
  if(!inverse) {
    alignas(32) float hsv[3][128];
    const auto scale=_mm256_set1_ps(255.f),half=_mm256_set1_ps(.5f);
    for(;c+8<=end;) {
      const auto count=std::min<std::size_t>((end-c)/8*8,128);
      rgb_to_hsv_u8_row_avx2(a+c,b+c,d+c,hsv[0],hsv[1],hsv[2],0,count);
      const std::array outputs{x+c,y+c,z+c};
      for(std::size_t ch=0;ch<3;++ch)for(std::size_t i=0;i<count;i+=8) {
        const auto scaled=_mm256_mul_ps(_mm256_load_ps(hsv[ch]+i),scale);
        const auto whole=_mm256_cvttps_epi32(scaled);
        const auto fraction=_mm256_sub_ps(scaled,_mm256_cvtepi32_ps(whole));
        const auto rounded=_mm256_sub_epi32(whole,_mm256_castps_si256(_mm256_cmp_ps(fraction,half,_CMP_GE_OQ)));
        const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(outputs[ch]+i),_mm_packus_epi16(words,words));
      }
      c+=count;
    }
    return c;
  }
  const auto one=_mm256_set1_pd(1.),six=_mm256_set1_pd(6.);
  const auto normalizer=_mm256_set1_pd(1./255.),scale=_mm256_set1_pd(255.);
  const auto read=[&](const std::uint8_t* input) {
    std::uint32_t bytes;std::memcpy(&bytes,input,sizeof(bytes));
    const auto values=_mm256_cvtepi32_pd(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(static_cast<int>(bytes))));
    return _mm256_cvtps_pd(_mm256_cvtpd_ps(_mm256_mul_pd(values,normalizer)));
  };
  const auto store=[&](std::uint8_t* out,__m256d value) {
    value=_mm256_mul_pd(value,scale);
    const auto whole=_mm256_cvttpd_epi32(value);
    const auto fraction=_mm256_sub_pd(value,_mm256_cvtepi32_pd(whole));
    const auto increment=_mm256_and_pd(_mm256_cmp_pd(fraction,_mm256_set1_pd(.5),_CMP_GE_OQ),one);
    const auto rounded=_mm_add_epi32(whole,_mm256_cvttpd_epi32(increment));
    const auto words=_mm_packus_epi32(rounded,rounded);
    const auto bytes=static_cast<std::uint32_t>(_mm_cvtsi128_si32(_mm_packus_epi16(words,words)));
    std::memcpy(out,&bytes,sizeof(bytes));
  };
  for(;c+4<=end;c+=4) {
    const auto hue=read(a+c),s=read(b+c),v=read(d+c);
    const auto h=_mm256_mul_pd(_mm256_sub_pd(hue,_mm256_floor_pd(hue)),six);
    const auto sector=_mm256_floor_pd(h),f=_mm256_sub_pd(h,sector);
    const auto p=_mm256_mul_pd(v,_mm256_sub_pd(one,s));
    const auto q=_mm256_mul_pd(v,_mm256_sub_pd(one,_mm256_mul_pd(s,f)));
    const auto t=_mm256_mul_pd(v,_mm256_sub_pd(one,_mm256_mul_pd(s,_mm256_sub_pd(one,f))));
    auto red=v,green=p,blue=q;
    const auto choose=[&](int index,__m256d r,__m256d g,__m256d bb) {
      const auto mask=_mm256_cmp_pd(sector,_mm256_set1_pd(double(index)),_CMP_EQ_OQ);
      red=_mm256_blendv_pd(red,r,mask);green=_mm256_blendv_pd(green,g,mask);blue=_mm256_blendv_pd(blue,bb,mask);
    };
    choose(0,v,t,p);choose(1,q,v,p);choose(2,p,v,t);choose(3,p,q,v);choose(4,t,p,v);
    store(x+c,red);store(y+c,green);store(z+c,blue);
  }
  return c;
}

namespace {
#if defined(_MSC_VER)
__forceinline
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((always_inline)) inline
#else
inline
#endif
std::uint32_t local_threshold_bits4(const std::uint8_t* row,__m256d mean,
    __m256d variance,__m256d scale,__m256d dynamic,bool light) {
  const auto one=_mm256_set1_pd(1.),maximum=_mm256_set1_pd(255.);
  const auto factor=_mm256_mul_pd(scale,_mm256_sub_pd(_mm256_div_pd(_mm256_sqrt_pd(variance),dynamic),one));
  const auto threshold=light
      ?_mm256_sub_pd(mean,_mm256_mul_pd(_mm256_sub_pd(maximum,mean),factor))
      :_mm256_mul_pd(mean,_mm256_add_pd(one,factor));
  std::uint32_t pixels;std::memcpy(&pixels,row,sizeof(pixels));
  const auto value=_mm256_cvtepi32_pd(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(static_cast<int>(pixels))));
  const auto selected=light?_mm256_cmp_pd(value,threshold,_CMP_GT_OQ):_mm256_cmp_pd(value,threshold,_CMP_LE_OQ);
  return std::uint32_t(_mm256_movemask_pd(selected));
}
inline __m256i inclusive_prefix64(__m256i difference) {
  const auto pairs=_mm256_add_epi64(difference,_mm256_slli_si256(difference,8));
  const auto carry=_mm256_blend_epi32(_mm256_setzero_si256(),_mm256_permute4x64_epi64(pairs,0x55),0xf0);
  return _mm256_add_epi64(pairs,carry);
}
} // namespace
std::uint32_t local_threshold_u8_mask_avx2(const std::uint8_t* row,
    const double* sums,const double* squares,double count,double k,double range,bool light) {
  const auto n=_mm256_set1_pd(count),scale=_mm256_set1_pd(k),dynamic=_mm256_set1_pd(range);
  const auto zero=_mm256_setzero_pd();
  std::uint32_t mask=0;
  for(unsigned lane=0;lane<32;lane+=4) {
    const auto mean=_mm256_div_pd(_mm256_loadu_pd(sums+lane),n);
    const auto variance=_mm256_max_pd(zero,_mm256_sub_pd(
        _mm256_div_pd(_mm256_loadu_pd(squares+lane),n),_mm256_mul_pd(mean,mean)));
    mask|=local_threshold_bits4(row+lane,mean,variance,scale,dynamic,light)<<lane;
  }
  return mask;
}

// 调用方保证四次列读取在图内且 65025*window_area<=INT32_MAX。
// 矩前缀按 UInt64 模精确，输出窗口及后续滚动状态均非负且在范围内。
std::uint32_t local_threshold_u8_window_mask_avx2(const std::uint8_t* row,
    const std::uint64_t* sums,const std::uint64_t* squares,std::size_t left,
    std::size_t right,std::uint64_t* running_sum,std::uint64_t* running_square,
    double count,double k,double range,bool light) {
  auto base_sum=_mm256_set1_epi64x(static_cast<long long>(*running_sum));
  auto base_square=_mm256_set1_epi64x(static_cast<long long>(*running_square));
  const auto n=_mm256_set1_pd(count),scale=_mm256_set1_pd(k),dynamic=_mm256_set1_pd(range);
  const auto zero=_mm256_setzero_pd();
  const auto low_words=_mm256_setr_epi32(0,2,4,6,0,2,4,6);
  std::uint32_t mask=0;
  for(unsigned lane=0;lane<32;lane+=4) {
    const auto delta_sum=_mm256_sub_epi64(
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(sums+right+lane)),
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(sums+left+lane)));
    const auto delta_square=_mm256_sub_epi64(
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(squares+right+lane)),
        _mm256_loadu_si256(reinterpret_cast<const __m256i*>(squares+left+lane)));
    const auto ps=inclusive_prefix64(delta_sum),pq=inclusive_prefix64(delta_square);
    const auto exclusive_sum=_mm256_blend_epi32(_mm256_setzero_si256(),_mm256_permute4x64_epi64(ps,0x90),0xfc);
    const auto exclusive_square=_mm256_blend_epi32(_mm256_setzero_si256(),_mm256_permute4x64_epi64(pq,0x90),0xfc);
    const auto current_sum=_mm256_add_epi64(base_sum,exclusive_sum);
    const auto current_square=_mm256_add_epi64(base_square,exclusive_square);
    const auto sm=_mm256_cvtepi32_pd(_mm256_castsi256_si128(_mm256_permutevar8x32_epi32(current_sum,low_words)));
    const auto sq=_mm256_cvtepi32_pd(_mm256_castsi256_si128(_mm256_permutevar8x32_epi32(current_square,low_words)));
    const auto mean=_mm256_div_pd(sm,n);
    const auto variance=_mm256_max_pd(zero,_mm256_sub_pd(_mm256_div_pd(sq,n),_mm256_mul_pd(mean,mean)));
    mask|=local_threshold_bits4(row+lane,mean,variance,scale,dynamic,light)<<lane;
    base_sum=_mm256_add_epi64(base_sum,_mm256_permute4x64_epi64(ps,0xff));
    base_square=_mm256_add_epi64(base_square,_mm256_permute4x64_epi64(pq,0xff));
  }
  *running_sum=static_cast<std::uint64_t>(_mm_cvtsi128_si64(_mm256_castsi256_si128(base_sum)));
  *running_square=static_cast<std::uint64_t>(_mm_cvtsi128_si64(_mm256_castsi256_si128(base_square)));
  return mask;
}

namespace {
// 四组完整 Byte 插值对；末列重复唯一源样本，不依赖行或末缓冲 padding。
struct BilinearSamplesU8 { __m256d tl, tr, bl, br; };
#if defined(_MSC_VER)
__forceinline
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((always_inline)) inline
#else
inline
#endif
BilinearSamplesU8 bilinear_samples_u8(const std::uint8_t* input,
    std::size_t stride,std::size_t width,std::size_t height,__m128i xi,__m128i yi) {
  // 仅 gather 完整源行字节，允许奇数 stride 和末行无 padding；行及总偏移须在有符号 Int32 内。
  if(width>=4 && stride<=INT32_MAX &&
     height-1<=std::size_t(INT32_MAX-width)/stride &&
     _mm_movemask_epi8(_mm_cmpgt_epi32(xi,_mm_set1_epi32(int(width-4))))==0) {
    const auto step=_mm_set1_epi32(int(stride));
    const auto ny=_mm_min_epi32(_mm_add_epi32(yi,_mm_set1_epi32(1)),_mm_set1_epi32(int(height-1)));
    const auto top_offset=_mm_add_epi32(_mm_mullo_epi32(yi,step),xi);
    const auto bottom_offset=_mm_add_epi32(_mm_mullo_epi32(ny,step),xi);
    const auto t=_mm_i32gather_epi32(reinterpret_cast<const int*>(input),top_offset,1);
    const auto b=_mm_i32gather_epi32(reinterpret_cast<const int*>(input),bottom_offset,1);
    const auto mask=_mm_set1_epi32(255);
    return {_mm256_cvtepi32_pd(_mm_and_si128(t,mask)),
            _mm256_cvtepi32_pd(_mm_and_si128(_mm_srli_epi32(t,8),mask)),
            _mm256_cvtepi32_pd(_mm_and_si128(b,mask)),
            _mm256_cvtepi32_pd(_mm_and_si128(_mm_srli_epi32(b,8),mask))};
  }
  std::int32_t xs[4],ys[4];
  _mm_storeu_si128(reinterpret_cast<__m128i*>(xs),xi);
  _mm_storeu_si128(reinterpret_cast<__m128i*>(ys),yi);
  std::int32_t tops[4],bottoms[4];
  for(unsigned j=0;j<4;++j) {
    const auto x=std::size_t(xs[j]),y=std::size_t(ys[j]);
    const auto* top=input+y*stride+x;
    const auto* bottom=input+std::min(y+1,height-1)*stride+x;
    if(x+1<width) {
      std::uint16_t t,b;
      std::memcpy(&t,top,sizeof(t));std::memcpy(&b,bottom,sizeof(b));
      tops[j]=t;bottoms[j]=b;
    } else {
      tops[j]=int(*top)*257;bottoms[j]=int(*bottom)*257;
    }
  }
  const auto t=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tops));
  const auto b=_mm_loadu_si128(reinterpret_cast<const __m128i*>(bottoms));
  const auto mask=_mm_set1_epi32(255);
  return {_mm256_cvtepi32_pd(_mm_and_si128(t,mask)),
          _mm256_cvtepi32_pd(_mm_srli_epi32(t,8)),
          _mm256_cvtepi32_pd(_mm_and_si128(b,mask)),
          _mm256_cvtepi32_pd(_mm_srli_epi32(b,8))};
}
template<bool Bilinear>
std::size_t remap_row(const std::uint8_t* input,std::uint8_t* output,std::size_t stride,
                     std::size_t width,std::size_t height,const double* m,std::size_t row,std::size_t first,std::size_t end) {
  const auto xr=_mm256_set1_pd(m[1]*row),yr=_mm256_set1_pd(m[4]*row);
  const auto mx=_mm256_set1_pd(m[0]),my=_mm256_set1_pd(m[3]),tx=_mm256_set1_pd(m[2]),ty=_mm256_set1_pd(m[5]);
  const auto zero=_mm256_setzero_pd(),one=_mm256_set1_pd(1.),half=_mm256_set1_pd(.5);
  auto c=first;
  for(;end-c>=4;c+=4) {
    // 坐标已限制到 INT32_MAX，整数列可由 Double 精确表示；一次转换替代四次，无坐标漂移。
    const auto columns=_mm256_add_pd(_mm256_set1_pd(double(c)),_mm256_set_pd(3.,2.,1.,0.));
    auto x=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(mx,columns),xr),tx);
    auto y=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(my,columns),yr),ty);
    std::int32_t ix[4],iy[4];
    if constexpr(!Bilinear) {
      _mm_storeu_si128(reinterpret_cast<__m128i*>(ix),_mm256_cvttpd_epi32(_mm256_add_pd(x,half)));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(iy),_mm256_cvttpd_epi32(_mm256_add_pd(y,half)));
      for(int j=0;j<4;++j)output[c+j]=input[std::size_t(iy[j])*stride+ix[j]];
    } else {
      x=_mm256_min_pd(_mm256_max_pd(x,zero),_mm256_set1_pd(double(width-1)));
      y=_mm256_min_pd(_mm256_max_pd(y,zero),_mm256_set1_pd(double(height-1)));
      const auto xi=_mm256_cvttpd_epi32(x),yi=_mm256_cvttpd_epi32(y);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(ix),xi);_mm_storeu_si128(reinterpret_cast<__m128i*>(iy),yi);
      const auto dx=_mm256_sub_pd(x,_mm256_cvtepi32_pd(xi)),dy=_mm256_sub_pd(y,_mm256_cvtepi32_pd(yi));
      const auto ex=_mm256_sub_pd(one,dx),ey=_mm256_sub_pd(one,dy);
      const auto samples=bilinear_samples_u8(input,stride,width,height,xi,yi);
      const auto top=_mm256_add_pd(_mm256_mul_pd(ex,samples.tl),_mm256_mul_pd(dx,samples.tr));
      const auto bottom=_mm256_add_pd(_mm256_mul_pd(ex,samples.bl),_mm256_mul_pd(dx,samples.br));
      const auto value=_mm256_add_pd(_mm256_mul_pd(ey,top),_mm256_mul_pd(dy,bottom));
      const auto rounded=_mm256_cvttpd_epi32(_mm256_add_pd(value,half));
      const auto words=_mm_packus_epi32(rounded,_mm_setzero_si128());
      const auto bytes=_mm_packus_epi16(words,_mm_setzero_si128());
      const std::uint32_t packed=std::uint32_t(_mm_cvtsi128_si32(bytes));
      std::memcpy(output+c,&packed,sizeof(packed));
    }
  }
  return c-first;
}
}
std::size_t remap_u8_row_avx2(const void* source,std::uint8_t* output,std::size_t stride,
                             std::size_t width,std::size_t height,const double* matrix,std::size_t row,
                             std::size_t first,std::size_t end,bool bilinear) {
  const auto* input=static_cast<const std::uint8_t*>(source);
  return bilinear ? remap_row<true>(input,output,stride,width,height,matrix,row,first,end)
                  : remap_row<false>(input,output,stride,width,height,matrix,row,first,end);
}

std::size_t remap_float_u8_row_avx2(const std::uint8_t* input,std::uint8_t* output,
    std::uint8_t* validity,std::size_t stride,std::size_t width,std::size_t height,
    const float* rows,const float* cols,std::size_t end,bool& all_valid) {
  const auto zero=_mm256_setzero_pd(),one=_mm256_set1_pd(1.),half=_mm256_set1_pd(.5);
  const auto low=_mm256_set1_pd(-.5),right=_mm256_set1_pd(double(width)-.5),bottom=_mm256_set1_pd(double(height)-.5);
  std::size_t c=0;
  const bool extent_fits=height>0 && (height==1 ||
      stride<=(std::numeric_limits<std::size_t>::max()-width)/(height-1));
  const auto extent=extent_fits ? (height-1)*stride+width : std::size_t{0};
  if(width<=(1u<<23) && height<=(1u<<23) && extent>=4 &&
      stride<=INT32_MAX && extent<=INT32_MAX && std::fegetround()==FE_TONEAREST) {
    const auto z=_mm256_setzero_ps(),o=_mm256_set1_ps(1.f),h=_mm256_set1_ps(.5f);
    const auto min=_mm256_set1_ps(-.5f),maxx=_mm256_set1_ps(float(width)-.5f),maxy=_mm256_set1_ps(float(height)-.5f);
    const auto max_load=_mm256_set1_epi32(int(extent-4)),step=_mm256_set1_epi32(int(stride));
    const auto lanes=_mm256_setr_epi32(0,1,2,3,4,5,6,7),byte_mask=_mm256_set1_epi32(255);
    // 坐标为精确 Float32 输入；64*epsilon*255 的保守绝对包络覆盖权重/值舍入，
    // 可能含糊的半整数按原 Double 表达式重放。
    const auto envelope=_mm256_set1_ps(64.f*std::numeric_limits<float>::epsilon()*255.f);
    for(;end-c>=8;c+=8) {
      auto x=_mm256_loadu_ps(cols+c),y=_mm256_loadu_ps(rows+c);
      const auto valid=_mm256_and_ps(_mm256_and_ps(_mm256_cmp_ps(x,min,_CMP_GE_OQ),_mm256_cmp_ps(x,maxx,_CMP_LT_OQ)),
                                     _mm256_and_ps(_mm256_cmp_ps(y,min,_CMP_GE_OQ),_mm256_cmp_ps(y,maxy,_CMP_LT_OQ)));
      const auto mask=_mm256_movemask_ps(valid);
      all_valid=all_valid && mask==255;
      if(mask==0) {
        _mm_storel_epi64(reinterpret_cast<__m128i*>(output+c),_mm_setzero_si128());
        _mm_storel_epi64(reinterpret_cast<__m128i*>(validity+c),_mm_setzero_si128());
        continue;
      }
      x=_mm256_and_ps(x,valid);y=_mm256_and_ps(y,valid);
      x=_mm256_min_ps(_mm256_max_ps(x,z),_mm256_set1_ps(float(width-1)));
      y=_mm256_min_ps(_mm256_max_ps(y,z),_mm256_set1_ps(float(height-1)));
      const auto xi=_mm256_cvttps_epi32(x),yi=_mm256_cvttps_epi32(y);
      const auto ny=_mm256_min_epi32(_mm256_add_epi32(yi,_mm256_set1_epi32(1)),_mm256_set1_epi32(int(height-1)));
      auto safe=_mm256_castps_si256(valid);
      const auto gather=[&](const __m256i rr,const __m256i cc) {
        const auto offset=_mm256_add_epi32(_mm256_mullo_epi32(rr,step),cc);
        const auto allowed=_mm256_and_si256(_mm256_castps_si256(valid),_mm256_xor_si256(
            _mm256_cmpgt_epi32(offset,max_load),_mm256_set1_epi32(-1)));
        safe=_mm256_and_si256(safe,allowed);
        return _mm256_mask_i32gather_epi32(_mm256_setzero_si256(),reinterpret_cast<const int*>(input),offset,allowed,1);
      };
      const auto dx=_mm256_sub_ps(x,_mm256_cvtepi32_ps(xi)),dy=_mm256_sub_ps(y,_mm256_cvtepi32_ps(yi));
      const auto ex=_mm256_sub_ps(o,dx),ey=_mm256_sub_ps(o,dy);
      __m256 tl,tr,bl,br;
      const auto base_col=_mm_cvtsi128_si32(_mm256_castsi256_si128(xi));
      const auto base_row=_mm_cvtsi128_si32(_mm256_castsi256_si128(yi));
      const auto consecutive=_mm256_cmpeq_epi32(xi,_mm256_add_epi32(_mm256_set1_epi32(base_col),lanes));
      const auto same_row=_mm256_cmpeq_epi32(yi,_mm256_set1_epi32(base_row));
      if(mask==255 && std::size_t(base_col)+8<width &&
          _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_and_si256(consecutive,same_row)))==255) {
        // 九个相邻字节覆盖八像素的两组水平样本；按实际 Map 检查，不假定相机或坐标模式，非规则 Map 安全 gather。
        const auto load=[](const std::uint8_t* p) {
          return _mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(p))));
        };
        const auto* top_row=input+std::size_t(base_row)*stride+base_col;
        const auto* bottom_row=input+std::min(std::size_t(base_row)+1,height-1)*stride+base_col;
        tl=load(top_row);tr=load(top_row+1);bl=load(bottom_row);br=load(bottom_row+1);
      } else if(mask==255 && std::size_t(base_col)+16<=width &&
                _mm256_movemask_ps(_mm256_castsi256_ps(same_row))==255 &&
                [&] {
                  const auto offsets=_mm256_sub_epi32(xi,_mm256_set1_epi32(base_col));
                  const auto outside=_mm256_or_si256(_mm256_cmpgt_epi32(_mm256_setzero_si256(),offsets),
                                                    _mm256_cmpgt_epi32(offsets,_mm256_set1_epi32(14)));
                  return _mm256_movemask_ps(_mm256_castsi256_ps(outside))==0;
                }()) {
        // 保留上方连续路径；已检查的 16 字节行窗内重复/跳列用两次 load 和 shuffle，保持行序及插值顺序。
        const auto offsets=_mm256_sub_epi32(xi,_mm256_set1_epi32(base_col));
        const auto indices=_mm256_add_epi32(_mm256_add_epi32(offsets,_mm256_slli_epi32(offsets,8)),
                                           _mm256_set1_epi32(static_cast<int>(0x80800100u)));
        const auto load=[&](const std::uint8_t* p) {
          return _mm256_shuffle_epi8(_mm256_broadcastsi128_si256(
              _mm_loadu_si128(reinterpret_cast<const __m128i*>(p))),indices);
        };
        const auto top=load(input+std::size_t(base_row)*stride+base_col);
        const auto bottom=load(input+std::min(std::size_t(base_row)+1,height-1)*stride+base_col);
        const auto convert=[&](__m256i bytes){return _mm256_cvtepi32_ps(_mm256_and_si256(bytes,byte_mask));};
        tl=convert(top);tr=convert(_mm256_srli_epi32(top,8));
        bl=convert(bottom);br=convert(_mm256_srli_epi32(bottom,8));
      } else {
        // 已检查的四字节 gather 含相邻字节，直接复用，无需再次读取右邻居。
        const auto top_bytes=gather(yi,xi),bottom_bytes=gather(ny,xi);
        const auto edge=_mm256_cmpeq_epi32(xi,_mm256_set1_epi32(int(width-1)));
        const auto right_bytes=[&](__m256i bytes) {
          return _mm256_blendv_epi8(_mm256_srli_epi32(bytes,8),bytes,edge);
        };
        const auto convert=[&](__m256i bytes){return _mm256_cvtepi32_ps(_mm256_and_si256(bytes,byte_mask));};
        tl=convert(top_bytes);tr=convert(right_bytes(top_bytes));
        bl=convert(bottom_bytes);br=convert(right_bytes(bottom_bytes));
      }
      const auto top=_mm256_add_ps(_mm256_mul_ps(ex,tl),_mm256_mul_ps(dx,tr));
      const auto bot=_mm256_add_ps(_mm256_mul_ps(ex,bl),_mm256_mul_ps(dx,br));
      const auto value=_mm256_add_ps(_mm256_mul_ps(ey,top),_mm256_mul_ps(dy,bot));
      const auto rounded=_mm256_cvttps_epi32(_mm256_add_ps(value,h));
      const auto whole=_mm256_cvttps_epi32(value);
      const auto fraction=_mm256_sub_ps(value,_mm256_cvtepi32_ps(whole));
      const auto distance=_mm256_andnot_ps(_mm256_set1_ps(-0.f),_mm256_sub_ps(fraction,h));
      const auto exact=_mm256_and_ps(_mm256_castsi256_ps(safe),_mm256_cmp_ps(distance,envelope,_CMP_GT_OQ));
      const unsigned fast=_mm256_movemask_ps(exact);
      if(fast==255) {
        const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(output+c),_mm_packus_epi16(words,words));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(validity+c),_mm_set1_epi8(1));
        continue;
      }
      if((_mm256_movemask_ps(_mm256_castsi256_ps(safe)) & mask)==mask) {
        // 含糊块复用寄存器中精确 byte 样本；Float32 坐标减整数部分在此精确或为零。
        // 以 Double 重算 1-dx/1-dy 并保留表达式序；不安全末缓冲 gather 仍按检查后的标量重放。
        const auto replay=[&](bool upper) {
          const auto widen=[&](__m256 v) {
            return _mm256_cvtps_pd(upper ? _mm256_extractf128_ps(v,1) : _mm256_castps256_ps128(v));
          };
          const auto ddx=widen(dx),ddy=widen(dy);
          const auto eex=_mm256_sub_pd(one,ddx),eey=_mm256_sub_pd(one,ddy);
          const auto t=_mm256_add_pd(_mm256_mul_pd(eex,widen(tl)),_mm256_mul_pd(ddx,widen(tr)));
          const auto b=_mm256_add_pd(_mm256_mul_pd(eex,widen(bl)),_mm256_mul_pd(ddx,widen(br)));
          const auto result=_mm256_cvttpd_epi32(_mm256_add_pd(
              _mm256_add_pd(_mm256_mul_pd(eey,t),_mm256_mul_pd(ddy,b)),half));
          const auto flags=upper ? _mm256_extracti128_si256(_mm256_castps_si256(valid),1)
                                 : _mm256_castsi256_si128(_mm256_castps_si256(valid));
          return _mm_and_si128(result,flags);
        };
        const auto words=_mm_packus_epi32(replay(false),replay(true));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(output+c),_mm_packus_epi16(words,words));
        for(unsigned j=0;j<8;++j)validity[c+j]=(mask>>j)&1;
        continue;
      }
      std::int32_t result[8];_mm256_storeu_si256(reinterpret_cast<__m256i*>(result),rounded);
      for(unsigned j=0;j<8;++j) {
        const bool inside=(mask&(1u<<j))!=0;validity[c+j]=inside;
        if(!inside){output[c+j]=0;continue;}
        if(fast&(1u<<j)){output[c+j]=static_cast<std::uint8_t>(result[j]);continue;}
        float source_x,source_y;
        std::memcpy(&source_x,cols+c+j,sizeof(source_x));
        std::memcpy(&source_y,rows+c+j,sizeof(source_y));
        double xx=source_x,yy=source_y;xx=std::clamp(xx,0.,double(width-1));yy=std::clamp(yy,0.,double(height-1));
        const auto ix=std::size_t(xx),iy=std::size_t(yy),nextx=std::min(ix+1,width-1),nexty=std::min(iy+1,height-1);
        const auto ddx=xx-ix,ddy=yy-iy;
        const double reference=(1-ddy)*((1-ddx)*input[iy*stride+ix]+ddx*input[iy*stride+nextx])+
            ddy*((1-ddx)*input[nexty*stride+ix]+ddx*input[nexty*stride+nextx]);
        output[c+j]=static_cast<std::uint8_t>(reference+.5);
      }
    }
  }
  for(;end-c>=4;c+=4) {
    auto x=_mm256_cvtps_pd(_mm_loadu_ps(cols+c)),y=_mm256_cvtps_pd(_mm_loadu_ps(rows+c));
    const auto valid=_mm256_and_pd(_mm256_and_pd(_mm256_cmp_pd(x,low,_CMP_GE_OQ),_mm256_cmp_pd(x,right,_CMP_LT_OQ)),
                                   _mm256_and_pd(_mm256_cmp_pd(y,low,_CMP_GE_OQ),_mm256_cmp_pd(y,bottom,_CMP_LT_OQ)));
    const auto mask=_mm256_movemask_pd(valid);
    all_valid=all_valid && mask==15;
    x=_mm256_blendv_pd(zero,x,valid);y=_mm256_blendv_pd(zero,y,valid);
    x=_mm256_min_pd(_mm256_max_pd(x,zero),_mm256_set1_pd(double(width-1)));
    y=_mm256_min_pd(_mm256_max_pd(y,zero),_mm256_set1_pd(double(height-1)));
    const auto xi=_mm256_cvttpd_epi32(x),yi=_mm256_cvttpd_epi32(y);
    std::int32_t ix[4],iy[4],nx[4],ny[4];
    _mm_storeu_si128(reinterpret_cast<__m128i*>(ix),xi);_mm_storeu_si128(reinterpret_cast<__m128i*>(iy),yi);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(nx),_mm_min_epi32(_mm_add_epi32(xi,_mm_set1_epi32(1)),_mm_set1_epi32(int(width-1))));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(ny),_mm_min_epi32(_mm_add_epi32(yi,_mm_set1_epi32(1)),_mm_set1_epi32(int(height-1))));
    const auto dx=_mm256_sub_pd(x,_mm256_cvtepi32_pd(xi)),dy=_mm256_sub_pd(y,_mm256_cvtepi32_pd(yi));
    const auto ex=_mm256_sub_pd(one,dx),ey=_mm256_sub_pd(one,dy);
    const auto gather=[&](const std::int32_t* rr,const std::int32_t* cc) {
      return _mm256_cvtepi32_pd(_mm_set_epi32(input[std::size_t(rr[3])*stride+cc[3]],input[std::size_t(rr[2])*stride+cc[2]],
                                             input[std::size_t(rr[1])*stride+cc[1]],input[std::size_t(rr[0])*stride+cc[0]]));
    };
    const auto top=_mm256_add_pd(_mm256_mul_pd(ex,gather(iy,ix)),_mm256_mul_pd(dx,gather(iy,nx)));
    const auto bot=_mm256_add_pd(_mm256_mul_pd(ex,gather(ny,ix)),_mm256_mul_pd(dx,gather(ny,nx)));
    const auto value=_mm256_add_pd(_mm256_mul_pd(ey,top),_mm256_mul_pd(dy,bot));
    std::int32_t result[4];_mm_storeu_si128(reinterpret_cast<__m128i*>(result),_mm256_cvttpd_epi32(_mm256_add_pd(value,half)));
    for(unsigned j=0;j<4;++j) {
      const bool inside=(mask&(1u<<j))!=0;
      validity[c+j]=inside;output[c+j]=inside ? static_cast<std::uint8_t>(result[j]) : 0;
    }
  }
  return c;
}

std::size_t projective_u8_row_avx2(const std::uint8_t* input,std::uint8_t* output,std::uint8_t* validity,
    std::size_t stride,std::size_t width,std::size_t height,const double* m,std::size_t row,std::size_t end,bool bilinear) {
  const auto zero=_mm256_setzero_pd(),one=_mm256_set1_pd(1.),half=_mm256_set1_pd(.5);
  const auto low=_mm256_set1_pd(-.5),right=_mm256_set1_pd(double(width)-.5),bottom=_mm256_set1_pd(double(height)-.5);
  const auto xr=_mm256_set1_pd(m[1]*row),yr=_mm256_set1_pd(m[4]*row),zr=_mm256_set1_pd(m[7]*row);
  const auto mx=_mm256_set1_pd(m[0]),my=_mm256_set1_pd(m[3]),mz=_mm256_set1_pd(m[6]);
  const auto tx=_mm256_set1_pd(m[2]),ty=_mm256_set1_pd(m[5]),tz=_mm256_set1_pd(m[8]);
  std::size_t c=0;
  for(;end-c>=4;c+=4) {
    const auto columns=_mm256_add_pd(_mm256_set1_pd(double(c)),_mm256_setr_pd(0.,1.,2.,3.));
    const auto denominator=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(mz,columns),zr),tz);
    const auto nonzero=_mm256_cmp_pd(denominator,zero,_CMP_NEQ_OQ);
    const auto divisor=_mm256_blendv_pd(one,denominator,nonzero);
    auto x=_mm256_div_pd(_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(mx,columns),xr),tx),divisor);
    auto y=_mm256_div_pd(_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(my,columns),yr),ty),divisor);
    const auto valid=_mm256_and_pd(nonzero,_mm256_and_pd(
        _mm256_and_pd(_mm256_cmp_pd(x,low,_CMP_GE_OQ),_mm256_cmp_pd(x,right,_CMP_LT_OQ)),
        _mm256_and_pd(_mm256_cmp_pd(y,low,_CMP_GE_OQ),_mm256_cmp_pd(y,bottom,_CMP_LT_OQ))));
    const unsigned mask=_mm256_movemask_pd(valid);
    if(!mask) {std::uint32_t empty=0;std::memcpy(output+c,&empty,4);std::memcpy(validity+c,&empty,4);continue;}
    x=_mm256_blendv_pd(zero,x,valid);y=_mm256_blendv_pd(zero,y,valid);
    std::int32_t ix[4],iy[4],result[4];
    __m128i rounded;
    if(!bilinear) {
      _mm_storeu_si128(reinterpret_cast<__m128i*>(ix),_mm256_cvttpd_epi32(_mm256_add_pd(x,half)));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(iy),_mm256_cvttpd_epi32(_mm256_add_pd(y,half)));
      for(unsigned j=0;j<4;++j){validity[c+j]=(mask>>j)&1;output[c+j]=validity[c+j]?input[std::size_t(iy[j])*stride+ix[j]]:0;}
      continue;
    }
    x=_mm256_min_pd(_mm256_max_pd(x,zero),_mm256_set1_pd(double(width-1)));
    y=_mm256_min_pd(_mm256_max_pd(y,zero),_mm256_set1_pd(double(height-1)));
    const auto xi=_mm256_cvttpd_epi32(x),yi=_mm256_cvttpd_epi32(y);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(ix),xi);_mm_storeu_si128(reinterpret_cast<__m128i*>(iy),yi);
    const auto dx=_mm256_sub_pd(x,_mm256_cvtepi32_pd(xi)),dy=_mm256_sub_pd(y,_mm256_cvtepi32_pd(yi));
    const auto ex=_mm256_sub_pd(one,dx),ey=_mm256_sub_pd(one,dy);
    const auto samples=bilinear_samples_u8(input,stride,width,height,xi,yi);
    const auto top=_mm256_add_pd(_mm256_mul_pd(ex,samples.tl),_mm256_mul_pd(dx,samples.tr));
    const auto bot=_mm256_add_pd(_mm256_mul_pd(ex,samples.bl),_mm256_mul_pd(dx,samples.br));
    rounded=_mm256_cvttpd_epi32(_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(ey,top),_mm256_mul_pd(dy,bot)),half));
    if(mask==15) {
      const auto words=_mm_packus_epi32(rounded,_mm_setzero_si128());
      const auto bytes=_mm_packus_epi16(words,_mm_setzero_si128());
      const std::uint32_t packed=std::uint32_t(_mm_cvtsi128_si32(bytes)),all_valid=0x01010101u;
      std::memcpy(output+c,&packed,4);std::memcpy(validity+c,&all_valid,4);
    } else {
      _mm_storeu_si128(reinterpret_cast<__m128i*>(result),rounded);
      for(unsigned j=0;j<4;++j){validity[c+j]=(mask>>j)&1;output[c+j]=validity[c+j]?std::uint8_t(result[j]):0;}
    }
  }
  return c;
}

void u16_moments_avx2(const void* data,std::size_t stride,std::span<const core::RegionRun> runs,
                      std::int64_t* sum,std::uint64_t* square) {
  const auto* base=static_cast<const std::byte*>(data);
  auto sums=_mm256_setzero_si256(),squares=_mm256_setzero_si256();
  std::int64_t tail_sum=0;std::uint64_t tail_square=0;
  for(const auto& run:runs) {
    const auto* row=reinterpret_cast<const std::uint16_t*>(base+std::int64_t(run.row)*stride);auto c=std::int64_t(run.col_begin);
    for(;std::int64_t(run.col_end)-c>=15;c+=16) {
      const auto v=_mm256_xor_si256(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(row+c)),_mm256_set1_epi16(-32768));
      const auto s=_mm256_madd_epi16(v,_mm256_set1_epi16(1)),q=_mm256_madd_epi16(v,v);
      sums=_mm256_add_epi64(sums,_mm256_add_epi64(_mm256_cvtepi32_epi64(_mm256_castsi256_si128(s)),
                                                 _mm256_cvtepi32_epi64(_mm256_extracti128_si256(s,1))));
      squares=_mm256_add_epi64(squares,_mm256_add_epi64(_mm256_cvtepu32_epi64(_mm256_castsi256_si128(q)),
                                                       _mm256_cvtepu32_epi64(_mm256_extracti128_si256(q,1))));
    }
    for(;c<=run.col_end;++c) {
      std::uint16_t raw;std::memcpy(&raw,row+c,sizeof(raw));const auto v=std::int64_t(raw)-32768;
      tail_sum+=v;tail_square+=std::uint64_t(v*v);
    }
  }
  std::int64_t sv[4];std::uint64_t qv[4];_mm256_storeu_si256(reinterpret_cast<__m256i*>(sv),sums);_mm256_storeu_si256(reinterpret_cast<__m256i*>(qv),squares);
  for(int i=0;i<4;++i){tail_sum+=sv[i];tail_square+=qv[i];}*sum=tail_sum;*square=tail_square;
}

namespace {
template<bool Float, bool Square>
bool shifted_sum(const void* data,std::size_t stride,std::span<const core::RegionRun> runs,
                 double anchor,double offset,double* result) {
  using T=std::conditional_t<Float,float,std::uint16_t>;
  const auto* base=static_cast<const std::byte*>(data);
  const auto origin=_mm256_set1_pd(anchor),center=_mm256_set1_pd(offset);
  __m256d sums[4]{_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd()};
  __m128i invalid[4]{_mm_setzero_si128(),_mm_setzero_si128(),_mm_setzero_si128(),_mm_setzero_si128()};double tail[4]{};
  const auto update=[&](const T* p,int lane) {
    __m256d value;
    if constexpr(Float) {
      const auto raw=_mm_loadu_ps(p);
      if constexpr(!Square)invalid[lane]=_mm_or_si128(invalid[lane],_mm_cmpeq_epi32(
          _mm_and_si128(_mm_castps_si128(raw),_mm_set1_epi32(0x7f800000)),_mm_set1_epi32(0x7f800000)));
      value=_mm256_cvtps_pd(raw);
    } else value=_mm256_cvtepi32_pd(_mm_cvtepu16_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(p))));
    value=_mm256_sub_pd(value,origin);
    if constexpr(Square){value=_mm256_sub_pd(value,center);value=_mm256_mul_pd(value,value);}
    sums[lane]=_mm256_add_pd(sums[lane],value);
  };
  for(const auto& run:runs) {
    const auto* row=reinterpret_cast<const T*>(base+std::int64_t(run.row)*stride);auto c=std::int64_t(run.col_begin);
    for(;std::int64_t(run.col_end)-c>=15;c+=16)for(int k=0;k<4;++k)update(row+c+k*4,k);
    for(;std::int64_t(run.col_end)-c>=3;c+=4)update(row+c,0);
    for(;c<=run.col_end;++c) {
      T raw;std::memcpy(&raw,row+c,sizeof(raw));const double value=raw;
      if constexpr(Float && !Square)if(!core::is_finite(value))return false;
      const auto delta=(value-anchor)-(Square ? offset : 0.);
      tail[c%4]+=Square ? delta*delta : delta;
    }
  }
  if constexpr(Float && !Square)if(_mm_movemask_epi8(_mm_or_si128(_mm_or_si128(invalid[0],invalid[1]),_mm_or_si128(invalid[2],invalid[3]))))return false;
  double values[4];_mm256_storeu_pd(values,_mm256_add_pd(_mm256_add_pd(sums[0],sums[1]),_mm256_add_pd(sums[2],sums[3])));
  *result=((values[0]+values[1])+(values[2]+values[3]))+((tail[0]+tail[1])+(tail[2]+tail[3]));return true;
}
}
bool shifted_sum_avx2(const void* data,std::size_t stride,std::span<const core::RegionRun> runs,
                      bool floating,double anchor,double offset,bool square,double* result) {
  if(floating)return square ? shifted_sum<true,true>(data,stride,runs,anchor,offset,result)
                            : shifted_sum<true,false>(data,stride,runs,anchor,offset,result);
  return square ? shifted_sum<false,true>(data,stride,runs,anchor,offset,result)
                : shifted_sum<false,false>(data,stride,runs,anchor,offset,result);
}

int f32_block_moments_avx2(const void* data,std::size_t stride,std::span<const core::RegionRun> runs,
                           double anchor,double* offset,double* m2,std::uint64_t* count) {
  if(!core::is_finite(anchor))return -1;
  const auto origin=_mm256_set1_pd(anchor);const auto* base=static_cast<const std::byte*>(data);
  double mean=0,total_m2=0;std::uint64_t total_count=0;
  for(const auto& run:runs) {
    const auto* row=reinterpret_cast<const float*>(base+std::int64_t(run.row)*stride);
    for(auto first=std::int64_t(run.col_begin);first<=run.col_end;) {
      const auto n=std::min<std::int64_t>(256,std::int64_t(run.col_end)-first+1),end=first+n;
      __m256d sums[4]{},squares[4]{};__m128i invalid=_mm_setzero_si128();
      for(int k=0;k<4;++k){sums[k]=_mm256_setzero_pd();squares[k]=_mm256_setzero_pd();}
      auto c=first;
      for(;end-c>=16;c+=16)for(int k=0;k<4;++k) {
        const auto raw=_mm_loadu_ps(row+c+4*k);
        invalid=_mm_or_si128(invalid,_mm_cmpeq_epi32(_mm_and_si128(_mm_castps_si128(raw),_mm_set1_epi32(0x7f800000)),_mm_set1_epi32(0x7f800000)));
        const auto d=_mm256_sub_pd(_mm256_cvtps_pd(raw),origin);
        sums[k]=_mm256_add_pd(sums[k],d);squares[k]=_mm256_add_pd(squares[k],_mm256_mul_pd(d,d));
      }
      if(_mm_movemask_epi8(invalid))return -1;
      double values[4],qvalues[4];
      _mm256_storeu_pd(values,_mm256_add_pd(_mm256_add_pd(sums[0],sums[1]),_mm256_add_pd(sums[2],sums[3])));
      _mm256_storeu_pd(qvalues,_mm256_add_pd(_mm256_add_pd(squares[0],squares[1]),_mm256_add_pd(squares[2],squares[3])));
      double sum=(values[0]+values[1])+(values[2]+values[3]),square=(qvalues[0]+qvalues[1])+(qvalues[2]+qvalues[3]);
      for(;c<end;++c) {
        const double raw=row[c];if(!core::is_finite(raw))return -1;
        const auto d=raw-anchor;sum+=d;square+=d*d;
      }
      const auto inverse_n=1./double(n);
      const double block_mean=sum*inverse_n,second=square*inverse_n,variance=second-block_mean*block_mean;
      // 最多 256 项的保守浮点误差界；相消或常量块回退既有中心化两遍内核。
      const auto bound=64*std::numeric_limits<double>::epsilon()*double(n)*(second+block_mean*block_mean);
      if(n>1 && square!=0 && (!(variance>0) || bound>1e-10*variance))return 0;
      const auto block_m2=n==1 ? 0. : std::max(0.,variance)*double(n);
      const auto next=total_count+std::uint64_t(n);const auto delta=block_mean-mean;
      const auto inverse_next=1./double(next);
      total_m2+=block_m2+delta*delta*(double(total_count)*double(n)*inverse_next);
      mean+=delta*(double(n)*inverse_next);total_count=next;first=end;
    }
  }
  *offset=mean;*m2=total_m2;*count=total_count;return 1;
}

namespace {
template<int Type>
void extrema(const void* data,std::size_t stride,const core::Region& domain,double* minimum,double* maximum) {
  using T=std::conditional_t<Type==0,std::uint8_t,std::conditional_t<Type==1,std::uint16_t,float>>;
  using V=std::conditional_t<Type==2,__m256,__m256i>;
  constexpr int lanes=32/sizeof(T);
  const auto initial_min=[] {if constexpr(Type==2)return _mm256_set1_ps(std::numeric_limits<float>::infinity());
                            else return _mm256_set1_epi32(-1);};
  const auto initial_max=[] {if constexpr(Type==2)return _mm256_set1_ps(-std::numeric_limits<float>::infinity());
                            else return _mm256_setzero_si256();};
  V low[4],high[4];
  for(int i=0;i<4;++i){low[i]=initial_min();high[i]=initial_max();}
  double lo=std::numeric_limits<double>::infinity(),hi=-std::numeric_limits<double>::infinity();
  const auto* base=static_cast<const std::byte*>(data);
  const auto update=[&](const std::byte* p,int i) {
    if constexpr(Type==2) {
      const auto v=_mm256_loadu_ps(reinterpret_cast<const float*>(p));
      const auto valid=_mm256_cmp_ps(v,v,_CMP_ORD_Q);
      low[i]=_mm256_min_ps(low[i],_mm256_blendv_ps(initial_min(),v,valid));
      high[i]=_mm256_max_ps(high[i],_mm256_blendv_ps(initial_max(),v,valid));
    } else {
      const auto v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(p));
      if constexpr(Type==0){low[i]=_mm256_min_epu8(low[i],v);high[i]=_mm256_max_epu8(high[i],v);}
      else {low[i]=_mm256_min_epu16(low[i],v);high[i]=_mm256_max_epu16(high[i],v);}
    }
  };
  for(const auto& run:domain.runs()) {
    const auto* row=base+static_cast<std::size_t>(run.row)*stride;
    auto c=std::int64_t(run.col_begin);
    for(;std::int64_t(run.col_end)-c+1>=4*lanes;c+=4*lanes) {
      if constexpr(Type==2) {
        __m256 values[4];
        for(int i=0;i<4;++i)values[i]=_mm256_loadu_ps(reinterpret_cast<const float*>(row)+(c+i*lanes));
        const auto valid01=_mm256_and_ps(_mm256_cmp_ps(values[0],values[0],_CMP_ORD_Q),_mm256_cmp_ps(values[1],values[1],_CMP_ORD_Q));
        const auto valid23=_mm256_and_ps(_mm256_cmp_ps(values[2],values[2],_CMP_ORD_Q),_mm256_cmp_ps(values[3],values[3],_CMP_ORD_Q));
        if(_mm256_movemask_ps(_mm256_and_ps(valid01,valid23))==255) {
          for(int i=0;i<4;++i){low[i]=_mm256_min_ps(low[i],values[i]);high[i]=_mm256_max_ps(high[i],values[i]);}
        } else {
          for(int i=0;i<4;++i) {
            const auto valid=_mm256_cmp_ps(values[i],values[i],_CMP_ORD_Q);
            low[i]=_mm256_min_ps(low[i],_mm256_blendv_ps(initial_min(),values[i],valid));
            high[i]=_mm256_max_ps(high[i],_mm256_blendv_ps(initial_max(),values[i],valid));
          }
        }
      } else for(int i=0;i<4;++i)update(row+(c+i*lanes)*sizeof(T),i);
    }
    for(;std::int64_t(run.col_end)-c+1>=lanes;c+=lanes)update(row+c*sizeof(T),0);
    if constexpr(Type!=2) {
      // 整数极值对重复样本幂等；末块向前重叠读取，避免长游程尾部逐像素转 Double。
      if(c<=run.col_end && std::int64_t(run.col_end)-run.col_begin+1>=lanes) {
        update(row+(std::int64_t(run.col_end)-lanes+1)*sizeof(T),0);
        c=std::int64_t(run.col_end)+1;
      }
    }
    for(;c<=run.col_end;++c){T value;std::memcpy(&value,row+c*sizeof(T),sizeof(T));lo=std::min(lo,double(value));hi=std::max(hi,double(value));}
    if constexpr(Type==0) {
      bool found_lo=lo==0,found_hi=hi==255;
      for(int i=0;i<4;++i) {
        found_lo|=_mm256_movemask_epi8(_mm256_cmpeq_epi8(low[i],_mm256_setzero_si256()))!=0;
        found_hi|=_mm256_movemask_epi8(_mm256_cmpeq_epi8(high[i],_mm256_set1_epi32(-1)))!=0;
      }
      if(found_lo && found_hi){*minimum=0;*maximum=255;return;}
    }
  }
  alignas(32) T lows[lanes],highs[lanes];
  for(int i=0;i<4;++i) {
    if constexpr(Type==2){_mm256_store_ps(lows,low[i]);_mm256_store_ps(highs,high[i]);}
    else {_mm256_store_si256(reinterpret_cast<__m256i*>(lows),low[i]);_mm256_store_si256(reinterpret_cast<__m256i*>(highs),high[i]);}
    for(int j=0;j<lanes;++j){lo=std::min(lo,double(lows[j]));hi=std::max(hi,double(highs[j]));}
  }
  if constexpr(Type==2) {
    // std::min/max 保留首个同值，包含零的符号。
    if(lo==0 || hi==0) {
      bool found=false;
      for(const auto& run:domain.runs()) {
        const auto* row=base+static_cast<std::size_t>(run.row)*stride;
        for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c) {
          float v;std::memcpy(&v,row+c*sizeof(float),sizeof(v));
          if(v==0){if(lo==0)lo=v;if(hi==0)hi=v;found=true;break;}
        }
        if(found)break;
      }
    }
  }
  *minimum=lo;*maximum=hi;
}
}
void min_max_gray_avx2(const void* data,std::size_t stride,const core::Region& domain,int type,double* lo,double* hi) {
  if(type==0)extrema<0>(data,stride,domain,lo,hi);
  else if(type==1)extrema<1>(data,stride,domain,lo,hi);
  else extrema<2>(data,stride,domain,lo,hi);
}

std::size_t divide_u8_row_avx2(const std::uint8_t* a,const std::uint8_t* b,std::uint8_t* out,
                              std::size_t begin,std::size_t end,double mult,double add) {
  const auto factor=_mm256_set1_ps(float(mult)),offset=_mm256_set1_ps(float(add));
  const auto zero=_mm256_setzero_ps(),half=_mm256_set1_ps(.5f);
  auto c=begin;
  for(;c+8<=end;c+=8) {
    const auto x=_mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(a+c))));
    const auto y=_mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(b+c))));
    const auto valid=_mm256_cmp_ps(y,zero,_CMP_NEQ_OQ);
    const auto denominator=_mm256_blendv_ps(_mm256_set1_ps(1),y,valid);
    auto value=_mm256_and_ps(valid,_mm256_add_ps(_mm256_mul_ps(_mm256_div_ps(x,denominator),factor),offset));
    value=_mm256_min_ps(value,_mm256_set1_ps(255));
    auto integral=_mm256_cvttps_epi32(value);
    // 比较小数部分；直接加 .5f 会把略低于半界的值提前舍入，与标量 llround 不同。
    const auto up=_mm256_cmp_ps(_mm256_sub_ps(value,_mm256_cvtepi32_ps(integral)),half,_CMP_GE_OQ);
    integral=_mm256_sub_epi32(integral,_mm256_castps_si256(up));
    const auto words=_mm_packus_epi32(_mm256_castsi256_si128(integral),_mm256_extracti128_si256(integral,1));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
  }
  return c;
}

std::size_t multiply_u16_row_avx2(const std::uint16_t* a,const std::uint16_t* b,std::uint16_t* out,
                                 std::size_t begin,std::size_t end,double mult,double add) {
  const double candidate=mult>0?std::round(1./mult):0;
  if(add==0 && candidate>=2 && candidate<=65535 && 1./candidate==mult) {
    const auto divisor=static_cast<std::uint32_t>(candidate);
    // 饱和后 d<=255 的乘积可由 float 精确表示；65536 以下除法误差最多 1/512，
    // 小于非半商到半界的最小距离 1/(2*255)。精确半值仅在原序 Double 倒数不低于 1/d 时向上取整，
    // 否则保留下方精确同值修正。
    const auto reciprocal_not_below=[&] {
      // d<=255 时 m*d 与 2^(52-e) 均在 uint64 内；直接比较存储 Double 和精确有理值，无需 FMA。
      const auto bits=std::bit_cast<std::uint64_t>(mult);
      const auto mantissa=(bits&((std::uint64_t{1}<<52)-1))|(std::uint64_t{1}<<52);
      const auto shift=1075-static_cast<unsigned>((bits>>52)&2047);
      return mantissa*divisor>=(std::uint64_t{1}<<shift);
    };
    if(divisor<=255 && ((divisor&1)!=0 || reciprocal_not_below())) {
      const auto cap=_mm256_set1_epi32(static_cast<int>(divisor*65535u));
      const auto denominator=_mm256_set1_ps(static_cast<float>(divisor));
      const auto half=_mm256_set1_ps(.5f);
      const auto convert=[&](__m256i product) {
        const auto value=_mm256_div_ps(_mm256_cvtepi32_ps(_mm256_min_epu32(product,cap)),denominator);
        return _mm256_cvttps_epi32(_mm256_add_ps(value,half));
      };
      auto c=begin;
      for(;c+16<=end;c+=16) {
        const auto x=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(a+c));
        const auto y=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(b+c));
        const auto lo=_mm256_mullo_epi16(x,y),hi=_mm256_mulhi_epu16(x,y);
        const auto q0=convert(_mm256_unpacklo_epi16(lo,hi));
        const auto q1=convert(_mm256_unpackhi_epi16(lo,hi));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(out+c),_mm256_packus_epi32(q0,q1));
      }
      return c;
    }
    const auto d=_mm256_set1_epi32(static_cast<int>(divisor));
    const auto reciprocal=_mm256_set1_epi32(static_cast<int>((0x100000000ULL+divisor-1)/divisor));
    const auto bias=_mm256_set1_epi32(static_cast<int>(divisor/2));
    auto c=begin;
    for(;c+8<=end;c+=8) {
      const auto x=_mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+c)));
      const auto y=_mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+c)));
      const auto product=_mm256_mullo_epi32(x,y),numerator=_mm256_add_epi32(product,bias);
      const auto saturation=_mm256_set1_epi32(static_cast<int>(divisor*65535u));
      if(_mm256_movemask_epi8(_mm256_cmpeq_epi32(_mm256_min_epu32(product,saturation),saturation))==-1) {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),_mm_set1_epi16(-1));
        continue;
      }
      __m256i quotient;
      {
        const auto even=_mm256_mul_epu32(numerator,reciprocal);
        const auto odd=_mm256_mul_epu32(_mm256_srli_epi64(numerator,32),reciprocal);
        quotient=_mm256_or_si256(_mm256_srli_epi64(even,32),
                                _mm256_slli_epi64(_mm256_srli_epi64(odd,32),32));
        const auto estimated=_mm256_mullo_epi32(quotient,d);
        const auto valid=_mm256_cmpeq_epi32(_mm256_min_epu32(estimated,numerator),estimated);
        quotient=_mm256_add_epi32(quotient,_mm256_xor_si256(valid,_mm256_set1_epi32(-1)));
      }
      const auto bounded=_mm256_min_epu32(quotient,_mm256_set1_epi32(65535));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi32(
          _mm256_castsi256_si128(bounded),_mm256_extracti128_si256(bounded,1)));
      if((divisor&1)==0) {
        const auto ties=_mm256_cmpeq_epi32(numerator,_mm256_mullo_epi32(quotient,d));
        const auto unsaturated=_mm256_cmpeq_epi32(quotient,bounded);
        unsigned mask=static_cast<unsigned>(_mm256_movemask_ps(_mm256_castsi256_ps(_mm256_and_si256(ties,unsaturated))));
        while(mask) {
          const auto lane=std::countr_zero(mask);mask&=mask-1;
          std::uint16_t ax,by;std::memcpy(&ax,a+c+lane,2);std::memcpy(&by,b+c+lane,2);
          out[c+lane]=static_cast<std::uint16_t>(std::llround(std::min(65535.,(double(ax)*by)*mult)));
        }
      }
    }
    return c;
  }
  const auto factor=_mm256_set1_pd(mult),offset=_mm256_set1_pd(add);
  auto c=begin;
  for(;c+4<=end;c+=4) {
    const auto x=_mm256_cvtepi32_pd(_mm_cvtepu16_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(a+c))));
    const auto y=_mm256_cvtepi32_pd(_mm_cvtepu16_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(b+c))));
    auto value=_mm256_add_pd(_mm256_mul_pd(_mm256_mul_pd(x,y),factor),offset);
    value=_mm256_min_pd(_mm256_max_pd(value,_mm256_setzero_pd()),_mm256_set1_pd(65535));
    const auto base=_mm256_round_pd(value,_MM_FROUND_TO_NEG_INF|_MM_FROUND_NO_EXC);
    const auto up=_mm256_cmp_pd(_mm256_sub_pd(value,base),_mm256_set1_pd(.5),_CMP_GE_OQ);
    const auto result=_mm256_cvttpd_epi32(_mm256_add_pd(base,_mm256_and_pd(up,_mm256_set1_pd(1))));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi32(result,result));
  }
  return c;
}
void threshold_masks_avx2(const void* data,std::size_t begin,std::size_t blocks,
                          double low,double high,int kind,std::uint32_t* masks) {
  if(kind<2) {
    low=std::max(0.,std::ceil(low));high=std::min(kind==0?255.:65535.,std::floor(high));
    if(low>high){std::fill_n(masks,blocks,0);return;}
    if(kind==0) {
      const auto l=_mm256_set1_epi8(static_cast<char>(static_cast<unsigned>(low)));
      const auto h=_mm256_set1_epi8(static_cast<char>(static_cast<unsigned>(high)));
      const auto* bytes=static_cast<const std::uint8_t*>(data)+begin;
      for(std::size_t i=0;i<blocks;++i) {
        const auto v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(bytes+i*32));
        masks[i]=static_cast<std::uint32_t>(_mm256_movemask_epi8(_mm256_cmpeq_epi8(v,
            _mm256_min_epu8(_mm256_max_epu8(v,l),h))));
      }
    } else {
      const auto l=_mm256_set1_epi16(static_cast<short>(static_cast<unsigned>(low)));
      const auto h=_mm256_set1_epi16(static_cast<short>(static_cast<unsigned>(high)));
      const auto* words=static_cast<const std::uint16_t*>(data)+begin;
      auto check=[&](__m256i v){return _mm256_cmpeq_epi16(v,_mm256_min_epu16(_mm256_max_epu16(v,l),h));};
      for(std::size_t i=0;i<blocks;++i) {
        const auto a=check(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(words+i*32)));
        const auto b=check(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(words+i*32+16)));
        masks[i]=static_cast<std::uint32_t>(_mm256_movemask_epi8(_mm256_permute4x64_epi64(_mm256_packs_epi16(a,b),0xd8)));
      }
    }
  } else {
    float l=static_cast<float>(low),h=static_cast<float>(high);
    if(double(l)<low)l=std::nextafter(l,std::numeric_limits<float>::infinity());
    if(double(h)>high)h=std::nextafter(h,-std::numeric_limits<float>::infinity());
    const auto lv=_mm256_set1_ps(l),hv=_mm256_set1_ps(h);
    const auto* values=static_cast<const float*>(data)+begin;
    for(std::size_t i=0;i<blocks;++i) {
      std::uint32_t mask=0;
      for(int j=0;j<4;++j) {
        const auto v=_mm256_loadu_ps(values+i*32+j*8);
        mask|=static_cast<std::uint32_t>(_mm256_movemask_ps(_mm256_and_ps(
            _mm256_cmp_ps(v,lv,_CMP_GE_OQ),_mm256_cmp_ps(v,hv,_CMP_LE_OQ))))<<(j*8);
      }
      masks[i]=mask;
    }
  }
}

namespace {
template<int Operation>
std::size_t float_row(const float* a,const float* b,float* out,std::size_t begin,
                      std::size_t end,double mult,double add) {
  const auto factor=_mm256_set1_pd(mult),offset=_mm256_set1_pd(add);
  auto c=begin;
  for(;c+4<=end;c+=4) {
    const auto x=_mm256_cvtps_pd(_mm_loadu_ps(a+c));
    __m256d value;
    if constexpr(Operation==4)value=_mm256_xor_pd(x,_mm256_set1_pd(-0.));
    else {
      const auto y=_mm256_cvtps_pd(_mm_loadu_ps(b+c));
      if constexpr(Operation==0)value=_mm256_add_pd(x,y);
      if constexpr(Operation==1)value=_mm256_sub_pd(x,y);
      if constexpr(Operation==2)value=_mm256_mul_pd(x,y);
        if constexpr(Operation==3)value=_mm256_andnot_pd(_mm256_set1_pd(-0.),_mm256_sub_pd(x,y));
        if constexpr(Operation==5) {
          const auto nonzero=_mm256_cmp_pd(y,_mm256_setzero_pd(),_CMP_NEQ_UQ);
          value=_mm256_div_pd(x,_mm256_blendv_pd(_mm256_set1_pd(1),y,nonzero));
        }
        value=_mm256_mul_pd(value,factor);
        if constexpr(Operation!=3)value=_mm256_add_pd(value,offset);
        if constexpr(Operation==5)value=_mm256_and_pd(value,_mm256_cmp_pd(y,_mm256_setzero_pd(),_CMP_NEQ_UQ));
    }
    _mm_storeu_ps(out+c,_mm256_cvtpd_ps(value));
  }
  return c;
}
}
std::size_t binary_f32_row_avx2(const float* a,const float* b,float* out,std::size_t begin,
                               std::size_t end,double mult,double add,int operation) {
  switch(operation) {
    case 0:return float_row<0>(a,b,out,begin,end,mult,add);
    case 1:return float_row<1>(a,b,out,begin,end,mult,add);
    case 2:return float_row<2>(a,b,out,begin,end,mult,add);
    case 3:return float_row<3>(a,b,out,begin,end,mult,add);
    case 4:return float_row<4>(a,b,out,begin,end,mult,add);
    case 5:return float_row<5>(a,b,out,begin,end,mult,add);
    default:return begin;
  }
}

std::size_t binary_u16_half_row_avx2(const std::uint16_t* a,const std::uint16_t* b,std::uint16_t* out,
                                    std::size_t begin,std::size_t end,int factor,int offset,bool subtract) {
  auto c=begin;
  if(!subtract && factor==1 && offset>=0 && (offset&1)==0 && offset/2<=65535) {
    const auto add=_mm256_set1_epi16(static_cast<short>(offset/2));
    for(;c+16<=end;c+=16) {
      const auto x=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(a+c));
      const auto y=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(b+c));
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(out+c),_mm256_adds_epu16(_mm256_avg_epu16(x,y),add));
    }
    return c;
  }
  const auto m=_mm256_set1_epi32(factor),n=_mm256_set1_epi32(offset+1);
  for(;c+8<=end;c+=8) {
    const auto x=_mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+c)));
    const auto y=_mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+c)));
    const auto value=subtract?_mm256_sub_epi32(x,y):_mm256_add_epi32(x,y);
    const auto rounded=_mm256_srai_epi32(_mm256_add_epi32(_mm256_mullo_epi32(value,m),n),1);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi32(
        _mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1)));
  }
  return c;
}

std::size_t certified_affine_u8_row_avx2(const std::uint8_t* source, std::uint8_t* output,
    std::size_t begin, std::size_t end, const CertifiedByteAffine& affine) {
  const auto factor=_mm256_set1_epi16(static_cast<short>(affine.factor));
  const auto offset=_mm256_set1_epi16(static_cast<short>(affine.offset));
  const auto limit=_mm256_set1_epi16(static_cast<short>(affine.limit));
  const auto maximum=_mm256_set1_epi16(255);
  const auto zero=_mm256_setzero_si256();
  const auto transform=[&](__m256i value) {
    if(affine.invert)value=_mm256_sub_epi16(maximum,value);
    value=_mm256_mullo_epi16(_mm256_min_epu16(value,limit),factor);
    value=affine.subtract?_mm256_subs_epu16(value,offset):_mm256_adds_epu16(value,offset);
    return _mm256_srli_epi16(value,8);
  };
  auto c=begin;
  for(;end-c>=32;c+=32) {
    const auto bytes=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source+c));
    const auto low=transform(_mm256_unpacklo_epi8(bytes,zero));
    const auto high=transform(_mm256_unpackhi_epi8(bytes,zero));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(output+c),_mm256_packus_epi16(low,high));
  }
  for(;end-c>=16;c+=16) {
    const auto value=transform(_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(source+c))));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(output+c),_mm_packus_epi16(
        _mm256_castsi256_si128(value),_mm256_extracti128_si256(value,1)));
  }
  return c;
}

std::size_t lookup_u8_row_avx2(const std::uint8_t* source, std::uint8_t* output,
                              std::size_t begin, std::size_t end, const std::int32_t* table) {
  auto c = begin;
  const auto gather = [&](std::size_t position) {
    const auto index = _mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source + position)));
    return _mm256_i32gather_epi32(table, index, 4);
  };
  const auto order = _mm256_setr_epi32(0, 4, 1, 5, 2, 6, 3, 7);
  // 合并四组精确查表与打包，重排 AVX2 两个 128 位 lane 的字节顺序。
  for (; end - c >= 32; c += 32) {
    const auto a = gather(c), b = gather(c + 8), d = gather(c + 16), e = gather(c + 24);
    const auto bytes = _mm256_packus_epi16(_mm256_packus_epi32(a, b), _mm256_packus_epi32(d, e));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(output + c), _mm256_permutevar8x32_epi32(bytes, order));
  }
  for (; end - c >= 8; c += 8) {
    const auto index = _mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(source + c)));
    const auto values = _mm256_i32gather_epi32(table, index, 4);
    const auto words = _mm_packus_epi32(_mm256_castsi256_si128(values), _mm256_extracti128_si256(values, 1));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(output + c), _mm_packus_epi16(words, words));
  }
  return c;
}

std::size_t multiply_u8_row_avx2(const std::uint8_t* a,const std::uint8_t* b,std::uint8_t* out,
                               std::size_t begin,std::size_t end,double mult,double add) {
  const auto factor=_mm256_set1_pd(mult),offset=_mm256_set1_pd(add);
  const auto zero=_mm256_setzero_pd(),maximum=_mm256_set1_pd(255.);
  const auto half=_mm256_set1_pd(.5);
  const double divisor_value=mult>0.?std::round(1./mult):0.;
  if(add==0. && divisor_value>=2. && divisor_value<=255. && 1./divisor_value==mult) {
    const int divisor=static_cast<int>(divisor_value);
    // FMA 精确保留倒数误差的符号。系数不小于数学倒数时，半值乘积
    // 不可能舍入到半值下方；只为向下舍入的倒数保留逐 lane 回退。
    const bool check_ties = (divisor & 1) == 0 && std::fma(mult, double(divisor), -1.) < 0.;
    const auto d=_mm256_set1_epi16(static_cast<short>(divisor));
    const auto reciprocal=_mm256_set1_epi16(static_cast<short>((65536+divisor-1)/divisor));
    const auto bias=_mm256_set1_epi16(static_cast<short>(divisor/2));
    auto c=begin;
    for(;c+16<=end;c+=16) {
      const auto x=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+c)));
      const auto y=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+c)));
      const auto numerator=_mm256_add_epi16(_mm256_mullo_epi16(x,y),bias);
      auto quotient=_mm256_mulhi_epu16(numerator,reciprocal);
      const auto product=_mm256_mullo_epi16(quotient,d);
      // 向上倒数估计精确或仅大一；byte 除数下，分子及估计商*除数均在 UInt16 内。
      const auto valid=_mm256_cmpeq_epi16(_mm256_min_epu16(product,numerator),product);
      quotient=_mm256_add_epi16(quotient,_mm256_xor_si256(valid,_mm256_set1_epi16(-1)));
      const auto bytes=_mm_packus_epi16(_mm256_castsi256_si128(quotient),_mm256_extracti128_si256(quotient,1));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),bytes);
      if(check_ties) {
        const unsigned ties=static_cast<unsigned>(_mm256_movemask_epi8(_mm256_cmpeq_epi16(
            numerator,_mm256_mullo_epi16(quotient,d))));
        if(ties)for(int lane=0;lane<16;++lane)if(ties&(1U<<(2*lane))) {
          const double exact=double(a[c+lane]*b[c+lane])*mult;
          const int integral=static_cast<int>(exact);
          out[c+lane]=static_cast<std::uint8_t>(std::min(255,integral+(exact-integral>=.5)));
        }
      }
    }
    return c;
  }
  const auto milli_mult=std::round(mult*1000.),milli_add=std::round(add*1000.);
  if(std::abs(milli_mult)*65025.+std::abs(milli_add)<=2147483000. &&
      milli_mult/1000.==mult && milli_add/1000.==add) {
    const auto m=_mm256_set1_epi32(static_cast<int>(milli_mult));
    const auto a0=_mm256_set1_epi32(static_cast<int>(milli_add));
    auto c=begin;
    for(;c+8<=end;c+=8) {
      const auto x=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(a+c)));
      const auto y=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(b+c)));
      auto numerator=_mm256_add_epi32(_mm256_mullo_epi32(_mm256_mullo_epi32(x,y),m),a0);
      numerator=_mm256_min_epi32(_mm256_max_epi32(numerator,_mm256_setzero_si256()),_mm256_set1_epi32(255000));
      numerator=_mm256_add_epi32(numerator,_mm256_set1_epi32(500));
      const auto rounded=_mm256_cvttps_epi32(_mm256_mul_ps(_mm256_cvtepi32_ps(numerator),_mm256_set1_ps(.001f)));
      const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
      // 精确千分系数与原序 Double 计算只在有理半整数处不同，保留原同值规则。
      const int ties=_mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(
          numerator,_mm256_mullo_epi32(rounded,_mm256_set1_epi32(1000)))));
      if(ties)for(int lane=0;lane<8;++lane)if(ties&(1<<lane)) {
        const auto exact=std::clamp(double(a[c+lane]*b[c+lane])*mult+add,0.,255.);
        const int integral=static_cast<int>(exact);
        out[c+lane]=static_cast<std::uint8_t>(integral+(exact-integral>=.5));
      }
    }
    return c;
  }
  if(std::abs(mult)*65025.+std::abs(add)<=65536.) {
    const auto fm=_mm256_set1_ps(static_cast<float>(mult));
    const auto fa=_mm256_set1_ps(static_cast<float>(add));
    const auto error=_mm256_set1_ps(static_cast<float>(
        3.*std::numeric_limits<float>::epsilon()*(std::abs(mult)*65025.+std::abs(add))+1e-30));
    auto c=begin;
    for(;c+8<=end;c+=8) {
      const auto x=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(a+c)));
      const auto y=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(b+c)));
      auto value=_mm256_add_ps(_mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_mullo_epi32(x,y)),fm),fa);
      value=_mm256_min_ps(_mm256_max_ps(value,_mm256_setzero_ps()),_mm256_set1_ps(255.f));
      const auto whole=_mm256_cvttps_epi32(value);
      const auto fraction=_mm256_sub_ps(value,_mm256_cvtepi32_ps(whole));
      const auto rounded=_mm256_sub_epi32(whole,_mm256_castps_si256(
          _mm256_cmp_ps(fraction,_mm256_set1_ps(.5f),_CMP_GE_OQ)));
      const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
      // 上述幅度界内，系数转换及两次独立 float 运算总误差 <1/32；仅距半界这么近的 lane 重算原序 Double。
      const auto distance=_mm256_andnot_ps(_mm256_set1_ps(-0.f),_mm256_sub_ps(fraction,_mm256_set1_ps(.5f)));
      const int uncertain=_mm256_movemask_ps(_mm256_cmp_ps(distance,error,_CMP_LE_OQ));
      if(uncertain)for(int lane=0;lane<8;++lane)if(uncertain&(1<<lane)) {
        const auto exact=std::clamp(double(a[c+lane]*b[c+lane])*mult+add,0.,255.);
        const int integral=static_cast<int>(exact);
        out[c+lane]=static_cast<std::uint8_t>(integral+(exact-integral>=.5));
      }
    }
    return c;
  }
  const auto convert=[&](__m128i product) {
    auto value=_mm256_add_pd(_mm256_mul_pd(_mm256_cvtepi32_pd(product),factor),offset);
    value=_mm256_min_pd(_mm256_max_pd(value,zero),maximum);
    const auto whole=_mm256_cvttpd_epi32(value);
    const auto remainder=_mm256_sub_pd(value,_mm256_cvtepi32_pd(whole));
    const auto up=_mm256_and_pd(_mm256_cmp_pd(remainder,half,_CMP_GE_OQ),_mm256_set1_pd(1.));
    return _mm_add_epi32(whole,_mm256_cvttpd_epi32(up));
  };
  auto c=begin;
  for(;c+8<=end;c+=8) {
    const auto x=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(a+c)));
    const auto y=_mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(b+c)));
    const auto product=_mm256_mullo_epi32(x,y);
    const auto words=_mm_packus_epi32(convert(_mm256_castsi256_si128(product)),
                                      convert(_mm256_extracti128_si256(product,1)));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
  }
  return c;
}

std::size_t zoom_nearest_u8_row_avx2(const std::uint8_t* input,std::uint8_t* output,
                                   std::span<const ZoomNearestU8Block> blocks) {
    std::size_t c=0;
    for(const auto& block:blocks) {
        const auto* p=input+block.offset;
        const auto a=_mm_loadu_si128(reinterpret_cast<const __m128i*>(p));
        const auto b=_mm_loadu_si128(reinterpret_cast<const __m128i*>(p+16));
        const auto lo=_mm_loadu_si128(reinterpret_cast<const __m128i*>(block.low.data()));
        const auto hi=_mm_loadu_si128(reinterpret_cast<const __m128i*>(block.high.data()));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(output+c),
            _mm_or_si128(_mm_shuffle_epi8(a,lo),_mm_shuffle_epi8(b,hi)));
        c+=16;
    }
    return c;
}

std::size_t zoom_bilinear_u8_row_avx2(const std::uint8_t* top,const std::uint8_t* bottom,
    std::uint8_t* output,std::span<const std::uint32_t> columns,
    std::span<const double> fractions,std::size_t width,double dy) {
  const auto one=_mm256_set1_pd(1.),half=_mm256_set1_pd(.5);
  const auto ey=_mm256_set1_pd(1.-dy),fy=_mm256_set1_pd(dy);
  std::size_t c=0;
  for(;columns.size()-c>=4;c+=4) {
    const auto* x=columns.data()+c;
    const auto x0=std::min(std::size_t(x[0])+1,width-1),x1=std::min(std::size_t(x[1])+1,width-1);
    const auto x2=std::min(std::size_t(x[2])+1,width-1),x3=std::min(std::size_t(x[3])+1,width-1);
    // 精确尺寸外部行的 byte 读取不越界，末行也如此；更宽整数 gather 会读过边缘。
    const auto gather=[&](const std::uint8_t* row,bool right) {
      return _mm256_cvtepi32_pd(right ? _mm_setr_epi32(row[x0],row[x1],row[x2],row[x3])
                                    : _mm_setr_epi32(row[x[0]],row[x[1]],row[x[2]],row[x[3]]));
    };
    const auto dx=_mm256_loadu_pd(fractions.data()+c),ex=_mm256_sub_pd(one,dx);
    const auto a=_mm256_add_pd(_mm256_mul_pd(ex,gather(top,false)),_mm256_mul_pd(dx,gather(top,true)));
    const auto b=_mm256_add_pd(_mm256_mul_pd(ex,gather(bottom,false)),_mm256_mul_pd(dx,gather(bottom,true)));
    const auto value=_mm256_add_pd(_mm256_mul_pd(ey,a),_mm256_mul_pd(fy,b));
    const auto whole=_mm256_cvttpd_epi32(value);
    const auto fraction=_mm256_sub_pd(value,_mm256_cvtepi32_pd(whole));
    const auto up=_mm256_cvttpd_epi32(_mm256_and_pd(_mm256_cmp_pd(fraction,half,_CMP_GE_OQ),one));
    // 比较余数而非加 .5，后者会将略低于半整数的值误舍入到同值。
    const auto words=_mm_packus_epi32(_mm_add_epi32(whole,up),_mm_setzero_si128());
    const auto bytes=_mm_packus_epi16(words,words);
    const auto packed=static_cast<std::uint32_t>(_mm_cvtsi128_si32(bytes));
    std::memcpy(output+c,&packed,sizeof(packed));
  }
  return c;
}
void dynamic_u8_masks_avx2(const std::uint8_t* a,const std::uint8_t* b,std::size_t begin,
                          std::size_t blocks,int low,int high,bool inverted,std::uint32_t* out) {
  if(blocks==0)return;
  if(low>high || low>255 || high<-255) {
    std::fill_n(out,blocks,inverted?UINT32_MAX:0);return;
  }
  if(low<=-255 && high>=255) {
    std::fill_n(out,blocks,inverted?0:UINT32_MAX);return;
  }
  auto emit=[&](auto predicate) {
    for(std::size_t i=0;i<blocks;++i) {
      const auto x=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(a+begin+i*32));
      const auto y=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(b+begin+i*32));
      auto mask=static_cast<std::uint32_t>(_mm256_movemask_epi8(predicate(x,y)));
      out[i]=inverted?~mask:mask;
    }
  };
  if(low>-255 && high<255) {
    // Equal/NotEqual 生成的双侧区间对称。
    const auto limit=_mm256_set1_epi8(static_cast<char>(high));
    emit([&](__m256i x,__m256i y) {
      const auto difference=_mm256_or_si256(_mm256_subs_epu8(x,y),_mm256_subs_epu8(y,x));
      return _mm256_cmpeq_epi8(_mm256_min_epu8(difference,limit),difference);
    });
    return;
  }
  if(low<=-255) { std::swap(a,b);low=-high; }
  if(low==0)emit([](__m256i x,__m256i y) { return _mm256_cmpeq_epi8(_mm256_max_epu8(x,y),x); });
  else if(low>0) {
    const auto limit=_mm256_set1_epi8(static_cast<char>(low));
    emit([&](__m256i x,__m256i y) {
      return _mm256_cmpeq_epi8(_mm256_min_epu8(_mm256_subs_epu8(x,y),limit),limit);
    });
  } else {
    const auto limit=_mm256_set1_epi8(static_cast<char>(-low));
    emit([&](__m256i x,__m256i y) {
      const auto difference=_mm256_subs_epu8(y,x);
      return _mm256_cmpeq_epi8(_mm256_min_epu8(difference,limit),difference);
    });
  }
}

std::size_t rgb_u8_row_avx2(const std::uint8_t* r,const std::uint8_t* g,const std::uint8_t* b,
                          std::uint8_t* out,std::size_t begin,std::size_t end) {
  auto load=[](const std::uint8_t* p) {
    return _mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(p)));
  };
  auto c=begin;
  for(;c+8<=end;c+=8) {
    const auto sum=_mm256_add_epi32(_mm256_add_epi32(
        _mm256_mullo_epi32(load(r+c),_mm256_set1_epi32(299)),
        _mm256_mullo_epi32(load(g+c),_mm256_set1_epi32(587))),
        _mm256_mullo_epi32(load(b+c),_mm256_set1_epi32(114)));
    const auto numerator=_mm256_add_epi32(sum,_mm256_set1_epi32(500));
    // [0,255500] 内 float 除以 1000 的误差 <0.001 且倒数误差为正，截断得到精确商。
    const auto rounded=_mm256_cvttps_epi32(_mm256_mul_ps(
        _mm256_cvtepi32_ps(numerator),_mm256_set1_ps(.001f)));
    const auto words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),
                                     _mm256_extracti128_si256(rounded,1));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
    // 只有精确有理半整数可能区别于原序 Double；重算这些 lane 并保留全部原同值规则。
    const int ties=_mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(
        numerator,_mm256_mullo_epi32(rounded,_mm256_set1_epi32(1000)))));
    if(ties) for(int lane=0;lane<8;++lane) if(ties&(1<<lane))
      out[c+lane]=static_cast<std::uint8_t>(std::llround(
          .299*r[c+lane]+.587*g[c+lane]+.114*b[c+lane]));
  }
  return c;
}

// 调用方保留原守卫及 runtime 选择，并在原 worker 执行；Float 转换也留在该 worker。
std::size_t divide_u16_row_avx2(const std::uint16_t* a, const std::uint16_t* b,
                           std::uint16_t* out, std::size_t begin, std::size_t end,
                           double mult, double add) {
  const auto scale = _mm256_set1_ps(static_cast<float>(mult));
  const auto bias = _mm256_set1_ps(static_cast<float>(add + .5));
  const auto zero = _mm256_setzero_ps();
  const auto one = _mm256_set1_ps(1);
  const auto two = _mm256_set1_ps(2);
  const auto upper = _mm256_set1_ps(65535);
  const auto offset = _mm256_set1_epi32(32768);
  auto c = begin;
  // 保留原八像素前缀及标量尾部；直接换成十六像素边界会改变哪些像素采用 Float。
  for (; end - c >= 8; c += 8) {
    const auto xi = _mm256_cvtepu16_epi32(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(a + c)));
    const auto yi = _mm256_cvtepu16_epi32(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(b + c)));
    const auto x = _mm256_cvtepi32_ps(xi);
    const auto y = _mm256_cvtepi32_ps(yi);
    const auto is_zero = _mm256_cmp_ps(y, zero, _CMP_EQ_OQ);
    const auto safe = _mm256_or_ps(_mm256_andnot_ps(is_zero, y),
                                  _mm256_and_ps(is_zero, one));
    const auto initial = _mm256_rcp_ps(safe);
    const auto reciprocal = _mm256_mul_ps(
        initial, _mm256_sub_ps(two, _mm256_mul_ps(safe, initial)));
    auto value = _mm256_add_ps(_mm256_mul_ps(_mm256_mul_ps(x, reciprocal), scale), bias);
    value = _mm256_andnot_ps(is_zero, _mm256_min_ps(upper, _mm256_max_ps(zero, value)));
    const auto shifted = _mm256_sub_epi32(_mm256_cvttps_epi32(value), offset);
    const auto words = _mm256_packs_epi32(shifted, _mm256_setzero_si256());
    const auto ordered = _mm_unpacklo_epi64(_mm256_castsi256_si128(words),
                                           _mm256_extracti128_si256(words, 1));
    const auto packed = _mm_xor_si128(ordered, _mm_set1_epi16(-32768));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(out + c), packed);
  }
  return c;
}

}
