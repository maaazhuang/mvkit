#include <imageproc/operators.hpp>
#include <imageproc/advanced.hpp>
#include "filters_avx2.hpp"
#include <runtime/cpu/cpu_features.hpp>

#include <algorithm>
#include <bit>
#include <array>
#include <cmath>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <vector>
#include <type_traits>
#include <unordered_map>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_MEDIAN_SSE2 1
#else
#define MVKIT_MEDIAN_SSE2 0
#endif

#include <core/exception.hpp>
#include <imageproc/border.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/pixel_traits.hpp>

namespace mv::imageproc {
namespace {

std::size_t normalize_mask(std::size_t value, const char* where) {
  if (value == 0) throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "mask size must be positive");
  return (value % 2 == 0) ? value + 1 : value;
}

template <typename T>
void separable_filter_channel(const mv::core::Image& image, mv::core::Image& output,
                              std::size_t channel,
                              const std::vector<double>& kernel_x,
                              const std::vector<double>& kernel_y) {
#if MVKIT_FILTER_HAS_AVX2
  if(mv::runtime::cpu_features().selection().avx2 &&
      (std::is_same_v<T,std::uint8_t> || std::is_same_v<T,std::uint16_t> || std::is_same_v<T,float>)) {
    const bool box5=kernel_x.size()==5 && kernel_y.size()==5 &&
      std::all_of(kernel_x.begin(),kernel_x.end(),[](double x){return x==.2;}) &&
      std::all_of(kernel_y.begin(),kernel_y.end(),[](double x){return x==.2;});
    separable_filter_avx2(image,output,channel,kernel_x,kernel_y,box5);return;
  }
#endif
  const ImageView src{image, channel};
  MutableImageView dst{output, channel};
  ExecutionContext context;
  std::pmr::vector<double> temp{context.scratch_resource()};
  const auto rx = static_cast<std::int64_t>(kernel_x.size() / 2);
  const auto ry = static_cast<std::int64_t>(kernel_y.size() / 2);
  const auto domain = image.domain();
  const auto box = domain.bounding_box();
  if (!box) return;
  const auto last_row = static_cast<std::int64_t>(image.height()) - 1;
  const auto last_col = static_cast<std::int64_t>(image.width()) - 1;
  const auto row0 = std::max<std::int64_t>(0, box->row_min - ry);
  const auto row1 = std::min(last_row, box->row_max + ry);
  const auto col0 = std::max<std::int64_t>(0, box->col_min - rx);
  const auto col1 = std::min(last_col, box->col_max + rx);
  const auto row_begin = static_cast<std::size_t>(row0);
  const auto row_count = static_cast<std::size_t>(row1 - row0 + 1);
  const auto col_begin = static_cast<std::size_t>(col0);
  const auto col_last = static_cast<std::size_t>(col1);
  const auto col_span = col_last - col_begin + 1;
  temp.resize(row_count * col_span);

  const auto horizontal_work = ExecutionContext::saturated_work(
      ExecutionContext::saturated_work(col_last - col_begin + 1, row_count),
      kernel_x.size());
  ExecutionContext::for_each_partition(
      row_count, horizontal_work, ExecutionContext::kFilterParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto row = row_begin + index;
          const auto* in = src.row_as<T>(row);
          auto* target = temp.data() + (row - row_begin) * col_span;
          std::fill(target, target + col_span, 0.0);
          // 连续内部跨度向量化时保持逐像素 tap 顺序，仅窄边界查询镜像索引。
          for (std::size_t k = 0; k < kernel_x.size(); ++k) {
            const auto offset = static_cast<std::int64_t>(k) - rx;
            const auto interior_begin = std::max(col0, -offset);
            const auto interior_end = std::min(col1 + 1,
                static_cast<std::int64_t>(image.width()) - offset);
            const auto add_border = [&](std::int64_t col) {
              const auto x = border_index(col + offset, image.width(), BorderMode::Mirrored).value();
              T sample;
              std::memcpy(&sample, reinterpret_cast<const std::byte*>(in) + x * sizeof(T), sizeof(T));
              target[static_cast<std::size_t>(col - col0)] += static_cast<double>(sample) * kernel_x[k];
            };
            const auto left_end = std::min(col1 + 1, interior_begin);
            for (auto col = col0; col < left_end; ++col) add_border(col);
            for (auto col = std::max(col0, interior_begin); col < interior_end; ++col) {
              T sample;
              std::memcpy(&sample, reinterpret_cast<const std::byte*>(in) + (col + offset) * sizeof(T), sizeof(T));
              target[static_cast<std::size_t>(col - col0)] += static_cast<double>(sample) * kernel_x[k];
            }
            for (auto col = std::max(left_end, interior_end); col <= col1; ++col) add_border(col);
          }
        }
      });

  const auto runs = domain.runs();
  const auto vertical_work = ExecutionContext::saturated_work(
      ExecutionContext::saturated_work(image.width(), image.height()),
      kernel_y.size());
  ExecutionContext::for_each_partition(
      runs.size(), vertical_work, ExecutionContext::kFilterParallelWork,
      [&](mv::runtime::IndexRange range) {
        ExecutionContext local;
        std::pmr::vector<double> sums(col_span, local.scratch_resource());
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          const auto row = static_cast<std::size_t>(run.row);
          const auto begin = static_cast<std::size_t>(run.col_begin);
          const auto end = static_cast<std::size_t>(run.col_end) + 1;
          auto* out = dst.row_as<T>(row);
          std::fill(sums.begin(), sums.begin() + (end - begin), 0.0);
          for (std::size_t k = 0; k < kernel_y.size(); ++k) {
            const auto y = border_index(static_cast<std::int64_t>(row) +
                static_cast<std::int64_t>(k) - ry, image.height(), BorderMode::Mirrored).value();
            const auto* source = temp.data() + (y - row_begin) * col_span + begin - col_begin;
            for (std::size_t i = 0; i < end - begin; ++i) sums[i] += source[i] * kernel_y[k];
          }
          for (std::size_t col = begin; col < end; ++col) out[col] = saturate_cast<T>(sums[col - begin]);
        }
      });
}

std::vector<double> box_kernel(std::size_t size) {
  return std::vector<double>(size, 1.0 / static_cast<double>(size));
}

std::vector<double> gaussian_kernel(std::size_t size) {
  // HALCON 24.11 real 脉冲响应，中心行归一化为单位质量。Size=11 外 tap
  // 比 sigma=(size-1)/4 的采样 Gaussian 更重，保留实测掩码。
  switch (size) {
    case 3: return {0.20232838, 0.59534324, 0.20232838};
    case 5: return {0.08145534, 0.23946876, 0.35815180, 0.23946876, 0.08145534};
    case 7: return {0.05338277, 0.11320386, 0.20691979, 0.25298716,
                    0.20691979, 0.11320386, 0.05338277};
    case 9: return {0.04195871, 0.06653697, 0.12092965, 0.17306223,
                    0.19502488, 0.17306223, 0.12092965, 0.06653697, 0.04195871};
    case 11: return {0.03593032, 0.04482634, 0.07789859, 0.11559786,
                     0.14648717, 0.15851944, 0.14648717, 0.11559786,
                     0.07789859, 0.04482634, 0.03593032};
    default: throw mv::core::Exception(mv::core::Status::Unsupported, "gauss_filter", "supported sizes are 3,5,7,9,11");
  }
}

template<typename T>
void integer_box_channel(const mv::core::Image& image,mv::core::Image& output,
                         std::size_t channel,std::size_t kw,std::size_t kh) {
  const ImageView src{image,channel};MutableImageView dst{output,channel};
  const auto width=image.width(),height=image.height();
  const auto rx=static_cast<std::int64_t>(kw/2),ry=static_cast<std::int64_t>(kh/2);
  const double inverse=1./double(kw*kh);
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::cpu_features().selection().avx2 &&
      kw*kh<=std::size_t(std::numeric_limits<std::int32_t>::max())/std::numeric_limits<T>::max();
#endif
  const auto read=[](const T* row,std::size_t c) {T v;std::memcpy(&v,reinterpret_cast<const std::byte*>(row)+c*sizeof(T),sizeof(T));return v;};
  ExecutionContext::for_each_partition(height,width*height,ExecutionContext::kFilterParallelWork,[&](auto range) {
    ExecutionContext context;
    std::pmr::vector<std::uint32_t> columns(width,0,context.scratch_resource());
    std::pmr::vector<std::size_t> leaving(width,context.scratch_resource()),entering(width,context.scratch_resource());
    for(std::size_t c=0;c<width;++c) {
      leaving[c]=border_index(static_cast<std::int64_t>(c)-rx,width,BorderMode::Mirrored).value();
      entering[c]=border_index(static_cast<std::int64_t>(c)+rx+1,width,BorderMode::Mirrored).value();
    }
    for(auto dy=-ry;dy<=ry;++dy) {
      const auto* row=src.row_as<T>(border_index(static_cast<std::int64_t>(range.begin)+dy,height,BorderMode::Mirrored).value());
#if MVKIT_FILTER_HAS_AVX2
      if(avx2)box_columns_avx2(columns.data(),nullptr,row,width,image.pixel_type(channel));
      else
#endif
      for(std::size_t c=0;c<width;++c)columns[c]+=read(row,c);
    }
    for(auto r=range.begin;r<range.end;++r) {
      std::uint64_t sum=0;
      for(auto dx=-rx;dx<=rx;++dx)sum+=columns[border_index(dx,width,BorderMode::Mirrored).value()];
      auto* out=dst.row_as<T>(r);
#if MVKIT_FILTER_HAS_AVX2
      if(avx2)box_row_avx2(columns.data(),leaving.data(),entering.data(),out,width,kw/2,
          static_cast<std::uint32_t>(kw*kh),static_cast<std::uint32_t>(sum),image.pixel_type(channel));
      else
#endif
      for(std::size_t c=0;c<width;++c) {
        // 奇数整数窗口没有半值同值；掩码<=255 时两种 Double 累加序误差均远小于最近半界。
        out[c]=static_cast<T>(double(sum)*inverse+.5);
        sum=sum-columns[leaving[c]]+columns[entering[c]];
      }
      if(r+1<range.end) {
        const auto* before=src.row_as<T>(border_index(static_cast<std::int64_t>(r)-ry,height,BorderMode::Mirrored).value());
        const auto* after=src.row_as<T>(border_index(static_cast<std::int64_t>(r)+ry+1,height,BorderMode::Mirrored).value());
#if MVKIT_FILTER_HAS_AVX2
        if(avx2)box_columns_avx2(columns.data(),before,after,width,image.pixel_type(channel));
        else
#endif
        for(std::size_t c=0;c<width;++c)columns[c]=columns[c]-read(before,c)+read(after,c);
      }
    }
  });
}

template <typename T>
void mean_channel(const mv::core::Image& image, mv::core::Image& output,
                  std::size_t channel, std::size_t width, std::size_t height) {
  if constexpr(std::is_same_v<T,std::uint8_t> || std::is_same_v<T,std::uint16_t>) {
    if(!image.has_reduced_domain() && (width>5 || height>5) && width<=255 && height<=255) {
      integer_box_channel<T>(image,output,channel,width,height);return;
    }
  }
  separable_filter_channel<T>(image, output, channel, box_kernel(width), box_kernel(height));
}

template <typename T>
void gauss_channel(const mv::core::Image& image, mv::core::Image& output,
                   std::size_t channel, std::size_t size,bool canonical=false) {
  // HALCON 整数像素使用单位和的 10 位掩码，与 real 内核不同；系数由 uint2 脉冲测得。
  const auto kernel = [&] {
    if constexpr (std::is_integral_v<T>) {
      if(canonical||std::is_same_v<T,std::uint8_t>||std::is_same_v<T,std::uint16_t>) {
      std::vector<double> weights;
      switch (size) {
        case 3: weights = {207, 610, 207}; break;
        case 5: weights = {83, 246, 366, 246, 83}; break;
        case 7: weights = {55, 116, 211, 260, 211, 116, 55}; break;
        case 9: weights = {43, 68, 124, 177, 200, 177, 124, 68, 43}; break;
        case 11: weights = {37, 46, 80, 118, 150, 162, 150, 118, 80, 46, 37}; break;
      }
      for (auto& weight : weights) weight /= 1024.0;
      return weights;
      }
    }
    return gaussian_kernel(size);
  }();
  if constexpr(std::is_integral_v<T> && !std::is_same_v<T,std::uint8_t>) {
    if(canonical) {
    // HALCON 整数路径先滤列，两次可分离阶段后各取整；全精度中间值会令 uint2 相差一。
    ExecutionContext context;
    const ImageView source{image,channel};MutableImageView destination{output,channel};
    const auto width=image.width(),height=image.height();
    const auto radius=static_cast<std::int64_t>(size/2);
    std::pmr::vector<T> temporary(width*height,context.scratch_resource());
    for(std::size_t r=0;r<height;++r)for(std::size_t c=0;c<width;++c) {
      double sum=0;
      for(std::size_t k=0;k<size;++k) {
        const auto row=border_index(static_cast<std::int64_t>(r)+static_cast<std::int64_t>(k)-radius,height,BorderMode::Mirrored).value();
        T value;std::memcpy(&value,reinterpret_cast<const std::byte*>(source.row_as<T>(row))+c*sizeof(T),sizeof(T));
        sum+=static_cast<double>(value)*kernel[k];
      }
      temporary[r*width+c]=saturate_cast<T>(std::floor(sum+.5));
    }
    const auto domain=image.domain();
    for(const auto& run:domain.runs()) {
      auto* row=destination.row_as<T>(static_cast<std::size_t>(run.row));
      for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c) {
        double sum=0;
        for(std::size_t k=0;k<size;++k) {
          const auto column=border_index(c+static_cast<std::int64_t>(k)-radius,width,BorderMode::Mirrored).value();
          sum+=static_cast<double>(temporary[static_cast<std::size_t>(run.row)*width+column])*kernel[k];
        }
        row[c]=saturate_cast<T>(std::floor(sum+.5));
      }
    }
    return;
    }
  }
  separable_filter_channel<T>(image, output, channel, kernel, kernel);
}

template<class V, class CompareSwap>
V median9(std::array<V, 9> p, CompareSwap cs) {
  cs(p[1],p[2]); cs(p[4],p[5]); cs(p[7],p[8]);
  cs(p[0],p[1]); cs(p[3],p[4]); cs(p[6],p[7]);
  cs(p[1],p[2]); cs(p[4],p[5]); cs(p[7],p[8]);
  cs(p[0],p[3]); cs(p[5],p[8]); cs(p[4],p[7]);
  cs(p[3],p[6]); cs(p[1],p[4]); cs(p[2],p[5]);
  cs(p[4],p[7]); cs(p[4],p[2]); cs(p[6],p[4]); cs(p[4],p[2]);
  return p[4];
}

template<class T>
void median3_integer(const mv::core::Image& image, mv::core::Image& output,
                     std::size_t channel, BorderMode border, double constant_gray) {
  const auto* input=static_cast<const std::byte*>(image.data(channel));
  auto* destination=static_cast<std::byte*>(output.mutable_data(channel));
  const auto stride=image.stride(channel),out_stride=output.stride(channel),height=image.height();
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::cpu_features().selection().avx2;
  const auto type=image.pixel_type(channel);
#endif
  const auto width=image.width(); const auto domain=image.domain();
#if MVKIT_MEDIAN_SSE2
  const auto& cpu=mv::runtime::cpu_features();
  const bool sse2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{true,false,false,false});
#endif
  for(const auto& run:domain.runs()) {
    const auto row=static_cast<std::size_t>(run.row);
    const T* rows[3]{};
    for(int k=0;k<3;++k) {const auto y=border_index(static_cast<std::int64_t>(row)+k-1,height,border);if(y)rows[k]=reinterpret_cast<const T*>(input+*y*stride);}
    auto* out=reinterpret_cast<T*>(destination+row*out_stride);auto c=static_cast<std::size_t>(run.col_begin);const auto end=static_cast<std::size_t>(run.col_end)+1;
    for(;c<end;) {
#if MVKIT_FILTER_HAS_AVX2
      if(rows[0] && rows[1] && rows[2] && c>=1 && avx2) {
        c+=median3_row_avx2(rows[0],rows[1],rows[2],out,c,std::min(end,width-1),type);
        if(c==end)break;
      }
#endif
#if MVKIT_MEDIAN_SSE2
      constexpr std::size_t lanes=16/sizeof(T);
      if(sse2 && rows[0] && rows[1] && rows[2] && c>=1 && c+lanes<=end && c+lanes<width) {
        std::array<__m128i,9> p;
        for(int r=0;r<3;++r)for(int k=0;k<3;++k)p[r*3+k]=_mm_loadu_si128(reinterpret_cast<const __m128i*>(rows[r]+c+k-1));
        const auto result=median9(p,[](__m128i& a,__m128i& b){
          if constexpr(sizeof(T)==1){const auto lo=_mm_min_epu8(a,b);b=_mm_max_epu8(a,b);a=lo;}
          else {const auto difference=_mm_subs_epu16(a,b);a=_mm_sub_epi16(a,difference);b=_mm_add_epi16(b,difference);}
        });
        _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),result);c+=lanes;continue;
      }
#endif
      // 边界常量在选择前保持 Double，与通用路径一致。
      std::array<double,9> p;
      for(int r=0;r<3;++r)for(int k=0;k<3;++k){const auto x=border_index(static_cast<std::int64_t>(c)+k-1,width,border);T v{};if(rows[r] && x)std::memcpy(&v,reinterpret_cast<const std::byte*>(rows[r])+*x*sizeof(T),sizeof(T));p[r*3+k]=rows[r]&&x?static_cast<double>(v):constant_gray;}
      out[c++]=saturate_cast<T>(median9(p,[](double& a,double& b){if(a>b)std::swap(a,b);}));
    }
  }
}

void median3_float(const mv::core::Image& image, mv::core::Image& output,
                   std::size_t channel, BorderMode border, double constant_gray) {
  const auto* input = static_cast<const std::byte*>(image.data(channel));
  auto* destination = static_cast<std::byte*>(output.mutable_data(channel));
  const auto stride = image.stride(channel), out_stride = output.stride(channel);
  const auto width = image.width(), height = image.height();
  const auto domain = image.domain();
  // nth_element 的同值/NaN 选择可观测；仅在同值 Float 位表示一致时使用网络。
  // halo 会读取缩减域外，须扫描完整通道。
#if MVKIT_MEDIAN_SSE2
  const auto& cpu=mv::runtime::cpu_features();
  const bool sse2_enabled=mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),
      {true,false,false,false});
  // 转换时 DAZ 可将负 subnormal 变为负零同值。
  const bool network_allowed=sse2_enabled && (_mm_getcsr()&0x40u)==0;
#else
  const bool network_allowed=false;
#endif
  bool ordinary = false;
  if constexpr (sizeof(float)==4 && std::numeric_limits<float>::is_iec559 &&
                sizeof(double)==8 && std::numeric_limits<double>::is_iec559) {
    std::uint64_t constant_bits;
    std::memcpy(&constant_bits,&constant_gray,sizeof(constant_bits));
    ordinary=network_allowed && constant_bits!=0x8000000000000000ull;
    for (std::size_t r=0; ordinary && r<height; ++r) {
      for (std::size_t c=0; c<width; ++c) {
        std::uint32_t bits;
        std::memcpy(&bits,input+r*stride+c*sizeof(float),sizeof(bits));
        if ((bits&0x7fffffffu)>=0x7f800000u || bits==0x80000000u) {
          ordinary=false; break;
        }
      }
    }
  } else {
    ordinary=false;
  }
#if MVKIT_MEDIAN_SSE2
  const bool sse2=ordinary && sse2_enabled;
#endif
  for (const auto& run : domain.runs()) {
    const std::byte* rows[3]{};
    for (int k=0; k<3; ++k) {
      const auto y = border_index(std::int64_t(run.row)+k-1, height, border);
      if (y) rows[k] = input+*y*stride;
    }
    auto* out = reinterpret_cast<float*>(destination+std::int64_t(run.row)*out_stride);
    for (auto c=static_cast<std::size_t>(run.col_begin);
         c<=static_cast<std::size_t>(run.col_end); ++c) {
#if MVKIT_MEDIAN_SSE2
      if (sse2 && rows[0] && rows[1] && rows[2] && c>=1 &&
          c+1<=static_cast<std::size_t>(run.col_end) && c+2<width) {
        std::array<__m128d,9> p;
        for (int r=0; r<3; ++r) for (int k=0; k<3; ++k) {
          const auto values=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(
              rows[r]+(c+k-1)*sizeof(float)));
          p[r*3+k]=_mm_cvtps_pd(_mm_castsi128_ps(values));
        }
        const auto result=median9(p,[](__m128d& a,__m128d& b) {
          const auto lo=_mm_min_pd(a,b); b=_mm_max_pd(a,b); a=lo;
        });
        _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),
            _mm_castps_si128(_mm_cvtpd_ps(result)));
        ++c; continue;
      }
#endif
      std::array<double,9> p;
      std::array<std::optional<std::size_t>,3> xs;
      for (int k=0; k<3; ++k)
        xs[k]=border_index(static_cast<std::int64_t>(c)+k-1,width,border);
      for (int r=0; r<3; ++r) {
        for (int k=0; k<3; ++k) {
          const auto x=xs[k];
          float value{};
          if (rows[r] && x) std::memcpy(&value,rows[r]+*x*sizeof(float),sizeof(value));
          p[r*3+k] = rows[r] && x ? static_cast<double>(value) : constant_gray;
        }
      }
      if (ordinary) {
        out[c]=static_cast<float>(median9(p,[](double& a,double& b) {
          if (a>b) std::swap(a,b);
        }));
      } else {
        // 保留行优先 Float→Double 转换及原秩选择过程。
        std::nth_element(p.begin(),p.begin()+4,p.end());
        out[c]=static_cast<float>(p[4]);
      }
    }
  }
}

template <typename T>
void median_channel(const mv::core::Image& image, mv::core::Image& output,
                    std::size_t channel, MedianMask mask, std::size_t radius,
                    BorderMode border, double constant_gray) {
  if constexpr (std::is_same_v<T,float>) {
    if (mask==MedianMask::Square && radius==1) {
      median3_float(image,output,channel,border,constant_gray); return;
    }
  }
  if constexpr (std::is_same_v<T,std::uint8_t> || std::is_same_v<T,std::uint16_t>) {
    if(mask==MedianMask::Square && radius==1) {
      median3_integer<T>(image,output,channel,border,constant_gray);return;
    }
  }
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if (mask == MedianMask::Square && radius > 1 && radius <= 63) {
      const ImageView source{image, channel};
      MutableImageView destination{output, channel};
      ExecutionContext context;
      const auto diameter = radius * 2 + 1, width = image.width();
      const auto middle = diameter * diameter / 2;
      const auto constant = saturate_cast<T>(constant_gray);
      std::pmr::vector<std::optional<std::size_t>> columns(width + 2 * radius, context.scratch_resource());
      for (std::size_t i = 0; i < columns.size(); ++i)
        columns[i] = border_index(static_cast<std::int64_t>(i) - static_cast<std::int64_t>(radius), width, border);
      std::pmr::vector<const T*> rows(diameter, context.scratch_resource());
      const auto active_domain = image.domain();
      for (const auto& run : active_domain.runs()) {
        for (std::size_t dy = 0; dy < diameter; ++dy) {
          const auto y = border_index(std::int64_t(run.row) + static_cast<std::int64_t>(dy) - static_cast<std::int64_t>(radius), image.height(), border);
          rows[dy] = y ? source.row_as<T>(*y) : nullptr;
        }
        const bool mapped_rows = std::all_of(rows.begin(), rows.end(),
                                             [](const auto* row) { return row != nullptr; });
        std::array<unsigned, 256> fine{};
        std::array<unsigned, 16> coarse{};
        const auto value = [&](std::size_t dy, std::size_t x) {
          return rows[dy] && columns[x] ? rows[dy][*columns[x]] : constant;
        };
        for (std::size_t dy = 0; dy < diameter; ++dy)
          for (std::size_t dx = 0; dx < diameter; ++dx) {
            const auto v = value(dy, static_cast<std::size_t>(run.col_begin) + dx);
            ++fine[v]; ++coarse[v >> 4];
          }
        auto* out = destination.row_as<T>(static_cast<std::size_t>(run.row));
        for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
          unsigned below = 0, bin = 0;
          while (below + coarse[bin] <= middle) below += coarse[bin++];
          unsigned v = bin << 4;
          while (below + fine[v] <= middle) below += fine[v++];
          out[c] = static_cast<T>(v);
          if (c == run.col_end) break;
          if (mapped_rows && c >= static_cast<std::int64_t>(radius) &&
              static_cast<std::size_t>(c) + radius + 1 < width) {
            const auto left = static_cast<std::size_t>(c) - radius;
            const auto right = static_cast<std::size_t>(c) + radius + 1;
            for (const auto* row : rows) {
              const auto old = row[left], next = row[right];
              if (old != next) { --fine[old]; --coarse[old >> 4]; ++fine[next]; ++coarse[next >> 4]; }
            }
          } else for (std::size_t dy = 0; dy < diameter; ++dy) {
            const auto old = value(dy, static_cast<std::size_t>(c));
            const auto next = value(dy, static_cast<std::size_t>(c) + diameter);
            if (old != next) { --fine[old]; --coarse[old >> 4]; ++fine[next]; ++coarse[next >> 4]; }
          }
        }
      }
      return;
    }
  }
  if constexpr (std::is_same_v<T, std::uint16_t> || std::is_same_v<T, float>) {
    // 精确秩编码将有界取值集合转为 byte 滑动直方图，不量化，解码取回原值。
    // 发现 257 个值即停止，保留通用原类型路径。
    if (!image.has_reduced_domain() && mask == MedianMask::Square &&
        radius > 1 && radius <= 63 &&
        !(border==BorderMode::Constant && constant_gray==0 && std::signbit(constant_gray)) &&
        (border != BorderMode::Constant ||
         static_cast<double>(saturate_cast<T>(constant_gray)) == constant_gray)) {
      ExecutionContext context;
      std::pmr::unordered_map<T,std::uint8_t> ranks{context.scratch_resource()};
      ranks.reserve(256);
      bool bounded=true;
      if (border==BorderMode::Constant) ranks.emplace(saturate_cast<T>(constant_gray),0);
      const auto width=image.width(),height=image.height();
      const auto* input=static_cast<const std::byte*>(image.data(channel));
      for (std::size_t r=0;r<height && bounded;++r) {
        const auto* row=input+r*image.stride(channel);
        for (std::size_t c=0;c<width;++c) {
          T value;std::memcpy(&value,row+c*sizeof(T),sizeof(T));
          if constexpr (std::is_same_v<T,float>) {
            if (!std::isfinite(value) || (value==0 && std::signbit(value))) {bounded=false;break;}
          }
          ranks.emplace(value,0);
          if(ranks.size()>256){bounded=false;break;}
        }
      }
      if (bounded) {
        std::array<T,256> palette{};
        std::size_t size=0;
        for(const auto& item:ranks)palette[size++]=item.first;
        std::sort(palette.begin(),palette.begin()+size);
        for(std::size_t i=0;i<size;++i)ranks.at(palette[i])=static_cast<std::uint8_t>(i);
        const std::array types{mv::core::PixelType::UInt8};
        auto encoded=mv::core::Image::create_uninitialized(width,height,types);
        auto selected=mv::core::Image::create_uninitialized(width,height,types);
        auto* bytes=static_cast<std::uint8_t*>(encoded.mutable_data());
        for(std::size_t r=0;r<height;++r)for(std::size_t c=0;c<width;++c) {
          T value;std::memcpy(&value,input+r*image.stride(channel)+c*sizeof(T),sizeof(T));
          bytes[r*encoded.stride()+c]=ranks.at(value);
        }
        const double encoded_constant=border==BorderMode::Constant ? ranks.at(saturate_cast<T>(constant_gray)) : 0;
        median_channel<std::uint8_t>(encoded,selected,0,mask,radius,border,encoded_constant);
        const auto* indices=static_cast<const std::uint8_t*>(selected.data());
        auto* destination=static_cast<std::byte*>(output.mutable_data(channel));
        for(std::size_t r=0;r<height;++r)for(std::size_t c=0;c<width;++c) {
          const T value=palette[indices[r*selected.stride()+c]];
          std::memcpy(destination+r*output.stride(channel)+c*sizeof(T),&value,sizeof(T));
        }
        return;
      }
    }
    // 一次解析边界并按原标量类型选择，避免逐 tap 重复坐标映射及视图访问；
    // 超出原类型的常量在选择前须保持 Double。
    if (mask == MedianMask::Square && radius > 1 && radius <= 63 &&
        (border != BorderMode::Constant ||
         static_cast<double>(saturate_cast<T>(constant_gray)) == constant_gray)) {
      ExecutionContext context;
      const auto width = image.width(), height = image.height();
      const auto diameter = radius * 2 + 1;
      const auto pitch = width + 2 * radius, rows = height + 2 * radius;
      std::pmr::vector<T> padded{context.scratch_resource()};
      if (rows > padded.max_size() / pitch)
        throw mv::core::Exception(mv::core::Status::OutOfRange, "median_image", "padded image size overflow");
      padded.resize(rows * pitch);
      const auto* input = static_cast<const std::byte*>(image.data(channel));
      const auto stride = image.stride(channel);
      const auto constant = saturate_cast<T>(constant_gray);
      for (std::size_t r = 0; r < rows; ++r) {
        auto* target = padded.data() + r * pitch;
        const auto y = border_index(static_cast<std::int64_t>(r) - static_cast<std::int64_t>(radius), height, border);
        if (!y) { std::fill_n(target, pitch, constant); continue; }
        const auto* source = input + *y * stride;
        std::memcpy(target + radius, source, width * sizeof(T));
        for (std::size_t x = 0; x < radius; ++x) {
          for (auto c : {x, width + radius + x}) {
            const auto mapped = border_index(static_cast<std::int64_t>(c) - static_cast<std::int64_t>(radius), width, border);
            if (mapped) std::memcpy(target + c, source + *mapped * sizeof(T), sizeof(T));
            else target[c] = constant;
          }
        }
      }
      std::pmr::vector<T> values(diameter * diameter, context.scratch_resource());
      auto* destination = static_cast<std::byte*>(output.mutable_data(channel));
      const auto destination_stride = output.stride(channel);
      const auto domain = image.domain();
      for (const auto& run : domain.runs()) {
        auto* out = reinterpret_cast<T*>(destination + std::int64_t(run.row) * destination_stride);
        for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
          for (std::size_t dy = 0; dy < diameter; ++dy)
            std::copy_n(padded.data() + (std::int64_t(run.row) + dy) * pitch + c,
                        diameter, values.data() + dy * diameter);
          auto middle = values.begin() + values.size() / 2;
          std::nth_element(values.begin(), middle, values.end());
          out[c] = *middle;
        }
      }
      return;
    }
  }
  const ImageView src{image, channel};
  MutableImageView dst{output, channel};
  ExecutionContext context;
  std::pmr::vector<double> values{context.scratch_resource()};
  const auto diameter = radius * 2 + 1;
  values.reserve(diameter * diameter);
  const auto rad = static_cast<std::int64_t>(radius);
  // 奇直径圆掩码包含与栅格边界相交的像素；单位半径为五像素十字。
  std::pmr::vector<std::int64_t> circle_widths{context.scratch_resource()};
  if(mask==MedianMask::Circle) {
    circle_widths.resize(radius+1);
    if(radius==1){circle_widths[0]=1;circle_widths[1]=0;}
    else for(std::int64_t dy=0;dy<=rad;++dy)
      circle_widths[static_cast<std::size_t>(dy)]=std::min(rad,static_cast<std::int64_t>(
          std::floor(std::sqrt((double(rad)+.5)*(double(rad)+.5)-double(dy)*double(dy)))));
  }

  const auto output_domain = image.domain();
  for (const auto& run : output_domain.runs()) {
    const auto row = static_cast<std::size_t>(run.row);
    auto* out = dst.row_as<T>(row);
    for (std::int64_t col_i = run.col_begin;; ++col_i) {
      const auto col = static_cast<std::size_t>(col_i);
      values.clear();
      for (std::int64_t dy = -rad; dy <= rad; ++dy) {
        const auto half=mask==MedianMask::Circle?circle_widths[static_cast<std::size_t>(std::abs(dy))]:rad;
        for (std::int64_t dx = -half; dx <= half; ++dx) {
          const auto y = border_index(static_cast<std::int64_t>(row) + dy, image.height(), border);
          const auto x = border_index(static_cast<std::int64_t>(col) + dx, image.width(), border);
          if (!x || !y) {
            values.push_back(constant_gray);
          } else {
            values.push_back(static_cast<double>(src.row_as<T>(*y)[*x]));
          }
        }
      }
      const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
      std::nth_element(values.begin(), mid, values.end());
      out[col] = saturate_cast<T>(*mid);
      if (col_i == run.col_end) break;
    }
  }
}

// 仅用于公开 Byte 入口；宽类型精确秩工作流在独立认证前保留原行直方图及内核。
bool median_byte_full_columns(const mv::core::Image& image, mv::core::Image& output,
    std::size_t channel, MedianMask mask, std::size_t radius,
    BorderMode border, double constant_gray) {
#if MVKIT_MEDIAN_SSE2
  if(image.has_reduced_domain() || mask!=MedianMask::Square || radius<=1 || radius>63)return false;
  const auto& cpu=mv::runtime::cpu_features();
  if(!mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{true,false,false,false}))return false;
  const ImageView source{image,channel};MutableImageView destination{output,channel};
  const auto width=image.width(),height=image.height(),diameter=2*radius+1;
  constexpr std::size_t pitch=272;
  ExecutionContext context;
  std::pmr::vector<std::uint8_t> columns{context.scratch_resource()};
  if(width>columns.max_size()/pitch)return false;
  columns.resize(width*pitch);
  const auto constant=saturate_cast<std::uint8_t>(constant_gray);
  std::array<std::uint8_t,pitch> constant_column{};
  constant_column[constant]=static_cast<std::uint8_t>(diameter);
  constant_column[256+(constant>>4)]=static_cast<std::uint8_t>(diameter);
  std::pmr::vector<const std::uint8_t*> mapped(width+2*radius,context.scratch_resource());
  for(std::size_t x=0;x<mapped.size();++x) {
    const auto c=border_index(static_cast<std::int64_t>(x)-static_cast<std::int64_t>(radius),width,border);
    mapped[x]=c ? columns.data()+*c*pitch : constant_column.data();
  }
  const auto row=[&](std::int64_t r) -> const std::uint8_t* {
    const auto y=border_index(r,height,border);return y ? source.row_as<std::uint8_t>(*y) : nullptr;
  };
  for(std::int64_t dy=-static_cast<std::int64_t>(radius);dy<=static_cast<std::int64_t>(radius);++dy) {
    const auto* values=row(dy);
    for(std::size_t x=0;x<width;++x) {
      const auto v=values ? values[x] : constant;
      auto* histogram=columns.data()+x*pitch;++histogram[v];++histogram[256+(v>>4)];
    }
  }
  const auto add16=[](std::uint16_t* target,const std::uint8_t* values) {
    const auto bytes=_mm_loadu_si128(reinterpret_cast<const __m128i*>(values));
    const auto zero=_mm_setzero_si128();
    _mm_storeu_si128(reinterpret_cast<__m128i*>(target),_mm_add_epi16(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(target)),_mm_unpacklo_epi8(bytes,zero)));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(target+8),_mm_add_epi16(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(target+8)),_mm_unpackhi_epi8(bytes,zero)));
  };
  const auto slide16=[](std::uint16_t* target,const std::uint8_t* old,const std::uint8_t* next) {
    const auto a=_mm_loadu_si128(reinterpret_cast<const __m128i*>(old));
    const auto b=_mm_loadu_si128(reinterpret_cast<const __m128i*>(next));
    const auto zero=_mm_setzero_si128();
    _mm_storeu_si128(reinterpret_cast<__m128i*>(target),_mm_add_epi16(_mm_sub_epi16(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(target)),_mm_unpacklo_epi8(a,zero)),_mm_unpacklo_epi8(b,zero)));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(target+8),_mm_add_epi16(_mm_sub_epi16(
        _mm_loadu_si128(reinterpret_cast<const __m128i*>(target+8)),_mm_unpackhi_epi8(a,zero)),_mm_unpackhi_epi8(b,zero)));
  };
  const auto middle=diameter*diameter/2;
  const auto select16=[](const std::uint16_t* values,std::size_t rank) {
    const auto prefix=[](__m128i v) {
      v=_mm_add_epi16(v,_mm_slli_si128(v,2));
      v=_mm_add_epi16(v,_mm_slli_si128(v,4));
      return _mm_add_epi16(v,_mm_slli_si128(v,8));
    };
    const auto low=prefix(_mm_loadu_si128(reinterpret_cast<const __m128i*>(values)));
    const auto target=_mm_set1_epi16(static_cast<std::int16_t>(rank));
    const auto mask=_mm_movemask_epi8(_mm_cmpgt_epi16(low,target));
    if(mask)return std::size_t(std::countr_zero(static_cast<unsigned>(mask))/2);
    const auto carry=_mm_shufflehi_epi16(low,0xff);
    const auto high=_mm_add_epi16(prefix(_mm_loadu_si128(reinterpret_cast<const __m128i*>(values+8))),
                                 _mm_unpackhi_epi64(carry,carry));
    // radius<=63 时完整窗口计数<=16129，有符号 16 位比较精确；所选粗桶必须包含该秩。
    const auto high_mask=_mm_movemask_epi8(_mm_cmpgt_epi16(high,target));
    return 8+std::size_t(std::countr_zero(static_cast<unsigned>(high_mask))/2);
  };
  for(std::size_t r=0;r<height;++r) {
    std::array<std::uint16_t,16> coarse{};
    std::array<std::array<std::uint16_t,16>,16> fine;
    std::array<std::size_t,16> last;
    last.fill(SIZE_MAX);
    for(std::size_t x=0;x<diameter;++x)add16(coarse.data(),mapped[x]+256);
    auto* out=destination.row_as<std::uint8_t>(r);
    for(std::size_t c=0;c<width;++c) {
      std::size_t below=0,bin=0;
      while(below+coarse[bin]<=middle)below+=coarse[bin++];
      auto& selected=fine[bin];const auto offset=bin*16;
      if(last[bin]==SIZE_MAX || c-last[bin]>=diameter) {
        selected.fill(0);
        for(std::size_t x=c;x<c+diameter;++x)add16(selected.data(),mapped[x]+offset);
      } else {
        for(auto x=last[bin];x<c;++x)slide16(selected.data(),mapped[x]+offset,mapped[x+diameter]+offset);
      }
      last[bin]=c;
      const auto value=select16(selected.data(),middle-below);
      out[c]=static_cast<std::uint8_t>(offset+value);
      if(c+1<width)slide16(coarse.data(),mapped[c]+256,mapped[c+diameter]+256);
    }
    if(r+1<height) {
      const auto* old=row(static_cast<std::int64_t>(r)-static_cast<std::int64_t>(radius));
      const auto* next=row(static_cast<std::int64_t>(r)+static_cast<std::int64_t>(radius)+1);
      for(std::size_t x=0;x<width;++x) {
        const auto a=old ? old[x] : constant,b=next ? next[x] : constant;
        if(a!=b) {auto* histogram=columns.data()+x*pitch;--histogram[a];--histogram[256+(a>>4)];++histogram[b];++histogram[256+(b>>4)];}
      }
    }
  }
  return true;
#else
  (void)image;(void)output;(void)channel;(void)mask;(void)radius;(void)border;(void)constant_gray;
  return false;
#endif
}

void validate_filter_image(const mv::core::Image& image, const char* where) {
  if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "image must not be empty");
}

}  // namespace

mv::core::Image mean_image(const mv::core::Image& image, std::size_t width, std::size_t height) {
  validate_filter_image(image, "mean_image");
  width = normalize_mask(width, "mean_image");
  height = normalize_mask(height, "mean_image");
  auto output = image.has_reduced_domain()?make_output_like(image):make_full_output_for_overwrite(image);
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
      using T = typename decltype(tag)::type;
      mean_channel<T>(image, output, channel, width, height);
    });
  }
  return output;
}

mv::core::Image gauss_filter(const mv::core::Image& image, std::size_t size) {
  validate_filter_image(image, "gauss_filter");
  if(size!=3&&size!=5&&size!=7&&size!=9&&size!=11)return advanced::gaussian_extended(image,size);
  auto output = image.has_reduced_domain()?make_output_like(image):make_full_output_for_overwrite(image);
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
      using T = typename decltype(tag)::type;
      gauss_channel<T>(image, output, channel, size);
    });
  }
  return output;
}

mv::core::Image gauss_filter_standard(const mv::core::Image& image,std::size_t size) {
  validate_filter_image(image,"gauss_filter");
  if(size!=3&&size!=5&&size!=7&&size!=9&&size!=11)return advanced::gaussian_extended(image,size);
  auto output=image.has_reduced_domain()?make_output_like(image):make_full_output_for_overwrite(image);
  for(std::size_t channel=0;channel<image.channel_count();++channel)
    dispatch_scalar(image.pixel_type(channel),[&](auto tag) {gauss_channel<typename decltype(tag)::type>(image,output,channel,size,true);});
  return output;
}

mv::core::Image median_image(const mv::core::Image& image,
                             MedianMask mask,
                             std::size_t radius,
                             BorderMode border,
                             double constant_gray) {
  validate_filter_image(image, "median_image");
  if (!std::isfinite(constant_gray)) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "median_image", "constant gray value must be finite");
  }
  if (radius == 0) return image.clone();
  auto output = image.has_reduced_domain()?make_output_like(image):make_full_output_for_overwrite(image);
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
      using T = typename decltype(tag)::type;
      if constexpr(std::is_same_v<T,std::uint8_t>) {
        if(median_byte_full_columns(image,output,channel,mask,radius,border,constant_gray))return;
      }
      median_channel<T>(image, output, channel, mask, radius, border, constant_gray);
    });
  }
  return output;
}

}  // namespace mv::imageproc
