#include <imageproc/operators.hpp>
#include <imageproc/histogram256.hpp>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <core/numeric_predicates.hpp>
#include <cstring>
#include <limits>
#include <memory>
#include <memory_resource>
#include <type_traits>
#include <vector>

#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/math.hpp>

#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_IMAGEPROC_SSE2 1
#else
#define MVKIT_IMAGEPROC_SSE2 0
#endif

namespace mv::imageproc {
namespace {
using namespace mv::core;

void require_single_channel(const Image& image, const char* operation) {
  if (image.empty() || image.channel_count() != 1) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    operation, "Image", -1,
                    "one nonempty single-channel image is required");
  }
}

template <class T>
T load_pixel(const std::byte* row, std::size_t column) {
  T value;
  std::memcpy(&value, row + column * sizeof(T), sizeof(T));
  return value;
}

template <class T>
T convert_pixel(double value) {
  if constexpr (std::is_floating_point_v<T>)
    return static_cast<T>(value);
  else {
    if (std::isnan(value)) return T{};
    // 保持 HALCON real→整数转换，包括有符号半值。
    return saturate_cast<T>(value);
  }
}

// 比较小数保留半整数前驱；直接加 0.5f 会误舍入。
void convert_f32_u8(const std::byte* source, std::uint8_t* destination,
                    std::size_t count, bool simd) {
  std::size_t i = 0;
#if MVKIT_IMAGEPROC_SSE2
  if (simd) {
    const auto zero = _mm_setzero_ps(), limit = _mm_set1_ps(255.f);
    const auto half = _mm_set1_ps(.5f);
    const auto one = _mm_set1_epi32(1);
    const auto convert = [&](std::size_t offset) {
      auto x = _mm_loadu_ps(reinterpret_cast<const float*>(source + offset * sizeof(float)));
      x = _mm_min_ps(_mm_max_ps(x, zero), limit);
      const auto integer = _mm_cvttps_epi32(x);
      const auto fraction = _mm_sub_ps(x, _mm_cvtepi32_ps(integer));
      return _mm_add_epi32(integer, _mm_and_si128(
          _mm_castps_si128(_mm_cmpge_ps(fraction, half)), one));
    };
    for (; count - i >= 16; i += 16) {
      const auto low = _mm_packs_epi32(convert(i), convert(i + 4));
      const auto high = _mm_packs_epi32(convert(i + 8), convert(i + 12));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(destination + i),
                      _mm_packus_epi16(low, high));
    }
  }
#else
  (void)simd;
#endif
  for (; i < count; ++i) {
    const double value = load_pixel<float>(source, i);
    if (!(value > 0)) destination[i] = 0;
    else if (value >= 255) destination[i] = 255;
    else {
      const auto integer = static_cast<unsigned>(value);
      destination[i] = static_cast<std::uint8_t>(integer + (value - integer >= .5));
    }
  }
}

void convert_f32_i32(const std::byte* source,std::int32_t* destination,std::size_t count,bool simd) {
  std::size_t i=0;
#if MVKIT_IMAGEPROC_SSE2
  if(simd) {
    const auto low=_mm_set1_pd(double(INT32_MIN)),high=_mm_set1_pd(double(INT32_MAX));
    const auto one=_mm_set1_pd(1.),half=_mm_set1_pd(.5),minus_half=_mm_set1_pd(-.5);
    const auto convert=[&](__m128d value) {
      value=_mm_min_pd(_mm_max_pd(value,low),high);
      const auto integer=_mm_cvttpd_epi32(value);
      const auto fraction=_mm_sub_pd(value,_mm_cvtepi32_pd(integer));
      const auto up=_mm_cvttpd_epi32(_mm_and_pd(_mm_cmpge_pd(fraction,half),one));
      const auto down=_mm_cvttpd_epi32(_mm_and_pd(_mm_cmple_pd(fraction,minus_half),one));
      return _mm_sub_epi32(_mm_add_epi32(integer,up),down);
    };
    for(;count-i>=4;i+=4) {
      auto value=_mm_loadu_ps(reinterpret_cast<const float*>(source+i*sizeof(float)));
      value=_mm_and_ps(value,_mm_cmpord_ps(value,value)); // NaN 按原转换规则输出零。
      const auto a=convert(_mm_cvtps_pd(value));
      const auto b=convert(_mm_cvtps_pd(_mm_movehl_ps(value,value)));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(destination+i),_mm_unpacklo_epi64(a,b));
    }
  }
#else
  (void)simd;
#endif
  for(;i<count;++i)destination[i]=convert_pixel<std::int32_t>(load_pixel<float>(source,i));
}

template <class Output>
void convert_u8(const std::uint8_t* source, Output* destination,
                std::size_t count, bool simd) {
  std::size_t index = 0;
#if MVKIT_IMAGEPROC_SSE2
  if (simd) {
    const auto zero = _mm_setzero_si128();
    for (; count - index >= 16; index += 16) {
      const auto bytes =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(source + index));
      const auto low = _mm_unpacklo_epi8(bytes, zero);
      const auto high = _mm_unpackhi_epi8(bytes, zero);
      if constexpr (std::is_same_v<Output, std::uint16_t>) {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(destination + index), low);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(destination + index + 8),
                         high);
      } else {
        _mm_storeu_ps(destination + index,
                      _mm_cvtepi32_ps(_mm_unpacklo_epi16(low, zero)));
        _mm_storeu_ps(destination + index + 4,
                      _mm_cvtepi32_ps(_mm_unpackhi_epi16(low, zero)));
        _mm_storeu_ps(destination + index + 8,
                      _mm_cvtepi32_ps(_mm_unpacklo_epi16(high, zero)));
        _mm_storeu_ps(destination + index + 12,
                      _mm_cvtepi32_ps(_mm_unpackhi_epi16(high, zero)));
      }
    }
  }
#else
  (void)simd;
#endif
  for (; index < count; ++index)
    destination[index] = static_cast<Output>(source[index]);
}

void convert_u16_f32(const std::byte* source,float* target,std::size_t count,bool simd) {
  std::size_t i=0;
#if MVKIT_IMAGEPROC_SSE2
  if(simd) {
    // UInt16 零扩展到 Float32 后每个值均可精确表示。
    const auto zero=_mm_setzero_si128();
    for(;count-i>=8;i+=8) {
      const auto x=_mm_loadu_si128(reinterpret_cast<const __m128i*>(source+i*sizeof(std::uint16_t)));
      _mm_storeu_ps(target+i,_mm_cvtepi32_ps(_mm_unpacklo_epi16(x,zero)));
      _mm_storeu_ps(target+i+4,_mm_cvtepi32_ps(_mm_unpackhi_epi16(x,zero)));
    }
  }
#else
  (void)simd;
#endif
  for(;i<count;++i)target[i]=convert_pixel<float>(load_pixel<std::uint16_t>(source,i));
}

std::size_t partition_count(const Region& domain) {
  return domain.area() < ExecutionContext::kArithmeticParallelWork
             ? 1U
             : std::min<std::size_t>(8, domain.runs().size());
}

Region statistics_domain(const Region& region, const Image& image,
                         const char* operation) {
  require_single_channel(image, operation);
  auto domain = region.intersect(image.domain());
  if (domain.empty())
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, operation,
                    "Region", -1, "region and image domain do not overlap");
  return domain;
}

std::size_t histogram_bins(const Image& image, const char* operation) {
  require_single_channel(image, operation);
  if (image.pixel_type() == PixelType::UInt8) return 256;
  if (image.pixel_type() == PixelType::UInt16) return 65536;
  throw Exception(Status::Unsupported, ErrorCode::InvalidType, operation,
                  "Image", -1,
                  "only UInt8 and UInt16 histograms are supported");
}

// 固定最多八个分区；调用方预分配，worker 不共享可变 arena，整数合并与后端无关。
std::pmr::vector<std::uint64_t> histogram_counts(const Image& image,
                                                 const Region& domain,
                                                 std::size_t bins,
                                                 ExecutionContext& context) {
  const auto parts = partition_count(domain);
  std::pmr::vector<std::uint64_t> counts(parts * bins, 0,
                                         context.scratch_resource());
  const auto runs = domain.runs();
  const auto* base = static_cast<const std::byte*>(image.data());
  const auto stride = image.stride();
  ExecutionContext::for_each_partition(
      parts, static_cast<std::size_t>(domain.area()),
      ExecutionContext::kArithmeticParallelWork, [&](auto range) {
        for (auto part = range.begin; part < range.end; ++part) {
          auto* histogram = counts.data() + part * bins;
          const auto begin =
              runs.size() / parts * part + std::min(part, runs.size() % parts);
          const auto end = begin + runs.size() / parts +
                           (part < runs.size() % parts ? 1U : 0U);
          if(bins==256) {
            const auto local=detail::histogram256<std::uint8_t>(image,runs.subspan(begin,end-begin));
            std::copy(local.begin(),local.end(),histogram);
            continue;
          }
          for (auto index = begin; index < end; ++index) {
            const auto& run = runs[index];
            const auto* row = base + static_cast<std::size_t>(run.row) * stride;
            for (auto col = std::int64_t(run.col_begin); col <= run.col_end; ++col) {
              const auto c = static_cast<std::size_t>(col);
              const auto value = load_pixel<std::uint16_t>(row, c);
              ++histogram[value];
            }
          }
        }
      });
  for (std::size_t part = 1; part < parts; ++part)
    for (std::size_t bin = 0; bin < bins; ++bin)
      counts[bin] += counts[part * bins + bin];
  counts.resize(bins);
  return counts;
}

template <class Fn>
IconicTuple map_images(const IconicTuple& images, const char* operation,
                       Fn&& fn) {
  images.require_kind(IconicKind::Image, operation);
  if (images.size() == 1) return IconicTuple{core::IconicValue::from_image(fn(images.at(0).as_image()))};
  // 此 vector 是既有 make_image_tuple 的适配边界，不保存像素或算法工作区。
  std::vector<Image> result;
  result.reserve(images.size());
  for (const auto& value : images.values())
    result.push_back(fn(value.as_image()));
  return make_image_tuple(std::move(result));
}
}  // namespace

Image convert_image_type(const Image& image, PixelType type) {
  (void)bytes_per_sample(type);
  if (image.empty())
    throw Exception(Status::InvalidArgument, "convert_image_type",
                    "image is empty");
  ExecutionContext context;
  std::pmr::vector<PixelType> types(image.channel_count(), type,
                                    context.scratch_resource());
  auto output = image.has_reduced_domain()
      ? Image::create(image.width(), image.height(), types)
      : Image::create_uninitialized(image.width(), image.height(), types);
  const auto domain = image.domain();
  const auto runs = domain.runs();
  const auto& cpu = context.runtime().cpu();
  const auto effective =
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware());
  const bool simd =
      MVKIT_IMAGEPROC_SSE2 &&
      mv::runtime::kernel_allowed(effective, {true, false, false, false});
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    const auto* source = static_cast<const std::byte*>(image.data(channel));
    auto* target = static_cast<std::byte*>(output.mutable_data(channel));
    const auto source_stride = image.stride(channel),
               target_stride = output.stride(channel);
    dispatch_scalar(image.pixel_type(channel), [&](auto source_tag) {
      using Input = typename decltype(source_tag)::type;
      dispatch_scalar(type, [&](auto target_tag) {
        using Output = typename decltype(target_tag)::type;
        ExecutionContext::for_each_partition(
            runs.size(), static_cast<std::size_t>(domain.area()),
            ExecutionContext::kArithmeticParallelWork, [&](auto range) {
              for (auto i = range.begin; i < range.end; ++i) {
                const auto& run = runs[i];
                const auto row = static_cast<std::size_t>(run.row),
                           begin = static_cast<std::size_t>(run.col_begin);
                const auto count =
                    static_cast<std::size_t>(std::int64_t(run.col_end) - std::int64_t(run.col_begin) + 1);
                const auto* in = source + row * source_stride;
                auto* out =
                    reinterpret_cast<Output*>(target + row * target_stride) +
                    begin;
                if constexpr (std::is_same_v<Input, std::uint8_t> &&
                              (std::is_same_v<Output, std::uint16_t> ||
                               std::is_same_v<Output, float>)) {
                  convert_u8(reinterpret_cast<const std::uint8_t*>(in) + begin,
                             out, count, simd);
                } else if constexpr (std::is_same_v<Input,std::uint16_t> && std::is_same_v<Output,float>) {
                  convert_u16_f32(in+begin*sizeof(Input),out,count,simd);
                } else if constexpr (std::is_same_v<Input, float> &&
                                     std::is_same_v<Output, std::uint8_t>) {
                  convert_f32_u8(in + begin * sizeof(Input), out, count, simd);
                } else if constexpr (std::is_same_v<Input,float> && std::is_same_v<Output,std::int32_t>) {
                  convert_f32_i32(in+begin*sizeof(Input),out,count,simd);
                } else {
                  for (std::size_t c = 0; c < count; ++c)
                    out[c] =
                        convert_pixel<Output>(load_pixel<Input>(in, begin + c));
                }
              }
            });
      });
    });
  }
  return image.has_reduced_domain() ? output.reduce_domain(domain) : output;
}

Image access_channel(const Image& image, std::size_t channel) {
  if (channel == 0 || channel > image.channel_count())
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                    "access_channel", "Channel", -1,
                    "channel must be in [1, channel_count]");
  return image.channel_view(channel-1);
}

Image compose3(const Image& first, const Image& second, const Image& third) {
  const std::array<const Image*, 3> sources{&first, &second, &third};
  std::array<ImagePlaneSpec, 3> planes;
  for (std::size_t index = 0; index < 3; ++index) {
    const auto& source = *sources[index];
    require_single_channel(source, "compose3");
    if (source.width() != first.width() || source.height() != first.height())
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                      "compose3", "Image", -1, "input dimensions must match");
    planes[index] = source.shared_plane(0);
  }
  auto output = Image::wrap_planes(first.width(), first.height(), planes);
  if (first.has_reduced_domain() || second.has_reduced_domain() ||
      third.has_reduced_domain())
    output = output.reduce_domain(
        first.domain().intersect(second.domain()).intersect(third.domain()));
  return output;
}

namespace {
template <class T>
IntensityResult shifted_intensity(const Region& domain, const Image& image) {
  const auto* base = static_cast<const std::byte*>(image.data());
  const auto stride = image.stride(), parts = partition_count(domain);
  const auto runs = domain.runs();
  const double anchor = load_pixel<T>(base + runs.front().row * stride, runs.front().col_begin);
  const double count = static_cast<double>(domain.area());
  ExecutionContext context;
  const auto& cpu = context.runtime().cpu();
  [[maybe_unused]] const bool simd = MVKIT_IMAGEPROC_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()), {true,false,false,false});
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif

#if MVKIT_FILTER_HAS_AVX2
  if constexpr(std::is_same_v<T,std::uint16_t>) {
    // 保守界内全部中间量为精确 Int64；转 Double 前将整数矩平移到整数均值，
    // 即使接近 UInt16 最大值也保留小方差。
    if(avx2 && domain.area()<=std::uint64_t(INT64_MAX)/(4ULL*32768*32768)) {
      std::array<std::int64_t,8> sums{};std::array<std::uint64_t,8> squares{};
      ExecutionContext::for_each_partition(parts,std::size_t(domain.area()),ExecutionContext::kArithmeticParallelWork,[&](auto range) {
        for(auto part=range.begin;part<range.end;++part) {
          const auto begin=runs.size()/parts*part+std::min(part,runs.size()%parts);
          const auto end=begin+runs.size()/parts+(part<runs.size()%parts ? 1U : 0U);
          u16_moments_avx2(base,stride,runs.subspan(begin,end-begin),&sums[part],&squares[part]);
        }
      });
      std::int64_t sum=0,square=0;for(std::size_t p=0;p<parts;++p){sum+=sums[p];square+=std::int64_t(squares[p]);}
      const auto n=std::int64_t(domain.area()),center=sum/n,remainder=sum-center*n;
      const auto centered_square=square-2*center*sum+center*center*n;
      const double fraction=double(remainder)/count;
      return {double(center+32768)+fraction,std::sqrt(std::max(0.,double(centered_square)/count-fraction*fraction))};
    }
  }
#endif
#if MVKIT_FILTER_HAS_AVX2
  if constexpr(std::is_same_v<T,float>) {
    if(avx2) {
      std::array<double,8> means{},m2{};std::array<std::uint64_t,8> sizes{};std::array<int,8> statuses{};
      ExecutionContext::for_each_partition(parts,std::size_t(domain.area()),ExecutionContext::kArithmeticParallelWork,[&](auto range) {
        for(auto part=range.begin;part<range.end;++part) {
          const auto begin=runs.size()/parts*part+std::min(part,runs.size()%parts);
          const auto end=begin+runs.size()/parts+(part<runs.size()%parts ? 1U : 0U);
          statuses[part]=f32_block_moments_avx2(base,stride,runs.subspan(begin,end-begin),anchor,&means[part],&m2[part],&sizes[part]);
          if(statuses[part]<0)throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"intensity","Image",-1,"non-finite value in effective domain");
        }
      });
      bool stable=true;for(std::size_t p=0;p<parts;++p)stable=stable && statuses[p]==1;
      if(stable) {
        double mean=0,variance_sum=0;std::uint64_t size=0;
        for(std::size_t p=0;p<parts;++p)if(sizes[p]) {
          const auto next=size+sizes[p];const auto d=means[p]-mean;
          variance_sum+=m2[p]+d*d*(double(size)*double(sizes[p])/double(next));
          mean+=d*(double(sizes[p])/double(next));size=next;
        }
        return {anchor+mean,std::sqrt(std::max(0.,variance_sum/count))};
      }
    }
  }
#endif
  std::array<double,8> totals{};
  double offset = 0;
  const auto pass = [&](auto variance) {
    static constexpr bool square = decltype(variance)::value;
    totals.fill(0);
    ExecutionContext::for_each_partition(parts, static_cast<std::size_t>(domain.area()),
        ExecutionContext::kArithmeticParallelWork, [&](auto range) {
      for (auto part = range.begin; part < range.end; ++part) {
        const auto begin = runs.size()/parts*part + std::min(part,runs.size()%parts);
        const auto end = begin + runs.size()/parts + (part < runs.size()%parts ? 1U : 0U);
#if MVKIT_FILTER_HAS_AVX2
        if(avx2) {
          if(!shifted_sum_avx2(base,stride,runs.subspan(begin,end-begin),std::is_same_v<T,float>,anchor,offset,square,&totals[part]))
            throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"intensity","Image",-1,"non-finite value in effective domain");
          continue;
        }
#endif
        double sums[4]{};
#if MVKIT_IMAGEPROC_SSE2
        auto sum0=_mm_setzero_pd(),sum1=_mm_setzero_pd();
        const auto origin=_mm_set1_pd(anchor), center=_mm_set1_pd(offset);
#endif
        for (auto i=begin;i<end;++i) {
          const auto& run=runs[i];
          const auto* row=base+std::int64_t(run.row)*stride;
          auto c=std::int64_t(run.col_begin);
#if MVKIT_IMAGEPROC_SSE2
          if (simd) for (;std::int64_t(run.col_end)-c>=3;c+=4) {
            __m128d a,b;
            if constexpr (std::is_same_v<T,float>) {
              const auto pixels=_mm_loadu_ps(reinterpret_cast<const float*>(row)+c);
              a=_mm_cvtps_pd(pixels);b=_mm_cvtps_pd(_mm_movehl_ps(pixels,pixels));
              if constexpr (!square) {
                const auto sign=_mm_set1_pd(-0.);
                const auto limit=_mm_set1_pd(std::numeric_limits<double>::max());
                if (_mm_movemask_pd(_mm_and_pd(_mm_cmple_pd(_mm_andnot_pd(sign,a),limit),
                                               _mm_cmple_pd(_mm_andnot_pd(sign,b),limit))) != 3)
                  throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"intensity","Image",-1,"non-finite value in effective domain");
              }
            } else {
              const auto pixels=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+c*sizeof(T)));
              const auto integers=_mm_unpacklo_epi16(pixels,_mm_setzero_si128());
              a=_mm_cvtepi32_pd(integers);b=_mm_cvtepi32_pd(_mm_srli_si128(integers,8));
            }
            a=_mm_sub_pd(a,origin);b=_mm_sub_pd(b,origin);
            if constexpr(square) {a=_mm_sub_pd(a,center);b=_mm_sub_pd(b,center);a=_mm_mul_pd(a,a);b=_mm_mul_pd(b,b);}
            sum0=_mm_add_pd(sum0,a);sum1=_mm_add_pd(sum1,b);
          }
#endif
          for (;std::int64_t(run.col_end)-c>=3;c+=4) for (int j=0;j<4;++j) {
            const double value=load_pixel<T>(row,c+j);
            if constexpr (!square && std::is_floating_point_v<T>)
              if (!core::is_finite(value)) throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"intensity","Image",-1,"non-finite value in effective domain");
            const auto delta=(value-anchor)-(square ? offset : 0.);
            sums[j]+=square ? delta*delta : delta;
          }
          for (;c<=run.col_end;++c) {
            const double value=load_pixel<T>(row,c);
            if constexpr (!square && std::is_floating_point_v<T>)
              if (!core::is_finite(value)) throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"intensity","Image",-1,"non-finite value in effective domain");
            const auto delta=(value-anchor)-(square ? offset : 0.);
            sums[0]+=square ? delta*delta : delta;
          }
        }
#if MVKIT_IMAGEPROC_SSE2
        double pair[2];_mm_storeu_pd(pair,_mm_add_pd(sum0,sum1));sums[0]+=pair[0];sums[1]+=pair[1];
#endif
        totals[part]=(sums[0]+sums[1])+(sums[2]+sums[3]);
      }
    });
    double total=0;for(std::size_t p=0;p<parts;++p)total+=totals[p];return total;
  };
  offset=pass(std::false_type{})/count;
  const auto variance=pass(std::true_type{})/count;
  return {anchor+offset,std::sqrt(std::max(0.,variance))};
}
} // namespace

IntensityResult intensity(const Region& region, const Image& image) {
  const auto domain = statistics_domain(region, image, "intensity");
  if (image.pixel_type()==PixelType::UInt16) return shifted_intensity<std::uint16_t>(domain,image);
  if (image.pixel_type()==PixelType::Float32) return shifted_intensity<float>(domain,image);
#if MVKIT_IMAGEPROC_SSE2
  ExecutionContext simd_context;
  const auto& simd_cpu=simd_context.runtime().cpu();
  if (mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
          simd_cpu.selection(),simd_cpu.hardware()),{true,false,false,false}) &&
      image.pixel_type() == PixelType::UInt8 &&
      domain.area() <= std::numeric_limits<std::uint64_t>::max() / 65025U) {
    std::uint64_t sum = 0, square = 0;
    const auto* base = static_cast<const std::byte*>(image.data());
    const auto stride = image.stride();
    const auto zero = _mm_setzero_si128();
    for (const auto& run : domain.runs()) {
      const auto* row = reinterpret_cast<const std::uint8_t*>(base + std::int64_t(run.row) * stride);
      auto c = std::int64_t(run.col_begin);
      while (std::int64_t(run.col_end) - c + 1 >= 16) {
        auto sums = zero, squares = zero;
        const auto blocks = std::min<std::int64_t>(4096, (std::int64_t(run.col_end) - c + 1) / 16);
        for (std::int64_t j = 0; j < blocks; ++j, c += 16) {
          const auto v = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row + c));
          sums = _mm_add_epi64(sums, _mm_sad_epu8(v, zero));
          const auto lo = _mm_unpacklo_epi8(v, zero), hi = _mm_unpackhi_epi8(v, zero);
          squares = _mm_add_epi32(squares, _mm_add_epi32(_mm_madd_epi16(lo, lo), _mm_madd_epi16(hi, hi)));
        }
        alignas(16) std::uint64_t s[2];
        alignas(16) std::uint32_t q[4];
        _mm_store_si128(reinterpret_cast<__m128i*>(s), sums);
        _mm_store_si128(reinterpret_cast<__m128i*>(q), squares);
        sum += s[0] + s[1];
        for (auto value : q) square += value;
      }
      for (; c <= run.col_end; ++c) { sum += row[c]; square += unsigned(row[c]) * unsigned(row[c]); }
    }
    const double n = static_cast<double>(domain.area()), mean = static_cast<double>(sum) / n;
    return {mean, std::sqrt(std::max(0.0, static_cast<double>(square) / n - mean * mean))};
  }
#endif
  const auto parts = partition_count(domain), stride = image.stride();
  struct Moment {
    std::uint64_t count{};
    double mean{};
    double m2{};
  };
  std::array<Moment, 8> moments{};
  const auto runs = domain.runs();
  const auto* base = static_cast<const std::byte*>(image.data());
  dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    ExecutionContext::for_each_partition(
        parts, static_cast<std::size_t>(domain.area()),
        ExecutionContext::kArithmeticParallelWork, [&](auto range) {
          for (auto part = range.begin; part < range.end; ++part) {
            auto& moment = moments[part];
            const auto begin = runs.size() / parts * part +
                               std::min(part, runs.size() % parts);
            const auto end = begin + runs.size() / parts +
                             (part < runs.size() % parts ? 1U : 0U);
            for (auto i = begin; i < end; ++i) {
              const auto& run = runs[i];
              const auto* row =
                  base + static_cast<std::size_t>(run.row) * stride;
              for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
                const double value =
                    load_pixel<T>(row, static_cast<std::size_t>(c));
                if (!core::is_finite(value))
                  throw Exception(Status::InvalidArgument,
                                  ErrorCode::InvalidValue, "intensity", "Image",
                                  -1, "non-finite value in effective domain");
                ++moment.count;
                const auto delta = value - moment.mean;
                moment.mean += delta / static_cast<double>(moment.count);
                moment.m2 += delta * (value - moment.mean);
              }
            }
          }
        });
  });
  auto sum = moments[0];
  for (std::size_t p = 1; p < parts; ++p) {
    const auto& next = moments[p];
    if (next.count == 0) continue;
    const auto total = sum.count + next.count;
    const auto delta = next.mean - sum.mean;
    sum.m2 += next.m2 + delta * delta *
                            (static_cast<double>(sum.count) *
                             static_cast<double>(next.count) /
                             static_cast<double>(total));
    sum.mean +=
        delta * (static_cast<double>(next.count) / static_cast<double>(total));
    sum.count = total;
  }
  return {sum.mean,
          std::sqrt(std::max(0.0, sum.m2 / static_cast<double>(sum.count)))};
}

HistogramResult gray_histo(const Region& region, const Image& image) {
  const auto bins = histogram_bins(image, "gray_histo");
  const auto domain = statistics_domain(region, image, "gray_histo");
  ExecutionContext context;
  const auto counts = histogram_counts(image, domain, bins, context);
  HistogramResult result;
  std::array<std::uint64_t, 256> reduced{};
  std::size_t first = 0, last = bins - 1;
  while (counts[first] == 0) ++first;
  while (counts[last] == 0) --last;
  for (std::size_t i = first; i <= last; ++i) {
    const auto bin =
        bins == 256
            ? i
            : (first == last ? 0U : (i - first) * 255U / (last - first));
    reduced[bin] += counts[i];
  }
  std::vector<std::int64_t> absolute;
  std::vector<double> relative;
  absolute.reserve(reduced.size());
  relative.reserve(reduced.size());
  const double total = static_cast<double>(domain.area());
  for (const auto count : reduced) {
    if (count >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw Exception(Status::OutOfRange, "gray_histo",
                      "histogram count exceeds Int64");
    absolute.push_back(static_cast<std::int64_t>(count));
    relative.push_back(static_cast<double>(count) / total);
  }
  result.absolute = Tuple{std::move(absolute)};
  result.relative = Tuple{std::move(relative)};
  return result;
}

HistogramResult gray_histo_standard(const Region& region,const Image& image) {
  require_single_channel(image,"gray_histo");
  const auto full=image.full_domain();
  const auto domain=region.intersect(full.domain());
  if(!domain.empty()&&image.pixel_type()==PixelType::UInt8)return gray_histo(domain,full);
  std::array<std::int64_t,256> counts{};
  if(!domain.empty()) {
    const auto* base=static_cast<const std::byte*>(image.data());
    const auto stride=image.stride();
    dispatch_scalar(image.pixel_type(),[&](auto tag) {
      using T=typename decltype(tag)::type;
      double minimum=std::numeric_limits<double>::infinity(),maximum=-minimum;
      for(const auto& run:domain.runs()) {
        const auto* row=base+static_cast<std::size_t>(run.row)*stride;
        for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c) {
          const double value=load_pixel<T>(row,static_cast<std::size_t>(c));
          if(!core::is_finite(value))throw Exception(Status::InvalidArgument,ErrorCode::InvalidValue,"gray_histo","Image",-1,"non-finite gray value");
          minimum=std::min(minimum,value);maximum=std::max(maximum,value);
        }
      }
      for(const auto& run:domain.runs()) {
        const auto* row=base+static_cast<std::size_t>(run.row)*stride;
        for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c) {
          const double mapped=maximum==minimum ? 0 : (load_pixel<T>(row,static_cast<std::size_t>(c))-minimum)*255.0/(maximum-minimum);
          const auto bin=static_cast<std::size_t>(std::clamp(std::is_floating_point_v<T> ? std::floor(mapped+.5) : std::floor(mapped),0.,255.));
          ++counts[bin];
        }
      }
    });
  }
  std::vector<std::int64_t> absolute(counts.begin(),counts.end());
  std::vector<double> relative;relative.reserve(counts.size());
  const auto total=static_cast<double>(domain.area());
  for(auto count:counts)relative.push_back(total==0 ? 0 : static_cast<double>(count)/total);
  return {Tuple{std::move(absolute)},Tuple{std::move(relative)}};
}

BinaryThresholdResult binary_threshold(const Image& image,
                                       ThresholdPolarity polarity) {
  if (polarity != ThresholdPolarity::Light &&
      polarity != ThresholdPolarity::Dark)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "binary_threshold", "Polarity", -1, "invalid polarity");
  const auto bins = histogram_bins(image, "binary_threshold");
  if(image.domain().empty())return {Region{},0};
  const auto domain =
      statistics_domain(image.domain(), image, "binary_threshold");
  ExecutionContext context;
  const auto counts = histogram_counts(image, domain, bins, context);
  const double total = static_cast<double>(domain.area());
  double total_sum = 0;
  std::size_t split = 0;
  for (std::size_t i = 0; i < bins; ++i)
    total_sum += static_cast<double>(i) * static_cast<double>(counts[i]);
  double lower_count = 0, lower_sum = 0, best = -1;
  for (std::size_t i = 0; i + 1 < bins; ++i) {
    lower_count += static_cast<double>(counts[i]);
    lower_sum += static_cast<double>(i) * static_cast<double>(counts[i]);
    if (lower_count == 0 || lower_count == total) continue;
    const double delta = lower_sum / lower_count -
                         (total_sum - lower_sum) / (total - lower_count);
    const double score =
        (lower_count / total) * ((total - lower_count) / total) * delta * delta;
    // 保留首个最大值，无对比时为零；一般直方图数值差异继续保留在严格 HALCON 矩阵。
    if (score > best) {
      best = score;
      split = i;
    }
  }
  const auto threshold_value = static_cast<double>(split);
  // 最大切分处仍防护反向区间。
  auto region =
      polarity == ThresholdPolarity::Dark
          ? threshold(image, 0, threshold_value)
          : (split == bins - 1 ? Region{}
                               : threshold(image, threshold_value + 1,
                                           static_cast<double>(bins - 1)));
  return {std::move(region), threshold_value};
}

IconicTuple convert_image_type(const IconicTuple& images, PixelType type) {
  (void)bytes_per_sample(type);
  return map_images(images, "convert_image_type", [&](const auto& image) {
    return convert_image_type(image, type);
  });
}
IconicTuple access_channel(const IconicTuple& images, std::size_t channel) {
  if (channel == 0)
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                    "access_channel", "Channel", -1, "channel is 1-based");
  return map_images(images, "access_channel", [&](const auto& image) {
    return access_channel(image, channel);
  });
}
IconicTuple compose3(const IconicTuple& first, const IconicTuple& second,
                     const IconicTuple& third) {
  first.require_kind(IconicKind::Image, "compose3");
  second.require_kind(IconicKind::Image, "compose3");
  third.require_kind(IconicKind::Image, "compose3");
  if (first.empty() || second.empty() || third.empty()) return {};
  const auto plan = BroadcastPlan::make({{first.size(), "Image1", true},
                                         {second.size(), "Image2", true},
                                         {third.size(), "Image3", true}},
                                        "compose3");
  std::vector<Image> result;
  result.reserve(plan.size());
  for (std::size_t i = 0; i < plan.size(); ++i)
    result.push_back(compose3(first.at(plan.source_index(0, i)).as_image(),
                              second.at(plan.source_index(1, i)).as_image(),
                              third.at(plan.source_index(2, i)).as_image()));
  return make_image_tuple(std::move(result));
}
IntensityTuples intensity(const Regions& regions, const IconicTuple& images) {
  images.require_kind(IconicKind::Image, "intensity");
  IntensityTuples result;
  if (regions.empty() || images.empty()) return result;
  const auto plan = BroadcastPlan::make(
      {{regions.size(), "Region", true}, {images.size(), "Image", true}},
      "intensity");
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const auto value = intensity(regions[plan.source_index(0, i)],
                                 images.at(plan.source_index(1, i)).as_image());
    result.mean.append(value.mean);
    result.deviation.append(value.deviation);
  }
  return result;
}
HistogramResult gray_histo(const Regions& regions, const IconicTuple& images) {
  images.require_kind(IconicKind::Image, "gray_histo");
  HistogramResult result;
  if (regions.empty() || images.empty()) return result;
  const auto plan = BroadcastPlan::make(
      {{regions.size(), "Region", true}, {images.size(), "Image", true}},
      "gray_histo");
  // 公开标量流程已拥有两组完整强类型 Tuple；保留广播验证后直接转移，
  // 避免将相同 512 个值再次展平、append 及扩容。
  if (plan.size() == 1)
    return gray_histo(regions[plan.source_index(0, 0)],
                      images.at(plan.source_index(1, 0)).as_image());
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const auto value =
        gray_histo(regions[plan.source_index(0, i)],
                   images.at(plan.source_index(1, i)).as_image());
    for (std::size_t bin = 0; bin < value.absolute.size(); ++bin) {
      result.absolute.append(value.absolute.as_int64(bin));
      result.relative.append(value.relative.as_real64(bin));
    }
  }
  return result;
}
BinaryThresholdTuples binary_threshold(const IconicTuple& images,
                                       ThresholdPolarity polarity) {
  images.require_kind(IconicKind::Image, "binary_threshold");
  BinaryThresholdTuples result;
  if (polarity != ThresholdPolarity::Light &&
      polarity != ThresholdPolarity::Dark)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "binary_threshold", "Polarity", -1, "invalid polarity");
  std::vector<IconicValue> regions;
  regions.reserve(images.size());
  for (const auto& image : images.values()) {
    auto value = binary_threshold(image.as_image(), polarity);
    result.thresholds.append(static_cast<std::int64_t>(value.threshold));
    regions.push_back(IconicValue::from_region(std::move(value.region)));
  }
  result.regions = IconicTuple{std::move(regions)};
  return result;
}
}  // namespace mv::imageproc
