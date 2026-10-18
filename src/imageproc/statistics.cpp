#include <imageproc/operators.hpp>
#include <imageproc/histogram256.hpp>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory_resource>
#include <vector>
#if defined(_M_X64) || defined(__x86_64__)
#include <emmintrin.h>
#endif

#include <core/exception.hpp>
#include <core/broadcast.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/pixel_traits.hpp>
#include <runtime/memory/scratch_arena.hpp>

namespace mv::imageproc {
namespace {

template <typename T>
MinMaxGrayResult min_max_impl(const mv::core::Image& image, const mv::core::Region& domain, double percent) {
  const ImageView view{image};
  if (percent == 0 && !domain.empty()) {
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
#if MVKIT_FILTER_HAS_AVX2
    if constexpr(std::is_same_v<T,std::uint8_t> || std::is_same_v<T,std::uint16_t> || std::is_same_v<T,float>) {
      ExecutionContext context;
      const auto& processor=context.runtime().cpu();
      if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(processor.selection(),processor.hardware()),{false,true,false,false})) {
        min_max_gray_avx2(image.data(),image.stride(),domain,
                         std::is_same_v<T,std::uint8_t>?0:std::is_same_v<T,std::uint16_t>?1:2,&minimum,&maximum);
        return {minimum,maximum,maximum-minimum};
      }
    }
#endif
#if defined(_M_X64) || defined(__x86_64__)
    ExecutionContext execution;
    const auto& cpu=execution.runtime().cpu();
    const bool simd=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
        cpu.selection(),cpu.hardware()),{true,false,false,false});
    auto vmin = _mm_set1_epi8(static_cast<char>(255));
    auto vmax = _mm_setzero_si128();
#endif
    for (const auto& run : domain.runs()) {
      const auto* row = view.row_as<T>(static_cast<std::size_t>(run.row));
      auto c = std::int64_t(run.col_begin);
#if defined(_M_X64) || defined(__x86_64__)
      if constexpr (std::is_same_v<T, std::uint8_t>) {
        for (; simd && std::int64_t(run.col_end) - c + 1 >= 16; c += 16) {
          const auto v = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row + c));
          vmin = _mm_min_epu8(vmin, v); vmax = _mm_max_epu8(vmax, v);
        }
      }
#endif
      for (; c <= run.col_end; ++c) {
        minimum = std::min(minimum, double(row[c]));
        maximum = std::max(maximum, double(row[c]));
      }
      if constexpr(std::is_same_v<T,std::uint8_t>) {
        bool found_min=minimum==0,found_max=maximum==255;
#if defined(_M_X64) || defined(__x86_64__)
        if(simd) {
          found_min|=_mm_movemask_epi8(_mm_cmpeq_epi8(vmin,_mm_setzero_si128()))!=0;
          found_max|=_mm_movemask_epi8(_mm_cmpeq_epi8(vmax,_mm_set1_epi8(static_cast<char>(255))))!=0;
        }
#endif
        if(found_min && found_max)return {0,255,255};
      }
    }
#if defined(_M_X64) || defined(__x86_64__)
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      alignas(16) std::uint8_t low[16], high[16];
      _mm_store_si128(reinterpret_cast<__m128i*>(low), vmin);
      _mm_store_si128(reinterpret_cast<__m128i*>(high), vmax);
      for (int i = 0; i < 16; ++i) { minimum = std::min(minimum, double(low[i])); maximum = std::max(maximum, double(high[i])); }
    }
#endif
    return {minimum, maximum, maximum - minimum};
  }
  if constexpr(std::is_same_v<T,std::uint8_t>) {
    if(!domain.empty()) {
      const auto counts=detail::histogram256<std::uint8_t>(image,domain.runs());
      const auto size=domain.area();
      auto trim=static_cast<std::uint64_t>(std::floor(static_cast<double>(size)*percent/100.0+0.5));
      trim=std::min(trim,size-1);
      std::uint64_t cumulative=0;
      double minimum=0,maximum=255;
      for(std::size_t i=0;i<counts.size();++i) {
        cumulative+=counts[i];
        if(cumulative>trim) {minimum=static_cast<double>(i);break;}
      }
      cumulative=0;
      for(std::size_t i=counts.size();i-->0;) {
        cumulative+=counts[i];
        if(cumulative>trim) {maximum=static_cast<double>(i);break;}
      }
      if(minimum>maximum){const double middle=std::floor((minimum+maximum)/2+0.5);return {middle,middle,0};}
      return {minimum,maximum,maximum-minimum};
    }
  }
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<double> values{scratch.resource()};
  if (domain.area() <= static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    values.reserve(static_cast<std::size_t>(domain.area()));
  }
  for (const auto& run : domain.runs()) {
    const auto* row = view.row_as<T>(static_cast<std::size_t>(run.row));
    for (std::int64_t c = run.col_begin;; ++c) {
      values.push_back(static_cast<double>(row[static_cast<std::size_t>(c)]));
      if (c == run.col_end) break;
    }
  }
  if (values.empty()) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "min_max_gray", "region and image domain do not overlap");
  }
  std::sort(values.begin(), values.end());
  std::size_t trim = static_cast<std::size_t>(std::floor(static_cast<double>(values.size()) * percent / 100.0+0.5));
  trim=std::min(trim,values.size()-1);
  const double min_value = values[trim];
  const double max_value = values[values.size() - 1 - trim];
  if(min_value>max_value){const double middle=std::is_integral_v<T>?std::floor((min_value+max_value)/2+0.5):(min_value+max_value)/2;return {middle,middle,0};}
  return {min_value, max_value, max_value - min_value};
}

}  // namespace

MinMaxGrayResult min_max_gray(const mv::core::Region& region, const mv::core::Image& image, double percent) {
  if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, "min_max_gray", "image must not be empty");
  if (image.channel_count() != 1U) throw mv::core::Exception(mv::core::Status::Unsupported, "min_max_gray", "single-channel image required");
  if (!std::isfinite(percent) || percent < 0.0 || percent > 50.0) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "min_max_gray", "percent must be in [0,50]");
  }
  const auto domain = region.intersect(image.domain());
  if(domain.empty())return {0,0,0};
  return dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    return min_max_impl<T>(image, domain, percent);
  });
}

MinMaxGrayTuples min_max_gray(const mv::core::Regions& regions,
                              const mv::core::IconicTuple& images,
                              double percent) {
  const auto image_values = images.images("min_max_gray");
  MinMaxGrayTuples output;
  if (regions.empty() || image_values.empty()) return output;
  const auto plan = mv::core::BroadcastPlan::make(
      {{regions.size(), "Region", true},
       {image_values.size(), "Image", true}}, "min_max_gray");
  for (std::size_t index = 0; index < plan.size(); ++index) {
    const auto value = min_max_gray(regions[plan.source_index(0, index)],
                                    image_values[plan.source_index(1, index)],
                                    percent);
    output.min.append(value.min);
    output.max.append(value.max);
    output.range.append(value.range);
  }
  return output;
}

}  // namespace mv::imageproc
