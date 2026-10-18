#include <imageproc/operators.hpp>

#include <cmath>
#include <algorithm>
#include <bit>
#include <cstddef>
#include <limits>
#include <memory_resource>
#include <string_view>
#include <vector>
#if defined(_M_X64) || defined(__x86_64__)
#include <emmintrin.h>
#endif

#include <core/broadcast.hpp>
#include <core/exception.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/execution_context.hpp>
#if MVKIT_FILTER_HAS_AVX2
#include "arithmetic_avx2.hpp"
#endif
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/pixel_traits.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::imageproc {
namespace {

template <typename Predicate>
mv::core::Region collect_region(const mv::core::Region& domain, Predicate&& predicate) {
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<mv::core::RegionRun> output{scratch.resource()};
  output.reserve(domain.runs().size());
  for (const auto& run : domain.runs()) {
    bool in_run = false;
    std::int64_t begin = 0;
    for (std::int64_t col = run.col_begin;; ++col) {
      const bool selected = predicate(run.row, col);
      if (selected && !in_run) {
        begin = col;
        in_run = true;
      }
      const bool last = col == run.col_end;
      if (in_run && (!selected || last)) {
        const auto end = selected && last ? col : col - 1;
        output.push_back({run.row, begin, end});
        in_run = false;
      }
      if (last) break;
    }
  }
  return mv::core::Region::from_runs(output);
}

template <typename T>
mv::core::Region threshold_t(const mv::core::Image& image, double min_gray, double max_gray) {
  const ImageView view{image};
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<mv::core::RegionRun> output{scratch.resource()};
  const auto domain = image.domain();
  ExecutionContext execution;
  const auto& cpu=execution.runtime().cpu();
  const auto capabilities=mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware());
#if MVKIT_FILTER_HAS_AVX2
  if constexpr(std::is_same_v<T,std::uint8_t> || std::is_same_v<T,std::uint16_t> || std::is_same_v<T,float>) {
    if(mv::runtime::kernel_allowed(capabilities,{false,true,false,false})) {
      std::size_t blocks=0;
      for(const auto& run:domain.runs())blocks+=static_cast<std::size_t>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+32)/32;
      if(!blocks)return {};
      // 使用前写完全部字，包括尾部 padding。
      auto* masks=std::pmr::polymorphic_allocator<std::uint32_t>{scratch.resource()}.allocate(blocks);
      std::size_t index=0;
      for(const auto& run:domain.runs()) {
        const auto* row=view.row_as<T>(run.row);
        const auto full=static_cast<std::size_t>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+1)/32;
        if(full)threshold_masks_avx2(row,static_cast<std::size_t>(run.col_begin),full,min_gray,max_gray,
            std::is_same_v<T,std::uint8_t>?0:std::is_same_v<T,std::uint16_t>?1:2,masks+index);
        index+=full;
        const auto tail=std::int64_t(run.col_begin)+static_cast<std::int64_t>(full*32);
        if(tail<=run.col_end) {
          std::uint32_t mask=0;
          for(auto c=tail;c<=run.col_end;++c)if(double(row[c])>=min_gray && double(row[c])<=max_gray)
            mask|=std::uint32_t{1}<<(c-tail);
          masks[index++]=mask;
        }
      }
      return mv::core::Region::from_row_masks(domain,{masks,blocks});
    }
  }
#endif
  output.reserve(domain.runs().size());
#if defined(_M_X64) || defined(__x86_64__)
  __m128i low{}, high{};
  __m128 fl{}, fh{};
  if constexpr (std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t>) {
    const double l = std::max(0.0, std::ceil(min_gray));
    const double h = std::min(double(std::numeric_limits<T>::max()), std::floor(max_gray));
    if (l > h) return {};
    if constexpr (sizeof(T) == 1) {
      low = _mm_set1_epi8(static_cast<char>(static_cast<unsigned>(l) ^ 128U));
      high = _mm_set1_epi8(static_cast<char>(static_cast<unsigned>(h) ^ 128U));
    } else {
      low = _mm_set1_epi16(static_cast<short>(static_cast<unsigned>(l) ^ 32768U));
      high = _mm_set1_epi16(static_cast<short>(static_cast<unsigned>(h) ^ 32768U));
    }
  } else if constexpr (std::is_same_v<T, float>) {
    float l = static_cast<float>(min_gray), h = static_cast<float>(max_gray);
    if (double(l) < min_gray) l = std::nextafter(l, std::numeric_limits<float>::infinity());
    if (double(h) > max_gray) h = std::nextafter(h, -std::numeric_limits<float>::infinity());
    fl = _mm_set1_ps(l); fh = _mm_set1_ps(h);
  }
#endif
  for (const auto& run : domain.runs()) {
    const auto* row = view.row_as<T>(static_cast<std::size_t>(run.row));
    auto c = std::int64_t(run.col_begin);
    std::int64_t begin = c;
    bool active = false;
    auto append = [&](std::int64_t first, std::int64_t last) {
      if (active && first == c) { /* 承接上一块的状态。 */ }
      else {
        if (active) output.push_back({run.row, begin, c - 1});
        begin = first;
      }
      active = true;
      c = last + 1;
    };
#if defined(_M_X64) || defined(__x86_64__)
    if constexpr (std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t> || std::is_same_v<T, float>) {
      constexpr int count = 16 / sizeof(T);
      while (mv::runtime::kernel_allowed(capabilities,{true,false,false,false}) && std::int64_t(run.col_end) - c + 1 >= count) {
        unsigned mask;
        if constexpr (std::is_same_v<T, float>) {
          const auto v = _mm_loadu_ps(row + c);
          mask = static_cast<unsigned>(_mm_movemask_ps(_mm_and_ps(_mm_cmpge_ps(v, fl), _mm_cmple_ps(v, fh))));
        } else {
          auto v = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row + c));
          __m128i outside;
          if constexpr (sizeof(T) == 1) {
            v = _mm_xor_si128(v, _mm_set1_epi8(static_cast<char>(128)));
            outside = _mm_or_si128(_mm_cmpgt_epi8(low, v), _mm_cmpgt_epi8(v, high));
            mask = (~static_cast<unsigned>(_mm_movemask_epi8(outside))) & 65535U;
          } else {
            v = _mm_xor_si128(v, _mm_set1_epi16(static_cast<short>(32768)));
            outside = _mm_or_si128(_mm_cmpgt_epi16(low, v), _mm_cmpgt_epi16(v, high));
            mask = (~static_cast<unsigned>(_mm_movemask_epi8(_mm_packs_epi16(outside, outside)))) & 255U;
          }
        }
        const auto block = c, end = block + count;
        while (mask) {
          const auto offset = std::countr_zero(mask);
          const unsigned shifted = mask >> offset;
          const auto length = std::countr_one(shifted);
          append(block + offset, block + offset + length - 1);
          mask &= ~(((1U << length) - 1) << offset);
        }
        if (active && c < end) { output.push_back({run.row, begin, c - 1}); active = false; }
        c = end;
      }
    }
#endif
    for (; c <= run.col_end; ++c) {
      const double v = static_cast<double>(row[c]);
      const bool selected = v >= min_gray && v <= max_gray;
      if (selected && !active) { begin = c; active = true; }
      if (!selected && active) { output.push_back({run.row, begin, c - 1}); active = false; }
    }
    if (active) output.push_back({run.row, begin, run.col_end});
  }
  return mv::core::Region::from_runs(output);
}

mv::core::Region dyn_threshold_byte(const mv::core::Image& a,const mv::core::Image& b,
                                   double offset,DynThresholdMode mode,const mv::core::Region& domain) {
  int low=-256,high=256;
  const auto bound=[](double v) { return static_cast<int>(std::clamp(v,-256.,256.)); };
  switch(mode) {
    case DynThresholdMode::Light: low=bound(offset);break;
    case DynThresholdMode::Dark: high=bound(-offset);break;
    case DynThresholdMode::Equal:
    case DynThresholdMode::NotEqual: low=bound(-offset);high=bound(offset);break;
    default:return {};
  }
  const bool inverted=mode==DynThresholdMode::NotEqual;
  if(a.data()==b.data() && a.stride()==b.stride())
    return ((0>=low && 0<=high)!=inverted)?domain:mv::core::Region{};
  mv::runtime::ScratchArena scratch;
  const ImageView av{a},bv{b};
  ExecutionContext context;
  const auto& cpu=context.runtime().cpu();
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
  if(avx2) {
    std::size_t blocks_total=0;
    for(const auto& run:domain.runs())blocks_total+=static_cast<std::size_t>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+32)/32;
    if(!blocks_total)return {};
    auto* masks=std::pmr::polymorphic_allocator<std::uint32_t>{scratch.resource()}.allocate(blocks_total);
    std::size_t cursor=0;
    for(const auto& run:domain.runs()) {
      const auto* ar=av.row_as<std::uint8_t>(run.row);
      const auto* br=bv.row_as<std::uint8_t>(run.row);
      const auto blocks=static_cast<std::size_t>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+1)/32;
      if(blocks)dynamic_u8_masks_avx2(ar,br,static_cast<std::size_t>(run.col_begin),blocks,
                                    low,high,inverted,masks+cursor);
      cursor+=blocks;
      const auto tail=std::int64_t(run.col_begin)+static_cast<std::int64_t>(blocks*32);
      if(tail<=run.col_end) {
        std::uint32_t mask=0;
        for(auto c=tail;c<=run.col_end;++c) {
          const int difference=int(ar[c])-int(br[c]);
          if((difference>=low && difference<=high)!=inverted)mask|=std::uint32_t{1}<<(c-tail);
        }
        masks[cursor++]=mask;
      }
    }
    return mv::core::Region::from_row_masks(domain,{masks,blocks_total});
  }
#endif
  std::pmr::vector<mv::core::RegionRun> output{mv::runtime::persist_resource()};
  output.reserve(domain.runs().size());
#if defined(_M_X64) || defined(__x86_64__)
  const bool simd=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{true,false,false,false});
  const auto lo=_mm_set1_epi16(static_cast<short>(low));
  const auto hi=_mm_set1_epi16(static_cast<short>(high));
  const auto zero=_mm_setzero_si128();
#endif
  for(const auto& run:domain.runs()) {
    const auto* ar=av.row_as<std::uint8_t>(run.row);
    const auto* br=bv.row_as<std::uint8_t>(run.row);
    auto c=std::int64_t(run.col_begin),begin=c;
    bool active=false;
#if defined(_M_X64) || defined(__x86_64__)
    if(simd) while(std::int64_t(run.col_end)-c+1>=16) {
      const auto x=_mm_loadu_si128(reinterpret_cast<const __m128i*>(ar+c));
      const auto y=_mm_loadu_si128(reinterpret_cast<const __m128i*>(br+c));
      const auto d0=_mm_sub_epi16(_mm_unpacklo_epi8(x,zero),_mm_unpacklo_epi8(y,zero));
      const auto d1=_mm_sub_epi16(_mm_unpackhi_epi8(x,zero),_mm_unpackhi_epi8(y,zero));
      const auto outside0=_mm_or_si128(_mm_cmpgt_epi16(lo,d0),_mm_cmpgt_epi16(d0,hi));
      const auto outside1=_mm_or_si128(_mm_cmpgt_epi16(lo,d1),_mm_cmpgt_epi16(d1,hi));
      unsigned mask=static_cast<unsigned>(_mm_movemask_epi8(_mm_packs_epi16(outside0,outside1)));
      if(!inverted)mask=(~mask)&65535U;
      const auto block=c,end=block+16;
      while(mask) {
        const auto first=std::countr_zero(mask),length=std::countr_one(mask>>first);
        if(!active || block+first!=c) {
          if(active)output.push_back({run.row,begin,c-1});
          begin=block+first;
        }
        active=true;c=block+first+length;
        mask&=~(((1U<<length)-1)<<first);
      }
      if(active && c<end) { output.push_back({run.row,begin,c-1});active=false; }
      c=end;
    }
#endif
    for(;c<=run.col_end;++c) {
      const int difference=int(ar[c])-int(br[c]);
      const bool selected=(difference>=low && difference<=high)!=inverted;
      if(selected&&!active){begin=c;active=true;}
      if(!selected&&active){output.push_back({run.row,begin,c-1});active=false;}
    }
    if(active)output.push_back({run.row,begin,run.col_end});
  }
  return mv::core::Region::from_canonical_runs(std::move(output));
}

template <typename A, typename B>
mv::core::Region dyn_threshold_t(const mv::core::Image& orig,
                                  const mv::core::Image& threshold_image,
                                  double offset,
                                  DynThresholdMode mode,
                                  const mv::core::Region& domain) {
  if constexpr(std::is_same_v<A,std::uint8_t> && std::is_same_v<B,std::uint8_t>) {
    // 小数偏移保留原浮点加法语义，包含加到整数参考值后被舍去的微小偏移。
    if(std::trunc(offset)==offset)return dyn_threshold_byte(orig,threshold_image,offset,mode,domain);
  }
  const ImageView a{orig};
  const ImageView b{threshold_image};
  // 定义域按行遍历，缓存行地址，避免逐像素查询 Image 所有权、尺寸和 stride。
  std::int64_t previous = -1;
  const A* ar = nullptr;
  const B* br = nullptr;
  return collect_region(domain, [&](std::int64_t row, std::int64_t col) {
    if (row != previous) {
      ar = a.row_as<A>(static_cast<std::size_t>(row));
      br = b.row_as<B>(static_cast<std::size_t>(row));
      previous = row;
    }
    const auto c = static_cast<std::size_t>(col);
    const double go = static_cast<double>(ar[c]);
    const double gt = static_cast<double>(br[c]);
    switch (mode) {
      case DynThresholdMode::Light: return go >= gt + offset;
      case DynThresholdMode::Dark: return go <= gt - offset;
      case DynThresholdMode::Equal: return go >= gt - offset && go <= gt + offset;
      case DynThresholdMode::NotEqual: return go < gt - offset || go > gt + offset;
    }
    return false;
  });
}

void validate_scalar_image(const mv::core::Image& image, const char* where) {
  if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "image must not be empty");
  if (image.channel_count() != 1U) {
    throw mv::core::Exception(mv::core::Status::Unsupported, where, "operator requires a single-channel image");
  }
}

double symbolic_limit(const mv::core::Image& image, std::string_view symbol,
                      const char* parameter, std::size_t index) {
  return dispatch_scalar(image.pixel_type(), [&](auto tag) -> double {
    using T = typename decltype(tag)::type;
    if (symbol == "min") return static_cast<double>((std::numeric_limits<T>::lowest)());
    if (symbol == "max") return static_cast<double>((std::numeric_limits<T>::max)());
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::InvalidValue,
                              "threshold", parameter,
                              static_cast<std::int64_t>(index),
                              "symbolic threshold value must be 'min' or 'max'");
  });
}

double threshold_value(const mv::core::Tuple& tuple, std::size_t index,
                       const mv::core::Image& image, const char* parameter) {
  const auto value = tuple.at(index);
  if (value.kind() == mv::core::ValueKind::Int64 ||
      value.kind() == mv::core::ValueKind::Real64) {
    return value.as_real64();
  }
  if (value.kind() == mv::core::ValueKind::String) {
    return symbolic_limit(image, value.as_string().view(), parameter, index);
  }
  {
    throw mv::core::Exception(mv::core::Status::TypeMismatch,
                              mv::core::ErrorCode::InvalidType,
                              "threshold", parameter,
                              static_cast<std::int64_t>(index),
                              "threshold value must be numeric, 'min', or 'max'");
  }
}

}  // namespace

mv::core::Region threshold(const mv::core::Image& image, double min_gray, double max_gray) {
  validate_scalar_image(image, "threshold");
  if (std::isnan(min_gray) || std::isnan(max_gray) || min_gray > max_gray) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "threshold", "invalid threshold interval");
  }
  return dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    return threshold_t<T>(image, min_gray, max_gray);
  });
}

mv::core::IconicTuple threshold(const mv::core::IconicTuple& images,
                                const mv::core::Tuple& min_gray,
                                const mv::core::Tuple& max_gray) {
  const auto image_values = images.images("threshold");
  if (image_values.empty()) return {};

  const auto plan = mv::core::BroadcastPlan::make(
      {{image_values.size(), "Image", false},
       {min_gray.size(), "MinGray", true},
       {max_gray.size(), "MaxGray", true}},
      "threshold");

  std::vector<mv::core::IconicValue> output;
  output.reserve(image_values.size());
  for (std::size_t index = 0; index < image_values.size(); ++index) {
    const auto minimum_index = plan.source_index(1, index);
    const auto maximum_index = plan.source_index(2, index);
    output.push_back(mv::core::IconicValue::from_region(
        threshold(image_values[index],
                  threshold_value(min_gray, minimum_index, image_values[index], "MinGray"),
                  threshold_value(max_gray, maximum_index, image_values[index], "MaxGray"))));
  }
  return mv::core::IconicTuple{std::move(output)};
}

mv::core::Region dyn_threshold(const mv::core::Image& orig,
                               const mv::core::Image& threshold_image,
                               double offset,
                               DynThresholdMode mode) {
  validate_scalar_image(orig, "dyn_threshold");
  validate_scalar_image(threshold_image, "dyn_threshold");
  if (orig.width() != threshold_image.width() || orig.height() != threshold_image.height()) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "dyn_threshold", "image sizes must match");
  }
  if (!std::isfinite(offset)) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "dyn_threshold", "offset must be finite");
  }
  const auto domain = orig.domain().intersect(threshold_image.domain());
  return dispatch_scalar(orig.pixel_type(), [&](auto a_tag) {
    using A = typename decltype(a_tag)::type;
    return dispatch_scalar(threshold_image.pixel_type(), [&](auto b_tag) {
      using B = typename decltype(b_tag)::type;
      return dyn_threshold_t<A, B>(orig, threshold_image, offset, mode, domain);
    });
  });
}

}  // namespace mv::imageproc
