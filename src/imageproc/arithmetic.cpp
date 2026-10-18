#include <imageproc/operators.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#if defined(_M_X64) || defined(__x86_64__)
#include <emmintrin.h>
#endif

#include <core/exception.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include "arithmetic_output.hpp"
#if MVKIT_FILTER_HAS_AVX2
#include "arithmetic_avx2.hpp"
#endif

namespace mv::imageproc {
namespace {

template <typename T, typename Op>
void unary_channel(const mv::core::Image& image, mv::core::Image& output,
                   std::size_t channel, Op&& op) {
  const ImageView src{image, channel};
  MutableImageView dst{output, channel};
  const auto domain = image.domain();
  const auto runs = domain.runs();
  const auto work = ExecutionContext::saturated_work(image.width(), image.height());
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          const auto row = static_cast<std::size_t>(run.row);
          const auto* in = src.row_as<T>(row);
          auto* out = dst.row_as<T>(row);
          for (std::int64_t c_i = run.col_begin;; ++c_i) {
            const auto c = static_cast<std::size_t>(c_i);
            out[c] = saturate_cast<T>(op(static_cast<double>(in[c])));
            if (c_i == run.col_end) break;
          }
        }
      });
}

#if MVKIT_FILTER_HAS_AVX2
CertifiedByteAffine certify_byte_affine(const std::array<std::uint8_t,256>& table,double mult) {
  CertifiedByteAffine result;
  // Q8 系数只用于产生候选。可行偏移由完整查表约束求交，最后验证实际饱和指令的全部输入。
  const auto scaled=std::round(std::abs(mult)*256.);
  if(!(scaled>=1. && scaled<=65535.))return result;
  const auto factor=static_cast<std::int64_t>(scaled);
  std::int64_t lower=-65535,upper=65535;
  for(std::int64_t x=0;x<256;++x) {
    const auto index=mult<0?255-x:x;
    const auto value=table[static_cast<std::size_t>(index)];
    if(value>0)lower=std::max(lower,std::int64_t(value)*256-factor*x);
    if(value<255)upper=std::min(upper,(std::int64_t(value)+1)*256-1-factor*x);
  }
  if(lower>upper)return result;
  const auto bias=lower+(upper-lower)/2;
  const auto limit=65535/factor;
  for(std::int64_t x=0;x<256;++x) {
    const auto input=mult<0?255-x:x;
    const auto value=std::clamp(std::min(input,limit)*factor+bias,std::int64_t{0},std::int64_t{65535})/256;
    if(value!=table[static_cast<std::size_t>(x)])return result;
  }
  result.factor=static_cast<std::uint16_t>(factor);
  result.offset=static_cast<std::uint16_t>(std::abs(bias));
  result.limit=static_cast<std::uint16_t>(limit);
  result.subtract=bias<0;result.invert=mult<0;result.valid=true;
  return result;
}
#endif

void scale_channel_u8(const mv::core::Image& image, mv::core::Image& output,
                      std::size_t channel,
                      const std::array<std::uint8_t, 256>& lookup,
                      double mult, double add, bool fused_zero = false) {
#if MVKIT_FILTER_HAS_AVX2
  const auto& lookup_cpu = mv::runtime::cpu_features();
  const bool vector_lookup = mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      lookup_cpu.selection(), lookup_cpu.hardware()), {false, true, false, false});
  // 原 SSE 整数路径已是精确乘加，不为该路径重复认证。
  const bool legacy_integer=std::trunc(mult)==mult && std::trunc(add)==add &&
                            std::abs(mult)*255.+std::abs(add)<=32767.;
  const auto affine=vector_lookup&&!legacy_integer?certify_byte_affine(lookup,mult):CertifiedByteAffine{};
  std::array<std::int32_t, 256> lookup32;
  if (vector_lookup && !affine.valid) for (std::size_t i = 0; i < lookup.size(); ++i) lookup32[i] = lookup[i];
#endif
  const auto apply_lookup = [&](const std::uint8_t* in, std::uint8_t* out,
                                std::size_t begin, std::size_t end) {
    auto c = begin;
#if MVKIT_FILTER_HAS_AVX2
    if (affine.valid) c = certified_affine_u8_row_avx2(in,out,c,end,affine);
    else if (vector_lookup) c = lookup_u8_row_avx2(in, out, c, end, lookup32.data());
#endif
    for (; c < end; ++c) out[c] = lookup[in[c]];
  };
#if defined(_M_X64) || defined(__x86_64__)
  ExecutionContext execution;
  const auto& cpu=execution.runtime().cpu();
  const bool simd=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{true,false,false,false});
#endif
  if (!image.has_reduced_domain()) {
    const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
    auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
    const auto source_stride = image.stride(channel);
    const auto destination_stride = output.stride(channel);
    const auto width = image.width();
    const auto work = ExecutionContext::saturated_work(width, image.height());
    // 缓存内 UInt8 仿射/LUT 遍历过短，无需唤醒全部 worker；大图仍自适应划分以平衡带宽和尾部。
    const std::size_t partition_budget =
        work < 4 * ExecutionContext::kArithmeticParallelWork ? 8 : 0;
#if defined(_M_X64) || defined(__x86_64__)
    const bool integer_affine = simd && std::trunc(mult) == mult &&
                                std::trunc(add) == add &&
                                std::abs(mult) * 255.0 + std::abs(add) <= 32767.0;
    if (integer_affine) {
      const auto factor = _mm_set1_epi16(static_cast<std::int16_t>(mult));
      const auto offset = _mm_set1_epi16(static_cast<std::int16_t>(add));
      const auto zero = _mm_setzero_si128();
      ExecutionContext::for_each_partition(
          image.height(), work, ExecutionContext::kArithmeticParallelWork,
          [&](mv::runtime::IndexRange range) {
            for (std::size_t row = range.begin; row < range.end; ++row) {
              const auto* in = source + row * source_stride;
              auto* out = destination + row * destination_stride;
              std::size_t column = 0;
              for (; column + 16 <= width; column += 16) {
                const auto bytes = _mm_loadu_si128(
                    reinterpret_cast<const __m128i*>(in + column));
                const auto low = _mm_add_epi16(
                    _mm_mullo_epi16(_mm_unpacklo_epi8(bytes, zero), factor), offset);
                const auto high = _mm_add_epi16(
                    _mm_mullo_epi16(_mm_unpackhi_epi8(bytes, zero), factor), offset);
                _mm_storeu_si128(reinterpret_cast<__m128i*>(out + column),
                                 _mm_packus_epi16(low, high));
              }
              for (; column < width; ++column)
                out[column] = lookup[in[column]];
            }
          }, partition_budget);
      return;
    }
#endif
    ExecutionContext::for_each_partition(
        image.height(), work, ExecutionContext::kArithmeticParallelWork,
        [&](mv::runtime::IndexRange range) {
          for (std::size_t row = range.begin; row < range.end; ++row) {
            const auto* in = source + row * source_stride;
            auto* out = destination + row * destination_stride;
            apply_lookup(in, out, 0, width);
          }
        }, partition_budget);
    return;
  }
  const ImageView src{image, channel};
  MutableImageView dst{output, channel};
  const auto domain = image.domain();
  const auto runs = domain.runs();
  const auto work = ExecutionContext::saturated_work(image.width(), image.height());
  if (fused_zero) {
    // 输出未初始化；一次遍历有序定义域，同时写有效跨度及其间空隙，避免像素二次遍历。
    const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
    auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
    const auto source_stride = image.stride(channel);
    const auto destination_stride = output.stride(channel);
#if defined(_M_X64) || defined(__x86_64__)
    const bool integer_affine = simd && std::trunc(mult) == mult &&
                                std::trunc(add) == add &&
                                std::abs(mult) * 255.0 + std::abs(add) <= 32767.0;
    const auto factor = _mm_set1_epi16(integer_affine ? static_cast<std::int16_t>(mult) : 0);
    const auto offset = _mm_set1_epi16(integer_affine ? static_cast<std::int16_t>(add) : 0);
    const auto zero = _mm_setzero_si128();
#endif
    std::size_t run_index = 0;
    for (std::size_t row = 0; row < image.height(); ++row) {
      const auto* in = source + row * source_stride;
      auto* out = destination + row * destination_stride;
      std::size_t cursor = 0;
      while (run_index < runs.size() && static_cast<std::size_t>(runs[run_index].row) == row) {
        const auto& run = runs[run_index++];
        const auto begin = static_cast<std::size_t>(run.col_begin);
        const auto end = static_cast<std::size_t>(run.col_end) + 1;
        if (begin > cursor) std::memset(out + cursor, 0, begin - cursor);
        std::size_t column = begin;
#if defined(_M_X64) || defined(__x86_64__)
        if (integer_affine) {
          for (; column + 16 <= end; column += 16) {
            const auto bytes = _mm_loadu_si128(
                reinterpret_cast<const __m128i*>(in + column));
            const auto low = _mm_add_epi16(
                _mm_mullo_epi16(_mm_unpacklo_epi8(bytes, zero), factor), offset);
            const auto high = _mm_add_epi16(
                _mm_mullo_epi16(_mm_unpackhi_epi8(bytes, zero), factor), offset);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(out + column),
                             _mm_packus_epi16(low, high));
          }
        }
#endif
        apply_lookup(in, out, column, end);
        cursor = end;
      }
      if (cursor < image.width()) std::memset(out + cursor, 0, image.width() - cursor);
    }
    return;
  }
#if defined(_M_X64) || defined(__x86_64__)
  const bool integer_affine = simd && std::trunc(mult) == mult &&
                              std::trunc(add) == add &&
                              std::abs(mult) * 255.0 + std::abs(add) <= 32767.0;
  if (integer_affine) {
    const auto factor = _mm_set1_epi16(static_cast<std::int16_t>(mult));
    const auto offset = _mm_set1_epi16(static_cast<std::int16_t>(add));
    const auto zero = _mm_setzero_si128();
    ExecutionContext::for_each_partition(
        runs.size(), work, ExecutionContext::kArithmeticParallelWork,
        [&](mv::runtime::IndexRange range) {
          for (std::size_t index = range.begin; index < range.end; ++index) {
            const auto& run = runs[index];
            const auto row = static_cast<std::size_t>(run.row);
            const auto* in = src.row_as<std::uint8_t>(row);
            auto* out = dst.row_as<std::uint8_t>(row);
            const auto end = static_cast<std::size_t>(run.col_end) + 1;
            std::size_t column = static_cast<std::size_t>(run.col_begin);
            for (; column + 16 <= end; column += 16) {
              const auto bytes = _mm_loadu_si128(
                  reinterpret_cast<const __m128i*>(in + column));
              const auto low = _mm_add_epi16(
                  _mm_mullo_epi16(_mm_unpacklo_epi8(bytes, zero), factor), offset);
              const auto high = _mm_add_epi16(
                  _mm_mullo_epi16(_mm_unpackhi_epi8(bytes, zero), factor), offset);
              _mm_storeu_si128(reinterpret_cast<__m128i*>(out + column),
                               _mm_packus_epi16(low, high));
            }
            for (; column < end; ++column)
              out[column] = lookup[in[column]];
          }
        });
    return;
  }
#endif
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          const auto row = static_cast<std::size_t>(run.row);
          const auto* in = src.row_as<std::uint8_t>(row);
          auto* out = dst.row_as<std::uint8_t>(row);
          apply_lookup(in, out, static_cast<std::size_t>(run.col_begin),
                       static_cast<std::size_t>(run.col_end) + 1);
        }
      });
}

template <typename T, typename Transform>
void scale_channel_typed(const mv::core::Image& image, mv::core::Image& output,
                         std::size_t channel, Transform transform) {
  const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
  auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
  const auto source_stride = image.stride(channel);
  const auto destination_stride = output.stride(channel);
  const auto work = ExecutionContext::saturated_work(image.width(), image.height());
  if (!image.has_reduced_domain()) {
    ExecutionContext::for_each_partition(
        image.height(), work, ExecutionContext::kArithmeticParallelWork,
        [&](mv::runtime::IndexRange range) {
          for (std::size_t row = range.begin; row < range.end; ++row) {
            const auto* in = reinterpret_cast<const T*>(source + row * source_stride);
            auto* out = reinterpret_cast<T*>(destination + row * destination_stride);
            for (std::size_t column = 0; column < image.width(); ++column)
              out[column] = transform(in[column]);
          }
        });
    return;
  }
  const auto runs = image.domain().runs();
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          const auto row = static_cast<std::size_t>(run.row);
          const auto* in = reinterpret_cast<const T*>(source + row * source_stride);
          auto* out = reinterpret_cast<T*>(destination + row * destination_stride);
          for (auto column = static_cast<std::size_t>(run.col_begin);
               column <= static_cast<std::size_t>(run.col_end); ++column)
            out[column] = transform(in[column]);
        }
      });
}

#if defined(_M_X64) || defined(__x86_64__)
void scale_channel_u16_positive(const mv::core::Image& image, mv::core::Image& output,
                                std::size_t channel, unsigned factor, unsigned offset) {
  const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
  auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
  const auto source_stride = image.stride(channel);
  const auto destination_stride = output.stride(channel);
  const auto addition = _mm_set1_epi16(static_cast<std::int16_t>(offset));
  const auto work = ExecutionContext::saturated_work(image.width(), image.height());
  auto span = [&](std::size_t row, std::size_t begin, std::size_t end) {
    const auto* in = reinterpret_cast<const std::uint16_t*>(source + row * source_stride);
    auto* out = reinterpret_cast<std::uint16_t*>(destination + row * destination_stride);
    std::size_t column = begin;
    for (; column + 8 <= end; column += 8) {
      const auto pixels = _mm_loadu_si128(reinterpret_cast<const __m128i*>(in + column));
      auto scaled = _mm_setzero_si128();
      for (unsigned i = 0; i < factor; ++i) scaled = _mm_adds_epu16(scaled, pixels);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(out + column),
                       _mm_adds_epu16(scaled, addition));
    }
    for (; column < end; ++column)
      out[column] = static_cast<std::uint16_t>((std::min)(65535u,
          factor * static_cast<unsigned>(in[column]) + offset));
  };
  if (!image.has_reduced_domain()) {
    ExecutionContext::for_each_partition(
        image.height(), work, ExecutionContext::kArithmeticParallelWork,
        [&](mv::runtime::IndexRange range) {
          for (std::size_t row = range.begin; row < range.end; ++row)
            span(row, 0, image.width());
        });
    return;
  }
  const auto runs = image.domain().runs();
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          span(static_cast<std::size_t>(run.row),
               static_cast<std::size_t>(run.col_begin),
               static_cast<std::size_t>(run.col_end) + 1);
        }
      });
}

void scale_channel_f32_exact_factors(const mv::core::Image& image,
                                     mv::core::Image& output, std::size_t channel,
                                     float factor, float offset) {
  const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
  auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
  const auto source_stride = image.stride(channel);
  const auto destination_stride = output.stride(channel);
  const auto factors = _mm_set1_ps(factor);
  const auto offsets = _mm_set1_ps(offset);
  const auto work = ExecutionContext::saturated_work(image.width(), image.height());
  auto span = [&](std::size_t row, std::size_t begin, std::size_t end) {
    const auto* in = reinterpret_cast<const float*>(source + row * source_stride);
    auto* out = reinterpret_cast<float*>(destination + row * destination_stride);
    std::size_t column = begin;
    for (; column + 4 <= end; column += 4) {
      const auto pixels = _mm_loadu_ps(in + column);
      _mm_storeu_ps(out + column, _mm_add_ps(_mm_mul_ps(pixels, factors), offsets));
    }
    for (; column < end; ++column) out[column] = in[column] * factor + offset;
  };
  if (!image.has_reduced_domain()) {
    ExecutionContext::for_each_partition(
        image.height(), work, ExecutionContext::kArithmeticParallelWork,
        [&](mv::runtime::IndexRange range) {
          for (std::size_t row = range.begin; row < range.end; ++row)
            span(row, 0, image.width());
        });
    return;
  }
  const auto runs = image.domain().runs();
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          span(static_cast<std::size_t>(run.row),
               static_cast<std::size_t>(run.col_begin),
               static_cast<std::size_t>(run.col_end) + 1);
        }
      });
}
#endif

template <typename T, typename Op>
void binary_channel(const mv::core::Image& a, const mv::core::Image& b,
                    mv::core::Image& output, std::size_t channel, Op&& op) {
  const auto domain = a.domain().intersect(b.domain());
  const ImageView av{a, channel};
  const ImageView bv{b, channel};
  MutableImageView dst{output, channel};
  const auto runs = domain.runs();
  const auto work = ExecutionContext::saturated_work(a.width(), a.height());
  ExecutionContext::for_each_partition(
      runs.size(), work, ExecutionContext::kArithmeticParallelWork,
      [&](mv::runtime::IndexRange range) {
        for (std::size_t index = range.begin; index < range.end; ++index) {
          const auto& run = runs[index];
          const auto row = static_cast<std::size_t>(run.row);
          const auto* ar = av.row_as<T>(row);
          const auto* br = bv.row_as<T>(row);
          auto* out = dst.row_as<T>(row);
          for (std::int64_t c_i = run.col_begin;; ++c_i) {
            const auto c = static_cast<std::size_t>(c_i);
            out[c] = saturate_cast<T>(op(static_cast<double>(ar[c]),
                                         static_cast<double>(br[c])));
            if (c_i == run.col_end) break;
          }
        }
      });
}

void validate_factors(double mult, double add, const char* where) {
  if (!std::isfinite(mult) || !std::isfinite(add)) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "mult and add must be finite");
  }
}

void validate_binary(const mv::core::Image& a, const mv::core::Image& b, const char* where) {
  if (a.empty() || b.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "images must not be empty");
  if (a.width() != b.width() || a.height() != b.height() ||
      a.channel_count() != b.channel_count()) {
    throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "images must have identical size and channel count");
  }
  for (std::size_t channel = 0; channel < a.channel_count(); ++channel) {
    if (a.pixel_type(channel) != b.pixel_type(channel)) {
      throw mv::core::Exception(mv::core::Status::InvalidArgument, where, "corresponding channels must have identical pixel types");
    }
  }
}

template <typename Op>
mv::core::Image unary_dispatch(const mv::core::Image& image, Op&& op) {
  if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, "image arithmetic", "image must not be empty");
  auto output = make_output_like(image);
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
      using T = typename decltype(tag)::type;
      unary_channel<T>(image, output, channel, op);
    });
  }
  return output;
}

enum class BinaryKind { Add, Subtract, Multiply };

template <BinaryKind Kind, bool Halcon = false>
void binary_u8(const mv::core::Image& a, const mv::core::Image& b,
               mv::core::Image& output, std::size_t channel, double mult, double add) {
  constexpr int count = Kind == BinaryKind::Multiply ? 65026 : 511;
  constexpr int bias = Kind == BinaryKind::Subtract ? 255 : 0;
  bool vector_multiply=false;
#if MVKIT_FILTER_HAS_AVX2
  ExecutionContext avx_context;
  const auto& avx_cpu=avx_context.runtime().cpu();
  vector_multiply=Kind==BinaryKind::Multiply && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(avx_cpu.selection(),avx_cpu.hardware()),
      {false,true,false,false});
#endif
  std::array<std::uint8_t, count> lookup;
  const bool halcon_float=Halcon&&!(std::trunc(mult*2)==mult*2&&
      std::abs(mult)*510+std::abs(std::round(add))<4194304)&&
      std::isfinite(static_cast<float>(mult))&&std::isfinite(static_cast<float>(std::round(add)));
  if(!vector_multiply) for (int i = 0; i < count; ++i)
    lookup[i] = saturate_cast<std::uint8_t>(halcon_float
        ? double(std::fma(static_cast<float>(i-bias),static_cast<float>(mult),static_cast<float>(std::round(add))))
        : (i - bias) * mult + add);
  const auto domain = output.domain();
  const auto runs = domain.runs();
  const ImageView av{a, channel}, bv{b, channel};
  MutableImageView dst{output, channel};
#if defined(_M_X64) || defined(__x86_64__)
  ExecutionContext context;
  const auto& cpu = context.runtime().cpu();
  const bool simd = mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(), cpu.hardware()), {true, false, false, false});
  const bool half_integer = Kind != BinaryKind::Multiply && simd &&
      std::trunc(mult * 2) == mult * 2 && std::trunc(add * 2) == add * 2 &&
      std::abs(mult * 2) * 510 + std::abs(add * 2) + 1 <= 32767;
#endif
  ExecutionContext::for_each_partition(runs.size(), static_cast<std::size_t>(domain.area()),
      ExecutionContext::kArithmeticParallelWork, [&](auto range) {
    for (auto i = range.begin; i < range.end; ++i) {
      const auto& run = runs[i];
      const auto* x = av.row_as<std::uint8_t>(run.row);
      const auto* y = bv.row_as<std::uint8_t>(run.row);
      auto* out = dst.row_as<std::uint8_t>(run.row);
      auto c = static_cast<std::size_t>(run.col_begin);
      const auto end = static_cast<std::size_t>(run.col_end) + 1;
#if MVKIT_FILTER_HAS_AVX2
      if(vector_multiply) {
        c=multiply_u8_row_avx2(x,y,out,c,end,mult,add);
        for(;c<end;++c)out[c]=saturate_cast<std::uint8_t>(double(x[c]*y[c])*mult+add);
        continue;
      }
#endif
#if defined(_M_X64) || defined(__x86_64__)
      if (half_integer) {
        const auto zero = _mm_setzero_si128();
        const auto factor = _mm_set1_epi16(static_cast<short>(mult * 2));
        const auto offset = _mm_set1_epi16(static_cast<short>(add * 2 + 1));
        // 既有半整数界证明这些有符号 16 位和不会溢出。
        if (mult == 1. && std::trunc(add) == add) {
          const auto integer_offset = _mm_set1_epi16(static_cast<short>(add));
          for (; c + 16 <= end; c += 16) {
            const auto a8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(x+c));
            const auto b8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(y+c));
            const auto lo = Kind == BinaryKind::Add
                ? _mm_add_epi16(_mm_unpacklo_epi8(a8,zero),_mm_unpacklo_epi8(b8,zero))
                : _mm_sub_epi16(_mm_unpacklo_epi8(a8,zero),_mm_unpacklo_epi8(b8,zero));
            const auto hi = Kind == BinaryKind::Add
                ? _mm_add_epi16(_mm_unpackhi_epi8(a8,zero),_mm_unpackhi_epi8(b8,zero))
                : _mm_sub_epi16(_mm_unpackhi_epi8(a8,zero),_mm_unpackhi_epi8(b8,zero));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),
                _mm_packus_epi16(_mm_add_epi16(lo,integer_offset),_mm_add_epi16(hi,integer_offset)));
          }
        } else if (Kind == BinaryKind::Add && mult == .5 &&
                   std::trunc(add) == add && add >= 0. && add <= 255.) {
          // 无符号平均对非负半和向上取整，与 llround 一致。
          const auto byte_offset = _mm_set1_epi8(static_cast<char>(static_cast<unsigned char>(add)));
          for (; c + 16 <= end; c += 16) {
            const auto a8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(x+c));
            const auto b8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(y+c));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),
                _mm_adds_epu8(_mm_avg_epu8(a8,b8),byte_offset));
          }
        } else {
        for (; c + 16 <= end; c += 16) {
          const auto a8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(x+c));
          const auto b8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(y+c));
          auto lo = Kind == BinaryKind::Add
              ? _mm_add_epi16(_mm_unpacklo_epi8(a8,zero),_mm_unpacklo_epi8(b8,zero))
              : _mm_sub_epi16(_mm_unpacklo_epi8(a8,zero),_mm_unpacklo_epi8(b8,zero));
          auto hi = Kind == BinaryKind::Add
              ? _mm_add_epi16(_mm_unpackhi_epi8(a8,zero),_mm_unpackhi_epi8(b8,zero))
              : _mm_sub_epi16(_mm_unpackhi_epi8(a8,zero),_mm_unpackhi_epi8(b8,zero));
          lo = _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(lo,factor),offset),1);
          hi = _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(hi,factor),offset),1);
          _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(lo,hi));
        }
        }
      }
#endif
      for (; c < end; ++c) {
        const int value = Kind == BinaryKind::Add ? x[c]+y[c]
            : Kind == BinaryKind::Subtract ? x[c]-y[c] : x[c]*y[c];
        out[c] = lookup[value+bias];
      }
    }
  });
}

template <BinaryKind Kind, typename Op>
mv::core::Image binary_dispatch(const mv::core::Image& a, const mv::core::Image& b,
                                double mult, double add, Op&& op, const char* where) {
  validate_binary(a, b, where);
  auto output = make_arithmetic_output(a, b);
  for (std::size_t channel = 0; channel < a.channel_count(); ++channel) {
#if MVKIT_FILTER_HAS_AVX2
    if constexpr(Kind==BinaryKind::Multiply) {
      ExecutionContext context;
      const auto& cpu=context.runtime().cpu();
      if(a.pixel_type(channel)==mv::core::PixelType::UInt16 &&
          mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
              cpu.selection(),cpu.hardware()),{false,true,false,false})) {
        const auto domain=output.domain();const auto runs=domain.runs();
        const ImageView av{a,channel},bv{b,channel};MutableImageView dst{output,channel};
        ExecutionContext::for_each_partition(runs.size(),static_cast<std::size_t>(domain.area()),
            ExecutionContext::kArithmeticParallelWork,[&](auto range) {
          for(auto i=range.begin;i<range.end;++i) {
            const auto& run=runs[i];const auto* x=av.row_as<std::uint16_t>(run.row);
            const auto* y=bv.row_as<std::uint16_t>(run.row);auto* out=dst.row_as<std::uint16_t>(run.row);
            const auto end=static_cast<std::size_t>(run.col_end)+1;
            auto c=multiply_u16_row_avx2(x,y,out,static_cast<std::size_t>(run.col_begin),end,mult,add);
            for(;c<end;++c) {
              std::uint16_t xx,yy;std::memcpy(&xx,x+c,2);std::memcpy(&yy,y+c,2);
              out[c]=saturate_cast<std::uint16_t>(op(double(xx),double(yy)));
            }
          }
        });
        continue;
      }
    }
    if(a.pixel_type(channel)==mv::core::PixelType::Float32) {
      ExecutionContext context;
      const auto& cpu=context.runtime().cpu();
      if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
          cpu.selection(),cpu.hardware()),{false,true,false,false})) {
        const auto domain=output.domain();const auto runs=domain.runs();
        const ImageView av{a,channel},bv{b,channel};MutableImageView dst{output,channel};
        ExecutionContext::for_each_partition(runs.size(),static_cast<std::size_t>(domain.area()),
            ExecutionContext::kArithmeticParallelWork,[&](auto range) {
          for(auto i=range.begin;i<range.end;++i) {
            const auto& run=runs[i];const auto* x=av.row_as<float>(run.row);
            const auto* y=bv.row_as<float>(run.row);auto* out=dst.row_as<float>(run.row);
            const auto end=static_cast<std::size_t>(run.col_end)+1;
            auto c=binary_f32_row_avx2(x,y,out,static_cast<std::size_t>(run.col_begin),end,
                mult,add,Kind==BinaryKind::Add?0:Kind==BinaryKind::Subtract?1:2);
            for(;c<end;++c)out[c]=static_cast<float>(op(double(x[c]),double(y[c])));
          }
        });
        continue;
      }
    }
    if constexpr(Kind!=BinaryKind::Multiply) {
      if(a.pixel_type(channel)==mv::core::PixelType::UInt16 &&
          std::trunc(mult*2)==mult*2 && std::trunc(add*2)==add*2 &&
          std::abs(mult*2)*131070.+std::abs(add*2)+1<=2147483647.) {
        ExecutionContext context;
        const auto& cpu=context.runtime().cpu();
        if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
            cpu.selection(),cpu.hardware()),{false,true,false,false})) {
          const auto domain=output.domain();
          const auto runs=domain.runs();
          const ImageView av{a,channel},bv{b,channel};
          MutableImageView dst{output,channel};
          ExecutionContext::for_each_partition(runs.size(),static_cast<std::size_t>(domain.area()),
              ExecutionContext::kArithmeticParallelWork,[&](auto range) {
            for(auto i=range.begin;i<range.end;++i) {
              const auto& run=runs[i];
              const auto* x=av.row_as<std::uint16_t>(run.row);
              const auto* y=bv.row_as<std::uint16_t>(run.row);
              auto* out=dst.row_as<std::uint16_t>(run.row);
              const auto end=static_cast<std::size_t>(run.col_end)+1;
              auto c=binary_u16_half_row_avx2(x,y,out,static_cast<std::size_t>(run.col_begin),end,
                  static_cast<int>(mult*2),static_cast<int>(add*2),Kind==BinaryKind::Subtract);
              for(;c<end;++c)out[c]=saturate_cast<std::uint16_t>(op(double(x[c]),double(y[c])));
            }
          });
          continue;
        }
      }
    }
#endif
    if (a.pixel_type(channel) == mv::core::PixelType::UInt8 &&
        a.width() * a.height() >= (Kind == BinaryKind::Multiply ? 65536 : 512)) {
      binary_u8<Kind>(a, b, output, channel, mult, add);
      continue;
    }
    dispatch_scalar(a.pixel_type(channel), [&](auto tag) {
      using T = typename decltype(tag)::type;
      binary_channel<T>(a, b, output, channel, op);
    });
  }
  return output;
}

}  // namespace

mv::core::Image scale_image(const mv::core::Image& image, double mult, double add) {
  validate_factors(mult, add, "scale_image");
  if (image.empty())
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              "image arithmetic", "image must not be empty");
  if (ExecutionContext::saturated_work(image.width(), image.height()) < 1024 ||
      (image.has_reduced_domain() && image.domain().area() < 1024))
    return unary_dispatch(image,
                          [=](double value) { return value * mult + add; });
  const bool fused_zero = image.has_reduced_domain() &&
                          image.channel_count() == 1 &&
                          image.pixel_type(0) == mv::core::PixelType::UInt8;
  auto output = image.has_reduced_domain() && !fused_zero
                    ? make_output_like(image)
                    : make_full_output_for_overwrite(image);
  if (fused_zero) output = output.reduce_domain(image.domain());
  std::array<std::uint8_t, 256> lookup;
  bool lookup_ready = false;
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    if (image.pixel_type(channel) == mv::core::PixelType::UInt8) {
      if (!lookup_ready) {
        for (std::size_t value = 0; value < lookup.size(); ++value)
          lookup[value] = saturate_cast<std::uint8_t>(
              static_cast<double>(value) * mult + add);
        lookup_ready = true;
      }
      scale_channel_u8(image, output, channel, lookup, mult, add, fused_zero);
    }
#if defined(_M_X64) || defined(__x86_64__)
    else if (image.pixel_type(channel) == mv::core::PixelType::UInt16 &&
             std::trunc(mult) == mult && std::trunc(add) == add &&
             mult >= 0.0 && mult <= 8.0 && add >= 0.0 && add <= 65535.0) {
      scale_channel_u16_positive(image, output, channel,
                                 static_cast<unsigned>(mult), static_cast<unsigned>(add));
    }
#endif
    else if (image.pixel_type(channel) == mv::core::PixelType::UInt16 &&
               std::trunc(mult) == mult && std::trunc(add) == add &&
               std::abs(mult) * 65535.0 + std::abs(add) <= 2147483647.0) {
      const auto factor = static_cast<std::int32_t>(mult);
      const auto offset = static_cast<std::int32_t>(add);
      scale_channel_typed<std::uint16_t>(image, output, channel,
          [=](std::uint16_t value) {
            const auto result = static_cast<std::int32_t>(value) * factor + offset;
            return static_cast<std::uint16_t>(std::clamp(result, 0, 65535));
          });
    } else if (image.pixel_type(channel) == mv::core::PixelType::Float32) {
      if (static_cast<double>(static_cast<float>(mult)) == mult &&
          static_cast<double>(static_cast<float>(add)) == add) {
        const auto factor = static_cast<float>(mult);
        const auto offset = static_cast<float>(add);
#if defined(_M_X64) || defined(__x86_64__)
        scale_channel_f32_exact_factors(image, output, channel, factor, offset);
#else
        scale_channel_typed<float>(image, output, channel,
            [=](float value) { return value * factor + offset; });
#endif
      } else {
        scale_channel_typed<float>(image, output, channel,
            [=](float value) { return static_cast<float>(static_cast<double>(value) * mult + add); });
      }
    } else {
      dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
        using T = typename decltype(tag)::type;
        unary_channel<T>(image, output, channel,
                         [=](double value) { return value * mult + add; });
      });
    }
  }
  return output;
}

namespace {
mv::core::Image add_mixed_or_rounded(const mv::core::Image& a,const mv::core::Image& b,
                                   double mult,double add,bool halcon) {
    if(a.empty()||b.empty()||a.width()!=b.width()||a.height()!=b.height()||a.channel_count()!=b.channel_count())
      throw mv::core::Exception(mv::core::Status::InvalidArgument,"add_image","images must have identical size");
    using P=mv::core::PixelType;
    std::vector<P> types;types.reserve(a.channel_count());
    for(std::size_t c=0;c<a.channel_count();++c) {
      auto x=a.pixel_type(c),y=b.pixel_type(c);
      if(x==y)types.push_back(x);
      else if(x==P::Float32||y==P::Float32||(x==P::Int16&&y==P::UInt16)||(x==P::UInt16&&y==P::Int16))
        throw mv::core::Exception(mv::core::Status::TypeMismatch,mv::core::ErrorCode::InvalidType,
            "add_image","Image2",c,"unsupported mixed pixel types");
      else if(x==P::Int32||y==P::Int32)types.push_back(P::Int32);
      else if(x==P::UInt16||y==P::UInt16)types.push_back(P::UInt16);
      else types.push_back(P::Int16);
    }
    const auto domain=a.domain().intersect(b.domain());
    auto out=mv::core::Image::create(a.width(),a.height(),types).reduce_domain(domain);
    const auto runs=domain.runs();
    for(std::size_t c=0;c<a.channel_count();++c) {
      const ImageView av{a,c},bv{b,c};MutableImageView dst{out,c};
      dispatch_scalar(a.pixel_type(c),[&](auto ax) {
        using X=typename decltype(ax)::type;
        dispatch_scalar(b.pixel_type(c),[&](auto by) {
          using Y=typename decltype(by)::type;
          dispatch_scalar(types[c],[&](auto oz) {
            using Z=typename decltype(oz)::type;
            ExecutionContext::for_each_partition(runs.size(),static_cast<std::size_t>(domain.area()),
                ExecutionContext::kArithmeticParallelWork,[&](auto range) {
              for(auto i=range.begin;i<range.end;++i) {
                const auto& run=runs[i];const auto* x=av.row_as<X>(run.row);
                const auto* y=bv.row_as<Y>(run.row);auto* z=dst.row_as<Z>(run.row);
                for(auto col=std::int64_t(run.col_begin);col<=run.col_end;++col)
                  {
                    const auto sum=double(x[col])+double(y[col]);
                    const auto product=halcon&&types[c]!=P::Int32&&types[c]!=P::Float32&&
                        std::isfinite(static_cast<float>(mult))
                        ? double(static_cast<float>(sum)*static_cast<float>(mult)) : sum*mult;
                    const auto value=halcon&&types[c]!=P::Float32
                        ? types[c]==P::Int32 ? product+std::round(add)
                            : double(std::fma(static_cast<float>(sum),static_cast<float>(mult),
                                              static_cast<float>(std::round(add))))
                        : product+add;
                    z[col]=saturate_cast<Z>(value);
                  }
              }
            });
          });
        });
      });
    }
    return out;
}
}

mv::core::Image add_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add) {
  validate_factors(mult,add,"add_image");
  bool mixed=false;
  if(!a.empty()&&!b.empty()&&a.channel_count()==b.channel_count())
    for(std::size_t c=0;c<a.channel_count();++c)mixed|=a.pixel_type(c)!=b.pixel_type(c);
  if(mixed)return add_mixed_or_rounded(a,b,mult,add,false);
  return binary_dispatch<BinaryKind::Add>(a,b,mult,add,[=](double x,double y){return (x+y)*mult+add;},"add_image");
}

mv::core::Image add_image_halcon(const mv::core::Image& a,const mv::core::Image& b,double mult,double add) {
  validate_factors(mult,add,"add_image");
  bool uniform=!a.empty()&&!b.empty()&&a.channel_count()==b.channel_count();
  if(uniform)for(std::size_t c=0;c<a.channel_count();++c)
    uniform&=a.pixel_type(c)==a.pixel_type(0)&&b.pixel_type(c)==a.pixel_type(0);
  if(uniform) {
    using P=mv::core::PixelType;
    if(a.pixel_type(0)==P::Float32)return add_image(a,b,mult,add);
    if(a.pixel_type(0)==P::UInt8) {
      validate_binary(a,b,"add_image");
      auto output=make_arithmetic_output(a,b);
      for(std::size_t c=0;c<a.channel_count();++c)
        binary_u8<BinaryKind::Add,true>(a,b,output,c,mult,std::round(add));
      return output;
    }
    const auto offset=std::round(add);
    const auto factor=a.pixel_type(0)!=P::Int32&&std::isfinite(static_cast<float>(mult))
        ? double(static_cast<float>(mult)) : mult;
    // HALCON 整数图像先取整 offset 再求和；平移取整值结果相同时保留快路径。
    // 一般窄整数控制采用单精度融合乘加，已由 HALCON 试点矩阵核对。
    const bool nonnegative=a.pixel_type(0)==P::UInt8||a.pixel_type(0)==P::UInt16;
    const bool exact_half=std::trunc(mult*2)==mult*2&&std::abs(mult)*131070+std::abs(offset)<4194304;
    if(factor==mult&&(exact_half||offset==0||std::trunc(mult)==mult||(nonnegative&&
        ((mult>=0&&offset>=0)||(mult<=0&&offset<=0)))))
      return add_image(a,b,factor,offset);
  }
  return add_mixed_or_rounded(a,b,mult,add,true);
}

mv::core::Image sub_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add) {
  validate_factors(mult, add, "sub_image");
  return binary_dispatch<BinaryKind::Subtract>(a, b, mult, add, [=](double x, double y) { return (x - y) * mult + add; }, "sub_image");
}

mv::core::Image mult_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add) {
  validate_factors(mult, add, "mult_image");
  return binary_dispatch<BinaryKind::Multiply>(a, b, mult, add, [=](double x, double y) { return (x * y) * mult + add; }, "mult_image");
}

}  // namespace mv::imageproc
