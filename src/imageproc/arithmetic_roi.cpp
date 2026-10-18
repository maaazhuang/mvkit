#include <imageproc/operators.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory_resource>
#include <type_traits>
#include <vector>
#include <core/exception.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include "arithmetic_output.hpp"
#if MVKIT_FILTER_HAS_AVX2
#include "arithmetic_avx2.hpp"
#endif

#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_ROI_SSE2 1
#else
#define MVKIT_ROI_SSE2 0
#endif

namespace mv::imageproc {
namespace {
using namespace mv::core;
enum class Arithmetic { Difference, Divide, Invert };

void require_image(const Image& image, const char* op) {
  if (image.empty())
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    "Image", -1, "image must not be empty");
}

void require_controls(double mult, double add, const char* op) {
  if (!std::isfinite(mult) || !std::isfinite(add))
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    !std::isfinite(mult) ? "Mult" : "Add", -1,
                    "factor must be finite");
}

template <class T>
T read(const std::byte* row, std::size_t c) {
  T value;
  std::memcpy(&value, row + c * sizeof(T), sizeof(T));
  return value;
}

#if MVKIT_ROI_SSE2
// 每行只分派一次操作分支；各向量仍写独立样本。
template<class T,bool Invert>
std::size_t unsigned_row_sse2(const std::byte* a,const std::byte* b,T* target,std::size_t c,std::size_t end) {
  constexpr auto block=16/sizeof(T);
  const auto one=[&](std::size_t i) {
    const auto x=_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+i*sizeof(T)));
    __m128i result;
    if constexpr(Invert)result=_mm_xor_si128(x,_mm_set1_epi8(-1));
    else {
      const auto y=_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+i*sizeof(T)));
      if constexpr(sizeof(T)==1)result=_mm_or_si128(_mm_subs_epu8(x,y),_mm_subs_epu8(y,x));
      else result=_mm_or_si128(_mm_subs_epu16(x,y),_mm_subs_epu16(y,x));
    }
    _mm_storeu_si128(reinterpret_cast<__m128i*>(target+i),result);
  };
  for(;end-c>=4*block;c+=4*block){one(c);one(c+block);one(c+2*block);one(c+3*block);}
  for(;end-c>=block;c+=block)one(c);
  return c;
}
#endif

Image arithmetic(const Image& a, const Image& b, Arithmetic mode, double mult,
                 double add, const char* op) {
  require_controls(mult, add, op);
  require_image(a, op);
  require_image(b, op);
  if (a.width() != b.width() || a.height() != b.height() ||
      a.channel_count() != b.channel_count())
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    "Image2", -1,
                    "images must have identical dimensions and channel count");
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch)
    if (a.pixel_type(ch) != b.pixel_type(ch))
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                      "Image2", -1, "corresponding channel types must match");
  auto out = make_arithmetic_output(a, b);
  const auto domain = out.domain();
  const auto runs = domain.runs();
  // HALCON byte 商在非负 scale 及整数 byte offset 时采用原序 float 算术；
  // 其他控制（尤其小数 offset、有符号算术）保持通用 Double 契约。
  const bool byte_quotient = mode == Arithmetic::Divide && mult >= 0 &&
      mult <= double(std::numeric_limits<float>::max()) / 255. &&
      add >= 0 && add <= 255 && std::trunc(add) == add;
  ExecutionContext context;
  const auto& cpu = context.runtime().cpu();
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif
  const bool simd = MVKIT_ROI_SSE2 && mv::runtime::kernel_allowed(
                                          mv::runtime::effective_capabilities(
                                              cpu.selection(), cpu.hardware()),
                                          {true, false, false, false});
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
    const auto* ap = static_cast<const std::byte*>(a.data(ch));
    const auto* bp = static_cast<const std::byte*>(b.data(ch));
    auto* dst = static_cast<std::byte*>(out.mutable_data(ch));
    dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(
          runs.size(), static_cast<std::size_t>(domain.area()),
          ExecutionContext::kArithmeticParallelWork, [&](auto range) {
            for (auto i = range.begin; i < range.end; ++i) {
              const auto& run = runs[i];
              const auto row = static_cast<std::size_t>(run.row);
              const auto* ar = ap + row * a.stride(ch);
              const auto* br = bp + row * b.stride(ch);
              auto* target = reinterpret_cast<T*>(dst + row * out.stride(ch));
              auto c = static_cast<std::size_t>(run.col_begin);
              const auto end = static_cast<std::size_t>(run.col_end) + 1;
#if MVKIT_FILTER_HAS_AVX2
              if constexpr(std::is_same_v<T,std::uint16_t>) {
                if(avx2 && simd && end-c>=8 && byte_quotient && add==0 &&
                    mult<=double(std::numeric_limits<float>::max())/65535.)
                  c=divide_u16_row_avx2(
                      reinterpret_cast<const std::uint16_t*>(ar),
                      reinterpret_cast<const std::uint16_t*>(br),target,c,end,mult,add);
              }
              if constexpr(std::is_same_v<T,float>) {
                if(avx2)
                  c=binary_f32_row_avx2(reinterpret_cast<const float*>(ar),
                      reinterpret_cast<const float*>(br),target,c,end,mult,add,
                      mode==Arithmetic::Invert?4:mode==Arithmetic::Divide?5:3);
              }
              if constexpr(std::is_same_v<T,std::uint8_t>) {
                if(avx2 && byte_quotient)
                  c=divide_u8_row_avx2(reinterpret_cast<const std::uint8_t*>(ar),
                      reinterpret_cast<const std::uint8_t*>(br),target,c,end,mult,add);
              }
#endif
#if MVKIT_ROI_SSE2
              if constexpr (std::is_same_v<T, std::uint16_t>) {
                if (simd && byte_quotient && add == 0 &&
                    mult <= double(std::numeric_limits<float>::max()) / 65535.) {
                  const auto scale=_mm_set1_ps(float(mult)),bias=_mm_set1_ps(float(add+.5));
                  const auto zero_i = _mm_setzero_si128();
                  const auto offset = _mm_set1_epi32(32768);
                  const auto divide = [&](const __m128i x_i, const __m128i y_i) {
                    const auto x = _mm_cvtepi32_ps(x_i), y = _mm_cvtepi32_ps(y_i);
                    const auto zero = _mm_cmpeq_ps(y, _mm_setzero_ps());
                    const auto safe = _mm_or_ps(_mm_andnot_ps(zero,y), _mm_and_ps(zero,_mm_set1_ps(1)));
                    const auto initial = _mm_rcp_ps(safe);
                    const auto reciprocal = _mm_mul_ps(initial,_mm_sub_ps(_mm_set1_ps(2),_mm_mul_ps(safe,initial)));
                    auto value = _mm_add_ps(_mm_mul_ps(_mm_mul_ps(x,reciprocal),scale),bias);
                    value = _mm_andnot_ps(zero,_mm_min_ps(_mm_set1_ps(65535),_mm_max_ps(_mm_setzero_ps(),value)));
                    return _mm_sub_epi32(_mm_cvttps_epi32(value), offset);
                  };
                  for (; end-c>=8; c+=8) {
                    const auto x = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ar+c*sizeof(T)));
                    const auto y = _mm_loadu_si128(reinterpret_cast<const __m128i*>(br+c*sizeof(T)));
                    const auto low = divide(_mm_unpacklo_epi16(x,zero_i), _mm_unpacklo_epi16(y,zero_i));
                    const auto high = divide(_mm_unpackhi_epi16(x,zero_i), _mm_unpackhi_epi16(y,zero_i));
                    const auto packed = _mm_xor_si128(_mm_packs_epi32(low,high), _mm_set1_epi16(-32768));
                    _mm_storeu_si128(reinterpret_cast<__m128i*>(target+c),packed);
                  }
                }
              }
              if constexpr (std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t>) {
                if (simd && (mode == Arithmetic::Invert ||
                             (mode == Arithmetic::Difference && mult == 1))) {
                  if(mode==Arithmetic::Invert)c=unsigned_row_sse2<T,true>(ar,br,target,c,end);
                  else c=unsigned_row_sse2<T,false>(ar,br,target,c,end);
                }
              }
#else
          (void)simd;
#endif
              for (; c < end; ++c) {
                const double x = read<T>(ar, c);
                double value;
                if (mode == Arithmetic::Invert) {
                  if constexpr (std::is_unsigned_v<T>)
                    value = std::numeric_limits<T>::max() - x;
                  else
                    value = -x;
                } else {
                  const double y = read<T>(br, c);
                  if constexpr (std::is_same_v<T, std::uint8_t>) {
                    if (byte_quotient) {
                      const float quotient = y == 0 ? 0 : float(x) / float(y);
                      const float scaled = quotient * float(mult);
                      target[c] = saturate_cast<T>(y == 0 ? 0 : scaled + float(add));
                      continue;
                    }
                  }
                  value = mode == Arithmetic::Difference
                              ? std::abs(x - y) * mult
                              : (y == 0 ? 0 : (x / y) * mult + add);
                }
                target[c] = saturate_cast<T>(value);
              }
            }
          });
    });
  }
  return out;
}

void validate_paint(const Tuple& gray, PaintMode mode) {
  if (mode != PaintMode::Fill && mode != PaintMode::Margin)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "paint_region", "Type", -1, "invalid paint mode");
  if (gray.empty())
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    "paint_region", "Grayval", -1,
                    "gray values must not be empty");
  for (std::size_t i = 0; i < gray.size(); ++i) {
    const auto value = gray.at(i);
    if ((value.kind() != ValueKind::Int64 &&
         value.kind() != ValueKind::Real64) ||
        !std::isfinite(value.as_real64()))
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                      "paint_region", "Grayval", static_cast<std::int64_t>(i),
                      "gray values must be finite numbers");
  }
}
}  // namespace

mv::core::Image abs_diff_image(const mv::core::Image& a,
                               const mv::core::Image& b, double mult) {
  return arithmetic(a, b, Arithmetic::Difference, mult, 0, "abs_diff_image");
}
mv::core::Image div_image(const mv::core::Image& a, const mv::core::Image& b,
                          double mult, double add) {
  return arithmetic(a, b, Arithmetic::Divide, mult, add, "div_image");
}
mv::core::Image invert_image(const mv::core::Image& image) {
  return arithmetic(image, image, Arithmetic::Invert, 1, 0, "invert_image");
}

mv::core::Image crop_domain(const mv::core::Image& image) {
  require_image(image, "crop_domain");
  const auto domain = image.domain();
  const auto box = domain.bounding_box();
  if (!box)
    throw mv::core::Exception(mv::core::Status::InvalidArgument,
                              mv::core::ErrorCode::InvalidValue, "crop_domain",
                              "Image", -1, "cannot crop an empty domain");
  ExecutionContext context;
  std::pmr::vector<mv::core::PixelType> types{context.scratch_resource()};
  for (std::size_t ch = 0; ch < image.channel_count(); ++ch)
    types.push_back(image.pixel_type(ch));
  const auto width = static_cast<std::size_t>(box->col_max - box->col_min + 1);
  const auto height = static_cast<std::size_t>(box->row_max - box->row_min + 1);
  auto out = mv::core::Image::create(width, height, types);
  for (std::size_t ch = 0; ch < image.channel_count(); ++ch) {
    const auto sample = mv::core::bytes_per_sample(image.pixel_type(ch));
    const auto* src = static_cast<const std::byte*>(image.data(ch));
    auto* dst = static_cast<std::byte*>(out.mutable_data(ch));
    ExecutionContext::for_each_partition(
        height, ExecutionContext::saturated_work(width, height),
        ExecutionContext::kArithmeticParallelWork, [&](auto range) {
          for (auto row = range.begin; row < range.end; ++row)
            std::memcpy(dst + row * out.stride(ch),
                        src +
                            (row + static_cast<std::size_t>(box->row_min)) *
                                image.stride(ch) +
                            static_cast<std::size_t>(box->col_min) * sample,
                        width * sample);
        });
  }
  return out.reduce_domain(domain.shifted(-box->row_min, -box->col_min));
}

mv::core::Image paint_region(const mv::core::Regions& regions,
                             const mv::core::Image& image,
                             const mv::core::Tuple& gray, PaintMode mode) {
  validate_paint(gray, mode);
  require_image(image, "paint_region");
  const auto channels = image.channel_count();
  const bool per_region =
      regions.size() <= std::numeric_limits<std::size_t>::max() / channels &&
      gray.size() == regions.size() * channels;
  if (gray.size() != 1 && gray.size() != channels && !per_region)
    throw mv::core::Exception(
        mv::core::Status::InvalidArgument,
        mv::core::ErrorCode::InvalidCardinality, "paint_region", "Grayval", -1,
        "expected one value, one per channel, or region-major channel values");
  auto out = image.clone();
  // 区域有意按顺序处理，后置区域覆盖重叠。
  for (std::size_t ri = 0; ri < regions.size(); ++ri) {
    const auto clipped = regions[ri].intersect(image.domain());
    const auto runs = clipped.runs();
    for (std::size_t ch = 0; ch < channels; ++ch) {
      const double value = gray.as_real64(
          gray.size() == 1
              ? 0
              : (gray.size() == channels ? ch : ri * channels + ch));
      auto* dst = static_cast<std::byte*>(out.mutable_data(ch));
      dispatch_scalar(image.pixel_type(ch), [&](auto tag) {
        using T = typename decltype(tag)::type;
        const auto pixel = saturate_cast<T>(value);
        ExecutionContext::for_each_partition(
            runs.size(), static_cast<std::size_t>(clipped.area()),
            ExecutionContext::kArithmeticParallelWork, [&](auto range) {
              for (auto i = range.begin; i < range.end; ++i) {
                const auto& run = runs[i];
                auto* row = reinterpret_cast<T*>(
                    dst + static_cast<std::size_t>(run.row) * out.stride(ch));
                if (mode == PaintMode::Fill)
                  std::fill(row + run.col_begin, row + std::int64_t(run.col_end) + 1, pixel);
                else
                  for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
                    if (!regions[ri].contains(std::int64_t(run.row) - 1, c) ||
                        !regions[ri].contains(std::int64_t(run.row) + 1, c) ||
                        !regions[ri].contains(run.row, c - 1) ||
                        !regions[ri].contains(run.row, c + 1))
                      row[c] = pixel;
                  }
              }
            });
      });
    }
  }
  return out;
}

// iconic 输入零对象时仍验证控制。
void validate_arithmetic_controls(double mult, double add, const char* op) {
  require_controls(mult, add, op);
}
void validate_paint_controls(const mv::core::Tuple& gray, PaintMode mode) {
  validate_paint(gray, mode);
}
}  // namespace mv::imageproc
