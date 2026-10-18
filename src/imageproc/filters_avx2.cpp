#include "filters_avx2.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <imageproc/border.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/math.hpp>
#include <immintrin.h>
#include <memory_resource>
#include <type_traits>
#include <vector>
namespace mv::imageproc {
std::size_t sobel3_sumabs_u8_row_avx2(const std::uint8_t* top,const std::uint8_t* middle,
                                     const std::uint8_t* bottom,std::uint8_t* output,
                                     std::size_t first,std::size_t end) {
  auto c=first;
  const auto load=[](const std::uint8_t* p){return _mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(p)));};
  for(;end-c>=16;c+=16) {
    const auto tl=load(top+c-1),tm=load(top+c),tr=load(top+c+1);
    const auto ml=load(middle+c-1),mr=load(middle+c+1);
    const auto bl=load(bottom+c-1),bm=load(bottom+c),br=load(bottom+c+1);
    const auto gx=_mm256_sub_epi16(_mm256_add_epi16(_mm256_add_epi16(tl,bl),_mm256_slli_epi16(ml,1)),
                                   _mm256_add_epi16(_mm256_add_epi16(tr,br),_mm256_slli_epi16(mr,1)));
    const auto gy=_mm256_sub_epi16(_mm256_add_epi16(_mm256_add_epi16(tl,tr),_mm256_slli_epi16(tm,1)),
                                   _mm256_add_epi16(_mm256_add_epi16(bl,br),_mm256_slli_epi16(bm,1)));
    const auto value=_mm256_srli_epi16(_mm256_add_epi16(_mm256_abs_epi16(gx),_mm256_abs_epi16(gy)),2);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(output+c),
                     _mm_packus_epi16(_mm256_castsi256_si128(value),_mm256_extracti128_si256(value,1)));
  }
  return c-first;
}
namespace {
template <class T> T read(const std::byte *p, std::size_t i) {
  T v;
  std::memcpy(&v, p + i * sizeof(T), sizeof(T));
  return v;
}
template <class T> __m256d load4(const std::byte *p, std::size_t i) {
  p += i * sizeof(T);
  if constexpr (std::is_same_v<T, float>)
    return _mm256_cvtps_pd(_mm_loadu_ps(reinterpret_cast<const float *>(p)));
  else if constexpr (sizeof(T) == 1) {
    int v;
    std::memcpy(&v, p, 4);
    return _mm256_cvtepi32_pd(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(v)));
  } else
    return _mm256_cvtepi32_pd(_mm_cvtepu16_epi32(
        _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p))));
}
template <class T> __m256i load8(const std::byte *p, std::size_t i) {
  p += i * sizeof(T);
  if constexpr (sizeof(T) == 1)
    return _mm256_cvtepu8_epi32(
        _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p)));
  else
    return _mm256_cvtepu16_epi32(
        _mm_loadu_si128(reinterpret_cast<const __m128i *>(p)));
}
template <class T> void store4(std::byte *p, std::size_t i, __m256d sum) {
  p += i * sizeof(T);
  if constexpr (std::is_same_v<T, float>)
    _mm_storeu_ps(reinterpret_cast<float *>(p), _mm256_cvtpd_ps(sum));
  else {
    const auto v = _mm256_cvttpd_epi32(_mm256_add_pd(sum, _mm256_set1_pd(.5)));
    const auto w = _mm_packus_epi32(v, _mm_setzero_si128());
    if constexpr (sizeof(T) == 2)
      _mm_storel_epi64(reinterpret_cast<__m128i *>(p), w);
    else {
      const int x = _mm_cvtsi128_si32(_mm_packus_epi16(w, w));
      std::memcpy(p, &x, 4);
    }
  }
}
template <class T> void store8(std::byte *p, std::size_t i, __m256i sum) {
  p += i * sizeof(T);
  const auto v = _mm256_cvttps_epi32(_mm256_add_ps(
      _mm256_mul_ps(_mm256_cvtepi32_ps(sum), _mm256_set1_ps(.04f)),
      _mm256_set1_ps(.5f)));
  const auto w = _mm_packus_epi32(_mm256_castsi256_si128(v),
                                  _mm256_extracti128_si256(v, 1));
  if constexpr (sizeof(T) == 2)
    _mm_storeu_si128(reinterpret_cast<__m128i *>(p), w);
  else
    _mm_storel_epi64(reinterpret_cast<__m128i *>(p), _mm_packus_epi16(w, w));
}
// 整数 Gaussian tap 是 1/1024 精确倍数；Byte 水平和需 18 位、完整窗口需 28 位，
// 整数与原序 Double 路径最终舍入一致。缩减域保留原稀疏行遍历。
bool byte_gaussian(const core::Image& image, core::Image& output,
                   std::size_t channel, std::span<const double> kx,
                   std::span<const double> ky) {
  if (image.has_reduced_domain() || kx.size() < 3 || kx.size() > 11 ||
      kx.size() != ky.size() || !std::equal(kx.begin(), kx.end(), ky.begin()))
    return false;
  std::array<std::uint32_t, 11> weights{};
  std::uint32_t total = 0;
  for (std::size_t k = 0; k < kx.size(); ++k) {
    const auto scaled = kx[k] * 1024.;
    if (!(scaled >= 0 && scaled <= 1024.)) return false;
    weights[k] = static_cast<std::uint32_t>(scaled);
    if (scaled != double(weights[k])) return false;
    total += weights[k];
  }
  if (total != 1024) return false;
  const auto width = image.width(), height = image.height(), taps = kx.size();
  const auto radius = static_cast<std::int64_t>(taps / 2);
  const auto stride = image.stride(channel), out_stride = output.stride(channel);
  const auto* input = static_cast<const std::uint8_t*>(image.data(channel));
  auto* destination = static_cast<std::uint8_t*>(output.mutable_data(channel));
  ExecutionContext::for_each_partition(height,
      ExecutionContext::saturated_work(width * height, taps),
      ExecutionContext::kFilterParallelWork, [&](auto range) {
    ExecutionContext local;
    std::pmr::vector<std::uint32_t> ring(taps * width, local.scratch_resource());
    std::pmr::vector<std::uint8_t> padded(width + taps - 1, local.scratch_resource());
    std::array<std::int64_t, 11> cached{}; cached.fill(-1);
    std::array<const std::uint32_t*, 11> rows{};
    for (auto y = range.begin; y < range.end; ++y) {
      for (std::size_t k = 0; k < taps; ++k) {
        const auto source_y = *border_index(std::int64_t(y) + std::int64_t(k) - radius,
                                           height, BorderMode::Mirrored);
        const auto slot = source_y % taps;
        auto* filtered = ring.data() + slot * width; rows[k] = filtered;
        if (cached[slot] == std::int64_t(source_y)) continue;
        const auto* source = input + source_y * stride;
        std::memcpy(padded.data() + radius, source, width);
        for (std::int64_t c = 0; c < radius; ++c) {
          padded[std::size_t(c)] = source[*border_index(c - radius, width, BorderMode::Mirrored)];
          padded[std::size_t(radius) + width + std::size_t(c)] =
              source[*border_index(std::int64_t(width) + c, width, BorderMode::Mirrored)];
        }
        std::size_t x = 0;
        for (; width - x >= 8; x += 8) {
          auto sum = _mm256_setzero_si256();
          for (std::size_t t = 0; t < taps; ++t) {
            const auto value = _mm256_cvtepu8_epi32(_mm_loadl_epi64(
                reinterpret_cast<const __m128i*>(padded.data() + x + t)));
            sum = _mm256_add_epi32(sum, _mm256_mullo_epi32(value,
                _mm256_set1_epi32(static_cast<int>(weights[t]))));
          }
          _mm256_storeu_si256(reinterpret_cast<__m256i*>(filtered + x), sum);
        }
        for (; x < width; ++x) {
          std::uint32_t sum = 0;
          for (std::size_t t = 0; t < taps; ++t) sum += padded[x + t] * weights[t];
          filtered[x] = sum;
        }
        cached[slot] = std::int64_t(source_y);
      }
      auto* out = destination + y * out_stride;
      std::size_t x = 0;
      for (; width - x >= 8; x += 8) {
        auto sum = _mm256_setzero_si256();
        for (std::size_t t = 0; t < taps; ++t)
          sum = _mm256_add_epi32(sum, _mm256_mullo_epi32(
              _mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[t] + x)),
              _mm256_set1_epi32(static_cast<int>(weights[t]))));
        sum = _mm256_srli_epi32(_mm256_add_epi32(sum, _mm256_set1_epi32(1 << 19)), 20);
        const auto words = _mm_packus_epi32(_mm256_castsi256_si128(sum),
                                           _mm256_extracti128_si256(sum, 1));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(out + x), _mm_packus_epi16(words, words));
      }
      for (; x < width; ++x) {
        std::uint32_t sum = 0;
        for (std::size_t t = 0; t < taps; ++t) sum += rows[t][x] * weights[t];
        out[x] = static_cast<std::uint8_t>((sum + (1U << 19)) >> 20);
      }
    }
  });
  return true;
}

template <class T,class Output=T>
void filter(const core::Image &image, core::Image &output, std::size_t channel,
            std::span<const double> kx, std::span<const double> ky) {
  using Acc = double;
  const auto width = image.width(), height = image.height(),
             stride = image.stride(channel),
             out_stride = output.stride(channel);
  const auto *input = static_cast<const std::byte *>(image.data(channel));
  auto *destination = static_cast<std::byte *>(output.mutable_data(channel));
  const auto domain = image.domain();
  const auto runs = domain.runs();
  const auto rx = static_cast<std::int64_t>(kx.size() / 2),
             ry = static_cast<std::int64_t>(ky.size() / 2);
  ExecutionContext::for_each_partition(
      runs.size(), ExecutionContext::saturated_work(width * height, ky.size()),
      ExecutionContext::kFilterParallelWork, [&](auto range) {
        ExecutionContext local;
        std::pmr::vector<Acc> ring(ky.size() * width, local.scratch_resource());
        std::pmr::vector<std::int64_t> cached(ky.size(), -1,
                                              local.scratch_resource());
        std::pmr::vector<const Acc *> rows(ky.size(), nullptr,
                                           local.scratch_resource());
        std::pmr::vector<double> padded(local.scratch_resource());
        if constexpr(std::is_same_v<Output,float> && !std::is_same_v<T,float>)padded.resize(width+2*std::size_t(rx));
        for (auto index = range.begin; index < range.end; ++index) {
          const auto &run = runs[index];

          if (runs.size() < height / 2 && kx.size() == ky.size() &&
              kx.size() > 1 && std::equal(kx.begin(), kx.end(), ky.begin())) {
            auto *values = ring.data() + rx;
            std::vector<const std::byte *> source(ky.size());
            for (std::size_t t = 0; t < ky.size(); ++t)
              source[t] = input + border_index(std::int64_t(run.row) + std::int64_t(t) - ry,
                                               height, BorderMode::Mirrored)
                                          .value() *
                                      stride;
            std::size_t x = 0;
            // 独立输出组隐藏原序 tap 加法延迟。
            for (; width - x >= 16; x += 16) {
              __m256d sums[4]{_mm256_setzero_pd(), _mm256_setzero_pd(),
                             _mm256_setzero_pd(), _mm256_setzero_pd()};
              for (std::size_t t = 0; t < ky.size(); ++t) {
                const auto weight = _mm256_set1_pd(ky[t]);
                for (int j = 0; j < 4; ++j)
                  sums[j] = _mm256_add_pd(sums[j],
                      _mm256_mul_pd(load4<T>(source[t], x + j * 4), weight));
              }
              for (int j = 0; j < 4; ++j)
                _mm256_storeu_pd(values + x + j * 4, sums[j]);
            }
            for (; x + 4 <= width; x += 4) {
              auto sum = _mm256_setzero_pd();
              for (std::size_t t = 0; t < ky.size(); ++t)
                sum = _mm256_add_pd(sum, _mm256_mul_pd(load4<T>(source[t], x),
                                                       _mm256_set1_pd(ky[t])));
              _mm256_storeu_pd(values + x, sum);
            }
            for (; x < width; ++x) {
              double sum = 0;
              for (std::size_t t = 0; t < ky.size(); ++t)
                sum += read<T>(source[t], x) * ky[t];
              values[x] = sum;
            }
            for (std::int64_t t = -rx; t < 0; ++t)
              values[t] =
                  values[border_index(t, width, BorderMode::Mirrored).value()];
            for (std::size_t t = width; t < width + std::size_t(rx); ++t)
              values[t] = values[border_index(std::int64_t(t), width,
                                              BorderMode::Mirrored)
                                     .value()];
            for (std::size_t t = 0; t < ky.size(); ++t)
              rows[t] = ring.data() + t;
          } else {
            for (std::size_t k = 0; k < ky.size(); ++k) {
              const auto y =
                  border_index(std::int64_t(run.row) + static_cast<std::int64_t>(k) - ry,
                               height, BorderMode::Mirrored)
                      .value();
              const auto slot = y % ky.size();
              auto *row = ring.data() + slot * width;
              rows[k] = row;
              if (cached[slot] == static_cast<std::int64_t>(y))
                continue;
              const auto *source = input + y * stride;
              if constexpr(std::is_same_v<Output,float> && !std::is_same_v<T,float>) {
                auto* values=padded.data()+rx;std::size_t c=0;
                for(;width-c>=4;c+=4)_mm256_storeu_pd(values+c,load4<T>(source,c));
                for(;c<width;++c)values[c]=double(read<T>(source,c));
                for(std::int64_t c=-rx;c<0;++c)values[c]=values[*border_index(c,width,BorderMode::Mirrored)];
                for(std::size_t c=width;c<width+std::size_t(rx);++c)values[c]=values[*border_index(std::int64_t(c),width,BorderMode::Mirrored)];
                convolve_double_row_avx2(padded.data(),kx,row,width);cached[slot]=std::int64_t(y);continue;
              }
              std::size_t x = 0;
              while (x < width) {

                if (x >= static_cast<std::size_t>(rx) &&
                    width - x >= 16 + static_cast<std::size_t>(rx)) {
                  __m256d sums[4]{_mm256_setzero_pd(), _mm256_setzero_pd(),
                                 _mm256_setzero_pd(), _mm256_setzero_pd()};
                  for (std::size_t tap = 0; tap < kx.size(); ++tap) {
                    const auto weight = _mm256_set1_pd(kx[tap]);
                    for (int j = 0; j < 4; ++j)
                      sums[j] = _mm256_add_pd(sums[j], _mm256_mul_pd(
                          load4<T>(source, x + tap - std::size_t(rx) + j * 4), weight));
                  }
                  for (int j = 0; j < 4; ++j)
                    _mm256_storeu_pd(row + x + j * 4, sums[j]);
                  x += 16;
                  continue;
                }
                if (x >= static_cast<std::size_t>(rx) &&
                    x + 4 + static_cast<std::size_t>(rx) <= width) {
                  auto sum = _mm256_setzero_pd();
                  for (std::size_t tap = 0; tap < kx.size(); ++tap)
                    sum = _mm256_add_pd(
                        sum,
                        _mm256_mul_pd(
                            load4<T>(source,
                                     x + tap - static_cast<std::size_t>(rx)),
                            _mm256_set1_pd(kx[tap])));
                  _mm256_storeu_pd(row + x, sum);
                  x += 4;
                  continue;
                }

                double sum = 0;
                for (std::size_t tap = 0; tap < kx.size(); ++tap) {
                  const auto c =
                      border_index(static_cast<std::int64_t>(x) +
                                       static_cast<std::int64_t>(tap) - rx,
                                   width, BorderMode::Mirrored)
                          .value();
                  sum += static_cast<double>(read<T>(source, c)) * kx[tap];
                }
                row[x++] = static_cast<Acc>(sum);
              }
              cached[slot] = static_cast<std::int64_t>(y);
            }
          }

          auto *out =
              destination + static_cast<std::size_t>(run.row) * out_stride;
          auto x = static_cast<std::size_t>(run.col_begin);
          const auto end = static_cast<std::size_t>(run.col_end) + 1;
          if constexpr(std::is_same_v<Output,float> && !std::is_same_v<T,float>) {
            if(x==0 && end==width){convolve_float_rows_avx2(rows,ky,reinterpret_cast<float*>(out),width);continue;}
          }

          for (; end - x >= 16; x += 16) {
            __m256d sums[4]{_mm256_setzero_pd(), _mm256_setzero_pd(),
                           _mm256_setzero_pd(), _mm256_setzero_pd()};
            for (std::size_t k = 0; k < ky.size(); ++k) {
              const auto weight = _mm256_set1_pd(ky[k]);
              for (int j = 0; j < 4; ++j)
                sums[j] = _mm256_add_pd(sums[j],
                    _mm256_mul_pd(_mm256_loadu_pd(rows[k] + x + j * 4), weight));
            }
            for (int j = 0; j < 4; ++j)
              store4<Output>(out, x + j * 4, sums[j]);
          }
          for (; x + 4 <= end; x += 4) {
            auto sum = _mm256_setzero_pd();
            for (std::size_t k = 0; k < ky.size(); ++k)
              sum =
                  _mm256_add_pd(sum, _mm256_mul_pd(_mm256_loadu_pd(rows[k] + x),
                                                   _mm256_set1_pd(ky[k])));
            store4<Output>(out, x, sum);
          }

          for (; x < end; ++x) {
            double sum = 0;
            for (std::size_t k = 0; k < ky.size(); ++k)
              sum += static_cast<double>(rows[k][x]) * ky[k];
            const auto v = saturate_cast<Output>(sum);
            std::memcpy(out + x * sizeof(Output), &v, sizeof(Output));
          }
        }
      });
}

// 五 tap 整数核保持精确和到最终舍入；Gaussian 系数和为 1024，UInt16 也只需 36 位累加。
template <class T, bool Box>
void integer5(const core::Image &image, core::Image &output,
              std::size_t channel) {
  constexpr bool narrow = Box && sizeof(T) == 1;
  using Acc = std::conditional_t<narrow, std::uint16_t, std::uint32_t>;
  constexpr std::size_t lanes = narrow ? 16 : 8;
  const auto width = image.width(), height = image.height(),
             stride = image.stride(channel), os = output.stride(channel);
  const auto *input = static_cast<const std::byte *>(image.data(channel));
  auto *destination = static_cast<std::byte *>(output.mutable_data(channel));
  const auto domain = image.domain();
  const auto runs = domain.runs();
  ExecutionContext::for_each_partition(
      runs.size(), ExecutionContext::saturated_work(width * height, 5),
      ExecutionContext::kFilterParallelWork, [&](auto range) {
        ExecutionContext local;
        std::pmr::vector<Acc> ring(5 * width, local.scratch_resource());
        std::array<std::int64_t, 5> cached{-1, -1, -1, -1, -1};
        const Acc *rows[5];
        constexpr int weights[5]{83, 246, 366, 246, 83};
        for (auto index = range.begin; index < range.end; ++index) {
          const auto &run = runs[index];
          if (runs.size() < height / 2) {
            // 输出行稀疏时先纵向滤波；整数和在两种顺序均精确，只需一条中间行。
            auto *values = ring.data() + 2;
            const std::byte *source[5];
            for (int t = 0; t < 5; ++t)
              source[t] = input + border_index(std::int64_t(run.row) + t - 2, height,
                                               BorderMode::Mirrored)
                                          .value() *
                                      stride;
            std::size_t x = 0;
            for (; x + lanes <= width; x += lanes) {
              auto sum = _mm256_setzero_si256();
              for (int t = 0; t < 5; ++t) {
                if constexpr (narrow)
                  sum = _mm256_add_epi16(
                      sum,
                      _mm256_cvtepu8_epi16(_mm_loadu_si128(
                          reinterpret_cast<const __m128i *>(source[t] + x))));
                else {
                  auto v = load8<T>(source[t], x);
                  if constexpr (!Box)
                    v = _mm256_mullo_epi32(v, _mm256_set1_epi32(weights[t]));
                  sum = _mm256_add_epi32(sum, v);
                }
              }
              _mm256_storeu_si256(reinterpret_cast<__m256i *>(values + x), sum);
            }
            for (; x < width; ++x) {
              std::uint32_t sum = 0;
              for (int t = 0; t < 5; ++t)
                sum += read<T>(source[t], x) * (Box ? 1 : weights[t]);
              values[x] = static_cast<Acc>(sum);
            }
            for (int t = -2; t < 0; ++t)
              values[t] =
                  values[border_index(t, width, BorderMode::Mirrored).value()];
            for (std::size_t t = width; t < width + 2; ++t)
              values[t] = values[border_index(std::int64_t(t), width,
                                              BorderMode::Mirrored)
                                     .value()];
            for (int t = 0; t < 5; ++t)
              rows[t] = ring.data() + t;
          } else {
            for (std::size_t k = 0; k < 5; ++k) {
              const auto y = border_index(std::int64_t(run.row) + std::int64_t(k) - 2, height,
                                          BorderMode::Mirrored)
                                 .value();
              const auto slot = y % 5;
              auto *row = ring.data() + slot * width;
              rows[k] = row;
              if (cached[slot] == std::int64_t(y))
                continue;
              const auto *source = input + y * stride;
              std::size_t x = 0;
              for (; x < std::min(width, std::size_t(2)); ++x) {
                std::uint32_t sum = 0;
                for (int t = 0; t < 5; ++t)
                  sum +=
                      read<T>(source, border_index(std::int64_t(x) + t - 2,
                                                   width, BorderMode::Mirrored)
                                          .value()) *
                      (Box ? 1 : weights[t]);
                row[x] = static_cast<Acc>(sum);
              }
              for (; x + lanes + 2 <= width; x += lanes) {
                if constexpr (narrow) {
                  auto sum = _mm256_setzero_si256();
                  for (int t = 0; t < 5; ++t)
                    sum = _mm256_add_epi16(
                        sum, _mm256_cvtepu8_epi16(_mm_loadu_si128(
                                 reinterpret_cast<const __m128i *>(source + x +
                                                                   t - 2))));
                  _mm256_storeu_si256(reinterpret_cast<__m256i *>(row + x),
                                      sum);
                } else {
                  auto sum = _mm256_setzero_si256();
                  for (int t = 0; t < 5; ++t) {
                    auto v = load8<T>(source, x + t - 2);
                    if constexpr (!Box)
                      v = _mm256_mullo_epi32(v, _mm256_set1_epi32(weights[t]));
                    sum = _mm256_add_epi32(sum, v);
                  }
                  _mm256_storeu_si256(reinterpret_cast<__m256i *>(row + x),
                                      sum);
                }
              }
              for (; x < width; ++x) {
                std::uint32_t sum = 0;
                for (int t = 0; t < 5; ++t)
                  sum +=
                      read<T>(source, border_index(std::int64_t(x) + t - 2,
                                                   width, BorderMode::Mirrored)
                                          .value()) *
                      (Box ? 1 : weights[t]);
                row[x] = static_cast<Acc>(sum);
              }
              cached[slot] = std::int64_t(y);
            }
          }
          auto *out = destination + std::size_t(run.row) * os;
          auto x = std::size_t(run.col_begin);
          const auto end = std::size_t(run.col_end) + 1;
          for (; x + lanes <= end; x += lanes) {
            if constexpr (narrow) {
              auto sum = _mm256_setzero_si256();
              for (int t = 0; t < 5; ++t)
                sum = _mm256_add_epi16(
                    sum, _mm256_loadu_si256(
                             reinterpret_cast<const __m256i *>(rows[t] + x)));
              // floor((sum+12)/25) 对全部可能的 5x5 UInt8 和均精确。
              sum = _mm256_srli_epi16(
                  _mm256_mulhi_epu16(
                      _mm256_add_epi16(sum, _mm256_set1_epi16(12)),
                      _mm256_set1_epi16(5243)),
                  1);
              _mm_storeu_si128(
                  reinterpret_cast<__m128i *>(out + x),
                  _mm_packus_epi16(_mm256_castsi256_si128(sum),
                                   _mm256_extracti128_si256(sum, 1)));
            } else {
              auto sum = _mm256_setzero_si256(), odd = _mm256_setzero_si256();
              for (int t = 0; t < 5; ++t) {
                auto v = _mm256_loadu_si256(
                    reinterpret_cast<const __m256i *>(rows[t] + x));
                if constexpr (Box)
                  sum = _mm256_add_epi32(sum, v);
                else if constexpr (sizeof(T) == 1)
                  sum = _mm256_add_epi32(
                      sum,
                      _mm256_mullo_epi32(v, _mm256_set1_epi32(weights[t])));
                else {
                  const auto w = _mm256_set1_epi32(weights[t]);
                  sum = _mm256_add_epi64(sum, _mm256_mul_epu32(v, w));
                  odd = _mm256_add_epi64(
                      odd, _mm256_mul_epu32(_mm256_srli_epi64(v, 32), w));
                }
              }
              if constexpr (Box)
                store8<T>(out, x, sum);
              else {
                if constexpr (sizeof(T) == 1)
                  sum = _mm256_srli_epi32(
                      _mm256_add_epi32(sum, _mm256_set1_epi32(1 << 19)), 20);
                else {
                  sum = _mm256_srli_epi64(
                      _mm256_add_epi64(sum, _mm256_set1_epi64x(1 << 19)), 20);
                  odd = _mm256_srli_epi64(
                      _mm256_add_epi64(odd, _mm256_set1_epi64x(1 << 19)), 20);
                  sum = _mm256_or_si256(sum, _mm256_slli_epi64(odd, 32));
                }
                const auto packed =
                    _mm_packus_epi32(_mm256_castsi256_si128(sum),
                                     _mm256_extracti128_si256(sum, 1));
                if constexpr (sizeof(T) == 1)
                  _mm_storel_epi64(reinterpret_cast<__m128i *>(out + x),
                                   _mm_packus_epi16(packed, packed));
                else
                  _mm_storeu_si128(
                      reinterpret_cast<__m128i *>(out + x * sizeof(T)), packed);
              }
            }
          }
          for (; x < end; ++x) {
            std::uint64_t sum = 0;
            for (int t = 0; t < 5; ++t)
              sum += std::uint64_t(rows[t][x]) * (Box ? 1 : weights[t]);
            const T v =
                static_cast<T>(Box ? (sum + 12) / 25 : (sum + (1 << 19)) >> 20);
            std::memcpy(out + x * sizeof(T), &v, sizeof(T));
          }
        }
      });
}
template <class T>
std::size_t median_original(const void *a, const void *b, const void *c, void *output,
                   std::size_t start, std::size_t end) {
  const std::byte *rows[3] = {static_cast<const std::byte *>(a),
                              static_cast<const std::byte *>(b),
                              static_cast<const std::byte *>(c)};
  constexpr std::size_t lanes = 32 / sizeof(T);
  auto i = start;
  for (; i + lanes <= end;) {
    __m256i p[9];
    for (int r = 0; r < 3; ++r)
      for (int k = 0; k < 3; ++k)
        p[r * 3 + k] = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(
            rows[r] + (i + k - 1) * sizeof(T)));
    const auto cs = [&](int x, int y) {
      auto lo = sizeof(T) == 1 ? _mm256_min_epu8(p[x], p[y])
                               : _mm256_min_epu16(p[x], p[y]);
      p[y] = sizeof(T) == 1 ? _mm256_max_epu8(p[x], p[y])
                            : _mm256_max_epu16(p[x], p[y]);
      p[x] = lo;
    };
    cs(1, 2);
    cs(4, 5);
    cs(7, 8);
    cs(0, 1);
    cs(3, 4);
    cs(6, 7);
    cs(1, 2);
    cs(4, 5);
    cs(7, 8);
    cs(0, 3);
    cs(5, 8);
    cs(4, 7);
    cs(3, 6);
    cs(1, 4);
    cs(2, 5);
    cs(4, 7);
    cs(4, 2);
    cs(6, 4);
    cs(4, 2);
    _mm256_storeu_si256(reinterpret_cast<__m256i *>(
                            static_cast<std::byte *>(output) + i * sizeof(T)),
                        p[4]);
    i += lanes;
    // 短尾部使用同一游程内的重叠向量完成。
    if (i < end && end - i < lanes)
      i = end - lanes;
  }
  return i - start;
}

// 相邻整数 SIMD 块复用精确排序的纵向列；原行代码处理短游程及尾部，不试探越界读取。
template<class T> std::size_t median(const void* a,const void* b,const void* c,void* output,std::size_t start,std::size_t end){
 const T* rows[3]={static_cast<const T*>(a),static_cast<const T*>(b),static_cast<const T*>(c)};
 constexpr std::size_t lanes=32/sizeof(T);auto i=start;
 if(end-start<2*lanes)return median_original<T>(a,b,c,output,start,end);
 const auto cs=[](__m256i& x,__m256i& y){auto lo=sizeof(T)==1?_mm256_min_epu8(x,y):_mm256_min_epu16(x,y);y=sizeof(T)==1?_mm256_max_epu8(x,y):_mm256_max_epu16(x,y);x=lo;};
 const auto load=[&](std::size_t at,__m256i* v){for(int r=0;r<3;++r)v[r]=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[r]+at));cs(v[1],v[2]);cs(v[0],v[1]);cs(v[1],v[2]);};
 __m256i current[3];load(i-1,current);
 for(;end-i>=2*lanes;i+=lanes){
  __m256i next[3],p[9];load(i+lanes-1,next);
  for(int r=0;r<3;++r){auto bridge=_mm256_permute2x128_si256(current[r],next[r],0x21);p[r]=current[r];p[3+r]=_mm256_alignr_epi8(bridge,current[r],sizeof(T));p[6+r]=_mm256_alignr_epi8(bridge,current[r],2*sizeof(T));}
  // 三条已排序列替代原前九次比较。
  cs(p[0],p[3]);cs(p[5],p[8]);cs(p[4],p[7]);cs(p[3],p[6]);cs(p[1],p[4]);cs(p[2],p[5]);cs(p[4],p[7]);cs(p[4],p[2]);cs(p[6],p[4]);cs(p[4],p[2]);
  _mm256_storeu_si256(reinterpret_cast<__m256i*>(static_cast<T*>(output)+i),p[4]);for(int r=0;r<3;++r)current[r]=next[r];
 }
 i+=median_original<T>(a,b,c,output,i,end);
 return i-start;
}
} // namespace

void convolve_integer_to_float_avx2(const core::Image& image,core::Image& output,std::size_t channel,
                                      std::span<const double> x,std::span<const double> y) {
  if(image.pixel_type(channel)==core::PixelType::UInt8)filter<std::uint8_t,float>(image,output,channel,x,y);
  else filter<std::uint16_t,float>(image,output,channel,x,y);
}
void convolve_double_row_avx2(const double* input,std::span<const double> kernel,double* output,std::size_t width) {
  std::size_t c=0;
  for(;width-c>=16;c+=16) {
    __m256d sums[4]{_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd()};
    for(std::size_t k=0;k<kernel.size();++k) {
      const auto weight=_mm256_set1_pd(kernel[k]);
      for(int j=0;j<4;++j)sums[j]=_mm256_add_pd(sums[j],_mm256_mul_pd(weight,_mm256_loadu_pd(input+c+k+j*4)));
    }
    for(int j=0;j<4;++j)_mm256_storeu_pd(output+c+j*4,sums[j]);
  }
  for(;c<width;++c){double sum=0;for(std::size_t k=0;k<kernel.size();++k)sum+=kernel[k]*input[c+k];output[c]=sum;}
}
void convolve_double_pair_row_avx2(const double* input,std::span<const double> a,
    std::span<const double> b,double* oa,double* ob,std::size_t width) {
  if(a.size()>b.size()) {
    convolve_double_row_avx2(input,a,oa,width);
    convolve_double_row_avx2(input+(a.size()-b.size())/2,b,ob,width);return;
  }
  const auto offset=(b.size()-a.size())/2;
  std::size_t c=0;
  for(;width-c>=16;c+=16) {
    __m256d sa[4]{},sb[4]{};
    for(auto& v:sa)v=_mm256_setzero_pd();for(auto& v:sb)v=_mm256_setzero_pd();
    for(std::size_t k=0;k<offset;++k) {
      const auto weight=_mm256_set1_pd(b[k]);
      for(int j=0;j<4;++j)sb[j]=_mm256_add_pd(sb[j],_mm256_mul_pd(weight,_mm256_loadu_pd(input+c+k+j*4)));
    }
    for(std::size_t k=0;k<a.size();++k) {
      const auto wa=_mm256_set1_pd(a[k]),wb=_mm256_set1_pd(b[offset+k]);
      for(int j=0;j<4;++j) {
        const auto value=_mm256_loadu_pd(input+c+offset+k+j*4);
        sa[j]=_mm256_add_pd(sa[j],_mm256_mul_pd(wa,value));
        sb[j]=_mm256_add_pd(sb[j],_mm256_mul_pd(wb,value));
      }
    }
    for(std::size_t k=offset+a.size();k<b.size();++k) {
      const auto weight=_mm256_set1_pd(b[k]);
      for(int j=0;j<4;++j)sb[j]=_mm256_add_pd(sb[j],_mm256_mul_pd(weight,_mm256_loadu_pd(input+c+k+j*4)));
    }
    for(int j=0;j<4;++j){_mm256_storeu_pd(oa+c+j*4,sa[j]);_mm256_storeu_pd(ob+c+j*4,sb[j]);}
  }
  for(;c<width;++c) {
    double x=0,y=0;
    for(std::size_t k=0;k<a.size();++k)x+=a[k]*input[c+offset+k];
    for(std::size_t k=0;k<b.size();++k)y+=b[k]*input[c+k];
    oa[c]=x;ob[c]=y;
  }
}
void convolve_float_rows_avx2(std::span<const double* const> input,std::span<const double> kernel,float* output,std::size_t width) {
  std::size_t c=0;
  for(;width-c>=16;c+=16) {
    __m256d sums[4]{_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd(),_mm256_setzero_pd()};
    for(std::size_t k=0;k<kernel.size();++k) {
      const auto weight=_mm256_set1_pd(kernel[k]);
      for(int j=0;j<4;++j)sums[j]=_mm256_add_pd(sums[j],_mm256_mul_pd(weight,_mm256_loadu_pd(input[k]+c+j*4)));
    }
    for(int j=0;j<4;++j)_mm_storeu_ps(output+c+j*4,_mm256_cvtpd_ps(sums[j]));
  }
  for(;c<width;++c){double sum=0;for(std::size_t k=0;k<kernel.size();++k)sum+=kernel[k]*input[k][c];output[c]=static_cast<float>(sum);}
}
void convolve_float_scaled_pair_rows_avx2(
    std::span<const double* const> a,std::span<const double> ka,
    std::span<const double* const> b,std::span<const double> kb,
    double* oa,double* ob,std::size_t width,double scale) {
  const auto common=std::min(ka.size(),kb.size());
  const auto factor=_mm256_set1_pd(scale);
  std::size_t c=0;
  for(;width-c>=16;c+=16) {
    __m256d sa[4]{},sb[4]{};
    for(auto& v:sa)v=_mm256_setzero_pd();
    for(auto& v:sb)v=_mm256_setzero_pd();
    for(std::size_t k=0;k<common;++k) {
      const auto wa=_mm256_set1_pd(ka[k]),wb=_mm256_set1_pd(kb[k]);
      for(int j=0;j<4;++j) {
        sa[j]=_mm256_add_pd(sa[j],_mm256_mul_pd(wa,_mm256_loadu_pd(a[k]+c+j*4)));
        sb[j]=_mm256_add_pd(sb[j],_mm256_mul_pd(wb,_mm256_loadu_pd(b[k]+c+j*4)));
      }
    }
    for(std::size_t k=common;k<ka.size();++k)
      for(int j=0;j<4;++j)sa[j]=_mm256_add_pd(sa[j],_mm256_mul_pd(_mm256_set1_pd(ka[k]),_mm256_loadu_pd(a[k]+c+j*4)));
    for(std::size_t k=common;k<kb.size();++k)
      for(int j=0;j<4;++j)sb[j]=_mm256_add_pd(sb[j],_mm256_mul_pd(_mm256_set1_pd(kb[k]),_mm256_loadu_pd(b[k]+c+j*4)));
    for(int j=0;j<4;++j) {
      // 公开导数在缩放前舍入到 Float32；只省去临时行存取，
      // 不能省略 Float32 舍入或原序 Double 求和。
      _mm256_storeu_pd(oa+c+j*4,_mm256_mul_pd(_mm256_cvtps_pd(_mm256_cvtpd_ps(sa[j])),factor));
      _mm256_storeu_pd(ob+c+j*4,_mm256_mul_pd(_mm256_cvtps_pd(_mm256_cvtpd_ps(sb[j])),factor));
    }
  }
  for(;c<width;++c) {
    double x=0,y=0;
    for(std::size_t k=0;k<ka.size();++k)x+=ka[k]*a[k][c];
    for(std::size_t k=0;k<kb.size();++k)y+=kb[k]*b[k][c];
    oa[c]=double(static_cast<float>(x))*scale;
    ob[c]=double(static_cast<float>(y))*scale;
  }
}
void separable_filter_avx2(const core::Image &a, core::Image &b, std::size_t c,
                           std::span<const double> x, std::span<const double> y,
                           bool box5) {
  constexpr double g[5]{83. / 1024, 246. / 1024, 366. / 1024, 246. / 1024,
                        83. / 1024};
  const bool gauss5 = x.size() == 5 && y.size() == 5 &&
                      std::equal(x.begin(), x.end(), g) &&
                      std::equal(y.begin(), y.end(), g);
  switch (a.pixel_type(c)) {
  case core::PixelType::UInt8:
    if (box5)
      integer5<std::uint8_t, true>(a, b, c);
    else if (gauss5)
      integer5<std::uint8_t, false>(a, b, c);
    else if (!byte_gaussian(a, b, c, x, y))
      filter<std::uint8_t>(a, b, c, x, y);
    break;
  case core::PixelType::UInt16:
    if (box5)
      integer5<std::uint16_t, true>(a, b, c);
    else if (gauss5)
      integer5<std::uint16_t, false>(a, b, c);
    else
      filter<std::uint16_t>(a, b, c, x, y);
    break;
  case core::PixelType::Float32:
    filter<float>(a, b, c, x, y);
    break;
  default:
    break;
  }
}
std::size_t median3_row_avx2(const void *a, const void *b, const void *c,
                             void *output, std::size_t begin, std::size_t end,
                             core::PixelType type) {
  return type == core::PixelType::UInt8
             ? median<std::uint8_t>(a, b, c, output, begin, end)
             : median<std::uint16_t>(a, b, c, output, begin, end);
}
namespace {
template<class T>
void box_columns(std::uint32_t* columns,const T* before,const T* after,std::size_t width) {
  auto load=[](const T* row) {
    if constexpr(sizeof(T)==1)return _mm256_cvtepu8_epi32(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row)));
    else return _mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(row)));
  };
  std::size_t c=0;
  for(;c+8<=width;c+=8) {
    auto value=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(columns+c));
    if(before)value=_mm256_sub_epi32(value,load(before+c));
    value=_mm256_add_epi32(value,load(after+c));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(columns+c),value);
  }
  for(;c<width;++c) {
    T a,b=0;std::memcpy(&a,after+c,sizeof(T));if(before)std::memcpy(&b,before+c,sizeof(T));
    columns[c]=columns[c]-b+a;
  }
}
template<class T>
void box_row(const std::uint32_t* columns,const std::size_t* leaving,const std::size_t* entering,
             T* out,std::size_t width,std::size_t radius,std::uint32_t area,std::uint32_t sum) {
  const double inverse=1./double(area);
  const auto inv=_mm256_set1_pd(inverse),half=_mm256_set1_pd(.5);
  auto normalize=[&](__m128i v){return _mm256_cvttpd_epi32(_mm256_add_pd(_mm256_mul_pd(_mm256_cvtepi32_pd(v),inv),half));};
  out[0]=static_cast<T>(double(sum)*inverse+.5);
  std::size_t c=1;
  // 内部滚动进位始终留在 SIMD 寄存器；边界索引和归一化表达式不变。
  for(;c<width && (c<radius+1 || width-c<8+radius);++c) {
    sum=sum-columns[leaving[c-1]]+columns[entering[c-1]];
    out[c]=static_cast<T>(double(sum)*inverse+.5);
  }
  if(c==width)return;
  auto carry=_mm256_set1_epi32(static_cast<int>(sum));
  for(;c<width && width-c>=8+radius;c+=8) {
    auto delta=_mm256_sub_epi32(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(columns+c+radius)),
                                _mm256_loadu_si256(reinterpret_cast<const __m256i*>(columns+c-radius-1)));
    delta=_mm256_add_epi32(delta,_mm256_slli_si256(delta,4));
    delta=_mm256_add_epi32(delta,_mm256_slli_si256(delta,8));
    const auto bridge=_mm256_inserti128_si256(_mm256_setzero_si256(),
        _mm_shuffle_epi32(_mm256_castsi256_si128(delta),0xff),1);
    const auto totals=_mm256_add_epi32(_mm256_add_epi32(delta,bridge),carry);
    __m128i words;
    if(sizeof(T)==1 && area<=1023) {
      // 整数和精确且分母为奇数，最近半界距离>=1/(2*1023)，大于 byte 尺度的有界 float 误差。
      const auto rounded=_mm256_cvttps_epi32(_mm256_add_ps(_mm256_mul_ps(
          _mm256_cvtepi32_ps(totals),_mm256_set1_ps(static_cast<float>(inverse))),_mm256_set1_ps(.5f)));
      words=_mm_packus_epi32(_mm256_castsi256_si128(rounded),_mm256_extracti128_si256(rounded,1));
    } else words=_mm_packus_epi32(normalize(_mm256_castsi256_si128(totals)),normalize(_mm256_extracti128_si256(totals,1)));
    if constexpr(sizeof(T)==1)_mm_storel_epi64(reinterpret_cast<__m128i*>(out+c),_mm_packus_epi16(words,words));
    else _mm_storeu_si128(reinterpret_cast<__m128i*>(out+c),words);
    carry=_mm256_permutevar8x32_epi32(totals,_mm256_set1_epi32(7));
  }
  sum=static_cast<std::uint32_t>(_mm_cvtsi128_si32(_mm256_castsi256_si128(carry)));
  for(;c<width;++c) {
    sum=sum-columns[leaving[c-1]]+columns[entering[c-1]];
    out[c]=static_cast<T>(double(sum)*inverse+.5);
  }
}
}
void box_columns_avx2(std::uint32_t* columns,const void* before,const void* after,std::size_t width,core::PixelType type) {
  if(type==core::PixelType::UInt8)box_columns(columns,static_cast<const std::uint8_t*>(before),static_cast<const std::uint8_t*>(after),width);
  else box_columns(columns,static_cast<const std::uint16_t*>(before),static_cast<const std::uint16_t*>(after),width);
}
void box_row_avx2(const std::uint32_t* columns,const std::size_t* leaving,const std::size_t* entering,
                  void* out,std::size_t width,std::size_t radius,std::uint32_t area,std::uint32_t sum,core::PixelType type) {
  if(type==core::PixelType::UInt8)box_row(columns,leaving,entering,static_cast<std::uint8_t*>(out),width,radius,area,sum);
  else box_row(columns,leaving,entering,static_cast<std::uint16_t*>(out),width,radius,area,sum);
}

// 独立子树缩短依赖链，不改变 Byte 极值。
template<bool Maximum, std::size_t Count, class Load>
__m256i morphology_reduce(Load&& load, std::size_t first=0) {
  if constexpr(Count==1) return load(first);
  else {
    const auto a=morphology_reduce<Maximum,Count/2>(load,first);
    const auto b=morphology_reduce<Maximum,Count-Count/2>(load,first+Count/2);
    return Maximum?_mm256_max_epu8(a,b):_mm256_min_epu8(a,b);
  }
}

template<bool Maximum, std::size_t FixedMask = 0>
void morphology_u8_impl(const core::Image& image, core::Image& output,
                        std::size_t channel, std::size_t requested_width,
                        std::size_t requested_height) {
  constexpr bool maximum=Maximum;
  const auto mask_width=FixedMask?FixedMask:requested_width;
  const auto mask_height=FixedMask?FixedMask:requested_height;
  ExecutionContext context;
  const auto width=image.width(), height=image.height(), radius=mask_width/2;
  const auto* input=static_cast<const std::uint8_t*>(image.data(channel));
  auto* out=static_cast<std::uint8_t*>(output.mutable_data(channel));
  const auto in_stride=image.stride(channel), out_stride=output.stride(channel);
  auto combine=[&](__m256i a,__m256i b){return maximum?_mm256_max_epu8(a,b):_mm256_min_epu8(a,b);};
  ExecutionContext::for_each_partition(height,width*height,ExecutionContext::kFilterParallelWork,[&](auto range){
    ExecutionContext scratch;
    std::pmr::vector<std::uint8_t> ring(width*mask_height,scratch.scratch_resource());
    std::pmr::vector<std::uint8_t> padded(width+mask_width-1,scratch.scratch_resource());
    std::array<std::int64_t,31> cached{};cached.fill(-1);
    std::array<const std::uint8_t*,31> rows{};
    for(auto y=range.begin;y<range.end;++y) {
      for(std::size_t i=0;i<mask_height;++i) {
        const auto source=std::size_t(std::clamp<std::int64_t>(std::int64_t(y)+std::int64_t(i)-std::int64_t(mask_height/2),0,std::int64_t(height)-1));
        const auto slot=source%mask_height;
        auto* filtered=ring.data()+slot*width;rows[i]=filtered;
        if(cached[slot]==static_cast<std::int64_t>(source))continue;
        const auto* row=input+source*in_stride;
        std::fill_n(padded.data(),radius,row[0]);
        std::memcpy(padded.data()+radius,row,width);
        std::fill_n(padded.data()+radius+width,radius,row[width-1]);
        std::size_t x=0;
        for(;x+32<=width;x+=32) {
          __m256i v;
          if constexpr(FixedMask) v=morphology_reduce<Maximum,FixedMask>([&](auto k){return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(padded.data()+x+k));});
          else {
            v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(padded.data()+x));
            for(std::size_t k=1;k<mask_width;++k)v=combine(v,_mm256_loadu_si256(reinterpret_cast<const __m256i*>(padded.data()+x+k)));
          }
          _mm256_storeu_si256(reinterpret_cast<__m256i*>(filtered+x),v);
        }
        for(;x<width;++x){auto v=padded[x];for(std::size_t k=1;k<mask_width;++k)v=maximum?std::max(v,padded[x+k]):std::min(v,padded[x+k]);filtered[x]=v;}
        cached[slot]=static_cast<std::int64_t>(source);
      }
      std::size_t x=0;
      for(;x+32<=width;x+=32) {
        __m256i v;
        if constexpr(FixedMask) v=morphology_reduce<Maximum,FixedMask>([&](auto i){return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[i]+x));});
        else {
          v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[0]+x));
          for(std::size_t i=1;i<mask_height;++i)v=combine(v,_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[i]+x)));
        }
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(out+y*out_stride+x),v);
      }
      for(;x<width;++x){auto v=rows[0][x];for(std::size_t i=1;i<mask_height;++i)v=maximum?std::max(v,rows[i][x]):std::min(v,rows[i][x]);out[y*out_stride+x]=v;}
    }
  });
}
void morphology_u8_avx2(const core::Image& image,core::Image& output,std::size_t channel,
                        std::size_t width,std::size_t height,bool maximum) {
  if(width==height && width==5) {
    if(maximum)morphology_u8_impl<true,5>(image,output,channel,width,height);
    else morphology_u8_impl<false,5>(image,output,channel,width,height);
    return;
  }
  if(width==height && width==15) {
    if(maximum)morphology_u8_impl<true,15>(image,output,channel,width,height);
    else morphology_u8_impl<false,15>(image,output,channel,width,height);
    return;
  }
  if(maximum)morphology_u8_impl<true>(image,output,channel,width,height);
  else morphology_u8_impl<false>(image,output,channel,width,height);
}
} // namespace mv::imageproc

#include <imageproc/filters_avx2.hpp>
#include <cmath>

#if defined(_MSC_VER)
#pragma float_control(precise, on, push)
#pragma fp_contract(off)
#endif
namespace mv::imageproc {
namespace {
void log_store(std::int16_t* output, double sum, double divisor) {
  const double reduced = std::fmod(std::trunc(sum / divisor), 65536.0);
  const auto bits = static_cast<std::uint16_t>(static_cast<std::int32_t>(reduced));
  std::memcpy(output, &bits, sizeof(bits));
}
}
std::size_t log_double_row_avx2(const double* center,
    std::span<const std::ptrdiff_t> offsets, std::span<const int> weights,
    std::int16_t* output, std::size_t width, double divisor) {
  std::size_t c = 0;
  for (; width - c >= 8; c += 8) {
    auto first = _mm256_setzero_pd(), second = _mm256_setzero_pd();
    for (std::size_t i = 0; i < weights.size(); ++i) {
      const auto coefficient = _mm256_set1_pd(weights[i]);
      const auto* source = center + static_cast<std::ptrdiff_t>(c) + offsets[i];
      first = _mm256_add_pd(first, _mm256_mul_pd(coefficient, _mm256_loadu_pd(source)));
      second = _mm256_add_pd(second, _mm256_mul_pd(coefficient, _mm256_loadu_pd(source + 4)));
    }
    double sums[8];
    _mm256_storeu_pd(sums, first);
    _mm256_storeu_pd(sums + 4, second);
    for (std::size_t j = 0; j < 8; ++j) log_store(output + c + j, sums[j], divisor);
  }
  return c;
}

// 调用方证明 255*sum(abs(weight))<=INT32_MAX；每个乘积及累加前缀在 Int32 和原 Double 中均精确。
void log_byte_image_avx2(const core::Image& image, core::Image& output,
    std::size_t channel, std::size_t radius, std::span<const std::ptrdiff_t> offsets,
    std::span<const int> weights, double divisor) {
  ExecutionContext context;
  const auto width = image.width(), height = image.height(), pitch = width + 2 * radius;
  std::pmr::vector<std::int32_t> padded(pitch * (height + 2 * radius), context.scratch_resource());
  const auto* source = static_cast<const std::uint8_t*>(image.data(channel));
  for (std::size_t r = 0; r < height; ++r) {
    auto* row = padded.data() + (r + radius) * pitch;
    for (std::size_t c = 0; c < width; ++c) row[c + radius] = source[r * image.stride(channel) + c];
    for (std::size_t c = 0; c < radius; ++c) {
      row[c] = row[radius + *border_index(static_cast<std::int64_t>(c) - static_cast<std::int64_t>(radius), width, BorderMode::Mirrored)];
      row[width + radius + c] = row[radius + *border_index(static_cast<std::int64_t>(width + c), width, BorderMode::Mirrored)];
    }
  }
  for (std::size_t r = 0; r < radius; ++r) {
    std::copy_n(padded.data() + (radius + *border_index(static_cast<std::int64_t>(r) - static_cast<std::int64_t>(radius), height, BorderMode::Mirrored)) * pitch,
                pitch, padded.data() + r * pitch);
    std::copy_n(padded.data() + (radius + *border_index(static_cast<std::int64_t>(height + r), height, BorderMode::Mirrored)) * pitch,
                pitch, padded.data() + (height + radius + r) * pitch);
  }
  auto* destination = static_cast<std::byte*>(output.mutable_data(channel));
  ExecutionContext::for_each_partition(height, ExecutionContext::saturated_work(width * height, weights.size()),
      ExecutionContext::kFilterParallelWork, [&](auto range) {
    for (auto r = range.begin; r < range.end; ++r) {
      auto* row = reinterpret_cast<std::int16_t*>(destination + r * output.stride(channel));
      const auto* center = padded.data() + (r + radius) * pitch + radius;
      std::size_t c = 0;
      for (; width - c >= 8; c += 8) {
        auto sum = _mm256_setzero_si256();
        for (std::size_t i = 0; i < weights.size(); ++i) {
          const auto samples = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(center + static_cast<std::ptrdiff_t>(c) + offsets[i]));
          sum = _mm256_add_epi32(sum, _mm256_mullo_epi32(samples, _mm256_set1_epi32(weights[i])));
        }
        std::int32_t sums[8];
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(sums), sum);
        for (std::size_t j = 0; j < 8; ++j) log_store(row + c + j, sums[j], divisor);
      }
      for (; c < width; ++c) {
        std::int32_t sum = 0;
        for (std::size_t i = 0; i < weights.size(); ++i) sum += weights[i] * center[static_cast<std::ptrdiff_t>(c) + offsets[i]];
        log_store(row + c, sum, divisor);
      }
    }
  });
}
}  // namespace mv::imageproc
#if defined(_MSC_VER)
#pragma float_control(pop)
#endif
