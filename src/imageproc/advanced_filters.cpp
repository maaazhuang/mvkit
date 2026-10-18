#include <runtime/memory/overwrite_buffer.hpp>
#include <imageproc/advanced.hpp>
#include <imageproc/filters_avx2.hpp>
#include <imageproc/box_mean.hpp>
#include <runtime/cpu/cpu_features.hpp>

#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_FILTER_SSE2 1
#else
#define MVKIT_FILTER_SSE2 0
#endif

namespace mv::imageproc {
namespace {
using namespace advanced;
// 单调队列逐行线性工作，worker 独立拥有有界 scratch。
template <class Read, class Write>
void extrema_line(std::size_t n, std::size_t mask, bool maximum,
                  Read&& read_value, Write&& write_value) {
  ExecutionContext context;
  const auto length = n + mask - 1, radius = mask / 2;
  std::pmr::vector<std::size_t> indices(length, context.scratch_resource());
  std::pmr::vector<double> values(length, context.scratch_resource());
  std::size_t begin = 0, end = 0;
  for (std::size_t i = 0; i < length; ++i) {
    const auto pos = static_cast<std::size_t>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(i) - static_cast<std::int64_t>(radius), 0,
        n - 1));
    const double value = read_value(pos);
    while (begin < end &&
           (maximum ? values[end - 1] <= value : values[end - 1] >= value))
      --end;
    indices[end] = i;
    values[end++] = value;
    while (begin < end && i - indices[begin] >= mask) ++begin;
    if (i + 1 >= mask) write_value(i + 1 - mask, values[begin]);
  }
}
Image morphology(const Image& a, std::size_t w, std::size_t h, bool maximum) {
  image(a, "gray morphology");
  window(w, h, "gray morphology");
  auto out = make_full_output_for_overwrite(a);
#if MVKIT_FILTER_HAS_AVX2
  if(w<=31 && h<=31 && mv::runtime::cpu_features().selection().avx2) {
    bool bytes=true;
    for(std::size_t ch=0;ch<a.channel_count();++ch)bytes=bytes&&a.pixel_type(ch)==PixelType::UInt8;
    if(bytes){
      for(std::size_t ch=0;ch<a.channel_count();++ch)morphology_u8_avx2(a,out,ch,w,h,maximum);
      return out;
    }
  }
#endif
  ExecutionContext context;
  std::pmr::vector<double> temp(a.width() * a.height(),
                                context.scratch_resource());
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
    ExecutionContext::for_each_partition(
        a.height(), a.width() * a.height(),
        ExecutionContext::kFilterParallelWork, [&](auto range) {
          for (auto r = range.begin; r < range.end; ++r)
            extrema_line(
                a.width(), w, maximum,
                [&](auto c) { return finite_read(a, ch, r, c); },
                [&](auto c, double v) { temp[r * a.width() + c] = v; });
        });
    auto* data = static_cast<std::byte*>(out.mutable_data(ch));
    dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(
          a.width(), a.width() * a.height(),
          ExecutionContext::kFilterParallelWork, [&](auto range) {
            for (auto c = range.begin; c < range.end; ++c)
              extrema_line(
                  a.height(), h, maximum,
                  [&](auto r) { return temp[r * a.width() + c]; },
                  [&](auto r, double v) {
                    reinterpret_cast<T*>(data + r * out.stride(ch))[c] =
                        saturate_cast<T>(v);
                  });
          });
    });
  }
  return out;
}
Image composite_morph(const Image& a, std::size_t w, std::size_t h,
                      bool closing) {
  auto first = morphology(a, w, h, closing);
  return morphology(first, w, h, !closing);
}

using Kernel =
    std::vector<double>;  // 控制元数据有界：Gaussian 最多 193 项，box 最多 4095 项系数。
Kernel gaussian(double sigma, std::size_t order, std::size_t size = 0) {
  // HALCON 将 Gaussian 导数在像素单元积分，并将截断尾部合入最外侧 tap。
  const auto radius =
      size ? size / 2
           : static_cast<std::size_t>(order == 0 ? std::floor(3.0 * sigma) + 1.0
                                                : order == 1 ? std::ceil(3.5 * sigma)
                                                             : std::round(4.0 * sigma));
  Kernel g(2 * radius + 1);
  if (order <= 2 && !size) {
    const double inv_sqrt2_sigma = 1.0 / (std::sqrt(2.0) * sigma);
    const double inv_normalizer = 1.0 / (2.5066282746310005 * sigma);
    const auto derivative_at = [&](double x) {
      return -x * inv_normalizer / (sigma * sigma) *
             std::exp(-(x * x) / (2.0 * sigma * sigma));
    };
    for (std::size_t i = 0; i < g.size(); ++i) {
      const double x = static_cast<double>(i) - static_cast<double>(radius);
      g[i] = order == 0 ?
                 0.5 * (std::erf((x + 0.5) * inv_sqrt2_sigma) -
                        std::erf((x - 0.5) * inv_sqrt2_sigma)) :
             order == 1 ?
                 inv_normalizer *
                     (std::exp(-((x - 0.5) * (x - 0.5)) /
                               (2.0 * sigma * sigma)) -
                      std::exp(-((x + 0.5) * (x + 0.5)) /
                               (2.0 * sigma * sigma))) :
                 derivative_at(x + 0.5) - derivative_at(x - 0.5);
      if (i == 0 || i + 1 == g.size()) {
        const double edge = static_cast<double>(radius) - 0.5;
        g[i] = order == 0
                   ? 0.5 * std::erfc(edge * inv_sqrt2_sigma)
                   : order == 1
                         ? (i == 0 ? -1.0 : 1.0) * inv_normalizer *
                               std::exp(-(edge * edge) / (2.0 * sigma * sigma))
                         : (i == 0 ? derivative_at(-edge) : -derivative_at(edge));
      }
    }
    return g;
  }
  double sum = 0;
  for (std::size_t i = 0; i < g.size(); ++i) {
    const auto x = double(i) - radius;
    g[i] = std::exp(-x * x / (2 * sigma * sigma));
    sum += g[i];
  }
  for (auto& v : g) v /= sum;
  return g;
}
Image convolve(const Image& a, const Kernel& x, const Kernel& y, bool real) {
  image(a, "convolution");
  const auto width = a.width(), height = a.height();
  std::vector<PixelType> output_types(a.channel_count());
  for(std::size_t ch=0;ch<output_types.size();++ch)output_types[ch]=real ? PixelType::Float32 : a.pixel_type(ch);
  auto out = Image::create_uninitialized(width,height,output_types);
  ExecutionContext context;
  std::pmr::vector<double> temp(context.scratch_resource());
  const auto& cpu = context.runtime().cpu();
  const bool simd = MVKIT_FILTER_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
      {true, false, false, false});
#if MVKIT_FILTER_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false});
#endif
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
#if MVKIT_FILTER_HAS_AVX2
    if(real && avx2 && !a.has_reduced_domain() &&
        (a.pixel_type(ch)==PixelType::UInt8 || a.pixel_type(ch)==PixelType::UInt16)) {
      convolve_integer_to_float_avx2(a,out,ch,x,y);continue;
    }
#endif
    if(temp.empty())temp.resize(width*height);
    const auto* source = static_cast<const std::byte*>(a.data(ch));
    const auto stride = a.stride(ch), out_stride = out.stride(ch);
    dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(height,
          ExecutionContext::saturated_work(width * height, x.size()),
          ExecutionContext::kFilterParallelWork, [&](auto range) {
        ExecutionContext worker;
        const auto radius = x.size() / 2;
        std::pmr::vector<double> padded(width + 2 * radius, worker.scratch_resource());
        for (auto r = range.begin; r < range.end; ++r) {
          for (std::size_t c = 0; c < width; ++c) {
            T value;
            std::memcpy(&value, source + r * stride + c * sizeof(T), sizeof(T));
            check(std::isfinite(double(value)), "image processing", "Image", "nonfinite pixel is not supported");
            padded[c + radius] = double(value);
          }
          for (std::size_t i = 0; i < radius; ++i) {
            padded[i] = padded[radius + *border_index(static_cast<std::int64_t>(i) - static_cast<std::int64_t>(radius), width, BorderMode::Mirrored)];
            padded[width + radius + i] = padded[radius + *border_index(static_cast<std::int64_t>(width + i), width, BorderMode::Mirrored)];
          }
          auto* destination = temp.data() + r * width;
#if MVKIT_FILTER_HAS_AVX2
          if(avx2){convolve_double_row_avx2(padded.data(),x,destination,width);continue;}
#endif
          std::size_t c = 0;
#if MVKIT_FILTER_SSE2
          if (simd) for (; width - c >= 2; c += 2) {
            auto sum = _mm_setzero_pd();
            for (std::size_t i = 0; i < x.size(); ++i)
              sum = _mm_add_pd(sum, _mm_mul_pd(_mm_set1_pd(x[i]), _mm_loadu_pd(padded.data() + c + i)));
            _mm_storeu_pd(destination + c, sum);
          }
#endif
          for (; c < width; ++c) {
            double sum = 0;
            for (std::size_t i = 0; i < x.size(); ++i) sum += x[i] * padded[c + i];
            destination[c] = sum;
          }
        }
      });
    });
    auto* data = static_cast<std::byte*>(out.mutable_data(ch));
    dispatch_scalar(out.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(height,
          ExecutionContext::saturated_work(width * height, y.size()),
          ExecutionContext::kFilterParallelWork, [&](auto range) {
        ExecutionContext worker;
        std::pmr::vector<const double*> rows(y.size(), worker.scratch_resource());
        for (auto r = range.begin; r < range.end; ++r) {
          for (std::size_t i = 0; i < y.size(); ++i)
            rows[i] = temp.data() + *border_index(static_cast<std::int64_t>(r) + static_cast<std::int64_t>(i) - static_cast<std::int64_t>(y.size() / 2), height, BorderMode::Mirrored) * width;
          auto* row = reinterpret_cast<T*>(data + r * out_stride);
#if MVKIT_FILTER_HAS_AVX2
          if constexpr(std::is_same_v<T,float>) {
            if(avx2){convolve_float_rows_avx2(rows,y,row,width);continue;}
          }
#endif
          std::size_t c = 0;
#if MVKIT_FILTER_SSE2
          if (simd) for (; width - c >= 2; c += 2) {
            auto sum = _mm_setzero_pd();
            for (std::size_t i = 0; i < y.size(); ++i)
              sum = _mm_add_pd(sum, _mm_mul_pd(_mm_set1_pd(y[i]), _mm_loadu_pd(rows[i] + c)));
            double pair[2]; _mm_storeu_pd(pair, sum);
            row[c] = saturate_cast<T>(pair[0]); row[c + 1] = saturate_cast<T>(pair[1]);
          }
#endif
          for (; c < width; ++c) {
            double sum = 0;
            for (std::size_t i = 0; i < y.size(); ++i) sum += y[i] * rows[i][c];
            row[c] = saturate_cast<T>(sum);
          }
        }
      });
    });
  }
  return out;
}
}  // namespace

mv::core::Image gray_erosion_rect(const mv::core::Image& a, std::size_t w,
                                  std::size_t h) {
  return keep_domain(morphology(a, w, h, false), a.domain());
}
mv::core::Image gray_dilation_rect(const mv::core::Image& a, std::size_t w,
                                   std::size_t h) {
  return keep_domain(morphology(a, w, h, true), a.domain());
}
mv::core::Image gray_opening_rect(const mv::core::Image& a, std::size_t w,
                                  std::size_t h) {
  return keep_domain(composite_morph(a, w, h, false), a.domain());
}
mv::core::Image gray_closing_rect(const mv::core::Image& a, std::size_t w,
                                  std::size_t h) {
  return keep_domain(composite_morph(a, w, h, true), a.domain());
}
mv::core::Image top_hat(const mv::core::Image& a, std::size_t w,
                        std::size_t h) {
  auto opened = composite_morph(a, w, h, false);
  return sub_image(a, opened, 1, 0);
}
mv::core::Image bottom_hat(const mv::core::Image& a, std::size_t w,
                           std::size_t h) {
  auto closed = composite_morph(a, w, h, true);
  return sub_image(closed, a, 1, 0);
}
namespace {
template<class Output>
bool gaussian_gradient_pair_impl(const Image& a,double s,double scale,
    Output& gx,Output& gy) {
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  if(a.channel_count()!=1 || a.has_reduced_domain() ||
      (a.pixel_type()!=PixelType::UInt8 && a.pixel_type()!=PixelType::UInt16) ||
      !mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),
                                  {false,false,true,false}))return false;
  image(a,"convolution");sigma(s,"derivate_gauss");
  const auto smooth=gaussian(s,0),derivative=gaussian(s,1);
  const auto width=a.width(),height=a.height(),r0=smooth.size()/2,r1=derivative.size()/2;
  const auto radius=std::max(r0,r1),taps=2*radius+1;
  if constexpr(std::is_same_v<Output,mv::runtime::OverwriteBuffer<double>>) {
    // 成对内核在 NMS 读取前写完全部行列。
    gx.reset_for_overwrite(width*height);gy.reset_for_overwrite(width*height);
  } else {
    gx.resize(width*height);gy.resize(width*height);
  }
  {
    const auto* data=static_cast<const std::byte*>(a.data());
    ExecutionContext::for_each_partition(height,ExecutionContext::saturated_work(width*height,taps),
        ExecutionContext::kFilterParallelWork,[&](auto range) {
      mv::runtime::ScratchArena local;auto* memory=local.resource();
      mv::runtime::OverwriteBuffer<double> h0(taps*width,memory),h1(taps*width,memory),padded(width+2*radius,memory);
      std::pmr::vector<std::size_t> cached(taps,SIZE_MAX,memory);
      std::pmr::vector<const double*> rows0(smooth.size(),memory),rows1(derivative.size(),memory);
      const auto prepare=[&](std::size_t row) {
        const auto slot=row%taps;
        if(cached[slot]==row)return;
        const auto* source=data+row*a.stride();
        if(a.pixel_type()==PixelType::UInt8)for(std::size_t c=0;c<width;++c)
          padded[radius+c]=double(std::to_integer<std::uint8_t>(source[c]));
        else for(std::size_t c=0;c<width;++c) {
          std::uint16_t value;std::memcpy(&value,source+c*sizeof(value),sizeof(value));
          padded[radius+c]=double(value);
        }
        for(std::size_t c=0;c<radius;++c) {
          padded[c]=padded[radius+*border_index(std::int64_t(c)-std::int64_t(radius),width,BorderMode::Mirrored)];
          padded[width+radius+c]=padded[radius+*border_index(std::int64_t(width+c),width,BorderMode::Mirrored)];
        }
        convolve_double_pair_row_avx2(padded.data(),smooth,derivative,h0.data()+slot*width,h1.data()+slot*width,width);
        cached[slot]=row;
      };
      for(auto r=range.begin;r<range.end;++r) {
        for(std::size_t k=0;k<taps;++k)prepare(*border_index(
            std::int64_t(r)+std::int64_t(k)-std::int64_t(radius),height,BorderMode::Mirrored));
        for(std::size_t k=0;k<smooth.size();++k) {
          const auto row=*border_index(std::int64_t(r)+std::int64_t(k)-std::int64_t(r0),height,BorderMode::Mirrored);
          rows0[k]=h1.data()+(row%taps)*width;
        }
        for(std::size_t k=0;k<derivative.size();++k) {
          const auto row=*border_index(std::int64_t(r)+std::int64_t(k)-std::int64_t(r1),height,BorderMode::Mirrored);
          rows1[k]=h0.data()+(row%taps)*width;
        }
        convolve_float_scaled_pair_rows_avx2(rows0,smooth,rows1,derivative,
            gx.data()+r*width,gy.data()+r*width,width,scale);
      }
    });
  }
  return true;
#else
  (void)a;(void)s;(void)scale;(void)gx;(void)gy;return false;
#endif
}
} // namespace
bool advanced::gaussian_gradient_pair(const Image& a,double s,double scale,
    std::pmr::vector<double>& gx,std::pmr::vector<double>& gy) {
  return gaussian_gradient_pair_impl(a,s,scale,gx,gy);
}
bool advanced::gaussian_gradient_pair(const Image& a,double s,double scale,
    mv::runtime::OverwriteBuffer<double>& gx,mv::runtime::OverwriteBuffer<double>& gy) {
  return gaussian_gradient_pair_impl(a,s,scale,gx,gy);
}
mv::core::Image derivate_gauss(const mv::core::Image& a, double s,
                               std::size_t ox, std::size_t oy) {
  return derivate_gauss(a,s,s,ox,oy);
}
mv::core::Image derivate_gauss(const mv::core::Image& a,double sx,double sy,
                               std::size_t ox,std::size_t oy) {
  advanced::sigma(sx, "derivate_gauss");advanced::sigma(sy, "derivate_gauss");
  check(ox <= 2 && oy <= 2 && ox + oy <= 2, "derivate_gauss", "Order",
        "total order must be 0,1 or 2");
  return keep_domain(convolve(a, gaussian(sx, ox), gaussian(sy, oy), true),
                     a.domain());
}
static mv::core::Image laplace_of_gauss_impl(const mv::core::Image& a,double s,bool canonical) {
  image(a, "laplace_of_gauss");
  sigma(s, "laplace_of_gauss");
  // HALCON 将采样 LoG 量化为中心 -1000 的整数掩码；节点来自实际输出，
  // 已认证 sigma 节点之间线性插值。
  constexpr std::array<std::pair<double, double>, 14> normalization{{
      {0.5, 3606}, {0.75, 6083}, {1.0, 7550}, {1.25, 9849},
      {1.5, 11705}, {1.75, 13601}, {2.0, 15395}, {2.25, 17464},
      {2.5, 19313}, {3.0, 23000}, {3.5, 26851}, {4.0, 30545},
      {5.0, 38171}, {7.0, 53376}}};
  double divisor = 0;
  if (s <= normalization.front().first) {
    divisor = normalization.front().second * s / normalization.front().first;
  } else if (s >= normalization.back().first) {
    divisor = normalization.back().second * s / normalization.back().first;
  } else {
    const auto upper = std::lower_bound(
        normalization.begin(), normalization.end(), s,
        [](const auto& item, double value) { return item.first < value; });
    const auto& lower = *(upper - 1);
    const double fraction = (s - lower.first) / (upper->first - lower.first);
    divisor = lower.second + fraction * (upper->second - lower.second);
  }
  struct Tap { std::int64_t dy, dx; int weight; };
  ExecutionContext context;
  std::pmr::vector<Tap> taps{context.scratch_resource()};
  const auto radius = static_cast<std::int64_t>(std::round(4 * s));
  for (std::int64_t dy = -radius; dy <= radius; ++dy)
    for (std::int64_t dx = -radius; dx <= radius; ++dx) {
      const double t = double(dx * dx + dy * dy) / (2 * s * s);
      const int weight = static_cast<int>(std::round(1000 * (t - 1) * std::exp(-t)));
      if (weight) taps.push_back({dy, dx, weight});
    }
  std::pmr::vector<PixelType> types(a.channel_count(), PixelType::Int16,
                                    context.scratch_resource());
  auto out = Image::create(a.width(), a.height(), types);
  const auto width = a.width(), height = a.height();
  const auto pitch = width + 2 * static_cast<std::size_t>(radius);
  const auto padded_height = height + 2 * static_cast<std::size_t>(radius);
  std::pmr::vector<double> padded(context.scratch_resource());
  std::pmr::vector<std::ptrdiff_t> offsets(context.scratch_resource());
  for (const auto& tap : taps) offsets.push_back(tap.dy * static_cast<std::ptrdiff_t>(pitch) + tap.dx);
  std::pmr::vector<int> weights(context.scratch_resource());
  weights.reserve(taps.size());
  std::uint64_t absolute_weights = 0;
  for (const auto& tap : taps) {
    weights.push_back(tap.weight);
    const auto weight = static_cast<std::int64_t>(tap.weight);
    absolute_weights += static_cast<std::uint64_t>(weight < 0 ? -weight : weight);
  }
  const auto& cpu = context.runtime().cpu();
  const bool wide = MVKIT_FILTER_HAS_AVX2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
      {false, true, false, false});
  const bool simd = MVKIT_FILTER_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
      {true, false, false, false});
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
#if MVKIT_FILTER_HAS_AVX2
    // Int32 padding 降低大 Byte 栅格流量；隔离诊断中小栅格使用原序 Double 内核更快。
    if (wide && width * height >= 1048576 && a.pixel_type(ch) == PixelType::UInt8 &&
        absolute_weights <= static_cast<std::uint64_t>(INT32_MAX) / 255U) {
      log_byte_image_avx2(a, out, ch, static_cast<std::size_t>(radius), offsets, weights, divisor);
      continue;
    }
#endif
    padded.resize(pitch * padded_height);
    const auto* source = static_cast<const std::byte*>(a.data(ch));
    const auto stride = a.stride(ch), out_stride = out.stride(ch);
    auto* destination = static_cast<std::byte*>(out.mutable_data(ch));
    dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      for (std::size_t r = 0; r < height; ++r) {
        auto* row = padded.data() + (r + radius) * pitch;
        for (std::size_t c = 0; c < width; ++c) {
          T value; std::memcpy(&value, source + r * stride + c * sizeof(T), sizeof(T));
          check(std::isfinite(double(value)), "laplace_of_gauss", "Image", "nonfinite pixel is not supported");
          row[c + radius] = double(value);
        }
        for (std::int64_t c = 0; c < radius; ++c) {
          row[c] = row[radius + *border_index(c - radius, width, BorderMode::Mirrored)];
          row[width + radius + c] = row[radius + *border_index(static_cast<std::int64_t>(width) + c, width, BorderMode::Mirrored)];
        }
      }
      for (std::int64_t r = 0; r < radius; ++r) {
        std::copy_n(padded.data() + (radius + *border_index(r - radius, height, BorderMode::Mirrored)) * pitch, pitch, padded.data() + r * pitch);
        std::copy_n(padded.data() + (radius + *border_index(static_cast<std::int64_t>(height) + r, height, BorderMode::Mirrored)) * pitch, pitch, padded.data() + (height + radius + r) * pitch);
      }
      ExecutionContext::for_each_partition(height,
          ExecutionContext::saturated_work(width * height, taps.size()),
          ExecutionContext::kFilterParallelWork, [&](auto range) {
        for (auto r = range.begin; r < range.end; ++r) {
          auto* row = reinterpret_cast<std::int16_t*>(destination + r * out_stride);
          const auto* center = padded.data() + (r + radius) * pitch + radius;
          const auto store = [&](std::size_t c, double sum) {
            const double reduced = std::fmod(std::trunc(sum / divisor), 65536.0);
            const auto bits = static_cast<std::uint16_t>(static_cast<std::int32_t>(reduced));
            std::memcpy(&row[c], &bits, sizeof(bits));
          };
          if constexpr(std::is_same_v<T,std::int32_t>) {
            if(canonical) {
            // HALCON int4 LoG 累加按 32 位模算术；使用无符号操作复现，避免 C++ 有符号 UB。
            for(std::size_t c=0;c<width;++c) {
              std::uint32_t bits=0;
              for(std::size_t i=0;i<taps.size();++i)
                bits+=static_cast<std::uint32_t>(static_cast<std::int64_t>(center[static_cast<std::ptrdiff_t>(c)+offsets[i]])*weights[i]);
              std::int32_t value;std::memcpy(&value,&bits,sizeof(value));
              store(c,static_cast<double>(value));
            }
            continue;
            }
          }
          std::size_t c = 0;
#if MVKIT_FILTER_HAS_AVX2
          if (wide) c = log_double_row_avx2(center, offsets, weights, row, width, divisor);
#endif
#if MVKIT_FILTER_SSE2
          if (simd) for (; width - c >= 2; c += 2) {
            auto sum = _mm_setzero_pd();
            for (std::size_t i = 0; i < taps.size(); ++i)
              sum = _mm_add_pd(sum, _mm_mul_pd(_mm_set1_pd(taps[i].weight),
                  _mm_loadu_pd(center + static_cast<std::ptrdiff_t>(c) + offsets[i])));
            double pair[2]; _mm_storeu_pd(pair, sum);
            store(c, pair[0]); store(c + 1, pair[1]);
          }
#endif
          for (; c < width; ++c) {
            double sum = 0;
            for (std::size_t i = 0; i < taps.size(); ++i)
              sum += taps[i].weight * center[static_cast<std::ptrdiff_t>(c) + offsets[i]];
            store(c, sum);
          }
        }
      });
    });
  }
  return keep_domain(std::move(out), a.domain());
}
mv::core::Image laplace_of_gauss(const mv::core::Image& a,double s) {return laplace_of_gauss_impl(a,s,false);}
mv::core::Image laplace_of_gauss_standard(const mv::core::Image& a,double s) {return laplace_of_gauss_impl(a,s,true);}
mv::core::Image emphasize(const mv::core::Image &a, std::size_t w,
                          std::size_t h, double factor) {
  window(w, h, "emphasize");
  check(std::isfinite(factor) && factor >= 0, "emphasize", "Factor",
        "factor must be finite and nonnegative");
  auto prepared = exact_integer_box_mean(a,w,h);
  auto mean = prepared ? std::move(*prepared) : convolve(a, Kernel(w, 1. / w), Kernel(h, 1. / h), true);
  auto out = make_full_output_for_overwrite(a);
  // 卷积已校验全部输入；均值仍保持 Float32 舍入，后处理按行读取。
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
    const auto *source = static_cast<const std::byte *>(a.data(ch));
    const auto *averages = static_cast<const std::byte *>(mean.data(ch));
    auto *destination = static_cast<std::byte *>(out.mutable_data(ch));
    const auto stride = a.stride(ch), mean_stride = mean.stride(ch),
               out_stride = out.stride(ch);
    dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(
          a.height(), a.width() * a.height(),
          ExecutionContext::kArithmeticParallelWork, [&](auto range) {
            for (auto r = range.begin; r < range.end; ++r) {
              const auto *in = source + r * stride;
              const auto *avg =
                  reinterpret_cast<const float *>(averages + r * mean_stride);
              auto *row = reinterpret_cast<T *>(destination + r * out_stride);
              if constexpr(std::is_same_v<T,std::uint8_t>) {
                emphasize_byte_row(reinterpret_cast<const std::uint8_t*>(in),avg,row,a.width(),factor);
                continue;
              }
              for (std::size_t c = 0; c < a.width(); ++c) {
                const double v = double(load_pixel<T>(in + c * sizeof(T)));
                row[c] = saturate_cast<T>(v + factor * (v - double(avg[c])));
              }
            }
          });
    });
  }
  return keep_domain(std::move(out), a.domain());
}
namespace advanced {
Image gaussian_extended(const Image& a, std::size_t size) {
  check(size >= 1 && size <= 63 && (size & 1), "gauss_filter", "Size",
        "odd size in [1,63] required");
  if (size == 1) return keep_domain(a.clone(), a.domain());
  return keep_domain(convolve(a, gaussian(double(size - 1) / 6, 0, size),
                              gaussian(double(size - 1) / 6, 0, size), false),
                     a.domain());
}
Image sobel_extended(const Image& a, SobelFilter filter, std::size_t size) {
  image(a, "sobel_amp");
  check(size >= 3 && size <= 31 && (size & 1), "sobel_amp", "Size",
        "odd size in [3,31] required");
  check(filter == SobelFilter::X || filter == SobelFilter::Y ||
            filter == SobelFilter::SumAbs || filter == SobelFilter::SumSqrt,
        "sobel_amp", "Filter", "invalid filter");
  Kernel smooth(size, 0), base(size - 2, 0), derivative(size, 0);
  smooth[0] = base[0] = 1;
  for (std::size_t i = 1; i < size; ++i)
    smooth[i] = smooth[i - 1] * double(size - i) / i;
  for (std::size_t i = 1; i < base.size(); ++i)
    base[i] = base[i - 1] * double(base.size() - i) / i;
  double sum = 0;
  for (auto v : smooth) sum += v;
  for (auto& v : smooth) v /= sum;
  sum = 0;
  for (auto v : base) sum += v;
  for (std::size_t i = 0; i < base.size(); ++i) {
    derivative[i] += base[i] / sum;
    derivative[i + 2] -= base[i] / sum;
  }
  const auto dx = convolve(a, derivative, smooth, true),
             dy = convolve(a, smooth, derivative, true);
  ExecutionContext context;
  std::pmr::vector<PixelType> types{context.scratch_resource()};
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
    const bool amplitude = filter == SobelFilter::SumAbs ||
                           filter == SobelFilter::SumSqrt;
    types.push_back(amplitude && a.pixel_type(ch) == PixelType::UInt8
                        ? PixelType::UInt8
                    : amplitude || a.pixel_type(ch) == PixelType::Float32
                        ? PixelType::Float32
                    : a.pixel_type(ch) == PixelType::UInt8 ? PixelType::Int16
                                                           : PixelType::Int32);
  }
  auto out = Image::create(a.width(), a.height(), types);
  pixels(out, [&](auto ch, auto r, auto c) {
    const auto x = read(dx, ch, r, c), y = read(dy, ch, r, c);
    if (size == 3 && filter == SobelFilter::SumAbs &&
        a.pixel_type(ch) == PixelType::UInt8)
      return std::floor(std::abs(x) + std::abs(y));
    return filter == SobelFilter::X        ? x
           : filter == SobelFilter::Y      ? y
           : filter == SobelFilter::SumAbs ? std::abs(x) + std::abs(y)
                                           : std::hypot(x, y);
  });
  return keep_domain(std::move(out), a.domain());
}
}  // namespace advanced
}  // namespace mv::imageproc
