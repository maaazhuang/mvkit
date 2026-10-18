#include <imageproc/advanced.hpp>
#include <imageproc/border.hpp>
#include <imageproc/histogram256.hpp>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>
#endif

namespace mv::imageproc {
namespace {
using namespace advanced;
void histogram_image(const Image& a, const char* op) {
  image(a, op);
  check(a.channel_count() == 1 && (a.pixel_type() == PixelType::UInt8 ||
                                   a.pixel_type() == PixelType::UInt16),
        op, "Image", "single-channel UInt8/UInt16 required");
}

Region local_threshold_byte(const Image& a, std::size_t w, std::size_t h,
                            double k, double range, ThresholdPolarity polarity) {
  // 整数列矩保持精确窗口，工作区 O(width)，无需两张完整 Double 积分图和边界重扫。
  ExecutionContext context;
  const auto width = a.width(), height = a.height();
  std::pmr::vector<std::uint64_t> sums(width,0,context.scratch_resource()),
      squares(width,0,context.scratch_resource());
  std::pmr::vector<std::size_t> leaving(width,context.scratch_resource()),
      entering(width,context.scratch_resource());
  const auto row_words=(width-1)/32+1;
  std::pmr::vector<std::uint32_t> mask(row_words*height,0,context.scratch_resource());
  const auto* data = static_cast<const std::uint8_t*>(a.data());
  const auto mirror = [](std::int64_t i, std::size_t n) {
    return border_index(i,n,BorderMode::Mirrored).value();
  };
  const auto rx = static_cast<std::int64_t>(w/2), ry = static_cast<std::int64_t>(h/2);
  for (std::int64_t dy=-ry;dy<=ry;++dy) {
    const auto* row = data+mirror(dy,height)*a.stride();
    for (std::size_t c=0;c<width;++c) { sums[c]+=row[c]; squares[c]+=unsigned(row[c])*row[c]; }
  }
  for (std::size_t c=0;c<width;++c) {
    leaving[c]=mirror(static_cast<std::int64_t>(c)-rx,width);
    entering[c]=mirror(static_cast<std::int64_t>(c)+rx+1,width);
  }
  const double count=static_cast<double>(w*h);
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  const bool avx2=mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false});
#endif
  const auto domain=a.domain();
  const auto runs=domain.runs();
  std::size_t run_index=0;
  for (std::size_t r=0;r<height;++r) {
    if (run_index<runs.size() && runs[run_index].row==static_cast<std::int64_t>(r)) {
    std::uint64_t sum=0,square=0;
    for (std::int64_t dx=-rx;dx<=rx;++dx) {
      const auto c=mirror(dx,width);sum+=sums[c];square+=squares[c];
    }
    const auto* row=data+r*a.stride();
    std::uint32_t word=0;unsigned bit=0;auto word_index=r*row_words;
    std::size_t c=0;
#if MVKIT_FILTER_HAS_AVX2
    if(avx2) {
      alignas(32) double block_sums[32],block_squares[32];
      for(;width-c>=32;c+=32) {
        const auto radius=static_cast<std::size_t>(rx);
        if(w*h<=std::size_t(INT32_MAX)/65025 && c>=radius && width-c>32+radius) {
          mask[word_index++]=local_threshold_u8_window_mask_avx2(row+c,sums.data(),squares.data(),
              c-radius,c+radius+1,&sum,&square,count,k,range,polarity==ThresholdPolarity::Light);
          continue;
        }
        for(std::size_t lane=0;lane<32;++lane) {
          block_sums[lane]=static_cast<double>(sum);
          block_squares[lane]=static_cast<double>(square);
          const auto x=c+lane;
          sum=sum-sums[leaving[x]]+sums[entering[x]];
          square=square-squares[leaving[x]]+squares[entering[x]];
        }
        mask[word_index++]=local_threshold_u8_mask_avx2(row+c,block_sums,block_squares,
            count,k,range,polarity==ThresholdPolarity::Light);
      }
    }
#endif
    for (;c<width;++c) {
      const double mean=static_cast<double>(sum)/count;
      const double variance=std::max(0.,static_cast<double>(square)/count-mean*mean);
      const double factor=k*(std::sqrt(variance)/range-1);
      const double threshold=polarity==ThresholdPolarity::Light
          ? mean-(255.-mean)*factor : mean*(1+factor);
      const bool selected=polarity==ThresholdPolarity::Light ? row[c]>threshold : row[c]<=threshold;
      word|=std::uint32_t(selected)<<bit;
      if(++bit==32){mask[word_index++]=word;word=0;bit=0;}
      sum = sum-sums[leaving[c]]+sums[entering[c]];
      square = square-squares[leaving[c]]+squares[entering[c]];
    }
    if(bit)mask[word_index]=word;
    while (run_index<runs.size() && runs[run_index].row==static_cast<std::int64_t>(r)) ++run_index;
    }
    if (r+1<height) {
      const auto* before=data+mirror(static_cast<std::int64_t>(r)-ry,height)*a.stride();
      const auto* after=data+mirror(static_cast<std::int64_t>(r)+ry+1,height)*a.stride();
      for (std::size_t c=0;c<width;++c) {
        sums[c]=sums[c]-before[c]+after[c];
        squares[c]=squares[c]-unsigned(before[c])*before[c]+unsigned(after[c])*after[c];
      }
    }
  }
  if(domain.area()==width*height)return Region::from_row_masks(domain,mask);
  std::size_t count_words=0;
  for(const auto& run:runs)count_words+=static_cast<std::size_t>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+32)/32;
  std::pmr::vector<std::uint32_t> packed{context.scratch_resource()};
  packed.reserve(count_words);
  for(const auto& run:runs) {
    const auto* row=mask.data()+static_cast<std::size_t>(run.row)*row_words;
    for(auto c=std::int64_t(run.col_begin);;c+=32) {
      const auto index=static_cast<std::size_t>(c)/32;
      const auto shift=static_cast<unsigned>(c)%32;
      auto word=row[index]>>shift;
      if(shift && index+1<row_words)word|=row[index+1]<<(32-shift);
      const auto bits=static_cast<unsigned>(std::min<std::int64_t>(32,std::int64_t(run.col_end)-c+1));
      if(bits<32)word&=(std::uint32_t{1}<<bits)-1;
      packed.push_back(word);
      if(std::int64_t(run.col_end)-c<32)break;
    }
  }
  return Region::from_row_masks(domain,packed);
}
}  // namespace
mv::core::Image trans_from_rgb(const mv::core::Image &a) {
  image(a, "trans_from_rgb");
  check(a.channel_count() == 3, "trans_from_rgb", "Image",
        "three RGB channels required");
  for (std::size_t ch = 0; ch < 3; ++ch)
    check(a.pixel_type(ch) == PixelType::UInt8 ||
              a.pixel_type(ch) == PixelType::UInt16 ||
              a.pixel_type(ch) == PixelType::Float32,
          "trans_from_rgb", "Type", "RGB supports UInt8,UInt16,Float32");
  auto out = create_like(a, a.width(), a.height(), true, !a.has_reduced_domain());
  const auto domain = a.domain();
  if (a.pixel_type(0) == a.pixel_type(1) &&
      a.pixel_type(1) == a.pixel_type(2)) {
    std::array<const std::byte *, 3> source{};
    std::array<std::byte *, 3> destination{};
    std::array<std::size_t, 3> strides{}, out_strides{};
    for (std::size_t ch = 0; ch < 3; ++ch) {
      source[ch] = static_cast<const std::byte *>(a.data(ch));
      strides[ch] = a.stride(ch);
      destination[ch] = static_cast<std::byte *>(out.mutable_data(ch));
      out_strides[ch] = out.stride(ch);
    }
    const double scale = a.pixel_type() == PixelType::UInt8    ? 255.
                         : a.pixel_type() == PixelType::UInt16 ? 65535.
                                                               : 1.;
    const auto runs = domain.runs();
    // 同类型平面一次读取同时生成 HSV；Double 运算及 Float32 输出舍入不变。
#if MVKIT_FILTER_HAS_AVX2
    const auto& color_cpu=mv::runtime::cpu_features();
    const bool color_avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
        color_cpu.selection(),color_cpu.hardware()),{false,true,false,false});
#endif
    dispatch_scalar(a.pixel_type(), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(
          runs.size(), a.width() * a.height(),
          ExecutionContext::kArithmeticParallelWork, [&](auto range) {
            for (auto i = range.begin; i < range.end; ++i) {
              const auto &run = runs[i];
              const auto r = std::size_t(run.row);
              std::array<const std::byte *, 3> in{};
              std::array<float *, 3> row{};
              for (std::size_t ch = 0; ch < 3; ++ch) {
                in[ch] = source[ch] + r * strides[ch];
                row[ch] = reinterpret_cast<float *>(destination[ch] +
                                                    r * out_strides[ch]);
              }
              auto first=std::int64_t(run.col_begin);
#if MVKIT_FILTER_HAS_AVX2
              if constexpr (std::is_same_v<T,std::uint8_t>) {
                if(color_avx2)first=rgb_to_hsv_u8_row_avx2(
                    reinterpret_cast<const std::uint8_t*>(in[0]),
                    reinterpret_cast<const std::uint8_t*>(in[1]),
                    reinterpret_cast<const std::uint8_t*>(in[2]),row[0],row[1],row[2],
                    std::size_t(first),std::size_t(std::int64_t(run.col_end)+1));
              }
#endif
              for (auto c = first; c <= run.col_end;
                   ++c) {
                double rgb[3];
                for (std::size_t ch = 0; ch < 3; ++ch) {
                  const auto value =
                      double(load_pixel<T>(in[ch] + c * sizeof(T)));
                  check(std::isfinite(value), "image processing", "Image",
                        "nonfinite pixel is not supported");
                  rgb[ch] = value / scale;
                  check(rgb[ch] >= 0 && rgb[ch] <= 1, "trans_from_rgb", "Image",
                        "float RGB must be normalized to [0,1]");
                }
                const double maximum = std::max({rgb[0], rgb[1], rgb[2]}),
                             minimum = std::min({rgb[0], rgb[1], rgb[2]}),
                             delta = maximum - minimum;
                double hue = 0.;
                if (delta != 0.) {
                  hue = maximum == rgb[0]   ? (rgb[1] - rgb[2]) / delta
                        : maximum == rgb[1] ? 2 + (rgb[2] - rgb[0]) / delta
                                            : 4 + (rgb[0] - rgb[1]) / delta;
                  hue /= 6;
                  if (hue < 0)
                    hue += 1;
                }
                row[0][c] = static_cast<float>(hue);
                row[1][c] =
                    static_cast<float>(maximum == 0 ? 0. : delta / maximum);
                row[2][c] = static_cast<float>(maximum);
              }
            }
          });
    });
    return out.reduce_domain(domain);
  }
  pixels(out, [&](auto ch, auto r, auto c) {
    if (!domain.contains(r, c))
      return 0.;
    double rgb[3];
    for (int i = 0; i < 3; ++i) {
      rgb[i] = finite_read(a, i, r, c) /
               (a.pixel_type(i) == PixelType::UInt8    ? 255.
                : a.pixel_type(i) == PixelType::UInt16 ? 65535.
                                                       : 1.);
      check(rgb[i] >= 0 && rgb[i] <= 1, "trans_from_rgb", "Image",
            "float RGB must be normalized to [0,1]");
    }
    const double maximum = std::max({rgb[0], rgb[1], rgb[2]}),
                 minimum = std::min({rgb[0], rgb[1], rgb[2]}),
                 delta = maximum - minimum;
    if (ch == 2)
      return maximum;
    if (ch == 1)
      return maximum == 0 ? 0. : delta / maximum;
    if (delta == 0)
      return 0.;
    double hue = maximum == rgb[0]   ? (rgb[1] - rgb[2]) / delta
                 : maximum == rgb[1] ? 2 + (rgb[2] - rgb[0]) / delta
                                     : 4 + (rgb[0] - rgb[1]) / delta;
    hue /= 6;
    if (hue < 0)
      hue += 1;
    return hue;
  });
  return out.reduce_domain(domain);
}
mv::core::Image trans_to_rgb(const mv::core::Image &a,
                             mv::core::PixelType type) {
  image(a, "trans_to_rgb");
  check(a.channel_count() == 3, "trans_to_rgb", "Image",
        "three HSV channels required");
  for (std::size_t ch = 0; ch < 3; ++ch)
    check(a.pixel_type(ch) == PixelType::Float32, "trans_to_rgb", "Image",
          "HSV channels must be Float32");
  check(type == PixelType::UInt8 || type == PixelType::UInt16 ||
            type == PixelType::Float32,
        "trans_to_rgb", "Type", "RGB supports UInt8,UInt16,Float32");
  const std::array<PixelType, 3> types{type, type, type};
  auto out = Image::create(a.width(), a.height(), types);
  const auto domain = a.domain();
  std::array<const std::byte *, 3> source{};
  std::array<std::byte *, 3> destination{};
  std::array<std::size_t, 3> strides{}, out_strides{};
  for (std::size_t ch = 0; ch < 3; ++ch) {
    source[ch] = static_cast<const std::byte *>(a.data(ch));
    strides[ch] = a.stride(ch);
    destination[ch] = static_cast<std::byte *>(out.mutable_data(ch));
    out_strides[ch] = out.stride(ch);
  }
  const double scale = type == PixelType::UInt8    ? 255.
                       : type == PixelType::UInt16 ? 65535.
                                                   : 1.;
  const auto runs = domain.runs();
  dispatch_scalar(type, [&](auto tag) {
    using T = typename decltype(tag)::type;
    ExecutionContext::for_each_partition(
        runs.size(), a.width() * a.height(),
        ExecutionContext::kArithmeticParallelWork, [&](auto range) {
          for (auto i = range.begin; i < range.end; ++i) {
            const auto &run = runs[i];
            const auto r = std::size_t(run.row);
            std::array<const std::byte *, 3> in{};
            std::array<T *, 3> row{};
            for (std::size_t ch = 0; ch < 3; ++ch) {
              in[ch] = source[ch] + r * strides[ch];
              row[ch] =
                  reinterpret_cast<T *>(destination[ch] + r * out_strides[ch]);
            }
            for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
              double h = double(load_pixel<float>(in[0] + c * sizeof(float)));
              const double s = double(
                               load_pixel<float>(in[1] + c * sizeof(float))),
                           v = double(
                               load_pixel<float>(in[2] + c * sizeof(float)));
              check(std::isfinite(h) && std::isfinite(s) && std::isfinite(v),
                    "image processing", "Image",
                    "nonfinite pixel is not supported");
              check(s >= 0 && s <= 1 && v >= 0 && v <= 1, "trans_to_rgb",
                    "Image", "saturation and value must be in [0,1]");
              h = (h - std::floor(h)) * 6;
              const int sector = static_cast<int>(std::floor(h));
              const auto f = h - sector, p = v * (1 - s), q = v * (1 - s * f),
                         t = v * (1 - s * (1 - f));
              // 只选择当前扇区的三个分量，避免每个像素构造整张 6×3 临时表。
              double rgb[3];
              switch (sector % 6) {
                case 0: rgb[0]=v; rgb[1]=t; rgb[2]=p; break;
                case 1: rgb[0]=q; rgb[1]=v; rgb[2]=p; break;
                case 2: rgb[0]=p; rgb[1]=v; rgb[2]=t; break;
                case 3: rgb[0]=p; rgb[1]=q; rgb[2]=v; break;
                case 4: rgb[0]=t; rgb[1]=p; rgb[2]=v; break;
                default: rgb[0]=v; rgb[1]=p; rgb[2]=q; break;
              }
              for (std::size_t ch = 0; ch < 3; ++ch)
                row[ch][c] = saturate_cast<T>(rgb[ch] * scale);
            }
          }
        });
  });
  return out.reduce_domain(domain);
}
mv::core::Image equ_histo_image(const mv::core::Image& a) {
  histogram_image(a, "equ_histo_image");
  const auto domain = a.domain();
  const auto bins = a.pixel_type() == PixelType::UInt8 ? 256U : 65536U;
  ExecutionContext context;
  std::pmr::vector<std::uint64_t> counts(bins, 0, context.scratch_resource());
  if(a.pixel_type()==PixelType::UInt8) {
    const auto local=detail::histogram256<std::uint8_t>(a,domain.runs());
    std::copy(local.begin(),local.end(),counts.begin());
  } else {
  dispatch_scalar(a.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    const auto* base = static_cast<const std::byte*>(a.data());
    const auto stride = a.stride();
    for (const auto& run : domain.runs()) {
      const auto* row = reinterpret_cast<const T*>(base + std::int64_t(run.row) * stride);
      for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c)
        ++counts[static_cast<std::size_t>(row[c])];
    }
  });
  }
  auto out = create_like(a, a.width(), a.height());
  if (domain.empty()) return out.reduce_domain(domain);
#if MVKIT_FILTER_HAS_AVX2
  const auto& lookup_cpu = mv::runtime::cpu_features();
  const bool vector_lookup = mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      lookup_cpu.selection(), lookup_cpu.hardware()), {false, true, false, false});
#endif
  std::uint64_t cumulative = 0;
  std::pmr::vector<double> lut(bins, 0, context.scratch_resource());
  for (std::size_t i = 0; i < bins; ++i) {
    cumulative += counts[i];
    lut[i] = double(cumulative) * (bins - 1) / double(domain.area());
  }
  dispatch_scalar(a.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    std::pmr::vector<T> values(bins, T{}, context.scratch_resource());
    for (std::size_t i = 0; i < bins; ++i) values[i] = saturate_cast<T>(lut[i]);
#if MVKIT_FILTER_HAS_AVX2
    std::array<std::int32_t, 256> lookup32;
    if constexpr (std::is_same_v<T, std::uint8_t>)
      if (vector_lookup) for (std::size_t i = 0; i < lookup32.size(); ++i) lookup32[i] = values[i];
#endif
    const auto* source = static_cast<const std::byte*>(a.data());
    auto* destination = static_cast<std::byte*>(out.mutable_data());
    const auto source_stride = a.stride(), destination_stride = out.stride();
    for (const auto& run : domain.runs()) {
      const auto* in = reinterpret_cast<const T*>(source + std::int64_t(run.row) * source_stride);
      auto* row = reinterpret_cast<T*>(destination + std::int64_t(run.row) * destination_stride);
      auto c = std::int64_t(run.col_begin);
#if MVKIT_FILTER_HAS_AVX2
      if constexpr (std::is_same_v<T, std::uint8_t>)
        if (vector_lookup) c = lookup_u8_row_avx2(in, row, static_cast<std::size_t>(c),
                                                static_cast<std::size_t>(run.col_end) + 1, lookup32.data());
#endif
      for (; c <= run.col_end; ++c)
        row[c] = values[static_cast<std::size_t>(in[c])];
    }
  });
  return out.reduce_domain(domain);
}
mv::core::Regions auto_threshold(const mv::core::Image& a, double s) {
  histogram_image(a, "auto_threshold");
  check(std::isfinite(s) && s >= 0 && s <= 32, "auto_threshold", "Sigma",
        "histogram sigma must be in [0,32]");
  const auto domain = a.domain();
  check(!domain.empty(), "auto_threshold", "Image",
        "empty domain cannot be segmented");
  std::array<double, 256> hist{}, smoothed{};
  const auto counts=a.pixel_type()==PixelType::UInt8
      ?detail::histogram256<std::uint8_t>(a,domain.runs())
      :detail::histogram256<std::uint16_t,8>(a,domain.runs());
  for(std::size_t i=0;i<hist.size();++i)hist[i]=static_cast<double>(counts[i]);
  const int radius = static_cast<int>(std::ceil(3 * s));
  // Sigma 已限制在 [0,32]，最多 193 个 tap。每桶保持原权重、表达式及求和序，
  // 每 tap 只计算一次 exp，极小 sigma 也如此。
  std::array<double,193> weights;
  double weight_sum = 0;
  if (s != 0) {
    for (int j = -radius; j <= radius; ++j) {
      const double weight = std::exp(-double(j * j) / (2 * s * s));
      weights[static_cast<std::size_t>(j + radius)] = weight;
      weight_sum += weight;
    }
  }
  for (int i = 0; i < 256; ++i) {
    if (s == 0) {
      smoothed[i] = hist[i];
      continue;
    }
    for (int j = -radius; j <= radius; ++j) {
      const double weight = weights[static_cast<std::size_t>(j + radius)];
      smoothed[i] += weight * hist[std::clamp(i + j, 0, 255)];
    }
    smoothed[i] /= weight_sum;
  }
  int first = 0, last = 255;
  while (hist[first] == 0) ++first;
  while (hist[last] == 0) --last;
  std::array<int, 256> cuts{};
  std::size_t count = 0;
  for (int i = first + 1; i < last; ++i) {
    const int start = i;
    while (i + 1 < last && smoothed[i + 1] == smoothed[start]) ++i;
    // HALCON 将单桶谷中心归入后一类别；平谷切分位于中点左侧。
    if (smoothed[start] < smoothed[start - 1] && smoothed[i] < smoothed[i + 1])
      cuts[count++] = (start + i - 1) / 2;
  }
  ExecutionContext context;
  std::pmr::vector<std::pmr::vector<RegionRun>> groups{
      context.scratch_resource()};
  groups.resize(count + 1);
  // 预计算 8 位直方图桶标签，再扫描强类型行。
  std::array<std::size_t, 256> labels{};
  for (int bin = 0; bin < 256; ++bin)
    labels[bin] = static_cast<std::size_t>(
        std::lower_bound(cuts.begin(), cuts.begin() + count, bin) - cuts.begin());
  dispatch_scalar(a.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    const auto* base = static_cast<const std::byte*>(a.data());
    const auto stride = a.stride();
    for (const auto& run : domain.runs()) {
      const auto* row = reinterpret_cast<const T*>(base + std::int64_t(run.row) * stride);
      const auto label = [&](std::int64_t column) {
        // 直方图只接受 UInt8/UInt16，256 桶索引为精确整数转换，含 UInt16 低字节边界，无需逐像素 Double 除法。
        if constexpr (std::is_same_v<T, std::uint16_t>)
          return labels[static_cast<unsigned>(row[column]) >> 8];
        else
          return labels[static_cast<std::size_t>(row[column])];
      };
      auto c = std::int64_t(run.col_begin);
      while (c <= run.col_end) {
        const auto begin = c;
        const auto group = label(c);
        do { ++c; } while (c <= run.col_end &&
            label(c) == group);
        groups[group].push_back({run.row, begin, c - 1});
      }
    }
  });
  std::vector<Region> regions;
  regions.reserve(groups.size());
  for (const auto& runs : groups)
    if (!runs.empty()) regions.push_back(Region::from_runs(runs));
  return Regions{std::move(regions)};
}
mv::core::Region local_threshold(const mv::core::Image& a, std::size_t w,
                                 std::size_t h, double k, double range,
                                 ThresholdPolarity polarity) {
  image(a, "local_threshold");
  window(w, h, "local_threshold");
  check(a.channel_count() == 1, "local_threshold", "Image",
        "single channel required");
  check(
      std::isfinite(k) && k >= 0 && k <= 1 && std::isfinite(range) && range > 0,
      "local_threshold", "Parameters",
      "k in [0,1] and positive finite dynamic_range required");
  check(polarity == ThresholdPolarity::Light ||
            polarity == ThresholdPolarity::Dark,
        "local_threshold", "Polarity", "invalid polarity");
  if (a.pixel_type() == PixelType::UInt8)
    return local_threshold_byte(a,w,h,k,range,polarity);
  return dispatch_scalar(a.pixel_type(), [&](auto tag) {
  using T = typename decltype(tag)::type;
  const auto* base = static_cast<const std::byte*>(a.data());
  const auto stride = a.stride(), image_width = a.width(), image_height = a.height();
  const auto read_pixel = [&](std::size_t r, std::size_t c) {
    return double(reinterpret_cast<const T*>(base + r * stride)[c]);
  };
  const auto finite_pixel = [&](std::size_t r, std::size_t c) {
    const auto v = read_pixel(r, c);
    check(std::isfinite(v), "image processing", "Image", "nonfinite pixel is not supported");
    return v;
  };
  const auto width = image_width + 1, height = image_height + 1;
  check(width > 0 && height > 0 &&
            width <= std::numeric_limits<std::size_t>::max() / height,
        "local_threshold", "Image", "integral image size overflow");
  ExecutionContext context;
  std::pmr::vector<double> sums(width * height, 0, context.scratch_resource()),
      squares(width * height, 0, context.scratch_resource());
  // 平方前去除 DC 偏移，尤其用于 Int32 和浮点图像。
  const auto origin = finite_pixel(0, 0);
  for (std::size_t r = 0; r < image_height; ++r) {
    double sum = 0, square = 0;
    for (std::size_t c = 0; c < image_width; ++c) {
      const auto v = finite_pixel(r, c) - origin;
      sum += v;
      square += v * v;
      sums[(r + 1) * width + c + 1] = sums[r * width + c + 1] + sum;
      squares[(r + 1) * width + c + 1] = squares[r * width + c + 1] + square;
    }
  }
  const auto domain = a.domain();
  const auto type = a.pixel_type(0);
  return mask_region(domain, [&](std::size_t r, std::size_t c) {
    const auto top = r > h / 2 ? r - h / 2 : 0,
               left = c > w / 2 ? c - w / 2 : 0,
               bottom = r + std::min(h / 2 + 1, image_height - r),
               right = c + std::min(w / 2 + 1, image_width - c);
    const double n = double(bottom - top) * double(right - left);
    const auto integral = [&](const auto& data) {
      return data[bottom * width + right] - data[top * width + right] -
             data[bottom * width + left] + data[top * width + left];
    };
    const auto centered_mean = integral(sums) / n;
    auto mean = centered_mean + origin;
    auto variance = std::max(0., integral(squares) / n - centered_mean * centered_mean);
    const double cancellation_bound = 64 * std::numeric_limits<double>::epsilon() *
        (std::abs(squares[bottom * width + right]) + std::abs(squares[top * width + right]) +
         std::abs(squares[bottom * width + left]) + std::abs(squares[top * width + left])) / n;
    if (cancellation_bound > 1e-6 * range * range && variance <= 100 * cancellation_bound) {
      // 少见的大偏移/低对比窗口使用稳定局部归约。
      double local_mean = 0, m2 = 0, count = 0;
      for (auto y = top; y < bottom; ++y) for (auto x = left; x < right; ++x) {
        const auto value = read_pixel(y, x);
        const auto delta = value - local_mean;
        local_mean += delta / ++count;
        m2 += delta * (value - local_mean);
      }
      mean = local_mean;
      variance = std::max(0., m2 / count);
    }
    const bool halcon_integer = type == PixelType::UInt8 || type == PixelType::UInt16;
    if (halcon_integer &&
        (r < h / 2 || c < w / 2 || r + h / 2 >= image_height ||
         c + w / 2 >= image_width)) {
      double local_mean = 0, m2 = 0, count = 0;
      for (std::int64_t dy = -static_cast<std::int64_t>(h / 2);
           dy <= static_cast<std::int64_t>(h / 2); ++dy)
        for (std::int64_t dx = -static_cast<std::int64_t>(w / 2);
             dx <= static_cast<std::int64_t>(w / 2); ++dx) {
          const auto sy = border_index(static_cast<std::int64_t>(r) + dy,
                                       image_height, BorderMode::Mirrored).value();
          const auto sx = border_index(static_cast<std::int64_t>(c) + dx,
                                       image_width, BorderMode::Mirrored).value();
          const auto value = read_pixel(sy, sx);
          const auto delta = value - local_mean;
          local_mean += delta / ++count;
          m2 += delta * (value - local_mean);
        }
      mean = local_mean;
      variance = std::max(0., m2 / count);
    }
    const auto deviation_factor = k * (std::sqrt(variance) / range - 1);
    // HALCON light 极性对反相 byte 图像执行 dark 规则；反相所得阈值给出此上截断。
    const auto threshold = polarity == ThresholdPolarity::Light && halcon_integer
                               ? mean - ((type == PixelType::UInt8 ? 255.0 : 65535.0) - mean) * deviation_factor
                               : mean * (1 + deviation_factor);
    const auto v = read_pixel(r, c);
    return polarity == ThresholdPolarity::Light ? v > threshold
                                                : v <= threshold;
  });
  });
}
}  // namespace mv::imageproc
