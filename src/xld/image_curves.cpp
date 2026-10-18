#include <xld/contour_utils.hpp>
#include <runtime/memory/overwrite_buffer.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/border.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <unordered_map>
#include <deque>
#include <bit>
#include <xld/fast_paths_avx2.hpp>
#include <type_traits>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_CELLS_SSE2 1
#else
#define MVKIT_CELLS_SSE2 0
#endif
namespace mv::xld {
namespace {
using namespace detail;
struct Node {
  Point2d p;
  double response = 0;
  bool strong = false;
};
struct Edge {
  std::size_t a, b;
  bool used = false;
};
XldContours trace(std::span<const Node> nodes, std::pmr::vector<Edge>& edges,
                  bool attributes, std::pmr::memory_resource* memory) {
  // 等值平台可能从两个单元贡献同一网格边。
  for (auto& e : edges)
    if (e.a > e.b) std::swap(e.a, e.b);
  if(edges.size()>=4096 && nodes.size()<=edges.size()*2) {
    // 稳定计数遍历产生与比较排序相同的 (a,b) 次序。
    std::pmr::vector<std::size_t> offsets(nodes.size(),0,memory);
    std::pmr::vector<Edge> temporary(edges.size(),memory);
    const auto sort_key=[&](const auto& input,auto& output,bool first) {
      std::fill(offsets.begin(),offsets.end(),0);
      for(const auto& e:input)++offsets[first ? e.a : e.b];
      std::size_t next=0;for(auto& count:offsets){const auto n=count;count=next;next+=n;}
      for(const auto& e:input)output[offsets[first ? e.a : e.b]++]=e;
    };
    sort_key(edges,temporary,false);sort_key(temporary,edges,true);
  } else std::sort(edges.begin(), edges.end(), [](auto a, auto b) {
    return a.a < b.a || (a.a == b.a && a.b < b.b);
  });
  edges.erase(
      std::unique(edges.begin(), edges.end(),
                  [](auto a, auto b) { return a.a == b.a && a.b == b.b; }),
      edges.end());
  const auto none=SIZE_MAX;
  // 紧凑邻接保留原逆插入遍历序；填充时递减节点末位置，
  // 完成后 offsets[i+1] 即节点 i 起点，无需独立 cursor/head/degree 数组。
  std::pmr::vector<std::size_t> offsets(nodes.size()+1,0,memory);
  for(auto& edge:edges) {
    if(edge.a==edge.b){edge.used=true;continue;}
    ++offsets[edge.a+1];++offsets[edge.b+1];
  }
  for(std::size_t i=1;i<offsets.size();++i)offsets[i]+=offsets[i-1];
  std::pmr::vector<std::size_t> links(offsets.back(),memory);
  for(std::size_t i=0;i<edges.size();++i) {
    const auto edge=edges[i];if(edge.a==edge.b)continue;
    links[--offsets[edge.a+1]]=i;links[--offsets[edge.b+1]]=i;
  }
  const auto adjacent=[&](std::size_t i) {
    const auto begin=offsets[i+1],end=i+1<nodes.size() ? offsets[i+2] : links.size();
    return std::span<const std::size_t>{links}.subspan(begin,end-begin);
  };
  std::pmr::vector<Point2d> points{memory};
  std::pmr::vector<double> response{memory}, normal{memory};
  std::pmr::vector<XldContour> contours{memory};
  auto walk = [&](std::size_t start, std::size_t edge) {
    points.clear();
    response.clear();
    normal.clear();
    auto current = start;
    bool closed = false;
    while (true) {
      points.push_back(nodes[current].p);
      if(attributes)response.push_back(nodes[current].response);
      edges[edge].used = true;
      auto next = edges[edge].a == current ? edges[edge].b : edges[edge].a;
      if (next == start) {
        closed = true;
        break;
      }
      current = next;
      if (adjacent(current).size() != 2) {
        points.push_back(nodes[current].p);
        if(attributes)response.push_back(nodes[current].response);
        break;
      }
      auto candidate = none;
      for (const auto linked : adjacent(current))
        if (!edges[linked].used) {
          candidate = linked;
          break;
        }
      if (candidate == none) {
        points.push_back(nodes[current].p);
        if(attributes)response.push_back(nodes[current].response);
        break;
      }
      edge = candidate;
    }
    if (points.size() < 2) return;
    if (attributes)
      for (std::size_t i = 0; i < points.size(); ++i) {
        auto a = points[i        ? i - 1
                        : closed ? points.size() - 1
                                 : 0],
             b = points[i + 1 < points.size() ? i + 1
                        : closed              ? 0
                                              : i];
        normal.push_back(positive_atan2(-(b.row - a.row), b.col - a.col));
      }
    contours.push_back(XldContour::from_points_and_attributes(
        points, closed, ContourApproximation::Unknown,
        attributes ? std::span<const double>{response}
                   : std::span<const double>{},
        {}, normal));
  };
  for (std::size_t i = 0; i < nodes.size(); ++i)
    if (adjacent(i).size() != 2)
      for (const auto linked : adjacent(i))
        if (!edges[linked].used) walk(i,linked);
  for (std::size_t i = 0; i < edges.size(); ++i)
    if (!edges[i].used) walk(edges[i].a, i);
  return objects(contours);
}
double ridge_length(double x, double y) {
  const double squared = x*x+y*y;
  if (squared >= 1e-300 && squared < std::numeric_limits<double>::infinity())
    return std::sqrt(squared);
  if (x == 0 && y == 0) return 0;
  return std::hypot(x,y);
}
template<class Buffer=std::pmr::vector<double>>
Buffer pixels(const Image& image, const char* op,
                                std::pmr::memory_resource* memory) {
  require(!image.empty() && image.channel_count() == 1, op, "Image",
          "one-channel image required");
  mv::imageproc::ImageView view{image};
  const auto width = image.width(), height = image.height();
  return mv::imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    Buffer p(width * height, memory);
    ExecutionContext::for_each_partition(
        height, p.size(), 32768, [&](auto range) {
          for (auto r = range.begin; r < range.end; ++r) {
            auto row = view.row_as<T>(r);
            for (std::size_t c = 0; c < width; ++c) {
              double v = row[c];
              require(mv::core::is_finite(v), op, "Image",
                      "finite gray values required");
              p[r * width + c] = v;
            }
          }
        });
    return p;
  });
}
}  // namespace
mv::core::XldContours threshold_sub_pix(const mv::core::Image& image,
                                        double threshold) {
  using namespace detail;
  constexpr auto op = "threshold_sub_pix";
  require(std::isfinite(threshold), op, "Threshold",
          "finite threshold required");
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  require(!image.empty() && image.channel_count() == 1, op, "Image",
          "one-channel image required");
  const auto w = image.width(), h = image.height();
  auto generate = [&](const auto& p) {
  auto domain = image.domain();
  std::pmr::vector<Node> nodes{memory};
  std::pmr::vector<Edge> edges{memory};
  // 单元只与当前及下一栅格行共享顶点；完整键标记两行缓存，
  // 保持端点共享，不逐交点分配哈希或建立整图索引缓冲。
  std::pmr::vector<std::size_t> keys(6 * w, SIZE_MAX, memory), ids(6 * w, 0, memory);
  auto vertex = [&](std::size_t a, std::size_t b) {
    double t = (threshold - p[a]) / (p[b] - p[a]);
    const auto endpoint = t == 0 ? a : b;
    const auto origin = t == 0 || t == 1 ? endpoint : a;
    const auto kind = t == 0 || t == 1 ? 2u : (b - a == w ? 1u : 0u);
    const auto key = kind == 2 ? 2 * w * h + origin : a * 2 + kind;
    const auto slot = (kind * 2 + (origin / w) % 2) * w + origin % w;
    if (keys[slot] == key) return ids[slot];
    auto id = nodes.size();
    keys[slot] = key;
    ids[slot] = id;
    const double row=double(a/w),column=double(a%w);
    // 网格边仅一个坐标变化，只对此轴 lerp，避免重复整数除法或插值常量。
    nodes.push_back({b-a==w ? Point2d{std::lerp(row,row+1,t),column}
                           : Point2d{row,std::lerp(column,column+1,t)}});
    return id;
  };
  // 仅遍历四角均在域内的单元，一次求游程区间交集，避免逐像素四次二分。
  if (w < 2 || h < 2) return XldContours{};
  const auto cells = domain.intersect(domain.shifted(-1, 0))
      .intersect(domain.shifted(0, -1)).intersect(domain.shifted(-1, -1));
  const auto& cpu=context.cpu();
  [[maybe_unused]] const bool simd=MVKIT_CELLS_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{true,false,false,false});
#if MVKIT_CELLS_SSE2
  const auto byte_limit=_mm_set1_epi8(static_cast<char>(
      static_cast<int>(std::ceil(std::clamp(threshold,0.,255.)))-1-128));
  const auto byte_sign=_mm_set1_epi8(char(-128));
#endif
  for (const auto& run : cells.runs()) {
    const auto r = static_cast<std::size_t>(run.row);
    std::int64_t pending_end=std::int64_t(run.col_begin)-1;
    std::uint32_t pending=0;
    for (auto column = std::int64_t(run.col_begin); column <= run.col_end; ++column) {
#if MVKIT_CELLS_SSE2
      if constexpr(std::is_same_v<typename std::decay_t<decltype(p)>::value_type,std::uint8_t>) {
        if(simd && threshold>0 && threshold<=255) {
          if(column>pending_end && std::int64_t(run.col_end)-column>=15) {
            const auto* top=p.data()+r*w+column;const auto* bottom=top+w;
            const auto above=[&](const auto* data){return std::uint32_t(_mm_movemask_epi8(_mm_cmpgt_epi8(
                _mm_xor_si128(_mm_loadu_si128(reinterpret_cast<const __m128i*>(data)),byte_sign),byte_limit)));};
            const auto a=above(top);pending=(a^above(top+1))|(a^above(bottom))|(a^above(bottom+1));
            pending_end=column+15;
          }
          if(column<=pending_end) {
            if(!pending){column=pending_end;continue;}
            const auto first=std::countr_zero(pending);
            column=pending_end-15+first;pending&=pending-1;
          }
        }
      }
#endif
      const auto c = static_cast<std::size_t>(column);
      std::array<std::size_t, 4> corner{r * w + c, r * w + c + 1,
                                        (r + 1) * w + c + 1, (r + 1) * w + c};
      const std::array<bool,4> above{p[corner[0]] >= threshold, p[corner[1]] >= threshold,
                                     p[corner[2]] >= threshold, p[corner[3]] >= threshold};
      if (above[0] == above[1] && above[0] == above[2] && above[0] == above[3]) continue;
      std::array<std::size_t, 4> v{};
      std::array<bool, 4> active{};
      int count = 0;
      for (int k = 0; k < 4; ++k) {
        auto a = corner[k], b = corner[(k + 1) % 4];
        if (above[k] != above[(k + 1) % 4]) {
          if (a > b) std::swap(a, b);
          v[k] = vertex(a, b);
          active[k] = true;
          ++count;
        }
      }
      auto add = [&](int a, int b) {
        if (v[a] != v[b]) edges.push_back({v[a], v[b]});
      };
      if (count == 2) {
        int a = -1;
        for (int k = 0; k < 4; ++k)
          if (active[k]) {
            if (a < 0)
              a = k;
            else
              add(a, k);
          }
      } else if (count == 4) {
        double a = p[corner[0]] - threshold, b = p[corner[1]] - threshold,
               cc = p[corner[2]] - threshold, d = p[corner[3]] - threshold;
        double det = a * cc - b * d;
        if (det == 0) {
          double denominator = a - b + cc - d;
          double x = denominator == 0 ? .5 : (a - d) / denominator,
                 y = denominator == 0 ? .5 : (a - b) / denominator;
          auto center = nodes.size();
          nodes.push_back(
              {{r + std::clamp(y, 0., 1.), c + std::clamp(x, 0., 1.)}});
          for (int k = 0; k < 4; ++k) edges.push_back({v[k], center});
        } else if (det > 0) {
          add(0, 1);
          add(2, 3);
        } else {
          add(0, 3);
          add(1, 2);
        }
      }
    }
  }
  return trace(nodes, edges, false, memory);
  };
  if (image.pixel_type() == PixelType::UInt8 && image.stride() == w)
    return generate(std::span<const std::uint8_t>{static_cast<const std::uint8_t*>(image.data()), w*h});
  const auto values=pixels(image, op, memory);
  return generate(values);
}
mv::core::XldContours lines_gauss(const mv::core::Image& image, double sigma,
                                  double low, double high,
                                  LinePolarity polarity) {
  using namespace detail;
  constexpr auto op = "lines_gauss";
  require(std::isfinite(sigma) && sigma >= .5 && sigma <= 64, op, "Sigma",
          "expected sigma in [0.5,64]");
  require(std::isfinite(low) && std::isfinite(high) && low >= 0 && high >= low,
          op, "Low/High", "finite ordered nonnegative thresholds required");
  require(polarity == LinePolarity::Light || polarity == LinePolarity::Dark, op,
          "Polarity", "invalid polarity");
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  auto source = pixels<mv::runtime::OverwriteBuffer<double>>(image, op, memory);
  auto w = image.width(), h = image.height();
  if (w < 3 || h < 3) return {};
  auto domain = image.domain();
  auto border = [](std::int64_t i, std::size_t size) {
    return mv::imageproc::border_index(i, size,
                                       mv::imageproc::BorderMode::Mirrored)
        .value();
  };
  // ridge 位置/响应都使用像素积分 Gaussian 导数；采样平滑加三 tap 差分会让窄/宽线产生相反偏差。
  const int derivative_radius = int(std::ceil(5 * sigma));
  const auto taps = static_cast<std::size_t>(2 * derivative_radius + 1);
  std::pmr::vector<std::size_t> column_indices(w * taps, memory), row_offsets(h * taps, memory);
  for (std::size_t c = 0; c < w; ++c)
    for (int k = -derivative_radius; k <= derivative_radius; ++k)
      column_indices[c * taps + k + derivative_radius] = border(std::int64_t(c) + k, w);
  for (std::size_t r = 0; r < h; ++r)
    for (int k = -derivative_radius; k <= derivative_radius; ++k)
      row_offsets[r * taps + k + derivative_radius] = border(std::int64_t(r) + k, h) * w;
  const bool full_domain = !image.has_reduced_domain();
  std::pmr::vector<double> g0(2 * derivative_radius + 1, memory),
      g1(2 * derivative_radius + 1, memory), g2(2 * derivative_radius + 1, memory);
  const double inv_sigma = 1.0 / sigma;
  const double density_scale = inv_sigma / std::sqrt(2 * pi);
  auto cdf = [&](double x) { return .5 * std::erfc(-x * inv_sigma / std::sqrt(2.0)); };
  auto density = [&](double x) {
    return density_scale * std::exp(-.5 * x * x * inv_sigma * inv_sigma);
  };
  for (int k = -derivative_radius; k <= derivative_radius; ++k) {
    const double left = k - .5, right = k + .5;
    g0[k + derivative_radius] = cdf(right) - cdf(left);
    g1[k + derivative_radius] = density(left) - density(right);
    g2[k + derivative_radius] = -right * inv_sigma * inv_sigma * density(right) +
                                left * inv_sigma * inv_sigma * density(left);
  }
  mv::runtime::OverwriteBuffer<double> horizontal0(source.size(), memory),
      horizontal1(source.size(), memory), horizontal2(source.size(), memory);
  const auto& cpu = mv::runtime::cpu_features();
  [[maybe_unused]] const bool simd = MVKIT_CELLS_SSE2 && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()), {true,false,false,false});
#if MVKIT_XLD_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,false,true,false});
  // 仅这些支持类型能保证配对和远小于 Double 溢出界。
  const bool bounded_pixels=image.pixel_type()==mv::core::PixelType::UInt8 ||
      image.pixel_type()==mv::core::PixelType::UInt16 || image.pixel_type()==mv::core::PixelType::Int16 ||
      image.pixel_type()==mv::core::PixelType::Int32 || image.pixel_type()==mv::core::PixelType::Float32;
#endif
  ExecutionContext::for_each_partition(h, source.size(), 32768, [&](auto range) {
    for (auto r = range.begin; r < range.end; ++r) {
      std::size_t c = 0;
#if MVKIT_XLD_HAS_AVX2
      if(avx2) {
        ridge_horizontal_avx2(source.data()+r*w,column_indices.data(),w,derivative_radius,g0,g1,g2,
            horizontal0.data()+r*w,horizontal1.data()+r*w,horizontal2.data()+r*w,bounded_pixels);
        continue;
      }
#endif
#if MVKIT_CELLS_SSE2
      if (simd) for (; c + 1 < w; c += 2) {
        auto v0 = _mm_setzero_pd(), v1 = _mm_setzero_pd(), v2 = _mm_setzero_pd();
        for (std::size_t k = 0; k < taps; ++k) {
          const auto value = c >= static_cast<std::size_t>(derivative_radius) &&
                            w-c > static_cast<std::size_t>(derivative_radius+1)
              ? _mm_loadu_pd(source.data()+r*w+c+k-derivative_radius)
              : _mm_set_pd(source[r*w+column_indices[(c+1)*taps+k]],
                           source[r*w+column_indices[c*taps+k]]);
          v0 = _mm_add_pd(v0,_mm_mul_pd(_mm_set1_pd(g0[k]),value));
          v1 = _mm_add_pd(v1,_mm_mul_pd(_mm_set1_pd(g1[k]),value));
          v2 = _mm_add_pd(v2,_mm_mul_pd(_mm_set1_pd(g2[k]),value));
        }
        _mm_storeu_pd(horizontal0.data()+r*w+c,v0);
        _mm_storeu_pd(horizontal1.data()+r*w+c,v1);
        _mm_storeu_pd(horizontal2.data()+r*w+c,v2);
      }
#endif
      for (; c < w; ++c) {
        double v0 = 0, v1 = 0, v2 = 0;
        for (int k = -derivative_radius; k <= derivative_radius; ++k) {
          const double value = source[r * w + column_indices[c * taps + k + derivative_radius]];
          const auto offset = k + derivative_radius;
          v0 += g0[offset] * value;
          v1 += g1[offset] * value;
          v2 += g2[offset] * value;
        }
        const auto i = r * w + c;
        horizontal0[i] = v0;
        horizontal1[i] = v1;
        horizontal2[i] = v2;
      }
    }
  });
  struct Ridge {
    bool active = false, strong = false;
    Point2d p;
    std::size_t cell=0;
    double response = 0, nr = 0, nc = 0;
  };
  std::vector<std::vector<Ridge>> row_candidates(h);
  ExecutionContext::for_each_partition(h, source.size(), 32768, [&](auto range) {
    // Hessian 行先定位响应，仅对所需 tile 计算梯度。
    std::vector<double> row_values(3*w);
    auto* response_xx=row_values.data();auto* response_xy=response_xx+w;auto* response_yy=response_xy+w;
    for (auto r = range.begin; r < range.end; ++r) {
      std::size_t c = 0;
#if MVKIT_XLD_HAS_AVX2
      if(avx2) {
        ridge_response_avx2(horizontal0.data(),horizontal1.data(),horizontal2.data(),row_offsets.data()+r*taps,
            w,g0,g1,g2,response_xx,response_xy,response_yy,bounded_pixels);
        c=w;
      }
#endif
#if MVKIT_CELLS_SSE2
      if (simd) for (; c + 1 < w; c += 2) {
        auto gx = _mm_setzero_pd(), gy = _mm_setzero_pd(), xx = _mm_setzero_pd(),
             xy = _mm_setzero_pd(), yy = _mm_setzero_pd();
        for (std::size_t k = 0; k < taps; ++k) {
          const auto i = row_offsets[r*taps+k]+c;
          const auto h0 = _mm_loadu_pd(horizontal0.data()+i), h1 = _mm_loadu_pd(horizontal1.data()+i),
                     h2 = _mm_loadu_pd(horizontal2.data()+i);
          const auto k0 = _mm_set1_pd(g0[k]), k1 = _mm_set1_pd(g1[k]), k2 = _mm_set1_pd(g2[k]);
          xx = _mm_add_pd(xx,_mm_mul_pd(k0,h2)); xy = _mm_add_pd(xy,_mm_mul_pd(k1,h1));
          yy = _mm_add_pd(yy,_mm_mul_pd(k2,h0));

        }
        _mm_storeu_pd(response_xx+c,xx); _mm_storeu_pd(response_xy+c,xy);
        _mm_storeu_pd(response_yy+c,yy);

      }
#endif
      for (; c < w; ++c) {
        double gx = 0, gy = 0, xx = 0, xy = 0, yy = 0;
        for (int k = -derivative_radius; k <= derivative_radius; ++k) {
          const auto i = row_offsets[r * taps + k + derivative_radius] + c;
          const auto offset = k + derivative_radius;
          xx += g0[offset] * horizontal2[i];
          xy += g1[offset] * horizontal1[i];
          yy += g2[offset] * horizontal0[i];

        }
        response_xx[c] = xx;
        response_xy[c] = xy;
        response_yy[c] = yy;

      }
      std::size_t gradient_column=SIZE_MAX;
      double tile_gx[4]{},tile_gy[4]{};
      [[maybe_unused]] std::size_t pending_end=0;
      [[maybe_unused]] std::uint32_t pending=0;
      for (std::size_t c = 0; c < w; ++c) {
#if MVKIT_XLD_HAS_AVX2
        if(avx2 && full_domain) {
          if(c>=pending_end && w-c>=32) {
            pending=ridge_candidates_mask32_avx2(response_xx+c,response_xy+c,response_yy+c,low);
            pending_end=c+32;
          }
          if(c<pending_end) {
            if(!pending){c=pending_end-1;continue;}
            c=pending_end-32+std::countr_zero(pending);pending&=pending-1;
          }
        }
#endif
        bool valid = true;
        if (!full_domain) for (int dr = -1; dr <= 1; ++dr)
          for (int dc = -1; dc <= 1; ++dc)
            valid &= domain.contains(
                static_cast<std::int64_t>(border(std::int64_t(r) + dr, h)),
                static_cast<std::int64_t>(border(std::int64_t(c) + dc, w)));
        if (!valid) continue;
        auto i = r * w + c;
        double xx = response_xx[c], yy = response_yy[c], xy = response_xy[c];
        // Gershgorin 给出谱响应界，留两倍余量，使 Low 附近舍入仍走原特征系统路径。
        if (std::abs(xx)+std::abs(yy)+2*std::abs(xy) < low*.5) continue;
        double disc = ridge_length(xx - yy, 2 * xy),
               l1 = (xx + yy + disc) * .5, l2 = (xx + yy - disc) * .5,
               lambda = std::abs(l1) > std::abs(l2) ? l1 : l2;
        const double response = polarity == LinePolarity::Light ? -lambda : lambda;
        if (response <= 0 || response < low) continue;
        double nx = xy, ny = lambda - xx;
        if (ridge_length(lambda-yy, xy)>ridge_length(nx,ny)) {nx=lambda-yy;ny=xy;}
        if (nx == 0 && ny == 0) {
          nx = 1;
          ny = 0;
        }
        double norm = ridge_length(nx, ny);
        nx /= norm;
        ny /= norm;
        double gx=0,gy=0;bool tile=false;
#if MVKIT_XLD_HAS_AVX2
        const auto column=c/4*4;
        if(avx2 && w-column>=4) {
          if(gradient_column!=column) {
            ridge_gradient_tile_avx2(horizontal0.data(),horizontal1.data(),row_offsets.data()+r*taps,
                                     column,g0,g1,tile_gx,tile_gy);
            gradient_column=column;
          }
          gx=tile_gx[c-column];gy=tile_gy[c-column];tile=true;
        }
#endif
        if(!tile)for(std::size_t k=0;k<taps;++k) {
          const auto offset=row_offsets[r*taps+k]+c;
          gx+=g0[k]*horizontal1[offset];gy+=g1[k]*horizontal0[offset];
        }
        double t = -(gx * nx + gy * ny) / lambda;
        double dr = t * ny, dc = t * nx;
        // 邻接定位单元重叠；二次模型可把真实半像素 ridge 推过 0.5，图连接前抑制重复物理 ridge。
        if (dr < -.6 || dr > .6 || dc < -.6 || dc > .6) continue;
        row_candidates[r].push_back({
            true, response >= high, {r + dr, c + dc}, i, response, ny, nx});
      }
    }
  });
  std::pmr::vector<Ridge> candidates{memory};
  std::size_t candidate_count=0;
  for(const auto& row:row_candidates)candidate_count+=row.size();
  candidates.reserve(candidate_count);
  std::pmr::vector<std::size_t> cells(source.size(),SIZE_MAX,memory);
  for(auto& row:row_candidates) {
    for(const auto& value:row) {
      cells[value.cell]=candidates.size();candidates.push_back(value);
    }
    std::vector<Ridge>{}.swap(row);
  }
  // 相邻 Taylor 拟合可在两像素定位同一 ridge，追踪前保留强响应以避免三角捷径。
  for (std::size_t i=0;i<candidates.size();++i) {
    if (!candidates[i].active) continue;
    const auto r=candidates[i].cell/w,c=candidates[i].cell%w;
    for (auto [dr,dc] : std::array<std::pair<int,int>,4>{{{0,1},{1,-1},{1,0},{1,1}}}) {
      const auto rr=std::int64_t(r)+dr,cc=std::int64_t(c)+dc;
      if(rr<0||cc<0||rr>=std::int64_t(h)||cc>=std::int64_t(w))continue;
      const auto j=cells[std::size_t(rr)*w+std::size_t(cc)];
      if(j==SIZE_MAX)continue;
      if(!candidates[j].active)continue;
      const auto distance=ridge_length(candidates[i].p.row-candidates[j].p.row,candidates[i].p.col-candidates[j].p.col);
      if(distance>=.5)continue;
      if(candidates[i].response>=candidates[j].response)candidates[j].active=false;
      else {candidates[i].active=false;break;}
    }
  }
  std::pmr::vector<Node> nodes{memory};
  std::pmr::vector<std::size_t> ids(candidates.size(), SIZE_MAX, memory);
  for (std::size_t i = 0; i < candidates.size(); ++i)
    if (candidates[i].active) {
      ids[i] = nodes.size();
      auto v = candidates[i];
      nodes.push_back({v.p, v.response, v.strong});
    }
  std::pmr::vector<Edge> edges{memory};
  for (std::size_t i = 0; i < candidates.size(); ++i)
    if (ids[i] != SIZE_MAX) {
      auto r = candidates[i].cell / w, c = candidates[i].cell % w;
      for (auto delta : std::array<std::pair<int, int>, 4>{
               {{0, 1}, {1, -1}, {1, 0}, {1, 1}}}) {
        auto rr = std::int64_t(r) + delta.first,
             cc = std::int64_t(c) + delta.second;
        if (rr < 0 || cc < 0 || rr >= std::int64_t(h) || cc >= std::int64_t(w))
          continue;
        auto j = cells[std::size_t(rr) * w + std::size_t(cc)];
        if(j==SIZE_MAX)continue;
        if (ids[j] == SIZE_MAX) continue;
        double dy = candidates[j].p.row - candidates[i].p.row,
               dx = candidates[j].p.col - candidates[i].p.col,
               d = ridge_length(dx, dy);
        if (d == 0) continue;
        if (std::abs(dy * candidates[i].nr + dx * candidates[i].nc) > .6 * d ||
            std::abs(dy * candidates[j].nr + dx * candidates[j].nc) > .6 * d)
          continue;
        edges.push_back({ids[i], ids[j]});
      }
    }
  // 连通分量迟滞使用 scratch 并查集，不反复扫描图像。
  std::pmr::vector<std::size_t> parent(nodes.size(), memory);
  std::pmr::vector<unsigned char> strong(nodes.size(), 0, memory);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    parent[i] = i;
    strong[i] = nodes[i].strong;
  }
  auto root = [&](std::size_t i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  };
  for (auto e : edges) {
    auto a = root(e.a), b = root(e.b);
    if (a != b) {
      parent[b] = a;
      strong[a] |= strong[b];
    }
  }
  edges.erase(std::remove_if(edges.begin(), edges.end(),
                             [&](auto e) { return !strong[root(e.a)]; }),
              edges.end());
  return trace(nodes, edges, true, memory);
}
}  // namespace mv::xld
