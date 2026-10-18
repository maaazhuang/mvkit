#include <imageproc/advanced.hpp>
#include <bit>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#endif
#include <runtime/memory/memory_resource.hpp>
#include <runtime/cpu/cpu_features.hpp>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_REMAP_SSE2 1
#else
#define MVKIT_REMAP_SSE2 0
#endif

namespace mv::imageproc {
namespace {
using namespace advanced;
double cubic(double x) {
  x = std::abs(x);
  return x <= 1  ? (1.5 * x - 2.5) * x * x + 1
         : x < 2 ? ((-0.5 * x + 2.5) * x - 4) * x + 2
                 : 0;
}

#if MVKIT_REMAP_SSE2
template <int Quadrant>
void rotate_byte_tiles(const std::byte* source,std::size_t stride,std::byte* destination,
                       std::size_t out_stride,std::size_t width,std::size_t height) {
  static_assert(Quadrant==1 || Quadrant==3);
  const auto store=[&](std::size_t r,std::size_t c,__m128i column) {
    if constexpr(Quadrant==1) {
      // 列内八行反序，对应逆时针旋转；只写真实存在的八个像素。
      column=_mm_or_si128(_mm_slli_epi16(column,8),_mm_srli_epi16(column,8));
      column=_mm_shufflelo_epi16(column,_MM_SHUFFLE(0,1,2,3));
      _mm_storel_epi64(reinterpret_cast<__m128i*>(destination+c*out_stride+height-r-8),column);
    } else _mm_storel_epi64(reinterpret_cast<__m128i*>(destination+(width-1-c)*out_stride+r),column);
  };
  const auto scalar=[&](std::size_t r,std::size_t c) {
    if constexpr(Quadrant==1)destination[c*out_stride+height-1-r]=source[r*stride+c];
    else destination[(width-1-c)*out_stride+r]=source[r*stride+c];
  };
  std::size_t r=0;
  for(;r+8<=height;r+=8) {
    std::size_t c=0;
    for(;c+8<=width;c+=8) {
      const auto load=[&](std::size_t y){return _mm_loadl_epi64(reinterpret_cast<const __m128i*>(source+(r+y)*stride+c));};
      const auto a=_mm_unpacklo_epi8(load(0),load(1)),b=_mm_unpacklo_epi8(load(2),load(3));
      const auto d=_mm_unpacklo_epi8(load(4),load(5)),e=_mm_unpacklo_epi8(load(6),load(7));
      const auto ab0=_mm_unpacklo_epi16(a,b),ab1=_mm_unpackhi_epi16(a,b);
      const auto de0=_mm_unpacklo_epi16(d,e),de1=_mm_unpackhi_epi16(d,e);
      const __m128i columns[]{_mm_unpacklo_epi32(ab0,de0),_mm_unpackhi_epi32(ab0,de0),
                              _mm_unpacklo_epi32(ab1,de1),_mm_unpackhi_epi32(ab1,de1)};
      for(std::size_t k=0;k<4;++k) {
        store(r,c+2*k,columns[k]);store(r,c+2*k+1,_mm_srli_si128(columns[k],8));
      }
    }
    for(;c<width;++c)for(std::size_t k=0;k<8;++k)scalar(r+k,c);
  }
  for(;r<height;++r)for(std::size_t c=0;c<width;++c)scalar(r,c);
}
#endif

template <int Quadrant>
Image rotate_right_angle(const Image& image) {
  constexpr bool transpose = Quadrant == 1 || Quadrant == 3;
  const auto width = transpose ? image.height() : image.width();
  const auto height = transpose ? image.width() : image.height();
  dimensions(width,height,"remap");
  // 直角最近邻覆盖整个输出，包含输入缩减域之外的像素；与原 rotate 规则一致。
  auto output = create_like(image,width,height,false,true);
  for (std::size_t channel=0;channel<image.channel_count();++channel) {
    const auto* source=static_cast<const std::byte*>(image.data(channel));
    auto* destination=static_cast<std::byte*>(output.mutable_data(channel));
    const auto stride=image.stride(channel),out_stride=output.stride(channel);
#if MVKIT_REMAP_SSE2
    if constexpr(transpose) {
      const auto& selected=mv::runtime::cpu_features().selection();
      if(image.pixel_type(channel)==PixelType::UInt8 && (selected.sse2 || selected.avx2)) {
        rotate_byte_tiles<Quadrant>(source,stride,destination,out_stride,image.width(),image.height());
        continue;
      }
    }
#endif
    if constexpr (Quadrant == 0) {
      const auto bytes=width*bytes_per_sample(image.pixel_type(channel));
      for(std::size_t r=0;r<height;++r)
        std::memcpy(destination+r*out_stride,source+r*stride,bytes);
    } else dispatch_scalar(image.pixel_type(channel),[&](auto tag) {
      using T=typename decltype(tag)::type;
      // 转置按小块读取，避免跨整幅图像逐列访问造成缓存抖动。
      constexpr std::size_t tile=32;
      for(std::size_t r0=0;r0<height;r0+=tile)
        for(std::size_t c0=0;c0<width;c0+=tile)
          for(auto r=r0;r<std::min(r0+tile,height);++r) {
            auto* row=destination+r*out_stride;
            for(auto c=c0;c<std::min(c0+tile,width);++c) {
              const auto y=Quadrant==1 ? image.height()-1-c : Quadrant==2 ? image.height()-1-r : c;
              const auto x=Quadrant==1 ? r : Quadrant==2 ? image.width()-1-c : image.width()-1-r;
              std::memcpy(row+c*sizeof(T),source+y*stride+x*sizeof(T),sizeof(T));
            }
          }
    });
  }
  return output;
}

double sample(const Image& a, std::size_t ch, double r, double c,
              Interpolation mode) {
  r = std::clamp(r, 0., double(a.height() - 1));
  c = std::clamp(c, 0., double(a.width() - 1));
  if (mode == Interpolation::NearestNeighbor || mode == Interpolation::Constant)
    return read(a, ch, std::size_t(std::floor(r + 0.5)),
                std::size_t(std::floor(c + 0.5)));
  const auto y = static_cast<std::int64_t>(std::floor(r)),
             x = static_cast<std::int64_t>(std::floor(c));
  const auto at = [&](std::int64_t rr, std::int64_t cc) {
    return read(a, ch,
                static_cast<std::size_t>(
                    std::clamp<std::int64_t>(rr, 0, a.height() - 1)),
                static_cast<std::size_t>(
                    std::clamp<std::int64_t>(cc, 0, a.width() - 1)));
  };
  if (mode == Interpolation::Bilinear) {
    const auto dy = r - y, dx = c - x;
    return (1 - dy) * ((1 - dx) * at(y, x) + dx * at(y, x + 1)) +
           dy * ((1 - dx) * at(y + 1, x) + dx * at(y + 1, x + 1));
  }
  double value = 0;
  for (int j = -1; j <= 2; ++j)
    for (int i = -1; i <= 2; ++i)
      value += cubic(r - y - j) * cubic(c - x - i) * at(y + j, x + i);
  return value;
}
template <class Map>
Image remap(const Image& a, std::size_t w, std::size_t h, Interpolation mode,
            Map&& map, bool ignore_input_domain = false) {
  advanced::image(a, "remap");
  dimensions(w, h, "remap");
  advanced::interpolation(mode, "remap");
  auto out = create_like(a, w, h);
  const auto domain = a.domain();
  ExecutionContext context;
  if (mode == Interpolation::NearestNeighbor || mode == Interpolation::Constant ||
      mode == Interpolation::Bilinear) {
    const auto width = a.width(), height = a.height();
    const bool full_domain = ignore_input_domain || !a.has_reduced_domain();
    std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};
    runs.reserve(h);
    for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
      const auto* source = static_cast<const std::byte*>(a.data(ch));
      const auto stride = a.stride(ch), output_stride = out.stride(ch);
      auto* destination = static_cast<std::byte*>(out.mutable_data(ch));
      dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
        using T = typename decltype(tag)::type;
        const auto at = [&](std::size_t r, std::size_t c) {
          T value;
          std::memcpy(&value, source + r * stride + c * sizeof(T), sizeof(T));
          return double(value);
        };
        for (std::size_t r = 0; r < h; ++r) {
          auto* row = reinterpret_cast<T*>(destination + r * output_stride);
          std::size_t begin = 0;
          bool active = false;
          for (std::size_t c = 0; c < w; ++c) {
            const auto xy = map(r, c);
            auto x = xy[0], y = xy[1];
            const bool inside = x >= -.5 && y >= -.5 &&
                x < double(width) - .5 && y < double(height) - .5 &&
                (full_domain || domain.contains(static_cast<std::int64_t>(y + .5),
                                               static_cast<std::int64_t>(x + .5)));
            if (ch == 0) {
              if (inside && !active) begin = c;
              if (!inside && active) runs.push_back({static_cast<std::int64_t>(r), static_cast<std::int64_t>(begin), static_cast<std::int64_t>(c - 1)});
              active = inside;
            }
            if (!inside) { row[c] = T{}; continue; }
            x = std::clamp(x, 0., double(width - 1));
            y = std::clamp(y, 0., double(height - 1));
            double value;
            if (mode != Interpolation::Bilinear) {
              T exact;
              std::memcpy(&exact, source + static_cast<std::size_t>(y + .5) * stride +
                  static_cast<std::size_t>(x + .5) * sizeof(T), sizeof(T));
              row[c] = exact;
              continue;
            } else {
              const auto yy = static_cast<std::size_t>(y), xx = static_cast<std::size_t>(x);
              const auto y1 = std::min(yy + 1, height - 1), x1 = std::min(xx + 1, width - 1);
              const auto dy = y - yy, dx = x - xx;
              value = (1 - dy) * ((1 - dx) * at(yy, xx) + dx * at(yy, x1)) +
                      dy * ((1 - dx) * at(y1, xx) + dx * at(y1, x1));

            }
            if constexpr (std::is_floating_point_v<T>) row[c] = static_cast<T>(value);
            else {
              value = std::clamp(value, double(std::numeric_limits<T>::lowest()), double(std::numeric_limits<T>::max()));
              row[c] = static_cast<T>(value < 0 ? value - .5 : value + .5);
            }
          }
          if (ch == 0 && active) runs.push_back({static_cast<std::int64_t>(r), static_cast<std::int64_t>(begin), static_cast<std::int64_t>(w - 1)});
        }
      });
    }
    return out.reduce_domain(Region::from_canonical_runs(std::move(runs)));
  }
  std::pmr::vector<unsigned char> valid(w * h, 0, context.scratch_resource());
  pixels(out, [&](std::size_t ch, std::size_t r, std::size_t c) {
    const auto xy = map(r, c);
    double x = xy[0], y = xy[1];
    if (x >= -1e-9 && x <= double(a.width() - 1) + 1e-9)
      x = std::clamp(x, 0., double(a.width() - 1));
    if (y >= -1e-9 && y <= double(a.height() - 1) + 1e-9)
      y = std::clamp(y, 0., double(a.height() - 1));
    // nearest 像素占中心周围半开单位格；仅保留中心 [0,width-1] 会在
    // 旋转/小数平移映射到 width-0.3 等位置时误丢边缘格。
    const bool inside =
        std::isfinite(x) && std::isfinite(y) &&
        x >= -0.5 && y >= -0.5 &&
        x < double(a.width()) - 0.5 &&
        y < double(a.height()) - 0.5 &&
        (ignore_input_domain ||
         domain.contains(static_cast<std::int64_t>(std::floor(y + 0.5)),
                         static_cast<std::int64_t>(std::floor(x + 0.5))));
    if (ch == 0) valid[r * w + c] = inside ? 1 : 0;
    return inside ? sample(a, ch, y, x, mode) : 0.;
  });
  return out.reduce_domain(
      mask_region(w, h, [&](auto r, auto c) { return valid[r * w + c] != 0; }));
}
// 仿射坐标沿输出行单调；按采样使用的同一浮点表达式定位精确有效区间，
// 逐像素只做插值，不用增量坐标避免漂移。
template <bool Bilinear>
Image affine_full_domain(const Image& a, const std::array<double,9>& m,
                         std::size_t w, std::size_t h) {
  advanced::image(a, "remap");
  dimensions(w,h,"remap");
  const auto width=a.width(), height=a.height();
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  const bool avx2=width<=INT32_MAX && height<=INT32_MAX && mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false});
#endif
  auto out=create_like(a,w,h);
  std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};
  runs.reserve(h);
  for(std::size_t r=0;r<h;++r) {
    std::size_t first=0,last=w;
    for(std::size_t axis=0;axis<2 && first<last;++axis) {
      const auto k=axis*3;
      const auto coordinate=[&](std::size_t c){return m[k]*c+m[k+1]*r+m[k+2];};
      const auto search=[&](auto predicate) {
        auto lo=first,hi=last;
        while(lo<hi){const auto mid=lo+(hi-lo)/2;if(predicate(coordinate(mid)))hi=mid;else lo=mid+1;}
        return lo;
      };
      const auto upper=double(axis==0?width:height)-.5;
      if(m[k]>=0) {
        first=search([](double v){return v>=-.5;});
        last=search([&](double v){return v>=upper;});
      } else {
        first=search([&](double v){return v<upper;});
        last=search([](double v){return v<-.5;});
      }
    }
    if(first<last)runs.push_back({static_cast<std::int64_t>(r),static_cast<std::int64_t>(first),static_cast<std::int64_t>(last-1)});
  }
  for(std::size_t ch=0;ch<a.channel_count();++ch) {
    const auto* source=static_cast<const std::byte*>(a.data(ch));
    const auto stride=a.stride(ch), out_stride=out.stride(ch);
    auto* destination=static_cast<std::byte*>(out.mutable_data(ch));
    dispatch_scalar(a.pixel_type(ch),[&](auto tag){
      using T=typename decltype(tag)::type;
      const auto at=[&](std::size_t y,std::size_t x){T v;std::memcpy(&v,source+y*stride+x*sizeof(T),sizeof(T));return double(v);};
      for(const auto& run:runs) {
        auto* row=reinterpret_cast<T*>(destination+std::int64_t(run.row)*out_stride);
        const auto xr=m[1]*run.row, yr=m[4]*run.row;
        auto c=std::int64_t(run.col_begin);
#if MVKIT_FILTER_HAS_AVX2
        if constexpr(std::is_same_v<T,std::uint8_t>) {
          if(avx2)c+=remap_u8_row_avx2(source,row,stride,width,height,m.data(),std::size_t(run.row),std::size_t(c),std::size_t(std::int64_t(run.col_end)+1),Bilinear);
        }
#endif
#if MVKIT_REMAP_SSE2
        if constexpr (std::is_same_v<T, std::uint8_t>) {
          if ((mv::runtime::cpu_features().selection().sse2 || mv::runtime::cpu_features().selection().avx2) &&
              width <= INT32_MAX && height <= INT32_MAX) {
            const auto zero=_mm_setzero_pd(), one=_mm_set1_pd(1.), half=_mm_set1_pd(.5);
            for (; c+1<=run.col_end; c+=2) {
              const auto columns=_mm_set_pd(double(c+1),double(c));
              auto x=_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(m[0]),columns),_mm_set1_pd(xr)),_mm_set1_pd(m[2]));
              auto y=_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(m[3]),columns),_mm_set1_pd(yr)),_mm_set1_pd(m[5]));
              alignas(16) std::int32_t ix[4],iy[4];
              if constexpr (!Bilinear) {
                _mm_store_si128(reinterpret_cast<__m128i*>(ix),_mm_cvttpd_epi32(_mm_add_pd(x,half)));
                _mm_store_si128(reinterpret_cast<__m128i*>(iy),_mm_cvttpd_epi32(_mm_add_pd(y,half)));
                row[c]=std::to_integer<T>(source[std::size_t(iy[0])*stride+std::size_t(ix[0])]);
                row[c+1]=std::to_integer<T>(source[std::size_t(iy[1])*stride+std::size_t(ix[1])]);
              } else {
                x=_mm_min_pd(_mm_max_pd(x,zero),_mm_set1_pd(double(width-1)));
                y=_mm_min_pd(_mm_max_pd(y,zero),_mm_set1_pd(double(height-1)));
                const auto xi=_mm_cvttpd_epi32(x),yi=_mm_cvttpd_epi32(y);
                _mm_store_si128(reinterpret_cast<__m128i*>(ix),xi);
                _mm_store_si128(reinterpret_cast<__m128i*>(iy),yi);
                const auto dx=_mm_sub_pd(x,_mm_cvtepi32_pd(xi)),dy=_mm_sub_pd(y,_mm_cvtepi32_pd(yi));
                const auto ex=_mm_sub_pd(one,dx),ey=_mm_sub_pd(one,dy);
                const auto x0=ix[0],x1=ix[1],y0=iy[0],y1=iy[1];
                const auto nx0=std::min(x0+1,int(width-1)),nx1=std::min(x1+1,int(width-1));
                const auto ny0=std::min(y0+1,int(height-1)),ny1=std::min(y1+1,int(height-1));
                const auto top=_mm_add_pd(_mm_mul_pd(ex,_mm_set_pd(at(y1,x1),at(y0,x0))),_mm_mul_pd(dx,_mm_set_pd(at(y1,nx1),at(y0,nx0))));
                const auto bottom=_mm_add_pd(_mm_mul_pd(ex,_mm_set_pd(at(ny1,x1),at(ny0,x0))),_mm_mul_pd(dx,_mm_set_pd(at(ny1,nx1),at(ny0,nx0))));
                const auto value=_mm_add_pd(_mm_mul_pd(ey,top),_mm_mul_pd(dy,bottom));
                _mm_store_si128(reinterpret_cast<__m128i*>(ix),_mm_cvttpd_epi32(_mm_add_pd(value,half)));
                row[c]=static_cast<T>(ix[0]);row[c+1]=static_cast<T>(ix[1]);
              }
            }
          }
        }
#endif
        for(;c<=run.col_end;++c) {
          auto x=m[0]*c+xr+m[2],y=m[3]*c+yr+m[5];
          if constexpr(!Bilinear) {
            std::memcpy(row+c,source+static_cast<std::int64_t>(y+.5)*stride+static_cast<std::int64_t>(x+.5)*sizeof(T),sizeof(T));
          } else {
            x=std::clamp(x,0.,double(width-1));y=std::clamp(y,0.,double(height-1));
            const auto xx=static_cast<std::int64_t>(x),yy=static_cast<std::int64_t>(y);
            const auto x1=std::min(xx+1,static_cast<std::int64_t>(width-1)),y1=std::min(yy+1,static_cast<std::int64_t>(height-1));
            const auto dx=x-xx,dy=y-yy;
            auto value=(1-dy)*((1-dx)*at(yy,xx)+dx*at(yy,x1))+dy*((1-dx)*at(y1,xx)+dx*at(y1,x1));
            if constexpr(std::is_floating_point_v<T>)row[c]=static_cast<T>(value);
            else {value=std::clamp(value,double(std::numeric_limits<T>::lowest()),double(std::numeric_limits<T>::max()));row[c]=static_cast<T>(value<0?value-.5:value+.5);}
          }
        }
      }
    });
  }
  return out.reduce_domain(Region::from_canonical_runs(std::move(runs)));
}
#if MVKIT_FILTER_HAS_AVX2
Image projective_full_domain(const Image& a,const std::array<double,9>& m,
                              std::size_t w,std::size_t h,Interpolation mode) {
  advanced::image(a,"remap");dimensions(w,h,"remap");advanced::interpolation(mode,"remap");
  const auto width=a.width(),height=a.height(),stride=a.stride();
  const std::array types{mv::core::PixelType::UInt8};
  auto out=Image::create_uninitialized(w,h,types);
  const auto* source=static_cast<const std::uint8_t*>(a.data());
  auto* destination=static_cast<std::uint8_t*>(out.mutable_data());
  ExecutionContext context;
  std::pmr::vector<std::uint8_t> validity(w,context.scratch_resource());
  std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};runs.reserve(h);
  for(std::size_t r=0;r<h;++r) {
    auto* row=destination+r*out.stride();
    auto c=projective_u8_row_avx2(source,row,validity.data(),stride,width,height,m.data(),r,w,mode==Interpolation::Bilinear);
    for(;c<w;++c) {
      const auto denominator=m[6]*c+m[7]*r+m[8];
      double x=NAN,y=NAN;
      if(denominator!=0){x=(m[0]*c+m[1]*r+m[2])/denominator;y=(m[3]*c+m[4]*r+m[5])/denominator;}
      const bool inside=x>=-.5 && y>=-.5 && x<double(width)-.5 && y<double(height)-.5;
      validity[c]=inside;row[c]=0;if(!inside)continue;
      x=std::clamp(x,0.,double(width-1));y=std::clamp(y,0.,double(height-1));
      if(mode!=Interpolation::Bilinear){row[c]=source[std::size_t(y+.5)*stride+std::size_t(x+.5)];continue;}
      const auto ix=std::size_t(x),iy=std::size_t(y),nx=std::min(ix+1,width-1),ny=std::min(iy+1,height-1);
      const auto dx=x-ix,dy=y-iy;
      row[c]=std::uint8_t((1-dy)*((1-dx)*source[iy*stride+ix]+dx*source[iy*stride+nx])+
                           dy*((1-dx)*source[ny*stride+ix]+dx*source[ny*stride+nx])+.5);
    }
    for(std::size_t first=0;first<w;) {
      while(first<w&&!validity[first])++first;
      auto last=first;while(last<w&&validity[last])++last;
      if(first<last)runs.push_back({std::int64_t(r),std::int64_t(first),std::int64_t(last-1)});
      first=last;
    }
  }
  return out.reduce_domain(Region::from_canonical_runs(std::move(runs)));
}
#endif
// 整数平移的最近邻映射精确落在像素中心；裁剪后的有效行可整段复制。
// 坐标均限制在 int32 内，使原 Double 坐标表达式保持精确整数结果。
Image translated_full_domain(const Image& a, std::int64_t dx, std::int64_t dy,
                             std::size_t w, std::size_t h) {
  advanced::image(a, "remap");
  dimensions(w, h, "remap");
  auto out = create_like(a, w, h);
  const auto first_column = std::max<std::int64_t>(0, -dx);
  const auto last_column = std::min<std::int64_t>(w, std::int64_t(a.width()) - dx);
  const auto first_row = std::max<std::int64_t>(0, -dy);
  const auto last_row = std::min<std::int64_t>(h, std::int64_t(a.height()) - dy);
  std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};
  if (first_column < last_column && first_row < last_row) {
    runs.reserve(static_cast<std::size_t>(last_row - first_row));
    for (auto r = first_row; r < last_row; ++r)
      runs.push_back({r, first_column, last_column - 1});
    for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
      const auto bytes = mv::core::bytes_per_sample(a.pixel_type(ch));
      const auto* source = static_cast<const std::byte*>(a.data(ch));
      auto* destination = static_cast<std::byte*>(out.mutable_data(ch));
      for (auto r = first_row; r < last_row; ++r)
        std::memcpy(destination + r * out.stride(ch) + first_column * bytes,
                    source + (r + dy) * a.stride(ch) + (first_column + dx) * bytes,
                    static_cast<std::size_t>(last_column - first_column) * bytes);
    }
  }
  return out.reduce_domain(Region::from_canonical_runs(std::move(runs)));
}
Image matrix_map(const Image& a, const std::array<double, 9>& m, std::size_t w,
                 std::size_t h, Interpolation mode, bool ignore_input_domain = false) {
  const bool affine = m[6] == 0 && m[7] == 0 && m[8] == 1;
  bool bounded = true;
  for (std::size_t k : {0U,3U}) for (double r : {0.,double(h)}) for (double c : {0.,double(w)})
    bounded = bounded && std::isfinite(m[k]*c) && std::isfinite(m[k+1]*r) && std::isfinite(m[k]*c+m[k+1]*r+m[k+2]);
  if (affine && bounded && (ignore_input_domain || !a.has_reduced_domain())) {
    if (mode == Interpolation::NearestNeighbor || mode == Interpolation::Constant) {
      if (m[0] == 1 && m[1] == 0 && m[3] == 0 && m[4] == 1 &&
          w <= INT32_MAX && h <= INT32_MAX && a.width() <= INT32_MAX && a.height() <= INT32_MAX &&
          m[2] >= INT32_MIN && m[2] <= INT32_MAX && m[5] >= INT32_MIN && m[5] <= INT32_MAX &&
          std::trunc(m[2]) == m[2] && std::trunc(m[5]) == m[5])
        return translated_full_domain(a, static_cast<std::int64_t>(m[2]),
                                      static_cast<std::int64_t>(m[5]), w, h);
      return affine_full_domain<false>(a,m,w,h);
    }
    if (mode == Interpolation::Bilinear) return affine_full_domain<true>(a,m,w,h);
  }
  if (affine)
    return remap(a, w, h, mode, [&](auto r, auto c) {
      return std::array<double, 2>{m[0] * c + m[1] * r + m[2],
                                   m[3] * c + m[4] * r + m[5]};
    }, ignore_input_domain);
#if MVKIT_FILTER_HAS_AVX2
  if(!a.has_reduced_domain() && a.channel_count()==1 && a.pixel_type()==mv::core::PixelType::UInt8 &&
      a.width()<=INT32_MAX && a.height()<=INT32_MAX && w<=INT32_MAX && h<=INT32_MAX &&
      (mode==Interpolation::NearestNeighbor || mode==Interpolation::Constant || mode==Interpolation::Bilinear)) {
    const auto& cpu=mv::runtime::cpu_features();
    if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false}))
      return projective_full_domain(a,m,w,h,mode);
  }
#endif
  return remap(a, w, h, mode, [&](auto r, auto c) {
    const double denominator = m[6] * c + m[7] * r + m[8];
    if (denominator == 0) return std::array<double, 2>{NAN, NAN};
    return std::array<double, 2>{(m[0] * c + m[1] * r + m[2]) / denominator,
                                 (m[3] * c + m[4] * r + m[5]) / denominator};
  }, ignore_input_domain);
}
}  // namespace
namespace advanced {
std::array<double, 9> inverse(const Tuple& matrix, bool affine,
                              const char* op) {
  check(matrix.size() == (affine ? 6U : 9U), op, "Matrix",
        "expected six affine or nine projective coefficients");
  double a[3][6]{};
  for (std::size_t i = 0; i < 9; ++i) {
    double v = i < matrix.size() ? matrix.as_real64(i) : (i == 8 ? 1. : 0.);
    check(std::isfinite(v), op, "Matrix", "matrix must be finite");
    a[i / 3][i % 3] = v;
  }
  for (int i = 0; i < 3; ++i) a[i][i + 3] = 1;
  for (int c = 0; c < 3; ++c) {
    int pivot = c;
    for (int r = c + 1; r < 3; ++r)
      if (std::abs(a[r][c]) > std::abs(a[pivot][c])) pivot = r;
    check(a[pivot][c] != 0, op, "Matrix", "matrix is singular");
    for (int j = 0; j < 6; ++j) std::swap(a[pivot][j], a[c][j]);
    const double v = a[c][c];
    for (int j = 0; j < 6; ++j) a[c][j] /= v;
    for (int r = 0; r < 3; ++r)
      if (r != c) {
        const double f = a[r][c];
        for (int j = 0; j < 6; ++j) a[r][j] -= f * a[c][j];
      }
  }
  std::array<double, 9> result{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      result[r * 3 + c] = a[r][c + 3];
      check(std::isfinite(result[r * 3 + c]), op, "Matrix",
            "inverse is not representable");
    }
  return result;
}
Image zoom_extended(const Image& a, std::size_t w, std::size_t h,
                    Interpolation mode) {
  image(a, "zoom_image_size");
  dimensions(w, h, "zoom_image_size");
  interpolation(mode, "zoom_image_size", true);
  ExecutionContext allocation_context;
  std::pmr::vector<PixelType> types{allocation_context.scratch_resource()};
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch) types.push_back(a.pixel_type(ch));
  auto out = Image::create_uninitialized(w, h, types);
  const double sx = double(a.width()) / w, sy = double(a.height()) / h;
  if (mode == Interpolation::NearestNeighbor || mode == Interpolation::Constant ||
      mode == Interpolation::Bilinear) {
    ExecutionContext context;
    const bool nearest = mode != Interpolation::Bilinear;
    const bool half_size = !nearest && sx == 2 && sy == 2;
#if MVKIT_FILTER_HAS_AVX2
    const auto& zoom_cpu=context.runtime().cpu();
    const bool zoom_avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
        zoom_cpu.selection(),zoom_cpu.hardware()),{false,true,false,false});
#endif
    const auto width = a.width(), height = a.height();
    std::pmr::vector<std::uint32_t> columns(w, context.scratch_resource());
    std::pmr::vector<double> fractions(nearest ? 0 : w, context.scratch_resource());
    for (std::size_t c = 0; c < w; ++c) {
      const double x = std::clamp(nearest || half_size ? c * sx : (c + .5) * sx - .5,
                                  0., double(width - 1));
      columns[c] = static_cast<std::uint32_t>(x);
      if (!nearest) fractions[c] = x - columns[c];
    }
#if MVKIT_FILTER_HAS_AVX2
    std::pmr::vector<ZoomNearestU8Block> blocks{context.scratch_resource()};
    bool has_byte=false;
    for(auto type:types)has_byte|=type==PixelType::UInt8;
    if(nearest && zoom_avx2 && has_byte && width<=std::numeric_limits<std::uint32_t>::max()) {
      blocks.reserve(w/16);
      for(std::size_t c=0;c+16<=w;c+=16) {
        const auto first=columns[c];
        // 单调源索引必须容纳两次完整且不越界的 16 字节 load；大比例缩小及末源窗口保留标量 gather。
        if(first>width || width-first<32 || columns[c+15]-first>=32)break;
        ZoomNearestU8Block block{first};
        for(std::size_t k=0;k<16;++k) {
          const auto x=columns[c+k]-first;
          block.low[k]=static_cast<std::uint8_t>(x<16?x:128);
          block.high[k]=static_cast<std::uint8_t>(x>=16?x-16:128);
        }
        blocks.push_back(block);
      }
    }
#endif
    for (std::size_t ch = 0; ch < a.channel_count(); ++ch) {
      const auto* source = static_cast<const std::byte*>(a.data(ch));
      const auto stride = a.stride(ch), output_stride = out.stride(ch);
      auto* destination = static_cast<std::byte*>(out.mutable_data(ch));
      dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
        using T = typename decltype(tag)::type;
        ExecutionContext::for_each_partition(h, ExecutionContext::saturated_work(w, h),
            ExecutionContext::kArithmeticParallelWork, [&](auto range) {
          for (auto r = range.begin; r < range.end; ++r) {
            const double y = std::clamp(nearest || half_size ? r * sy : (r + .5) * sy - .5,
                                        0., double(height - 1));
            const auto yy = static_cast<std::size_t>(y);
            const auto* top = source + yy * stride;
            const auto* bottom = source + std::min(yy + 1, height - 1) * stride;
            auto* row = reinterpret_cast<T*>(destination + r * output_stride);
            const double dy = y - yy;
            const auto at = [](const std::byte* data, std::size_t x) {
              T value; std::memcpy(&value, data + x * sizeof(T), sizeof(T)); return value;
            };
            if (nearest) {
              std::size_t c = 0;
              if constexpr (std::is_same_v<T,std::uint8_t>) {
#if MVKIT_FILTER_HAS_AVX2
                if(!blocks.empty())c=zoom_nearest_u8_row_avx2(
                    reinterpret_cast<const std::uint8_t*>(top),row,blocks);
#endif
                for (; c+8 <= w; c+=8) {
                  const std::uint64_t packed = std::uint64_t(at(top,columns[c])) |
                      (std::uint64_t(at(top,columns[c+1])) << 8) |
                      (std::uint64_t(at(top,columns[c+2])) << 16) |
                      (std::uint64_t(at(top,columns[c+3])) << 24) |
                      (std::uint64_t(at(top,columns[c+4])) << 32) |
                      (std::uint64_t(at(top,columns[c+5])) << 40) |
                      (std::uint64_t(at(top,columns[c+6])) << 48) |
                      (std::uint64_t(at(top,columns[c+7])) << 56);
                  if constexpr (std::endian::native == std::endian::little)
                    std::memcpy(row+c,&packed,sizeof(packed));
                  else for (std::size_t k=0;k<8;++k) row[c+k]=static_cast<T>(packed >> (8*k));
                }
              }
              for (; c < w; ++c) row[c] = at(top, columns[c]);
            } else {
              std::size_t c=0;
#if MVKIT_FILTER_HAS_AVX2
              if constexpr(std::is_same_v<T,std::uint8_t>) {
                if(zoom_avx2)c=zoom_bilinear_u8_row_avx2(
                    reinterpret_cast<const std::uint8_t*>(top),reinterpret_cast<const std::uint8_t*>(bottom),
                    row,columns,fractions,width,dy);
              }
#endif
              for (; c < w; ++c) {
                const std::size_t x = columns[c], x1 = std::min(x + 1, width - 1);
                const auto dx = fractions[c];
                row[c] = saturate_cast<T>((1 - dy) * ((1 - dx) * double(at(top, x)) + dx * double(at(top, x1))) +
                                        dy * ((1 - dx) * double(at(bottom, x)) + dx * double(at(bottom, x1))));
              }
            }
          }
        });
      });
    }
    return out;
  }
  pixels(out, [&](auto ch, auto r, auto c) {
    if (mode != Interpolation::Weighted) {
      // 精确 2x2 降采样对应 HALCON 双线性半尺寸特例。
      const bool half_size = mode == Interpolation::Bilinear && sx == 2 && sy == 2;
      const double source_row = half_size ? r * sy : (r + .5) * sy - .5;
      const double source_col = half_size ? c * sx : (c + .5) * sx - .5;
      if (mode == Interpolation::NearestNeighbor || mode == Interpolation::Constant) {
        const auto rr = static_cast<std::size_t>(std::clamp(
            std::floor(r * sy), 0., double(a.height() - 1)));
        const auto cc = static_cast<std::size_t>(std::clamp(
            std::floor(c * sx), 0., double(a.width() - 1)));
        return read(a, ch, rr, cc);
      }
      return sample(a, ch, source_row, source_col, mode);
    }
    const double x0 = c * sx, x1 = (c + 1) * sx, y0 = r * sy, y1 = (r + 1) * sy;
    double sum = 0;
    const auto end_y = std::min(a.height(), std::size_t(std::ceil(y1))),
               end_x = std::min(a.width(), std::size_t(std::ceil(x1)));
    for (auto y = std::size_t(std::floor(y0)); y < end_y; ++y)
      for (auto x = std::size_t(std::floor(x0)); x < end_x; ++x)
        sum += read(a, ch, y, x) *
               (std::min(x1, double(x + 1)) - std::max(x0, double(x))) *
               (std::min(y1, double(y + 1)) - std::max(y0, double(y)));
    return sum / (sx * sy);
  });
  return out;
}
}  // namespace advanced
mv::core::Image affine_trans_image(const mv::core::Image& a,
                                   const mv::core::Tuple& matrix, std::size_t w,
                                   std::size_t h, Interpolation mode) {
  return matrix_map(a, advanced::inverse(matrix, true, "affine_trans_image"), w,
                    h, mode, true);
}
mv::core::Image projective_trans_image(const mv::core::Image& a,
                                       const mv::core::Tuple& matrix,
                                       std::size_t w, std::size_t h,
                                       Interpolation mode) {
  return matrix_map(a,
                    advanced::inverse(matrix, false, "projective_trans_image"),
                    w, h, mode);
}
mv::core::Image rotate_image(const mv::core::Image& a, double angle,
                             Interpolation mode) {
  advanced::image(a, "rotate_image");
  check(std::isfinite(angle), "rotate_image", "Angle",
        "angle must be finite radians");
  constexpr double half_pi = 1.57079632679489661923;
  const double normalized = std::remainder(angle, 4 * half_pi);
  const double quarter_turns = std::round(normalized / half_pi);
  const bool right_angle = std::abs(normalized - quarter_turns * half_pi) < 1e-12;
  double co = std::cos(angle), si = std::sin(angle);
  if (right_angle) {
    const auto quadrant = (static_cast<int>(quarter_turns) % 4 + 4) % 4;
    if ((mode==Interpolation::NearestNeighbor || mode==Interpolation::Constant) &&
        a.width()<=INT32_MAX && a.height()<=INT32_MAX) {
      switch(quadrant) {
        case 0:return rotate_right_angle<0>(a);
        case 1:return rotate_right_angle<1>(a);
        case 2:return rotate_right_angle<2>(a);
        default:return rotate_right_angle<3>(a);
      }
    }
    co = quadrant == 0 ? 1.0 : quadrant == 2 ? -1.0 : 0.0;
    si = quadrant == 1 ? 1.0 : quadrant == 3 ? -1.0 : 0.0;
  }
  const bool swap_axes = right_angle && static_cast<int>(quarter_turns) % 2 != 0;
  const auto width = swap_axes ? a.height() : a.width();
  const auto height = swap_axes ? a.width() : a.height();
  const double cx = (a.width() - 1) * .5, cy = (a.height() - 1) * .5;
  const double out_cx = (width - 1) * .5, out_cy = (height - 1) * .5;
  return matrix_map(a,
                    {co, si, cx - co * out_cx - si * out_cy, -si, co,
                     cy + si * out_cx - co * out_cy, 0, 0, 1},
                    width, height, mode, true);
}
mv::core::Image map_image(const mv::core::Image& a, const mv::core::Image& rows,
                          const mv::core::Image& cols, Interpolation mode) {
  advanced::image(rows, "map_image");
  advanced::image(cols, "map_image");
  check(rows.width() == cols.width() && rows.height() == cols.height() &&
            rows.channel_count() == 1 && cols.channel_count() == 1 &&
            rows.pixel_type() == mv::core::PixelType::Float32 &&
            cols.pixel_type() == mv::core::PixelType::Float32,
        "map_image", "Maps",
        "maps must be equally sized single-channel Float32 images");
  const auto domain = rows.domain().intersect(cols.domain());
  const bool full_maps=!rows.has_reduced_domain() && !cols.has_reduced_domain();
  const auto* row_data=static_cast<const std::byte*>(rows.data());
  const auto* col_data=static_cast<const std::byte*>(cols.data());
  const auto row_stride=rows.stride(),col_stride=cols.stride();
  advanced::image(a,"remap");
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  if(full_maps && !a.has_reduced_domain() && a.channel_count()==1 &&
      a.pixel_type()==mv::core::PixelType::UInt8 && mode==Interpolation::Bilinear &&
      a.width()<=INT32_MAX && a.height()<=INT32_MAX &&
      mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),{false,false,true,false})) {
    const auto w=rows.width(),h=rows.height(),width=a.width(),height=a.height();
    const std::array types{mv::core::PixelType::UInt8};
    auto out=Image::create_uninitialized(w,h,types);
    const auto* input=static_cast<const std::uint8_t*>(a.data());
    auto* output=static_cast<std::uint8_t*>(out.mutable_data());
    const auto stride=a.stride(),out_stride=out.stride();
    ExecutionContext context;
    std::pmr::vector<std::uint8_t> validity(w,context.scratch_resource());
    std::pmr::vector<RegionRun> runs{mv::runtime::persist_resource()};runs.reserve(h);
    for(std::size_t r=0;r<h;++r) {
      const auto* rr=reinterpret_cast<const float*>(row_data+r*row_stride);
      const auto* cc=reinterpret_cast<const float*>(col_data+r*col_stride);
      auto* target=output+r*out_stride;
      bool all_valid=true;
      auto c=remap_float_u8_row_avx2(input,target,validity.data(),stride,width,height,rr,cc,w,all_valid);
      for(;c<w;++c) {
        float fx,fy;std::memcpy(&fx,col_data+r*col_stride+c*sizeof(float),sizeof(float));
        std::memcpy(&fy,row_data+r*row_stride+c*sizeof(float),sizeof(float));
        double x=fx,y=fy;
        const bool inside=x>=-.5 && y>=-.5 && x<double(width)-.5 && y<double(height)-.5;
        validity[c]=inside;
        all_valid=all_valid && inside;
        if(!inside){target[c]=0;continue;}
        x=std::clamp(x,0.,double(width-1));y=std::clamp(y,0.,double(height-1));
        const auto xx=static_cast<std::size_t>(x),yy=static_cast<std::size_t>(y);
        const auto nx=std::min(xx+1,width-1),ny=std::min(yy+1,height-1);
        const auto dx=x-xx,dy=y-yy;
        const double value=(1-dy)*((1-dx)*input[yy*stride+xx]+dx*input[yy*stride+nx])+
                              dy*((1-dx)*input[ny*stride+xx]+dx*input[ny*stride+nx]);
        target[c]=static_cast<std::uint8_t>(value+.5);
      }
      if(all_valid && w)runs.push_back({static_cast<std::int64_t>(r),0,static_cast<std::int64_t>(w-1)});
      else for(std::size_t start=0;start<w;) {
        while(start<w && !validity[start])++start;
        auto end=start;while(end<w && validity[end])++end;
        if(start<end)runs.push_back({static_cast<std::int64_t>(r),static_cast<std::int64_t>(start),static_cast<std::int64_t>(end-1)});
        start=end;
      }
    }
    return out.reduce_domain(Region::from_canonical_runs(std::move(runs)));
  }
#endif
  return remap(a, rows.width(), rows.height(), mode, [&](auto r, auto c) {
    if (!full_maps && !domain.contains(r, c)) return std::array<double, 2>{NAN, NAN};
    float row,col;
    std::memcpy(&row,row_data+r*row_stride+c*sizeof(float),sizeof(float));
    std::memcpy(&col,col_data+r*col_stride+c*sizeof(float),sizeof(float));
    return std::array<double, 2>{col,row};
  }, false);
}
mv::core::Image polar_trans_image_ext(const mv::core::Image& a, double cr,
                                      double cc, double as, double ae,
                                      double rs, double re, std::size_t w,
                                      std::size_t h, Interpolation mode) {
  check(std::isfinite(cr) && std::isfinite(cc) && std::isfinite(as) &&
            std::isfinite(ae) && std::isfinite(rs) && std::isfinite(re) &&
            ae > as && ae - as <= 2 * 3.14159265358979323846 + 1e-12 &&
            rs >= 0 && re > rs,
        "polar_trans_image_ext", "Range", "invalid polar ranges");
  return remap(a, w, h, mode, [&](auto r, auto c) {
    const double angle = as + (ae - as) * (w == 1 ? 0. : double(c) / (w - 1)),
                 radius = rs + (re - rs) * (h == 1 ? 0. : double(r) / (h - 1));
    return std::array<double, 2>{cc + radius * std::cos(angle),
                                 cr - radius * std::sin(angle)};
  });
}

Image transform_image_halcon(const Image& image,const Tuple& matrix,Interpolation mode,
                            bool projective,bool adapt_size,bool transform_domain) {
  constexpr auto op="transform_image";
  for(std::size_t ch=0;ch<image.channel_count();++ch) {
    const auto type=image.pixel_type(ch);
    if(type==PixelType::Int32||(projective&&type==PixelType::Int16))
      throw Exception(Status::TypeMismatch,ErrorCode::InvalidType,op,"Image",-1,
                      projective?"byte, uint2 or real image required":"byte, int2, uint2 or real image required");
  }
  (void)advanced::inverse(matrix,!projective,op);
  std::array<double,9> raw{0,0,0,0,0,0,0,0,1};
  for(std::size_t i=0;i<matrix.size();++i)raw[i]=matrix.as_real64(i);
  auto width=image.width(),height=image.height();
  if(adapt_size) {
    double max_row=0,max_col=0;
    for(double row:{0.,double(height)})for(double col:{0.,double(width)}) {
      const auto d=raw[6]*row+raw[7]*col+raw[8];
      check(d!=0,op,"HomMat2D","image corner projects to infinity");
      const auto y=(raw[0]*row+raw[1]*col+raw[2])/d;
      const auto x=(raw[3]*row+raw[4]*col+raw[5])/d;
      check(std::isfinite(x)&&std::isfinite(y),op,"HomMat2D","finite output corners required");
      max_row=std::max(max_row,y);max_col=std::max(max_col,x);
    }
    check(max_row>0&&max_col>0&&max_row<=INT32_MAX&&max_col<=INT32_MAX,op,"AdaptImageSize","positive representable output size required");
    height=static_cast<std::size_t>(std::ceil(max_row));width=static_cast<std::size_t>(std::ceil(max_col));
  }
  // HALCON 行列像素角点转为内核列行像素中心：P*Translate(-.5,-.5)*H*Translate(.5,.5)*P。
  auto centered=raw;
  for(std::size_t r=0;r<3;++r)centered[r*3+2]+=.5*(raw[r*3]+raw[r*3+1]);
  for(std::size_t c=0;c<3;++c){centered[c]-=.5*centered[6+c];centered[3+c]-=.5*centered[6+c];}
  const std::array<std::size_t,9> permutation{4,3,5,1,0,2,7,6,8};
  Tuple kernel_matrix;for(std::size_t i=0;i<(projective?9u:6u);++i)kernel_matrix.append(centered[permutation[i]]);
  const auto source=projective&&transform_domain?image:image.full_domain();
  if(mode==Interpolation::Constant||mode==Interpolation::Weighted) {
    const bool shrinking=std::hypot(raw[0],raw[3])<1-1e-12||std::hypot(raw[1],raw[4])<1-1e-12;
    if(shrinking)throw Exception(Status::Unsupported,ErrorCode::UnsupportedValue,op,"Interpolation",-1,"anti-aliased affine downsampling is not implemented");
    mode=Interpolation::Bilinear;
  }
  return projective?projective_trans_image(source,kernel_matrix,width,height,mode):affine_trans_image(source,kernel_matrix,width,height,mode);
}
Image map_image_linear(const Image &image, const Image &map) {
  constexpr auto op = "map_image";
  for (std::size_t ch = 0; ch < image.channel_count(); ++ch)
    if (image.pixel_type(ch) == PixelType::Int16 ||
        image.pixel_type(ch) == PixelType::Int32)
      throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, op, "Image",
                      -1, "byte, uint2 or real image required");
  check(map.channel_count() == 1 || map.channel_count() == 5, op, "Map",
        "one linear-index or five weighted channels required");
  check(map.pixel_type() == PixelType::Int32, op, "Map",
        "first map channel must be int4");
  if (map.channel_count() == 5)
    for (std::size_t c = 1; c < 5; ++c)
      check(map.pixel_type(c) == PixelType::UInt16, op, "Map",
            "weight channels must be uint2");
  const auto count = image.width() * image.height();
  const auto domain = map.domain();
  // 读取前拒绝非法索引，不复现 HALCON 未检查的 gather。
  const auto *indices = static_cast<const std::byte *>(map.data());
  const auto map_stride = map.stride();
  const bool weighted = map.channel_count() == 5;
  bool contiguous = !weighted;
#if MVKIT_REMAP_SSE2
  const auto& map_cpu=runtime::cpu_features();
  const bool validate_simd=!weighted && runtime::kernel_allowed(runtime::effective_capabilities(
      map_cpu.selection(),map_cpu.hardware()),{true,false,false,false});
#endif
  for (const auto &run : domain.runs()) {
    const auto *row = indices + std::size_t(run.row) * map_stride;
    const auto first = load_pixel<std::int32_t>(
        row + std::size_t(run.col_begin) * sizeof(std::int32_t));
    auto c=std::int64_t(run.col_begin);
#if MVKIT_REMAP_SSE2
    if(validate_simd) {
      // 每个索引仍检查范围；连续性独立归约，遇到非法块按原顺序逐项报告。
      const bool bounded=first>=0 && std::int64_t(first)+run.col_end-run.col_begin<=INT32_MAX;
      contiguous=contiguous && bounded;
      auto expected=_mm_add_epi32(_mm_set1_epi32(first),_mm_setr_epi32(0,1,2,3));
      for(;std::int64_t(run.col_end)-c+1>=4;c+=4) {
        const auto values=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+c*sizeof(std::int32_t)));
        auto valid=_mm_cmpgt_epi32(values,_mm_set1_epi32(-1));
        if(count<=INT32_MAX)valid=_mm_and_si128(valid,_mm_cmpgt_epi32(_mm_set1_epi32(static_cast<int>(count)),values));
        if(_mm_movemask_epi8(valid)!=65535)break;
        contiguous=contiguous && _mm_movemask_epi8(_mm_cmpeq_epi32(values,expected))==65535;
        expected=_mm_add_epi32(expected,_mm_set1_epi32(4));
      }
    }
#endif
    for (; c <= run.col_end; ++c) {
      const auto index =
          load_pixel<std::int32_t>(row + c * sizeof(std::int32_t));
      contiguous = contiguous && std::int64_t(index) ==
                                     std::int64_t(first) + c - run.col_begin;
      check(index >= 0 && std::uint64_t(index) < count, op, "Map",
            "source index outside image");
      const auto i = static_cast<std::size_t>(index);
      if (weighted)
        check(i / image.width() + 1 < image.height() &&
                  i % image.width() + 1 < image.width(),
              op, "Map", "weighted map requires all four source neighbors");
    }
  }
  auto out = create_like(image, map.width(), map.height());
  std::array<const std::byte *, 4> weights{};
  std::array<std::size_t, 4> weight_strides{};
  if (weighted)
    for (std::size_t k = 0; k < 4; ++k) {
      weights[k] = static_cast<const std::byte *>(map.data(k + 1));
      weight_strides[k] = map.stride(k + 1);
    }
  // 类型和地址只解析一次，逐行程遍历；保留四项 Double 累加、除数和饱和舍入。
  for (std::size_t ch = 0; ch < image.channel_count(); ++ch) {
    const auto *source = static_cast<const std::byte *>(image.data(ch));
    auto *destination = static_cast<std::byte *>(out.mutable_data(ch));
    const auto stride = image.stride(ch), out_stride = out.stride(ch),
               width = image.width();
    const double divisor =
        image.pixel_type(ch) == PixelType::Float32 ? 65535. : 65536.;
    dispatch_scalar(image.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      const bool packed = stride == width * sizeof(T);
      for (const auto &run : domain.runs()) {
        const auto r = std::size_t(run.row);
        const auto *index = indices + r * map_stride;
        auto *row = reinterpret_cast<T *>(destination + r * out_stride);
        if constexpr (std::is_integral_v<T>) {
          // 连续整数行程已逐索引验证，仅无 padding 的源可跨行复制。
          if (contiguous && packed) {
            const auto first = load_pixel<std::int32_t>(
                index + std::size_t(run.col_begin) * sizeof(std::int32_t));
            std::memcpy(
                row + run.col_begin, source + std::size_t(first) * sizeof(T),
                (std::int64_t(run.col_end) - run.col_begin + 1) * sizeof(T));
            continue;
          }
        }
        std::array<const std::byte *, 4> w{};
        if (weighted)
          for (std::size_t k = 0; k < 4; ++k)
            w[k] = weights[k] + r * weight_strides[k];
        for (auto c = std::int64_t(run.col_begin); c <= run.col_end; ++c) {
          const auto i = static_cast<std::size_t>(
              load_pixel<std::int32_t>(index + c * sizeof(std::int32_t)));
          if (!weighted && packed) {
            const auto value = load_pixel<T>(source + i * sizeof(T));
            if constexpr (std::is_integral_v<T>)
              row[c] = value;
            else
              row[c] = saturate_cast<T>(double(value));
            continue;
          }
          const auto y = i / width, x = i % width;
          const auto *in = source + y * stride;
          if (!weighted) {
            const auto value = load_pixel<T>(in + x * sizeof(T));
            if constexpr (std::is_integral_v<T>)
              row[c] = value;
            else
              row[c] = saturate_cast<T>(double(value));
          } else {
            const auto *next = source + (y + 1) * stride;
            row[c] = saturate_cast<T>(
                (double(load_pixel<std::uint16_t>(w[0] +
                                                  c * sizeof(std::uint16_t))) *
                     double(load_pixel<T>(in + x * sizeof(T))) +
                 double(load_pixel<std::uint16_t>(w[1] +
                                                  c * sizeof(std::uint16_t))) *
                     double(load_pixel<T>(in + (x + 1) * sizeof(T))) +
                 double(load_pixel<std::uint16_t>(w[2] +
                                                  c * sizeof(std::uint16_t))) *
                     double(load_pixel<T>(next + x * sizeof(T))) +
                 double(load_pixel<std::uint16_t>(w[3] +
                                                  c * sizeof(std::uint16_t))) *
                     double(load_pixel<T>(next + (x + 1) * sizeof(T)))) /
                divisor);
          }
        }
      }
    });
  }
  return out.reduce_domain(domain);
}
Image polar_trans_image_halcon(const Image &image, double row, double col,
                               double start_angle, double end_angle,
                               double start_radius, double end_radius,
                               std::size_t width, std::size_t height,
                               Interpolation mode) {
  constexpr auto op = "polar_trans_image_ext";
  for (std::size_t ch = 0; ch < image.channel_count(); ++ch)
    if (image.pixel_type(ch) == PixelType::Int32)
      throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, op, "Image",
                      -1, "byte, int2, uint2 or real image required");
  check(start_radius >= 0 && end_radius >= 0 && start_radius != end_radius, op,
        "Radius", "nonnegative, distinct radii required");
  return remap(
             image.full_domain(), width, height, mode,
             [&](auto r, auto c) {
               const auto angle =
                   start_angle +
                   (end_angle - start_angle) *
                       (width == 1 ? 0. : double(c) / (width - 1));
               const auto radius =
                   start_radius +
                   (end_radius - start_radius) *
                       (height == 1 ? 0. : double(r) / (height - 1));
               return std::array<double, 2>{col + radius * std::cos(angle),
                                            row - radius * std::sin(angle)};
             },
             true)
      .full_domain();
}

} // namespace mv::imageproc
