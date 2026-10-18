#include <imageproc/box_mean.hpp>
#include <imageproc/border.hpp>
#include <imageproc/math.hpp>
#include <imageproc/execution_context.hpp>
#include <cfenv>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#include <imageproc/filters_avx2.hpp>
#endif

namespace mv::imageproc {
void emphasize_byte_row(const std::uint8_t* source,const float* mean,std::uint8_t* output,
                        std::size_t width,double factor) {
  std::size_t first=0;
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  if(mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false}))
    first=emphasize_u8_row_avx2(source,mean,output,width,factor);
#endif
  for(auto c=first;c<width;++c) {
    const double value=source[c];
    output[c]=saturate_cast<std::uint8_t>(value+factor*(value-double(mean[c])));
  }
}
std::optional<core::Image> exact_integer_box_mean(const core::Image& image,
                                                std::size_t w, std::size_t h) {
  if (image.empty() || !w || !h || !(w & 1) || !(h & 1) || w > 127 || h > 127 ||
      std::fegetround()!=FE_TONEAREST)
    return std::nullopt;
  for (std::size_t ch=0;ch<image.channel_count();++ch)
    if (image.pixel_type(ch)!=core::PixelType::UInt8 && image.pixel_type(ch)!=core::PixelType::UInt16)
      return std::nullopt;
  // 原正权卷积相对误差由 gamma_(w+h+4) 界定；奇数面积的精确均值与
  // Float32 半舍入点相对距离至少 1/(area*2^25)。此限制保留至少 16 倍余量。
  const auto area=static_cast<std::uint32_t>(w*h);
  if (std::uint64_t(area)*(w+h+4) > (1U<<24)) return std::nullopt;
  const auto width=image.width(),height=image.height();
  std::vector<core::PixelType> types(image.channel_count(),core::PixelType::Float32);
  auto output=core::Image::create_uninitialized(width,height,types);
  ExecutionContext context;
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=context.runtime().cpu();
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif
  const auto rx=static_cast<std::int64_t>(w/2),ry=static_cast<std::int64_t>(h/2);
  std::pmr::vector<std::size_t> leaving(width,context.scratch_resource()),entering(width,context.scratch_resource());
  for(std::size_t c=0;c<width;++c) {
    leaving[c]=*border_index(static_cast<std::int64_t>(c)-rx,width,BorderMode::Mirrored);
    entering[c]=*border_index(static_cast<std::int64_t>(c)+rx+1,width,BorderMode::Mirrored);
  }
  for (std::size_t ch=0;ch<image.channel_count();++ch) {
    const auto* source=static_cast<const std::byte*>(image.data(ch));
    const auto stride=image.stride(ch),out_stride=output.stride(ch);
    auto* destination=static_cast<std::byte*>(output.mutable_data(ch));
    const bool byte=image.pixel_type(ch)==core::PixelType::UInt8;
    const auto calculate=[&]<class T>() {
      ExecutionContext::for_each_partition(height,width*height,ExecutionContext::kArithmeticParallelWork,[&](auto range) {
        ExecutionContext local;
        // 每个 worker 只保留一行列和；行前缀在 SIMD 寄存器内滚动。
        std::pmr::vector<std::uint32_t> columns(width,0,local.scratch_resource());
        const auto update=[&](const std::byte* before,const std::byte* after) {
#if MVKIT_FILTER_HAS_AVX2
          if(avx2) {box_columns_avx2(columns.data(),before,after,width,image.pixel_type(ch));return;}
#endif
          for(std::size_t c=0;c<width;++c)columns[c]=columns[c]-
              (before?load_pixel<T>(before+c*sizeof(T)):0)+load_pixel<T>(after+c*sizeof(T));
        };
        for(auto dy=-ry;dy<=ry;++dy)update(nullptr,source+*border_index(static_cast<std::int64_t>(range.begin)+dy,height,BorderMode::Mirrored)*stride);
        for(auto r=range.begin;r<range.end;++r) {
          std::uint32_t sum=0;
          for(auto dx=-rx;dx<=rx;++dx)sum+=columns[*border_index(dx,width,BorderMode::Mirrored)];
          auto* row=reinterpret_cast<float*>(destination+r*out_stride);
#if MVKIT_FILTER_HAS_AVX2
          if(avx2)box_mean_columns_row_avx2(columns.data(),leaving.data(),entering.data(),row,width,w/2,area,sum,byte);
          else
#endif
          for(std::size_t c=0;c<width;++c) {
            row[c]=byte?float(sum)/float(area):static_cast<float>(double(sum)/area);
            if(c+1<width)sum=sum-columns[leaving[c]]+columns[entering[c]];
          }
          if(r+1<range.end)update(source+*border_index(static_cast<std::int64_t>(r)-ry,height,BorderMode::Mirrored)*stride,
              source+*border_index(static_cast<std::int64_t>(r)+ry+1,height,BorderMode::Mirrored)*stride);
        }
      });
    };
    if(byte)calculate.template operator()<std::uint8_t>();
    else calculate.template operator()<std::uint16_t>();
  }
  return output;
}
}
