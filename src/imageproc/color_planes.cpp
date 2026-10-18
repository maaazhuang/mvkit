#include <imageproc/color_planes.hpp>
#include <imageproc/math.hpp>
#include <cmath>
#if MVKIT_FILTER_HAS_AVX2
#include <imageproc/arithmetic_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>
#endif

namespace mv::imageproc {
std::size_t split_byte_interleaved(const std::uint8_t* source,std::uint8_t* red,
    std::uint8_t* green,std::uint8_t* blue,std::size_t count,std::size_t components,bool bgr) {
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=runtime::cpu_features();
  if(runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(),cpu.hardware()),
                            {false,true,false,false}))
    return split_byte_interleaved_row_avx2(source,red,green,blue,count,components,bgr);
#else
  (void)source;(void)red;(void)green;(void)blue;(void)count;(void)components;(void)bgr;
#endif
  return 0;
}
std::optional<core::Image> try_transform_byte_planes(const std::array<core::Image,3>& source,
                                                     bool inverse) {
  for(const auto& image:source)
    if(image.empty() || image.channel_count()!=1 || image.pixel_type()!=core::PixelType::UInt8 ||
       image.has_reduced_domain() || image.width()!=source[0].width() ||
       image.height()!=source[0].height())return {};
  const std::array types{core::PixelType::UInt8,core::PixelType::UInt8,core::PixelType::UInt8};
  auto output=core::Image::create_uninitialized(source[0].width(),source[0].height(),types);
  std::array<const std::uint8_t*,3> inputs;
  std::array<std::uint8_t*,3> outputs;
  for(std::size_t ch=0;ch<3;++ch) {
    inputs[ch]=static_cast<const std::uint8_t*>(source[ch].data());
    outputs[ch]=static_cast<std::uint8_t*>(output.mutable_data(ch));
  }
#if MVKIT_FILTER_HAS_AVX2
  const auto& cpu=runtime::cpu_features();
  const bool avx2=runtime::kernel_allowed(runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif
  for(std::size_t r=0;r<output.height();++r) {
    std::array<const std::uint8_t*,3> in;
    std::array<std::uint8_t*,3> out;
    for(std::size_t ch=0;ch<3;++ch) {
      in[ch]=inputs[ch]+r*source[ch].stride();out[ch]=outputs[ch]+r*output.stride(ch);
    }
    std::size_t c=0;
#if MVKIT_FILTER_HAS_AVX2
    if(avx2)c=transform_byte_planes_row_avx2(in[0],in[1],in[2],out[0],out[1],out[2],0,output.width(),inverse);
#endif
    for(;c<output.width();++c) {
      if(inverse) {
        // 保留 convert(real)、scale(real) 的中间 Float32 舍入，不能直接用 Double 归一化。
        double h=static_cast<float>(double(in[0][c])*(1./255.)+0.);
        const double s=static_cast<float>(double(in[1][c])*(1./255.)+0.);
        const double v=static_cast<float>(double(in[2][c])*(1./255.)+0.);
        h=(h-std::floor(h))*6;
        const auto sector=static_cast<int>(std::floor(h));
        const auto f=h-sector,p=v*(1-s),q=v*(1-s*f),t=v*(1-s*(1-f));
        double rgb[3];
        switch(sector%6) {
          case 0:rgb[0]=v;rgb[1]=t;rgb[2]=p;break;
          case 1:rgb[0]=q;rgb[1]=v;rgb[2]=p;break;
          case 2:rgb[0]=p;rgb[1]=v;rgb[2]=t;break;
          case 3:rgb[0]=p;rgb[1]=q;rgb[2]=v;break;
          case 4:rgb[0]=t;rgb[1]=p;rgb[2]=v;break;
          default:rgb[0]=v;rgb[1]=p;rgb[2]=q;break;
        }
        for(std::size_t ch=0;ch<3;++ch)out[ch][c]=saturate_cast<std::uint8_t>(rgb[ch]*255.);
      } else {
        const double red=double(in[0][c])/255.,green=double(in[1][c])/255.,blue=double(in[2][c])/255.;
        const auto maximum=std::max({red,green,blue}),minimum=std::min({red,green,blue}),delta=maximum-minimum;
        double hue=0;
        if(delta!=0) {
          hue=maximum==red?(green-blue)/delta:maximum==green?2+(blue-red)/delta:4+(red-green)/delta;
          hue/=6;if(hue<0)hue+=1;
        }
        const float hsv[3]{static_cast<float>(hue),static_cast<float>(maximum==0?0.:delta/maximum),static_cast<float>(maximum)};
        for(std::size_t ch=0;ch<3;++ch) {
          const auto scaled=static_cast<float>(double(hsv[ch])*255.+0.);
          out[ch][c]=saturate_cast<std::uint8_t>(scaled);
        }
      }
    }
  }
  return output;
}
}
