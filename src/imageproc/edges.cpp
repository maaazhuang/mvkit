#include <imageproc/operators.hpp>
#include <imageproc/advanced.hpp>
#include <imageproc/filters_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>

#include <cmath>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <core/exception.hpp>
#include <imageproc/border.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <imageproc/pixel_traits.hpp>

namespace mv::imageproc {
namespace {

template <typename T>
std::pair<double, double> sobel_xy(const ImageView& src, std::size_t row, std::size_t col) {
  static constexpr int ky[3][3] = {{1,2,1},{0,0,0},{-1,-2,-1}};
  static constexpr int kx[3][3] = {{1,0,-1},{2,0,-2},{1,0,-1}};
  double a = 0.0;
  double b = 0.0;
  for (int dy=-1;dy<=1;++dy) {
    const auto y=border_index(static_cast<std::int64_t>(row)+dy,src.height(),BorderMode::Mirrored).value();
    const auto* in=src.row_as<T>(y);
    for (int dx=-1;dx<=1;++dx) {
      const auto x=border_index(static_cast<std::int64_t>(col)+dx,src.width(),BorderMode::Mirrored).value();
      T sample;std::memcpy(&sample,reinterpret_cast<const std::byte*>(in)+x*sizeof(T),sizeof(T));
      const double v=static_cast<double>(sample);
      a += v * static_cast<double>(ky[dy+1][dx+1]);
      b += v * static_cast<double>(kx[dy+1][dx+1]);
    }
  }
  return {b / 4.0, a / 4.0};
}

template <typename In, typename Out>
mv::core::Image sobel_signed(const mv::core::Image& image, SobelFilter filter, mv::core::PixelType out_format) {
  const ImageView src{image};
  auto output=make_output_like(image,out_format);
  MutableImageView dst{output};
  const auto domain=image.domain();
  for (const auto& run:domain.runs()) {
    auto* out=dst.row_as<Out>(static_cast<std::size_t>(run.row));
    for (std::int64_t c_i=run.col_begin;;++c_i) {
      const auto [x,y]=sobel_xy<In>(src,static_cast<std::size_t>(run.row),static_cast<std::size_t>(c_i));
      out[static_cast<std::size_t>(c_i)]=saturate_cast<Out>(filter==SobelFilter::X?x:y);
      if(c_i==run.col_end) break;
    }
  }
  return output;
}

template <typename In>
mv::core::Image sobel_amplitude(const mv::core::Image& image,SobelFilter filter) {
  const ImageView src{image};
  constexpr bool byte_input = std::is_same_v<In,std::uint8_t>;
  auto output=make_output_like(image,byte_input ? mv::core::PixelType::UInt8
                                               : mv::core::PixelType::Float32);
  MutableImageView dst{output};
  const auto domain=image.domain();
  for(const auto& run:domain.runs()) {
    using Out = std::conditional_t<byte_input,std::uint8_t,float>;
    auto* out=dst.row_as<Out>(static_cast<std::size_t>(run.row));
    for(std::int64_t c_i=run.col_begin;;++c_i) {
      const auto [x,y]=sobel_xy<In>(src,static_cast<std::size_t>(run.row),static_cast<std::size_t>(c_i));
      const double value=filter==SobelFilter::SumAbs ? std::abs(x)+std::abs(y) : std::sqrt(x*x+y*y);
      if constexpr(byte_input) {
        // HALCON byte 3x3 sum_abs 截断缩放后的掩码和。
        out[static_cast<std::size_t>(c_i)]=saturate_cast<Out>(filter==SobelFilter::SumAbs?std::floor(value):value);
      } else out[static_cast<std::size_t>(c_i)]=saturate_cast<Out>(value);
      if(c_i==run.col_end) break;
    }
  }
  return output;
}

template <typename In>
mv::core::Image sobel_impl(const mv::core::Image& image,SobelFilter filter) {
  if(filter==SobelFilter::SumAbs || filter==SobelFilter::SumSqrt) return sobel_amplitude<In>(image,filter);
  if constexpr (std::is_same_v<In,std::uint8_t>) {
    return sobel_signed<In,std::int16_t>(image,filter,mv::core::PixelType::Int16);
  } else if constexpr (std::is_same_v<In,float>) {
    return sobel_signed<In,float>(image,filter,mv::core::PixelType::Float32);
  } else {
    return sobel_signed<In,std::int32_t>(image,filter,mv::core::PixelType::Int32);
  }
}

} // namespace

mv::core::Image sobel_amp(const mv::core::Image& image,SobelFilter filter,std::size_t size) {
  if(image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument,"sobel_amp","image must not be empty");
  if(size==3 && filter==SobelFilter::SumAbs) {
    bool bytes=true;
    for(std::size_t ch=0;ch<image.channel_count();++ch)
      bytes=bytes && image.pixel_type(ch)==mv::core::PixelType::UInt8;
    if(bytes) {
      auto out=image.has_reduced_domain()?make_output_like(image):make_full_output_for_overwrite(image);
      const auto domain=image.domain();const auto width=image.width(),height=image.height();
#if MVKIT_FILTER_HAS_AVX2
      const auto& cpu=mv::runtime::cpu_features();
      const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware()),kSobel3SumAbsU8Avx2Requirement);
#endif
      for(std::size_t ch=0;ch<image.channel_count();++ch) {
        const auto* source=static_cast<const std::uint8_t*>(image.data(ch));
        const auto stride=image.stride(ch);
        auto* target=static_cast<std::uint8_t*>(out.mutable_data(ch));
        const auto target_stride=out.stride(ch);
        for(const auto& run:domain.runs()) {
          const auto r=static_cast<std::size_t>(run.row);
          const auto* top=source+border_index(static_cast<std::int64_t>(r)-1,height,BorderMode::Mirrored).value()*stride;
          const auto* middle=source+r*stride;
          const auto* bottom=source+border_index(static_cast<std::int64_t>(r)+1,height,BorderMode::Mirrored).value()*stride;
          auto* row=target+r*target_stride;
          auto c=static_cast<std::size_t>(run.col_begin);
          const auto end=static_cast<std::size_t>(std::int64_t(run.col_end)+1);
          if(c==0) {
            // 镜像对称使边缘水平导数抵消。
            const auto neighbor=width==1?0:1;
            row[c++]=static_cast<std::uint8_t>(std::abs(int(top[0])+top[neighbor]-bottom[0]-bottom[neighbor])/2);
          }
          const auto interior_end=std::min(end,width-1);
#if MVKIT_FILTER_HAS_AVX2
          if(c<interior_end && avx2)
            c+=sobel3_sumabs_u8_row_avx2(top,middle,bottom,row,c,interior_end);
#endif
          for(;c<interior_end;++c) {
            const int gx=int(top[c-1])+2*middle[c-1]+bottom[c-1]-top[c+1]-2*middle[c+1]-bottom[c+1];
            const int gy=int(top[c-1])+2*top[c]+top[c+1]-bottom[c-1]-2*bottom[c]-bottom[c+1];
            row[c]=static_cast<std::uint8_t>(std::min(255,(std::abs(gx)+std::abs(gy))/4));
          }
          if(c<end) {
            const auto neighbor=width==1?0:width-2;
            row[c]=static_cast<std::uint8_t>(std::abs(int(top[c])+top[neighbor]-bottom[c]-bottom[neighbor])/2);
          }
        }
      }
      return out;
    }
  }
  if(size!=3 || image.channel_count()!=1U)return advanced::sobel_extended(image,filter,size);
  if(size!=3) throw mv::core::Exception(mv::core::Status::Unsupported,"sobel_amp","v1 supports size 3 only");
  return dispatch_scalar(image.pixel_type(),[&](auto tag){using T=typename decltype(tag)::type;return sobel_impl<T>(image,filter);});
}

} // namespace mv::imageproc
