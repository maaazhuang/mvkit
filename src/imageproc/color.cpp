#include <imageproc/operators.hpp>

#include <cstddef>
#include <cstdint>

#include <core/exception.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_traits.hpp>
#include <imageproc/execution_context.hpp>
#if MVKIT_FILTER_HAS_AVX2
#include "arithmetic_avx2.hpp"
#endif

namespace mv::imageproc {

mv::core::Image rgb1_to_gray(const mv::core::Image& image) {
  if(image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument,"rgb1_to_gray","image must not be empty");
  if(image.channel_count()==1U) return image;
  if(image.channel_count()!=3U || image.pixel_type(0)!=mv::core::PixelType::UInt8 ||
     image.pixel_type(1)!=mv::core::PixelType::UInt8 ||
     image.pixel_type(2)!=mv::core::PixelType::UInt8)
    throw mv::core::Exception(mv::core::Status::Unsupported,"rgb1_to_gray","operator requires three UInt8 channels in RGB semantic order");
  const ImageView red{image,0};
  const ImageView green{image,1};
  const ImageView blue{image,2};
  const mv::core::PixelType gray_type[]={mv::core::PixelType::UInt8};
  auto output=image.has_reduced_domain() ? make_output_like(image,mv::core::PixelType::UInt8)
      : mv::core::Image::create_uninitialized(image.width(),image.height(),gray_type);
#if MVKIT_FILTER_HAS_AVX2
  ExecutionContext context;
  const auto& cpu=context.runtime().cpu();
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif
  MutableImageView dst{output};
  const auto domain=image.domain();
  for(const auto& run:domain.runs()) {
    const auto row=static_cast<std::size_t>(run.row);
    const auto* r=red.row_as<std::uint8_t>(row);
    const auto* g=green.row_as<std::uint8_t>(row);
    const auto* b=blue.row_as<std::uint8_t>(row);
    auto* out=dst.row_as<std::uint8_t>(row);
    auto c=static_cast<std::size_t>(run.col_begin);
    const auto end=static_cast<std::size_t>(run.col_end)+1;
#if MVKIT_FILTER_HAS_AVX2
    if(avx2)c=rgb_u8_row_avx2(r,g,b,out,c,end);
#endif
    for(;c<end;++c) {
      const double value=0.299*r[c]+0.587*g[c]+0.114*b[c];
      const int integral=static_cast<int>(value);
      out[c]=static_cast<std::uint8_t>(integral+(value-integral>=.5));
    }
  }
  return output;
}

} // namespace mv::imageproc
