#include <imageproc/operators.hpp>
#include <imageproc/advanced.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory_resource>
#include <vector>

#include <core/exception.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/memory/scratch_arena.hpp>

namespace mv::imageproc {
namespace {

mv::core::Region shifted_crop_domain(const mv::core::Image& image,std::int64_t row1,std::int64_t col1,std::int64_t row2,std::int64_t col2) {
  const auto max_row=std::min<std::int64_t>(row2,static_cast<std::int64_t>(image.height()-1));
  const auto max_col=std::min<std::int64_t>(col2,static_cast<std::int64_t>(image.width()-1));
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<mv::core::RegionRun> runs{scratch.resource()};
  const auto domain=image.domain();
  for(const auto& run:domain.runs()) {
    if(run.row<row1 || run.row>max_row) continue;
    const auto begin=std::max<std::int64_t>(run.col_begin,col1);
    const auto end=std::min<std::int64_t>(run.col_end,max_col);
    if(begin<=end) runs.push_back({std::int64_t(run.row)-row1,begin-col1,end-col1});
  }
  return mv::core::Region::from_runs(runs);
}

mv::core::Region mirror_domain(const mv::core::Image& image,MirrorMode mode) {
  mv::runtime::ScratchArena scratch;
  std::pmr::vector<mv::core::RegionRun> runs{scratch.resource()};
  const auto domain=image.domain();
  if(mode==MirrorMode::Row) {
    runs.reserve(domain.runs().size());
    for(const auto& run:domain.runs()) runs.push_back({static_cast<std::int64_t>(image.height()-1)-run.row,run.col_begin,run.col_end});
  } else if(mode==MirrorMode::Column) {
    runs.reserve(domain.runs().size());
    const auto last=static_cast<std::int64_t>(image.width()-1);
    for(const auto& run:domain.runs()) runs.push_back({run.row,last-run.col_end,last-run.col_begin});
  } else {
    for(const auto& run:domain.runs()) {
      for(std::int64_t c=run.col_begin;;++c) {
        runs.push_back({c,run.row,run.row});
        if(c==run.col_end) break;
      }
    }
  }
  return mv::core::Region::from_runs(runs);
}

std::vector<mv::core::PixelType> channel_types(const mv::core::Image& image) {
  std::vector<mv::core::PixelType> result;
  result.reserve(image.channel_count());
  for(std::size_t channel=0;channel<image.channel_count();++channel) result.push_back(image.pixel_type(channel));
  return result;
}

template <typename T>
void zoom_channel(const mv::core::Image& image,mv::core::Image& output,std::size_t channel,std::size_t out_w,std::size_t out_h,Interpolation interpolation) {
  const ImageView src{image,channel};
  MutableImageView dst{output,channel};
  const double sx=static_cast<double>(image.width())/static_cast<double>(out_w);
  const double sy=static_cast<double>(image.height())/static_cast<double>(out_h);
  for(std::size_t y=0;y<out_h;++y) {
    auto* out=dst.row_as<T>(y);
    double fy=(static_cast<double>(y)+0.5)*sy-0.5;
    if(interpolation==Interpolation::NearestNeighbor) {
      auto iy=static_cast<std::int64_t>(std::llround(fy));
      iy=std::clamp<std::int64_t>(iy,0,static_cast<std::int64_t>(image.height()-1));
      const auto* in=src.row_as<T>(static_cast<std::size_t>(iy));
      for(std::size_t x=0;x<out_w;++x) {
        double fx=(static_cast<double>(x)+0.5)*sx-0.5;
        auto ix=static_cast<std::int64_t>(std::llround(fx));
        ix=std::clamp<std::int64_t>(ix,0,static_cast<std::int64_t>(image.width()-1));
        out[x]=in[static_cast<std::size_t>(ix)];
      }
    } else {
      fy=std::clamp(fy,0.0,static_cast<double>(image.height()-1));
      const auto y0=static_cast<std::size_t>(std::floor(fy));
      const auto y1=std::min(y0+1,image.height()-1);
      const double wy=fy-static_cast<double>(y0);
      const auto* r0=src.row_as<T>(y0);const auto* r1=src.row_as<T>(y1);
      for(std::size_t x=0;x<out_w;++x) {
        double fx=(static_cast<double>(x)+0.5)*sx-0.5;
        fx=std::clamp(fx,0.0,static_cast<double>(image.width()-1));
        const auto x0=static_cast<std::size_t>(std::floor(fx));const auto x1=std::min(x0+1,image.width()-1);
        const double wx=fx-static_cast<double>(x0);
        const double top=(1.0-wx)*static_cast<double>(r0[x0])+wx*static_cast<double>(r0[x1]);
        const double bottom=(1.0-wx)*static_cast<double>(r1[x0])+wx*static_cast<double>(r1[x1]);
        out[x]=saturate_cast<T>((1.0-wy)*top+wy*bottom);
      }
    }
  }
}

} // namespace

mv::core::Image crop_rectangle1(const mv::core::Image& image,std::int64_t row1,std::int64_t col1,std::int64_t row2,std::int64_t col2) {
  if(image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument,"crop_rectangle1","image must not be empty");
  if(row1<0||col1<0||row2<row1||col2<col1||row1>=static_cast<std::int64_t>(image.height())||col1>=static_cast<std::int64_t>(image.width()))
    throw mv::core::Exception(mv::core::Status::InvalidArgument,"crop_rectangle1","invalid rectangle");
  const auto out_h_u=static_cast<std::uint64_t>(row2-row1)+1u;const auto out_w_u=static_cast<std::uint64_t>(col2-col1)+1u;
  if(out_h_u>std::numeric_limits<std::size_t>::max()||out_w_u>std::numeric_limits<std::size_t>::max()) throw mv::core::Exception(mv::core::Status::OutOfRange,"crop_rectangle1","output dimensions overflow");
  const auto out_h=static_cast<std::size_t>(out_h_u),out_w=static_cast<std::size_t>(out_w_u);
  auto output=mv::core::Image::create(out_w,out_h,channel_types(image));
  const auto copy_w=std::min<std::size_t>(out_w,image.width()-static_cast<std::size_t>(col1));
  const auto copy_h=std::min<std::size_t>(out_h,image.height()-static_cast<std::size_t>(row1));
  for(std::size_t channel=0;channel<image.channel_count();++channel) {
    const auto bytes=mv::core::bytes_per_sample(image.pixel_type(channel));
    const auto* src=static_cast<const std::byte*>(image.data(channel));
    auto* dst=static_cast<std::byte*>(output.mutable_data(channel));
    for(std::size_t r=0;r<copy_h;++r) std::memcpy(dst+r*output.stride(channel),src+(static_cast<std::size_t>(row1)+r)*image.stride(channel)+static_cast<std::size_t>(col1)*bytes,copy_w*bytes);
  }
  const auto domain=shifted_crop_domain(image,row1,col1,row2,col2);
  if(domain.area()!=static_cast<std::uint64_t>(out_w)*static_cast<std::uint64_t>(out_h)) output=output.reduce_domain(domain);
  return output;
}

mv::core::Image mirror_image(const mv::core::Image &image, MirrorMode mode) {
  if (image.empty())
    throw mv::core::Exception(mv::core::Status::InvalidArgument, "mirror_image",
                              "image must not be empty");
  const bool transpose = mode == MirrorMode::Diagonal;
  auto output = mv::core::Image::create(
      transpose ? image.height() : image.width(),
      transpose ? image.width() : image.height(), channel_types(image));
  const auto reflected_domain = image.has_reduced_domain()
                                    ? mirror_domain(image, mode)
                                    : mv::core::Region{};
  const auto width = image.width(), height = image.height();
  for (std::size_t channel = 0; channel < image.channel_count(); ++channel) {
    const auto bytes = mv::core::bytes_per_sample(image.pixel_type(channel));
    const auto *src = static_cast<const std::byte *>(image.data(channel));
    auto *dst = static_cast<std::byte *>(output.mutable_data(channel));
    const auto stride = image.stride(channel),
               out_stride = output.stride(channel);
    if (mode == MirrorMode::Row) {
      // 行镜像按整行或有效行程复制，避免逐像素解析地址。
      if (image.has_reduced_domain())
        for (const auto &run : reflected_domain.runs())
          std::memcpy(dst + std::size_t(run.row) * out_stride +
                          std::size_t(run.col_begin) * bytes,
                      src + (height - 1 - std::size_t(run.row)) * stride +
                          std::size_t(run.col_begin) * bytes,
                      (std::int64_t(run.col_end) - run.col_begin + 1) * bytes);
      else
        for (std::size_t r = 0; r < height; ++r)
          std::memcpy(dst + r * out_stride, src + (height - 1 - r) * stride,
                      width * bytes);
    } else
      dispatch_scalar(image.pixel_type(channel), [&](auto tag) {
        using T = typename decltype(tag)::type;
        if (mode == MirrorMode::Column) {
          const auto copy = [&](std::size_t r, std::size_t begin,
                                std::size_t end) {
            const auto *in = src + r * stride;
            auto *row = reinterpret_cast<T *>(dst + r * out_stride);
            for (auto c = begin; c < end; ++c)
              row[c] = load_pixel<T>(in + (width - 1 - c) * sizeof(T));
          };
          if (image.has_reduced_domain())
            for (const auto &run : reflected_domain.runs())
              copy(run.row, run.col_begin, std::size_t(run.col_end) + 1);
          else
            for (std::size_t r = 0; r < height; ++r)
              copy(r, 0, width);
        } else {
          // 对角镜像分块转置，保持已有定义域外像素语义。
          constexpr std::size_t tile = 32;
          for (std::size_t r0 = 0; r0 < width; r0 += tile)
            for (std::size_t c0 = 0; c0 < height; c0 += tile)
              for (auto r = r0; r < std::min(r0 + tile, width); ++r) {
                auto *row = reinterpret_cast<T *>(dst + r * out_stride);
                for (auto c = c0; c < std::min(c0 + tile, height); ++c)
                  row[c] = load_pixel<T>(src + c * stride + r * sizeof(T));
              }
        }
      });
  }
  if (image.has_reduced_domain())
    output = output.reduce_domain(reflected_domain);
  return output;
}

mv::core::Image zoom_image_size(const mv::core::Image& image,std::size_t width,std::size_t height,Interpolation interpolation) {
  return advanced::zoom_extended(image,width,height,interpolation);
}

} // namespace mv::imageproc
