#pragma once
#include <imageproc/operators.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/math.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/memory/overwrite_buffer.hpp>
#include <core/exception.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory_resource>
#include <vector>

namespace mv::imageproc::advanced {
using namespace mv::core;
inline void check(bool ok, const char* op, const char* parameter,
                  const char* message) {
  if (!ok)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op,
                    parameter, -1, message);
}
inline void image(const Image& a, const char* op) {
  check(!a.empty(), op, "Image", "image must not be empty");
}
inline void dimensions(std::size_t w, std::size_t h, const char* op) {
  check(w > 0 && h > 0 && w <= static_cast<std::size_t>(INT32_MAX) &&
            h <= static_cast<std::size_t>(INT32_MAX) &&
            w <= std::numeric_limits<std::size_t>::max() / h,
        op, "Size", "invalid output dimensions");
}
inline void window(std::size_t w, std::size_t h, const char* op) {
  check(w > 0 && h > 0 && w <= 4095 && h <= 4095 && (w & 1) && (h & 1), op,
        "MaskSize", "odd window sizes in [1,4095] required");
}
inline void sigma(double s, const char* op) {
  check(std::isfinite(s) && s >= 0.1 && s <= 32, op, "Sigma",
        "sigma must be in [0.1,32]");
}
inline void interpolation(Interpolation mode, const char* op,
                          bool area = false) {
  check(mode == Interpolation::NearestNeighbor ||
            mode == Interpolation::Constant ||
            mode == Interpolation::Bilinear || mode == Interpolation::Bicubic ||
            (area && mode == Interpolation::Weighted),
        op, "Interpolation",
        "use nearest, constant, bilinear or bicubic; weighted is available "
        "only for zoom");
}
inline double read(const Image& a, std::size_t ch, std::size_t r,
                   std::size_t c) {
  const auto* p = static_cast<const std::byte*>(a.data(ch)) + r * a.stride(ch) +
                  c * bytes_per_sample(a.pixel_type(ch));
  return dispatch_scalar(a.pixel_type(ch), [&](auto tag) {
    typename decltype(tag)::type v;
    std::memcpy(&v, p, sizeof(v));
    return double(v);
  });
}
inline double finite_read(const Image& a, std::size_t ch, std::size_t r,
                          std::size_t c) {
  const auto v = read(a, ch, r, c);
  check(std::isfinite(v), "image processing", "Image",
        "nonfinite pixel is not supported");
  return v;
}
inline Image create_like(const Image& a, std::size_t w, std::size_t h,
                         bool real = false, bool overwrite = false) {
  ExecutionContext context;
  std::pmr::vector<PixelType> types{context.scratch_resource()};
  for (std::size_t ch = 0; ch < a.channel_count(); ++ch)
    types.push_back(real ? PixelType::Float32 : a.pixel_type(ch));
  if(!overwrite)return Image::create(w, h, types);
  // 调用方保证完整覆盖输出；保持原构造失败的错误来源。
  try {return Image::create_uninitialized(w,h,types);}
  catch(const Exception& error) {
    if(error.where()=="Image::create_uninitialized")
      throw Exception(error.status(),error.error_code(),"Image::create",error.parameter(),error.parameter_index(),error.message());
    throw;
  }
}
template <class Fn>
void pixels(Image& out, Fn&& fn) {
  const auto width = out.width(), height = out.height();
  for (std::size_t ch = 0; ch < out.channel_count(); ++ch) {
    auto* data = static_cast<std::byte*>(out.mutable_data(ch));
    const auto stride = out.stride(ch);
    dispatch_scalar(out.pixel_type(ch), [&](auto tag) {
      using T = typename decltype(tag)::type;
      ExecutionContext::for_each_partition(
          height,
          ExecutionContext::saturated_work(width, height),
          ExecutionContext::kArithmeticParallelWork, [&](auto range) {
            for (auto r = range.begin; r < range.end; ++r) {
              auto* row = reinterpret_cast<T*>(data + r * stride);
              for (std::size_t c = 0; c < width; ++c)
                row[c] = saturate_cast<T>(fn(ch, r, c));
            }
          });
    });
  }
}
inline Image keep_domain(Image out, const Region& domain) {
  const auto own = out.domain();
  const auto outside = own.difference(domain);
  for (std::size_t ch = 0; ch < out.channel_count(); ++ch) {
    auto* data = static_cast<std::byte*>(out.mutable_data(ch));
    const auto bytes = bytes_per_sample(out.pixel_type(ch));
    for (const auto& run : outside.runs())
      std::memset(data + std::int64_t(run.row) * out.stride(ch) + std::int64_t(run.col_begin) * bytes, 0,
                  (std::int64_t(run.col_end) - std::int64_t(run.col_begin) + 1) * bytes);
  }
  return out.reduce_domain(domain);
}
template <class Fn>
Region mask_region(std::size_t w, std::size_t h, Fn&& fn) {
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  for (std::size_t r = 0; r < h; ++r) {
    std::size_t c = 0;
    while (c < w) {
      while (c < w && !fn(r, c)) ++c;
      const auto begin = c;
      while (c < w && fn(r, c)) ++c;
      if (begin < c)
        runs.push_back({static_cast<std::int64_t>(r),
                        static_cast<std::int64_t>(begin),
                        static_cast<std::int64_t>(c - 1)});
    }
  }
  return Region::from_runs(runs);
}
std::array<double, 9> inverse(const Tuple& matrix, bool affine, const char* op);
template <class Fn>
Region mask_region(const Region& domain, Fn&& fn) {
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  for (const auto& span : domain.runs()) {
    bool active = false;
    auto begin = std::int64_t(span.col_begin);
    for (auto c = std::int64_t(span.col_begin); c <= span.col_end; ++c) {
      const bool selected = fn(static_cast<std::size_t>(span.row), static_cast<std::size_t>(c));
      if (selected && !active) { begin = c; active = true; }
      if (!selected && active) { runs.push_back({span.row, begin, c - 1}); active = false; }
    }
    if (active) runs.push_back({span.row, begin, span.col_end});
  }
  return Region::from_runs(runs);
}
Image zoom_extended(const Image&, std::size_t, std::size_t, Interpolation);
Image sobel_extended(const Image&, SobelFilter, std::size_t);
// 私有 Canny 生产路径在缩放前保留公开 Float32 舍入。
bool gaussian_gradient_pair(const Image&,double,double,std::pmr::vector<double>&,std::pmr::vector<double>&);
bool gaussian_gradient_pair(const Image&,double,double,mv::runtime::OverwriteBuffer<double>&,mv::runtime::OverwriteBuffer<double>&);
Image gaussian_extended(const Image&, std::size_t);
}  // namespace mv::imageproc::advanced
