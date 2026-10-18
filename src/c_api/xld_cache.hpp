#pragma once
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include <c_api/storage_access.hpp>
#include <utility>
namespace mv::c_api {
inline ::MvXldContourApproximation to_c(mv::core::ContourApproximation v) noexcept {
  switch (v) {
    case mv::core::ContourApproximation::Line: return MV_XLD_CONTOUR_APPROX_LINE;
    case mv::core::ContourApproximation::Ellipse: return MV_XLD_CONTOUR_APPROX_ELLIPSE;
    case mv::core::ContourApproximation::Circle: return MV_XLD_CONTOUR_APPROX_CIRCLE;
    case mv::core::ContourApproximation::Unknown: default: return MV_XLD_CONTOUR_APPROX_UNKNOWN;
  }
}
inline void invalidate_xld_cache(MvObjectImpl& h) {
  if(!h.export_cache.load(std::memory_order_acquire))return;
  std::lock_guard lock(h.ExportCache().point_cache_mutex);
  h.ExportCache().point_cache_ready.store(false,std::memory_order_release);
  h.ExportCache().point_cache.clear();
}
inline void ensure_xld_cache(MvObjectImpl& h, std::size_t index) {
  if(!h.ExportCache().point_cache_ready.load(std::memory_order_acquire)) {
    std::lock_guard lock(h.ExportCache().point_cache_mutex);
    if(!h.ExportCache().point_cache_ready.load(std::memory_order_relaxed)) {
      const auto contours=mv::core::XldContours{h.value};
      // 固定描述符使独立发布的点缓冲稳定；append 按既有修改契约使当前代失效。
      h.ExportCache().point_cache=std::vector<MVXldPointExport>(contours.contour_count());
      h.ExportCache().point_cache_ready.store(true,std::memory_order_release);
    }
  }
  auto& exported=h.ExportCache().point_cache[index];
  if(exported.ready.load(std::memory_order_acquire))return;
  std::lock_guard lock(h.ExportCache().point_cache_mutex);
  if(exported.ready.load(std::memory_order_relaxed))return;
  const auto& contour=h.value.at(index).as_xld_contour();
  std::vector<::MvPoint2d> points(contour.size());
  const auto source=contour.points();
  for(std::size_t i=0;i<source.size();++i)points[i]={source[i].row,source[i].col};
  exported.points=std::move(points);
  exported.ready.store(true,std::memory_order_release);
}
inline ::MvObject make_xld_contours(mv::core::XldContours value) {
  auto out=std::make_unique<MvObjectImpl>();out->value=value.iconic_tuple();return out.release();
}
inline ::MvObject make_xld_contours(mv::core::IconicTuple value) {
  value.require_kind(mv::core::IconicKind::XldContour,"make_xld_contours");
  auto out=std::make_unique<MvObjectImpl>();out->value=std::move(value);return out.release();
}
}
