#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>

namespace mv::c_api {
struct IconicFeaturePublication {
  enum Feature : std::uint32_t { Centroid=1, Rectangle2=2, Circle=4, Circularity=8, Eccentricity=16, Orientation=32 };
  std::atomic<std::uint32_t> ready_features{0};
  template<class Compute> void ensure_feature(std::uint32_t bit, std::once_flag& once, Compute&& compute) {
    // 首次初始化仍由 call_once 串行化；仅成功后发布，失败允许重试。
    if (ready_features.load(std::memory_order_acquire) & bit) return;
    std::call_once(once, [&] {
      compute();
      ready_features.fetch_or(bit, std::memory_order_release);
    });
  }
};
}
