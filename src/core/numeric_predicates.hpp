#pragma once
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace mv::core {
// IEEE 指数分类避免 strict-FP 扫描中的外部 CRT 调用，其他表示保留可移植回退。
template<class T>
inline bool is_finite(T value) noexcept {
  if constexpr(std::is_same_v<T,double> && sizeof(T)==sizeof(std::uint64_t) &&
               std::numeric_limits<T>::is_iec559 && std::numeric_limits<T>::digits==53)
    return (std::bit_cast<std::uint64_t>(value)&UINT64_C(0x7ff0000000000000))!=UINT64_C(0x7ff0000000000000);
  else if constexpr(std::is_same_v<T,float> && sizeof(T)==sizeof(std::uint32_t) &&
                    std::numeric_limits<T>::is_iec559 && std::numeric_limits<T>::digits==24)
    return (std::bit_cast<std::uint32_t>(value)&UINT32_C(0x7f800000))!=UINT32_C(0x7f800000);
  else return std::isfinite(value);
}
} // namespace mv::core
