#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>
#include <core/image.hpp>
#include <core/region.hpp>

namespace mv::imageproc::detail {

// 独立整数计数器避免重复灰度的串行依赖；各调用/分区 worker 独立拥有。
// Shift=8 将 UInt16 映射到 auto_threshold 的同一 256 桶。
template<class T,unsigned Shift=0>
std::array<std::uint64_t,256> histogram256(const core::Image& image,
                                         std::span<const core::RegionRun> runs) {
  alignas(64) std::uint64_t counts[4][256]{};
  const auto* base=static_cast<const std::byte*>(image.data());
  for(const auto& run:runs) {
    const auto* row=base+static_cast<std::size_t>(run.row)*image.stride();
    auto c=std::int64_t(run.col_begin);
    const auto value=[&](std::int64_t column) {
      T pixel;
      std::memcpy(&pixel,row+static_cast<std::size_t>(column)*sizeof(T),sizeof(T));
      return static_cast<unsigned>(pixel)>>Shift;
    };
    for(;std::int64_t(run.col_end)-c>=3;c+=4) {
      if constexpr (std::endian::native == std::endian::little &&
                    (sizeof(T) == 1 || sizeof(T) == 2)) {
        // 一次非对齐 load 读取四个完整样本；游程界保证不读源 padding，计数保持精确 UInt64。
        using Packed = std::conditional_t<sizeof(T)==1,std::uint32_t,std::uint64_t>;
        Packed packed;
        static_assert(sizeof(Packed)==4*sizeof(T));
        std::memcpy(&packed, row + static_cast<std::size_t>(c)*sizeof(T), sizeof(Packed));
        constexpr unsigned bits = sizeof(T)*8;
        constexpr std::uint64_t mask = (std::uint64_t{1} << bits)-1;
        ++counts[0][static_cast<unsigned>(packed & mask) >> Shift];
        ++counts[1][static_cast<unsigned>((packed >> bits) & mask) >> Shift];
        ++counts[2][static_cast<unsigned>((packed >> (2*bits)) & mask) >> Shift];
        ++counts[3][static_cast<unsigned>((packed >> (3*bits)) & mask) >> Shift];
      } else {
        ++counts[0][value(c)];++counts[1][value(c+1)];
        ++counts[2][value(c+2)];++counts[3][value(c+3)];
      }
    }
    for(;c<=run.col_end;++c)++counts[0][value(c)];
  }
  std::array<std::uint64_t,256> result;
  for(std::size_t i=0;i<256;++i)
    result[i]=counts[0][i]+counts[1][i]+counts[2][i]+counts[3][i];
  return result;
}

} // namespace mv::imageproc::detail
