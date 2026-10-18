#include <core/run_copy_avx2.hpp>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <immintrin.h>
namespace mv::core {
void stream_run_bytes_avx2(std::byte* destination,const std::byte* source,std::size_t bytes) noexcept {
  const auto head=std::min<std::size_t>(bytes,(32-(reinterpret_cast<std::uintptr_t>(destination)&31))&31);
  std::memcpy(destination,source,head);destination+=head;source+=head;bytes-=head;
  for(;bytes>=128;bytes-=128,destination+=128,source+=128) {
    const auto a=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source));
    const auto b=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source+32));
    const auto c=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source+64));
    const auto d=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source+96));
    _mm256_stream_si256(reinterpret_cast<__m256i*>(destination),a);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(destination+32),b);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(destination+64),c);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(destination+96),d);
  }
  for(;bytes>=32;bytes-=32,destination+=32,source+=32)
    _mm256_stream_si256(reinterpret_cast<__m256i*>(destination),_mm256_loadu_si256(reinterpret_cast<const __m256i*>(source)));
  _mm_sfence(); // 发布/读取前完成 non-temporal 写入。
  std::memcpy(destination,source,bytes);
}
}
