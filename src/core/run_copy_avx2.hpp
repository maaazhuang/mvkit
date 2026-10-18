#pragma once
#include <cstddef>
#include <cstdint>
namespace mv::core {
void stream_run_bytes_avx2(std::byte* destination,const std::byte* source,std::size_t bytes) noexcept;
}
