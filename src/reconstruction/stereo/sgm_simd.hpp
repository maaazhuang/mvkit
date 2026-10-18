#pragma once
#include <cstddef>
#include <cstdint>
namespace mv::reconstruction {
using SgmPathKernel = std::int32_t (*)(const std::int32_t*, const std::int32_t*, std::int32_t*,
                                       std::size_t, std::int32_t, std::int32_t, bool, std::int32_t,
                                       std::int32_t*);
std::int32_t sgm_path_sse2(const std::int32_t*, const std::int32_t*, std::int32_t*, std::size_t,
                           std::int32_t, std::int32_t, bool, std::int32_t, std::int32_t*);
std::int32_t sgm_path_avx2(const std::int32_t*, const std::int32_t*, std::int32_t*, std::size_t,
                           std::int32_t, std::int32_t, bool, std::int32_t, std::int32_t*);
}  // namespace mv::reconstruction
