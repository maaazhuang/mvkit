#pragma once

#include <cstddef>
#include <memory_resource>

namespace mv::runtime {

// 跨算子结果、模型和句柄大 payload。
[[nodiscard]] std::pmr::memory_resource* persist_resource() noexcept;
// 函数工作区和 ScratchArena 的线程安全 upstream。
[[nodiscard]] std::pmr::memory_resource* scratch_resource() noexcept;
// 明确使用系统 allocator；不是第三个缓存池。
[[nodiscard]] std::pmr::memory_resource* default_resource() noexcept;

[[nodiscard]] void* allocate_bytes(std::size_t bytes, std::size_t alignment);
void deallocate_bytes(void* pointer, std::size_t bytes,
                      std::size_t alignment) noexcept;

}  // namespace mv::runtime
