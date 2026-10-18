#include "runtime/memory/memory_resource.hpp"

#include <mvkit/runtime/generated_backend_config.hpp>

#include <cstddef>
#include <exception>
#include <stdexcept>

#include "runtime/memory/system_resource.hpp"
#if MVKIT_MEMORY_BACKEND_BUILTIN
#include "runtime/memory/builtin_pool_resource.hpp"
#endif

namespace mv::runtime {
namespace {
bool valid_alignment(std::size_t alignment) noexcept {
  return alignment != 0 && (alignment & (alignment - 1)) == 0;
}
}  // namespace

std::pmr::memory_resource* persist_resource() noexcept {
#if MVKIT_MEMORY_BACKEND_BUILTIN
  return builtin_persist_pool();
#else
  return system_memory_resource();
#endif
}

std::pmr::memory_resource* scratch_resource() noexcept {
#if MVKIT_MEMORY_BACKEND_BUILTIN
  return builtin_scratch_pool();
#else
  return system_memory_resource();
#endif
}

std::pmr::memory_resource* default_resource() noexcept {
  return system_memory_resource();
}

void* allocate_bytes(std::size_t bytes, std::size_t alignment) {
  if (bytes == 0) return nullptr;
  if (!valid_alignment(alignment))
    throw std::invalid_argument("alignment must be a non-zero power of two");
  return persist_resource()->allocate(bytes, alignment);
}

void deallocate_bytes(void* pointer, std::size_t bytes,
                      std::size_t alignment) noexcept {
  if (pointer == nullptr) return;
  if (!valid_alignment(alignment)) std::terminate();
  persist_resource()->deallocate(pointer, bytes, alignment);
}

}  // namespace mv::runtime
