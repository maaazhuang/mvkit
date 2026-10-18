#include "runtime/memory/scratch_arena.hpp"

#include <stdexcept>

#include "runtime/memory/memory_resource.hpp"

namespace mv::runtime {

std::pmr::memory_resource* ScratchArena::require_upstream(
    std::pmr::memory_resource* upstream) {
  if (upstream == nullptr)
    throw std::invalid_argument("ScratchArena upstream must not be null");
  return upstream;
}

ScratchArena::ScratchArena()
    : resource_(inline_buffer_, sizeof(inline_buffer_),
                scratch_resource()) {}

ScratchArena::ScratchArena(std::pmr::memory_resource* upstream)
    : resource_(inline_buffer_, sizeof(inline_buffer_),
                require_upstream(upstream)) {}

std::pmr::memory_resource* ScratchArena::resource() noexcept {
  return &resource_;
}

void ScratchArena::reset() noexcept { resource_.release(); }

}  // namespace mv::runtime
