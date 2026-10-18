#pragma once

#include <cstddef>
#include <memory_resource>

namespace mv::runtime {

class ScratchArena {
 public:
  ScratchArena();
  explicit ScratchArena(std::pmr::memory_resource* upstream);

  ScratchArena(const ScratchArena&) = delete;
  ScratchArena& operator=(const ScratchArena&) = delete;

  [[nodiscard]] std::pmr::memory_resource* resource() noexcept;
  void reset() noexcept;

 private:
  static std::pmr::memory_resource* require_upstream(
      std::pmr::memory_resource* upstream);

  alignas(64) std::byte inline_buffer_[4096];
  std::pmr::monotonic_buffer_resource resource_;
};

}  // namespace mv::runtime
