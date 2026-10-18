#include "runtime/memory/system_resource.hpp"

namespace mv::runtime {

std::pmr::memory_resource* system_memory_resource() noexcept {
  return std::pmr::new_delete_resource();
}

}  // namespace mv::runtime
