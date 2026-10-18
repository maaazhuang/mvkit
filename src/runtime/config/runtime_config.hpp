#pragma once

#include <cstddef>
#include <cstdint>
#include "runtime/cpu/cpu_topology.hpp"

namespace mv::runtime {

enum class MemoryBackend { System, Builtin };
enum class ParallelBackend { Serial, Builtin };

struct RuntimeBackendConfig {
  MemoryBackend memory;
  ParallelBackend parallel;
  bool memory_pool_enabled;
  bool parallel_enabled;
};

struct RuntimeObservability {
  CpuTopology topology;
  const char* requested_isa = "auto";
  const char* effective_isa = "scalar";
  const char* isa_reason = "";
  const char* memory_backend = "system";
  const char* parallel_backend = "serial";
  std::size_t persist_live_bytes = 0;
  std::size_t persist_cached_bytes = 0;
  std::size_t persist_cache_limit = 0;
  std::size_t persist_large_cached_bytes = 0;
  std::uint64_t persist_allocation_count = 0;
  std::size_t scratch_live_bytes = 0;
  std::size_t scratch_cached_bytes = 0;
  std::size_t scratch_cache_limit = 0;
  std::size_t scratch_large_cached_bytes = 0;
  std::uint64_t scratch_allocation_count = 0;
  std::size_t global_cache_limit = 0;
  std::size_t global_cached_bytes = 0;
  std::size_t configured_parallelism = 1;
  bool executor_started = false;
  std::size_t started_worker_count = 0;
  std::size_t max_parallelism = 1;
  std::uint64_t batches_queued = 0;
  std::uint64_t batches_run = 0;
  std::uint64_t wait_ns = 0;
};

using RuntimeDiagnosticSink = void (*)(const char* component, const char* event);

// 回调在执行器初始化/锁之外执行，可重入 runtime；忽略异常。替换回调不等待在途调用，
// 其代码和引用状态须存活至全部调用静止。
void set_runtime_diagnostic_sink(RuntimeDiagnosticSink sink) noexcept;

[[nodiscard]] RuntimeBackendConfig runtime_backend_config() noexcept;
[[nodiscard]] const char* memory_backend_name() noexcept;
[[nodiscard]] const char* parallel_backend_name() noexcept;
[[nodiscard]] RuntimeObservability runtime_observability() noexcept;

}  // namespace mv::runtime
