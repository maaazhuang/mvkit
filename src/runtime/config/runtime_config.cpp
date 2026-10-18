#include "runtime/config/runtime_config.hpp"

#include "runtime/cpu/cpu_features.hpp"
#include "runtime/parallel/parallel_for.hpp"
#include "runtime/parallel/parallel_state.hpp"

#include <mvkit/runtime/generated_backend_config.hpp>

#if MVKIT_MEMORY_BACKEND_BUILTIN
#include "runtime/memory/builtin_pool_resource.hpp"
#endif

namespace mv::runtime {
namespace {

const char* simd_policy_name(SimdOverride policy) noexcept {
  switch (policy) {
    case SimdOverride::Scalar:
      return "scalar";
    case SimdOverride::Sse2:
      return "sse2";
    case SimdOverride::Avx2:
      return "avx2";
    case SimdOverride::Neon:
      return "neon";
    case SimdOverride::Invalid:
      return "invalid";
    case SimdOverride::Auto:
      break;
  }
  return "auto";
}

}  // namespace

RuntimeBackendConfig runtime_backend_config() noexcept {
#if MVKIT_MEMORY_BACKEND_BUILTIN
  constexpr MemoryBackend memory_backend = MemoryBackend::Builtin;
#else
  constexpr MemoryBackend memory_backend = MemoryBackend::System;
#endif
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  constexpr ParallelBackend parallel_backend = ParallelBackend::Builtin;
#else
  constexpr ParallelBackend parallel_backend = ParallelBackend::Serial;
#endif
  return {memory_backend, parallel_backend,
          MVKIT_ENABLE_MEMORY_POOL != 0, MVKIT_ENABLE_PARALLEL != 0};
}

const char* memory_backend_name() noexcept {
#if MVKIT_MEMORY_BACKEND_BUILTIN
  return "builtin";
#else
  return "system";
#endif
}
const char* parallel_backend_name() noexcept {
#if MVKIT_PARALLEL_BACKEND_BUILTIN
  return "builtin";
#else
  return "serial";
#endif
}

RuntimeObservability runtime_observability() noexcept {
  RuntimeObservability snapshot;
  snapshot.topology = cpu_topology();
  const auto& cpu = cpu_features();
  snapshot.requested_isa = simd_policy_name(cpu.policy());
  snapshot.effective_isa = cpu.selection().kernel;
  snapshot.isa_reason = cpu.fallback_reason();
  snapshot.memory_backend = memory_backend_name();
  snapshot.parallel_backend = parallel_backend_name();
  snapshot.configured_parallelism = configured_parallelism();
  snapshot.executor_started = parallel_executor_started();
  snapshot.started_worker_count = started_parallel_worker_count();
  snapshot.max_parallelism = snapshot.executor_started
                                 ? snapshot.started_worker_count
                                 : snapshot.configured_parallelism;
  const auto scheduler = detail::scheduler_snapshot();
  snapshot.batches_queued = scheduler.batches_queued;
  snapshot.batches_run = scheduler.batches_run;
  snapshot.wait_ns = scheduler.wait_ns;
#if MVKIT_MEMORY_BACKEND_BUILTIN
  const auto persist = builtin_persist_pool()->stats();
  snapshot.persist_live_bytes = persist.live_bytes;
  snapshot.persist_cached_bytes = persist.cached_bytes;
  snapshot.persist_cache_limit = builtin_persist_pool()->cache_limit();
  snapshot.persist_large_cached_bytes = persist.large_cached_bytes;
  snapshot.persist_allocation_count = persist.allocation_count;
  const auto scratch = builtin_scratch_pool()->stats();
  snapshot.scratch_live_bytes = scratch.live_bytes;
  snapshot.scratch_cached_bytes = scratch.cached_bytes;
  snapshot.scratch_cache_limit = builtin_scratch_pool()->cache_limit();
  snapshot.scratch_large_cached_bytes = scratch.large_cached_bytes;
  snapshot.scratch_allocation_count = scratch.allocation_count;
  snapshot.global_cache_limit = global_cache_limit();
  snapshot.global_cached_bytes = global_cached_bytes();
#endif
  return snapshot;
}

void set_runtime_diagnostic_sink(RuntimeDiagnosticSink sink) noexcept {
  detail::set_scheduler_diagnostic_sink(sink);
}

}  // namespace mv::runtime
