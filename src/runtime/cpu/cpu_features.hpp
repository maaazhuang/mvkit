#pragma once

#include "runtime/cpu/architecture.hpp"

namespace mv::runtime {

enum class SimdOverride { Auto, Scalar, Sse2, Avx2, Neon, Invalid };

struct CpuCapabilities {
  bool sse2 = false;
  bool avx = false;
  bool avx2 = false;
  bool fma = false;
  bool neon = false;
  bool osxsave = false;
  bool xcr0_xmm_ymm = false;
};

struct KernelAvailability {
  bool scalar = true;
  bool sse2 = false;
  bool avx2 = false;
  bool neon = false;
};

struct KernelRequirement {
  bool sse2 = false;
  bool avx2 = false;
  bool fma = false;
  bool neon = false;
};

struct IsaSelection {
  const char* kernel = "scalar";
  const char* reason = "";
  bool sse2 = false;
  bool avx2 = false;
  bool neon = false;
};

[[nodiscard]] SimdOverride parse_simd_override(const char* text) noexcept;

[[nodiscard]] IsaSelection select_isa(Architecture architecture,
                                      CpuCapabilities hardware,
                                      KernelAvailability compiled,
                                      SimdOverride policy,
                                      bool disable_avx2_ceiling) noexcept;

[[nodiscard]] CpuCapabilities effective_capabilities(
    const IsaSelection& selected, const CpuCapabilities& hardware) noexcept;

[[nodiscard]] bool kernel_allowed(const CpuCapabilities& effective,
                                  KernelRequirement requirement) noexcept;

class CpuFeatures {
 public:
  [[nodiscard]] Architecture architecture() const noexcept {
    return architecture_;
  }
  [[nodiscard]] CpuCapabilities hardware() const noexcept { return hardware_; }
  [[nodiscard]] SimdOverride policy() const noexcept { return policy_; }
  [[nodiscard]] IsaSelection selection() const noexcept { return selection_; }
  [[nodiscard]] KernelAvailability compiled_kernels() const noexcept {
    return compiled_;
  }
  [[nodiscard]] const char* fallback_reason() const noexcept {
    return selection_.reason;
  }

 private:
  friend const CpuFeatures& cpu_features() noexcept;
  static CpuFeatures capture() noexcept;
  Architecture architecture_ = Architecture::Unknown;
  CpuCapabilities hardware_{};
  KernelAvailability compiled_{};
  SimdOverride policy_ = SimdOverride::Auto;
  IsaSelection selection_{};
};

[[nodiscard]] const CpuFeatures& cpu_features() noexcept;

}  // namespace mv::runtime
