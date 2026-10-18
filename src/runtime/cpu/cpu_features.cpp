#include "runtime/cpu/cpu_features.hpp"
#include "runtime/env.hpp"

#include <mvkit/runtime/generated_backend_config.hpp>

#include <cstring>

#if defined(_MSC_VER) && defined(_M_X64)
#include <intrin.h>
#elif defined(__GNUC__) || defined(__clang__)
#if defined(__x86_64__) || defined(__i386__)
#include <cpuid.h>
#endif
#endif

#include <cstdlib>

namespace mv::runtime {
namespace {

CpuCapabilities detect_hardware() noexcept {
  CpuCapabilities caps;
#if defined(_MSC_VER) && defined(_M_X64)
  int info[4]{};
  __cpuid(info, 1);
  caps.sse2 = (info[3] & (1 << 26)) != 0;
  caps.osxsave = (info[2] & (1 << 27)) != 0;
  caps.avx = (info[2] & (1 << 28)) != 0;
  caps.fma = (info[2] & (1 << 12)) != 0;
  if (caps.osxsave) {
    const unsigned long long xcr0 = _xgetbv(0);
    caps.xcr0_xmm_ymm = (xcr0 & 0x6ull) == 0x6ull;
  }
  __cpuidex(info, 7, 0);
  caps.avx2 = (info[1] & (1 << 5)) != 0;
#elif (defined(__GNUC__) || defined(__clang__)) && \
    (defined(__x86_64__) || defined(__i386__))
  unsigned eax = 0, ebx = 0, ecx = 0, edx = 0;
  if (__get_cpuid(1, &eax, &ebx, &ecx, &edx)) {
    caps.sse2 = (edx & (1u << 26)) != 0;
    caps.osxsave = (ecx & (1u << 27)) != 0;
    caps.avx = (ecx & (1u << 28)) != 0;
    caps.fma = (ecx & (1u << 12)) != 0;
  }
  if (caps.osxsave) {
    unsigned eax_xcr = 0, edx_xcr = 0;
    __asm__ volatile("xgetbv" : "=a"(eax_xcr), "=d"(edx_xcr) : "c"(0));
    const unsigned long long xcr0 =
        (static_cast<unsigned long long>(edx_xcr) << 32) | eax_xcr;
    caps.xcr0_xmm_ymm = (xcr0 & 0x6ull) == 0x6ull;
  }
  if (__get_cpuid_count(7, 0, &eax, &ebx, &ecx, &edx))
    caps.avx2 = (ebx & (1u << 5)) != 0;
#elif defined(_M_ARM64) || defined(__aarch64__)
  caps.neon = true;
#endif
  return caps;
}

KernelAvailability compiled_kernels_for(Architecture architecture) noexcept {
  KernelAvailability compiled;
  compiled.scalar = true;
  if (architecture == Architecture::X64 || architecture == Architecture::X86) {
    compiled.sse2 = true;
    compiled.avx2 = MVKIT_ENABLE_AVX2_KERNEL != 0;
  }
  compiled.neon = false;
  return compiled;
}

}  // namespace

CpuFeatures CpuFeatures::capture() noexcept {
  CpuFeatures features;
  features.architecture_ = compile_architecture();
  features.hardware_ = detect_hardware();
  features.compiled_ = compiled_kernels_for(features.architecture_);
  features.policy_ = parse_simd_override(getenv_or_null("MVKIT_SIMD"));
  const bool disable_avx2 =
      getenv_or_null("MVKIT_SHAPE_DISABLE_AVX2") != nullptr;
  features.selection_ =
      select_isa(features.architecture_, features.hardware_, features.compiled_,
                 features.policy_, disable_avx2);
  return features;
}

const char* architecture_name(Architecture architecture) noexcept {
  switch (architecture) {
    case Architecture::X86:
      return "x86";
    case Architecture::X64:
      return "x64";
    case Architecture::Arm:
      return "arm";
    case Architecture::Arm64:
      return "arm64";
    case Architecture::Arm64EC:
      return "arm64ec";
    case Architecture::Unknown:
      break;
  }
  return "unknown";
}

SimdOverride parse_simd_override(const char* text) noexcept {
  if (text == nullptr || text[0] == '\0' || std::strcmp(text, "auto") == 0)
    return SimdOverride::Auto;
  if (std::strcmp(text, "scalar") == 0) return SimdOverride::Scalar;
  if (std::strcmp(text, "sse2") == 0) return SimdOverride::Sse2;
  if (std::strcmp(text, "avx2") == 0) return SimdOverride::Avx2;
  if (std::strcmp(text, "neon") == 0) return SimdOverride::Neon;
  return SimdOverride::Invalid;
}

IsaSelection select_isa(Architecture architecture, CpuCapabilities hardware,
                        KernelAvailability compiled, SimdOverride policy,
                        bool disable_avx2_ceiling) noexcept {
  if (architecture != Architecture::X64 && architecture != Architecture::X86) {
    compiled.sse2 = false;
    compiled.avx2 = false;
  }
  if (architecture != Architecture::Arm64 && architecture != Architecture::Arm)
    compiled.neon = false;
  if (architecture == Architecture::Unknown ||
      architecture == Architecture::Arm64EC) {
    compiled.sse2 = false;
    compiled.avx2 = false;
    compiled.neon = false;
  }

  CpuCapabilities legal = hardware;
  if (!hardware.osxsave || !hardware.xcr0_xmm_ymm) {
    legal.avx = false;
    legal.avx2 = false;
    legal.fma = false;
  }

  bool avx2_legal = compiled.avx2 && legal.avx && legal.avx2;
  bool sse2_legal = compiled.sse2 && legal.sse2;
  const bool neon_legal = compiled.neon && legal.neon;
  if (disable_avx2_ceiling) avx2_legal = false;

  IsaSelection selected;
  auto choose_auto = [&] {
    if (avx2_legal) {
      selected.kernel = "avx2";
      selected.avx2 = true;
      selected.sse2 = sse2_legal;
      return;
    }
    if (sse2_legal) {
      selected.kernel = "sse2";
      selected.sse2 = true;
      if (compiled.avx2) selected.reason = "os_or_hardware_blocks_avx2";
      return;
    }
    if (neon_legal) {
      selected.kernel = "neon";
      selected.neon = true;
      return;
    }
    selected.kernel = "scalar";
  };

  switch (policy) {
    case SimdOverride::Auto:
      choose_auto();
      break;
    case SimdOverride::Scalar:
      selected.kernel = "scalar";
      break;
    case SimdOverride::Sse2:
      if (sse2_legal) {
        selected.kernel = "sse2";
        selected.sse2 = true;
      } else {
        selected.kernel = "scalar";
        selected.reason = "forced_sse2_unavailable";
      }
      break;
    case SimdOverride::Avx2:
      if (avx2_legal) {
        selected.kernel = "avx2";
        selected.avx2 = true;
        selected.sse2 = sse2_legal;
      } else {
        selected.kernel = "scalar";
        selected.reason = disable_avx2_ceiling ? "shape_disable_avx2_ceiling"
                                               : "forced_avx2_unavailable";
      }
      break;
    case SimdOverride::Neon:
      if (neon_legal) {
        selected.kernel = "neon";
        selected.neon = true;
      } else {
        selected.kernel = "scalar";
        selected.reason = compiled.neon ? "forced_neon_unavailable"
                                        : "neon_kernel_not_compiled";
      }
      break;
    case SimdOverride::Invalid:
      selected.kernel = "scalar";
      selected.reason = "invalid_mvkit_simd";
      break;
  }
  return selected;
}

CpuCapabilities effective_capabilities(const IsaSelection& selected,
                                       const CpuCapabilities& hardware) noexcept {
  CpuCapabilities legal = hardware;
  if (!hardware.osxsave || !hardware.xcr0_xmm_ymm) {
    legal.avx = false;
    legal.avx2 = false;
    legal.fma = false;
  }
  CpuCapabilities effective;
  effective.sse2 = selected.sse2 || selected.avx2;
  effective.avx = selected.avx2;
  effective.avx2 = selected.avx2;
  effective.fma = selected.avx2 && legal.fma;
  effective.neon = selected.neon;
  return effective;
}

bool kernel_allowed(const CpuCapabilities& effective,
                    KernelRequirement requirement) noexcept {
  if (requirement.sse2 && !effective.sse2) return false;
  if (requirement.avx2 && !effective.avx2) return false;
  if (requirement.fma && !effective.fma) return false;
  if (requirement.neon && !effective.neon) return false;
  return true;
}

const CpuFeatures& cpu_features() noexcept {
  static const CpuFeatures features = CpuFeatures::capture();
  return features;
}

}  // namespace mv::runtime
