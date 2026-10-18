#pragma once

namespace mv::runtime {

enum class Architecture {
  Unknown = 0,
  X86,
  X64,
  Arm,
  Arm64,
  Arm64EC,
};

[[nodiscard]] const char* architecture_name(Architecture architecture) noexcept;
[[nodiscard]] constexpr Architecture compile_architecture() noexcept {
#if defined(_M_ARM64EC)
  return Architecture::Arm64EC;
#elif defined(_M_ARM64) || defined(__aarch64__)
  return Architecture::Arm64;
#elif defined(_M_X64) || defined(__x86_64__)
  return Architecture::X64;
#elif defined(_M_IX86) || defined(__i386__)
  return Architecture::X86;
#elif defined(_M_ARM) || defined(__arm__)
  return Architecture::Arm;
#else
  return Architecture::Unknown;
#endif
}

}  // namespace mv::runtime
