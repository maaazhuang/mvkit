#include "runtime/cpu/cpu_topology.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <thread>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#elif defined(__linux__)
#include <sched.h>
#endif

namespace mv::runtime {
namespace {
CpuTopology capture() noexcept {
  CpuTopology result;
  result.logical_count = (std::max)(1u, std::thread::hardware_concurrency());
  result.available_logical_count = result.logical_count;
#ifdef _WIN32
  result.processor_groups = GetActiveProcessorGroupCount();
  if (const auto count = GetActiveProcessorCount(ALL_PROCESSOR_GROUPS))
    result.logical_count = result.available_logical_count = count;
  // 多组 Windows 上此 API 可能仅返回主组；不能静默将系统预算限制到主组或宣称完整可见性。
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (result.processor_groups == 1 &&
      GetProcessAffinityMask(GetCurrentProcess(), &process_mask, &system_mask) && process_mask) {
    result.available_logical_count = std::popcount(process_mask);
    result.affinity_known = true;
  }
  try {
    DWORD bytes = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &bytes);
    std::vector<unsigned char> buffer(bytes);
    if (!bytes || !GetLogicalProcessorInformationEx(RelationProcessorCore,
        reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data()), &bytes)) return result;
    std::array<std::size_t, 256> classes{};
    std::size_t cores = 0;
    for (DWORD offset = 0; offset < bytes;) {
      const auto* entry = reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data() + offset);
      if (!entry->Size || entry->Size > bytes - offset) return result;
      if (entry->Relationship == RelationProcessorCore) {
        ++cores;
        ++classes[entry->Processor.EfficiencyClass];
      }
      offset += entry->Size;
    }
    unsigned distinct = 0, highest = 0;
    for (unsigned i = 0; i < classes.size(); ++i)
      if (classes[i]) { ++distinct; highest = i; }
    result.physical_count = cores;
    result.heterogeneous = distinct > 1;
    if (result.heterogeneous) {
      result.high_performance_cores = classes[highest];
      result.efficiency_cores = cores - classes[highest];
    }
  } catch (...) { /** 可选诊断不能阻止执行。 */ }
#elif defined(__linux__)
  cpu_set_t mask;
  CPU_ZERO(&mask);
  if (sched_getaffinity(0, sizeof(mask), &mask) == 0 && CPU_COUNT(&mask) > 0) {
    result.available_logical_count = static_cast<std::size_t>(CPU_COUNT(&mask));
    result.affinity_known = true;
  }
#endif
  return result;
}
}
const CpuTopology& cpu_topology() noexcept {
  static const auto value = capture();
  return value;
}
}
