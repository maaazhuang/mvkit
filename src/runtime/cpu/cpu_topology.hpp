#pragma once
#include <cstddef>

namespace mv::runtime {
struct CpuTopology {
  std::size_t logical_count = 1;
  std::size_t available_logical_count = 1;
  std::size_t physical_count = 0; // 零表示未知，不表示没有核心。
  std::size_t high_performance_cores = 0;
  std::size_t efficiency_cores = 0;
  std::size_t processor_groups = 0;
  bool heterogeneous = false;
  bool affinity_known = false;
};
// 首次查询快照，不绑定线程。核心分类数描述整个系统；已知时 available_logical_count 反映支持的亲和性掩码。
[[nodiscard]] const CpuTopology& cpu_topology() noexcept;
}
