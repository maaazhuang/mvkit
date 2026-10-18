#pragma once

#include <cstddef>

namespace mv::runtime {

struct IndexRange {
  // 半开区间 [begin,end)，begin>=end 表示空区间。
  std::size_t begin;
  std::size_t end;
};

struct ParallelBodyView {
  void* context;
  void (*invoke)(void*, IndexRange);
};

class Executor {
 public:
  virtual ~Executor() = default;
  // grain_size==0 使用实现定义的自动块大小；同步完成并重新抛出首个 body 异常。
  virtual void execute(IndexRange range, std::size_t grain_size,
                       ParallelBodyView body) = 0;
  [[nodiscard]] virtual std::size_t max_parallelism() const noexcept = 0;
};

}  // namespace mv::runtime
