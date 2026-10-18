#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <span>
#include <utility>
#include <vector>

#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {

inline constexpr std::size_t kNeighborhoodNoSkip = (std::numeric_limits<std::size_t>::max)();

// kNN 复用工作区，不跨线程共享；堆项为（欧氏距离平方，原点索引）。
struct NeighborhoodQueryScratch {
  explicit NeighborhoodQueryScratch(std::pmr::memory_resource* memory = nullptr);
  void prepare(std::size_t k);
  std::pmr::vector<std::pair<double, std::size_t>> heap;
};

// 对 ObjectModel3D 有效有限点建立私有精确 nearest/kNN 索引，距离参数为欧氏平方距离。
// 有限 max_d2 包含端点，非有限表示不限距精确查询。建树后只读，调用方提供且不共享 scratch。
// nearest 同距保留最大原索引（原 last-wins），knn 同距保留最小原索引。
// 自排除用原索引，同坐标其他点仍为邻居；k 超可用邻居数时返回实际数。空参考 nearest=false、knn=0。
class NeighborhoodIndex;
// 每操作单调排除工作区，索引不变；此处标记点也须在调用方掩码标记。
// 子树全部点排除前保留子节点活跃状态。
class NeighborhoodVisitedState {
 public:
  explicit NeighborhoodVisitedState(const NeighborhoodIndex& index,
      std::pmr::memory_resource* memory = nullptr);
  void exclude(std::size_t point);
 private:
  friend class NeighborhoodIndex;
  const NeighborhoodIndex* owner_;
  std::pmr::vector<std::uint32_t> parents_, ranks_;
  std::pmr::vector<std::uint8_t> active_;
};

class NeighborhoodIndex {
 public:
  NeighborhoodIndex(const ObjectModel3DData& model, std::pmr::memory_resource* memory = nullptr);

  NeighborhoodIndex(const NeighborhoodIndex&) = delete;
  NeighborhoodIndex& operator=(const NeighborhoodIndex&) = delete;

  [[nodiscard]] std::size_t size() const noexcept { return orig_.size(); }
  [[nodiscard]] std::span<const std::size_t> valid_indices() const noexcept {
    return {orig_.data(), orig_.size()};
  }

  bool nearest(const Point3d& query, double max_d2, std::size_t* idx, double* d2,
               std::size_t skip_index = kNeighborhoodNoSkip, bool prefer_first = false) const;

  std::size_t knn(std::size_t index, std::size_t k, std::size_t* out_idx, double* out_d2,
                  NeighborhoodQueryScratch* scratch) const;

  // 可选查询排除掩码避免物化已访问图顶点，但树剪枝仍须经过被排除顶点。
  void radius(std::size_t index, double radius, std::vector<std::size_t>* out,
              std::span<const std::uint8_t> excluded = {}) const;

  // 距离包含端点及遍历序同 radius，仅剪空子树。
  void radius_unvisited(std::size_t index, double radius, std::vector<std::size_t>* out,
      std::span<const std::uint8_t> excluded, const NeighborhoodVisitedState& state) const;

 private:
  friend class NeighborhoodVisitedState;
  struct Node {
    std::uint32_t left{(std::numeric_limits<std::uint32_t>::max)()};
    std::uint32_t right{(std::numeric_limits<std::uint32_t>::max)()};
    std::size_t point{};
    std::uint8_t axis{};
    double split{};
  };

  std::uint32_t build(std::size_t* ids, std::size_t begin, std::size_t end, int depth);

  const ObjectModel3DData* model_{};
  std::pmr::vector<std::size_t> orig_;
  std::pmr::vector<Node> nodes_;
  std::uint32_t root_{(std::numeric_limits<std::uint32_t>::max)()};
  int split_axes_[3]{0, 1, 2};
  int split_axis_count_{3};
};

}  // namespace mv::reconstruction
