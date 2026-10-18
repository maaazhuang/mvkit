#include <reconstruction/neighborhood_index.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace mv::reconstruction {
namespace {

constexpr std::uint32_t kNullNode = (std::numeric_limits<std::uint32_t>::max)();

bool slot_indexable(const ObjectModel3DData& model, std::size_t index) {
  return model.valid.empty() || model.valid[index] != 0U;
}

double axis_value(const Point3d& p, int axis) {
  switch (axis) {
    case 0:
      return p.x;
    case 1:
      return p.y;
    default:
      return p.z;
  }
}

double dist2(const Point3d& a, const Point3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

std::pmr::memory_resource* require_memory(std::pmr::memory_resource* memory) {
  return memory != nullptr ? memory : std::pmr::new_delete_resource();
}

}  // namespace

NeighborhoodQueryScratch::NeighborhoodQueryScratch(std::pmr::memory_resource* memory)
    : heap(require_memory(memory)) {}

void NeighborhoodQueryScratch::prepare(std::size_t k) {
  heap.clear();
  if (k > 0 && heap.capacity() < k) heap.reserve(k);
}

NeighborhoodIndex::NeighborhoodIndex(const ObjectModel3DData& model,
                                     std::pmr::memory_resource* memory)
    : model_(&model), orig_(require_memory(memory)), nodes_(require_memory(memory)) {
  orig_.reserve(model.points.size());
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (!slot_indexable(model, i)) continue;
    const auto& p = model.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    orig_.push_back(i);
  }
  if (orig_.empty()) return;
  // 每不可变参考一次选择活跃坐标；平面/线中不为恒定坐标浪费树层。
  const auto& first = model.points[orig_.front()];
  double low[3] = {first.x, first.y, first.z};
  double high[3] = {first.x, first.y, first.z};
  for (std::size_t index : orig_) {
    const auto& point = model.points[index];
    const double values[3] = {point.x, point.y, point.z};
    for (int axis = 0; axis < 3; ++axis) {
      low[axis] = std::min(low[axis], values[axis]);
      high[axis] = std::max(high[axis], values[axis]);
    }
  }
  const double spreads[3] = {high[0] - low[0], high[1] - low[1], high[2] - low[2]};
  split_axis_count_ = 0;
  for (int axis = 0; axis < 3; ++axis)
    if (spreads[axis] > 0.0) split_axes_[split_axis_count_++] = axis;
  if (split_axis_count_ == 0) {
    split_axes_[0] = 0; split_axes_[1] = 1; split_axes_[2] = 2;
    split_axis_count_ = 3;
  } else {
    std::sort(split_axes_, split_axes_ + split_axis_count_, [&](int a, int b) {
      return spreads[a] > spreads[b] || (spreads[a] == spreads[b] && a < b);
    });
  }
  nodes_.reserve(orig_.size());
  std::pmr::vector<std::size_t> order(orig_.begin(), orig_.end(), require_memory(memory));
  root_ = build(order.data(), 0, order.size(), 0);
}

std::uint32_t NeighborhoodIndex::build(std::size_t* ids, std::size_t begin, std::size_t end,
                                       int depth) {
  if (begin >= end) return kNullNode;
  int axis = split_axes_[depth % split_axis_count_];
  const std::size_t count = end - begin;
  std::size_t pivot = begin + count / 2;
  std::size_t split = pivot;
  bool sampled = false;
  auto widest_axis = [&](auto position, std::size_t n) {
    const auto& first=model_->points[ids[position(0)]];
    double low[3]={first.x,first.y,first.z},high[3]={first.x,first.y,first.z};
    for (std::size_t j=1;j<n;++j) {
      const auto& p=model_->points[ids[position(j)]];
      const double v[3]={p.x,p.y,p.z};
      for(int k=0;k<3;++k) { low[k]=std::min(low[k],v[k]);high[k]=std::max(high[k],v[k]); }
    }
    for(int k=0;k<3;++k) if(high[k]-low[k]>high[axis]-low[axis])axis=k;
  };
  if (count >= 64) {
    // 样本中位数只需一次线性划分，而 nth_element 反复划分；重复值/聚类使浅树过度失衡时回退。
    std::array<std::pair<double, std::size_t>, 9> sample;
    std::uint64_t state = (static_cast<std::uint64_t>(begin) + 1) *
                              UINT64_C(0x9e3779b97f4a7c15) ^
                          (static_cast<std::uint64_t>(end) + 1) *
                              UINT64_C(0xbf58476d1ce4e5b9) ^
                          static_cast<std::uint64_t>(depth + 1);
    for (auto& entry : sample) {
      state ^= state >> 12;
      state ^= state << 25;
      state ^= state >> 27;
      const std::size_t position = begin + static_cast<std::size_t>(state % count);
      entry = {axis_value(model_->points[ids[position]], axis), position};
    }
    // 局部长宽比比点云全局尺寸更关键，复用主元样本，避免大节点再完整扫描。
    widest_axis([&](std::size_t j) { return sample[j].second; },sample.size());
    for(auto& entry:sample) entry.first=axis_value(model_->points[ids[entry.second]],axis);
    std::nth_element(sample.begin(), sample.begin() + 4, sample.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    pivot = sample[4].second;
    const double pivot_value = sample[4].first;
    std::swap(ids[begin], ids[pivot]);
    split = static_cast<std::size_t>(std::partition(
        ids + begin + 1, ids + end, [&](std::size_t id) {
          return axis_value(model_->points[id], axis) < pivot_value;
        }) - ids);
    sampled = split - begin >= count / 8 && end - split >= count / 8;
  }
  if (!sampled) {
    if (count < 64 && count > 1)
      widest_axis([&](std::size_t j) { return begin+j; },count);
    pivot = begin + count / 2;
    std::nth_element(ids + begin, ids + pivot, ids + end,
                     [&](std::size_t a, std::size_t b) {
                       return axis_value(model_->points[a], axis) <
                              axis_value(model_->points[b], axis);
                     });
    split = pivot + 1;
  }
  const auto node = static_cast<std::uint32_t>(nodes_.size());
  Node built;
  built.point = ids[sampled ? begin : pivot];
  built.axis = static_cast<std::uint8_t>(axis);
  built.split = axis_value(model_->points[built.point], axis);
  nodes_.push_back(built);
  nodes_[node].left = build(ids, sampled ? begin + 1 : begin,
                            sampled ? split : pivot, depth + 1);
  nodes_[node].right = build(ids, split, end, depth + 1);
  return node;
}

bool NeighborhoodIndex::nearest(const Point3d& query, double max_d2, std::size_t* idx, double* d2,
                                std::size_t skip_index, bool prefer_first) const {
  if (idx == nullptr || d2 == nullptr || orig_.empty() || root_ == kNullNode) return false;
  const double infinity = std::numeric_limits<double>::infinity();
  const double cap = max_d2 > -infinity && max_d2 < infinity ? max_d2 : infinity;
  double best = cap;
  std::size_t best_i = 0;
  bool found = false;

  auto consider = [&](std::size_t other) {
    if (other == skip_index) return;
    const double d = dist2(query, model_->points[other]);
    // 近邻平方距离非负；此比较同时拒绝 NaN/Inf，无需外部 CRT 分类。
    if (!(d < infinity) || d > cap) return;
    if (!found || d < best || (d == best && (prefer_first ? other < best_i : other > best_i))) {
      best = d;
      best_i = other;
      found = true;
    }
  };

  // 沿递归路径保留三个轴的单元距离下界，返回后恢复；不增加持久节点布局。
  std::array<double, 3> cell_distance{};
  auto search = [&](auto&& self, std::uint32_t node) -> void {
    if (node == kNullNode) return;
    const Node& nd = nodes_[node];
    consider(nd.point);
    const double delta = axis_value(query, nd.axis) - nd.split;
    const std::uint32_t first = delta < 0.0 ? nd.left : nd.right;
    const std::uint32_t second = delta < 0.0 ? nd.right : nd.left;
    self(self, first);
    if (second != kNullNode) {
      const double previous = cell_distance[nd.axis];
      cell_distance[nd.axis] = delta * delta;
      // 按 dist2 的 x/y/z 顺序求和；只剪去严格超界单元，距离并列仍继续搜索。
      if (cell_distance[0] + cell_distance[1] + cell_distance[2] <= best) self(self, second);
      cell_distance[nd.axis] = previous;
    }
  };
  search(search, root_);
  if (!found) return false;
  *idx = best_i;
  *d2 = best;
  return true;
}

std::size_t NeighborhoodIndex::knn(std::size_t index, std::size_t k, std::size_t* out_idx,
                                   double* out_d2, NeighborhoodQueryScratch* scratch) const {
  if (k == 0 || out_idx == nullptr || out_d2 == nullptr || scratch == nullptr || orig_.empty() ||
      index >= model_->points.size()) {
    return 0;
  }
  scratch->prepare(k);
  auto& heap = scratch->heap;
  const Point3d& query = model_->points[index];
  const double infinity = std::numeric_limits<double>::infinity();

  auto better = [](const std::pair<double, std::size_t>& cand,
                   const std::pair<double, std::size_t>& worst) {
    return cand.first < worst.first || (cand.first == worst.first && cand.second < worst.second);
  };
  auto consider = [&](std::size_t other) {
    if (other == index) return;
    const double d = dist2(query, model_->points[other]);
    if (!(d < infinity)) return;
    const std::pair<double, std::size_t> cand{d, other};
    if (heap.size() < k) {
      heap.push_back(cand);
      std::push_heap(heap.begin(), heap.end());
    } else if (better(cand, heap.front())) {
      // 替换最大项只需向下 sift，不必先 pop 再 push；点对排序保留原索引同值规则。
      std::size_t hole = 0;
      while (hole < heap.size() / 2) {
        std::size_t child = hole * 2 + 1;
        if (child + 1 < heap.size() && heap[child] < heap[child + 1]) ++child;
        if (!(cand < heap[child])) break;
        heap[hole] = heap[child];
        hole = child;
      }
      heap[hole] = cand;
    }
  };

  // 沿递归路径保留三个轴的单元距离下界，返回后恢复；不增加持久节点布局。
  std::array<double, 3> cell_distance{};
  auto search = [&](auto&& self, std::uint32_t node) -> void {
    if (node == kNullNode) return;
    const Node& nd = nodes_[node];
    consider(nd.point);
    const double delta = axis_value(query, nd.axis) - nd.split;
    const std::uint32_t first = delta < 0.0 ? nd.left : nd.right;
    const std::uint32_t second = delta < 0.0 ? nd.right : nd.left;
    self(self, first);
    const double bound = heap.size() < k ? std::numeric_limits<double>::infinity() : heap.front().first;
    if (second != kNullNode) {
      const double previous = cell_distance[nd.axis];
      cell_distance[nd.axis] = delta * delta;
      // 按 dist2 的 x/y/z 顺序求和；只剪去严格超界单元，距离并列仍继续搜索。
      if (cell_distance[0] + cell_distance[1] + cell_distance[2] <= bound) self(self, second);
      cell_distance[nd.axis] = previous;
    }
  };
  search(search, root_);
  std::sort(heap.begin(), heap.end());
  const std::size_t n = heap.size();
  for (std::size_t i = 0; i < n; ++i) {
    out_d2[i] = heap[i].first;
    out_idx[i] = heap[i].second;
  }
  return n;
}

void NeighborhoodIndex::radius(std::size_t index, double radius, std::vector<std::size_t>* out,
                               std::span<const std::uint8_t> excluded) const {
  if (out == nullptr) return;
  out->clear();
  if (!(radius >= 0.0) || orig_.empty() || index >= model_->points.size()) return;
  const double r2 = radius * radius;
  const double infinity = std::numeric_limits<double>::infinity();
  const Point3d& query = model_->points[index];
  // 沿递归路径保留三个轴的单元距离下界，返回后恢复；不增加持久节点布局。
  std::array<double, 3> cell_distance{};
  auto search = [&](auto&& self, std::uint32_t node) -> void {
    if (node == kNullNode) return;
    const Node& nd = nodes_[node];
    if (nd.point != index && (nd.point >= excluded.size() || excluded[nd.point] == 0U)) {
      const double d = dist2(query, model_->points[nd.point]);
      if (d < infinity && d <= r2) out->push_back(nd.point);
    }
    const double delta = axis_value(query, nd.axis) - nd.split;
    const std::uint32_t first = delta < 0.0 ? nd.left : nd.right;
    const std::uint32_t second = delta < 0.0 ? nd.right : nd.left;
    self(self, first);
    if (second != kNullNode) {
      const double previous = cell_distance[nd.axis];
      cell_distance[nd.axis] = delta * delta;
      // 按 dist2 的 x/y/z 顺序求和；只剪去严格超界单元，距离并列仍继续搜索。
      if (cell_distance[0] + cell_distance[1] + cell_distance[2] <= r2) self(self, second);
      cell_distance[nd.axis] = previous;
    }
  };
  search(search, root_);
}

NeighborhoodVisitedState::NeighborhoodVisitedState(const NeighborhoodIndex& index,
                                                   std::pmr::memory_resource* memory)
    : owner_(&index), parents_(require_memory(memory)), ranks_(require_memory(memory)),
      active_(require_memory(memory)) {
  parents_.assign(index.nodes_.size(), kNullNode);
  ranks_.assign(index.model_->points.size(), kNullNode);
  active_.resize(index.nodes_.size());
  for (std::size_t i = 0; i < index.nodes_.size(); ++i) {
    const auto& node = index.nodes_[i];
    ranks_[node.point] = static_cast<std::uint32_t>(i);
    if (node.left != kNullNode) parents_[node.left] = static_cast<std::uint32_t>(i);
    if (node.right != kNullNode) parents_[node.right] = static_cast<std::uint32_t>(i);
    // 位 0 表示本点未排除，位 1/2 表示左/右子树仍有未见点。
    active_[i] = static_cast<std::uint8_t>(1U | (node.left != kNullNode ? 2U : 0U) |
                                        (node.right != kNullNode ? 4U : 0U));
  }
}

void NeighborhoodVisitedState::exclude(std::size_t point) {
  if (point >= ranks_.size()) return;
  auto node = ranks_[point];
  if (node == kNullNode) return;
  ranks_[point] = kNullNode;  // 重复排除不能二次传播。
  active_[node] &= static_cast<std::uint8_t>(~1U);
  while (active_[node] == 0U) {
    const auto parent = parents_[node];
    if (parent == kNullNode) break;
    const auto bit = owner_->nodes_[parent].left == node ? 2U : 4U;
    active_[parent] &= static_cast<std::uint8_t>(~bit);
    node = parent;
  }
}

void NeighborhoodIndex::radius_unvisited(std::size_t index, double radius, std::vector<std::size_t>* out,
                               std::span<const std::uint8_t> excluded, const NeighborhoodVisitedState& state) const {
  if (state.owner_ != this) {
    this->radius(index, radius, out, excluded);
    return;
  }
  if (out == nullptr) return;
  out->clear();
  if (!(radius >= 0.0) || orig_.empty() || index >= model_->points.size()) return;
  const double r2 = radius * radius;
  const double infinity = std::numeric_limits<double>::infinity();
  const Point3d& query = model_->points[index];
  // 沿递归路径保留三个轴的单元距离下界，返回后恢复；不增加持久节点布局。
  std::array<double, 3> cell_distance{};
  auto search = [&](auto&& self, std::uint32_t node) -> void {
    if (node == kNullNode || state.active_[node] == 0U) return;
    const Node& nd = nodes_[node];
    if (nd.point != index && (nd.point >= excluded.size() || excluded[nd.point] == 0U)) {
      const double d = dist2(query, model_->points[nd.point]);
      if (d < infinity && d <= r2) out->push_back(nd.point);
    }
    const double delta = axis_value(query, nd.axis) - nd.split;
    const std::uint32_t first = delta < 0.0 ? nd.left : nd.right;
    const std::uint32_t second = delta < 0.0 ? nd.right : nd.left;
    self(self, first);
    if (second != kNullNode) {
      const double previous = cell_distance[nd.axis];
      cell_distance[nd.axis] = delta * delta;
      // 按 dist2 的 x/y/z 顺序求和；只剪去严格超界单元，距离并列仍继续搜索。
      if (cell_distance[0] + cell_distance[1] + cell_distance[2] <= r2) self(self, second);
      cell_distance[nd.axis] = previous;
    }
  };
  search(search, root_);
}

}  // namespace mv::reconstruction
