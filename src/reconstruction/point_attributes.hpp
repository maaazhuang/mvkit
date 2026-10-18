#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <runtime/memory/memory_resource.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace mv::reconstruction {
// 几何不可变；共享控制句柄原子发布点属性，读取者持有完整不可变快照。
class PointAttributes {
 public:
  using Values = std::pmr::vector<double>;
  struct State {
    using Map = std::pmr::map<std::pmr::string, Values, std::less<>>;
    Map values{std::less<>{}, runtime::persist_resource()};
    State() = default;
    State(const State& other) : values(other.values, runtime::persist_resource()) {}
  };
  using Snapshot = std::shared_ptr<const State>;
  struct Update {
    std::string_view name;
    std::span<const double> values;
  };

  PointAttributes() = default;
  PointAttributes(const PointAttributes& other) noexcept : state_(other.Read()) {}
  PointAttributes(PointAttributes&& other) noexcept : state_(other.Read()) {}
  PointAttributes& operator=(const PointAttributes& other) noexcept {
    std::atomic_store_explicit(&state_, other.Read(), std::memory_order_release);
    return *this;
  }
  PointAttributes& operator=(PointAttributes&& other) noexcept { return *this = other; }
  Snapshot Read() const noexcept { return std::atomic_load_explicit(&state_, std::memory_order_acquire); }
  static const Values* Find(const Snapshot& snapshot, std::string_view name) noexcept {
    if (!snapshot) return nullptr;
    const auto i = snapshot->values.find(name);
    return i == snapshot->values.end() ? nullptr : &i->second;
  }
  void Replace(std::span<const Update> updates) {
    auto previous = Read();
    for (;;) {
      auto next = previous ? NewState(*previous) : NewState();
      for (const auto& update : updates)
        next->values.insert_or_assign(
            std::pmr::string(update.name, runtime::persist_resource()),
            Values(update.values.begin(), update.values.end(), runtime::persist_resource()));
      Snapshot candidate = std::move(next);
      if (std::atomic_compare_exchange_weak_explicit(&state_, &previous, std::move(candidate),
                                                     std::memory_order_release, std::memory_order_acquire))
        return;
    }
  }
  PointAttributes Select(std::span<const std::size_t> indices) const { return Select(Read(), indices); }
  static PointAttributes Select(const Snapshot& source, std::span<const std::size_t> indices) {
    PointAttributes result;
    if (!source) return result;
    auto next = NewState();
    for (const auto& [name, values] : source->values) {
      Values selected(runtime::persist_resource());
      selected.reserve(indices.size());
      for (auto i : indices) selected.push_back(values.at(i));
      next->values.emplace(name, std::move(selected));
    }
    result.state_ = std::move(next);
    return result;
  }
  bool UsesPersistResource() const noexcept {
    const auto snapshot = Read();
    if (!snapshot) return true;
    const auto resource = runtime::persist_resource();
    if (snapshot->values.get_allocator().resource() != resource) return false;
    for (const auto& [name, values] : snapshot->values)
      if (name.get_allocator().resource() != resource || values.get_allocator().resource() != resource)
        return false;
    return true;
  }

 private:
  template <class... Args>
  static std::shared_ptr<State> NewState(Args&&... args) {
    return std::allocate_shared<State>(std::pmr::polymorphic_allocator<State>{runtime::persist_resource()},
                                       std::forward<Args>(args)...);
  }
  Snapshot state_;
};
}  // namespace mv::reconstruction
