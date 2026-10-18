#pragma once

#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

#include <core/iconic_tuple.hpp>
#include <core/region.hpp>

namespace mv::core {

/** @brief 私有 Region 对象有序集合；Public cardinality 由该容器承载。 */
class Regions {
 public:
  Regions() = default;
  Regions(std::initializer_list<Region> regions);
  explicit Regions(std::vector<Region> regions);
  explicit Regions(IconicTuple tuple);

  [[nodiscard]] bool empty() const noexcept { return tuple_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return tuple_.size(); }
  [[nodiscard]] const Region& operator[](std::size_t index) const;
  [[nodiscard]] const IconicTuple& iconic_tuple() const noexcept { return tuple_; }

 private:
  IconicTuple tuple_;
};

}  // namespace mv::core
