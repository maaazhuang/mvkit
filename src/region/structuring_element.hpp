#pragma once

#include <cstdint>
#include <vector>

namespace mv::regionproc {

struct StructuringOffset { std::int64_t row{0}; std::int64_t column{0}; };
class StructuringElement {
 public:
  explicit StructuringElement(std::vector<StructuringOffset> offsets) : offsets_(std::move(offsets)) {}
  [[nodiscard]] const std::vector<StructuringOffset>& offsets() const noexcept { return offsets_; }
 private:
  std::vector<StructuringOffset> offsets_;
};

}  // namespace mv::regionproc
