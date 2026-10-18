#pragma once

#include <cstddef>
#include <array>
#include <initializer_list>
#include <string_view>
#include <utility>
#include <vector>

namespace mv::core {

struct BroadcastInput {
  std::size_t size{0};
  std::string_view parameter;
  bool allow_scalar{true};
};

/** Tuple 算子共享已验证的 scalar-or-N 广播映射；默认拒绝空元组。
 * 有特殊空语义的算子须在构造计划前处理该语义。 */
class BroadcastPlan final {
 public:
  [[nodiscard]] static BroadcastPlan make(
      std::initializer_list<BroadcastInput> inputs,
      std::string_view operation);

  [[nodiscard]] std::size_t size() const noexcept { return output_size_; }
  [[nodiscard]] std::size_t argument_count() const noexcept { return input_count_; }
  [[nodiscard]] std::size_t source_index(std::size_t argument, std::size_t output_index) const {
    // 索引映射是逐坐标热路径，异常构造保持在独立冷路径。
    if (argument >= input_count_) throw_source_index(true);
    if (output_index >= output_size_) throw_source_index(false);
    const auto size = input_count_ <= inline_sizes_.size() ? inline_sizes_[argument] : input_sizes_[argument];
    return size == 1 ? 0 : output_index;
  }

 private:
  BroadcastPlan() = default;
  [[noreturn]] static void throw_source_index(bool argument);
  std::array<std::size_t,8> inline_sizes_{};
  std::size_t input_count_{0};
  std::vector<std::size_t> input_sizes_;
  std::size_t output_size_{0};
};

}  // namespace mv::core
