#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace mv::imageproc {

enum class BorderMode : std::uint8_t {
  Mirrored,
  Continued,
  Cyclic,
  Constant,
};

[[nodiscard]] inline std::optional<std::size_t> border_index(std::int64_t index,
                                                              std::size_t size,
                                                              BorderMode mode) noexcept {
  if (size == 0) return std::nullopt;
  const auto n = static_cast<std::int64_t>(size);
  if (index >= 0 && index < n) return static_cast<std::size_t>(index);
  switch (mode) {
    case BorderMode::Constant: return std::nullopt;
    case BorderMode::Continued:
      return index < 0 ? std::optional<std::size_t>{0} : std::optional<std::size_t>{size - 1};
    case BorderMode::Cyclic: {
      auto value = index % n;
      if (value < 0) value += n;
      return static_cast<std::size_t>(value);
    }
    case BorderMode::Mirrored: {
      if (size == 1) return 0;
      // 绕边界像素反射而不重复边界：-1→1，n→n-2。
      const auto period = (n - 1) * 2;
      auto value = index % period;
      if (value < 0) value += period;
      if (value >= n) value = period - value;
      return static_cast<std::size_t>(value);
    }
  }
  return std::nullopt;
}

}  // namespace mv::imageproc
