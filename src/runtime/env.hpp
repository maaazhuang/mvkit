#pragma once

#include <cstdlib>

namespace mv::runtime {

// 可移植 getenv；MSVC 建议 _dupenv_s 并报告 C4996，仅在此包装处抑制，不全局关闭。
[[nodiscard]] inline const char* getenv_or_null(const char* name) noexcept {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  const char* value = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
  return value;
}

}  // namespace mv::runtime
