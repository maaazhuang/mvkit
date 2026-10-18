#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>

namespace mv::core {

class String {
 public:
  String() = default;
  String(const char* value) : value_(value == nullptr ? "" : value) {}
  String(std::string value) : value_(std::move(value)) {}
  String(std::string_view value) : value_(value) {}

  [[nodiscard]] const char* c_str() const noexcept { return value_.c_str(); }
  [[nodiscard]] std::string_view view() const noexcept { return value_; }
  [[nodiscard]] std::size_t size() const noexcept { return value_.size(); }
  [[nodiscard]] bool empty() const noexcept { return value_.empty(); }

  friend bool operator==(const String&, const String&) = default;

 private:
  std::string value_;
};

}  // namespace mv::core
