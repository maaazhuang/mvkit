#pragma once

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <utility>
#include <variant>
#include <vector>

namespace mv::core {

// 私有同质元组存储：单元素直接驻留，多个元素沿用连续 vector。
// 短结果无需另分配载荷；移动大型结果仍直接接管原 vector。
template <class T> class TupleStorage {
 public:
  TupleStorage() : data_(std::in_place_type<std::vector<T>>) {}
  explicit TupleStorage(T value) : data_(std::in_place_type<T>, std::move(value)) {}
  explicit TupleStorage(std::size_t size) : TupleStorage() { resize(size); }
  TupleStorage(std::initializer_list<T> values) : TupleStorage(values.begin(), values.end()) {}
  explicit TupleStorage(std::vector<T> values) : TupleStorage() {
    if (values.size() == 1) data_ = std::move(values.front());
    else data_ = std::move(values);
  }
  template <class Iterator> TupleStorage(Iterator first, Iterator last) : TupleStorage() {
    if (first == last) return;
    auto next = first;
    if (++next == last) data_ = *first;
    else data_ = std::vector<T>(first, last);
  }

  [[nodiscard]] std::size_t size() const noexcept {
    if (const auto* values = std::get_if<std::vector<T>>(&data_)) return values->size();
    return 1;
  }
  [[nodiscard]] bool empty() const noexcept { return size() == 0; }
  [[nodiscard]] std::size_t max_size() const noexcept { return std::vector<T>{}.max_size(); }
  [[nodiscard]] T* data() noexcept {
    if (auto* values = std::get_if<std::vector<T>>(&data_)) return values->data();
    return &std::get<T>(data_);
  }
  [[nodiscard]] const T* data() const noexcept {
    if (const auto* values = std::get_if<std::vector<T>>(&data_)) return values->data();
    return &std::get<T>(data_);
  }
  T* begin() noexcept { return data(); }
  const T* begin() const noexcept { return data(); }
  T* end() noexcept { auto* first = data(); return empty() ? first : first + size(); }
  const T* end() const noexcept { const auto* first = data(); return empty() ? first : first + size(); }
  T& operator[](std::size_t index) noexcept { return data()[index]; }
  const T& operator[](std::size_t index) const noexcept { return data()[index]; }

  void reserve(std::size_t capacity) {
    if (capacity <= 1) return;
    if (auto* values = std::get_if<std::vector<T>>(&data_)) values->reserve(capacity);
    else {
      std::vector<T> expanded;
      expanded.reserve(capacity);
      expanded.push_back(std::move(std::get<T>(data_)));
      data_ = std::move(expanded);
    }
  }
  void push_back(T value) {
    if (auto* values = std::get_if<std::vector<T>>(&data_)) {
      if (values->empty() && values->capacity() == 0) data_ = std::move(value);
      else values->push_back(std::move(value));
    } else {
      std::vector<T> expanded;
      expanded.reserve(2);
      expanded.push_back(std::move(std::get<T>(data_)));
      expanded.push_back(std::move(value));
      data_ = std::move(expanded);
    }
  }
  template <class... Args> void emplace_back(Args&&... args) {
    push_back(T(std::forward<Args>(args)...));
  }
  void append_range(const T* first, const T* last) {
    if (first == last) return;
    if (auto* values = std::get_if<std::vector<T>>(&data_); values && values->capacity() > 0)
      values->insert(values->end(), first, last);
    else
      for (; first != last; ++first) push_back(*first);
  }
  void resize(std::size_t length) {
    if (length == size()) return;
    if (!length) data_.template emplace<std::vector<T>>();
    else if (length == 1) {
      T value = empty() ? T{} : std::move(data()[0]);
      data_ = std::move(value);
    } else {
      reserve(length);
      std::get<std::vector<T>>(data_).resize(length);
    }
  }
  friend bool operator==(const TupleStorage& left, const TupleStorage& right) {
    return left.size() == right.size() && std::equal(left.begin(), left.end(), right.begin());
  }

 private:
  std::variant<T, std::vector<T>> data_;
};

}  // namespace mv::core
