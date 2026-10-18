#pragma once

#include "runtime/memory/memory_resource.hpp"

#include <iterator>
#include <memory_resource>
#include <span>

namespace mv::runtime {

// Persistent payload 使用 persist_resource() 作为 allocator。从 scratch/arena
// 产生的容器必须 assign/copy 到 persist 存储，禁止把 arena allocator 直接 move
// 进长期对象。
//
// 已接入的持久 payload 包括 Image plane、Region runs、XLD 点/属性、
// IconicTuple 对象表、ShapeModel 点和 ObjectModel3D 点云。控制块、短字符串、
// Public/C ABI 适配容器和第三方库内部对象保留系统分配。

template <class T>
[[nodiscard]] std::pmr::vector<T> persist_vector() {
  return std::pmr::vector<T>(persist_resource());
}

template <class T>
[[nodiscard]] std::pmr::vector<T> persist_copy(const std::pmr::vector<T>& source) {
  return std::pmr::vector<T>(source, persist_resource());
}

template <class T, class Range>
void assign_persist(std::pmr::vector<T>& destination, const Range& source) {
  destination.assign(std::begin(source), std::end(source));
}

template <class T>
[[nodiscard]] std::pmr::vector<T> copy_to_persist(std::span<const T> source) {
  std::pmr::vector<T> destination(persist_resource());
  destination.assign(source.begin(), source.end());
  return destination;
}

}  // namespace mv::runtime
