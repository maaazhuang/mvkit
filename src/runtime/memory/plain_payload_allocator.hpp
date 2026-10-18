#pragma once

#include <cstddef>
#include <memory>
#include <memory_resource>
#include <new>
#include <type_traits>
#include <utility>
#include <runtime/memory/memory_resource.hpp>

namespace mv::runtime {

// 点坐标是平凡可复制载荷，不是感知 allocator 的子对象；保持原 PMR 资源策略。
// 显式局部覆盖缓冲可不清零而启动 Point3d 生命周期，但暴露前须完整复制每个元素。
// 普通模型容器仍零初始化；移动赋值保留目标 allocator，覆盖模式不能离开该缓冲。
template<class T>
class PlainPayloadAllocator {
 public:
  using value_type=T;
  using propagate_on_container_copy_assignment=std::false_type;
  using propagate_on_container_move_assignment=std::false_type;
  using propagate_on_container_swap=std::false_type;
  using is_always_equal=std::false_type;

  PlainPayloadAllocator() noexcept : resource_(mv::runtime::persist_resource()) {}
  PlainPayloadAllocator(std::pmr::memory_resource* resource,bool overwrite=false) noexcept
      : resource_(resource),overwrite_(overwrite) {}
  template<class U>
  PlainPayloadAllocator(const PlainPayloadAllocator<U>& other) noexcept
      : resource_(other.resource()),overwrite_(other.overwrite_initialization()) {}

  [[nodiscard]] T* allocate(std::size_t count) {
    auto* pointer=std::pmr::polymorphic_allocator<T>{resource_}.allocate(count);
    if constexpr(std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T> &&
                 (std::is_aggregate_v<T> || std::is_scalar_v<T>)) {
      if(overwrite_) ::new (static_cast<void*>(pointer)) std::byte[count*sizeof(T)];
    }
    return pointer;
  }
  void deallocate(T* pointer,std::size_t count) noexcept {
    std::pmr::polymorphic_allocator<T>{resource_}.deallocate(pointer,count);
  }
  template<class U,class... Args>
  void construct(U* pointer,Args&&... args) {
    if constexpr(sizeof...(Args)==0 && std::is_trivially_copyable_v<U> &&
                 std::is_trivially_destructible_v<U> && (std::is_aggregate_v<U> || std::is_scalar_v<U>)) {
      // 隐式生命周期在 allocate 时一次启动，不逐元素启动；依据 C++20 P0593 https://wg21.link/p0593r6（3.2、3.8）。
      if(overwrite_) return;
    }
    std::construct_at(pointer,std::forward<Args>(args)...);
  }
  [[nodiscard]] std::pmr::memory_resource* resource() const noexcept { return resource_; }
  [[nodiscard]] bool overwrite_initialization() const noexcept { return overwrite_; }
  [[nodiscard]] PlainPayloadAllocator select_on_container_copy_construction() const noexcept {
    return PlainPayloadAllocator{mv::runtime::persist_resource()};
  }
  template<class U>
  bool operator==(const PlainPayloadAllocator<U>& other) const noexcept {
    return resource_==other.resource() || resource_->is_equal(*other.resource());
  }
  template<class U>
  bool operator!=(const PlainPayloadAllocator<U>& other) const noexcept { return !(*this==other); }

 private:
  std::pmr::memory_resource* resource_;
  bool overwrite_{false};
};

} // namespace mv::runtime
