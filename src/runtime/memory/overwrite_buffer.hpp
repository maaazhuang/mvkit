#pragma once
#include <cstddef>
#include <memory_resource>
#include <span>
#include <type_traits>
#include <utility>

namespace mv::runtime {
// 平凡标量工作区的每个元素须在首次读取前完整写入；不用于掩码、累加器或部分填充输出。
template<class T> class OverwriteBuffer {
  static_assert(std::is_trivial_v<T>);
 public:
  OverwriteBuffer(std::size_t count,std::pmr::memory_resource* resource)
      :allocator_(resource),count_(count),data_(count ? allocator_.allocate(count) : nullptr) {}
  ~OverwriteBuffer(){if(data_)allocator_.deallocate(data_,count_);}
  OverwriteBuffer(const OverwriteBuffer&)=delete;
  OverwriteBuffer& operator=(const OverwriteBuffer&)=delete;
  OverwriteBuffer(OverwriteBuffer&& other) noexcept
      :allocator_(other.allocator_),count_(std::exchange(other.count_,0)),data_(std::exchange(other.data_,nullptr)) {}
  void reset_for_overwrite(std::size_t count) {
    T* next=count?allocator_.allocate(count):nullptr;
    if(data_)allocator_.deallocate(data_,count_);
    data_=next;count_=count;
  }
  T* data() noexcept{return data_;}
  const T* data() const noexcept{return data_;}
  std::size_t size() const noexcept{return count_;}
  T& operator[](std::size_t i) noexcept{return data_[i];}
  const T& operator[](std::size_t i) const noexcept{return data_[i];}
  operator std::span<T>() noexcept{return {data_,count_};}
 private:
  std::pmr::polymorphic_allocator<T> allocator_;
  std::size_t count_;
  T* data_;
};
}
