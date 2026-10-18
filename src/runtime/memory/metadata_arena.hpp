#pragma once
#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <memory_resource>
#include <new>
#include <runtime/memory/memory_resource.hpp>

namespace mv::runtime {
// 发布前串行分配；控制块可在其他线程独立释放。只有有界内联点共享 arena，大点数组保持独立所有权。
class MetadataArena {
 public:
  struct Release { void operator()(MetadataArena* p) const noexcept { if(p)p->release(); } };
  using Owner=std::unique_ptr<MetadataArena,Release>;
  static Owner create() {
    // 池大小类别须包含 arena 头，不能只计算缓冲区。
    static_assert(sizeof(MetadataArena)<=65536);
    auto* resource=persist_resource();
    auto* bytes=resource->allocate(sizeof(MetadataArena),alignof(MetadataArena));
    try { return Owner{::new(bytes) MetadataArena{resource}}; }
    catch(...) {resource->deallocate(bytes,sizeof(MetadataArena),alignof(MetadataArena));throw;}
  }
  void* allocate(std::size_t bytes,std::size_t alignment) {
    auto* p=memory_.allocate(bytes,alignment);
    references_.fetch_add(1,std::memory_order_relaxed);
    return p;
  }
  void release() noexcept {
    if(references_.fetch_sub(1,std::memory_order_acq_rel)==1) {
      auto* resource=resource_;
      this->~MetadataArena();
      resource->deallocate(this,sizeof(MetadataArena),alignof(MetadataArena));
    }
  }
 private:
  explicit MetadataArena(std::pmr::memory_resource* resource)
      :resource_(resource),memory_(buffer_.data(),buffer_.size(),resource) {}
  std::atomic<std::size_t> references_{1};
  std::pmr::memory_resource* resource_;
  alignas(std::max_align_t) std::array<std::byte,65536-256> buffer_;
  std::pmr::monotonic_buffer_resource memory_;
};
template<class T> class MetadataAllocator {
 public:
  using value_type=T;
  explicit MetadataAllocator(MetadataArena* owner) noexcept:owner_(owner) {}
  template<class U> MetadataAllocator(const MetadataAllocator<U>& other) noexcept:owner_(other.owner()) {}
  T* allocate(std::size_t n) {
    if(n>std::numeric_limits<std::size_t>::max()/sizeof(T))throw std::bad_array_new_length{};
    return static_cast<T*>(owner_->allocate(n*sizeof(T),alignof(T)));
  }
  void deallocate(T*,std::size_t) noexcept {owner_->release();}
  MetadataArena* owner() const noexcept {return owner_;}
  template<class U> bool operator==(const MetadataAllocator<U>& other) const noexcept {return owner_==other.owner();}
 private:
  MetadataArena* owner_;
};
}
