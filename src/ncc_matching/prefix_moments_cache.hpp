#pragma once

#include "model.hpp"
#include <runtime/memory/memory_resource.hpp>
#include <runtime/memory/overwrite_buffer.hpp>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <memory_resource>
#include <new>
#include <span>
#include <vector>

namespace mv::ncc_matching {

// 可选查询局部晶格窗口和；调用方确认像素类型、验证前缀窗口并保持图像不可变。
// 重建/复用图像缓冲前先失效，即使地址/尺寸不变；不保存 Patch 指针或临时游程视图。
// 相同有序游程的和不受权重、原点、角度影响；缓存不提供同步，每个查询独占写平面。
class PrefixMomentsCache {
public:
  static constexpr std::size_t byte_budget = 64 * 1024 * 1024;
  static constexpr std::size_t owner_bytes = 512;
  static constexpr std::size_t max_entries = 256;

  struct Entry {
    runtime::OverwriteBuffer<double> sum;
    runtime::OverwriteBuffer<double> squared;
    std::pmr::vector<SampleRun> runs;
    const double* pixels;
    std::size_t width, height, sample_count, plane_elements, fingerprint;
    std::uint64_t epoch;
    bool ready = false;
    Entry* next = nullptr;

    Entry(const double* image, std::size_t w, std::size_t h, std::size_t count,
        std::size_t hash, std::uint64_t generation,
        std::span<const SampleRun> geometry, std::pmr::memory_resource* memory)
        : sum(0, memory), squared(0, memory),
          runs(geometry.begin(), geometry.end(), memory),
          pixels(image), width(w), height(h), sample_count(count),
          plane_elements(w * h), fingerprint(hash), epoch(generation) {}
    Entry(const Entry&) = delete;
    Entry& operator=(const Entry&) = delete;
  };

private:
  static_assert(sizeof(Entry) <= owner_bytes);
  // 成功请求永久计入预算，部分 entry/plane 构造失败也如此；释放不能返还查询额度，
  // 上游 arena 可能保留内存至查询结束。计量为请求存储，不是 arena 块大小或 RSS 峰值。
  class BoundedResource final : public std::pmr::memory_resource {
    std::pmr::memory_resource* upstream_;
    std::size_t limit_, used_ = 0;
    void* do_allocate(std::size_t bytes, std::size_t alignment) override {
      if (bytes > limit_ - used_) throw std::bad_alloc();
      auto* result = upstream_->allocate(bytes, alignment);
      used_ += bytes;
      return result;
    }
    void do_deallocate(void* pointer, std::size_t bytes,
        std::size_t alignment) override {
      upstream_->deallocate(pointer, bytes, alignment);
    }
    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override {
      return this == &other;
    }
  public:
    BoundedResource(std::pmr::memory_resource* memory, std::size_t limit)
        : upstream_(memory ? memory : runtime::default_resource()),
          limit_(std::min(limit, byte_budget)) {}
    std::size_t used() const noexcept { return used_; }
    std::size_t remaining() const noexcept { return limit_ - used_; }
    std::size_t limit() const noexcept { return limit_; }
  };

  BoundedResource resource_;
  Entry* entries_ = nullptr;
  std::uint64_t epoch_ = 0;
  bool disabled_ = false;
  std::size_t size_ = 0, requests_ = 0, hits_ = 0, ready_hits_ = 0;
  std::size_t materializations_ = 0;

  static bool valid_key(const double* pixels, std::size_t width,
      std::size_t height, const Patch& patch) noexcept {
    if (!pixels || !width || !height || patch.samples.empty() || patch.runs.empty())
      return false;
    if (width > std::numeric_limits<std::size_t>::max() / height) return false;
    const auto count = width * height;
    if (count > byte_budget / (2 * sizeof(double))) return false;
    const auto payload = count * (2 * sizeof(double));
    if (payload > byte_budget - owner_bytes ||
        patch.runs.size() > (byte_budget - owner_bytes - payload) / sizeof(SampleRun))
      return false;
    return true;
  }

  std::size_t fingerprint(const double* pixels, std::size_t width,
      std::size_t height, const Patch& patch) const noexcept {
    std::size_t result = std::hash<const double*>{}(pixels);
    const auto mix = [&](std::size_t value) {
      result ^= value + std::size_t(0x9e3779b9) + (result << 6) + (result >> 2);
    };
    mix(width); mix(height); mix(patch.samples.size());
    mix(std::hash<std::uint64_t>{}(epoch_));
    for (const auto& run : patch.runs) {
      mix(std::hash<std::int32_t>{}(run.row));
      mix(std::hash<std::int32_t>{}(run.begin));
      mix(std::hash<std::int32_t>{}(run.end));
    }
    return result;
  }

  Entry* lookup(const double* pixels, std::size_t width, std::size_t height,
      const Patch& patch, std::size_t hash) const noexcept {
    for (auto* entry = entries_; entry; entry = entry->next) {
      if (entry->epoch != epoch_ || entry->fingerprint != hash ||
          entry->pixels != pixels || entry->width != width || entry->height != height ||
          entry->sample_count != patch.samples.size() || entry->runs.size() != patch.runs.size())
        continue;
      bool equal = true;
      for (std::size_t i = 0; i < patch.runs.size(); ++i) {
        const auto& a = entry->runs[i];
        const auto& b = patch.runs[i];
        if (a.row != b.row || a.begin != b.begin || a.end != b.end) {
          equal = false;
          break;
        }
      }
      if (equal) return entry;
    }
    return nullptr;
  }

  Entry* materialize(Entry& entry) {
    const auto bytes = entry.plane_elements * sizeof(double);
    const auto missing = (entry.sum.size() ? 0 : bytes) +
        (entry.squared.size() ? 0 : bytes);
    if (missing > resource_.remaining()) return nullptr;
    try {
      const auto nan = std::numeric_limits<double>::quiet_NaN();
      if (!entry.sum.size()) {
        entry.sum.reset_for_overwrite(entry.plane_elements);
        std::fill_n(entry.sum.data(), entry.plane_elements, nan);
      }
      if (!entry.squared.size()) {
        entry.squared.reset_for_overwrite(entry.plane_elements);
        std::fill_n(entry.squared.data(), entry.plane_elements, nan);
      }
      entry.ready = true;
      ++materializations_;
      return &entry;
    } catch (const std::bad_alloc&) {
      // 首平面成功后保持 pending 并计费；后续精确命中可完成 entry，不再次分配或计费该平面。
      return nullptr;
    }
  }

public:
  explicit PrefixMomentsCache(std::pmr::memory_resource* memory,
      std::size_t limit = byte_budget) : resource_(memory, limit) {}
  PrefixMomentsCache(const PrefixMomentsCache&) = delete;
  PrefixMomentsCache& operator=(const PrefixMomentsCache&) = delete;
  ~PrefixMomentsCache() {
    while (entries_) {
      auto* entry = entries_;
      entries_ = entry->next;
      std::destroy_at(entry);
      resource_.deallocate(entry, owner_bytes, alignof(Entry));
    }
  }

  // 首次遇到仅存元数据/拥有型游程并返回 nullptr；第二次精确命中物化两个完整 W*H NaN 平面。
  // 之后即使预算满也返回同一 ready Entry，索引始终 r*width+c，不裁剪或截断平面。
  Entry* get(const double* pixels, std::size_t width, std::size_t height,
      const Patch& patch) {
    ++requests_;
    if (disabled_ || !valid_key(pixels, width, height, patch)) return nullptr;
    const auto hash = fingerprint(pixels, width, height, patch);
    if (auto* entry = lookup(pixels, width, height, patch, hash)) {
      ++hits_;
      if (entry->ready) { ++ready_hits_; return entry; }
      return materialize(*entry);
    }
    // 即使每模型掩码不同，也限制几何查找工作；仅限制缓存元数据，仍评估全部窗口。
    if (size_ >= max_entries) return nullptr;
    const auto metadata = owner_bytes + patch.runs.size() * sizeof(SampleRun);
    if (metadata > resource_.remaining()) return nullptr;
    void* owner = nullptr;
    try {
      owner = resource_.allocate(owner_bytes, alignof(Entry));
      auto* entry = std::construct_at(static_cast<Entry*>(owner), pixels, width,
          height, patch.samples.size(), hash, epoch_,
          std::span<const SampleRun>{patch.runs.data(), patch.runs.size()}, &resource_);
      entry->next = entries_;
      entries_ = entry;
      ++size_;
    } catch (const std::bad_alloc&) {
      if (owner) resource_.deallocate(owner, owner_bytes, alignof(Entry));
    }
    return nullptr;
  }

  const Entry* find(const double* pixels, std::size_t width, std::size_t height,
      const Patch& patch) const noexcept {
    if (disabled_ || !valid_key(pixels, width, height, patch)) return nullptr;
    const auto* entry = lookup(pixels, width, height, patch,
        fingerprint(pixels, width, height, patch));
    return entry && entry->ready ? entry : nullptr;
  }

  void invalidate() noexcept {
    if (epoch_ == std::numeric_limits<std::uint64_t>::max()) disabled_ = true;
    else ++epoch_;
  }
  // size 含冷态/pending/旧 epoch 项；hits 计精确几何命中，含物化或失败命中。
  std::size_t size() const noexcept { return size_; }
  std::size_t requests() const noexcept { return requests_; }
  std::size_t hits() const noexcept { return hits_; }
  std::size_t ready_hits() const noexcept { return ready_hits_; }
  std::size_t materializations() const noexcept { return materializations_; }
  std::size_t allocation_bytes() const noexcept { return resource_.used(); }
  std::size_t allocation_limit() const noexcept { return resource_.limit(); }
  std::uint64_t epoch() const noexcept { return epoch_; }
};

} // namespace mv::ncc_matching
