#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <memory_resource>
#include <span>
#include <string_view>
#include <vector>

#include <core/handle.hpp>
#include <core/tuple.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::runtime { class MetadataArena; }
namespace mv::core {

class Image;
class Region;
class XldContour;

enum class IconicKind : std::uint8_t {
  Image,
  Region,
  XldContour,
};

class IconicValue {
 public:
  class Payload;

  [[nodiscard]] static IconicValue from_image(Image image);
  [[nodiscard]] static IconicValue from_region(Region region);
  // 仅当 Region 句柄共享同一存储拥有者时批量包装。
  [[nodiscard]] static std::vector<IconicValue> from_regions(std::vector<Region> regions);
  [[nodiscard]] static IconicValue from_xld_contour(XldContour contour, bool pooled = false,
                                                  mv::runtime::MetadataArena* arena = nullptr);

  [[nodiscard]] IconicKind kind() const noexcept;
  [[nodiscard]] TypeId type_id() const noexcept;
  [[nodiscard]] const Image& as_image() const;
  [[nodiscard]] const Region& as_region() const;
  [[nodiscard]] const XldContour& as_xld_contour() const;
  [[nodiscard]] IconicValue clone() const;

  friend bool shares_payload(const IconicValue& lhs, const IconicValue& rhs) noexcept {
    return lhs.payload_.get() == rhs.payload_.get();
  }

 private:
  explicit IconicValue(std::shared_ptr<const Payload> payload) : payload_(std::move(payload)) {}
  std::shared_ptr<const Payload> payload_;
};

class IconicTuple {
 public:
  IconicTuple() = default;
  IconicTuple(std::initializer_list<IconicValue> values);
  explicit IconicTuple(std::vector<IconicValue> values);

  [[nodiscard]] bool empty() const noexcept { return size() == 0; }
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] const IconicValue& at(std::size_t index) const;
  [[nodiscard]] std::span<const IconicValue> values() const noexcept;

  [[nodiscard]] bool all_of_kind(IconicKind kind) const noexcept;
  void require_kind(IconicKind kind, std::string_view operation) const;
  [[nodiscard]] std::vector<Image> images(std::string_view operation) const;
  [[nodiscard]] std::vector<Region> regions(std::string_view operation) const;
  [[nodiscard]] std::vector<XldContour> xld_contours(std::string_view operation) const;

  [[nodiscard]] IconicTuple select(const Tuple& indices) const;
  [[nodiscard]] IconicTuple copy(std::int64_t index, std::int64_t num_obj) const;
  [[nodiscard]] IconicTuple concat(const IconicTuple& other) const;
  [[nodiscard]] IconicTuple clone() const;
  [[nodiscard]] bool payload_uses_persist_resource() const noexcept;

 private:
  struct Storage : std::pmr::vector<IconicValue> {
    using Base = std::pmr::vector<IconicValue>;
    using Base::Base;
    Storage(Storage&& other) noexcept : Base(std::move(other)),
        validated_kinds(other.validated_kinds.load(std::memory_order_relaxed)) {}
    // 已发布存储不可变，仅缓存成功的完整验证；拒绝视图保持首个非法对象的错误契约。
    mutable std::atomic<unsigned> validated_kinds{0};
  };
  explicit IconicTuple(Storage values)
      : storage_(values.empty()
                     ? nullptr
                     : std::make_shared<const Storage>(std::move(values))) {}
  explicit IconicTuple(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}
  std::shared_ptr<const Storage> storage_;
};

/** 构造仅含 Image 的 iconic 元组，保持对象顺序。 */
[[nodiscard]] IconicTuple make_image_tuple(std::vector<Image> images);

}  // namespace mv::core
