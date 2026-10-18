#include <core/iconic_tuple.hpp>

#include <algorithm>
#include <iterator>
#include <utility>

#include <core/exception.hpp>
#include <core/image.hpp>
#include <core/region.hpp>
#include <core/xld.hpp>
#include <runtime/memory/metadata_arena.hpp>

namespace mv::core {
namespace {

constexpr TypeId kImageTypeId = make_type_id("mvkit.iconic.Image");
constexpr TypeId kRegionTypeId = make_type_id("mvkit.iconic.Region");
constexpr TypeId kXldContourTypeId = make_type_id("mvkit.iconic.XldContour");

constexpr unsigned kind_bit(IconicKind kind) noexcept {
  switch (kind) {
    case IconicKind::Image: return 1;
    case IconicKind::Region: return 2;
    case IconicKind::XldContour: return 4;
    default: return 0;
  }
}

[[noreturn]] void throw_wrong_kind(std::string_view operation, IconicKind expected) {
  const char* name = expected == IconicKind::Image
                         ? "Image"
                         : expected == IconicKind::Region ? "Region" : "XldContour";
  throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                  "Objects", -1, std::string{"iconic object is not "} + name);
}

}  // namespace

class IconicValue::Payload {
 public:
  virtual ~Payload() = default;
  [[nodiscard]] virtual IconicKind kind() const noexcept = 0;
  [[nodiscard]] virtual TypeId type_id() const noexcept = 0;
  [[nodiscard]] virtual const Image* image() const noexcept { return nullptr; }
  [[nodiscard]] virtual const Region* region() const noexcept { return nullptr; }
  [[nodiscard]] virtual const XldContour* xld_contour() const noexcept { return nullptr; }
  [[nodiscard]] virtual std::shared_ptr<const Payload> clone() const = 0;
};

namespace {

class ImagePayload final : public IconicValue::Payload {
 public:
  explicit ImagePayload(Image value) : value_(std::move(value)) {}
  [[nodiscard]] IconicKind kind() const noexcept override { return IconicKind::Image; }
  [[nodiscard]] TypeId type_id() const noexcept override { return kImageTypeId; }
  [[nodiscard]] const Image* image() const noexcept override { return &value_; }
  [[nodiscard]] std::shared_ptr<const Payload> clone() const override {
    return std::make_shared<ImagePayload>(value_.clone());
  }

 private:
  Image value_;
};

class RegionPayload final : public IconicValue::Payload {
 public:
  explicit RegionPayload(Region value) : value_(std::move(value)) {}
  [[nodiscard]] IconicKind kind() const noexcept override { return IconicKind::Region; }
  [[nodiscard]] TypeId type_id() const noexcept override { return kRegionTypeId; }
  [[nodiscard]] const Region* region() const noexcept override { return &value_; }
  [[nodiscard]] std::shared_ptr<const Payload> clone() const override {
    return std::make_shared<RegionPayload>(value_);
  }

 private:
  Region value_;
};

class XldContourPayload final : public IconicValue::Payload {
 public:
  explicit XldContourPayload(XldContour value) : value_(std::move(value)) {}
  [[nodiscard]] IconicKind kind() const noexcept override { return IconicKind::XldContour; }
  [[nodiscard]] TypeId type_id() const noexcept override { return kXldContourTypeId; }
  [[nodiscard]] const XldContour* xld_contour() const noexcept override { return &value_; }
  [[nodiscard]] std::shared_ptr<const Payload> clone() const override {
    return std::make_shared<XldContourPayload>(value_);
  }

 private:
  XldContour value_;
};

}  // namespace

IconicValue IconicValue::from_image(Image image) {
  if (image.empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "IconicValue::from_image", "image", -1,
                    "an uninitialized Image cannot be an iconic object");
  }
  return IconicValue{std::make_shared<ImagePayload>(std::move(image))};
}

IconicValue IconicValue::from_region(Region region) {
  return IconicValue{std::make_shared<RegionPayload>(std::move(region))};
}

std::vector<IconicValue> IconicValue::from_regions(std::vector<Region> regions) {
  std::vector<IconicValue> values;
  values.reserve(regions.size());
  for(std::size_t first=0;first<regions.size();) {
    auto last=first+1;
    while(last<regions.size() && last-first<128 &&
          regions[first].shares_storage_owner(regions[last]))++last;
    if(last==first+1)values.push_back(from_region(std::move(regions[first])));
    else {
      auto batch=std::make_shared<std::vector<RegionPayload>>();
      batch->reserve(last-first);
      for(auto i=first;i<last;++i)batch->emplace_back(std::move(regions[i]));
      // 填充后发布，不允许重分配使别名失效；包装引用仍持有同一 Region 存储拥有者。
      for(const auto& payload:*batch)
        values.push_back(IconicValue{std::shared_ptr<const Payload>{batch,&payload}});
    }
    first=last;
  }
  return values;
}

IconicValue IconicValue::from_xld_contour(XldContour contour, bool pooled,mv::runtime::MetadataArena* arena) {
  if(!pooled)return IconicValue{std::make_shared<XldContourPayload>(std::move(contour))};
  if(arena)return IconicValue{std::allocate_shared<XldContourPayload>(
      mv::runtime::MetadataAllocator<XldContourPayload>{arena},std::move(contour))};
  return IconicValue{std::allocate_shared<XldContourPayload>(
      std::pmr::polymorphic_allocator<XldContourPayload>{mv::runtime::persist_resource()},
      std::move(contour))};
}

IconicKind IconicValue::kind() const noexcept { return payload_->kind(); }
TypeId IconicValue::type_id() const noexcept { return payload_->type_id(); }

const Image& IconicValue::as_image() const {
  if (const Image* value = payload_->image()) return *value;
  throw_wrong_kind("IconicValue::as_image", IconicKind::Image);
}

const Region& IconicValue::as_region() const {
  if (const Region* value = payload_->region()) return *value;
  throw_wrong_kind("IconicValue::as_region", IconicKind::Region);
}

const XldContour& IconicValue::as_xld_contour() const {
  if (const XldContour* value = payload_->xld_contour()) return *value;
  throw_wrong_kind("IconicValue::as_xld_contour", IconicKind::XldContour);
}

IconicValue IconicValue::clone() const { return IconicValue{payload_->clone()}; }

IconicTuple::IconicTuple(std::initializer_list<IconicValue> values)
    : storage_(values.size() == 0
                   ? nullptr
                   : std::make_shared<const Storage>(
                         values, mv::runtime::persist_resource())) {}

IconicTuple::IconicTuple(std::vector<IconicValue> values) {
  if (values.empty()) return;
  auto storage = std::make_shared<Storage>(mv::runtime::persist_resource());
  storage->reserve(values.size());
  std::move(values.begin(), values.end(), std::back_inserter(*storage));
  storage_ = std::move(storage);
}

std::size_t IconicTuple::size() const noexcept { return storage_ ? storage_->size() : 0; }

const IconicValue& IconicTuple::at(std::size_t index) const {
  if (index >= size()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                    "IconicTuple::at", "index", static_cast<std::int64_t>(index),
                    "iconic tuple index is out of range");
  }
  return (*storage_)[index];
}

std::span<const IconicValue> IconicTuple::values() const noexcept {
  return storage_ ? std::span<const IconicValue>{*storage_} : std::span<const IconicValue>{};
}

bool IconicTuple::all_of_kind(IconicKind kind) const noexcept {
  if (!storage_) return true;
  const unsigned bit = kind_bit(kind);
  if (storage_->validated_kinds.load(std::memory_order_relaxed) & bit) return true;
  const bool result = std::all_of(values().begin(), values().end(),
                     [kind](const IconicValue& value) { return value.kind() == kind; });
  if (result) storage_->validated_kinds.fetch_or(bit, std::memory_order_relaxed);
  return result;
}

void IconicTuple::require_kind(IconicKind kind, std::string_view operation) const {
  if (!storage_) return;
  const unsigned bit = kind_bit(kind);
  if (storage_->validated_kinds.load(std::memory_order_relaxed) & bit) return;
  for (std::size_t index = 0; index < size(); ++index) {
    if (at(index).kind() != kind) {
      throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                      "Objects", static_cast<std::int64_t>(index),
                      "iconic tuple contains an incompatible object kind");
    }
  }
  storage_->validated_kinds.fetch_or(bit, std::memory_order_relaxed);
}

std::vector<Image> IconicTuple::images(std::string_view operation) const {
  require_kind(IconicKind::Image, operation);
  std::vector<Image> output;
  output.reserve(size());
  for (const IconicValue& value : values()) output.push_back(value.as_image());
  return output;
}

std::vector<Region> IconicTuple::regions(std::string_view operation) const {
  require_kind(IconicKind::Region, operation);
  std::vector<Region> output;
  output.reserve(size());
  for (const IconicValue& value : values()) output.push_back(value.as_region());
  return output;
}

std::vector<XldContour> IconicTuple::xld_contours(std::string_view operation) const {
  require_kind(IconicKind::XldContour, operation);
  std::vector<XldContour> output;
  output.reserve(size());
  for (const IconicValue& value : values()) output.push_back(value.as_xld_contour());
  return output;
}

IconicTuple IconicTuple::select(const Tuple& indices) const {
  constexpr std::string_view operation = "select_obj";
  if (empty() || indices.empty()) return {};
  Storage selected{mv::runtime::persist_resource()};
  selected.reserve(indices.size());
  for (std::size_t tuple_index = 0; tuple_index < indices.size(); ++tuple_index) {
    const Value value = indices.at(tuple_index);
    if (value.kind() != ValueKind::Int64) {
      throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation,
                      "Index", static_cast<std::int64_t>(tuple_index),
                      "object index must be int64");
    }
    const std::int64_t object_index = value.as_int64();
    if (object_index < 1 || static_cast<std::uint64_t>(object_index) > size()) {
      throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, operation,
                      "Index", static_cast<std::int64_t>(tuple_index),
                      "object index is outside the 1-based tuple range");
    }
    selected.push_back(at(static_cast<std::size_t>(object_index - 1)));
  }
  return IconicTuple{std::move(selected)};
}

IconicTuple IconicTuple::copy(std::int64_t index, std::int64_t num_obj) const {
  constexpr std::string_view operation = "copy_obj";
  if (empty()) return {};
  if (index < 1 || static_cast<std::uint64_t>(index) > size()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, operation,
                    "Index", -1, "copy start is outside the 1-based tuple range");
  }
  if (num_obj == 0) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, operation,
                    "NumObj", -1, "NumObj must be positive or negative for copy-to-end");
  }
  const std::size_t begin = static_cast<std::size_t>(index - 1);
  const std::size_t available = size() - begin;
  const std::size_t count = num_obj < 0
                                ? available
                                : std::min(available, static_cast<std::size_t>(num_obj));
  Storage copied{mv::runtime::persist_resource()};
  copied.reserve(count);
  copied.insert(copied.end(), storage_->begin() + static_cast<std::ptrdiff_t>(begin),
                storage_->begin() + static_cast<std::ptrdiff_t>(begin + count));
  return IconicTuple{std::move(copied)};
}

IconicTuple IconicTuple::concat(const IconicTuple& other) const {
  if (empty()) return other;
  if (other.empty()) return *this;
  Storage result{mv::runtime::persist_resource()};
  result.reserve(size() + other.size());
  result.insert(result.end(), storage_->begin(), storage_->end());
  result.insert(result.end(), other.storage_->begin(), other.storage_->end());
  return IconicTuple{std::move(result)};
}

IconicTuple IconicTuple::clone() const {
  Storage result{mv::runtime::persist_resource()};
  result.reserve(size());
  for (const IconicValue& value : values()) result.push_back(value.clone());
  return IconicTuple{std::move(result)};
}

bool IconicTuple::payload_uses_persist_resource() const noexcept {
  return !storage_ || storage_->get_allocator().resource() ==
                          mv::runtime::persist_resource();
}

IconicTuple make_image_tuple(std::vector<Image> images) {
  if (images.size() == 1) return IconicTuple{IconicValue::from_image(std::move(images.front()))};
  std::vector<IconicValue> values;
  values.reserve(images.size());
  for (auto& image : images) values.push_back(IconicValue::from_image(std::move(image)));
  return IconicTuple{std::move(values)};
}

}  // namespace mv::core
