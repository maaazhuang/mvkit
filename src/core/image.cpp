#include <core/image.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory_resource>
#include <mutex>
#include <vector>

#include <core/exception.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::core {


namespace {

void validate_dimensions(std::size_t width, std::size_t height) {
  if (width == 0) {
    throw Exception(Status::InvalidArgument, "Image", "width must be greater than zero");
  }
  if (height == 0) {
    throw Exception(Status::InvalidArgument, "Image", "height must be greater than zero");
  }
}

std::size_t checked_row_bytes(std::size_t width, PixelType type) {
  const auto bytes = bytes_per_sample(type);
  if (width > std::numeric_limits<std::size_t>::max() / bytes) {
    throw Exception(Status::OutOfRange, "Image", "row size overflows size_t");
  }
  return width * bytes;
}


}  // namespace

[[noreturn]] void Image::throw_channel_out_of_range(std::size_t channel) {
  throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                  "Image::plane", "channel", static_cast<std::int64_t>(channel),
                  "channel index is out of range");
}

Image Image::create(std::size_t width, std::size_t height, PixelType type) {
  return create(width, height, std::span<const PixelType>{&type, 1});
}

Image Image::create(std::size_t width, std::size_t height,
                   std::span<const PixelType> channel_types) {
  validate_dimensions(width, height);
  if (channel_types.empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    "Image::create", "channel_types", -1,
                    "an image must contain at least one channel");
  }

  auto storage = std::make_shared<Storage>();
  storage->width = width;
  storage->height = height;
  storage->planes.reserve(channel_types.size());
  for (const PixelType type : channel_types) {
    const auto row_bytes = checked_row_bytes(width, type);
    if (height > std::numeric_limits<std::size_t>::max() / row_bytes) {
      throw Exception(Status::OutOfRange, "Image::create",
                      "image plane allocation size overflows size_t");
    }
    Storage::Plane plane;
    plane.pixel_type = type;
    plane.stride = row_bytes;
    plane.owned.resize(row_bytes * height);
    storage->planes.push_back(std::move(plane));
  }
  return Image{std::move(storage)};
}

Image Image::create_uninitialized(std::size_t width, std::size_t height,
                                  std::span<const PixelType> channel_types) {
  validate_dimensions(width, height);
  if (channel_types.empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    "Image::create_uninitialized", "channel_types", -1,
                    "an image must contain at least one channel");
  }
  constexpr std::size_t alignment = alignof(std::max_align_t);
  std::vector<ImagePlaneSpec> planes;
  planes.reserve(channel_types.size());
  for (PixelType type : channel_types) {
    const auto row_bytes = checked_row_bytes(width, type);
    if (height > std::numeric_limits<std::size_t>::max() / row_bytes)
      throw Exception(Status::OutOfRange, "Image::create_uninitialized",
                      "image plane allocation size overflows size_t");
    const auto size = row_bytes * height;
    void* data = mv::runtime::allocate_bytes(size, alignment);
    std::shared_ptr<void> owner(data, [size](void* pointer) noexcept {
      mv::runtime::deallocate_bytes(pointer, size, alignment);
    });
    planes.push_back({type, data, row_bytes, std::move(owner)});
  }
  return wrap_planes(width, height, planes);
}

Image Image::wrap(std::size_t width, std::size_t height, PixelType type,
                  void* data, std::size_t stride, std::shared_ptr<void> owner) {
  const ImagePlaneSpec plane{type, data, stride, std::move(owner)};
  return wrap_planes(width, height, std::span<const ImagePlaneSpec>{&plane, 1});
}

ImagePlaneSpec Image::shared_plane(std::size_t channel) const {
  if(!storage_ || channel>=storage_->planes.size())throw_channel_out_of_range(channel);
  const auto& plane=storage_->planes[channel];
  return {plane.pixel_type,const_cast<void*>(plane.data()),plane.stride,
          std::shared_ptr<void>(storage_,storage_.get())};
}

Image Image::channel_view(std::size_t channel) const {
  if(!storage_ || channel>=storage_->planes.size())throw_channel_out_of_range(channel);
  if(storage_->planes.size()==1)return *this;
  auto storage=std::make_shared<Storage>();
  storage->width=storage_->width;storage->height=storage_->height;
  storage->planes.reserve(1);
  const auto spec=shared_plane(channel);
  Storage::Plane plane;plane.pixel_type=spec.pixel_type;plane.stride=spec.stride;
  plane.external_data=spec.data;plane.external_owner=spec.owner;
  storage->planes.push_back(std::move(plane));
  return Image{std::move(storage),domain_};
}

std::array<Image,3> Image::channel_views3() const {
  if(channel_count()!=3)
    throw Exception(Status::InvalidArgument,ErrorCode::InvalidCardinality,
                    "Image::channel_views3","image",-1,"exactly three channels required");
  auto bundle=std::make_shared<std::array<Storage,3>>();
  std::array<Image,3> result;
  for(std::size_t channel=0;channel<3;++channel) {
    auto& view=(*bundle)[channel];
    view.width=storage_->width;view.height=storage_->height;
    view.planes.resize(1);
    const auto& source=storage_->planes[channel];
    auto& plane=view.planes[0];
    plane.pixel_type=source.pixel_type;plane.stride=source.stride;
    plane.external_data=const_cast<void*>(source.data());
    plane.external_owner=std::shared_ptr<void>(storage_,storage_.get());
    result[channel]=Image{std::shared_ptr<Storage>(bundle,&view),domain_};
  }
  return result;
}

Image Image::wrap_planes(std::size_t width, std::size_t height,
                         std::span<const ImagePlaneSpec> planes) {
  validate_dimensions(width, height);
  if (planes.empty()) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    "Image::wrap_planes", "planes", -1,
                    "an image must contain at least one channel");
  }

  auto storage = std::make_shared<Storage>();
  storage->width = width;
  storage->height = height;
  storage->planes.reserve(planes.size());
  for (std::size_t index = 0; index < planes.size(); ++index) {
    const auto& input = planes[index];
    const auto row_bytes = checked_row_bytes(width, input.pixel_type);
    if (input.data == nullptr) {
      throw Exception(Status::InvalidArgument, ErrorCode::NullArgument,
                      "Image::wrap_planes", "planes", static_cast<std::int64_t>(index),
                      "plane data must not be null");
    }
    if (!input.owner) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                      "Image::wrap_planes", "planes", static_cast<std::int64_t>(index),
                      "plane owner token must not be empty");
    }
    if (input.stride < row_bytes) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                      "Image::wrap_planes", "planes", static_cast<std::int64_t>(index),
                      "plane stride is smaller than its row size");
    }
    Storage::Plane plane;
    plane.pixel_type = input.pixel_type;
    plane.stride = input.stride;
    plane.external_data = input.data;
    plane.external_owner = input.owner;
    storage->planes.push_back(std::move(plane));
  }
  return Image{std::move(storage)};
}

Image Image::import_packed_rgb(std::size_t width, std::size_t height,
                               ChannelOrder order, const void* data,
                               std::size_t stride) {
  validate_dimensions(width, height);
  if (data == nullptr) {
    throw Exception(Status::InvalidArgument, ErrorCode::NullArgument,
                    "Image::import_packed_rgb", "data", -1,
                    "packed data must not be null");
  }
  if (width > std::numeric_limits<std::size_t>::max() / 3U || stride < width * 3U) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "Image::import_packed_rgb", "stride", -1,
                    "packed stride is smaller than the RGB row size");
  }

  const PixelType types[]{PixelType::UInt8, PixelType::UInt8, PixelType::UInt8};
  Image output = create(width, height, types);
  const auto* source = static_cast<const std::uint8_t*>(data);
  auto* red = static_cast<std::uint8_t*>(output.mutable_data(0));
  auto* green = static_cast<std::uint8_t*>(output.mutable_data(1));
  auto* blue = static_cast<std::uint8_t*>(output.mutable_data(2));
  for (std::size_t row = 0; row < height; ++row) {
    const auto* packed = source + row * stride;
    for (std::size_t col = 0; col < width; ++col) {
      const auto* pixel = packed + col * 3U;
      red[row * output.stride(0) + col] = pixel[order == ChannelOrder::RGB ? 0 : 2];
      green[row * output.stride(1) + col] = pixel[1];
      blue[row * output.stride(2) + col] = pixel[order == ChannelOrder::RGB ? 2 : 0];
    }
  }
  return output;
}

void Image::export_packed_rgb(ChannelOrder order, void* data, std::size_t stride) const {
  if (channel_count() != 3U || pixel_type(0) != PixelType::UInt8 ||
      pixel_type(1) != PixelType::UInt8 || pixel_type(2) != PixelType::UInt8) {
    throw Exception(Status::TypeMismatch, ErrorCode::InvalidType,
                    "Image::export_packed_rgb", "image", -1,
                    "packed RGB export requires exactly three UInt8 channels");
  }
  if (data == nullptr) {
    throw Exception(Status::InvalidArgument, ErrorCode::NullArgument,
                    "Image::export_packed_rgb", "data", -1,
                    "packed output data must not be null");
  }
  if (width() > std::numeric_limits<std::size_t>::max() / 3U || stride < width() * 3U) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue,
                    "Image::export_packed_rgb", "stride", -1,
                    "packed stride is smaller than the RGB row size");
  }

  const auto* red = static_cast<const std::uint8_t*>(this->data(0));
  const auto* green = static_cast<const std::uint8_t*>(this->data(1));
  const auto* blue = static_cast<const std::uint8_t*>(this->data(2));
  auto* destination = static_cast<std::uint8_t*>(data);
  for (std::size_t row = 0; row < height(); ++row) {
    auto* packed = destination + row * stride;
    for (std::size_t col = 0; col < width(); ++col) {
      auto* pixel = packed + col * 3U;
      pixel[order == ChannelOrder::RGB ? 0 : 2] = red[row * this->stride(0) + col];
      pixel[1] = green[row * this->stride(1) + col];
      pixel[order == ChannelOrder::RGB ? 2 : 0] = blue[row * this->stride(2) + col];
    }
  }
}

Region Image::domain() const {
  if (empty()) return {};
  if (domain_) return *domain_;
  // 共享像素存储的尺寸不变，一次发布不可变完整定义域；缩减域仍属于各 Image 值。
  std::call_once(storage_->full_domain_once, [&] {
    storage_->full_domain_region = Region::rectangle(
        RectI64{0, 0, static_cast<std::int64_t>(height() - 1),
                static_cast<std::int64_t>(width() - 1)});
  });
  return storage_->full_domain_region;
}

Image Image::reduce_domain(const Region& region) const {
  if (empty()) return {};
  const RectI64 bounds{0, 0, static_cast<std::int64_t>(height() - 1),
                      static_cast<std::int64_t>(width() - 1)};
  const auto is_rectangle = [](const Region& value, const RectI64& box) {
    const auto rows = std::uint64_t(box.row_max) - std::uint64_t(box.row_min) + 1;
    const auto cols = std::uint64_t(box.col_max) - std::uint64_t(box.col_min) + 1;
    return rows && cols && rows <= UINT64_MAX / cols && value.area() == rows * cols;
  };
  // 空/矩形捷径前保留 clipped 的数量/宽度/面积/容量检查，包括巨大外部图像尺寸。
  const auto rows = std::uint64_t(bounds.row_max) + 1;
  const auto cols = std::uint64_t(bounds.col_max) + 1;
  const bool safe_bounds = bounds.row_max >= 0 && bounds.col_max >= 0 &&
      rows && cols && rows <= std::pmr::vector<RegionRun>{mv::runtime::persist_resource()}.max_size() &&
      rows <= UINT64_MAX / cols;
  const auto requested = region.bounding_box();
  if (safe_bounds && requested && is_rectangle(region, *requested)) {
    const auto previous = domain_ ? domain_->bounding_box() : std::nullopt;
    if (!domain_ || !previous || is_rectangle(*domain_, *previous)) {
      if (domain_ && !previous) return Image{storage_, Region{}};
      auto box = RectI64{
          std::max(bounds.row_min, requested->row_min),
          std::max(bounds.col_min, requested->col_min),
          std::min(bounds.row_max, requested->row_max),
          std::min(bounds.col_max, requested->col_max)};
      if (previous) {
        box.row_min = std::max(box.row_min, previous->row_min);
        box.col_min = std::max(box.col_min, previous->col_min);
        box.row_max = std::min(box.row_max, previous->row_max);
        box.col_max = std::min(box.col_max, previous->col_max);
      }
      if (box.row_min > box.row_max || box.col_min > box.col_max)
        return Image{storage_, Region{}};
      if (!domain_ && box == *requested) return Image{storage_, region};
      if (previous && box == *previous) return Image{storage_, *domain_};
      return Image{storage_, Region::rectangle(box)};
    }
  }
  auto clipped = region.clipped(bounds);
  if (domain_) {
    const auto previous = domain_->bounding_box(), requested = clipped.bounding_box();
    if (previous && requested && is_rectangle(*domain_, *previous) &&
        is_rectangle(clipped, *requested)) {
      clipped = Region::rectangle({
          std::max(previous->row_min, requested->row_min),
          std::max(previous->col_min, requested->col_min),
          std::min(previous->row_max, requested->row_max),
          std::min(previous->col_max, requested->col_max)});
    } else {
      clipped = domain_->intersect(clipped);
    }
  }
  return Image{storage_, std::move(clipped)};
}

Image Image::full_domain() const {
  if (empty()) return {};
  return Image{storage_};
}

Image Image::clone() const {
  if (empty()) return {};
  std::vector<PixelType> types;
  types.reserve(channel_count());
  for (std::size_t channel = 0; channel < channel_count(); ++channel) {
    types.push_back(pixel_type(channel));
  }
  Image result = create(width(), height(), types);
  for (std::size_t channel = 0; channel < channel_count(); ++channel) {
    const auto row_bytes = checked_row_bytes(width(), pixel_type(channel));
    const auto* source = static_cast<const std::byte*>(data(channel));
    auto* destination = static_cast<std::byte*>(result.mutable_data(channel));
    for (std::size_t row = 0; row < height(); ++row) {
      std::memcpy(destination + row * result.stride(channel),
                  source + row * stride(channel), row_bytes);
    }
  }
  result.domain_ = domain_;
  return result;
}

}  // namespace mv::core
