#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <runtime/memory/memory_resource.hpp>
#include <span>
#include <vector>

#include <core/exception.hpp>
#include <core/region.hpp>

namespace mv::core {

enum class PixelType : std::uint8_t {
  UInt8,
  UInt16,
  Int16,
  Int32,
  Float32,
};

enum class ChannelOrder : std::uint8_t { RGB, BGR };

[[nodiscard]] inline std::size_t bytes_per_sample(PixelType type) {
  switch (type) {
  case PixelType::UInt8:
    return 1;
  case PixelType::UInt16:
    return 2;
  case PixelType::Int16:
    return 2;
  case PixelType::Int32:
    return 4;
  case PixelType::Float32:
    return 4;
  }
  throw Exception(Status::Unsupported, "bytes_per_sample",
                  "unsupported pixel type");
}

struct ImagePlaneSpec {
  PixelType pixel_type{PixelType::UInt8};
  void *data{nullptr};
  std::size_t stride{0};
  std::shared_ptr<void> owner;
};

class Image {
public:
  Image() = default;

  [[nodiscard]] static Image create(std::size_t width, std::size_t height,
                                    PixelType type);
  [[nodiscard]] static Image create(std::size_t width, std::size_t height,
                                    std::span<const PixelType> channel_types);
  // 内部快路径：暴露前须写完每平面的全部字节。
  [[nodiscard]] static Image
  create_uninitialized(std::size_t width, std::size_t height,
                       std::span<const PixelType> channel_types);
  [[nodiscard]] static Image wrap(std::size_t width, std::size_t height,
                                  PixelType type, void *data,
                                  std::size_t stride,
                                  std::shared_ptr<void> owner);
  [[nodiscard]] static Image
  wrap_planes(std::size_t width, std::size_t height,
              std::span<const ImagePlaneSpec> planes);
  [[nodiscard]] static Image
  import_packed_rgb(std::size_t width, std::size_t height, ChannelOrder order,
                    const void *data, std::size_t stride);

  [[nodiscard]] bool empty() const noexcept { return storage_ == nullptr; }
  [[nodiscard]] std::size_t width() const noexcept {
    return storage_ ? storage_->width : 0;
  }
  [[nodiscard]] std::size_t height() const noexcept {
    return storage_ ? storage_->height : 0;
  }
  [[nodiscard]] std::size_t channel_count() const noexcept {
    return storage_ ? storage_->planes.size() : 0;
  }
  [[nodiscard]] std::size_t stride(std::size_t channel = 0) const {
    if (!storage_ || channel >= storage_->planes.size())
      throw_channel_out_of_range(channel);
    return storage_->planes[channel].stride;
  }
  [[nodiscard]] PixelType pixel_type(std::size_t channel = 0) const {
    if (!storage_ || channel >= storage_->planes.size())
      throw_channel_out_of_range(channel);
    return storage_->planes[channel].pixel_type;
  }
  [[nodiscard]] const void *data(std::size_t channel = 0) const {
    if (!storage_ || channel >= storage_->planes.size())
      throw_channel_out_of_range(channel);
    return storage_->planes[channel].data();
  }
  [[nodiscard]] void *mutable_data(std::size_t channel = 0) {
    if (!storage_ || channel >= storage_->planes.size())
      throw_channel_out_of_range(channel);
    return storage_->planes[channel].data();
  }
  void export_packed_rgb(ChannelOrder order, void *data,
                         std::size_t stride) const;

  [[nodiscard]] bool has_reduced_domain() const noexcept {
    return domain_.has_value();
  }
  [[nodiscard]] Region domain() const;
  [[nodiscard]] Image reduce_domain(const Region &region) const;
  [[nodiscard]] Image full_domain() const;
  [[nodiscard]] Image clone() const;
  // 私有通道视图直接持有源 Storage，保持像素共享和独立定义域，不再包装中间 Image。
  [[nodiscard]] Image channel_view(std::size_t channel) const;
  // 三个视图共用元数据所有权，像素仍由源 Storage 持有。
  [[nodiscard]] std::array<Image,3> channel_views3() const;
  [[nodiscard]] ImagePlaneSpec shared_plane(std::size_t channel) const;

private:
  // 私有平面布局供内联读取，Image 自身的指针/定义域布局保持不变。
  struct Storage {
    struct Plane {
      PixelType pixel_type = PixelType::UInt8;
      std::size_t stride = 0;
      std::pmr::vector<std::byte> owned{mv::runtime::persist_resource()};
      void *external_data = nullptr;
      std::shared_ptr<void> external_owner;

      [[nodiscard]] void *data() noexcept {
        return owned.empty() ? external_data : owned.data();
      }
      [[nodiscard]] const void *data() const noexcept {
        return owned.empty() ? external_data : owned.data();
      }
    };

    std::size_t width = 0;
    std::size_t height = 0;
    std::vector<Plane> planes;
    mutable std::once_flag full_domain_once;
    mutable Region full_domain_region;
  };
  [[noreturn]] static void throw_channel_out_of_range(std::size_t channel);
  explicit Image(std::shared_ptr<Storage> storage,
                 std::optional<Region> domain = std::nullopt)
      : storage_(std::move(storage)), domain_(std::move(domain)) {}
  std::shared_ptr<Storage> storage_;
  std::optional<Region> domain_;
};

} // namespace mv::core
