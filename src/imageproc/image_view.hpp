#pragma once

#include <cstddef>
#include <cstdint>

#include <core/exception.hpp>
#include <core/image.hpp>

namespace mv::imageproc {

class ImageView {
 public:
  explicit ImageView(const mv::core::Image& image, std::size_t channel = 0)
      : image_(&image), channel_(channel) {
    if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, "ImageView", "image must not be empty");
    if (channel >= image.channel_count()) throw mv::core::Exception(mv::core::Status::OutOfRange, "ImageView", "channel is out of range");
  }

  [[nodiscard]] std::size_t width() const noexcept { return image_->width(); }
  [[nodiscard]] std::size_t height() const noexcept { return image_->height(); }
  [[nodiscard]] std::size_t stride() const { return image_->stride(channel_); }
  [[nodiscard]] mv::core::PixelType pixel_type() const { return image_->pixel_type(channel_); }
  [[nodiscard]] const std::byte* data() const { return static_cast<const std::byte*>(image_->data(channel_)); }
  [[nodiscard]] mv::core::Region domain() const { return image_->domain(); }

  template <typename T>
  [[nodiscard]] const T* row_as(std::size_t row) const {
    if (row >= height()) throw mv::core::Exception(mv::core::Status::OutOfRange, "ImageView::row_as", "row is outside image");
    return reinterpret_cast<const T*>(data() + row * stride());
  }

 private:
  const mv::core::Image* image_;
  std::size_t channel_;
};

class MutableImageView {
 public:
  explicit MutableImageView(mv::core::Image& image, std::size_t channel = 0)
      : image_(&image), channel_(channel) {
    if (image.empty()) throw mv::core::Exception(mv::core::Status::InvalidArgument, "MutableImageView", "image must not be empty");
    if (channel >= image.channel_count()) throw mv::core::Exception(mv::core::Status::OutOfRange, "MutableImageView", "channel is out of range");
  }

  [[nodiscard]] std::size_t width() const noexcept { return image_->width(); }
  [[nodiscard]] std::size_t height() const noexcept { return image_->height(); }
  [[nodiscard]] std::size_t stride() const { return image_->stride(channel_); }
  [[nodiscard]] mv::core::PixelType pixel_type() const { return image_->pixel_type(channel_); }
  [[nodiscard]] std::byte* data() { return static_cast<std::byte*>(image_->mutable_data(channel_)); }

  template <typename T>
  [[nodiscard]] T* row_as(std::size_t row) {
    if (row >= height()) throw mv::core::Exception(mv::core::Status::OutOfRange, "MutableImageView::row_as", "row is outside image");
    return reinterpret_cast<T*>(data() + row * stride());
  }

 private:
  mv::core::Image* image_;
  std::size_t channel_;
};

}  // namespace mv::imageproc
