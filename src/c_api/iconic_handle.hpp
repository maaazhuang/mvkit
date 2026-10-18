#pragma once

#include <memory>
#include <string_view>
#include <utility>

#include "opaque.hpp"

#include <core/exception.hpp>

namespace mv::c_api {

inline void require_iconic(::MvObject handle, std::string_view operation,
                           std::string_view parameter = "objects") {
  if (handle == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, parameter, -1,
                          "iconic handle must not be null");
  }
}

inline void require_output(const void* output, std::string_view operation,
                           std::string_view parameter = "output") {
  if (output == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, parameter, -1,
                          "output pointer must not be null");
  }
}

inline std::unique_ptr<MvObjectImpl> make_iconic(core::IconicTuple value) {
  auto output = std::make_unique<MvObjectImpl>();
  output->value = std::move(value);
  return output;
}

inline std::unique_ptr<MvObjectImpl> make_image_handle(core::IconicTuple value,
                                                        std::string_view operation) {
  value.require_kind(core::IconicKind::Image, operation);
  auto output = std::make_unique<MvObjectImpl>();
  output->value = std::move(value);
  return output;
}

inline std::unique_ptr<MvObjectImpl> make_image_handle(core::Image value) {
  return make_image_handle(
      core::IconicTuple{core::IconicValue::from_image(std::move(value))},
      "make_image_handle");
}

inline const core::Image& require_single_image(::MvObject handle,
                                                std::string_view operation,
                                                std::string_view parameter = "image") {
  if (handle == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, parameter, -1, "image handle must not be null");
  }
  handle->value.require_kind(core::IconicKind::Image, operation);
  if (handle->value.size() != 1U) {
    throw core::Exception(core::Status::InvalidArgument,
                          core::ErrorCode::InvalidCardinality, operation,
                          parameter, -1,
                          "operator requires exactly one image object");
  }
  return handle->value.at(0).as_image();
}

inline core::Regions regions_value(::MvObject handle,
                                   std::string_view operation,
                                   std::string_view parameter = "regions") {
  if (handle == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, parameter, -1, "regions handle must not be null");
  }
  return core::Regions{handle->value};
}

inline core::XldContours xld_value(::MvObject handle,
                                   std::string_view operation,
                                   std::string_view parameter = "contours") {
  if (handle == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument,
                          operation, parameter, -1,
                          "xld contours handle must not be null");
  }
  return core::XldContours{handle->value};
}

}  // namespace mv::c_api
