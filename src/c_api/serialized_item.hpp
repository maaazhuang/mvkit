#pragma once

#include <core/handle.hpp>
#include <atomic>
#include <string>
#include "model_snapshot.hpp"

namespace mv::c_api {
// 字节存储拥有独立生命周期；指针导出借用该存储，不转移所有权。
class SerializedItemObject final : public core::Object {
 public:
  static constexpr core::TypeId kTypeId = core::make_type_id("mvkit.control.SerializedItem");
  explicit SerializedItemObject(std::string bytes) : bytes_(std::move(bytes)) {}
  explicit SerializedItemObject(std::shared_ptr<const ModelSnapshot> snapshot) : prepared(std::move(snapshot)) {}
  core::TypeId type_id() const noexcept override { return kTypeId; }
  bool is_valid() const noexcept override { return valid_.load(); }
  const std::string& bytes() const {
    if (!is_valid()) throw core::Exception(core::Status::InvalidState, "serialized_item", "item has been cleared");
    return bytes_.empty() && prepared ? prepared->bytes : bytes_;
  }
  void clear() override {
    if (!valid_.exchange(false))
      throw core::Exception(core::Status::InvalidState, "clear_handle", "item has already been cleared");
    std::string{}.swap(bytes_);
    prepared.reset();
  }
  // 只读字节保持原地址；导入项首次验证后可复用准备状态，失败不发布缓存。
  mutable std::mutex prepare_mutex;
  std::shared_ptr<const ModelSnapshot> prepared;
 private:
  std::atomic<bool> valid_{true};
  std::string bytes_;
};
}  // namespace mv::c_api
