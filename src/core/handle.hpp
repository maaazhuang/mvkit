#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>

#include <core/exception.hpp>

namespace mv::core {

using TypeId = std::uint64_t;

[[nodiscard]] constexpr TypeId make_type_id(std::string_view name) noexcept {
  TypeId hash = 14695981039346656037ull;
  for (const unsigned char ch : name) {
    hash ^= static_cast<TypeId>(ch);
    hash *= 1099511628211ull;
  }
  return hash == 0 ? TypeId{1} : hash;
}

class Object {
 public:
  virtual ~Object() = default;
  [[nodiscard]] virtual TypeId type_id() const noexcept = 0;
  [[nodiscard]] virtual bool is_valid() const noexcept { return true; }
  virtual void clear() {
    throw Exception(Status::Unsupported, "clear_handle", "handle type does not support clearing");
  }
};

class Handle {
 public:
  Handle() = default;

  template <class T, class... Args>
    requires std::derived_from<T, Object>
  [[nodiscard]] static Handle make(Args&&... args) {
    return Handle{std::make_shared<T>(std::forward<Args>(args)...)};
  }

  [[nodiscard]] bool empty() const noexcept { return object_ == nullptr; }
  [[nodiscard]] explicit operator bool() const noexcept { return !empty(); }
  [[nodiscard]] TypeId type_id() const noexcept { return object_ ? object_->type_id() : TypeId{0}; }
  [[nodiscard]] bool is_valid() const noexcept { return object_ && object_->is_valid(); }
  // 同步适配器可一次检查动态类型并借用；空或类型不符由调用方保留原诊断。
  template <class T>
    requires std::derived_from<T, Object>
  [[nodiscard]] T* object_if() const noexcept {
    if (!object_ || object_->type_id() != T::kTypeId) return nullptr;
    return static_cast<T*>(object_.get());
  }
  void clear() const {
    if (!object_) throw Exception(Status::InvalidState, "clear_handle", "handle is empty");
    object_->clear();
  }

  template <class T>
    requires std::derived_from<T, Object>
  [[nodiscard]] T& object() const {
    if (!object_) throw Exception(Status::InvalidState, "Handle::as", "handle is empty");
    if (object_->type_id() != T::kTypeId)
      throw Exception(Status::TypeMismatch, "Handle::as", "handle runtime type does not match requested type");
    // 仅在本 Handle 存活期间借用；适配器无须为同步查询额外增减共享引用。
    return static_cast<T&>(*object_);
  }

  template <class T>
    requires std::derived_from<T, Object>
  [[nodiscard]] std::shared_ptr<T> as() const {
    if (!object_) {
      throw Exception(Status::InvalidState, "Handle::as", "handle is empty");
    }
    if (object_->type_id() != T::kTypeId) {
      throw Exception(Status::TypeMismatch, "Handle::as", "handle runtime type does not match requested type");
    }
    return std::static_pointer_cast<T>(object_);
  }

  friend bool operator==(const Handle& lhs, const Handle& rhs) noexcept {
    return lhs.object_.get() == rhs.object_.get();
  }

 private:
  explicit Handle(std::shared_ptr<Object> object) : object_(std::move(object)) {}
  std::shared_ptr<Object> object_;
};

}  // namespace mv::core
