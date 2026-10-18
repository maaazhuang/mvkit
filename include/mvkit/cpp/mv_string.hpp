#pragma once

#include <mvkit/cpp/mv_foundation.hpp>

namespace mv {

/**
 * @brief 拥有型 UTF-8 字符串，宽字符输入在构造时转换为 UTF-8。
 */
class MVKIT_CPP_API MvString {
 public:
  /**
   * @brief 构造空字符串。
   */
  MvString() noexcept;

  /**
   * @brief 复制字符内容，构造拥有型 UTF-8 字符串。
   *
   * @param[in] value 需要复制的字符内容。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvString(const char* value);

  /**
   * @brief 复制字符内容，构造拥有型 UTF-8 字符串。
   *
   * @param[in] value 需要复制的字符内容。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvString(const wchar_t* value);

  /**
   * @brief 复制字符内容，构造拥有型 UTF-8 字符串。
   *
   * @param[in] value 需要复制的字符内容。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvString(MvStringView value);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvString(const MvString&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvString(MvString&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvString& operator=(const MvString&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvString& operator=(MvString&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvString() noexcept;

  /**
   * @brief 借用以空字符结尾的 UTF-8 字符串。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return 以空字符结尾的 UTF-8 文本借用指针。
   */
  const char* Text() const noexcept;

  /**
   * @brief 查询 UTF-8 字符串的字节长度。
   *
   * @return UTF-8 字符串的字节数。
   */
  std::size_t Length() const noexcept;

  /**
   * @brief 借用 UTF-8 字符串视图。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return UTF-8 文本的借用视图。
   */
  MvStringView View() const noexcept;

 private:
  struct Impl;
  Impl* impl_{nullptr};
};

}  // namespace mv
