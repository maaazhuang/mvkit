#pragma once

#include <exception>
#include <mvkit/cpp/mv_foundation.hpp>

namespace mv {

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4275)  // std::exception 不是 DLL 导出基类。
#endif
/**
 * @brief 保存 C++ 接口的状态码、错误定位和说明文本。
 */
class MVKIT_CPP_API MvException final : public std::exception {
 public:
  /**
   * @brief 复制错误定位和说明文本，构造异常对象。
   *
   * @param[in] status 逐项操作状态。
   * @param[in] code 具体错误码。
   * @param[in] operation 多个特征条件的组合方式。
   * @param[in] parameter 参数或数据项名称。
   * @param[in] index 发生错误的参数元素索引，未指定时为 -1。
   * @param[in] message 错误说明文本视图。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvException(MvStatus status, MvErrorCode code, MvStringView operation, MvStringView parameter, std::int64_t index,
              MvStringView message);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvException(const MvException&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvException(MvException&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvException& operator=(const MvException&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvException& operator=(MvException&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvException() noexcept override;

  /**
   * @brief 查询异常携带的状态码。
   *
   * @return 异常携带的状态码。
   */
  MvStatus GetStatus() const noexcept;

  /**
   * @brief 查询异常携带的具体错误码。
   *
   * @return 异常携带的具体错误码。
   */
  MvErrorCode GetErrorCode() const noexcept;

  /**
   * @brief 借用异常携带的算子名称。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return 算子名称的借用视图。
   */
  MvStringView GetOperatorName() const noexcept;

  /**
   * @brief 借用异常携带的参数名称。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return 参数名称的借用视图。
   */
  MvStringView GetParameterName() const noexcept;

  /**
   * @brief 查询异常携带的参数元素索引。
   *
   * @return 发生错误的参数元素索引；未指定时为 -1。
   */
  std::int64_t GetParameterIndex() const noexcept;

  /**
   * @brief 借用以空字符结尾的异常说明文本。
   *
   * @details 返回值借用当前实例的存储，在实例修改、移动或销毁后可能失效。
   *
   * @return 异常说明文本的借用指针。
   */
  const char* what() const noexcept override;

 private:
  struct Impl;
  Impl* impl_{nullptr};
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}  // namespace mv
