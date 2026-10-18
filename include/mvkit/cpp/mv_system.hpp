#pragma once

#include <mvkit/c/mv_system.h>

#include <cstddef>
#include <cstdint>
#include <mvkit/cpp/mv_export.hpp>
#include <mvkit/cpp/mv_iconic.hpp>

namespace mv {

/**
 * @brief 设置当前系统参数。
 *
 * @param[in] SystemParameter 参数或数据项名称元组；调用期间借用，不转移所有权。
 * @param[in] Value 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_system
 */
MVKIT_CPP_API void SetSystem(const MvTuple& SystemParameter, const MvTuple& Value);

/**
 * @brief 查询当前系统参数。
 *
 * @param[in] Query 待查询的系统参数名称元组；调用期间借用，不转移所有权。
 * @param[out] Information 求解或拟合的信息量元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_system
 */
MVKIT_CPP_API void GetSystem(const MvTuple& Query, MvTuple* Information);

/**
 * @brief 提供系统参数查询和设置的静态便捷接口。
 */
class MVKIT_CPP_API MvSystem {
 public:
  /**
   * @brief 设置当前系统参数。
   *
   * @param[in] SystemParameter 参数或数据项名称元组；调用期间借用，不转移所有权。
   * @param[in] Value 保存、追加或读取的数据元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_system
   */
  static void SetSystem(const MvTuple& SystemParameter, const MvTuple& Value);

  /**
   * @brief 设置当前系统参数。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  static void SetSystem(const MvString& SystemParameter, const MvString& Value);

  /**
   * @brief 设置当前系统参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  static void SetSystem(const char* SystemParameter, const char* Value);

  /**
   * @brief 设置当前系统参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 无返回值。
   */
  static void SetSystem(const wchar_t* SystemParameter, const wchar_t* Value);

  /**
   * @brief 查询当前系统参数。
   *
   * @param[in] Query 待查询的系统参数名称元组；调用期间借用，不转移所有权。
   *
   * @return 求解或拟合的信息量元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_system
   */
  static MvTuple GetSystem(const MvTuple& Query);

  /**
   * @brief 查询当前系统参数。
   * @overload 采用字符串对象参数。
   *
   * @return 求解或拟合的信息量元组，由返回对象自动管理资源。
   */
  static MvTuple GetSystem(const MvString& Query);

  /**
   * @brief 查询当前系统参数。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 求解或拟合的信息量元组，由返回对象自动管理资源。
   */
  static MvTuple GetSystem(const char* Query);

  /**
   * @brief 查询当前系统参数。
   * @overload 采用宽字符字符串参数。
   *
   * @return 求解或拟合的信息量元组，由返回对象自动管理资源。
   */
  static MvTuple GetSystem(const wchar_t* Query);

};

}  // namespace mv
