#pragma once
#include <mvkit/c/mv_tuple.h>

#include <mvkit/cpp/mv_string.hpp>
#include <mvkit/cpp/mv_handle.hpp>

namespace mv {

/** @brief 控制元组的整体元素类型；Mixed 表示包含不同类型的元素。 */
enum class MvTupleKind : std::int32_t {
  Empty = MV_TUPLE_EMPTY,
  Int64 = MV_TUPLE_INT64,
  Real64 = MV_TUPLE_REAL64,
  String = MV_TUPLE_STRING,
  Handle = MV_TUPLE_HANDLE,
  Mixed = MV_TUPLE_MIXED,
};

class MvTuple;
namespace detail {
struct MvTupleAccess;
}  // namespace detail

/**
 * @brief 拥有型控制元组，支持整数、实数、字符串和共享控制句柄。
 *
 * @details 复制产生独立的元组内容，其中的控制句柄仍共享底层对象；移动接管内容，析构释放拥有的引用。
 */
class MVKIT_CPP_API MvTuple {
  friend struct detail::MvTupleAccess;

 public:
  /**
   * @brief 构造空的控制值对象。
   */
  MvTuple() noexcept;

  /**
   * @brief 以单个数值构造控制元组。
   *
   * @param[in] value 用于构造或追加的值。
   */
  MvTuple(int value) noexcept;

  /**
   * @brief 以单个数值构造控制元组。
   *
   * @param[in] value 用于构造或追加的值。
   */
  MvTuple(std::int64_t value) noexcept;

  /**
   * @brief 以单个数值构造控制元组。
   *
   * @param[in] value 用于构造或追加的值。
   */
  MvTuple(double value) noexcept;

  /**
   * @brief 以单个字符串构造控制元组。
   *
   * @param[in] value 用于构造或追加的值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple(const char* value);

  /**
   * @brief 以单个字符串构造控制元组。
   *
   * @param[in] value 用于构造或追加的值；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple(const MvString& value);

  /**
   * @brief 以单个控制句柄构造控制元组。
   *
   * @param[in] value 用于构造或追加的值；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple(const MvHandle& value);

  /**
   * @brief 复制构造，生成独立的值副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple(const MvTuple&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvTuple(MvTuple&&) noexcept;

  /**
   * @brief 复制赋值，生成独立的值副本。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple& operator=(const MvTuple&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvTuple& operator=(MvTuple&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvTuple() noexcept;

  /**
   * @brief 查询控制元组的元素数量。
   *
   * @return 当前容器的元素数量。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::size_t Length() const;

  /**
   * @brief 读取指定索引的整数元素。
   *
   * @param[in] index 从 0 开始的值元素索引。
   *
   * @return 指定元素的整数值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  std::int64_t I(std::size_t index = 0) const;

  /**
   * @brief 读取指定索引的实数元素。
   *
   * @param[in] index 从 0 开始的值元素索引。
   *
   * @return 指定元素的实数值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  double D(std::size_t index = 0) const;

  /**
   * @brief 复制指定索引的字符串元素。
   *
   * @param[in] index 从 0 开始的值元素索引。
   *
   * @return 指定字符串元素的拥有型副本。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvString S(std::size_t index = 0) const;

  /**
   * @brief 取得指定索引的控制句柄元素的共享引用。
   *
   * @param[in] index 从 0 开始的值元素索引。
   *
   * @return 自动管理共享引用的控制句柄。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHandle H(std::size_t index = 0) const;

  /**
   * @brief 向控制元组末尾追加一个元素。
   *
   * @param[in] value 用于构造或追加的值。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   * @throws std::bad_alloc 分配内存失败时抛出。
   */
  MvTuple& Append(int value);

  /**
   * @brief 向控制元组末尾追加一个元素。
   * @overload 采用标量数值参数。
   *
   * @return 当前实例的引用。
   */
  MvTuple& Append(std::int64_t value);

  /**
   * @brief 向控制元组末尾追加一个元素。
   * @overload 采用标量数值参数。
   *
   * @return 当前实例的引用。
   */
  MvTuple& Append(double value);

  /**
   * @brief 向控制元组末尾追加一个元素。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 当前实例的引用。
   */
  MvTuple& Append(const char* value);

  /**
   * @brief 向控制元组末尾追加一个元素。
   * @overload 追加控制句柄元素，保存底层对象的共享引用。
   *
   * @return 当前实例的引用。
   */
  MvTuple& Append(const MvHandle& value);

  /**
   * @brief 连接两个控制元组，结果保持元素顺序。
   *
   * @param[in] other 参与操作的另一控制元组；调用期间借用，不转移所有权。
   *
   * @return 操作结果元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_tuple_concat
   */
  MvTuple TupleConcat(const MvTuple& other) const;

  /**
   * @brief 按 从 0 开始的整数值索引选择元素；接受无小数部分的 real，保留顺序与重复项。
   *
   * @param[in] indices 从 0 开始的控制元组元素索引；调用期间借用，不转移所有权。
   *
   * @return 操作结果元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_tuple_select
   */
  MvTuple TupleSelect(const MvTuple& indices) const;

  /**
   * @brief 选择闭区间 [left_index, right_index]。
   *
   * @param[in] left 选择区间的起止索引元组；调用期间借用，不转移所有权。
   * @param[in] right 选择区间的起止索引元组；调用期间借用，不转移所有权。
   *
   * @return 操作结果元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_tuple_select_range
   */
  MvTuple TupleSelectRange(const MvTuple& left, const MvTuple& right) const;

  /**
   * @brief 借用当前控制元组，供 C 算子读取输入。
   *
   * @details 借用输入，在该元组修改、移动或销毁前有效。
   * 借用句柄在当前元组修改、移动或销毁前有效；调用方不得释放该句柄。
   *
   * @return C 控制元组的只读借用句柄。
   */
  ::MvTuple ControlInput() const noexcept;

 private:
  /**
   * @brief 将控制元组的内联值转换为拥有型存储。
   *
   * @return 当前控制元组的拥有型 C 句柄。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  ::MvTuple Materialize();

  /**
   * @brief 交换两个控制元组的内部状态。
   *
   * @return 无返回值。
   */
  void Swap(MvTuple&) noexcept;

  ::MvTuple tuple_{nullptr};
  MvString* text_{nullptr};
  ::MvValue scalar_{};
  /**
   * @brief 初始化控制元组的借用输入存储。
   *
   * @return 无返回值。
   */
  void InitInput() noexcept;

  /**
   * @brief 销毁控制元组的借用输入存储。
   *
   * @return 无返回值。
   */
  void DestroyInput() noexcept;

  alignas(std::max_align_t) unsigned char input_storage_[96];
};

/**
 * @brief 连接两个控制元组，结果保持元素顺序。
 *
 * @param[in] left 待连接或组合的控制元组；调用期间借用，不转移所有权。
 * @param[in] right 待连接或组合的控制元组；调用期间借用，不转移所有权。
 * @param[out] concatenated 操作结果元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_tuple_concat
 */
MVKIT_CPP_API void TupleConcat(const MvTuple& left, const MvTuple& right, MvTuple* concatenated);

/**
 * @brief 按 从 0 开始的整数值索引选择元素；接受无小数部分的 real，保留顺序与重复项。
 *
 * @param[in] tuple 控制元组；调用期间借用，不转移所有权。
 * @param[in] indices 数据、元素或对象索引元组；调用期间借用，不转移所有权。
 * @param[out] selected 操作结果元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_tuple_select
 */
MVKIT_CPP_API void TupleSelect(const MvTuple& tuple, const MvTuple& indices, MvTuple* selected);

/**
 * @brief 选择闭区间 [left_index, right_index]。
 *
 * @param[in] tuple 控制元组；调用期间借用，不转移所有权。
 * @param[in] left 选择区间的起止索引元组；调用期间借用，不转移所有权。
 * @param[in] right 选择区间的起止索引元组；调用期间借用，不转移所有权。
 * @param[out] selected 操作结果元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_tuple_select_range
 */
MVKIT_CPP_API void TupleSelectRange(const MvTuple& tuple, const MvTuple& left, const MvTuple& right,
                                    MvTuple* selected);

/**
 * @brief 查询控制元组的 HALCON 类型码，空元组返回 31。
 *
 * @param[in] T 输入控制元组。
 * @param[out] Type 单元素整数结果元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数、对象状态或底层操作无效时抛出。
 */
MVKIT_CPP_API void TupleType(const MvTuple& T, MvTuple* Type);

}  // namespace mv
