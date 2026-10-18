#pragma once
#include <mvkit/cpp/mv_iconic.hpp>

namespace mv {
/**
 * @brief 自动管理共享引用的可变字典；通过副本修改会影响同一字典。
 */
class MVKIT_CPP_API MvDict : public MvHandle {
 public:
  /**
   * @brief 构造已初始化的空字典。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvDict();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvDict(const MvDict&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvDict(MvDict&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvDict& operator=(const MvDict&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvDict& operator=(MvDict&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvDict() noexcept;

  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 通用控制对象句柄；调用期间借用，不转移所有权。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvDict(const MvHandle& handle);

  /**
   * @brief 将当前实例初始化为空字典。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_create_dict
   */
  void CreateDict();

  /**
   * @brief 单键保存完整控制元组（可为空），多键使用 scalar-or-N 值。
   *
   * @param[in] key 字典键元组；调用期间借用，不转移所有权。
   * @param[in] value 待保存的数据元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_dict_tuple
   */
  void SetDictTuple(const MvTuple& key, const MvTuple& value) const;

  /**
   * @brief 单键保存完整控制元组（可为空），多键使用 scalar-or-N 值。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetDictTuple(const MvString& key, const MvTuple& value) const;

  /**
   * @brief 单键保存完整控制元组（可为空），多键使用 scalar-or-N 值。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetDictTuple(const char* key, const MvTuple& value) const;

  /**
   * @brief 多键要求每个键存储一个控制值；失败保持原输出。
   *
   * @param[in] key 字典键元组；调用期间借用，不转移所有权。
   *
   * @return 读取的数据元组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_dict_tuple
   */
  MvTuple GetDictTuple(const MvTuple& key) const;

  /**
   * @brief 多键要求每个键存储一个控制值；失败保持原输出。
   * @overload 采用字符串对象参数。
   *
   * @return 读取的数据元组，由返回对象自动管理资源。
   */
  MvTuple GetDictTuple(const MvString& key) const;

  /**
   * @brief 多键要求每个键存储一个控制值；失败保持原输出。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 读取的数据元组，由返回对象自动管理资源。
   */
  MvTuple GetDictTuple(const char* key) const;

  /**
   * @brief HALCON set/get_dict_object 要求一个键，保存完整图标对象数组。
   *
   * @param[in] object 待保存的图标对象数组；调用期间借用，不转移所有权。
   * @param[in] key 字典键元组；调用期间借用，不转移所有权。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_set_dict_object
   */
  void SetDictObject(const MvObject& object, const MvTuple& key) const;

  /**
   * @brief HALCON set/get_dict_object 要求一个键，保存完整图标对象数组。
   * @overload 采用字符串对象参数。
   *
   * @return 无返回值。
   */
  void SetDictObject(const MvObject& object, const MvString& key) const;

  /**
   * @brief HALCON set/get_dict_object 要求一个键，保存完整图标对象数组。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 无返回值。
   */
  void SetDictObject(const MvObject& object, const char* key) const;

  /**
   * @brief 读取指定字典键保存的完整图标对象数组。
   *
   * @param[in] key 字典键元组；调用期间借用，不转移所有权。
   *
   * @return 读取的图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_get_dict_object
   */
  MvObject GetDictObject(const MvTuple& key) const;

  /**
   * @brief 读取指定字典键保存的完整图标对象数组。
   * @overload 采用字符串对象参数。
   *
   * @return 读取的图标对象数组，由返回对象自动管理资源。
   */
  MvObject GetDictObject(const MvString& key) const;

  /**
   * @brief 读取指定字典键保存的完整图标对象数组。
   * @overload 采用 UTF-8 字符串参数。
   *
   * @return 读取的图标对象数组，由返回对象自动管理资源。
   */
  MvObject GetDictObject(const char* key) const;

};
/**
 * @brief 创建字典，返回包含一个受检控制句柄的 拥有型控制元组。
 *
 * @param[out] dict 字典元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_create_dict
 */
MVKIT_CPP_API void CreateDict(MvTuple* dict);

/**
 * @brief 单键保存完整控制元组（可为空），多键使用 scalar-or-N 值。
 *
 * @param[in] dict 字典元组；调用期间借用，不转移所有权。
 * @param[in] key 字典键元组；调用期间借用，不转移所有权。
 * @param[in] value 待保存的数据元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_dict_tuple
 */
MVKIT_CPP_API void SetDictTuple(const MvTuple& dict, const MvTuple& key, const MvTuple& value);

/**
 * @brief 多键要求每个键存储一个控制值；失败保持原输出。
 *
 * @param[in] dict 字典元组；调用期间借用，不转移所有权。
 * @param[in] key 字典键元组；调用期间借用，不转移所有权。
 * @param[out] value 读取的数据元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_dict_tuple
 */
MVKIT_CPP_API void GetDictTuple(const MvTuple& dict, const MvTuple& key, MvTuple* value);

/**
 * @brief HALCON set/get_dict_object 要求一个键，保存完整图标对象数组。
 *
 * @param[in] object 待保存的图标对象数组；调用期间借用，不转移所有权。
 * @param[in] dict 字典元组；调用期间借用，不转移所有权。
 * @param[in] key 字典键元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_set_dict_object
 */
MVKIT_CPP_API void SetDictObject(const MvObject& object, const MvTuple& dict, const MvTuple& key);

/**
 * @brief 读取指定字典键保存的完整图标对象数组。
 *
 * @param[out] object 读取的图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] dict 字典元组；调用期间借用，不转移所有权。
 * @param[in] key 字典键元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_get_dict_object
 */
MVKIT_CPP_API void GetDictObject(MvObject* object, const MvTuple& dict, const MvTuple& key);
/**
 * @brief 查询字典的键、键存在性或键值种类。
 *
 * @param[in] DictHandle 包含一个字典句柄的元组。
 * @param[in] GenParamName keys、key_exists 或 key_data_type。
 * @param[in] Key 查询键元组；keys 查询时须为空。
 * @param[out] GenParamValue 结果元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数、对象状态或底层操作无效时抛出。
 */
MVKIT_CPP_API void GetDictParam(const MvTuple& DictHandle, const MvTuple& GenParamName, const MvTuple& Key, MvTuple* GenParamValue);

/**
 * @brief 删除指定字典键，发生错误时仍尝试处理后续键。
 *
 * @param[in,out] DictHandle 包含一个字典句柄的元组。
 * @param[in] Key 非空的字符串或整数键元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数、对象状态或底层操作无效时抛出。
 */
MVKIT_CPP_API void RemoveDictKey(const MvTuple& DictHandle, const MvTuple& Key);

}
