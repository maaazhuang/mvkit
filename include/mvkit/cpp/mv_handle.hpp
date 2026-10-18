#pragma once
#include <mvkit/c/mv_handle.h>

#include <mvkit/cpp/mv_foundation.hpp>

namespace mv {

/** @brief 通用控制句柄的稳定公开类型标识。 */
enum class MvObjectType : std::uint64_t {
  Unknown = MV_OBJECT_TYPE_UNKNOWN,
  NccModel = MV_OBJECT_TYPE_NCC_MODEL,
  Dict = MV_OBJECT_TYPE_DICT,
  CalibrationData = MV_OBJECT_TYPE_CALIBRATION_DATA,
  ShapeModel = MV_OBJECT_TYPE_SHAPE_MODEL,
  ObjectModel3D = MV_OBJECT_TYPE_OBJECT_MODEL_3D,
  CameraSetupModel = MV_OBJECT_TYPE_CAMERA_SETUP_MODEL,
  StereoRectification = MV_OBJECT_TYPE_STEREO_RECTIFICATION,
  StructuredLightModel = MV_OBJECT_TYPE_STRUCTURED_LIGHT_MODEL,
  StereoModel = MV_OBJECT_TYPE_STEREO_MODEL,
  SerializedItem = MV_OBJECT_TYPE_SERIALIZED_ITEM,
};

class MvHandle;
class MvSerializedItem;
class MvTuple;
class MvString;
namespace detail {
struct MvHandleAccess;
}  // namespace detail

/**
 * @brief 自动管理共享引用的通用控制句柄。
 */
class MVKIT_CPP_API MvHandle {
  friend struct detail::MvHandleAccess;

 public:
  /**
   * @brief 构造尚未初始化的控制句柄。
   */
  MvHandle() noexcept;

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHandle(const MvHandle&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvHandle(MvHandle&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvHandle& operator=(const MvHandle&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   */
  MvHandle& operator=(MvHandle&&) noexcept;

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  ~MvHandle() noexcept;

  /**
   * @brief 从借用的底层 C 句柄构造拥有型 C++ 对象。
   *
   * @details 保留输入句柄的共享引用；源拥有者修改或销毁后，返回对象仍独立持有资源。
   *
   * @param[in] handle 原生 C 通用控制句柄。
   *
   * @return 自动管理保留引用的 C++ 对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  static MvHandle FromNative(::MvHandle handle);

  /**
   * @brief 检查当前对象是否持有有效控制句柄。
   *
   * @return 持有有效控制句柄时为 true；空或已清理时为 false。
   */
  bool IsInitialized() const noexcept;

  /**
   * @brief 查询通用控制句柄的公开类型。
   *
   * @return 通用控制对象的公开类型标识。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObjectType Type() const;

  /**
   * @brief 借用底层 C 句柄。
   *
   * @details 原生句柄在所属对象释放或替换其引用前有效；调用方不得直接释放借用引用。
   *
   * @return 原生 C 句柄的借用值，不增加引用计数。
   */
  ::MvHandle NativeHandle() const noexcept;

  /** @brief 创建独立的模型字节快照，支持 Shape 和 NCC 模型。
   * @return 拥有字节存储的序列化项；可变模型须与修改操作同步。
   * @throws MvException 类型不支持、对象已清理或编码失败时抛出。
   */
  MvSerializedItem SerializeHandle() const;

  /** @brief 从字节快照恢复独立模型，成功后替换当前引用。
   * @param[in] SerializedItem 有效序列化项，调用期间借用。
   * @throws MvException 数据损坏、版本不支持或恢复失败时抛出；失败保留原值。
   */
  void DeserializeHandle(const MvSerializedItem& SerializedItem);

 private:
  /**
   * @brief 标识接管原生句柄引用的内部构造路径。
   */
  struct AdoptTag {};
  /**
   * @brief 验证控制句柄类型，构造共享引用的强类型对象。
   *
   * @param[in] handle 原生 C 通用控制句柄。
   */
  MvHandle(::MvHandle handle, AdoptTag) noexcept;

  /**
   * @brief 释放当前对象拥有的底层引用。
   *
   * @return 无返回值。
   */
  void Release() noexcept;

  ::MvHandle handle_{nullptr};
};

/**
 * @brief 清除共享控制句柄的内容，使全部别名失效。
 *
 * @param[in,out] Handle 控制句柄元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数、对象状态或底层操作无效时抛出。
 */
MVKIT_CPP_API void ClearHandle(const MvTuple& Handle);

/**
 * @brief 查询控制元组的语义类型；已清理句柄返回 handle。
 *
 * @param[in] T 输入控制元组。
 * @param[out] SemType 类型字符串结果元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数、对象状态或底层操作无效时抛出。
 */
MVKIT_CPP_API void TupleSemType(const MvTuple& T, MvTuple* SemType);

/** @brief 拥有共享字节存储的序列化项，复制共享身份，清除使全部别名失效。 */
class MVKIT_CPP_API MvSerializedItem : public MvHandle {
 public:
  /** @brief 构造未初始化的序列化项。 */
  MvSerializedItem() noexcept;

  /** @brief 验证序列化项类型并保留输入句柄的共享引用。
   * @param[in] handle 有效序列化项句柄。
   * @throws MvException 类型不符或句柄已清理时抛出。
   */
  explicit MvSerializedItem(const MvHandle& handle);

  /** @brief 复制外部可读字节；不接管输入内存。
   * @param[in] Pointer 外部缓冲区，须覆盖 Size 个可读字节。
   * @param[in] Size 正字节数。
   * @param[in] Copy 复制模式，当前仅支持 true。
   * @throws MvException 参数非法或复制失败时抛出。
   */
  explicit MvSerializedItem(void* Pointer, std::int64_t Size, const MvString& Copy);

  /** @brief 使用 UTF-8 复制模式构造序列化项，规则同主重载。 */
  explicit MvSerializedItem(void* Pointer, std::int64_t Size, const char* Copy);

  /** @brief 使用宽字符串复制模式构造序列化项，规则同主重载。 */
  explicit MvSerializedItem(void* Pointer, std::int64_t Size, const wchar_t* Copy);

  /** @brief 复制外部字节，成功后替换当前引用，失败保留原值。
   * @param[in] Pointer 外部可读缓冲区。
   * @param[in] Size 正字节数。
   * @param[in] Copy 复制模式，当前仅支持 true。
   * @throws MvException 参数非法或复制失败时抛出。
   */
  void CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const MvString& Copy);

  /** @brief 使用 UTF-8 复制模式导入字节，规则同主重载。 */
  void CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const char* Copy);

  /** @brief 使用宽字符串复制模式导入字节，规则同主重载。 */
  void CreateSerializedItemPtr(void* Pointer, std::int64_t Size, const wchar_t* Copy);

  /** @brief 借用字节地址并返回大小，禁止修改或释放借用存储。
   * @param[out] Size 字节数，失败时保持原值。
   * @return 地址在序列化项仍有拥有者且未清除时有效，不能与清除操作并发。
   * @throws MvException 输出为空、类型不符或项已清理时抛出。
   */
  void* GetSerializedItemPtr(std::int64_t* Size) const;
};

/** @brief 将一个 Shape、NCC 或计量模型保存为独立字节快照。
 * @param[in] Handle 单元素模型句柄元组。
 * @param[out] SerializedItem 拥有型序列化项元组，失败时保持原值。
 * @throws MvException 参数、对象状态、类型或编码无效时抛出。
 */
MVKIT_CPP_API void SerializeHandle(const MvTuple& Handle, MvTuple* SerializedItem);

/** @brief 从字节快照恢复独立模型身份。
 * @param[in] SerializedItem 单元素序列化项元组。
 * @param[out] Handle 拥有型模型句柄元组，失败时保持原值。
 * @throws MvException 数据损坏、版本不支持或恢复失败时抛出。
 */
MVKIT_CPP_API void DeserializeHandle(const MvTuple& SerializedItem, MvTuple* Handle);

/** @brief 复制外部字节为序列化项，当前仅支持 Copy=true。
 * @param[in] Pointer 可读缓冲区地址的整数元组。
 * @param[in] Size 正字节数元组。
 * @param[in] Copy 复制模式字符串元组。
 * @param[out] SerializedItemHandle 拥有型序列化项元组，失败时保持原值。
 * @throws MvException 参数非法或复制失败时抛出。
 */
MVKIT_CPP_API void CreateSerializedItemPtr(const MvTuple& Pointer, const MvTuple& Size,
                                          const MvTuple& Copy, MvTuple* SerializedItemHandle);

/** @brief 借用序列化项的只读字节地址，导出的地址整数不拥有存储。
 * @param[in] SerializedItemHandle 单元素序列化项元组。
 * @param[out] Pointer 字节地址整数元组，项未清除且有拥有者期间有效。
 * @param[out] Size 字节数元组，与 Pointer 输出不得别名；失败时两个输出保持原值。
 * @throws MvException 参数、类型或对象状态无效时抛出。
 */
MVKIT_CPP_API void GetSerializedItemPtr(const MvTuple& SerializedItemHandle, MvTuple* Pointer,
                                       MvTuple* Size);

}  // namespace mv
