#pragma once
#include <mvkit/c/mv_iconic.h>

#include <mvkit/cpp/mv_api.hpp>
#include <mvkit/cpp/mv_tuple.hpp>

namespace mv {

/** @brief 图标对象的图像、区域或 XLD 轮廓类型。 */
enum class MvIconicKind : std::int32_t {
  Image = MV_ICONIC_IMAGE,
  Region = MV_ICONIC_REGION,
  XldContour = MV_ICONIC_XLD_CONTOUR,
};

class MvObject;
namespace detail {
struct MvObjectAccess;
}  // namespace detail

/**
 * @brief 自动管理共享载荷的图标对象数组，可包含零个、一个或多个对象。
 *
 * @details 通道属于单张图像；复制共享载荷，拥有型引用在析构时自动释放。
 */
class MVKIT_CPP_API MvObject {
  friend struct detail::MvObjectAccess;

 public:
  /**
   * @brief 构造有效的空图标对象数组。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObject();

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObject(const MvObject&);

  /**
   * @brief 移动构造，接管源实例的内容。
   */
  MvObject(MvObject&&) noexcept;

  /**
   * @brief 复制赋值，共享源实例的底层引用。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObject& operator=(const MvObject&);

  /**
   * @brief 移动赋值，接管源实例的内容。
   *
   * @return 当前实例的引用。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObject& operator=(MvObject&&);

  /**
   * @brief 销毁当前实例并释放其拥有的资源。
   */
  virtual ~MvObject() noexcept;

  /**
   * @brief 查询图标对象数组的对象数量。
   *
   * @return 图标对象数量。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_count_obj
   */
  std::int64_t CountObj() const;

  /**
   * @brief 将当前图标对象数组置为空。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_gen_empty_obj
   */
  void GenEmptyObj();

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   *
   * @param[in] index 从 1 开始的图标对象索引；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_select_obj
   */
  MvObject SelectObj(const MvTuple& index) const;

  /**
   * @brief 按 从 1 开始的对象索引选择图标对象。
   * @overload 采用标量数值参数。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   */
  MvObject SelectObj(std::int64_t index) const;

  /**
   * @brief 按顺序拼接两个图标对象数组。
   *
   * @param[in] other 参与操作的另一图标对象数组；调用期间借用，不转移所有权。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_concat_obj
   */
  MvObject ConcatObj(const MvObject& other) const;

  /**
   * @brief 复制指定范围的图标对象引用并共享其载荷。
   *
   * @param[in] index 从 1 开始的图标对象索引。
   * @param[in] count 需要复制的图标对象数量。
   *
   * @return 图标对象数组，由返回对象自动管理资源。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   *
   * @see mv_copy_obj
   */
  MvObject CopyObj(std::int64_t index, std::int64_t count) const;

  /**
   * @brief 借用底层 C 句柄。
   *
   * @details 原生句柄在所属对象释放或替换其引用前有效；调用方不得直接释放借用引用。
   *
   * @return 原生 C 句柄的借用值，不增加引用计数。
   */
  ::MvObject NativeHandle() const noexcept;

  /**
   * @brief 保留底层 C 句柄的一次引用，构造拥有型 C++ 对象。
   *
   * @details 保留输入句柄的共享引用及其像素、区域或轮廓载荷；调用方继续持有原引用。
   *
   * @param[in] handle 原生 C 图标对象句柄。
   *
   * @return 自动管理保留引用的 C++ 对象。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  static MvObject FromNative(::MvObject handle);

 protected:
  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] kind 图标对象类型约束。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  explicit MvObject(::MvIconicKind kind);

  /**
   * @brief 复制构造，共享源实例的底层引用。
   *
   * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
   * @param[in] kind 图标对象类型约束。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  MvObject(const MvObject& objects, ::MvIconicKind kind);

  /**
   * @brief 移动构造，接管源实例的内容。
   *
   * @param[in] objects 图标对象数组。
   * @param[in] kind 图标对象类型约束。
   */
  MvObject(MvObject&& objects, ::MvIconicKind kind) noexcept;

  /**
   * @brief 验证图标对象与当前类型约束相符。
   *
   * @param[in] handle 原生 C 图标对象句柄。
   *
   * @return 无返回值。
   *
   * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
   */
  void CheckKind(::MvObject handle) const;

 private:
  /**
   * @brief 标识接管原生句柄引用的内部构造路径。
   */
  struct AdoptTag {};
  /**
   * @brief 验证对象类型，构造共享载荷的强类型视图。
   *
   * @param[in] handle 原生 C 图标对象句柄。
   */
  MvObject(::MvObject handle, AdoptTag) noexcept;

  /**
   * @brief 释放当前对象拥有的底层引用。
   *
   * @return 无返回值。
   */
  void Release() noexcept;

  // 所有 iconic 派生类型共用一个状态。通过 MvObject* 赋值仍验证目标类型约束。
  // 类型约束由统一状态保证，不维护派生类缓存。
  ::MvObject handle_{nullptr};
  ::MvIconicKind kind_{-1};
};

class MvImage;
class MvRegion;
class MvXLD;
class MvXLDCont;
class MvCamPar;
class MvPose;
class MvHomMat2D;
class MvNCCModel;
class MvNCCModelArray;
class MvShapeModel;
class MvShapeModelArray;

/**
 * @brief 创建不含任何图标对象的空对象数组。
 *
 * @param[out] objects 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_gen_empty_obj
 */
MVKIT_CPP_API void GenEmptyObj(MvObject* objects);

/**
 * @brief 查询图标对象数组的对象数量。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] count 图标对象数量元组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_count_obj
 */
MVKIT_CPP_API void CountObj(const MvObject& objects, MvTuple* count);

/**
 * @brief 按 从 1 开始的对象索引选择图标对象。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] selected 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] index 数据、元素或对象索引元组；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_select_obj
 */
MVKIT_CPP_API void SelectObj(const MvObject& objects, MvObject* selected, const MvTuple& index);

/**
 * @brief 复制指定范围的图标对象引用并共享其载荷。
 *
 * @param[in] objects 图标对象数组；调用期间借用，不转移所有权。
 * @param[out] copied 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 * @param[in] index 数据、元素或对象索引元组；调用期间借用，不转移所有权。
 * @param[in] count 需要复制的图标对象数量；调用期间借用，不转移所有权。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_copy_obj
 */
MVKIT_CPP_API void CopyObj(const MvObject& objects, MvObject* copied, const MvTuple& index,
                           const MvTuple& count);

/**
 * @brief 按顺序拼接两个图标对象数组。
 *
 * @param[in] left 左侧图标对象数组；调用期间借用，不转移所有权。
 * @param[in] right 右侧图标对象数组；调用期间借用，不转移所有权。
 * @param[out] concatenated 图标对象数组；接收结果的 C++ 对象，自动管理资源。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数非法、对象状态不符或底层操作失败时抛出。
 *
 * @see mv_concat_obj
 */
MVKIT_CPP_API void ConcatObj(const MvObject& left, const MvObject& right, MvObject* concatenated);

/**
 * @brief 查询图标对象数组各元素的种类。
 *
 * @param[in] Objects 图标对象数组。
 * @param[out] Classes 种类字符串元组。
 *
 * @return 无返回值。
 *
 * @throws MvException 输入无效或查询失败时抛出。
 *
 * @see mv_get_obj_class
 */
MVKIT_CPP_API void GetObjClass(const MvObject& Objects, MvTuple* Classes);

/**
 * @brief 深复制图像对象数组的像素及定义域。
 *
 * @param[in] Image 图像对象数组。
 * @param[out] DupImage 独立的图像对象数组。
 *
 * @return 无返回值。
 *
 * @throws MvException 输入类型错误或复制失败时抛出。
 *
 * @see mv_copy_image
 */
MVKIT_CPP_API void CopyImage(const MvObject& Image, MvObject* DupImage);

/**
 * @brief 清除当前拥有的图标对象数组，重置为有效零对象数组。
 *
 * @details 释放当前对象的引用，其他复制对象仍有效；C++ 析构也会自动执行引用清理。
 * 使用可写对象指针，以便同步更新 C++ 拥有状态并避免重复释放。
 *
 * @param[in,out] Objects 待清理的图标对象。
 *
 * @return 无返回值。
 *
 * @throws MvException 参数为空或创建空对象失败时抛出。
 *
 * @see mv_clear_obj
 */
MVKIT_CPP_API void ClearObj(MvObject* Objects);

}  // namespace mv
