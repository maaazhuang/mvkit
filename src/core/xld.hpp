#pragma once

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <memory_resource>
#include <span>
#include <vector>
#include <algorithm>
#include <utility>

#include <core/iconic_tuple.hpp>
#include <runtime/memory/plain_payload_allocator.hpp>

namespace mv::core {

enum class XldPointAttribute { Response, EdgeDirection, NormalAngle };
enum class ContourApproximation { Unknown, Line, Ellipse, Circle };

struct Point2d {
  double row = 0.0;
  double col = 0.0;
  friend bool operator==(const Point2d&, const Point2d&) = default;
};

class XldContour {
 public:
  struct GeometrySummary {
    double row_min=0,row_max=0,column_min=0,column_max=0;
    bool bounded=true,strictly_convex=false;
  };
  XldContour() = default;

  [[nodiscard]] static XldContour from_points(std::span<const Point2d> points, bool closed = false);
  [[nodiscard]] static XldContour from_points(std::initializer_list<Point2d> points, bool closed = false) {
    return from_points(std::span<const Point2d>{points.begin(), points.size()}, closed);
  }
  [[nodiscard]] static XldContour from_points_and_attributes(
      std::span<const Point2d> points, bool closed, ContourApproximation approximation,
      std::span<const double> response = {},
      std::span<const double> edge_direction = {},
      std::span<const double> normal_angle = {});

  // 内部所有权转移；allocator 不匹配时由 PMR vector 移动赋值保持 persist 存储契约。
  [[nodiscard]] static XldContour from_owned_points_and_attributes(
      std::pmr::vector<Point2d>&& points, bool closed, ContourApproximation approximation,
      std::span<const double> response = {}, std::span<const double> edge_direction = {},
      std::span<const double> normal_angle = {}, bool pooled = false,
      mv::runtime::MetadataArena* arena = nullptr);
  [[nodiscard]] static XldContour from_inline_points(std::span<const Point2d> points,
      bool closed,ContourApproximation approximation,std::span<const double> response,
      bool pooled,mv::runtime::MetadataArena* arena);
  // 完整覆盖的私有存储，避免 vector::resize 逐元素 allocator 构造；普通 Point2d 容器仍初始化。
  class OwnedPointPayload {
   public:
    explicit OwnedPointPayload(std::pmr::memory_resource* resource=mv::runtime::persist_resource())
        :allocator_(resource,true) {}
    explicit OwnedPointPayload(mv::runtime::PlainPayloadAllocator<Point2d> allocator)
        :allocator_(allocator.resource(),true) {}
    ~OwnedPointPayload(){clear();}
    OwnedPointPayload(const OwnedPointPayload&)=delete;
    OwnedPointPayload& operator=(const OwnedPointPayload&)=delete;
    OwnedPointPayload(OwnedPointPayload&& other) noexcept
        :allocator_(other.allocator_),count_(std::exchange(other.count_,0)),data_(std::exchange(other.data_,nullptr)) {}
    OwnedPointPayload& operator=(OwnedPointPayload&& other) {
      if(this==&other)return *this;
      if(allocator_==other.allocator_) {
        clear();count_=std::exchange(other.count_,0);data_=std::exchange(other.data_,nullptr);
      } else {
        OwnedPointPayload replacement{allocator_};
        replacement.reset_for_overwrite(other.count_);
        if(other.count_)std::copy_n(other.data_,other.count_,replacement.data_);
        std::swap(count_,replacement.count_);std::swap(data_,replacement.data_);
      }
      return *this;
    }
    void reset_for_overwrite(std::size_t count) {
      auto* next=count ? allocator_.allocate(count) : nullptr;
      clear();data_=next;count_=count;
    }
    Point2d* data() noexcept{return data_;}
    const Point2d* data() const noexcept{return data_;}
    std::size_t size() const noexcept{return count_;}
    Point2d& operator[](std::size_t i) noexcept{return data_[i];}
    const Point2d& operator[](std::size_t i) const noexcept{return data_[i];}
    auto get_allocator() const noexcept{return allocator_;}
   private:
    void clear() noexcept{if(data_)allocator_.deallocate(data_,count_);data_=nullptr;count_=0;}
    mv::runtime::PlainPayloadAllocator<Point2d> allocator_;
    std::size_t count_=0;
    Point2d* data_=nullptr;
  };
  // 内部完整输出：每个点须在转移前初始化。
  [[nodiscard]] static XldContour from_owned_point_payload(OwnedPointPayload&& points,
      bool closed,ContourApproximation approximation,std::span<const double> response,
      bool pooled,mv::runtime::MetadataArena* arena);

  [[nodiscard]] bool empty() const noexcept { return size() == 0; }
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] bool closed() const noexcept;
  [[nodiscard]] std::span<const Point2d> points() const noexcept;
  [[nodiscard]] bool has_point_attribute(XldPointAttribute attribute) const noexcept;
  [[nodiscard]] std::span<const double> point_attribute(XldPointAttribute attribute) const noexcept;
  [[nodiscard]] ContourApproximation approximation() const noexcept;
  // 不可变点集的按需几何事实，不包含属性；非法坐标由调用算子按原诊断规则拒绝。
  [[nodiscard]] const GeometrySummary& geometry_summary() const;
  // 不可变完整点视图；派生元数据不包含属性。
  [[nodiscard]] XldContour geometry_view(bool closed, ContourApproximation approximation) const;

 private:
  struct Storage;
  struct InlineStorage;
  struct PayloadStorage;
  explicit XldContour(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}
  std::shared_ptr<const Storage> storage_;
};

/** @brief 私有 XLD 轮廓有序集合。 */
class XldContours {
 public:
  XldContours() = default;
  explicit XldContours(IconicTuple tuple);

  [[nodiscard]] bool empty() const noexcept { return contour_count() == 0; }
  [[nodiscard]] std::size_t contour_count() const noexcept;
  [[nodiscard]] const XldContour& contour(std::size_t index) const;
  [[nodiscard]] const IconicTuple& iconic_tuple() const noexcept { return tuple_; }
  XldContours& append(XldContour contour);

 private:
  IconicTuple tuple_;
};

}  // namespace mv::core
