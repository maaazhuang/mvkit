#include <core/xld.hpp>

#include <memory_resource>
#include <array>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <core/exception.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <runtime/memory/metadata_arena.hpp>

namespace mv::core {

struct XldContour::Storage {
  std::pmr::vector<Point2d> points{mv::runtime::persist_resource()};
  std::shared_ptr<const Storage> point_owner;
  std::span<const Point2d> shared_points;
  bool closed = false;
  ContourApproximation approximation = ContourApproximation::Unknown;
  std::pmr::vector<double> response{mv::runtime::persist_resource()};
  std::pmr::vector<double> edge_direction{mv::runtime::persist_resource()};
  std::pmr::vector<double> normal_angle{mv::runtime::persist_resource()};
  mutable std::atomic<GeometrySummary*> summary{nullptr};
  ~Storage(){delete summary.load(std::memory_order_relaxed);}
};
struct XldContour::InlineStorage : Storage { std::array<Point2d,16> inline_points; };
struct XldContour::PayloadStorage : Storage {
  OwnedPointPayload owned_points{mv::runtime::persist_resource()};
};

namespace {
void validate_attribute_size(std::span<const double> values, std::size_t count, const char* name) {
  if (!values.empty() && values.size() != count) {
    throw Exception(Status::InvalidArgument, "XldContour::from_points_and_attributes",
                    std::string{name} + " size must match point count");
  }
}
}

XldContour XldContour::from_points(std::span<const Point2d> points, bool closed) {
  return from_points_and_attributes(points, closed, ContourApproximation::Unknown);
}

XldContour XldContour::from_points_and_attributes(
    std::span<const Point2d> points, bool closed, ContourApproximation approximation,
    std::span<const double> response, std::span<const double> edge_direction,
    std::span<const double> normal_angle) {
  validate_attribute_size(response, points.size(), "response");
  validate_attribute_size(edge_direction, points.size(), "edge_direction");
  validate_attribute_size(normal_angle, points.size(), "normal_angle");
  auto storage = std::make_shared<Storage>();
  storage->points.assign(points.begin(), points.end());
  storage->closed = closed;
  storage->approximation = approximation;
  storage->response.assign(response.begin(), response.end());
  storage->edge_direction.assign(edge_direction.begin(), edge_direction.end());
  storage->normal_angle.assign(normal_angle.begin(), normal_angle.end());
  return XldContour{std::move(storage)};
}

XldContour XldContour::from_owned_points_and_attributes(
    std::pmr::vector<Point2d>&& points, bool closed, ContourApproximation approximation,
    std::span<const double> response, std::span<const double> edge_direction,
    std::span<const double> normal_angle, bool pooled,mv::runtime::MetadataArena* arena) {
  validate_attribute_size(response, points.size(), "response");
  validate_attribute_size(edge_direction, points.size(), "edge_direction");
  validate_attribute_size(normal_angle, points.size(), "normal_angle");
  auto storage = pooled && arena ? std::allocate_shared<Storage>(mv::runtime::MetadataAllocator<Storage>{arena})
      : pooled ? std::allocate_shared<Storage>(
      std::pmr::polymorphic_allocator<Storage>{mv::runtime::persist_resource()})
      : std::make_shared<Storage>();
  storage->points = std::move(points);
  storage->closed = closed;
  storage->approximation = approximation;
  storage->response.assign(response.begin(), response.end());
  storage->edge_direction.assign(edge_direction.begin(), edge_direction.end());
  storage->normal_angle.assign(normal_angle.begin(), normal_angle.end());
  return XldContour{std::move(storage)};
}

std::size_t XldContour::size() const noexcept { return points().size(); }

const XldContour::GeometrySummary& XldContour::geometry_summary() const {
  static const GeometrySummary empty;
  if(!storage_)return empty;
  auto* existing=storage_->summary.load(std::memory_order_acquire);
  if(existing)return *existing;
  auto candidate=std::make_unique<GeometrySummary>();
  const auto input=points();
  if(!input.empty()) {
    auto lo=input.front(),hi=lo;
    std::vector<Point2d> polygon;polygon.reserve(input.size());
    for(const auto p:input) {
      candidate->bounded=candidate->bounded && std::isfinite(p.row) && std::isfinite(p.col) &&
          std::abs(p.row)<=0x1p30 && std::abs(p.col)<=0x1p30;
      lo.row=std::min(lo.row,p.row);hi.row=std::max(hi.row,p.row);
      lo.col=std::min(lo.col,p.col);hi.col=std::max(hi.col,p.col);
      if(polygon.empty() || polygon.back()!=p)polygon.push_back(p);
    }
    candidate->row_min=lo.row;candidate->row_max=hi.row;
    candidate->column_min=lo.col;candidate->column_max=hi.col;
    if(polygon.size()>1 && polygon.front()==polygon.back())polygon.pop_back();
    const auto count=polygon.size();
    if(candidate->bounded && count>=3) {
      const auto cross=[](Point2d a,Point2d b,Point2d c) {
        return (b.col-a.col)*(c.row-a.row)-(b.row-a.row)*(c.col-a.col);
      };
      const auto half=[](Point2d d){return d.row>0 || (d.row==0 && d.col>=0)?0:1;};
      const auto first=cross(polygon[0],polygon[1],polygon[2]);
      const bool positive=first>0;bool convex=first!=0;std::size_t wraps=0;
      for(std::size_t i=0;i<count && convex;++i) {
        const auto a=polygon[i],b=polygon[(i+1)%count],c=polygon[(i+2)%count];
        const auto turn=cross(a,b,c);
        convex=turn!=0 && (turn>0)==positive;
        const auto from=half({b.row-a.row,b.col-a.col}),to=half({c.row-b.row,c.col-b.col});
        wraps+=positive?from>to:from<to;
      }
      // 同向转角且只绕一周才证明严格凸，星形自交不能通过。
      candidate->strictly_convex=convex && wraps==1;
    }
  }
  if(storage_->summary.compare_exchange_strong(existing,candidate.get(),std::memory_order_acq_rel,std::memory_order_acquire))
    return *candidate.release();
  return *existing;
}
XldContour XldContour::from_inline_points(std::span<const Point2d> points,bool closed,
    ContourApproximation approximation,std::span<const double> response,bool pooled,
    mv::runtime::MetadataArena* arena) {
  if(points.size()>16)throw Exception(Status::InvalidArgument,"XldContour::from_inline_points","inline point capacity exceeded");
  validate_attribute_size(response,points.size(),"response");
  auto storage=pooled && arena ? std::allocate_shared<InlineStorage>(mv::runtime::MetadataAllocator<InlineStorage>{arena})
      : pooled ? std::allocate_shared<InlineStorage>(std::pmr::polymorphic_allocator<InlineStorage>{mv::runtime::persist_resource()})
      : std::make_shared<InlineStorage>();
  std::copy(points.begin(),points.end(),storage->inline_points.begin());
  storage->shared_points={storage->inline_points.data(),points.size()};
  storage->closed=closed;storage->approximation=approximation;
  storage->response.assign(response.begin(),response.end());
  return XldContour{std::move(storage)};
}
bool XldContour::closed() const noexcept { return storage_ ? storage_->closed : false; }
XldContour XldContour::from_owned_point_payload(OwnedPointPayload&& points,
    bool closed,ContourApproximation approximation,std::span<const double> response,
    bool pooled,mv::runtime::MetadataArena* arena) {
  validate_attribute_size(response,points.size(),"response");
  auto storage=pooled && arena ? std::allocate_shared<PayloadStorage>(mv::runtime::MetadataAllocator<PayloadStorage>{arena})
      : pooled ? std::allocate_shared<PayloadStorage>(std::pmr::polymorphic_allocator<PayloadStorage>{mv::runtime::persist_resource()})
      : std::make_shared<PayloadStorage>();
  // 移动完整分配时保留目标资源；资源不同时先复制全部点，再发布只读跨度。
  storage->owned_points=std::move(points);
  storage->shared_points={storage->owned_points.data(),storage->owned_points.size()};
  storage->closed=closed;storage->approximation=approximation;
  storage->response.assign(response.begin(),response.end());
  return XldContour{std::move(storage)};
}
std::span<const Point2d> XldContour::points() const noexcept {
  if (!storage_) return {};
  return storage_->point_owner || storage_->shared_points.data() ? storage_->shared_points : std::span<const Point2d>{storage_->points};
}
XldContour XldContour::geometry_view(bool closed, ContourApproximation approximation) const {
  if (!storage_) return {};
  auto storage=std::make_shared<Storage>();
  storage->point_owner=storage_->point_owner ? storage_->point_owner : storage_;
  storage->shared_points=points();storage->closed=closed;storage->approximation=approximation;
  return XldContour{std::move(storage)};
}
bool XldContour::has_point_attribute(XldPointAttribute attribute) const noexcept { return !point_attribute(attribute).empty(); }
std::span<const double> XldContour::point_attribute(XldPointAttribute attribute) const noexcept {
  if (!storage_) return {};
  switch (attribute) {
    case XldPointAttribute::Response: return storage_->response;
    case XldPointAttribute::EdgeDirection: return storage_->edge_direction;
    case XldPointAttribute::NormalAngle: return storage_->normal_angle;
  }
  return {};
}
ContourApproximation XldContour::approximation() const noexcept { return storage_ ? storage_->approximation : ContourApproximation::Unknown; }

XldContours::XldContours(IconicTuple tuple) : tuple_(std::move(tuple)) {
  tuple_.require_kind(IconicKind::XldContour, "XldContours::XldContours");
}

std::size_t XldContours::contour_count() const noexcept { return tuple_.size(); }
const XldContour& XldContours::contour(std::size_t index) const {
  if (index >= contour_count()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                    "XldContours::contour", "index", static_cast<std::int64_t>(index),
                    "contour index is out of range");
  }
  return tuple_.at(index).as_xld_contour();
}
XldContours& XldContours::append(XldContour contour) {
  const IconicTuple one{IconicValue::from_xld_contour(std::move(contour))};
  tuple_ = tuple_.concat(one);
  return *this;
}

} // namespace mv::core
