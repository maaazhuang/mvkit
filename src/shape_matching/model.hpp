#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>
#include <memory_resource>

#include <core/image.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::shape_matching {

struct ShapeModelSearchCache;
struct CreateParameters;
std::shared_ptr<ShapeModelSearchCache> make_shape_model_search_cache();

enum class ShapeMetric { UsePolarity, IgnoreGlobalPolarity, IgnoreLocalPolarity };
enum class ShapeModelKind { Rigid, IsotropicScale };

struct ShapeModelPoint {
  double row{0.0};
  double column{0.0};
  double normal_row{0.0};
  double normal_column{0.0};
  double response{0.0};
};

struct ShapeModelLevel {
  int level{0};
  double pyramid_scale{1.0};
  std::pmr::vector<ShapeModelPoint> points{mv::runtime::persist_resource()};
  mv::core::XldContours contours;
  double radius{0.0};

  ShapeModelLevel() = default;
  ShapeModelLevel(const ShapeModelLevel& other);
  ShapeModelLevel& operator=(const ShapeModelLevel& other);
  ShapeModelLevel(ShapeModelLevel&&) noexcept = default;
  ShapeModelLevel& operator=(ShapeModelLevel&&) noexcept = default;
};

class ShapeModel {
 public:
  ShapeModel() = default;
  ShapeModel(const ShapeModel& other);
  ShapeModel& operator=(const ShapeModel& other);
  ShapeModel(ShapeModel&&) noexcept = default;
  ShapeModel& operator=(ShapeModel&&) noexcept = default;

  [[nodiscard]] bool empty() const noexcept { return levels_.empty(); }
  [[nodiscard]] ShapeModelKind kind() const noexcept { return kind_; }
  [[nodiscard]] ShapeMetric metric() const noexcept { return metric_; }
  [[nodiscard]] bool is_xld_model() const noexcept { return xld_model_; }
  [[nodiscard]] const std::vector<ShapeModelLevel>& levels() const noexcept { return levels_; }
  [[nodiscard]] double default_origin_row() const noexcept { return default_origin_row_; }
  [[nodiscard]] double default_origin_column() const noexcept { return default_origin_column_; }
  [[nodiscard]] double origin_row() const noexcept { return origin_row_; }
  [[nodiscard]] double origin_column() const noexcept { return origin_column_; }
  [[nodiscard]] double angle_start() const noexcept { return angle_start_; }
  [[nodiscard]] double angle_extent() const noexcept { return angle_extent_; }
  [[nodiscard]] double angle_step() const noexcept { return angle_step_; }
  [[nodiscard]] double scale_min() const noexcept { return scale_min_; }
  [[nodiscard]] double scale_max() const noexcept { return scale_max_; }
  [[nodiscard]] double scale_step() const noexcept { return scale_step_; }
  [[nodiscard]] double min_contrast() const noexcept { return min_contrast_; }
  [[nodiscard]] double timeout_ms() const noexcept { return timeout_ms_; }
  [[nodiscard]] bool border_shape_models() const noexcept { return border_shape_models_; }
  [[nodiscard]] bool model_cache_enabled() const noexcept { return model_cache_enabled_; }
  [[nodiscard]] const std::string& metric_name() const noexcept { return metric_name_; }
  [[nodiscard]] std::span<const ShapeModelPoint> refinement_points() const noexcept {
    return refinement_points_;
  }

  void set_origin_offset(double row, double column);
  void set_min_contrast(double min_contrast);
  void set_timeout_ms(double timeout_ms);
  void set_border_shape_models(bool enabled);
  void set_model_cache_enabled(bool enabled);
  [[nodiscard]] const std::shared_ptr<ShapeModelSearchCache>& search_cache() const noexcept {
    return search_cache_;
  }
  [[nodiscard]] bool payloads_use_persist_resource() const noexcept;

 private:
  friend struct ShapeModelPersistence;
  friend ShapeModel create_model_xld(const mv::core::XldContours&, const struct XldCreateParameters&);
  friend ShapeModel create_model_impl(const mv::core::Image&, const mv::core::Tuple&, double, double,
                                      const mv::core::Tuple&, double, double, const mv::core::Tuple&,
                                      const mv::core::Tuple&, std::string, const mv::core::Tuple&,
                                      const mv::core::Tuple&, ShapeModelKind, const CreateParameters*);
  ShapeModelKind kind_{ShapeModelKind::Rigid};
  ShapeMetric metric_{ShapeMetric::UsePolarity};
  bool xld_model_{false};
  std::vector<ShapeModelLevel> levels_;
  double default_origin_row_{0.0};
  double default_origin_column_{0.0};
  double origin_row_{0.0};
  double origin_column_{0.0};
  double angle_start_{0.0};
  double angle_extent_{0.0};
  double angle_step_{0.0};
  double scale_min_{1.0};
  double scale_max_{1.0};
  double scale_step_{0.0};
  double min_contrast_{1.0};
  double timeout_ms_{-1.0};
  bool border_shape_models_{false};
  bool model_cache_enabled_{true};
  std::string metric_name_{"use_polarity"};
  std::pmr::vector<ShapeModelPoint> refinement_points_{
      mv::runtime::persist_resource()};
  std::shared_ptr<ShapeModelSearchCache> search_cache_;
};

struct MatchCandidate {
  double row{0.0};
  double column{0.0};
  double angle{0.0};
  double scale{1.0};
  double score{0.0};
};

inline ShapeModelLevel::ShapeModelLevel(const ShapeModelLevel& other)
    : level(other.level),
      pyramid_scale(other.pyramid_scale),
      points(other.points, mv::runtime::persist_resource()),
      contours(other.contours),
      radius(other.radius) {}

inline ShapeModelLevel& ShapeModelLevel::operator=(const ShapeModelLevel& other) {
  if (this == &other) return *this;
  level = other.level;
  pyramid_scale = other.pyramid_scale;
  points = std::pmr::vector<ShapeModelPoint>(other.points,
                                            mv::runtime::persist_resource());
  contours = other.contours;
  radius = other.radius;
  return *this;
}

inline ShapeModel::ShapeModel(const ShapeModel& other)
    : kind_(other.kind_),
      metric_(other.metric_),
      xld_model_(other.xld_model_),
      levels_(other.levels_),
      default_origin_row_(other.default_origin_row_),
      default_origin_column_(other.default_origin_column_),
      origin_row_(other.origin_row_),
      origin_column_(other.origin_column_),
      angle_start_(other.angle_start_),
      angle_extent_(other.angle_extent_),
      angle_step_(other.angle_step_),
      scale_min_(other.scale_min_),
      scale_max_(other.scale_max_),
      scale_step_(other.scale_step_),
      min_contrast_(other.min_contrast_),
      timeout_ms_(other.timeout_ms_),
      border_shape_models_(other.border_shape_models_),
      model_cache_enabled_(other.model_cache_enabled_),
      metric_name_(other.metric_name_),
      refinement_points_(other.refinement_points_,
                         mv::runtime::persist_resource()),
      search_cache_(other.search_cache_) {}

inline ShapeModel& ShapeModel::operator=(const ShapeModel& other) {
  if (this == &other) return *this;
  kind_ = other.kind_;
  metric_ = other.metric_;
  xld_model_ = other.xld_model_;
  levels_ = other.levels_;
  default_origin_row_ = other.default_origin_row_;
  default_origin_column_ = other.default_origin_column_;
  origin_row_ = other.origin_row_;
  origin_column_ = other.origin_column_;
  angle_start_ = other.angle_start_;
  angle_extent_ = other.angle_extent_;
  angle_step_ = other.angle_step_;
  scale_min_ = other.scale_min_;
  scale_max_ = other.scale_max_;
  scale_step_ = other.scale_step_;
  min_contrast_ = other.min_contrast_;
  timeout_ms_ = other.timeout_ms_;
  border_shape_models_ = other.border_shape_models_;
  model_cache_enabled_ = other.model_cache_enabled_;
  metric_name_ = other.metric_name_;
  refinement_points_ = std::pmr::vector<ShapeModelPoint>(
      other.refinement_points_, mv::runtime::persist_resource());
  search_cache_ = other.search_cache_;
  return *this;
}

inline bool ShapeModel::payloads_use_persist_resource() const noexcept {
  auto* persist = mv::runtime::persist_resource();
  if (refinement_points_.get_allocator().resource() != persist) return false;
  for (const auto& level : levels_) {
    if (level.points.get_allocator().resource() != persist) return false;
  }
  return true;
}

}  // namespace mv::shape_matching
