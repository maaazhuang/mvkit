#pragma once

#include <cstdint>
#include <matching/multi.hpp>
#include <string>
#include <vector>

#include <core/image.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <shape_matching/model.hpp>

namespace mv::shape_matching {

struct CreateParameters {
  int levels{}, metric{}, reduction{1};
  double angle_start{}, angle_extent{}, angle_step{};
  double scale_min{1}, scale_max{1}, scale_step{};
  double contrast_low{-1}, contrast_high{-1}, min_contrast{-1};
  size_t min_size{};
};
[[nodiscard]] ShapeModel create_model(const core::Image&,const CreateParameters&);
struct XldCreateParameters {
  int levels{}, metric{2}, reduction{1};
  double angle_start{}, angle_extent{}, angle_step{};
  double scale_min{1}, scale_max{1}, scale_step{}, min_contrast{10};
};
[[nodiscard]] ShapeModel create_model_xld(const core::XldContours&, const XldCreateParameters&);
[[nodiscard]] ShapeModel create_shape_model_xld(const core::XldContours&, const core::Tuple&,
    double, double, const core::Tuple&, const core::Tuple&, std::string, double);
[[nodiscard]] ShapeModel create_scaled_shape_model_xld(const core::XldContours&, const core::Tuple&,
    double, double, const core::Tuple&, double, double, const core::Tuple&, const core::Tuple&,
    std::string, double);
struct SearchParameters {
  double angle_start{}, angle_extent{}, scale_min{1}, scale_max{1};
  double min_score{.5}, max_overlap{.5}, greediness{.9}, timeout_ms{};
  int64_t num_matches{1};
  int subpixel{1}, max_deformation{};
  int scale_search{-1}; // -1 为私有旧模型类型分派，0/1 为正式算子。
  size_t coarsest{}, finest{};
  bool operator==(const SearchParameters&) const = default;
};
[[nodiscard]] std::vector<MatchCandidate> find_model(const core::Image&,const ShapeModel&,const SearchParameters&);
std::vector<matching::Match<MatchCandidate>> find_models(const core::Image&,
    std::span<const ShapeModel* const>,std::span<const SearchParameters>,matching::Selection,
    std::span<const core::Image* const> model_images = {});

[[nodiscard]] ShapeModel create_shape_model(const mv::core::Image& image,
                                            const mv::core::Tuple& num_levels,
                                            double angle_start,
                                            double angle_extent,
                                            const mv::core::Tuple& angle_step,
                                            const mv::core::Tuple& optimization,
                                            std::string metric,
                                            const mv::core::Tuple& contrast,
                                            const mv::core::Tuple& min_contrast);

[[nodiscard]] ShapeModel create_scaled_shape_model(const mv::core::Image& image,
                                                   const mv::core::Tuple& num_levels,
                                                   double angle_start,
                                                   double angle_extent,
                                                   const mv::core::Tuple& angle_step,
                                                   double scale_min,
                                                   double scale_max,
                                                   const mv::core::Tuple& scale_step,
                                                   const mv::core::Tuple& optimization,
                                                   std::string metric,
                                                   const mv::core::Tuple& contrast,
                                                   const mv::core::Tuple& min_contrast);

[[nodiscard]] std::vector<MatchCandidate> find_shape_model(const mv::core::Image& image,
                                                           const ShapeModel& model,
                                                           double angle_start,
                                                           double angle_extent,
                                                           double min_score,
                                                           std::int64_t num_matches,
                                                           double max_overlap,
                                                           const mv::core::Tuple& sub_pixel,
                                                           const mv::core::Tuple& num_levels,
                                                           double greediness, double timeout_ms = 0);

[[nodiscard]] std::vector<MatchCandidate> find_scaled_shape_model(const mv::core::Image& image,
                                                                  const ShapeModel& model,
                                                                  double angle_start,
                                                                  double angle_extent,
                                                                  double scale_min,
                                                                  double scale_max,
                                                                  double min_score,
                                                                  std::int64_t num_matches,
                                                                  double max_overlap,
                                                                  const mv::core::Tuple& sub_pixel,
                                                                  const mv::core::Tuple& num_levels,
                                                                  double greediness, double timeout_ms = 0);

[[nodiscard]] mv::core::XldContours get_shape_model_contours(const ShapeModel& model, std::int64_t level);

void set_shape_model_param(ShapeModel& model, const mv::core::Tuple& names, const mv::core::Tuple& values);

void get_shape_model_params(const ShapeModel& model,
                            mv::core::Tuple* num_levels,
                            double* angle_start,
                            double* angle_extent,
                            double* angle_step,
                            double* scale_min,
                            double* scale_max,
                            double* scale_step,
                            mv::core::Tuple* metric,
                            mv::core::Tuple* min_contrast);

void write_model(const ShapeModel& model, std::string_view path);
[[nodiscard]] ShapeModel read_model(std::string_view path);
// 内存快照与模型文件采用相同格式，不保存搜索缓存。
[[nodiscard]] std::string serialize_model(const ShapeModel& model);
[[nodiscard]] ShapeModel deserialize_model(std::string_view bytes);

}  // namespace mv::shape_matching
