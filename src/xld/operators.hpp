#pragma once

#include <span>
#include <vector>
#include <core/xld.hpp>
#include <core/image.hpp>
#include <core/region.hpp>
#include <core/regions.hpp>

namespace mv::xld {

enum class PointOrder { Positive, Negative };
enum class RegionContourMode { Center, Border, BorderHoles };
enum class XldSelectionFeature { ContourLength, MaximumExtent, Direction, Curvature, Closed, Open };
enum class ContourSegmentationMode { Lines, LinesCircles, LinesEllipses };
enum class LineFitAlgorithm { Regression, Huber, Tukey, Drop, Gaussian };
struct LineFitOptions { LineFitAlgorithm algorithm{LineFitAlgorithm::Tukey}; int max_num_points{-1}; double clipping_factor{2.0}; int iterations{5}; int clipping_end_points{0}; };
struct LineFitResult { mv::core::Point2d begin; mv::core::Point2d end; double normal_row{0.0}; double normal_col{0.0}; double distance{0.0}; };
enum class CircleFitAlgorithm { Algebraic, AlgebraicHuber, AlgebraicTukey, Geometric, GeometricHuber, GeometricTukey };
struct CircleFitOptions { CircleFitAlgorithm algorithm{CircleFitAlgorithm::AlgebraicTukey}; int max_num_points{-1}; double clipping_factor{2.0}; int iterations{5}; double min_arc_angle{0.0}; int clipping_end_points{0}; double max_closure_dist{-1}; bool halcon_controls{false}; };
struct CircleFitResult { double row{0.0}; double column{0.0}; double radius{0.0}; double start_phi{0.0}; double end_phi{0.0}; PointOrder point_order{PointOrder::Positive}; double rms{}, arc_angle{}, information_ratio{}; };
enum class EdgeFilter { Canny, Sobel };

[[nodiscard]] mv::core::XldContours gen_contour_polygon_xld(std::span<const mv::core::Point2d> points, bool closed=false);
[[nodiscard]] mv::core::XldContours gen_circle_contour_xld(double row,double col,double radius,double start_phi,double end_phi,PointOrder order,double resolution);
[[nodiscard]] double length_xld(const mv::core::XldContour& contour) noexcept;
[[nodiscard]] std::vector<double> length_xld(const mv::core::XldContours& xld);
[[nodiscard]] mv::core::XldContours gen_contour_region_xld(const mv::core::Region& region, RegionContourMode mode);
[[nodiscard]] mv::core::XldContours gen_contour_region_xld(const mv::core::Regions& regions, RegionContourMode mode);
[[nodiscard]] mv::core::XldContours select_contours_xld(const mv::core::XldContours& contours, XldSelectionFeature feature, double min1, double max1, double min2=0.0, double max2=0.0);
[[nodiscard]] mv::core::XldContours smooth_contours_xld(const mv::core::XldContours& contours, int num_regr_points);
[[nodiscard]] mv::core::XldContours segment_contours_xld(const mv::core::XldContours& contours, ContourSegmentationMode mode, int smooth_cont, double max_line_dist);
[[nodiscard]] mv::core::XldContours segment_contours_xld(const mv::core::XldContours&,ContourSegmentationMode,int,double max_line_dist1,double max_line_dist2);
[[nodiscard]] LineFitResult fit_line_contour_xld(const mv::core::XldContour& contour, const LineFitOptions& options = {});
[[nodiscard]] std::vector<LineFitResult> fit_line_contour_xld(const mv::core::XldContours& contours, const LineFitOptions& options = {});
[[nodiscard]] CircleFitResult fit_circle_contour_xld(const mv::core::XldContour& contour, const CircleFitOptions& options = {});
[[nodiscard]] std::vector<CircleFitResult> fit_circle_contour_xld(const mv::core::XldContours& contours, const CircleFitOptions& options = {});
[[nodiscard]] std::vector<CircleFitResult> fit_circle_contour_xld_diagnostics(const mv::core::XldContours&,const CircleFitOptions& = {});
[[nodiscard]] mv::core::XldContours edges_sub_pix(const mv::core::Image& image, EdgeFilter filter, double alpha, double low, double high);

[[nodiscard]] mv::core::XldContours affine_trans_contour_xld(const mv::core::XldContours& contours,const mv::core::Tuple& matrix);
[[nodiscard]] mv::core::XldContours projective_trans_contour_xld(const mv::core::XldContours& contours,const mv::core::Tuple& matrix);
[[nodiscard]] mv::core::XldContours close_contours_xld(const mv::core::XldContours& contours);
enum class ContourRegionMode { Filled, Margin };
// 筛选复用内部数值测量，不构造临时 Tuple。
struct ContourAreaMeasurement { double area=0, row=0, col=0; bool positive=true; };
// 面积筛选复用同一测量路径，避免构造控制 Tuple。
[[nodiscard]] bool contour_area_in_range(const core::XldContour& contour, double lower, double upper);
[[nodiscard]] ContourAreaMeasurement measure_contour_area(const mv::core::XldContour& contour, bool centers);
struct AreaCenterXldResult { mv::core::Tuple area, row, column, point_order; };
[[nodiscard]] AreaCenterXldResult area_center_xld(const mv::core::XldContours& contours, bool centers = true);
[[nodiscard]] mv::core::Regions gen_region_contour_xld(const mv::core::XldContours& contours, ContourRegionMode mode);
enum class XldAttributeMode { Keep, Forget };
[[nodiscard]] mv::core::XldContours union_adjacent_contours_xld(const mv::core::XldContours&, double max_dist_abs, double max_dist_rel, XldAttributeMode);
[[nodiscard]] mv::core::XldContours union_collinear_contours_xld(const mv::core::XldContours&, double max_dist_abs, double max_dist_rel, double max_shift, double max_angle, XldAttributeMode);
enum class XldSortMode { UpperLeft, UpperRight, LowerLeft, LowerRight, Character };
enum class XldSortOrder { Ascending, Descending };
enum class XldSortAxis { Row, Column };
[[nodiscard]] mv::core::XldContours crop_contours_xld(const mv::core::XldContours&,double row1,double column1,double row2,double column2,bool close_contours);
[[nodiscard]] mv::core::XldContours sort_contours_xld(const mv::core::XldContours&,XldSortMode,XldSortOrder,XldSortAxis);
enum class EllipseFitAlgorithm { Algebraic, Geometric, Huber, Tukey, AlgebraicHuber, AlgebraicTukey };
struct EllipseFitOptions { EllipseFitAlgorithm algorithm{EllipseFitAlgorithm::Huber}; int max_num_points{-1}; double clipping_factor{2}; int iterations{20}; double min_arc_angle{0.2}; int clipping_end_points{0}; double max_closure_dist{-1}; bool halcon_controls{false}; };
struct EllipseFitResult {double row{},column{},phi{},radius1{},radius2{},start_phi{},end_phi{};PointOrder point_order{PointOrder::Positive};};
[[nodiscard]] mv::core::XldContours gen_ellipse_contour_xld(double row,double column,double phi,double radius1,double radius2,double start,double end,PointOrder order,double resolution);
[[nodiscard]] std::vector<EllipseFitResult> fit_ellipse_contour_xld(const mv::core::XldContours&,const EllipseFitOptions&);
enum class ContourDistanceMode { PointToPoint, PointToSegment };
enum class IntersectionMode { All, Mutual, Self };
struct DistanceResult { mv::core::Tuple minimum,maximum; };
struct IntersectionResult { mv::core::Tuple row,column,overlapping; };
[[nodiscard]] DistanceResult distance_pc(const mv::core::XldContours&,const mv::core::Tuple& row,const mv::core::Tuple& column);
[[nodiscard]] DistanceResult distance_cc(const mv::core::XldContours&,const mv::core::XldContours&,ContourDistanceMode);
[[nodiscard]] IntersectionResult intersection_contours_xld(const mv::core::XldContours&,const mv::core::XldContours&,IntersectionMode);
enum class LinePolarity { Light, Dark };
[[nodiscard]] mv::core::XldContours threshold_sub_pix(const mv::core::Image&,double threshold);
[[nodiscard]] mv::core::XldContours lines_gauss(const mv::core::Image&,double sigma,double low,double high,LinePolarity);
[[nodiscard]] mv::core::XldContours select_shape_xld(const mv::core::XldContours&,const mv::core::Tuple& features,bool all,const mv::core::Tuple& minimum,const mv::core::Tuple& maximum);
struct CocircularOptions {double max_arc_angle_diff{0.5},max_arc_overlap{0.1},max_tangent_angle{0.5},max_dist{10},max_radius_diff{1},max_center_dist{1};bool merge_small_contours{false};int iterations{-1};std::size_t min_num_points{9};};
[[nodiscard]] mv::core::XldContours union_cocircular_contours_xld(const mv::core::XldContours&,const CocircularOptions&,XldAttributeMode);
} // namespace mv::xld
