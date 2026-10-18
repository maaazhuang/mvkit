#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <array>
#include <span>
#include <core/tuple.hpp>

#include <core/region.hpp>
#include <core/regions.hpp>
#include <core/runtime_context.hpp>

namespace mv::regionproc {

enum class Connectivity : std::uint8_t { Four = 4, Eight = 8 };

struct AreaCenterResult { std::uint64_t area{0}; double row{0.0}; double column{0.0}; };
struct Rectangle1 { std::int64_t row1{0}; std::int64_t column1{0}; std::int64_t row2{-1}; std::int64_t column2{-1}; };

struct Rectangle2 { double row{0.0}; double column{0.0}; double phi{0.0}; double length1{0.0}; double length2{0.0}; };
struct Circle { double row{0.0}; double column{0.0}; double radius{0.0}; };

enum class ShapeFeature : std::uint8_t { Area, Row, Column, Width, Height, Ratio, Row1, Column1, Row2, Column2,
  Circularity, Orientation, Anisometry, Bulkiness, StructureFactor, Convexity, Ra, Rb, MomentRow, MomentColumn, MomentCross };
constexpr std::size_t kShapeFeatureCount=21;
using ShapeFeatures=std::array<double,kShapeFeatureCount>;
void validate_feature(ShapeFeature feature,const char* operation);
[[nodiscard]] ShapeFeatures shape_features(const mv::core::Region&,std::uint32_t mask=(1U<<kShapeFeatureCount)-1);
[[nodiscard]] ShapeFeatures halcon_shape_features(const mv::core::Region&,std::uint32_t mask);
[[nodiscard]] mv::core::Tuple region_features(const mv::core::Regions&,std::span<const ShapeFeature>);
[[nodiscard]] mv::core::Tuple circularity(const mv::core::Regions&);
[[nodiscard]] mv::core::Tuple orientation_region(const mv::core::Regions&);
struct EccentricityTuples { mv::core::Tuple anisometry,bulkiness,structure_factor; };
[[nodiscard]] EccentricityTuples eccentricity(const mv::core::Regions&);
enum class RegionSortMode : std::uint8_t { FirstPoint,LastPoint,UpperLeft,UpperRight,LowerLeft,LowerRight,Center,Character };
enum class SortOrder : std::uint8_t { Ascending,Descending };
enum class SortAxis : std::uint8_t { Row,Column };
enum class StandardShape : std::uint8_t { MaxArea, Rectangle1, Rectangle2 };
[[nodiscard]] mv::core::Regions sort_region(const mv::core::Regions&,RegionSortMode,SortOrder,SortAxis);
[[nodiscard]] mv::core::Regions select_shape_std(const mv::core::Regions&,StandardShape,double percent);
enum class SelectionOperation : std::uint8_t { And, Or };
struct ShapeCriterion { ShapeFeature feature; std::optional<double> min; std::optional<double> max; };

[[nodiscard]] Rectangle2 smallest_rectangle2(const mv::core::Region& region);
[[nodiscard]] std::vector<Rectangle2> smallest_rectangle2(const mv::core::Regions& regions);
[[nodiscard]] Circle smallest_circle(const mv::core::Region& region);
[[nodiscard]] std::vector<Circle> smallest_circle(const mv::core::Regions& regions);
[[nodiscard]] mv::core::Regions select_shape(const mv::core::Regions& regions,
                                                 const std::vector<ShapeCriterion>& criteria,
                                                 SelectionOperation operation, bool halcon_features=false);

[[nodiscard]] mv::core::Regions connection(const mv::core::Region& region, Connectivity connectivity);
[[nodiscard]] mv::core::Regions connection(const mv::core::Regions& regions, Connectivity connectivity);
[[nodiscard]] mv::core::Regions connection(const mv::core::Regions& regions,
                                            const mv::core::RuntimeContext& context);
[[nodiscard]] mv::core::Region fill_up(const mv::core::Region& region, Connectivity connectivity);
[[nodiscard]] mv::core::Regions fill_up(const mv::core::Regions& regions, Connectivity connectivity);
[[nodiscard]] mv::core::Regions fill_up(const mv::core::Regions& regions,
                                        const mv::core::RuntimeContext& context);

[[nodiscard]] AreaCenterResult area_center(const mv::core::Region& region);
[[nodiscard]] std::vector<AreaCenterResult> area_center(const mv::core::Regions& regions);
[[nodiscard]] Rectangle1 smallest_rectangle1(const mv::core::Region& region);
[[nodiscard]] std::vector<Rectangle1> smallest_rectangle1(const mv::core::Regions& regions);


[[nodiscard]] mv::core::Region gen_rectangle1(std::int64_t row1, std::int64_t column1,
                                               std::int64_t row2, std::int64_t column2);
[[nodiscard]] mv::core::Region gen_rectangle2(double row, double column, double phi,
                                               double length1, double length2);
[[nodiscard]] mv::core::Region gen_circle(double row, double column, double radius);

[[nodiscard]] mv::core::Region union1(const mv::core::Region& region);
[[nodiscard]] mv::core::Region union1(const mv::core::Regions& regions);

[[nodiscard]] mv::core::Region union2(const mv::core::Region& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Region union2(const mv::core::Region& left, const mv::core::Regions& right);
[[nodiscard]] mv::core::Regions union2(const mv::core::Regions& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Regions union2(const mv::core::Regions& left, const mv::core::Regions& right);

[[nodiscard]] mv::core::Region intersection(const mv::core::Region& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Region intersection(const mv::core::Region& left, const mv::core::Regions& right);
[[nodiscard]] mv::core::Regions intersection(const mv::core::Regions& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Regions intersection(const mv::core::Regions& left, const mv::core::Regions& right);

[[nodiscard]] mv::core::Region difference(const mv::core::Region& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Region difference(const mv::core::Region& left, const mv::core::Regions& right);
[[nodiscard]] mv::core::Regions difference(const mv::core::Regions& left, const mv::core::Region& right);
[[nodiscard]] mv::core::Regions difference(const mv::core::Regions& left, const mv::core::Regions& right);


[[nodiscard]] mv::core::Region dilation_circle(const mv::core::Region&, double radius);
[[nodiscard]] mv::core::Regions dilation_circle(const mv::core::Regions&, double radius);
[[nodiscard]] mv::core::Region erosion_circle(const mv::core::Region&, double radius);
[[nodiscard]] mv::core::Regions erosion_circle(const mv::core::Regions&, double radius);
[[nodiscard]] mv::core::Region opening_circle(const mv::core::Region&, double radius);
[[nodiscard]] mv::core::Regions opening_circle(const mv::core::Regions&, double radius);
[[nodiscard]] mv::core::Region closing_circle(const mv::core::Region&, double radius);
[[nodiscard]] mv::core::Regions closing_circle(const mv::core::Regions&, double radius);

[[nodiscard]] mv::core::Region dilation_rectangle1(const mv::core::Region&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Regions dilation_rectangle1(const mv::core::Regions&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Region erosion_rectangle1(const mv::core::Region&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Regions erosion_rectangle1(const mv::core::Regions&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Region opening_rectangle1(const mv::core::Region&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Regions opening_rectangle1(const mv::core::Regions&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Region closing_rectangle1(const mv::core::Region&, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Regions closing_rectangle1(const mv::core::Regions&, std::size_t width, std::size_t height);

[[nodiscard]] mv::core::Regions move_region(const mv::core::Regions&, std::int64_t row, std::int64_t column);
[[nodiscard]] mv::core::Regions symm_difference(const mv::core::Regions&, const mv::core::Regions&);
[[nodiscard]] mv::core::Tuple test_equal_region(const mv::core::Regions&, const mv::core::Regions&);
[[nodiscard]] mv::core::Tuple test_region_point(const mv::core::Regions&, const mv::core::Tuple& rows, const mv::core::Tuple& columns);
[[nodiscard]] mv::core::Region gen_region_polygon_filled(const mv::core::Tuple& rows, const mv::core::Tuple& columns, bool halcon_raster = false);
[[nodiscard]] mv::core::Region gen_ellipse(double row,double column,double phi,double radius1,double radius2);
enum class ShapeTransform : std::uint8_t { Convex, Rectangle1 };
enum class RegionBoundary : std::uint8_t { Inner, InnerFilled, Outer };
enum class DistanceMetric : std::uint8_t { Euclidean, CityBlock, Chessboard };
[[nodiscard]] mv::core::Regions shape_trans(const mv::core::Regions&,ShapeTransform,bool halcon_boundary=false);
[[nodiscard]] mv::core::Regions affine_trans_region(const mv::core::Regions&,const mv::core::Tuple& matrix);
[[nodiscard]] mv::core::Regions boundary(const mv::core::Regions&,RegionBoundary,Connectivity);
[[nodiscard]] mv::core::Regions fill_up_shape(const mv::core::Regions&,ShapeFeature,double minimum,double maximum,Connectivity, bool halcon_features=false);
[[nodiscard]] mv::core::Regions skeleton(const mv::core::Regions&);
[[nodiscard]] mv::core::IconicTuple distance_transform(const mv::core::Regions&,DistanceMetric,bool foreground,std::size_t width,std::size_t height);
[[nodiscard]] mv::core::Regions partition_rectangle(const mv::core::Regions&,double width,double height, bool halcon_layout = false);
[[nodiscard]] mv::core::Regions partition_dynamic(const mv::core::Regions&,double distance,double percent, bool halcon_layout = false);
[[nodiscard]] mv::core::Regions select_rectangles(const mv::core::Regions&,StandardShape,double percent);
[[nodiscard]] mv::core::Regions sort_characters(const mv::core::Regions&,SortOrder,SortAxis,double overlap=.25,bool coordinate_overlap=false);
[[nodiscard]] mv::core::Regions dilation1(const mv::core::Regions&,const mv::core::Regions& element,std::int64_t anchor_row,std::int64_t anchor_column,std::size_t iterations);
[[nodiscard]] mv::core::Regions erosion1(const mv::core::Regions&,const mv::core::Regions& element,std::int64_t anchor_row,std::int64_t anchor_column,std::size_t iterations);
[[nodiscard]] mv::core::Regions opening(const mv::core::Regions&,const mv::core::Regions& element,std::int64_t anchor_row,std::int64_t anchor_column,std::size_t iterations);
[[nodiscard]] mv::core::Regions closing(const mv::core::Regions&,const mv::core::Regions& element,std::int64_t anchor_row,std::int64_t anchor_column,std::size_t iterations);
}  // namespace mv::regionproc
