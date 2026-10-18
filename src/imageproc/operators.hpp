#pragma once

#include <cstdint>

#include <core/image.hpp>
#include <core/iconic_tuple.hpp>
#include <core/region.hpp>
#include <core/regions.hpp>
#include <core/tuple.hpp>
#include <imageproc/border.hpp>

namespace mv::imageproc {


enum class ThresholdPolarity : std::uint8_t { Light, Dark };
[[nodiscard]] mv::core::Regions watersheds_marker(const mv::core::IconicTuple&,
    const mv::core::Regions&, int connectivity = 8);
struct IntensityResult { double mean{}; double deviation{}; };
struct HistogramResult { mv::core::Tuple absolute; mv::core::Tuple relative; };
struct BinaryThresholdResult { mv::core::Region region; double threshold{}; };
struct IntensityTuples { mv::core::Tuple mean; mv::core::Tuple deviation; };
struct BinaryThresholdTuples { mv::core::IconicTuple regions; mv::core::Tuple thresholds; };

/** @brief 类型转换保持通道、对象和定义域；整数转换截断并饱和，NaN 转零。 */
[[nodiscard]] mv::core::Image convert_image_type(const mv::core::Image&, mv::core::PixelType);
[[nodiscard]] mv::core::IconicTuple convert_image_type(const mv::core::IconicTuple&, mv::core::PixelType);
/** @brief 返回共享像素存储的单通道视图，channel 使用 1-based 索引。 */
[[nodiscard]] mv::core::Image access_channel(const mv::core::Image&, std::size_t channel);
[[nodiscard]] mv::core::IconicTuple access_channel(const mv::core::IconicTuple&, std::size_t channel);
/** @brief 合并三个单通道图像，输出定义域为交集，允许各通道不同类型。 */
[[nodiscard]] mv::core::Image compose3(const mv::core::Image&, const mv::core::Image&, const mv::core::Image&);
[[nodiscard]] mv::core::IconicTuple compose3(const mv::core::IconicTuple&, const mv::core::IconicTuple&, const mv::core::IconicTuple&);
[[nodiscard]] IntensityResult intensity(const mv::core::Region&, const mv::core::Image&);
[[nodiscard]] IntensityTuples intensity(const mv::core::Regions&, const mv::core::IconicTuple&);
[[nodiscard]] HistogramResult gray_histo(const mv::core::Region&, const mv::core::Image&);
// HALCON 标准语义：显式 ROI，忽略图像定义域，支持全部标量类型。
[[nodiscard]] HistogramResult gray_histo_standard(const mv::core::Region&, const mv::core::Image&);
[[nodiscard]] HistogramResult gray_histo(const mv::core::Regions&, const mv::core::IconicTuple&);
[[nodiscard]] BinaryThresholdResult binary_threshold(const mv::core::Image&, ThresholdPolarity);
[[nodiscard]] BinaryThresholdTuples binary_threshold(const mv::core::IconicTuple&, ThresholdPolarity);

enum class MedianMask : std::uint8_t { Square, Circle };
enum class PaintMode : std::uint8_t { Fill, Margin };
[[nodiscard]] mv::core::Image abs_diff_image(const mv::core::Image&, const mv::core::Image&, double mult);
[[nodiscard]] mv::core::IconicTuple abs_diff_image(const mv::core::IconicTuple&, const mv::core::IconicTuple&, double mult);
[[nodiscard]] mv::core::Image div_image(const mv::core::Image&, const mv::core::Image&, double mult, double add);
[[nodiscard]] mv::core::IconicTuple div_image(const mv::core::IconicTuple&, const mv::core::IconicTuple&, double mult, double add);
[[nodiscard]] mv::core::Image invert_image(const mv::core::Image&);
[[nodiscard]] mv::core::IconicTuple invert_image(const mv::core::IconicTuple&);
[[nodiscard]] mv::core::Image crop_domain(const mv::core::Image&);
[[nodiscard]] mv::core::IconicTuple crop_domain(const mv::core::IconicTuple&);
[[nodiscard]] mv::core::Image paint_region(const mv::core::Regions&, const mv::core::Image&, const mv::core::Tuple&, PaintMode);
[[nodiscard]] mv::core::IconicTuple paint_region(const mv::core::Regions&, const mv::core::IconicTuple&, const mv::core::Tuple&, PaintMode);
enum class SobelFilter : std::uint8_t { SumAbs, SumSqrt, X, Y };
enum class MirrorMode : std::uint8_t { Row, Column, Diagonal };
enum class Interpolation : std::uint8_t { NearestNeighbor, Bilinear, Constant, Weighted, Bicubic };
// HALCON 行列像素角点变换及编码线性 Map。
[[nodiscard]] mv::core::Image transform_image_halcon(const mv::core::Image& image, const mv::core::Tuple& matrix, Interpolation mode, bool projective, bool adapt_size, bool transform_domain);
[[nodiscard]] mv::core::Image map_image_linear(const mv::core::Image& image, const mv::core::Image& map);
[[nodiscard]] mv::core::Image polar_trans_image_halcon(const mv::core::Image& image, double row, double col, double start_angle, double end_angle, double start_radius, double end_radius, std::size_t width, std::size_t height, Interpolation mode);


enum class DynThresholdMode : std::uint8_t {
  Light,
  Dark,
  Equal,
  NotEqual,
};

[[nodiscard]] mv::core::Region get_domain(const mv::core::Image& image);
[[nodiscard]] mv::core::IconicTuple get_domain(const mv::core::IconicTuple& images);
[[nodiscard]] mv::core::Image reduce_domain(const mv::core::Image& image, const mv::core::Region& region);
[[nodiscard]] mv::core::IconicTuple reduce_domain(const mv::core::IconicTuple& images, const mv::core::Regions& regions);
[[nodiscard]] mv::core::Image full_domain(const mv::core::Image& image);
[[nodiscard]] mv::core::IconicTuple full_domain(const mv::core::IconicTuple& images);


struct MinMaxGrayResult {
  double min{0.0};
  double max{0.0};
  double range{0.0};
};

struct MinMaxGrayTuples {
  mv::core::Tuple min;
  mv::core::Tuple max;
  mv::core::Tuple range;
};


[[nodiscard]] mv::core::Image sobel_amp(const mv::core::Image& image, SobelFilter filter, std::size_t size);
[[nodiscard]] mv::core::IconicTuple sobel_amp(const mv::core::IconicTuple& images, SobelFilter filter, std::size_t size);
[[nodiscard]] mv::core::Image crop_rectangle1(const mv::core::Image& image, std::int64_t row1, std::int64_t col1, std::int64_t row2, std::int64_t col2);
[[nodiscard]] mv::core::IconicTuple crop_rectangle1(const mv::core::IconicTuple& images, std::int64_t row1, std::int64_t col1, std::int64_t row2, std::int64_t col2);
[[nodiscard]] mv::core::Image mirror_image(const mv::core::Image& image, MirrorMode mode);
[[nodiscard]] mv::core::IconicTuple mirror_image(const mv::core::IconicTuple& images, MirrorMode mode);
[[nodiscard]] mv::core::Image zoom_image_size(const mv::core::Image& image, std::size_t width, std::size_t height, Interpolation interpolation);
[[nodiscard]] mv::core::IconicTuple zoom_image_size(const mv::core::IconicTuple& images, std::size_t width, std::size_t height, Interpolation interpolation);
[[nodiscard]] mv::core::Image rgb1_to_gray(const mv::core::Image& image);
[[nodiscard]] mv::core::IconicTuple rgb1_to_gray(const mv::core::IconicTuple& images);

[[nodiscard]] mv::core::Image scale_image(const mv::core::Image& image, double mult, double add);
[[nodiscard]] mv::core::IconicTuple scale_image(const mv::core::IconicTuple& images, double mult, double add);
[[nodiscard]] mv::core::Image add_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add);
// 已收敛公开算子的 HALCON 整数控制/舍入语义。
[[nodiscard]] mv::core::Image add_image_halcon(const mv::core::Image& a, const mv::core::Image& b, double mult, double add);
[[nodiscard]] mv::core::IconicTuple add_image(const mv::core::IconicTuple& a, const mv::core::IconicTuple& b, double mult, double add);
[[nodiscard]] mv::core::Image sub_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add);
[[nodiscard]] mv::core::IconicTuple sub_image(const mv::core::IconicTuple& a, const mv::core::IconicTuple& b, double mult, double add);
[[nodiscard]] mv::core::Image mult_image(const mv::core::Image& a, const mv::core::Image& b, double mult, double add);
[[nodiscard]] mv::core::IconicTuple mult_image(const mv::core::IconicTuple& a, const mv::core::IconicTuple& b, double mult, double add);
[[nodiscard]] MinMaxGrayResult min_max_gray(const mv::core::Region& region, const mv::core::Image& image, double percent);
[[nodiscard]] MinMaxGrayTuples min_max_gray(const mv::core::Regions& regions, const mv::core::IconicTuple& images, double percent);

[[nodiscard]] mv::core::Image mean_image(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple mean_image(const mv::core::IconicTuple& images, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image gauss_filter(const mv::core::Image& image, std::size_t size);
[[nodiscard]] mv::core::Image gauss_filter_standard(const mv::core::Image&,std::size_t);
[[nodiscard]] mv::core::IconicTuple gauss_filter_standard(const mv::core::IconicTuple&,std::size_t);
[[nodiscard]] mv::core::IconicTuple gauss_filter(const mv::core::IconicTuple& images, std::size_t size);
[[nodiscard]] mv::core::Image median_image(const mv::core::Image& image, MedianMask mask, std::size_t radius, BorderMode border, double constant_gray = 0.0);
[[nodiscard]] mv::core::IconicTuple median_image(const mv::core::IconicTuple& images, MedianMask mask, std::size_t radius, BorderMode border, double constant_gray = 0.0);

[[nodiscard]] mv::core::Region threshold(const mv::core::Image& image, double min_gray, double max_gray);
[[nodiscard]] mv::core::IconicTuple threshold(const mv::core::IconicTuple& images,
                                               const mv::core::Tuple& min_gray,
                                               const mv::core::Tuple& max_gray);
[[nodiscard]] mv::core::Region dyn_threshold(const mv::core::Image& orig,
                                              const mv::core::Image& threshold_image,
                                              double offset,
                                              DynThresholdMode mode);
[[nodiscard]] mv::core::IconicTuple dyn_threshold(const mv::core::IconicTuple& orig,
                                                   const mv::core::IconicTuple& threshold_images,
                                                   double offset,
                                                   DynThresholdMode mode);

[[nodiscard]] mv::core::Image affine_trans_image(const mv::core::Image& image, const mv::core::Tuple& matrix, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::IconicTuple affine_trans_image(const mv::core::IconicTuple& image, const mv::core::Tuple& matrix, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::Image projective_trans_image(const mv::core::Image& image, const mv::core::Tuple& matrix, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::IconicTuple projective_trans_image(const mv::core::IconicTuple& image, const mv::core::Tuple& matrix, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::Image rotate_image(const mv::core::Image& image, double angle, Interpolation mode);
[[nodiscard]] mv::core::IconicTuple rotate_image(const mv::core::IconicTuple& image, double angle, Interpolation mode);
[[nodiscard]] mv::core::Image map_image(const mv::core::Image& image, const mv::core::Image& row_map, const mv::core::Image& col_map, Interpolation mode);
[[nodiscard]] mv::core::IconicTuple map_image(const mv::core::IconicTuple& image, const mv::core::IconicTuple& row_map, const mv::core::IconicTuple& col_map, Interpolation mode);
[[nodiscard]] mv::core::Image polar_trans_image_ext(const mv::core::Image& image, double center_row, double center_col, double angle_start, double angle_end, double radius_start, double radius_end, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::IconicTuple polar_trans_image_ext(const mv::core::IconicTuple& image, double center_row, double center_col, double angle_start, double angle_end, double radius_start, double radius_end, std::size_t width, std::size_t height, Interpolation mode);
[[nodiscard]] mv::core::Image gray_erosion_rect(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple gray_erosion_rect(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image gray_dilation_rect(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple gray_dilation_rect(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image gray_opening_rect(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple gray_opening_rect(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image gray_closing_rect(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple gray_closing_rect(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image top_hat(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple top_hat(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image bottom_hat(const mv::core::Image& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::IconicTuple bottom_hat(const mv::core::IconicTuple& image, std::size_t width, std::size_t height);
[[nodiscard]] mv::core::Image trans_from_rgb(const mv::core::Image& image);
[[nodiscard]] mv::core::IconicTuple trans_from_rgb(const mv::core::IconicTuple& image);
[[nodiscard]] mv::core::Image trans_to_rgb(const mv::core::Image& image, mv::core::PixelType type);
[[nodiscard]] mv::core::IconicTuple trans_to_rgb(const mv::core::IconicTuple& image, mv::core::PixelType type);
[[nodiscard]] mv::core::Image derivate_gauss(const mv::core::Image& image, double sigma, std::size_t order_x, std::size_t order_y);
[[nodiscard]] mv::core::Image derivate_gauss(const mv::core::Image& image, double sigma_x, double sigma_y, std::size_t order_x, std::size_t order_y);
[[nodiscard]] mv::core::IconicTuple derivate_gauss(const mv::core::IconicTuple& image, double sigma, std::size_t order_x, std::size_t order_y);
[[nodiscard]] mv::core::Image laplace_of_gauss(const mv::core::Image& image, double sigma);
[[nodiscard]] mv::core::Image laplace_of_gauss_standard(const mv::core::Image&,double);
[[nodiscard]] mv::core::IconicTuple laplace_of_gauss_standard(const mv::core::IconicTuple&,double);
[[nodiscard]] mv::core::IconicTuple laplace_of_gauss(const mv::core::IconicTuple& image, double sigma);
[[nodiscard]] mv::core::Image emphasize(const mv::core::Image& image, std::size_t width, std::size_t height, double factor);
[[nodiscard]] mv::core::IconicTuple emphasize(const mv::core::IconicTuple& image, std::size_t width, std::size_t height, double factor);
[[nodiscard]] mv::core::Regions auto_threshold(const mv::core::Image& image, double sigma);
[[nodiscard]] mv::core::IconicTuple auto_threshold(const mv::core::IconicTuple& image, double sigma);
[[nodiscard]] mv::core::Region local_threshold(const mv::core::Image& image, std::size_t width, std::size_t height, double k, double dynamic_range, ThresholdPolarity polarity);
[[nodiscard]] mv::core::IconicTuple local_threshold(const mv::core::IconicTuple& image, std::size_t width, std::size_t height, double k, double dynamic_range, ThresholdPolarity polarity);
[[nodiscard]] mv::core::Image equ_histo_image(const mv::core::Image& image);
[[nodiscard]] mv::core::IconicTuple equ_histo_image(const mv::core::IconicTuple& image);

}  // namespace mv::imageproc
