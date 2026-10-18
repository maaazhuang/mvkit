#include <imageproc/advanced.hpp>
#include <core/broadcast.hpp>
namespace mv::imageproc {
mv::core::IconicTuple affine_trans_image(const mv::core::IconicTuple& image,
                                         const mv::core::Tuple& matrix,
                                         std::size_t width, std::size_t height,
                                         Interpolation mode) {
  (void)advanced::inverse(matrix, true, "affine_trans_image");
  advanced::interpolation(mode, "affine_trans_image");
  advanced::dimensions(width, height, "affine_trans_image");
  const auto image_values = image.images("affine_trans_image");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result =
        affine_trans_image(image_values[i], matrix, width, height, mode);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple projective_trans_image(const mv::core::IconicTuple& image,
                                             const mv::core::Tuple& matrix,
                                             std::size_t width,
                                             std::size_t height,
                                             Interpolation mode) {
  (void)advanced::inverse(matrix, false, "projective_trans_image");
  advanced::interpolation(mode, "projective_trans_image");
  advanced::dimensions(width, height, "projective_trans_image");
  const auto image_values = image.images("projective_trans_image");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result =
        projective_trans_image(image_values[i], matrix, width, height, mode);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple rotate_image(const mv::core::IconicTuple& image,
                                   double angle, Interpolation mode) {
  advanced::interpolation(mode, "rotate_image");
  advanced::check(std::isfinite(angle), "rotate_image", "angle",
                  "control must be finite");
  const auto image_values = image.images("rotate_image");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = rotate_image(image_values[i], angle, mode);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple map_image(const mv::core::IconicTuple& image,
                                const mv::core::IconicTuple& row_map,
                                const mv::core::IconicTuple& col_map,
                                Interpolation mode) {
  advanced::interpolation(mode, "map_image");
  const auto image_values = image.images("map_image");
  const auto row_map_values = row_map.images("map_image");
  const auto col_map_values = col_map.images("map_image");
  if (image_values.empty() || row_map_values.empty() || col_map_values.empty())
    return {};
  const auto plan =
      mv::core::BroadcastPlan::make({{image_values.size(), "image", true},
                                     {row_map_values.size(), "row_map", true},
                                     {col_map_values.size(), "col_map", true}},
                                    "map_image");
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    auto result = map_image(image_values[plan.source_index(0, i)],
                            row_map_values[plan.source_index(1, i)],
                            col_map_values[plan.source_index(2, i)], mode);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple polar_trans_image_ext(
    const mv::core::IconicTuple& image, double center_row, double center_col,
    double angle_start, double angle_end, double radius_start,
    double radius_end, std::size_t width, std::size_t height,
    Interpolation mode) {
  advanced::interpolation(mode, "polar_trans_image_ext");
  advanced::dimensions(width, height, "polar_trans_image_ext");
  advanced::check(std::isfinite(center_row), "polar_trans_image_ext",
                  "center_row", "control must be finite");
  advanced::check(std::isfinite(center_col), "polar_trans_image_ext",
                  "center_col", "control must be finite");
  advanced::check(std::isfinite(angle_start), "polar_trans_image_ext",
                  "angle_start", "control must be finite");
  advanced::check(std::isfinite(angle_end), "polar_trans_image_ext",
                  "angle_end", "control must be finite");
  advanced::check(std::isfinite(radius_start), "polar_trans_image_ext",
                  "radius_start", "control must be finite");
  advanced::check(std::isfinite(radius_end), "polar_trans_image_ext",
                  "radius_end", "control must be finite");
  advanced::check(angle_end > angle_start &&
                      angle_end - angle_start <= 6.283185307179586 + 1e-12 &&
                      radius_start >= 0 && radius_end > radius_start,
                  "polar_trans_image_ext", "Range", "invalid polar ranges");
  const auto image_values = image.images("polar_trans_image_ext");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = polar_trans_image_ext(image_values[i], center_row, center_col,
                                        angle_start, angle_end, radius_start,
                                        radius_end, width, height, mode);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple gray_erosion_rect(const mv::core::IconicTuple& image,
                                        std::size_t width, std::size_t height) {
  advanced::window(width, height, "gray_erosion_rect");
  const auto image_values = image.images("gray_erosion_rect");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = gray_erosion_rect(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple gray_dilation_rect(const mv::core::IconicTuple& image,
                                         std::size_t width,
                                         std::size_t height) {
  advanced::window(width, height, "gray_dilation_rect");
  const auto image_values = image.images("gray_dilation_rect");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = gray_dilation_rect(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple gray_opening_rect(const mv::core::IconicTuple& image,
                                        std::size_t width, std::size_t height) {
  advanced::window(width, height, "gray_opening_rect");
  const auto image_values = image.images("gray_opening_rect");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = gray_opening_rect(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple gray_closing_rect(const mv::core::IconicTuple& image,
                                        std::size_t width, std::size_t height) {
  advanced::window(width, height, "gray_closing_rect");
  const auto image_values = image.images("gray_closing_rect");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = gray_closing_rect(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple top_hat(const mv::core::IconicTuple& image,
                              std::size_t width, std::size_t height) {
  advanced::window(width, height, "top_hat");
  const auto image_values = image.images("top_hat");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = top_hat(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple bottom_hat(const mv::core::IconicTuple& image,
                                 std::size_t width, std::size_t height) {
  advanced::window(width, height, "bottom_hat");
  const auto image_values = image.images("bottom_hat");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = bottom_hat(image_values[i], width, height);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple trans_from_rgb(const mv::core::IconicTuple& image) {
  const auto image_values = image.images("trans_from_rgb");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = trans_from_rgb(image_values[i]);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple trans_to_rgb(const mv::core::IconicTuple& image,
                                   mv::core::PixelType type) {
  advanced::check(type == mv::core::PixelType::UInt8 ||
                      type == mv::core::PixelType::UInt16 ||
                      type == mv::core::PixelType::Float32,
                  "trans_to_rgb", "Type", "invalid RGB type");
  const auto image_values = image.images("trans_to_rgb");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = trans_to_rgb(image_values[i], type);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple derivate_gauss(const mv::core::IconicTuple& image,
                                     double sigma, std::size_t order_x,
                                     std::size_t order_y) {
  advanced::check(std::isfinite(sigma), "derivate_gauss", "sigma",
                  "control must be finite");
  advanced::sigma(sigma, "derivate_gauss");
  advanced::check(order_x <= 2 && order_y <= 2 && order_x + order_y <= 2,
                  "derivate_gauss", "Order", "total order must be <=2");
  const auto image_values = image.images("derivate_gauss");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = derivate_gauss(image_values[i], sigma, order_x, order_y);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple laplace_of_gauss(const mv::core::IconicTuple& image,
                                       double sigma) {
  advanced::check(std::isfinite(sigma), "laplace_of_gauss", "sigma",
                  "control must be finite");
  advanced::sigma(sigma, "laplace_of_gauss");
  const auto image_values = image.images("laplace_of_gauss");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = laplace_of_gauss(image_values[i], sigma);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple emphasize(const mv::core::IconicTuple& image,
                                std::size_t width, std::size_t height,
                                double factor) {
  advanced::window(width, height, "emphasize");
  advanced::check(std::isfinite(factor), "emphasize", "factor",
                  "control must be finite");
  advanced::check(factor >= 0, "emphasize", "Factor",
                  "factor must be nonnegative");
  const auto image_values = image.images("emphasize");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = emphasize(image_values[i], width, height, factor);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple auto_threshold(const mv::core::IconicTuple& image,
                                     double sigma) {
  advanced::check(std::isfinite(sigma), "auto_threshold", "sigma",
                  "control must be finite");
  advanced::check(sigma >= 0 && sigma <= 32, "auto_threshold", "Sigma",
                  "sigma must be in [0,32]");
  const auto image_values = image.images("auto_threshold");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = auto_threshold(image_values[i], sigma);
    for (const auto& value : result.iconic_tuple().values())
      output.push_back(value);
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple local_threshold(const mv::core::IconicTuple& image,
                                      std::size_t width, std::size_t height,
                                      double k, double dynamic_range,
                                      ThresholdPolarity polarity) {
  advanced::window(width, height, "local_threshold");
  advanced::check(std::isfinite(k), "local_threshold", "k",
                  "control must be finite");
  advanced::check(std::isfinite(dynamic_range), "local_threshold",
                  "dynamic_range", "control must be finite");
  advanced::check(k >= 0 && k <= 1 && dynamic_range > 0 &&
                      (polarity == ThresholdPolarity::Light ||
                       polarity == ThresholdPolarity::Dark),
                  "local_threshold", "Parameters", "invalid Sauvola controls");
  const auto image_values = image.images("local_threshold");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = local_threshold(image_values[i], width, height, k,
                                  dynamic_range, polarity);
    output.push_back(mv::core::IconicValue::from_region(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
mv::core::IconicTuple equ_histo_image(const mv::core::IconicTuple& image) {
  const auto image_values = image.images("equ_histo_image");
  if (image_values.empty()) return {};
  std::vector<mv::core::IconicValue> output;
  for (std::size_t i = 0; i < image_values.size(); ++i) {
    auto result = equ_histo_image(image_values[i]);
    output.push_back(mv::core::IconicValue::from_image(std::move(result)));
  }
  return mv::core::IconicTuple{std::move(output)};
}
}  // namespace mv::imageproc
