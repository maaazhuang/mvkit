#include <imageproc/operators.hpp>
#include <imageproc/advanced.hpp>

#include <cmath>
#include <utility>
#include <vector>

#include <core/broadcast.hpp>

namespace mv::imageproc {
void validate_arithmetic_controls(double mult, double add, const char* operation);
void validate_paint_controls(const mv::core::Tuple& gray, PaintMode mode);
namespace {

template <class Fn>
mv::core::IconicTuple map_images(const mv::core::IconicTuple& images,
                                 std::string_view operation, Fn&& fn) {
  const auto values = images.images(operation);
  std::vector<mv::core::Image> output;
  output.reserve(values.size());
  for (const auto& image : values) output.push_back(fn(image));
  return mv::core::make_image_tuple(std::move(output));
}

template <class Fn>
mv::core::IconicTuple map_image_pairs(const mv::core::IconicTuple& left,
                                      const mv::core::IconicTuple& right,
                                      std::string_view operation, Fn&& fn) {
  const auto left_values = left.images(operation);
  const auto right_values = right.images(operation);
  if (left_values.empty() || right_values.empty()) {
    return mv::core::make_image_tuple({});
  }
  const auto plan = mv::core::BroadcastPlan::make(
      {{left_values.size(), "Image1", true},
       {right_values.size(), "Image2", true}}, operation);
  std::vector<mv::core::Image> output;
  output.reserve(plan.size());
  for (std::size_t index = 0; index < plan.size(); ++index) {
    output.push_back(fn(left_values[plan.source_index(0, index)],
                        right_values[plan.source_index(1, index)]));
  }
  return mv::core::make_image_tuple(std::move(output));
}

// HALCON 整数图像先对 Add 取整再运算；混合整数/real 平面为 SDK 存储扩展，
// 逐平面正式实现认证前保留原标量内核行为。
double arithmetic_offset(const mv::core::Image& image,double add) {
  for(std::size_t c=0;c<image.channel_count();++c)
    if(image.pixel_type(c)==mv::core::PixelType::Float32)return add;
  return std::round(add);
}

}  // namespace

mv::core::IconicTuple abs_diff_image(const mv::core::IconicTuple& a, const mv::core::IconicTuple& b, double mult) {
  validate_arithmetic_controls(mult,0,"abs_diff_image");
  return map_image_pairs(a,b,"abs_diff_image",[&](const auto& x,const auto& y) { return abs_diff_image(x,y,mult); });
}
mv::core::IconicTuple gauss_filter_standard(const mv::core::IconicTuple& images,std::size_t size) {
  advanced::check(size>=1&&size<=63&&(size&1),"gauss_filter","Size","odd size in [1,63] required");
  return map_images(images,"gauss_filter",[&](const auto& image){return gauss_filter_standard(image,size);});
}
mv::core::IconicTuple laplace_of_gauss_standard(const mv::core::IconicTuple& images,double sigma) {
  advanced::sigma(sigma,"laplace_of_gauss");
  return map_images(images,"laplace_of_gauss",[&](const auto& image){return laplace_of_gauss_standard(image,sigma);});
}
mv::core::IconicTuple div_image(const mv::core::IconicTuple& a, const mv::core::IconicTuple& b, double mult,double add) {
  validate_arithmetic_controls(mult,add,"div_image");
  return map_image_pairs(a,b,"div_image",[&](const auto& x,const auto& y) { return div_image(x,y,mult,arithmetic_offset(x,add)); });
}
mv::core::IconicTuple invert_image(const mv::core::IconicTuple& images) {
  return map_images(images,"invert_image",[](const auto& image) { return invert_image(image); });
}
mv::core::IconicTuple crop_domain(const mv::core::IconicTuple& images) {
  return map_images(images,"crop_domain",[](const auto& image) { return crop_domain(image); });
}
mv::core::IconicTuple paint_region(const mv::core::Regions& regions,const mv::core::IconicTuple& images,const mv::core::Tuple& gray,PaintMode mode) {
  validate_paint_controls(gray,mode);
  return map_images(images,"paint_region",[&](const auto& image) { return paint_region(regions,image,gray,mode); });
}

mv::core::IconicTuple reduce_domain(const mv::core::IconicTuple& images,
                                    const mv::core::Regions& regions) {
  const auto image_values = images.images("reduce_domain");
  if (image_values.empty() || regions.empty()) return mv::core::make_image_tuple({});
  const auto plan = mv::core::BroadcastPlan::make(
      {{image_values.size(), "Image", true},
       {regions.size(), "Region", true}},
      "reduce_domain");
  std::vector<mv::core::Image> output;
  output.reserve(plan.size());
  for (std::size_t index = 0; index < plan.size(); ++index) {
    output.push_back(reduce_domain(image_values[plan.source_index(0, index)],
                                   regions[plan.source_index(1, index)]));
  }
  return mv::core::make_image_tuple(std::move(output));
}

mv::core::IconicTuple mean_image(const mv::core::IconicTuple& images,
                                 std::size_t width, std::size_t height) {
  return map_images(images, "mean_image", [&](const auto& image) {
    return mean_image(image, width, height);
  });
}

mv::core::IconicTuple gauss_filter(const mv::core::IconicTuple& images,
                                   std::size_t size) {
  advanced::check(size>=1&&size<=63&&(size&1),"gauss_filter","Size","odd size in [1,63] required");
  return map_images(images, "gauss_filter", [&](const auto& image) {
    return gauss_filter(image, size);
  });
}

mv::core::IconicTuple median_image(const mv::core::IconicTuple& images,
                                   MedianMask mask, std::size_t radius,
                                   BorderMode border, double constant_gray) {
  return map_images(images, "median_image", [&](const auto& image) {
    return median_image(image, mask, radius, border, constant_gray);
  });
}

mv::core::IconicTuple scale_image(const mv::core::IconicTuple& images,
                                  double mult, double add) {
  return map_images(images, "scale_image", [&](const auto& image) {
    return scale_image(image, mult, add);
  });
}

mv::core::IconicTuple add_image(const mv::core::IconicTuple& left,
                                const mv::core::IconicTuple& right,
                                double mult, double add) {
  validate_arithmetic_controls(mult,add,"add_image");
  left.require_kind(mv::core::IconicKind::Image,"add_image");
  right.require_kind(mv::core::IconicKind::Image,"add_image");
  if(left.empty()||right.empty())return {};
  if(right.size()!=1&&left.size()!=right.size())
    throw mv::core::Exception(mv::core::Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,
        "add_image","Image2",-1,"right input must contain one object or match the left input count");
  std::vector<mv::core::Image> result;result.reserve(left.size());
  for(std::size_t i=0;i<left.size();++i)
    result.push_back(add_image_halcon(left.at(i).as_image(),right.at(right.size()==1?0:i).as_image(),mult,add));
  return mv::core::make_image_tuple(std::move(result));
}

mv::core::IconicTuple sub_image(const mv::core::IconicTuple& left,
                                const mv::core::IconicTuple& right,
                                double mult, double add) {
  return map_image_pairs(left, right, "sub_image", [&](const auto& a, const auto& b) {
    return sub_image(a, b, mult, arithmetic_offset(a,add));
  });
}

mv::core::IconicTuple mult_image(const mv::core::IconicTuple& left,
                                 const mv::core::IconicTuple& right,
                                 double mult, double add) {
  return map_image_pairs(left, right, "mult_image", [&](const auto& a, const auto& b) {
    return mult_image(a, b, mult, arithmetic_offset(a,add));
  });
}

mv::core::IconicTuple dyn_threshold(const mv::core::IconicTuple& original,
                                    const mv::core::IconicTuple& threshold_images,
                                    double offset, DynThresholdMode mode) {
  const auto originals = original.images("dyn_threshold");
  const auto thresholds = threshold_images.images("dyn_threshold");
  if (originals.empty() || thresholds.empty()) return {};
  const auto plan = mv::core::BroadcastPlan::make(
      {{originals.size(), "OrigImage", true},
       {thresholds.size(), "ThresholdImage", true}}, "dyn_threshold");
  std::vector<mv::core::IconicValue> output;
  output.reserve(plan.size());
  for (std::size_t index = 0; index < plan.size(); ++index) {
    output.push_back(mv::core::IconicValue::from_region(
        dyn_threshold(originals[plan.source_index(0, index)],
                      thresholds[plan.source_index(1, index)], offset, mode)));
  }
  return mv::core::IconicTuple{std::move(output)};
}

mv::core::IconicTuple sobel_amp(const mv::core::IconicTuple& images,
                                SobelFilter filter, std::size_t size) {
  advanced::check(size>=3&&size<=31&&(size&1),"sobel_amp","Size","odd size in [3,31] required");
  return map_images(images, "sobel_amp", [&](const auto& image) {
    return sobel_amp(image, filter, size);
  });
}

mv::core::IconicTuple crop_rectangle1(const mv::core::IconicTuple& images,
                                      std::int64_t row1, std::int64_t col1,
                                      std::int64_t row2, std::int64_t col2) {
  return map_images(images, "crop_rectangle1", [&](const auto& image) {
    return crop_rectangle1(image, row1, col1, row2, col2);
  });
}

mv::core::IconicTuple mirror_image(const mv::core::IconicTuple& images,
                                   MirrorMode mode) {
  return map_images(images, "mirror_image", [&](const auto& image) {
    return mirror_image(image, mode);
  });
}

mv::core::IconicTuple zoom_image_size(const mv::core::IconicTuple& images,
                                      std::size_t width, std::size_t height,
                                      Interpolation interpolation) {
  advanced::dimensions(width,height,"zoom_image_size");
  advanced::interpolation(interpolation,"zoom_image_size",true);
  return map_images(images, "zoom_image_size", [&](const auto& image) {
    return zoom_image_size(image, width, height, interpolation);
  });
}

mv::core::IconicTuple rgb1_to_gray(const mv::core::IconicTuple& images) {
  return map_images(images, "rgb1_to_gray", [&](const auto& image) {
    return rgb1_to_gray(image);
  });
}

}  // namespace mv::imageproc
