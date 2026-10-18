#include <imageproc/operators.hpp>

// 图像定义域仅限制有效输出位置，像素矩阵存储保持共享。

namespace mv::imageproc {

mv::core::Region get_domain(const mv::core::Image& image) {
  return image.domain();
}

mv::core::IconicTuple get_domain(const mv::core::IconicTuple& images) {
  images.require_kind(mv::core::IconicKind::Image, "get_domain");
  if (images.size() == 1)
    return mv::core::IconicTuple{mv::core::IconicValue::from_region(get_domain(images.at(0).as_image()))};
  std::vector<mv::core::IconicValue> domains;
  domains.reserve(images.size());
  for (const auto& image : images.values()) {
    domains.push_back(mv::core::IconicValue::from_region(get_domain(image.as_image())));
  }
  return mv::core::IconicTuple{std::move(domains)};
}

mv::core::Image reduce_domain(const mv::core::Image& image, const mv::core::Region& region) {
  return image.reduce_domain(region);
}

mv::core::Image full_domain(const mv::core::Image& image) {
  return image.full_domain();
}

mv::core::IconicTuple full_domain(const mv::core::IconicTuple& images) {
  const auto image_values = images.images("full_domain");
  std::vector<mv::core::Image> output;
  output.reserve(image_values.size());
  for (const auto& image : image_values) output.push_back(full_domain(image));
  return mv::core::make_image_tuple(std::move(output));
}

}  // namespace mv::imageproc
