#include <reconstruction/structured_light/pattern.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

std::size_t gray_bit_count(std::int64_t extent, std::int64_t stripe_width) {
  const auto gray_cell_width = stripe_width / 2;
  const std::uint64_t stripes = static_cast<std::uint64_t>(
      (extent + gray_cell_width - 1) / gray_cell_width);
  std::size_t bits = 0;
  std::uint64_t capacity = 1;
  while (capacity < stripes) {
    capacity <<= 1U;
    ++bits;
  }
  return bits;
}

template<class Value>
core::Image directional_image(std::size_t width, std::size_t height,
                              PatternOrientation orientation, Value value) {
  const core::PixelType types[]{core::PixelType::UInt8};
  auto image=core::Image::create_uninitialized(width,height,types);
  auto* data=static_cast<std::uint8_t*>(image.mutable_data());
  const auto stride=image.stride();
  if(orientation==PatternOrientation::Vertical) {
    for(std::size_t c=0;c<width;++c)data[c]=value(c);
    for(std::size_t r=1;r<height;++r)std::memcpy(data+r*stride,data,width);
  } else {
    for(std::size_t r=0;r<height;++r)std::memset(data+r*stride,value(r),width);
  }
  return image;
}
core::Image constant_image(std::size_t w,std::size_t h,std::uint8_t value) {
  return directional_image(w,h,PatternOrientation::Horizontal,[&](auto){return value;});
}
core::Image gray_image(std::size_t w,std::size_t h,std::int64_t stripe_width,
                       PatternOrientation orientation,std::size_t bit,std::size_t bits,bool inverted) {
  return directional_image(w,h,orientation,[&](std::size_t coordinate){
    const auto stripe=coordinate/static_cast<std::uint64_t>(stripe_width/2);
    const auto gray=stripe^(stripe>>1U);
    const bool light=((gray>>(bits-1-bit))&1U)!=0;
    return static_cast<std::uint8_t>(light!=inverted?255:0);
  });
}
core::Image phase_image(std::size_t w,std::size_t h,std::int64_t stripe_width,
                        PatternOrientation orientation,int phase_index) {
  const double offset=static_cast<double>(phase_index)*std::numbers::pi/2.0;
  return directional_image(w,h,orientation,[&](std::size_t coordinate){
    const double phase=2.0*std::numbers::pi*(static_cast<double>(coordinate)+.5)/static_cast<double>(stripe_width)+offset;
    return static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(127.0*(1.0+std::cos(phase)))),0,255));
  });
}
core::Image single_stripe_image(std::size_t w,std::size_t h,std::int64_t stripe_width,
                                PatternOrientation orientation,std::size_t stripe_index) {
  const auto begin=stripe_index*static_cast<std::size_t>(stripe_width);
  const auto end=begin+static_cast<std::size_t>(stripe_width);
  return directional_image(w,h,orientation,[&](std::size_t coordinate){return static_cast<std::uint8_t>(coordinate>=begin&&coordinate<end?255:0);});
}

void append_single_stripe_direction(std::vector<core::Image>& images,
                                    std::vector<PatternMetadata>& metadata,
                                    const StructuredLightParameters& p,
                                    PatternOrientation orientation) {
  const auto extent = static_cast<std::size_t>(
      orientation == PatternOrientation::Vertical ? p.pattern_width : p.pattern_height);
  const auto stripe_width = static_cast<std::size_t>(p.single_stripe_width);
  const auto count = (extent + stripe_width - 1) / stripe_width;
  for (std::size_t index = 0; index < count; ++index) {
    images.push_back(single_stripe_image(static_cast<std::size_t>(p.pattern_width),
                                         static_cast<std::size_t>(p.pattern_height),
                                         p.single_stripe_width, orientation, index));
    metadata.push_back({orientation, PatternKind::SingleStripe,
                        static_cast<std::int64_t>(index), -1});
  }
}

void append_gray_direction(std::vector<core::Image>& images, std::vector<PatternMetadata>& metadata,
                           const StructuredLightParameters& p, PatternOrientation orientation) {
  const std::int64_t extent = p.pattern_type == StructuredLightPatternType::SingleStripe
                                  ? p.single_stripe_width
                                  : (orientation == PatternOrientation::Vertical
                                         ? p.pattern_width : p.pattern_height);
  const std::size_t bits = gray_bit_count(extent, p.min_stripe_width);
  for (std::size_t bit = 0; bit < bits; ++bit) {
    images.push_back(gray_image(static_cast<std::size_t>(p.pattern_width), static_cast<std::size_t>(p.pattern_height),
                                p.min_stripe_width, orientation, bit, bits, false));
    metadata.push_back({orientation, PatternKind::GrayBit, static_cast<std::int64_t>(bit), -1});
    if (p.normalization == StructuredLightNormalization::InvertedPattern) {
      images.push_back(gray_image(static_cast<std::size_t>(p.pattern_width), static_cast<std::size_t>(p.pattern_height),
                                  p.min_stripe_width, orientation, bit, bits, true));
      metadata.push_back({orientation, PatternKind::GrayBitInverted,
                          static_cast<std::int64_t>(bit), -1});
    }
  }
}

void append_phase_direction(std::vector<core::Image>& images, std::vector<PatternMetadata>& metadata,
                            const StructuredLightParameters& p, PatternOrientation orientation) {
  for (int phase = 0; phase < 4; ++phase) {
    images.push_back(phase_image(static_cast<std::size_t>(p.pattern_width), static_cast<std::size_t>(p.pattern_height),
                                 p.min_stripe_width, orientation, phase));
    metadata.push_back({orientation, PatternKind::Phase, -1, phase});
  }
}

}  // namespace

core::IconicTuple gen_structured_light_pattern(StructuredLightModel& model) {
  const auto& p = model.parameters_;
  if (p.pattern_width <= 0 || p.pattern_height <= 0) {
    throw core::Exception(core::Status::InvalidState, "gen_structured_light_pattern", "pattern_width and pattern_height must be configured");
  }
  std::vector<core::Image> candidate;
  std::vector<PatternMetadata> metadata;
  if (p.pattern_type == StructuredLightPatternType::SingleStripe &&
      (p.orientation == PatternOrientation::Vertical || p.orientation == PatternOrientation::Both)) {
    append_single_stripe_direction(candidate, metadata, p, PatternOrientation::Vertical);
  }
  if (p.pattern_type == StructuredLightPatternType::SingleStripe &&
      (p.orientation == PatternOrientation::Horizontal || p.orientation == PatternOrientation::Both)) {
    append_single_stripe_direction(candidate, metadata, p, PatternOrientation::Horizontal);
  }
  if (p.normalization == StructuredLightNormalization::Global) {
    candidate.push_back(constant_image(static_cast<std::size_t>(p.pattern_width), static_cast<std::size_t>(p.pattern_height), 255));
    metadata.push_back({PatternOrientation::Vertical, PatternKind::NormalizationWhite, -1, -1});
    candidate.push_back(constant_image(static_cast<std::size_t>(p.pattern_width), static_cast<std::size_t>(p.pattern_height), 0));
    metadata.push_back({PatternOrientation::Vertical, PatternKind::NormalizationBlack, -1, -1});
  }

  if (p.orientation == PatternOrientation::Vertical || p.orientation == PatternOrientation::Both) {
    append_gray_direction(candidate, metadata, p, PatternOrientation::Vertical);
  }
  if (p.orientation == PatternOrientation::Horizontal || p.orientation == PatternOrientation::Both) {
    append_gray_direction(candidate, metadata, p, PatternOrientation::Horizontal);
  }
  if (p.pattern_type != StructuredLightPatternType::GrayCode &&
      (p.orientation == PatternOrientation::Vertical || p.orientation == PatternOrientation::Both)) {
    append_phase_direction(candidate, metadata, p, PatternOrientation::Vertical);
  }
  if (p.pattern_type != StructuredLightPatternType::GrayCode &&
      (p.orientation == PatternOrientation::Horizontal || p.orientation == PatternOrientation::Both)) {
    append_phase_direction(candidate, metadata, p, PatternOrientation::Horizontal);
  }

  auto objects = core::make_image_tuple(std::move(candidate));
  model.generated_patterns_ = objects;
  model.pattern_metadata_ = std::move(metadata);
  model.decoded_correspondence_ = {};
  model.decoded_validity_ = {};
  model.decoded_contrast_ = {};
  model.has_decoded_state_ = false;
  return objects;
}

}  // namespace mv::reconstruction
