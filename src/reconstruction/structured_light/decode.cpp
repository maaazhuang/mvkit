#include <reconstruction/structured_light/decode.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>
#if !defined(_M_ARM64EC) && (defined(_M_X64) || defined(__SSE2__))
#include <emmintrin.h>
#define MVKIT_DECODE_SSE2 1
#else
#define MVKIT_DECODE_SSE2 0
#endif

#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <core/region.hpp>
#include <reconstruction/quality.hpp>
#include <runtime/execution/execution_context.hpp>

namespace mv::reconstruction {
namespace {

struct DirectionIndices {
  PatternOrientation orientation{PatternOrientation::Vertical};
  std::vector<std::size_t> gray;
  std::vector<std::size_t> gray_inverted;
  std::vector<std::size_t> phase;
  std::vector<std::size_t> single_stripe;
};

struct ReadPlane {const std::uint8_t* data; std::size_t stride;};
struct WritePlane {std::uint8_t* data; std::size_t stride;};
std::uint8_t gray8(const ReadPlane& image, std::size_t row, std::size_t col) {
  return image.data[row*image.stride+col];
}
void set_gray8(const WritePlane& image, std::size_t row, std::size_t col, std::uint8_t value) {
  image.data[row*image.stride+col]=value;
}
void set_float(const WritePlane& image, std::size_t row, std::size_t col, float value) {
  reinterpret_cast<float*>(image.data+row*image.stride)[col]=value;
}

std::uint64_t gray_to_binary(std::uint64_t gray) {
  gray^=gray>>32;gray^=gray>>16;gray^=gray>>8;gray^=gray>>4;gray^=gray>>2;gray^=gray>>1;return gray;
}
// UInt8 相位差具有有限精确取值域，缓存同一 libm atan2 结果，不近似相位或改变解包裹算术。
const std::vector<double>& phase_angles() {
  static const auto table=[] {std::vector<double> values(511*511);
    for(int y=-255;y<=255;++y)for(int x=-255;x<=255;++x){double phase=std::atan2(double(y),double(x));if(phase<0)phase+=2.*std::numbers::pi;values[std::size_t(y+255)*511+std::size_t(x+255)]=phase;}return values;}();
  return table;
}

std::vector<DirectionIndices> direction_indices(const StructuredLightModel& model) {
  std::vector<DirectionIndices> dirs;
  const auto add = [&](PatternOrientation orientation) {
    DirectionIndices d;
    d.orientation = orientation;
    for (std::size_t i = 0; i < model.pattern_metadata().size(); ++i) {
      const auto& m = model.pattern_metadata()[i];
      if (m.orientation != orientation) continue;
      if (m.kind == PatternKind::GrayBit) d.gray.push_back(i);
      else if (m.kind == PatternKind::GrayBitInverted) d.gray_inverted.push_back(i);
      else if (m.kind == PatternKind::Phase) d.phase.push_back(i);
      else if (m.kind == PatternKind::SingleStripe) d.single_stripe.push_back(i);
    }
    if (d.gray.empty()) {
      throw core::Exception(core::Status::InvalidState, "decode_structured_light_pattern", "generated pattern metadata has no Gray bits");
    }
    if (model.parameters().normalization == StructuredLightNormalization::InvertedPattern &&
        d.gray_inverted.size() != d.gray.size()) {
      throw core::Exception(core::Status::InvalidState, "decode_structured_light_pattern",
                            "inverted normalization requires one inverse image per Gray bit");
    }
    if (model.parameters().pattern_type != StructuredLightPatternType::GrayCode && d.phase.size() != 4U) {
      throw core::Exception(core::Status::InvalidState, "decode_structured_light_pattern", "phase-shift decode requires four phase images");
    }
    dirs.push_back(std::move(d));
  };
  if (model.parameters().orientation == PatternOrientation::Vertical || model.parameters().orientation == PatternOrientation::Both) add(PatternOrientation::Vertical);
  if (model.parameters().orientation == PatternOrientation::Horizontal || model.parameters().orientation == PatternOrientation::Both) add(PatternOrientation::Horizontal);
  return dirs;
}

void validate_sequence(const std::vector<core::Image>& images, const StructuredLightModel& model) {
  if (model.pattern_metadata().empty() || model.generated_patterns().empty()) {
    throw core::Exception(core::Status::InvalidState, "decode_structured_light_pattern", "generate structured-light patterns before decoding");
  }
  if (images.size() != model.pattern_metadata().size()) {
    throw core::Exception(core::Status::InvalidArgument, "decode_structured_light_pattern", "camera image count does not match generated pattern sequence");
  }
  if (images.empty()) {
    throw core::Exception(core::Status::InvalidArgument, "decode_structured_light_pattern", "camera image sequence is empty");
  }
  const auto width = images[0].width(), height = images[0].height();
  for (std::size_t i = 0; i < images.size(); ++i) {
    if (images[i].pixel_type() != core::PixelType::UInt8 || images[i].width() != width || images[i].height() != height) {
      throw core::Exception(core::Status::InvalidArgument, "decode_structured_light_pattern", "all camera images must be same-size Gray8 images");
    }
  }
}

}  // namespace

void decode_structured_light_pattern(const core::IconicTuple& camera_objects,
                                     StructuredLightModel& model) {
  const auto camera_images = camera_objects.images("decode_structured_light_pattern");
  validate_sequence(camera_images, model);
  const auto dirs = direction_indices(model);
  const std::size_t width = camera_images[0].width(), height = camera_images[0].height();
  std::vector<core::Image> correspondence;
  correspondence.reserve(dirs.size());
  for (std::size_t i = 0; i < dirs.size(); ++i) {
    correspondence.push_back(core::Image::create(width, height, core::PixelType::Float32));
  }
  auto validity = core::Image::create(width, height, core::PixelType::UInt8);
  auto contrast_image = core::Image::create(width, height, core::PixelType::Float32);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::size_t white_index = 0;
  std::size_t black_index = 0;
  if (model.parameters_.normalization == StructuredLightNormalization::Global) {
    for (std::size_t index = 0; index < model.pattern_metadata_.size(); ++index) {
      if (model.pattern_metadata_[index].kind == PatternKind::NormalizationWhite) white_index = index;
      if (model.pattern_metadata_[index].kind == PatternKind::NormalizationBlack) black_index = index;
    }
  }

  // 启动 worker 前一次解析存储；全部拥有者存活至完整解码的局部图像事务提交。
  std::vector<ReadPlane> input;
  for(const auto& image:camera_images) input.push_back({static_cast<const std::uint8_t*>(image.data()),image.stride()});
  std::vector<WritePlane> coordinates;
  for(auto& image:correspondence) coordinates.push_back({static_cast<std::uint8_t*>(image.mutable_data()),image.stride()});
  const WritePlane valid_plane{static_cast<std::uint8_t*>(validity.mutable_data()),validity.stride()};
  const WritePlane contrast_plane{static_cast<std::uint8_t*>(contrast_image.mutable_data()),contrast_image.stride()};
  const auto decode_pixel = [&](std::size_t r, std::size_t c) {
    double contrast = std::numeric_limits<double>::infinity();
    double midpoint = 0.0;
    bool valid = true;
    if (model.parameters_.normalization == StructuredLightNormalization::Global) {
      const double white = gray8(input[white_index], r, c);
      const double black = gray8(input[black_index], r, c);
      contrast = white - black;
      midpoint = 0.5 * (white + black);
      valid = mv::core::is_finite(contrast) && contrast >= model.parameters_.min_gray_difference;
    }

    for (std::size_t di = 0; di < dirs.size(); ++di) {
      const auto& d = dirs[di];
      std::size_t single_stripe_index = 0;
      if (model.parameters_.pattern_type == StructuredLightPatternType::SingleStripe) {
        double maximum = -1.0;
        for (std::size_t index = 0; index < d.single_stripe.size(); ++index) {
          const double intensity = gray8(input[d.single_stripe[index]], r, c);
          if (intensity > maximum) {
            maximum = intensity;
            single_stripe_index = index;
          }
        }
      }
      std::uint64_t gray_code = 0;
      for (std::size_t bit = 0; bit < d.gray.size(); ++bit) {
        bool light = false;
        if (model.parameters_.normalization == StructuredLightNormalization::Global) {
          light = gray8(input[d.gray[bit]], r, c) >= midpoint;
        } else {
          const double difference = static_cast<double>(gray8(input[d.gray[bit]], r, c)) -
                                    static_cast<double>(gray8(input[d.gray_inverted[bit]], r, c));
          contrast = (std::min)(contrast, std::abs(difference));
          valid = valid && std::abs(difference) >= model.parameters_.min_gray_difference;
          light = difference >= 0.0;
        }
        gray_code = (gray_code << 1U) | (light ? 1U : 0U);
      }
      if (model.parameters_.pattern_type == StructuredLightPatternType::SingleStripe) {
        const std::uint64_t gray_high =
            static_cast<std::uint64_t>(single_stripe_index) ^
            (static_cast<std::uint64_t>(single_stripe_index) >> 1U);
        gray_code |= gray_high << d.gray.size();
      }
      const std::uint64_t stripe = gray_to_binary(gray_code);
      const double period = static_cast<double>(model.parameters_.min_stripe_width);
      const double gray_cell_width = 0.5 * period;
      const double coarse_coordinate = (static_cast<double>(stripe) + 0.5) * gray_cell_width;
      double coordinate = coarse_coordinate;
      if (model.parameters().pattern_type != StructuredLightPatternType::GrayCode) {
        const double i0 = gray8(input[d.phase[0]], r, c);
        const double i90 = gray8(input[d.phase[1]], r, c);
        const double i180 = gray8(input[d.phase[2]], r, c);
        const double i270 = gray8(input[d.phase[3]], r, c);
        double phase = std::atan2(i270 - i90, i0 - i180);
        if (phase < 0.0) phase += 2.0 * std::numbers::pi;
        double phase_position = phase * period / (2.0 * std::numbers::pi) - 0.5;
        if (phase_position < 0.0) phase_position += period;
        const double period_index = std::round((coarse_coordinate - phase_position) / period);
        coordinate = period_index * period + phase_position;
      }
      const double extent = static_cast<double>(d.orientation == PatternOrientation::Vertical
                                                    ? model.parameters_.pattern_width
                                                    : model.parameters_.pattern_height);
      if (!(coordinate >= 0.0 && coordinate <= extent - 1.0 + 1e-6) || !mv::core::is_finite(coordinate)) {
        valid = false;
      }
      set_float(coordinates[di], r, c, static_cast<float>(coordinate));
    }
    set_float(contrast_plane, r, c, static_cast<float>(contrast));
    set_gray8(valid_plane, r, c, valid ? 255U : 0U);
    if (!valid) {
      for (const auto& image : coordinates) set_float(image, r, c, nan);
    }
  };

  const bool fast_global=model.parameters_.normalization==StructuredLightNormalization::Global &&
    model.parameters_.pattern_type!=StructuredLightPatternType::SingleStripe &&
    std::all_of(dirs.begin(),dirs.end(),[](const auto& d){return d.gray.size()<=16;});
  const bool phase_mode=model.parameters_.pattern_type!=StructuredLightPatternType::GrayCode;
  const auto* angles=fast_global&&phase_mode?phase_angles().data():nullptr;
  runtime::ExecutionContext::for_each_partition(
      height, runtime::ExecutionContext::saturated_work(width, height), 2048,
      [&](auto range) {
        for(std::size_t r=range.begin;r<range.end;++r) {
          if(!fast_global){for(std::size_t c=0;c<width;++c)decode_pixel(r,c);continue;}
          const auto* white=input[white_index].data+r*input[white_index].stride;
          const auto* black=input[black_index].data+r*input[black_index].stride;
          auto* vp=valid_plane.data+r*valid_plane.stride;
          auto* contrast=reinterpret_cast<float*>(contrast_plane.data+r*contrast_plane.stride);
          for(std::size_t start=0;start<width;start+=256) {
            const auto n=std::min(std::size_t(256),width-start);
            std::array<std::uint16_t,256> mid{};
            for(std::size_t c=0;c<n;++c){const auto x=start+c;mid[c]=std::uint16_t(white[x]+black[x]);contrast[x]=float(int(white[x])-int(black[x]));vp[x]=contrast[x]>=model.parameters_.min_gray_difference?255:0;}
            for(std::size_t di=0;di<dirs.size();++di) {
              const auto& d=dirs[di];std::array<std::uint16_t,256> codes{};
              for(auto bit:d.gray) {
                const auto* plane=input[bit].data+r*input[bit].stride+start;
                std::size_t c=0;
#if MVKIT_DECODE_SSE2
                for(;c+8<=n;c+=8) {
                  const auto pixels=_mm_unpacklo_epi8(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(plane+c)),_mm_setzero_si128());
                  const auto threshold=_mm_loadu_si128(reinterpret_cast<const __m128i*>(mid.data()+c));
                  const auto below=_mm_cmpgt_epi16(threshold,_mm_slli_epi16(pixels,1));
                  const auto bits=_mm_andnot_si128(below,_mm_set1_epi16(1));
                  const auto old=_mm_loadu_si128(reinterpret_cast<const __m128i*>(codes.data()+c));
                  _mm_storeu_si128(reinterpret_cast<__m128i*>(codes.data()+c),_mm_or_si128(_mm_slli_epi16(old,1),bits));
                }
#endif
                for(;c<n;++c)codes[c]=std::uint16_t((codes[c]<<1)|(2*plane[c]>=mid[c]));
              }
              auto* out=reinterpret_cast<float*>(coordinates[di].data+r*coordinates[di].stride)+start;
              const double period=static_cast<double>(model.parameters_.min_stripe_width);
              const double extent=static_cast<double>(d.orientation==PatternOrientation::Vertical?model.parameters_.pattern_width:model.parameters_.pattern_height);
              const std::uint8_t* phase[4]{};
              if(phase_mode)for(std::size_t k=0;k<4;++k)phase[k]=input[d.phase[k]].data+r*input[d.phase[k]].stride+start;
              std::size_t c=0;
#if MVKIT_DECODE_SSE2
              if(!phase_mode) {
                const auto half=_mm_set1_pd(.5);
                const auto scale=_mm_set1_pd(.5*period);
                const auto upper=_mm_set1_pd(extent-1.+1e-6);
                const auto zero=_mm_setzero_si128();
                for(;c+8<=n;c+=8) {
                  auto binary=_mm_loadu_si128(reinterpret_cast<const __m128i*>(codes.data()+c));
                  binary=_mm_xor_si128(binary,_mm_srli_epi16(binary,8));
                  binary=_mm_xor_si128(binary,_mm_srli_epi16(binary,4));
                  binary=_mm_xor_si128(binary,_mm_srli_epi16(binary,2));
                  binary=_mm_xor_si128(binary,_mm_srli_epi16(binary,1));
                  const auto low=_mm_unpacklo_epi16(binary,zero);
                  const auto high=_mm_unpackhi_epi16(binary,zero);
                  const auto pair=[&](__m128i words,std::size_t offset) {
                    // 转 Float 前保留原标量 Double 加法/乘法。
                    const auto coordinate=_mm_mul_pd(_mm_add_pd(_mm_cvtepi32_pd(words),half),scale);
                    const auto valid=_mm_and_pd(_mm_cmpge_pd(coordinate,_mm_setzero_pd()),_mm_cmple_pd(coordinate,upper));
                    // 有界 UInt16 编码及正 Int64 周期在 Double 中有限；有限上界也拒绝无界坐标。
                    const auto mask=_mm_movemask_pd(valid);
                    if(!(mask&1))vp[start+c+offset]=0;
                    if(!(mask&2))vp[start+c+offset+1]=0;
                    _mm_storel_epi64(reinterpret_cast<__m128i*>(out+c+offset),_mm_castps_si128(_mm_cvtpd_ps(coordinate)));
                  };
                  pair(low,0);pair(_mm_srli_si128(low,8),2);
                  pair(high,4);pair(_mm_srli_si128(high,8),6);
                }
              }
#endif
              for(;c<n;++c) {
                auto code=codes[c];code^=code>>8;code^=code>>4;code^=code>>2;code^=code>>1;
                const double coarse=(double(code)+.5)*(.5*period);
                double coordinate=coarse;
                if(phase_mode) {
                  const int y=int(phase[3][c])-int(phase[1][c]),x=int(phase[0][c])-int(phase[2][c]);
                  double position=angles[std::size_t(y+255)*511+std::size_t(x+255)]*period/(2.*std::numbers::pi)-.5;
                  if(position<0)position+=period;
                  coordinate=std::round((coarse-position)/period)*period+position;
                }
                if(!(coordinate>=0 && coordinate<=extent-1.+1e-6) || !mv::core::is_finite(coordinate))vp[start+c]=0;
                out[c]=static_cast<float>(coordinate);
              }
            }
            for(std::size_t c=0;c<n;++c)if(!vp[start+c])for(const auto& plane:coordinates)set_float(plane,r,start+c,nan);
          }
        }
      });

  core::Image quality_confidence,quality_reason;
  std::vector<core::Image> coordinate_sigma;
  if(model.parameters_.quality_output) {
    quality_confidence=core::Image::create(width,height,core::PixelType::Float32);
    quality_reason=core::Image::create(width,height,core::PixelType::UInt16);
    for(std::size_t di=0;di<dirs.size();++di)coordinate_sigma.push_back(core::Image::create(width,height,core::PixelType::Float32));
    const WritePlane qp{static_cast<std::uint8_t*>(quality_reason.mutable_data()),quality_reason.stride()};
    const WritePlane cf{static_cast<std::uint8_t*>(quality_confidence.mutable_data()),quality_confidence.stride()};
    std::vector<WritePlane> sigmas;
    for(auto& image:coordinate_sigma)sigmas.push_back({static_cast<std::uint8_t*>(image.mutable_data()),image.stride()});
    auto domain=camera_images[0].domain();
    for(std::size_t i=1;i<camera_images.size();++i)domain=domain.intersect(camera_images[i].domain());
    const bool full_domain=domain.area()==width*height;
    runtime::ExecutionContext::for_each_partition(height,runtime::ExecutionContext::saturated_work(width,height),2048,[&](auto range){
      for(auto r=range.begin;r<range.end;++r)for(std::size_t c=0;c<width;++c) {
        std::uint16_t reason=0;
        const auto contrast=reinterpret_cast<const float*>(contrast_plane.data+r*contrast_plane.stride)[c];
        if(!full_domain && !domain.contains(r,c))reason|=OutsideDomain;
        if(contrast<model.parameters_.min_gray_difference)reason|=LowContrast;
        if(!valid_plane.data[r*valid_plane.stride+c] && !reason)reason|=CoordinateOutOfRange;
        for(std::size_t di=0;di<dirs.size();++di) {
          double sigma=model.parameters_.projector_coordinate_stddev;
          const auto& d=dirs[di];
          if(!d.phase.empty()) {
            if(model.parameters_.saturation_margin>0)for(auto plane:d.phase){const auto intensity=gray8(input[plane],r,c);
              if(intensity<=model.parameters_.saturation_margin || intensity>=255-model.parameters_.saturation_margin)reason|=Saturated;
            }
            const double x=double(gray8(input[d.phase[0]],r,c))-gray8(input[d.phase[2]],r,c);
            const double y=double(gray8(input[d.phase[3]],r,c))-gray8(input[d.phase[1]],r,c);
            // UInt8 差不超过 255，直接平方半径不会溢出/下溢，避免通用 hypot 缩放。
            const double radius=std::sqrt(x*x+y*y),amplitude=.5*radius;
            if(!(amplitude>0) || amplitude<model.parameters_.min_phase_amplitude)reason|=WeakPhase;
            if(radius>0 && model.parameters_.intensity_stddev>0) {
              const double phase_sigma=std::sqrt(2.)*model.parameters_.intensity_stddev/radius;
              sigma=std::hypot(sigma,phase_sigma*double(model.parameters_.min_stripe_width)/(2*std::numbers::pi));
            }
          }
          set_float(sigmas[di],r,c,sigma>0 && mv::core::is_finite(static_cast<float>(sigma))?float(sigma):nan);
        }
        reinterpret_cast<std::uint16_t*>(qp.data+r*qp.stride)[c]=reason;
        set_gray8(valid_plane,r,c,reason?0:255);
        set_float(cf,r,c,reason?0:std::clamp(contrast/255.f,0.f,1.f));
        if(reason) {
          for(const auto& image:coordinates)set_float(image,r,c,nan);
          for(const auto& image:sigmas)set_float(image,r,c,nan);
        }
      }
    });
  } else if(model.parameters_.min_phase_amplitude>0 || model.parameters_.intensity_stddev>0 ||
            model.parameters_.projector_coordinate_stddev>0 || model.parameters_.hole_max_width>0 || model.parameters_.saturation_margin>0) {
    throw core::Exception(core::Status::InvalidArgument,"decode_structured_light_pattern","quality_output=true required for quality controls");
  }
  model.decoded_confidence_=std::move(quality_confidence);model.decoded_reason_=std::move(quality_reason);
  model.decoded_coordinate_stddev_=core::make_image_tuple(std::move(coordinate_sigma));
  model.decoded_correspondence_ = core::make_image_tuple(std::move(correspondence));
  model.decoded_validity_ = std::move(validity);
  model.decoded_contrast_ = std::move(contrast_image);
  model.has_decoded_state_ = true;
}

core::IconicTuple get_structured_light_object(const StructuredLightModel& model,
                                             std::string_view object_name) {
  if (!model.has_decoded_state_) {
    throw core::Exception(core::Status::InvalidState, "get_structured_light_object", "structured-light pattern has not been decoded");
  }
  if (object_name == "pattern_region") {
    std::vector<core::RegionRun> runs;
    const auto& mask = model.decoded_validity_;
    const auto* pixels = static_cast<const std::uint8_t*>(mask.data());
    for (std::size_t row = 0; row < mask.height(); ++row) {
      const auto* line = pixels + row * mask.stride();
      std::size_t col = 0;
      while (col < mask.width()) {
        while (col < mask.width() && !line[col]) ++col;
        const auto begin = col;
        while (col < mask.width() && line[col]) ++col;
        if (col > begin) runs.push_back({static_cast<std::int64_t>(row),
                                        static_cast<std::int64_t>(begin),
                                        static_cast<std::int64_t>(col - 1)});
      }
    }
    return core::IconicTuple{core::IconicValue::from_region(core::Region::from_runs(runs))};
  }
  if(object_name=="confidence_image" || object_name=="reason_image" || object_name=="coordinate_stddev_image") {
    if(!model.parameters_.quality_output)throw core::Exception(core::Status::InvalidState,"get_structured_light_object","quality_output=true required");
    if(object_name=="confidence_image")return core::make_image_tuple({model.decoded_confidence_});
    if(object_name=="reason_image")return core::make_image_tuple({model.decoded_reason_});
    return model.decoded_coordinate_stddev_;
  }
  if (object_name == "correspondence_image") return model.decoded_correspondence_;
  if (object_name == "validity_image") return core::make_image_tuple({model.decoded_validity_});
  if (object_name == "contrast_image") {
    if (!model.parameters_.persistence) {
      throw core::Exception(core::Status::InvalidState, "get_structured_light_object", "contrast_image requires persistence=true");
    }
    return core::make_image_tuple({model.decoded_contrast_});
  }
  throw core::Exception(core::Status::InvalidArgument, "get_structured_light_object", "unknown structured-light object name");
}

}  // namespace mv::reconstruction
