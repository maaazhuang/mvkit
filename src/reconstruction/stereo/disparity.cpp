#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <reconstruction/quality.hpp>
#include <reconstruction/stereo/census.hpp>
#include <reconstruction/stereo/cost_semantics.hpp>
#include <reconstruction/stereo/cost_volume.hpp>
#include <reconstruction/stereo/disparity.hpp>
#include <reconstruction/stereo/sgm.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <string>
#include <vector>

#include "winner.hpp"

namespace mv::reconstruction {
namespace {

enum class Similarity { CensusDense, CensusSparse, Sad };
enum class Method { Accurate, Fast, VeryFast };

struct Options {
  Similarity similarity{Similarity::CensusDense};
  Method method{Method::Accurate};
  bool quality{false};
  double min_confidence{}, disparity_stddev{};
  int hole_max_width{};
  double hole_max_depth_jump{.01};
  bool consistency{true};
  bool subpixel{true};
  double consistency_tolerance{5.0};
  int paths{8};
  int disparity_offset{3};
  int num_levels{0};
};

constexpr std::int32_t kBadCost = kStereoBadCost;

bool parse_boolean(const core::Value& value) {
  if (value.kind() != core::ValueKind::String) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "boolean generic parameters must be 'true' or 'false'");
  }
  const std::string text(value.as_string().view());
  if (text == "true") return true;
  if (text == "false") return false;
  throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                        "boolean generic parameters must be 'true' or 'false'");
}

int parse_integer(const core::Value& value, const char* name, bool allow_zero, int maximum) {
  if (value.kind() != core::ValueKind::Int64) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          std::string(name) + " must be an integer");
  }
  const auto number = value.as_int64();
  if (number < (allow_zero ? 0 : 1) || number > maximum) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          std::string(name) + " is out of range");
  }
  return static_cast<int>(number);
}

Options parse_options(const core::Tuple& names, const core::Tuple& values) {
  if (names.size() != values.size()) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "generic parameter tuple sizes must match");
  }
  Options options;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string name(names.string_view(i));
    const auto& value = values.at(i);
    if (name == "similarity_measure") {
      if (value.kind() != core::ValueKind::String) {
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "similarity_measure must be a string");
      }
      const std::string text(value.as_string().view());
      if (text == "census_dense")
        options.similarity = Similarity::CensusDense;
      else if (text == "census_sparse")
        options.similarity = Similarity::CensusSparse;
      else if (text == "sad")
        options.similarity = Similarity::Sad;
      else
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "invalid similarity_measure");
    } else if (name == "method") {
      if (value.kind() != core::ValueKind::String) {
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "method must be a string");
      }
      const std::string text(value.as_string().view());
      if (text == "accurate")
        options.method = Method::Accurate;
      else if (text == "fast")
        options.method = Method::Fast;
      else if (text == "very_fast")
        options.method = Method::VeryFast;
      else
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "invalid method");
    } else if (name == "num_levels") {
      if (value.kind() == core::ValueKind::String) {
        if (value.as_string().view() != "auto") {
          throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                                "num_levels must be a positive integer or 'auto'");
        }
        options.num_levels = 0;
      } else {
        options.num_levels = parse_integer(value, "num_levels", false, 8);
      }
    } else if (name == "disparity_offset") {
      options.disparity_offset = parse_integer(value, "disparity_offset", true, INT32_MAX);
    } else if (name == "consistency_check") {
      options.consistency = parse_boolean(value);
    } else if (name == "sub_disparity") {
      options.subpixel = parse_boolean(value);
    } else if (name == "consistency_tolerance") {
      options.consistency_tolerance = value.as_real64();
      if (!std::isfinite(options.consistency_tolerance) || options.consistency_tolerance < 0) {
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "invalid consistency_tolerance");
      }
    } else if (name == "quality_output") {
      options.quality = parse_boolean(value);
    } else if (name == "min_confidence" || name == "disparity_stddev" ||
               name == "hole_max_depth_jump") {
      const double v = value.as_real64();
      if (!std::isfinite(v) || v < 0 || (name == "min_confidence" && v > 1))
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "invalid quality parameter");
      if (name == "min_confidence")
        options.min_confidence = v;
      else if (name == "disparity_stddev")
        options.disparity_stddev = v;
      else
        options.hole_max_depth_jump = v;
    } else if (name == "hole_max_width") {
      options.hole_max_width = parse_integer(value, "hole_max_width", true, 4096);
    } else if (name == "paths") {
      options.paths = static_cast<int>(value.as_int64());
      if (options.paths != 4 && options.paths != 8) {
        throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                              "paths must be 4 or 8");
      }
    } else {
      throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                            "unknown generic parameter");
    }
  }
  if (!options.quality &&
      (options.min_confidence > 0 || options.disparity_stddev > 0 || options.hole_max_width > 0))
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "quality_output=true required for quality controls");
  return options;
}

float image_float(const core::Image& image, std::size_t row, std::size_t column) {
  return reinterpret_cast<const float*>(static_cast<const std::byte*>(image.data()) +
                                        row * image.stride())[column];
}

core::Image downsample_half(const core::Image& input, std::pmr::memory_resource* memory) {
  const auto source = image_to_scalar(input, memory);
  const std::size_t width = (input.width() + 1) / 2;
  const std::size_t height = (input.height() + 1) / 2;
  auto output = core::Image::create(width, height, core::PixelType::Float32);
  for (std::size_t row = 0; row < height; ++row) {
    auto* destination = reinterpret_cast<float*>(static_cast<std::byte*>(output.mutable_data()) +
                                                 row * output.stride());
    for (std::size_t column = 0; column < width; ++column) {
      double sum = 0.0;
      std::size_t count = 0;
      for (std::size_t dr = 0; dr < 2; ++dr) {
        const std::size_t source_row = row * 2 + dr;
        if (source_row >= input.height()) continue;
        for (std::size_t dc = 0; dc < 2; ++dc) {
          const std::size_t source_column = column * 2 + dc;
          if (source_column >= input.width()) continue;
          sum += source[source_row * input.width() + source_column];
          ++count;
        }
      }
      destination[column] = static_cast<float>(sum / static_cast<double>(count));
    }
  }
  return output;
}

StereoSimilarity to_similarity(Similarity similarity) {
  switch (similarity) {
    case Similarity::CensusSparse:
      return StereoSimilarity::CensusSparse;
    case Similarity::Sad:
      return StereoSimilarity::Sad;
    case Similarity::CensusDense:
    default:
      return StereoSimilarity::CensusDense;
  }
}

DisparityResult solve_one(const core::Image& left, const core::Image& right, int min_disparity,
                          int max_disparity, int surface_smoothing, int edge_smoothing,
                          const Options& options, const core::Image* coarse_prior,
                          std::pmr::memory_resource* memory) {
  if (left.height() != right.height()) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "rectified images must have equal heights");
  }
  if (min_disparity > max_disparity) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "minimum disparity exceeds maximum");
  }
  const std::size_t width = left.width();
  const std::size_t height = left.height();
  const std::size_t disparities = static_cast<std::size_t>(max_disparity - min_disparity + 1);
  auto cost = build_cost_volume(left, right, min_disparity, max_disparity,
                                to_similarity(options.similarity), coarse_prior,
                                options.disparity_offset, memory);
  const bool use_cost_directly = surface_smoothing == 0 && edge_smoothing == 0;
  CostVolume aggregated{memory};
  if (!use_cost_directly) {
    aggregated =
        aggregate_sgm(cost, surface_smoothing, (std::max)(surface_smoothing, edge_smoothing),
                      options.paths, memory);
    cost = CostVolume{memory};
  }
  const CostVolume& labels = use_cost_directly ? cost : aggregated;
  auto label_cost = [&](std::size_t pixel, std::size_t disparity) -> std::int32_t {
    return labels.unpacked_cost(pixel, disparity);
  };
  DenseWinnerKernel dense_winner = nullptr;
  if (!labels.is_banded && labels.census.empty() && disparities >= 32) {
    const auto isa = mv::runtime::cpu_features().selection();
#if MVKIT_RECONSTRUCTION_HAS_AVX2
    if (isa.avx2) dense_winner = &dense_winner_avx2;
#endif
    if (!dense_winner && isa.sse2) dense_winner = &dense_winner_sse2;
  }
  DisparityResult output{core::Image::create(width, height, core::PixelType::Float32),
                         core::Image::create(width, height, core::PixelType::Float32)};
  if (options.quality)
    output.quality_channels = {core::Image::create(width, height, core::PixelType::Float32),
                               core::Image::create(width, height, core::PixelType::UInt8),
                               core::Image::create(width, height, core::PixelType::UInt16)};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t row = 0; row < height; ++row) {
    auto* disparity_row = reinterpret_cast<float*>(
        static_cast<std::byte*>(output.disparity.mutable_data()) + row * output.disparity.stride());
    auto* score_row = reinterpret_cast<float*>(
        static_cast<std::byte*>(output.score.mutable_data()) + row * output.score.stride());
    auto* confidence_row =
        options.quality ? reinterpret_cast<float*>(
                              static_cast<std::byte*>(output.quality_channels[0].mutable_data()) +
                              row * output.quality_channels[0].stride())
                        : nullptr;
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t pixel = row * width + column;
      const auto winner = select_stereo_winner(labels, pixel, dense_winner);
      const auto best = winner.index;
      const auto best_cost = winner.cost;
      const auto second_cost = winner.second;
      if (best_cost >= kBadCost / 2) {
        disparity_row[column] = nan;
        score_row[column] = std::numeric_limits<float>::infinity();
        continue;
      }
      if (options.quality) {
        confidence_row[column] =
            second_cost >= kBadCost / 2
                ? 1.0f
                : std::clamp(float(second_cost - best_cost) / float(std::max(1, second_cost)), 0.0f,
                             1.0f);
      }
      double delta = 0.0;
      if (options.subpixel && best > 0 && best + 1 < disparities) {
        const double before = label_cost(pixel, best - 1);
        const double center = label_cost(pixel, best);
        const double after = label_cost(pixel, best + 1);
        const double denominator = before - 2.0 * center + after;
        if (center > 0.0 && before < kBadCost / 2 && after < kBadCost / 2 &&
            std::abs(denominator) > 1e-9) {
          delta = std::clamp(0.5 * (before - after) / denominator, -1.0, 1.0);
        }
      }
      disparity_row[column] = static_cast<float>(min_disparity + static_cast<int>(best) + delta);
      score_row[column] =
          static_cast<float>(best_cost) / static_cast<float>((std::max)(1, options.paths));
    }
  }
  return output;
}

int automatic_levels(const core::Image& image, int disparity_count, Method method) {
  if ((std::min)(image.width(), image.height()) < 512 || disparity_count <= 64) return 1;
  int levels = 1;
  std::size_t minimum_dimension = (std::min)(image.width(), image.height());
  int coarse_disparities = disparity_count;
  const int maximum_levels = method == Method::VeryFast ? 4 : 3;
  while (levels < maximum_levels && minimum_dimension >= 64 && coarse_disparities > 8) {
    ++levels;
    minimum_dimension = (minimum_dimension + 1) / 2;
    coarse_disparities = (coarse_disparities + 1) / 2;
  }
  return levels;
}

DisparityResult solve_method(const core::Image& left, const core::Image& right, int min_disparity,
                             int max_disparity, int surface_smoothing, int edge_smoothing,
                             Options options, std::pmr::memory_resource* memory) {
  if (options.method == Method::Accurate) {
    return solve_one(left, right, min_disparity, max_disparity, surface_smoothing, edge_smoothing,
                     options, nullptr, memory);
  }
  int levels = options.num_levels == 0
                   ? automatic_levels(left, max_disparity - min_disparity + 1, options.method)
                   : options.num_levels;
  levels = (std::min)(levels, 8);
  std::pmr::vector<core::Image> left_pyramid{memory};
  std::pmr::vector<core::Image> right_pyramid{memory};
  left_pyramid.push_back(left);
  right_pyramid.push_back(right);
  for (int level = 1; level < levels; ++level) {
    left_pyramid.push_back(downsample_half(left_pyramid.back(), memory));
    right_pyramid.push_back(downsample_half(right_pyramid.back(), memory));
  }
  if (options.method == Method::VeryFast && levels > 1) {
    options.paths = (std::min)(options.paths, 4);
  }
  DisparityResult result;
  for (int level = levels - 1; level >= 0; --level) {
    const int scale = 1 << level;
    const int level_min = static_cast<int>(
        std::floor(static_cast<double>(min_disparity) / static_cast<double>(scale)));
    const int level_max = static_cast<int>(
        std::ceil(static_cast<double>(max_disparity) / static_cast<double>(scale)));
    const core::Image* prior = result.disparity.empty() ? nullptr : &result.disparity;
    result = solve_one(left_pyramid[static_cast<std::size_t>(level)],
                       right_pyramid[static_cast<std::size_t>(level)], level_min, level_max,
                       surface_smoothing, edge_smoothing, options, prior, memory);
  }
  return result;
}

}  // namespace

DisparityResult binocular_disparity_ms(const core::Image& left, const core::Image& right,
                                       std::int64_t min_disparity_64, std::int64_t max_disparity_64,
                                       std::int64_t surface_smoothing_64,
                                       std::int64_t edge_smoothing_64,
                                       const core::Tuple& generic_names,
                                       const core::Tuple& generic_values) {
  if (min_disparity_64 < -32768 || min_disparity_64 > 32768 || max_disparity_64 < -32768 ||
      max_disparity_64 > 32768 || min_disparity_64 > max_disparity_64 || surface_smoothing_64 < 0 ||
      edge_smoothing_64 < 0 || surface_smoothing_64 > INT32_MAX ||
      edge_smoothing_64 > INT32_MAX) {
    throw core::Exception(core::Status::InvalidArgument, "binocular_disparity_ms",
                          "parameter out of range");
  }
  const int min_disparity = static_cast<int>(min_disparity_64);
  const int max_disparity = static_cast<int>(max_disparity_64);
  const int surface_smoothing = static_cast<int>(surface_smoothing_64);
  const int edge_smoothing = static_cast<int>(edge_smoothing_64);
  auto options = parse_options(generic_names, generic_values);
  auto* memory = mv::runtime::scratch_resource();
  auto output = solve_method(left, right, min_disparity, max_disparity, surface_smoothing,
                             edge_smoothing, options, memory);
  if (options.consistency) {
    Options reverse_options = options;
    reverse_options.consistency = false;
    reverse_options.quality = false;
    // 反向视差仅判左右一致性；两个相反极线方向保持对称水平支撑，
    // 重复全部前向路径只增成本，不改变主视差。
    reverse_options.paths = (std::min)(reverse_options.paths, 2);
    auto reverse = solve_method(right, left, -max_disparity, -min_disparity, surface_smoothing,
                                edge_smoothing, reverse_options, memory);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t row = 0; row < left.height(); ++row) {
      auto* disparity_row =
          reinterpret_cast<float*>(static_cast<std::byte*>(output.disparity.mutable_data()) +
                                   row * output.disparity.stride());
      auto* score_row = reinterpret_cast<float*>(
          static_cast<std::byte*>(output.score.mutable_data()) + row * output.score.stride());
      for (std::size_t column = 0; column < left.width(); ++column) {
        const float disparity = disparity_row[column];
        if (!std::isfinite(disparity)) continue;
        const int right_column =
            static_cast<int>(std::lround(static_cast<double>(column) + disparity));
        if (right_column < 0 || right_column >= static_cast<int>(right.width())) {
          disparity_row[column] = nan;
          score_row[column] = std::numeric_limits<float>::infinity();
          if (options.quality)
            reinterpret_cast<std::uint16_t*>(
                static_cast<std::byte*>(output.quality_channels[2].mutable_data()) +
                row * output.quality_channels[2].stride())[column] = OutsideDomain;
          continue;
        }
        const float reverse_disparity =
            image_float(reverse.disparity, row, static_cast<std::size_t>(right_column));
        if (!std::isfinite(reverse_disparity) ||
            std::abs(disparity + reverse_disparity) > options.consistency_tolerance) {
          disparity_row[column] = nan;
          score_row[column] = std::numeric_limits<float>::infinity();
          if (options.quality)
            reinterpret_cast<std::uint16_t*>(
                static_cast<std::byte*>(output.quality_channels[2].mutable_data()) +
                row * output.quality_channels[2].stride())[column] = Inconsistent;
        }
      }
    }
  }
  if (options.quality) {
    const auto left_domain = left.domain(), right_domain = right.domain();
    for (std::size_t row = 0; row < left.height(); ++row) {
      auto* d = reinterpret_cast<float*>(static_cast<std::byte*>(output.disparity.mutable_data()) +
                                         row * output.disparity.stride());
      auto* score = reinterpret_cast<float*>(static_cast<std::byte*>(output.score.mutable_data()) +
                                             row * output.score.stride());
      auto* confidence = reinterpret_cast<float*>(
          static_cast<std::byte*>(output.quality_channels[0].mutable_data()) +
          row * output.quality_channels[0].stride());
      auto* valid = static_cast<std::uint8_t*>(output.quality_channels[1].mutable_data()) +
                    row * output.quality_channels[1].stride();
      auto* reason = reinterpret_cast<std::uint16_t*>(
          static_cast<std::byte*>(output.quality_channels[2].mutable_data()) +
          row * output.quality_channels[2].stride());
      for (std::size_t col = 0; col < left.width(); ++col) {
        if (!left_domain.contains(row, col))
          reason[col] |= OutsideDomain;
        else if (std::isfinite(d[col])) {
          const auto rc = static_cast<std::int64_t>(std::lround(col + double(d[col])));
          if (!right_domain.contains(row, rc)) reason[col] |= OutsideDomain;
          if (options.min_confidence > 0 && confidence[col] < options.min_confidence)
            reason[col] |= Ambiguous;
        } else if (!reason[col])
          reason[col] = NoCorrespondence;
        if (reason[col]) {
          d[col] = std::numeric_limits<float>::quiet_NaN();
          score[col] = std::numeric_limits<float>::infinity();
          confidence[col] = 0;
          valid[col] = 0;
        } else
          valid[col] = 255;
      }
    }
    output.disparity_stddev = options.disparity_stddev;
    output.hole_max_width = options.hole_max_width;
    output.hole_max_depth_jump = options.hole_max_depth_jump;
  }
  return output;
}

DisparityResult binocular_disparity(const core::Image& left, const core::Image& right,
                                    std::int64_t min_disparity, std::int64_t max_disparity,
                                    const core::Tuple& generic_names,
                                    const core::Tuple& generic_values) {
  return binocular_disparity_ms(left, right, min_disparity, max_disparity, 0, 0, generic_names,
                                generic_values);
}

}  // namespace mv::reconstruction
