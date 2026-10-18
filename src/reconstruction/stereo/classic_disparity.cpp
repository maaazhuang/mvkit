#include "classic_disparity.hpp"

#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <optional>
#include <string_view>
#include <type_traits>
#include <vector>
namespace mv::reconstruction {
namespace {
constexpr std::string_view op = "mv_binocular_disparity";
[[noreturn]] void fail(std::string_view parameter, core::ErrorCode code, std::string_view message) {
  throw core::Exception(
      code == core::ErrorCode::InvalidType ? core::Status::TypeMismatch : core::Status::InvalidArgument, code,
      op, parameter, -1, message);
}
core::Value scalar(const core::Tuple& tuple, std::string_view name) {
  if (tuple.size() != 1)
    fail(name, core::ErrorCode::InvalidCardinality, "exactly one control value required");
  return tuple.at(0);
}
std::int64_t integer(const core::Tuple& tuple, std::string_view name) {
  auto v = scalar(tuple, name);
  if (v.kind() != core::ValueKind::Int64)
    fail(name, core::ErrorCode::InvalidType, "integer control value required");
  return v.as_int64();
}
double number(const core::Tuple& tuple, std::string_view name) {
  auto v = scalar(tuple, name);
  if (v.kind() != core::ValueKind::Int64 && v.kind() != core::ValueKind::Real64)
    fail(name, core::ErrorCode::InvalidType, "numeric control value required");
  return v.as_real64();
}
std::string_view text(const core::Tuple& tuple, std::string_view name) {
  auto v = scalar(tuple, name);
  if (v.kind() != core::ValueKind::String)
    fail(name, core::ErrorCode::InvalidType, "string control value required");
  return tuple.string_view(0);
}
std::int64_t disparity(const core::Tuple& tuple, std::string_view name) {
  auto v = number(tuple, name);
  if (!std::isfinite(v) || v < -32768 || v > 32767)
    fail(name, core::ErrorCode::InvalidValue, "disparity must be in [-32768,32767]");
  return static_cast<std::int64_t>(std::llround(v));
}
struct Plane {
  int width{}, height{};
  std::vector<std::uint8_t> pixels, domain;
  Plane(int w, int h) : width(w), height(h), pixels(std::size_t(w) * h), domain(pixels.size()) {}
  explicit Plane(const core::Image& image) {
    if (image.empty() || image.channel_count() == 0 || image.pixel_type() != core::PixelType::UInt8)
      fail("ImageRect", core::ErrorCode::InvalidType, "byte image required");
    if (image.width() > std::size_t(std::numeric_limits<int>::max()) ||
        image.height() > std::size_t(std::numeric_limits<int>::max()))
      fail("ImageRect", core::ErrorCode::InvalidValue, "image dimensions too large");
    width = static_cast<int>(image.width());
    height = static_cast<int>(image.height());
    pixels.resize(std::size_t(width) * height);
    domain.resize(pixels.size());
    for (int y = 0; y < height; ++y)
      std::copy_n(static_cast<const std::uint8_t*>(image.data()) + std::size_t(y) * image.stride(), width,
                  pixels.data() + std::size_t(y) * width);
    const auto region = image.domain();
    for (const auto& run : region.runs())
      for (auto x = std::max<std::int64_t>(0, run.col_begin);
           x <= std::min<std::int64_t>(width - 1, run.col_end); ++x)
        if (run.row >= 0 && run.row < height) domain[std::size_t(run.row) * width + std::size_t(x)] = 1;
  }
};
template <class Sum>
struct PrefixIntegral {
  using value_type = Sum;
  int stride{};
  std::vector<Sum> sum;
  PrefixIntegral(int w, int h) : stride(w + 1), sum(std::size_t(w + 1) * (h + 1)) {}
  template <class Fn>
  PrefixIntegral(int w, int h, Fn value) : PrefixIntegral(w, h) { fill(w, h, value); }
  template <class Fn>
  void fill(int w, int h, Fn value) {
    // 每层覆盖全部内点，首行/首列始终为零；视差层之间复用同一缓冲。
    for (int y = 0; y < h; ++y) {
      Sum row = 0;
      for (int x = 0; x < w; ++x) {
        row += value(y, x);
        sum[std::size_t(y + 1) * stride + x + 1] = sum[std::size_t(y) * stride + x + 1] + row;
      }
    }
  }
  double box(int y, int x, int hx, int hy) const {
    auto at = [&](int r, int c) { return sum[std::size_t(r) * stride + c]; };
    // 无符号差分按模运算，最终窗口和非负且不超过总和，因此转换前结果仍精确。
    return static_cast<double>(at(y + hy + 1, x + hx + 1) - at(y - hy, x + hx + 1) -
                               at(y + hy + 1, x - hx) + at(y - hy, x - hx));
  }
};
using Integral = PrefixIntegral<double>;
struct Statistics {
  Integral sum, squares;
  Statistics(const Plane& p)
      : sum(p.width, p.height, [&](int y, int x) { return double(p.pixels[std::size_t(y) * p.width + x]); }),
        squares(p.width, p.height, [&](int y, int x) {
          double v = p.pixels[std::size_t(y) * p.width + x];
          return v * v;
        }) {}
};
struct Matches {
  std::vector<int> disparity;
  std::vector<float> score;
  std::vector<std::uint8_t> valid;
};
float evaluate(double a, double b, double aa, double bb, double ab, double cost, double area,
               ClassicDisparityParameters::Method method) {
  if (method != ClassicDisparityParameters::Method::Ncc)
    return static_cast<float>(cost) * static_cast<float>(1. / area);
  const auto va = aa - a * a / area, vb = bb - b * b / area;
  if (va <= 0 || vb <= 0) return -1.f;
  return static_cast<float>(std::clamp((ab - a * b / area) / std::sqrt(va * vb), -1., 1.));
}
Matches match(const Plane& a, const Plane& b, const ClassicDisparityParameters& p,
              const Matches* seeds = nullptr, int seed_width = 0, int seed_height = 0) {
  const int hx = static_cast<int>(p.mask_width / 2), hy = static_cast<int>(p.mask_height / 2);
  const double area = double(2 * hx + 1) * (2 * hy + 1);
  const bool ncc = p.method == ClassicDisparityParameters::Method::Ncc;
  // 源图各不超过 2^26 像素且 byte 窗口 A<=2^20 时，前缀和及平方和均精确；
  // 非恒定窗口方差至少 (A-1)/A^2，
  // 远大于 Double 消减误差，恒定窗口则恰为零。阈值<=0 的 SAD/SSD 不消费这些统计。
  const bool need_statistics = ncc || p.texture_threshold > 0 || area > 1048576. ||
      a.pixels.size() > (std::size_t{1} << 26) || b.pixels.size() > (std::size_t{1} << 26);
  std::optional<Statistics> sa, sb;
  if (need_statistics) { sa.emplace(a); sb.emplace(b); }
  Matches result;
  result.disparity.resize(a.pixels.size());
  result.score.resize(a.pixels.size());
  result.valid.resize(a.pixels.size());
  std::vector<int> minimum(a.pixels.size(), static_cast<int>(p.minimum)),
      maximum(a.pixels.size(), static_cast<int>(p.maximum));
  if (seeds)
    for (int y = 0; y < a.height; ++y)
      for (int x = 0; x < a.width; ++x) {
        int low = std::numeric_limits<int>::max(), high = std::numeric_limits<int>::min();
        for (int r = std::max(0, y / 2 - 1); r <= std::min(seed_height - 1, y / 2 + 1); ++r)
          for (int c = std::max(0, x / 2 - 1); c <= std::min(seed_width - 1, x / 2 + 1); ++c) {
            const auto i = std::size_t(r) * seed_width + c;
            if (seeds->valid[i]) {
              low = std::min(low, seeds->disparity[i] * 2 - 2);
              high = std::max(high, seeds->disparity[i] * 2 + 2);
            }
          }
        const auto i = std::size_t(y) * a.width + x;
        minimum[i] = std::max(minimum[i], low);
        maximum[i] = std::min(maximum[i], high);
      }
  const int low = std::max<int>(static_cast<int>(p.minimum), -a.width),
            high = std::min<int>(static_cast<int>(p.maximum), b.width);
  const auto correlate = [&](auto& costs) {
    using Sum = typename std::remove_reference_t<decltype(costs)>::value_type;
    for (int d = low; d <= high; ++d) {
      costs.fill(a.width, a.height, [&](int y, int x) -> Sum {
        const int q = x + d;
        if (q < 0 || q >= b.width) return Sum{};
        const auto u = a.pixels[std::size_t(y) * a.width + x], v = b.pixels[std::size_t(y) * b.width + q];
        if constexpr (std::is_integral_v<Sum>) {
          const int difference = int(u) - int(v);
          return static_cast<Sum>(p.method == ClassicDisparityParameters::Method::Sad
                                      ? std::abs(difference) : difference * difference);
        } else {
          const double du = u, dv = v;
          return ncc ? du * dv : p.method == ClassicDisparityParameters::Method::Sad
                                    ? std::abs(du - dv) : (du - dv) * (du - dv);
        }
      });
      for (int y = hy; y < a.height - hy; ++y)
        for (int x = std::max(hx, hx - d); x < std::min(a.width - hx, b.width - hx - d); ++x) {
          const auto i = std::size_t(y) * a.width + x, j = std::size_t(y) * b.width + x + d;
          if (!a.domain[i] || !b.domain[j] || d < minimum[i] || d > maximum[i]) continue;
          double asum = 0, bsum = 0, asq = 0, bsq = 0;
          if (need_statistics) {
            asum = sa->sum.box(y, x, hx, hy); bsum = sb->sum.box(y, x + d, hx, hy);
            asq = sa->squares.box(y, x, hx, hy); bsq = sb->squares.box(y, x + d, hx, hy);
            if (asq / area - (asum / area) * (asum / area) < p.texture_threshold ||
                bsq / area - (bsum / area) * (bsum / area) < p.texture_threshold)
              continue;
          }
          const float score = evaluate(asum, bsum, asq, bsq, ncc ? costs.box(y, x, hx, hy) : 0,
                                       ncc ? 0 : costs.box(y, x, hx, hy), area, p.method);
          if (ncc ? !(score >= p.score_threshold) : !(score <= p.score_threshold)) continue;
          if (!result.valid[i] || (ncc ? score > result.score[i] : score < result.score[i])) {
            result.valid[i] = 1;
            result.disparity[i] = d;
            result.score[i] = score;
          }
        }
    }
  };
  // byte 整数代价在 N<=2^26 时总和小于 2^42，与旧 Double 前缀逐位等价。
  // 按最坏像素代价选择位宽；超出精确性界限及 NCC 仍保留原 Double 算术。
  if (!ncc && a.pixels.size() <= (std::size_t{1} << 26)) {
    const auto maximum_cost = p.method == ClassicDisparityParameters::Method::Sad ? 255u : 65025u;
    if (a.pixels.size() <= std::numeric_limits<std::uint32_t>::max() / maximum_cost) {
      PrefixIntegral<std::uint32_t> costs(a.width, a.height);
      correlate(costs);
    } else {
      PrefixIntegral<std::uint64_t> costs(a.width, a.height);
      correlate(costs);
    }
  } else {
    Integral costs(a.width, a.height);
    correlate(costs);
  }
  return result;
}
Plane downsample(const Plane& a) {
  Plane out((a.width + 1) / 2, (a.height + 1) / 2);
  constexpr int weights[5]{1, 4, 6, 4, 1};
  for (int y = 0; y < out.height; ++y)
    for (int x = 0; x < out.width; ++x) {
      unsigned value = 0;
      for (int r = -2; r <= 2; ++r)
        for (int c = -2; c <= 2; ++c) {
          const int sy = std::clamp(y * 2 + r, 0, a.height - 1), sx = std::clamp(x * 2 + c, 0, a.width - 1);
          value += unsigned(weights[r + 2] * weights[c + 2]) * a.pixels[std::size_t(sy) * a.width + sx];
        }
      const auto i = std::size_t(y) * out.width + x;
      out.pixels[i] = static_cast<std::uint8_t>((value + 128) / 256);
      out.domain[i] =
          a.domain[std::size_t(std::min(y * 2, a.height - 1)) * a.width + std::min(x * 2, a.width - 1)];
    }
  return out;
}
Matches pyramid_match(const Plane& a, const Plane& b, const ClassicDisparityParameters& p) {
  if (p.levels == 1) return match(a, b, p);
  std::vector<Plane> left{a}, right{b};
  for (std::int64_t level = 1; level < p.levels; ++level) {
    left.push_back(downsample(left.back()));
    right.push_back(downsample(right.back()));
  }
  Matches matches;
  for (int level = static_cast<int>(p.levels) - 1; level >= 0; --level) {
    auto controls = p;
    const double scale = std::ldexp(1., level);
    controls.minimum = static_cast<std::int64_t>(std::floor(double(p.minimum) / scale));
    controls.maximum = static_cast<std::int64_t>(std::ceil(double(p.maximum) / scale));
    const auto* seed = level == p.levels - 1 ? nullptr : &matches;
    matches = match(left[level], right[level], controls, seed, seed ? left[level + 1].width : 0,
                    seed ? left[level + 1].height : 0);
  }
  return matches;
}
float local_cost(const Plane& a, const Plane& b, int row, int col, int d,
                 const ClassicDisparityParameters& p) {
  const int hx = static_cast<int>(p.mask_width / 2), hy = static_cast<int>(p.mask_height / 2);
  double u = 0, v = 0, uu = 0, vv = 0, uv = 0, cost = 0;
  for (int y = row - hy; y <= row + hy; ++y)
    for (int x = col - hx; x <= col + hx; ++x) {
      double aa = a.pixels[std::size_t(y) * a.width + x], bb = b.pixels[std::size_t(y) * b.width + x + d];
      u += aa;
      v += bb;
      uu += aa * aa;
      vv += bb * bb;
      uv += aa * bb;
      cost += p.method == ClassicDisparityParameters::Method::Sad ? std::abs(aa - bb) : (aa - bb) * (aa - bb);
    }
  return evaluate(u, v, uu, vv, uv, cost, double(2 * hx + 1) * (2 * hy + 1), p.method);
}
}  // namespace
ClassicDisparityParameters classic_disparity_parameters(
    const core::Tuple& Method, const core::Tuple& MaskWidth, const core::Tuple& MaskHeight,
    const core::Tuple& TextureThresh, const core::Tuple& MinDisparity, const core::Tuple& MaxDisparity,
    const core::Tuple& NumLevels, const core::Tuple& ScoreThresh, const core::Tuple& Filter,
    const core::Tuple& SubDisparity) {
  ClassicDisparityParameters p;
  auto method = text(Method, "Method");
  if (method == "sad")
    p.method = ClassicDisparityParameters::Method::Sad;
  else if (method == "ssd")
    p.method = ClassicDisparityParameters::Method::Ssd;
  else if (method == "ncc")
    p.method = ClassicDisparityParameters::Method::Ncc;
  else
    fail("Method", core::ErrorCode::InvalidValue, "method must be sad, ssd, or ncc");
  p.mask_width = integer(MaskWidth, "MaskWidth");
  p.mask_height = integer(MaskHeight, "MaskHeight");
  if (p.mask_width < 2 || p.mask_width > std::numeric_limits<int>::max())
    fail("MaskWidth", core::ErrorCode::InvalidValue, "mask width must be at least two");
  if (p.mask_height < 2 || p.mask_height > std::numeric_limits<int>::max())
    fail("MaskHeight", core::ErrorCode::InvalidValue, "mask height must be at least two");
  p.texture_threshold = number(TextureThresh, "TextureThresh");
  p.minimum = disparity(MinDisparity, "MinDisparity");
  p.maximum = disparity(MaxDisparity, "MaxDisparity");
  if (p.maximum < p.minimum)
    fail("MaxDisparity", core::ErrorCode::InvalidValue, "maximum disparity must not be below minimum");
  p.levels = integer(NumLevels, "NumLevels");
  if (p.levels < 1 || p.levels > 31)
    fail("NumLevels", core::ErrorCode::InvalidValue, "number of levels must be positive");
  p.score_threshold = number(ScoreThresh, "ScoreThresh");
  for (std::size_t i = 0; i < Filter.size(); ++i) {
    if (Filter.at(i).kind() != core::ValueKind::String)
      fail("Filter", core::ErrorCode::InvalidType, "filter names must be strings");
    auto name = Filter.string_view(i);
    if (name == "left_right_check")
      p.left_right_check = true;
    else if (name != "none")
      fail("Filter", core::ErrorCode::InvalidValue, "unknown downstream filter");
  }
  auto sub = text(SubDisparity, "SubDisparity");
  if (sub == "interpolation")
    p.interpolation = true;
  else if (sub != "none")
    fail("SubDisparity", core::ErrorCode::InvalidValue, "unknown subpixel mode");
  return p;
}
ClassicDisparityResult classic_binocular_disparity(const core::Image& left, const core::Image& right,
                                                   const ClassicDisparityParameters& p) {
  const Plane a(left), b(right);
  if (a.height != b.height) fail("ImageRect2", core::ErrorCode::InvalidValue, "image heights must agree");
  const int hx = static_cast<int>(p.mask_width / 2), hy = static_cast<int>(p.mask_height / 2);
  if (std::min(a.width, b.width) <= 2 * hx || a.height <= 2 * hy)
    fail("NumLevels", core::ErrorCode::InvalidValue, "correlation window exceeds the image");
  const auto divisor = std::int64_t(1) << (p.levels - 1);
  if ((std::min(a.width, b.width) + divisor - 1) / divisor <= 2 * hx ||
      (a.height + divisor - 1) / divisor <= 2 * hy)
    fail("NumLevels", core::ErrorCode::InvalidValue, "correlation window exceeds the coarsest pyramid level");
  ClassicDisparityResult output{core::Image::create(left.width(), left.height(), core::PixelType::Float32),
                                core::Image::create(left.width(), left.height(), core::PixelType::Float32)};
  auto search = p;
  if (p.interpolation) {
    --search.minimum;
    ++search.maximum;
  }
  auto selected = pyramid_match(a, b, search);
  Matches reverse;
  if (p.left_right_check) {
    auto back = p;
    back.minimum = -p.maximum;
    back.maximum = -p.minimum;
    back.left_right_check = false;
    back.interpolation = false;
    reverse = pyramid_match(b, a, back);
  }
  std::vector<std::uint8_t> searchable(a.pixels.size());
  if (p.interpolation && p.method != ClassicDisparityParameters::Method::Ncc) {
    // SAD/SSD 仅在水平腐蚀后的相关支撑内插值；支撑由原视差区间及两输入定义域决定。
    std::vector<int> prefix(std::size_t(b.width + 1) * b.height);
    for (int y = 0; y < b.height; ++y)
      for (int x = 0; x < b.width; ++x)
        prefix[std::size_t(y) * (b.width + 1) + x + 1] =
            prefix[std::size_t(y) * (b.width + 1) + x] +
            ((x >= hx && x < b.width - hx) ? b.domain[std::size_t(y) * b.width + x] : 0);
    for (int y = hy; y < a.height - hy; ++y)
      for (int x = hx; x < a.width - hx; ++x) {
        const auto i = std::size_t(y) * a.width + x;
        const int low = std::max<std::int64_t>(hx, x + p.minimum),
                  high = std::min<std::int64_t>(b.width - hx - 1, x + p.maximum);
        searchable[i] =
            a.domain[i] && low <= high &&
            prefix[std::size_t(y) * (b.width + 1) + high + 1] > prefix[std::size_t(y) * (b.width + 1) + low];
      }
  }
  std::vector<core::RegionRun> runs;

  for (int y = 0; y < a.height; ++y) {
    auto* disp = reinterpret_cast<float*>(static_cast<std::byte*>(output.disparity.mutable_data()) +
                                          std::size_t(y) * output.disparity.stride());
    auto* score = reinterpret_cast<float*>(static_cast<std::byte*>(output.score.mutable_data()) +
                                           std::size_t(y) * output.score.stride());
    int start = -1;
    for (int x = 0; x < a.width; ++x) {
      const auto i = std::size_t(y) * a.width + x;
      bool valid = selected.valid[i];
      const int d = selected.disparity[i];
      float refined = static_cast<float>(d);
      if (valid && p.left_right_check) {
        const int q = x + d;
        const auto j = std::size_t(y) * b.width + q;
        valid = reverse.valid[j] && d + reverse.disparity[j] == 0;
      }
      if (valid && p.interpolation) {
        const int q = x + d;
        valid = x >= hx + 1 && x < a.width - hx - 1 && q >= hx + 1 && q < b.width - hx - 1 &&
                a.domain[i - 1] && a.domain[i + 1] && b.domain[std::size_t(y) * b.width + q - 1] &&
                b.domain[std::size_t(y) * b.width + q + 1];
        if (valid && p.method != ClassicDisparityParameters::Method::Ncc)
          valid = searchable[i - 1] && searchable[i + 1];
        if (valid && d >= p.minimum && d <= p.maximum) {
          const double lo = local_cost(a, b, y, x, d - 1, p), mid = selected.score[i],
                       hi = local_cost(a, b, y, x, d + 1, p), denom = lo - 2 * mid + hi;
          const double offset = denom == 0 ? 0 : .5 * (lo - hi) / denom;
          refined = static_cast<float>(d + std::clamp(offset, -1., 1.));
        }
      }
      if (valid) {
        disp[x] = refined;
        score[x] = selected.score[i];
        if (start < 0) start = x;
      } else {
        if (start >= 0) {
          runs.push_back({y, start, x - 1});
          start = -1;
        }
      }
    }
    if (start >= 0) runs.push_back({y, start, a.width - 1});
  }
  auto domain = core::Region::from_runs(runs);
  output.disparity = output.disparity.reduce_domain(domain);
  output.score = output.score.reduce_domain(domain);
  return output;
}
}  // namespace mv::reconstruction
