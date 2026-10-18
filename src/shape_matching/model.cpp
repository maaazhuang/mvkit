#include <shape_matching/operators.hpp>

#include <algorithm>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <type_traits>
#include <vector>

#include <core/exception.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/operators.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/memory/persist_container.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <xld/operators.hpp>

namespace mv::shape_matching {
namespace {
using mv::core::Exception;
using mv::core::Image;
using mv::core::Point2d;
using mv::core::Status;
using mv::core::Tuple;
using mv::core::TupleKind;
using mv::core::XldContour;
using mv::core::XldContours;

std::pmr::vector<double> to_scalar_double(
    const Image& image, std::pmr::memory_resource* memory) {
  if (image.empty()) throw Exception(Status::InvalidArgument, "create_shape_model", "image must not be empty");
  mv::imageproc::ImageView view{image};
  return mv::imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    std::pmr::vector<double> out(image.width() * image.height(), memory);
    for (std::size_t r = 0; r < image.height(); ++r) {
      const auto* row = view.row_as<T>(r);
      for (std::size_t c = 0; c < image.width(); ++c) out[r * image.width() + c] = static_cast<double>(row[c]);
    }
    return out;
  });
}

void sobel_u8(const Image& image, std::pmr::vector<double>& gx,
              std::pmr::vector<double>& gy) {
  mv::imageproc::ImageView view{image};
  const std::size_t w = image.width();
  const std::size_t h = image.height();
  gx.assign(w * h, 0.0);
  gy.assign(w * h, 0.0);
  for (std::size_t r = 0; r < h; ++r) {
    const auto* top = view.row_as<std::uint8_t>(r == 0 ? 0 : r - 1);
    const auto* middle = view.row_as<std::uint8_t>(r);
    const auto* bottom = view.row_as<std::uint8_t>(r + 1 < h ? r + 1 : h - 1);
    auto write = [&](std::size_t c, std::size_t left, std::size_t right) {
      const int sx =
          (top[right] + 2 * middle[right] + bottom[right]) -
          (top[left] + 2 * middle[left] + bottom[left]);
      const int sy =
          (bottom[left] + 2 * bottom[c] + bottom[right]) -
          (top[left] + 2 * top[c] + top[right]);
      const auto index = r * w + c;
      gx[index] = 0.25 * static_cast<double>(sx);
      gy[index] = 0.25 * static_cast<double>(sy);
    };
    if (w == 1) {
      write(0, 0, 0);
      continue;
    }
    write(0, 0, 1);
    for (std::size_t c = 1; c + 1 < w; ++c) write(c, c - 1, c + 1);
    write(w - 1, w - 2, w - 1);
  }
}

void sobel(std::span<const double> src, std::size_t w, std::size_t h,
           std::pmr::vector<double>& gx, std::pmr::vector<double>& gy) {
  gx.assign(w * h, 0.0); gy.assign(w * h, 0.0);
  for (std::size_t r=0;r<h;++r) {
    const std::size_t previous_row=r==0?0:r-1;
    const std::size_t next_row=r+1<h?r+1:h-1;
    const double* top=src.data()+previous_row*w;
    const double* middle=src.data()+r*w;
    const double* bottom=src.data()+next_row*w;
    for (std::size_t c=0;c<w;++c) {
      const std::size_t left=c==0?0:c-1;
      const std::size_t right=c+1<w?c+1:w-1;
      const double sx=(top[right]+2.0*middle[right]+bottom[right])-
                      (top[left]+2.0*middle[left]+bottom[left]);
      const double sy=(bottom[left]+2.0*bottom[c]+bottom[right])-
                      (top[left]+2.0*top[c]+top[right]);
      gx[r*w+c]=sx*0.25; gy[r*w+c]=sy*0.25;
    }
  }
}

ShapeMetric parse_metric(std::string_view metric) {
  if (metric == "use_polarity") return ShapeMetric::UsePolarity;
  if (metric == "ignore_global_polarity") return ShapeMetric::IgnoreGlobalPolarity;
  if (metric == "ignore_local_polarity" || metric == "ignore_color_polarity") return ShapeMetric::IgnoreLocalPolarity;
  throw Exception(Status::InvalidArgument, "create_shape_model", "unknown shape metric");
}

double tuple_number_or(const Tuple& t, double fallback) {
  if (t.empty()) return fallback;
  if (t.kind()==TupleKind::Real64) return t.as_real64();
  if (t.kind()==TupleKind::Int64) return static_cast<double>(t.as_int64());
  return fallback;
}

bool tuple_auto(const Tuple& t) {
  return t.size()==1 && t.kind()==TupleKind::String && t.string_view()=="auto";
}

int parse_levels(const Tuple& t, std::size_t point_count, std::size_t domain_width,
                 std::size_t domain_height) {
  if (t.empty() || tuple_auto(t) || ((t.kind()==TupleKind::Int64) && t.as_int64()==0)) {
    // 点数衰减匹配小稠密模板的历史启发式；几何按 HALCON 实际规则，
    // 粗模板短边仍约>=12 px 时继续分层。
    int by_points = 1;
    std::size_t n = point_count;
    while (by_points < 6 && n >= 80) {
      ++by_points;
      n /= 4;
    }
    int by_size = 1;
    std::size_t side = std::min(domain_width, domain_height);
    while (by_size < 6 && side / 2 >= 12) {
      ++by_size;
      side /= 2;
    }
    return std::max(by_points, by_size);
  }
  if (t.kind()!=TupleKind::Int64 || t.size()!=1 || t.as_int64()<1)
    throw Exception(Status::InvalidArgument,"create_shape_model","num_levels must be auto, 0 or positive integer");
  return static_cast<int>(t.as_int64());
}

enum class PointReduction { None, Auto, Low, Medium, High };

PointReduction parse_optimization(const Tuple& t) {
  auto parse_one = [](std::string_view value) {
    if (value == "none") return PointReduction::None;
    if (value == "auto") return PointReduction::Auto;
    if (value == "point_reduction_low") return PointReduction::Low;
    if (value == "point_reduction_medium") return PointReduction::Medium;
    if (value == "point_reduction_high") return PointReduction::High;
    if (value == "no_pregeneration" || value == "pregeneration") return PointReduction::None;
    throw Exception(Status::InvalidArgument, "create_shape_model",
                    "optimization must be none/auto/point_reduction_low|medium|high");
  };
  if (t.empty() || tuple_auto(t)) return PointReduction::Auto;
  if (t.kind() == TupleKind::String) return parse_one(t.string_view());
  PointReduction reduction = PointReduction::None;
  bool have_reduction = false;
  for (std::size_t i = 0; i < t.size(); ++i) {
    const auto item = t.at(i);
    if (item.kind() != mv::core::ValueKind::String) {
      throw Exception(Status::InvalidArgument, "create_shape_model", "optimization values must be strings");
    }
    const auto value = item.as_string().view();
    if (value == "no_pregeneration" || value == "pregeneration") continue;
    reduction = parse_one(value);
    have_reduction = true;
  }
  return have_reduction ? reduction : PointReduction::None;
}

double reduction_spacing(PointReduction reduction, std::size_t count) {
  switch (reduction) {
    case PointReduction::None: return 1.0;
    case PointReduction::Low: return 1.6;
    case PointReduction::Medium: return 2.4;
    case PointReduction::High: return 3.6;
    case PointReduction::Auto:
      if (count > 400) return 2.4;
      if (count > 160) return 1.6;
      return 1.0;
  }
  return 1.0;
}

struct ContrastSpec {
  double low{10.0};
  double high{10.0};
  std::size_t min_size{0};
};

double value_as_number(const mv::core::Value& value, const char* where) {
  if (value.kind() == mv::core::ValueKind::Int64) return static_cast<double>(value.as_int64());
  if (value.kind() == mv::core::ValueKind::Real64) return value.as_real64();
  throw Exception(Status::InvalidArgument, where, "numeric tuple value expected");
}

ContrastSpec auto_contrast_from(std::span<const double> mag, bool hysteresis,
                                bool auto_size,
                                std::pmr::memory_resource* memory) {
  ContrastSpec spec;
  if (mag.empty()) return spec;
  std::pmr::vector<double> sorted(mag.begin(), mag.end(), memory);
  const auto percentile = sorted.begin() + static_cast<std::size_t>(0.72 * (sorted.size() - 1));
  std::nth_element(sorted.begin(), percentile, sorted.end());
  const double hi = *percentile;
  spec.high = std::max(1.0, hi);
  spec.low = hysteresis ? std::max(1.0, 0.45 * spec.high) : spec.high;
  if (auto_size) spec.min_size = 5;
  return spec;
}

ContrastSpec parse_contrast(const Tuple& t, std::span<const double> mag,
                            std::pmr::memory_resource* memory) {
  if (t.empty() || tuple_auto(t) || (t.kind() == TupleKind::String && t.string_view() == "auto")) {
    return auto_contrast_from(mag, true, true, memory);
  }
  bool auto_contrast = false;
  bool auto_hyst = false;
  bool auto_size = false;
  std::vector<double> numbers;
  const auto consider = [&](const mv::core::Value& value) {
    if (value.kind() == mv::core::ValueKind::String) {
      const auto text = value.as_string().view();
      if (text == "auto") {
        auto_contrast = auto_hyst = auto_size = true;
      } else if (text == "auto_contrast") {
        auto_contrast = true;
      } else if (text == "auto_contrast_hyst") {
        auto_hyst = true;
      } else if (text == "auto_min_size") {
        auto_size = true;
      } else {
        throw Exception(Status::InvalidArgument, "create_shape_model", "unknown contrast selector");
      }
      return;
    }
    numbers.push_back(value_as_number(value, "create_shape_model"));
  };
  if (t.kind() == TupleKind::String) consider(mv::core::Value{std::string(t.string_view())});
  else if (t.size() == 1 && (t.kind() == TupleKind::Int64 || t.kind() == TupleKind::Real64)) {
    numbers.push_back(tuple_number_or(t, -1));
  } else {
    for (std::size_t i = 0; i < t.size(); ++i) consider(t.at(i));
  }
  auto spec = auto_contrast_from(
      mag, auto_hyst || (auto_contrast && numbers.empty()), auto_size,
      memory);
  if (!auto_contrast && !auto_hyst && !numbers.empty()) {
    if (numbers.size() == 1) {
      if (numbers[0] < 0) throw Exception(Status::InvalidArgument, "create_shape_model", "contrast must be nonnegative");
      spec.low = spec.high = numbers[0];
    } else {
      if (numbers[0] < 0 || numbers[1] < numbers[0]) {
        throw Exception(Status::InvalidArgument, "create_shape_model", "invalid contrast thresholds");
      }
      spec.low = numbers[0];
      spec.high = numbers[1];
    }
    if (numbers.size() >= 3) spec.min_size = static_cast<std::size_t>(std::max(0.0, numbers[2]));
  } else if (numbers.size() >= 2) {
    spec.low = numbers[0];
    spec.high = numbers[1];
  } else if (numbers.size() == 1 && !auto_size) {
    spec.low = spec.high = numbers[0];
  }
  if (auto_size && spec.min_size == 0) spec.min_size = 5;
  if (numbers.size() >= 3) spec.min_size = static_cast<std::size_t>(std::max(0.0, numbers.back()));
  return spec;
}

double parse_min_contrast(const Tuple& t, double contrast_low) {
  if (t.empty() || tuple_auto(t) || (t.kind() == TupleKind::String && t.string_view() == "auto")) {
    return std::max(1.0, 0.5 * contrast_low);
  }
  const double value = tuple_number_or(t, -1);
  if (value < 0) throw Exception(Status::InvalidArgument, "create_shape_model", "min_contrast must be nonnegative or auto");
  return value;
}

double domain_centroid_row(const Image& image, double* col) {
  const auto d=image.domain(); long double sr=0,sc=0,n=0;
  for (const auto& run:d.runs()) {
    const auto count=static_cast<long double>(std::int64_t(run.col_end)-std::int64_t(run.col_begin)+1);
    sr += static_cast<long double>(run.row)*count;
    sc += (static_cast<long double>(run.col_begin)+static_cast<long double>(run.col_end))*0.5L*count;
    n += count;
  }
  if (n<=0) throw Exception(Status::InvalidArgument,"create_shape_model","template domain must not be empty");
  *col=static_cast<double>(sc/n); return static_cast<double>(sr/n);
}

XldContours shifted_contours(const XldContours& input,double orow,double ocol,double div) {
  XldContours out;
  for (std::size_t index=0; index<input.contour_count(); ++index) {
    const auto& c=input.contour(index);
    std::vector<Point2d> pts; pts.reserve(c.size());
    for (const auto&p:c.points()) pts.push_back({(p.row-orow)/div,(p.col-ocol)/div});
    out.append(XldContour::from_points(pts,c.closed()));
  }
  return out;
}

double bilinear_mag(std::span<const double> mag, std::size_t w, std::size_t h,
                    double r, double c) {
  if (w == 0 || h == 0 || r < 0 || c < 0 || r > static_cast<double>(h - 1) || c > static_cast<double>(w - 1)) return 0.0;
  const auto r0 = static_cast<std::size_t>(std::floor(r));
  const auto c0 = static_cast<std::size_t>(std::floor(c));
  const auto r1 = std::min(r0 + 1, h - 1);
  const auto c1 = std::min(c0 + 1, w - 1);
  const double fr = r - static_cast<double>(r0);
  const double fc = c - static_cast<double>(c0);
  auto at = [&](std::size_t rr, std::size_t cc) { return mag[rr * w + cc]; };
  return (1.0 - fr) * ((1.0 - fc) * at(r0, c0) + fc * at(r0, c1)) +
         fr * ((1.0 - fc) * at(r1, c0) + fc * at(r1, c1));
}

bool is_local_maximum(std::span<const double> mag, std::span<const double> nr,
                      std::span<const double> nc,
                      std::size_t w, std::size_t h, std::size_t row, std::size_t col) {
  if (row == 0 || col == 0 || row + 1 >= h || col + 1 >= w) return false;
  const auto idx = row * w + col;
  const double nrm = std::abs(nr[idx]);
  const double ncm = std::abs(nc[idx]);
  int dr = 0;
  int dc = 0;
  if (ncm > 2.4142 * nrm) dc = 1;
  else if (nrm > 2.4142 * ncm) dr = 1;
  else {
    dr = 1;
    dc = nr[idx] * nc[idx] >= 0.0 ? 1 : -1;
  }
  const auto before = (row - static_cast<std::size_t>(dr)) * w + (col - static_cast<std::size_t>(dc));
  const auto after = (row + static_cast<std::size_t>(dr)) * w + (col + static_cast<std::size_t>(dc));
  return mag[idx] >= mag[before] && mag[idx] >= mag[after];
}

std::pmr::vector<double> downsample_gray(
    std::span<const double> src, std::size_t w, std::size_t h,
    std::size_t* out_w, std::size_t* out_h,
    std::pmr::memory_resource* memory) {
  const std::size_t nw = std::max<std::size_t>(1, w / 2);
  const std::size_t nh = std::max<std::size_t>(1, h / 2);
  std::pmr::vector<double> smooth(w * h, 0.0, memory);
  for (std::size_t r = 0; r < h; ++r) {
    for (std::size_t c = 0; c < w; ++c) {
      const auto left = c == 0 ? c : c - 1;
      const auto right = c + 1 >= w ? c : c + 1;
      smooth[r * w + c] = 0.25 * src[r * w + left] + 0.5 * src[r * w + c] + 0.25 * src[r * w + right];
    }
  }
  std::pmr::vector<double> dst(nw * nh, 0.0, memory);
  for (std::size_t r = 0; r < nh; ++r) {
    const std::size_t r0 = std::min(h - 1, 2 * r);
    const std::size_t r1 = std::min(h - 1, r0 + 1);
    for (std::size_t c = 0; c < nw; ++c) {
      const std::size_t c0 = std::min(w - 1, 2 * c);
      const std::size_t c1 = std::min(w - 1, c0 + 1);
      dst[r * nw + c] = 0.25 * (smooth[r0 * w + c0] + smooth[r0 * w + c1] + smooth[r1 * w + c0] + smooth[r1 * w + c1]);
    }
  }
  *out_w = nw;
  *out_h = nh;
  return dst;
}

std::pmr::vector<double> downsample_gray_u8(
    const Image& image, std::size_t* out_w, std::size_t* out_h,
    std::pmr::memory_resource* memory) {
  mv::imageproc::ImageView view{image};
  const std::size_t w = image.width();
  const std::size_t h = image.height();
  const std::size_t nw = std::max<std::size_t>(1, w / 2);
  const std::size_t nh = std::max<std::size_t>(1, h / 2);
  std::pmr::vector<double> dst(nw * nh, 0.0, memory);
  auto smooth = [&](const std::uint8_t* row, std::size_t column) {
    const auto left = column == 0 ? column : column - 1;
    const auto right = column + 1 >= w ? column : column + 1;
    return 0.25 * static_cast<double>(row[left]) +
           0.5 * static_cast<double>(row[column]) +
           0.25 * static_cast<double>(row[right]);
  };
  for (std::size_t r = 0; r < nh; ++r) {
    const std::size_t r0 = std::min(h - 1, 2 * r);
    const std::size_t r1 = std::min(h - 1, r0 + 1);
    const auto* row0 = view.row_as<std::uint8_t>(r0);
    const auto* row1 = view.row_as<std::uint8_t>(r1);
    for (std::size_t c = 0; c < nw; ++c) {
      const std::size_t c0 = std::min(w - 1, 2 * c);
      const std::size_t c1 = std::min(w - 1, c0 + 1);
      dst[r * nw + c] =
          0.25 * (smooth(row0, c0) + smooth(row0, c1) +
                  smooth(row1, c0) + smooth(row1, c1));
    }
  }
  *out_w = nw;
  *out_h = nh;
  return dst;
}

mv::core::Region downsample_region(const mv::core::Region& domain) {
  std::vector<mv::core::RegionRun> runs;
  for (const auto& run : domain.runs()) {
    if (run.row < 0 || (run.row % 2) != 0) continue;
    runs.push_back({run.row / 2, std::int64_t(run.col_begin) / 2, std::int64_t(run.col_end) / 2});
  }
  return runs.empty() ? mv::core::Region{} : mv::core::Region::from_runs(runs);
}

std::vector<ShapeModelPoint> extract_contour_points(
                                                    std::span<const double> gx,
                                                    std::span<const double> gy,
                                                    std::size_t w, std::size_t h, const mv::core::Region& domain,
                                                    double origin_row, double origin_col, double low, double high,
                                                    std::size_t min_size,
                                                    std::pmr::memory_resource* memory) {
  std::pmr::vector<double> mag(w * h, 0.0, memory);
  std::pmr::vector<double> nr(w * h, 0.0, memory);
  std::pmr::vector<double> nc(w * h, 0.0, memory);
  for (std::size_t i = 0; i < mag.size(); ++i) {
    mag[i] = std::hypot(gx[i], gy[i]);
    if (mag[i] > 1e-12) {
      nr[i] = gy[i] / mag[i];
      nc[i] = gx[i] / mag[i];
    }
  }

  std::pmr::vector<std::uint8_t> edge(w * h, 0, memory);
  for (const auto& run : domain.runs()) {
    if (run.row <= 0 || static_cast<std::size_t>(run.row) + 1 >= h) continue;
    for (std::int64_t c = run.col_begin; c <= run.col_end; ++c) {
      if (c <= 0 || static_cast<std::size_t>(c) + 1 >= w) continue;
      const auto idx = static_cast<std::size_t>(run.row) * w + static_cast<std::size_t>(c);
      if (mag[idx] < low) continue;
      if (!is_local_maximum(mag, nr, nc, w, h, static_cast<std::size_t>(run.row), static_cast<std::size_t>(c))) continue;
      edge[idx] = mag[idx] >= high ? 2 : 1;
    }
  }

  std::pmr::vector<std::uint8_t> accepted(w * h, 0, memory);
  std::pmr::vector<std::uint8_t> visited(w * h, 0, memory);
  std::pmr::vector<std::size_t> component{memory};
  for (const auto& run : domain.runs()) {
    for (std::int64_t c = run.col_begin; c <= run.col_end; ++c) {
      if (run.row < 0 || c < 0 || static_cast<std::size_t>(run.row) >= h || static_cast<std::size_t>(c) >= w) continue;
      const auto seed = static_cast<std::size_t>(run.row) * w + static_cast<std::size_t>(c);
      if (edge[seed] == 0 || visited[seed]) continue;
      component.clear();
      visited[seed] = 1;
      component.push_back(seed);
      bool strong = false;
      for (std::size_t head = 0; head < component.size(); ++head) {
        const auto pidx = component[head];
        const auto pr = static_cast<std::int64_t>(pidx / w);
        const auto pc = static_cast<std::int64_t>(pidx % w);
        strong = strong || edge[pidx] == 2;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            const auto nrw = pr + dr;
            const auto ncw = pc + dc;
            if (nrw < 0 || ncw < 0 || static_cast<std::size_t>(nrw) >= h || static_cast<std::size_t>(ncw) >= w) continue;
            const auto nidx = static_cast<std::size_t>(nrw) * w + static_cast<std::size_t>(ncw);
            // 仅域内像素可有非零边缘标签。
            if (edge[nidx] == 0 || visited[nidx]) continue;
            visited[nidx] = 1;
            component.push_back(nidx);
          }
        }
      }
      if (!strong || component.size() < std::max<std::size_t>(2, min_size)) continue;
      for (const auto index : component) accepted[index] = 1;
    }
  }

  std::vector<ShapeModelPoint> candidates;
  for (const auto& run : domain.runs()) {
    for (std::int64_t c = run.col_begin; c <= run.col_end; ++c) {
      if (run.row < 0 || c < 0 || static_cast<std::size_t>(run.row) >= h || static_cast<std::size_t>(c) >= w) continue;
      const auto idx = static_cast<std::size_t>(run.row) * w + static_cast<std::size_t>(c);
      if (!accepted[idx]) continue;
      const double before = bilinear_mag(mag, w, h, static_cast<double>(run.row) - nr[idx], static_cast<double>(c) - nc[idx]);
      const double after = bilinear_mag(mag, w, h, static_cast<double>(run.row) + nr[idx], static_cast<double>(c) + nc[idx]);
      const double denom = before - 2.0 * mag[idx] + after;
      const double offset = denom < -1e-6 ? std::clamp(0.5 * (before - after) / denom, -0.5, 0.5) : 0.0;
      candidates.push_back({static_cast<double>(run.row) + offset * nr[idx] - origin_row,
                            static_cast<double>(c) + offset * nc[idx] - origin_col,
                            nr[idx], nc[idx], mag[idx]});
    }
  }

  if (candidates.size() < 4) {
    candidates.clear();
    for (const auto& run : domain.runs()) {
      for (std::int64_t c = run.col_begin; c <= run.col_end; ++c) {
        if (run.row < 0 || c < 0 || static_cast<std::size_t>(run.row) >= h || static_cast<std::size_t>(c) >= w) continue;
        const auto idx = static_cast<std::size_t>(run.row) * w + static_cast<std::size_t>(c);
        if (mag[idx] < high || mag[idx] <= 1e-12) continue;
        candidates.push_back({static_cast<double>(run.row) - origin_row, static_cast<double>(c) - origin_col,
                              nr[idx], nc[idx], mag[idx]});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const ShapeModelPoint& a, const ShapeModelPoint& b) {
    return a.response > b.response;
  });
  return candidates;
}

std::vector<ShapeModelPoint> reduce_contour_points(
    std::span<const ShapeModelPoint> candidates, std::size_t limit, PointReduction reduction) {
  const double spacing = reduction_spacing(reduction, candidates.size());
  // Auto 采用各层实际密度，含存储上限以下粗层；稀疏层保留全部特征，显式 reduction 保持原选择语义。
  if (spacing <= 1.01 || (reduction != PointReduction::Auto && candidates.size() <= limit)) {
    return {candidates.begin(), candidates.begin() + std::min(limit, candidates.size())};
  }

  std::vector<ShapeModelPoint> selected;
  selected.reserve(std::min(limit, candidates.size()));
  double gap = spacing;
  while (selected.size() < limit && gap >= 0.5) {
    selected.clear();
    const double gap2 = gap * gap;
    for (const auto& candidate : candidates) {
      bool far = true;
      for (const auto& chosen : selected) {
        const double dr = candidate.row - chosen.row;
        const double dc = candidate.column - chosen.column;
        if (dr * dr + dc * dc < gap2) {
          far = false;
          break;
        }
      }
      if (far) selected.push_back(candidate);
      if (selected.size() == limit) break;
    }
    if (selected.size() >= std::min<std::size_t>(16, candidates.size()) || gap <= 0.5) break;
    gap *= 0.7;
  }
  if (selected.empty()) return {candidates.begin(), candidates.end()};
  return selected;
}

}  // namespace

void ShapeModel::set_origin_offset(double row, double column) {
  if (!std::isfinite(row) || !std::isfinite(column)) {
    throw Exception(Status::InvalidArgument, "set_shape_model_origin", "origin offset must be finite");
  }
  const double new_r = default_origin_row_ + row;
  const double new_c = default_origin_column_ + column;
  const double dr = new_r - origin_row_;
  const double dc = new_c - origin_column_;
  // 重复设置同一原点不改变训练坐标，也无需重新分配轮廓和失效搜索缓存。
  // XLD 模型还借此同步轮廓半径，保留其原路径。
  if(dr==0. && dc==0. && std::signbit(new_r)==std::signbit(origin_row_) &&
     std::signbit(new_c)==std::signbit(origin_column_) && !xld_model_)return;
  std::vector<XldContours> adjusted_levels;
  adjusted_levels.reserve(levels_.size());
  for (const auto& level : levels_) {
    const double div = level.pyramid_scale;
    XldContours adjusted;
    for (std::size_t index=0; index<level.contours.contour_count(); ++index) {
      const auto& contour = level.contours.contour(index);
      std::vector<Point2d> pts;
      pts.reserve(contour.size());
      for (const auto& q : contour.points()) pts.push_back({q.row - dr / div, q.col - dc / div});
      adjusted.append(XldContour::from_points_and_attributes(pts, contour.closed(), contour.approximation(),
          contour.point_attribute(core::XldPointAttribute::Response),
          contour.point_attribute(core::XldPointAttribute::EdgeDirection),
          contour.point_attribute(core::XldPointAttribute::NormalAngle)));
    }
    adjusted_levels.push_back(std::move(adjusted));
  }
  auto next_cache = make_shape_model_search_cache();
  static_assert(std::is_nothrow_move_assignable_v<XldContours>);
  // 所有分配完成后再提交；不复制不会修改的训练信息和点数组。
  for (std::size_t i = 0; i < levels_.size(); ++i) {
    auto& level = levels_[i];
    const double div = level.pyramid_scale;
    for (auto& p : level.points) {
      p.row -= dr / div;
      p.column -= dc / div;
    }
    level.contours = std::move(adjusted_levels[i]);
    if (xld_model_) {
      level.radius=0;
      for(std::size_t j=0;j<level.contours.contour_count();++j)
        for(const auto& p:level.contours.contour(j).points())
          level.radius=std::max(level.radius,std::hypot(p.row,p.col));
    }
  }
  origin_row_ = new_r;
  origin_column_ = new_c;
  for (auto& p : refinement_points_) {
    p.row -= dr;
    p.column -= dc;
  }
  search_cache_ = std::move(next_cache);
}

void ShapeModel::set_min_contrast(double min_contrast) {
  if (!std::isfinite(min_contrast) || min_contrast < 0.0) {
    throw Exception(Status::InvalidArgument, "set_shape_model_param", "min_contrast must be nonnegative");
  }
  min_contrast_ = min_contrast;
}

void ShapeModel::set_timeout_ms(double timeout_ms) {
  if (!std::isfinite(timeout_ms)) {
    throw Exception(Status::InvalidArgument, "set_shape_model_param", "timeout must be finite");
  }
  timeout_ms_ = timeout_ms;
}

void ShapeModel::set_border_shape_models(bool enabled) { border_shape_models_ = enabled; }

void ShapeModel::set_model_cache_enabled(bool enabled) {
  model_cache_enabled_ = enabled;
  search_cache_ = enabled ? make_shape_model_search_cache() : nullptr;
}

ShapeModel create_model_impl(const Image& image, const Tuple& num_levels, double angle_start, double angle_extent,
                             const Tuple& angle_step, double scale_min, double scale_max, const Tuple& scale_step,
                             const Tuple& optimization, std::string metric, const Tuple& contrast,
                             const Tuple& min_contrast, ShapeModelKind kind, const CreateParameters* typed) {
  if (!std::isfinite(angle_start) || !std::isfinite(angle_extent) || angle_extent < 0) {
    throw Exception(Status::InvalidArgument, "create_shape_model", "invalid angle range");
  }
  if (kind == ShapeModelKind::IsotropicScale &&
      (!std::isfinite(scale_min) || !std::isfinite(scale_max) || scale_min <= 0 || scale_max < scale_min)) {
    throw Exception(Status::InvalidArgument, "create_scaled_shape_model", "invalid scale range");
  }
  if (image.empty() || image.channel_count() != 1) {
    throw Exception(Status::InvalidArgument, "create_shape_model",
                    "shape model input must be a non-empty single-channel grayscale image");
  }
  const auto reduction = typed ? static_cast<PointReduction>(typed->reduction) : parse_optimization(optimization);
  ShapeModel model;
  model.search_cache_ = make_shape_model_search_cache();
  model.kind_ = kind;
  model.metric_ = typed ? static_cast<ShapeMetric>(typed->metric) : parse_metric(metric);
  model.metric_name_ = std::move(metric);
  model.angle_start_ = angle_start;
  model.angle_extent_ = angle_extent;
  model.scale_min_ = scale_min;
  model.scale_max_ = scale_max;
  double origin_column = 0;
  model.default_origin_row_ = domain_centroid_row(image, &origin_column);
  model.default_origin_column_ = origin_column;
  model.origin_row_ = model.default_origin_row_;
  model.origin_column_ = origin_column;
  const auto source_domain = image.domain();
  const auto source_bounds = source_domain.bounding_box();
  if (!source_bounds) {
    throw Exception(Status::InvalidArgument, "create_shape_model",
                    "template domain must not be empty");
  }
  // 金字塔滤波需要少量域外上下文；自动最多六层，32 像素源 halo 足以保留上下文，
  // 小模型 ROI 无需处理任意大的底图。
  constexpr std::int64_t training_halo = 32;
  const std::int64_t work_row1 =
      std::max<std::int64_t>(0, source_bounds->row_min - training_halo);
  const std::int64_t work_col1 =
      std::max<std::int64_t>(0, source_bounds->col_min - training_halo);
  const std::int64_t work_row2 = std::min<std::int64_t>(
      static_cast<std::int64_t>(image.height()) - 1,
      source_bounds->row_max + training_halo);
  const std::int64_t work_col2 = std::min<std::int64_t>(
      static_cast<std::int64_t>(image.width()) - 1,
      source_bounds->col_max + training_halo);
  const Image working_image = mv::imageproc::crop_rectangle1(
      image, work_row1, work_col1, work_row2, work_col2);
  const double local_origin_row = model.origin_row_ - work_row1;
  const double local_origin_column = model.origin_column_ - work_col1;
  mv::runtime::ScratchArena scratch;
  const bool direct_u8 =
      working_image.pixel_type() == mv::core::PixelType::UInt8;
  std::pmr::vector<double> gray{scratch.resource()};
  std::pmr::vector<double> gx{scratch.resource()};
  std::pmr::vector<double> gy{scratch.resource()};
  if (direct_u8) {
    sobel_u8(working_image, gx, gy);
  } else {
    gray = to_scalar_double(working_image, scratch.resource());
    sobel(gray, working_image.width(), working_image.height(), gx, gy);
  }
  const auto domain = working_image.domain();
  std::pmr::vector<double> domain_mags{scratch.resource()};
  // 数值阈值不使用幅度百分位，显式 contrast 参数无需收集/排序完整 ROI。
  bool needs_statistics = contrast.empty();
  for (std::size_t i = 0; i < contrast.size(); ++i)
    needs_statistics |= contrast.at(i).kind() == mv::core::ValueKind::String;
  if (needs_statistics) for (const auto& run : domain.runs()) {
    for (std::int64_t c = run.col_begin; c <= run.col_end; ++c) {
      if (run.row < 0 || c < 0 || static_cast<std::size_t>(run.row) >= working_image.height() ||
          static_cast<std::size_t>(c) >= working_image.width())
        continue;
      domain_mags.push_back(std::hypot(
          gx[static_cast<std::size_t>(run.row) * working_image.width() +
             static_cast<std::size_t>(c)],
          gy[static_cast<std::size_t>(run.row) * working_image.width() +
             static_cast<std::size_t>(c)]));
    }
  }
  auto contrast_spec = typed
      ? (typed->contrast_low < 0 ? auto_contrast_from(domain_mags,true,true,scratch.resource())
          : ContrastSpec{typed->contrast_low,typed->contrast_high,typed->min_size})
      : parse_contrast(contrast, domain_mags, scratch.resource());
  if(typed && typed->min_size)contrast_spec.min_size=typed->min_size;
  model.min_contrast_ = typed ? (typed->min_contrast < 0 ? std::max(1.0,.5*contrast_spec.low) : typed->min_contrast)
      : parse_min_contrast(min_contrast, contrast_spec.low);
  const std::size_t keep = reduction == PointReduction::None ? 1200 : 400;
  ShapeModelLevel level0;
  level0.level = 0;
  level0.pyramid_scale = 1.0;
  {
    // reduction 使用完整有序候选集，不能先截到细化上限而丢掉空间独立的弱边。
    const auto candidates = extract_contour_points(
        gx, gy, working_image.width(), working_image.height(), domain,
        local_origin_row, local_origin_column, contrast_spec.low,
        contrast_spec.high, contrast_spec.min_size, scratch.resource());
    const auto refinement = std::span(candidates).first(std::min<std::size_t>(1200, candidates.size()));
    mv::runtime::assign_persist(model.refinement_points_, refinement);
    const auto reduced = reduce_contour_points(candidates, keep, reduction);
    mv::runtime::assign_persist(level0.points, reduced);
  }
  if (level0.points.size() < 4) {
    throw Exception(Status::InvalidArgument, "create_shape_model", "template contains too few model points");
  }
  if (model.refinement_points_.empty()) {
    mv::runtime::assign_persist(model.refinement_points_, level0.points);
  }
  for (const auto& p : level0.points) level0.radius = std::max(level0.radius, std::hypot(p.row, p.column));
  const double auto_angle = std::clamp(1.0 / std::max(1.0, level0.radius), 0.002, 0.2);
  model.angle_step_ = (angle_step.empty() || tuple_auto(angle_step) || tuple_number_or(angle_step, 0) == 0)
                          ? auto_angle
                          : tuple_number_or(angle_step, auto_angle);
  if(typed && typed->angle_step>0)model.angle_step_=typed->angle_step;
  if (model.angle_step_ <= 0) {
    throw Exception(Status::InvalidArgument, "create_shape_model", "angle_step must be positive or auto");
  }
  const double auto_scale = std::clamp(1.0 / std::max(1.0, level0.radius), 0.005, 0.1);
  model.scale_step_ = (scale_step.empty() || tuple_auto(scale_step) || tuple_number_or(scale_step, 0) == 0)
                          ? auto_scale
                          : tuple_number_or(scale_step, auto_scale);
  if(typed && typed->scale_step>0)model.scale_step_=typed->scale_step;
  if (model.scale_step_ <= 0) {
    throw Exception(Status::InvalidArgument, "create_scaled_shape_model", "scale_step must be positive or auto");
  }
  try {
    level0.contours = shifted_contours(
        mv::xld::edges_sub_pix(working_image, mv::xld::EdgeFilter::Canny,
                              1.0, contrast_spec.low,
                              contrast_spec.high),
        local_origin_row, local_origin_column, 1.0);
  } catch (...) {
  }
  model.levels_.push_back(level0);
  const std::size_t domain_height = static_cast<std::size_t>(
      source_bounds->row_max - source_bounds->row_min + 1);
  const std::size_t domain_width = static_cast<std::size_t>(
      source_bounds->col_max - source_bounds->col_min + 1);
  const int requested = typed && typed->levels>0 ? typed->levels : parse_levels(num_levels, level0.points.size(), domain_width,
                                     domain_height);
  auto level_domain = domain;
  std::size_t lw = working_image.width();
  std::size_t lh = working_image.height();
  double origin_row = local_origin_row;
  double origin_col = local_origin_column;
  for (int level = 1; level < requested; ++level) {
    if (direct_u8 && level == 1) {
      gray = downsample_gray_u8(working_image, &lw, &lh,
                                scratch.resource());
    } else {
      gray = downsample_gray(gray, lw, lh, &lw, &lh,
                             scratch.resource());
    }
    level_domain = downsample_region(level_domain);
    if (lw < 8 || lh < 8 || level_domain.empty()) break;
    origin_row *= 0.5;
    origin_col *= 0.5;
    std::pmr::vector<double> lgx{scratch.resource()};
    std::pmr::vector<double> lgy{scratch.resource()};
    sobel(gray, lw, lh, lgx, lgy);
    ShapeModelLevel next;
    next.level = level;
    next.pyramid_scale = std::ldexp(1.0, level);
    const auto extracted = extract_contour_points(
        lgx, lgy, lw, lh, level_domain, origin_row, origin_col,
        contrast_spec.low / next.pyramid_scale,
        contrast_spec.high / next.pyramid_scale,
        std::max<std::size_t>(
            2, contrast_spec.min_size /
                   static_cast<std::size_t>(next.pyramid_scale)),
        scratch.resource());
    const auto reduced_level = reduce_contour_points(extracted, keep, reduction);
    mv::runtime::assign_persist(next.points, reduced_level);
    if (next.points.size() < 4) {
      const auto& prev = model.levels_.back();
      for (std::size_t i = 0; i < prev.points.size(); i += 4) {
        auto p = prev.points[i];
        p.row /= 2.0;
        p.column /= 2.0;
        next.points.push_back(p);
      }
    }
    if (next.points.size() < 4) break;
    for (const auto& p : next.points) next.radius = std::max(next.radius, std::hypot(p.row, p.column));
    next.contours = shifted_contours(level0.contours, 0.0, 0.0, next.pyramid_scale);
    model.levels_.push_back(std::move(next));
  }
  if (std::getenv("MVKIT_SHAPE_MODEL_DIAGNOSTICS")) {
    std::fprintf(stderr, "model_info,mvkit,%zu,%.17g,%.17g,%zu\n",
                 model.levels_.size(), model.angle_step_, model.min_contrast_,
                 model.refinement_points_.size());
    for (const auto& level : model.levels_) {
      std::uint64_t hash = 14695981039346656037ULL;
      for (const auto& p : level.points)
        for (double value : {p.row, p.column, p.normal_row, p.normal_column, p.response}) {
          hash ^= std::bit_cast<std::uint64_t>(value);
          hash *= 1099511628211ULL;
        }
      std::fprintf(stderr, "model_points,mvkit,%d,%zu,%llu\n", level.level + 1,
                   level.points.size(), static_cast<unsigned long long>(hash));
    }
  }
  return model;
}

ShapeModel create_model(const core::Image& image,const CreateParameters& p) {
  const char* names[]{"use_polarity","ignore_global_polarity","ignore_local_polarity"};
  return create_model_impl(image,{},p.angle_start,p.angle_extent,{},p.scale_min,p.scale_max,{}, {},names[p.metric],{}, {},
      p.scale_min==1 && p.scale_max==1 ? ShapeModelKind::Rigid : ShapeModelKind::IsotropicScale,&p);
}

ShapeModel create_shape_model(const Image& image, const Tuple& num_levels, double angle_start, double angle_extent,
                              const Tuple& angle_step, const Tuple& optimization, std::string metric,
                              const Tuple& contrast, const Tuple& min_contrast) {
  return create_model_impl(image, num_levels, angle_start, angle_extent, angle_step, 1.0, 1.0, Tuple{}, optimization,
                           std::move(metric), contrast, min_contrast, ShapeModelKind::Rigid, nullptr);
}

ShapeModel create_scaled_shape_model(const Image& image, const Tuple& num_levels, double angle_start,
                                     double angle_extent, const Tuple& angle_step, double scale_min, double scale_max,
                                     const Tuple& scale_step, const Tuple& optimization, std::string metric,
                                     const Tuple& contrast, const Tuple& min_contrast) {
  return create_model_impl(image, num_levels, angle_start, angle_extent, angle_step, scale_min, scale_max, scale_step,
                           optimization, std::move(metric), contrast, min_contrast, ShapeModelKind::IsotropicScale, nullptr);
}

XldContours get_shape_model_contours(const ShapeModel& model, std::int64_t level) {
  if (model.empty()) throw Exception(Status::InvalidState, "get_shape_model_contours", "model is empty");
  if (level < 0 || static_cast<std::size_t>(level) >= model.levels().size()) {
    throw Exception(Status::OutOfRange, "get_shape_model_contours", "level out of range");
  }
  return model.levels()[static_cast<std::size_t>(level)].contours;
}

void set_shape_model_param(ShapeModel& model, const Tuple& names, const Tuple& values) {
  if (names.size() != values.size()) {
    throw Exception(Status::InvalidArgument, "set_shape_model_param", "GenParamName and GenParamValue size must match");
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names.at(i).kind() != mv::core::ValueKind::String) {
      throw Exception(Status::InvalidArgument, "set_shape_model_param", "GenParamName must be strings");
    }
    const auto name = names.string_view(i);
    const auto value = values.at(i);
    if (name == "min_contrast") {
      model.set_min_contrast(value_as_number(value, "set_shape_model_param"));
    } else if (name == "timeout") {
      model.set_timeout_ms(value_as_number(value, "set_shape_model_param"));
    } else if (name == "border_shape_models") {
      if (value.kind() == mv::core::ValueKind::String) {
        const auto text = value.as_string().view();
        if (text == "true") model.set_border_shape_models(true);
        else if (text == "false") model.set_border_shape_models(false);
        else throw Exception(Status::InvalidArgument, "set_shape_model_param", "border_shape_models must be true or false");
      } else if (value.kind() == mv::core::ValueKind::Int64) {
        model.set_border_shape_models(value.as_int64() != 0);
      } else {
        model.set_border_shape_models(value_as_number(value, "set_shape_model_param") != 0.0);
      }
    } else if (name == "model_cache") {
      if (value.kind() != mv::core::ValueKind::String) {
        throw Exception(Status::InvalidArgument, "set_shape_model_param",
                        "model_cache must be true, false, or system");
      }
      const auto text = value.as_string().view();
      if (text == "true" || text == "system") {
        model.set_model_cache_enabled(true);
      } else if (text == "false") {
        model.set_model_cache_enabled(false);
      } else {
        throw Exception(Status::InvalidArgument, "set_shape_model_param",
                        "model_cache must be true, false, or system");
      }
    } else {
      throw Exception(Status::InvalidArgument, "set_shape_model_param", "unsupported shape model parameter");
    }
  }
}

void get_shape_model_params(const ShapeModel& model, Tuple* num_levels, double* angle_start, double* angle_extent,
                            double* angle_step, double* scale_min, double* scale_max, double* scale_step, Tuple* metric,
                            Tuple* min_contrast) {
  if (model.empty()) throw Exception(Status::InvalidState, "get_shape_model_params", "model is empty");
  if (num_levels) *num_levels = Tuple{static_cast<std::int64_t>(model.levels().size())};
  if (angle_start) *angle_start = model.angle_start();
  if (angle_extent) *angle_extent = model.angle_extent();
  if (angle_step) *angle_step = model.angle_step();
  if (scale_min) *scale_min = model.scale_min();
  if (scale_max) *scale_max = model.scale_max();
  if (scale_step) *scale_step = model.scale_step();
  if (metric) *metric = Tuple(model.metric_name());
  if (min_contrast) *min_contrast = Tuple{model.min_contrast()};
}

}  // namespace mv::shape_matching
