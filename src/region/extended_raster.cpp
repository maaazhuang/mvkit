#include <cstdint>
#include <region/extended_common.hpp>
#include <unordered_set>

namespace mv::regionproc {
namespace {
using namespace extended;
// 抛物线下包络给出单轴精确欧氏平方距离。
void edt_line(std::span<const double> f, std::span<double> out, std::span<std::size_t> sites,
              std::span<double> cuts) {
  std::size_t count = 0;
  for (std::size_t q = 0; q < f.size(); ++q) {
    if (!std::isfinite(f[q])) continue;
    double crossing = -INFINITY;
    while (count) {
      const auto p = sites[count - 1];
      crossing = ((f[q] - f[p]) + (double(q) - double(p)) * (double(q) + double(p))) /
                 (2 * (double(q) - double(p)));
      if (crossing > cuts[count - 1]) break;
      --count;
    }
    sites[count] = q;
    cuts[count] = count ? crossing : -INFINITY;
    ++count;
  }
  if (!count) {
    std::fill(out.begin(), out.end(), INFINITY);
    return;
  }
  std::size_t k = 0;
  for (std::size_t q = 0; q < f.size(); ++q) {
    while (k + 1 < count && cuts[k + 1] < double(q)) ++k;
    const double d = double(q) - double(sites[k]);
    out[q] = d * d + f[sites[k]];
  }
}
// 二值输入仅有零种子及 infinity；此有界范围内差、平方及旧种子中点切分均为精确 Double。
void binary_edt_line(std::span<double> values) {
  auto seed = values.size();
  for (std::size_t q = 0; q < values.size(); ++q) {
    if (values[q] == 0)
      seed = q;
    else if (seed != values.size()) {
      const auto d = double(q - seed);
      values[q] = d * d;
    }
  }
  seed = values.size();
  for (std::size_t q = values.size(); q-- > 0;) {
    if (values[q] == 0)
      seed = q;
    else if (seed != values.size()) {
      const auto d = double(seed - q);
      values[q] = std::min(values[q], d * d);
    }
  }
}
void scan_grid(std::span<std::uint32_t> d, std::size_t w, std::size_t h, bool diagonal) {
  for (std::size_t y = 0; y < h; ++y)
    for (std::size_t x = 0; x < w; ++x) {
      auto& v = d[y * w + x];
      if (x) v = std::min(v, d[y * w + x - 1] + 1);
      if (y) {
        v = std::min(v, d[(y - 1) * w + x] + 1);
        if (diagonal) {
          if (x) v = std::min(v, d[(y - 1) * w + x - 1] + 1);
          if (x + 1 < w) v = std::min(v, d[(y - 1) * w + x + 1] + 1);
        }
      }
    }
  for (std::size_t y = h; y-- > 0;)
    for (std::size_t x = w; x-- > 0;) {
      auto& v = d[y * w + x];
      if (x + 1 < w) v = std::min(v, d[y * w + x + 1] + 1);
      if (y + 1 < h) {
        v = std::min(v, d[(y + 1) * w + x] + 1);
        if (diagonal) {
          if (x) v = std::min(v, d[(y + 1) * w + x - 1] + 1);
          if (x + 1 < w) v = std::min(v, d[(y + 1) * w + x + 1] + 1);
        }
      }
    }
}
Image integer_distance(const Region& r, DistanceMetric metric, bool foreground, std::size_t width,
                       std::size_t height, std::pmr::memory_resource* memory) {
  const auto w = width + 2, h = height + 2;
  // 原尺寸守卫保证 w+h<=UINT32_MAX-5，有限种子距离最多 w+h-2。
  // 各单元保留自身旧值，min(old,neighbor+1) 即截断 infinity，无需饱和分支。
  const auto infinity = static_cast<std::uint32_t>(w + h);
  std::pmr::vector<std::uint32_t> d(w * h, foreground ? 0 : infinity, memory);
  for (const auto& run : r.runs()) {
    if (run.row < 0 || run.row >= static_cast<std::int64_t>(height)) continue;
    const auto first = std::max<std::int64_t>(run.col_begin, 0);
    const auto last = std::min<std::int64_t>(run.col_end, width - 1);
    if (first <= last)
      std::fill(d.begin() + (std::int64_t(run.row) + 1) * w + first + 1, d.begin() + (std::int64_t(run.row) + 1) * w + last + 2,
                foreground ? infinity : 0);
  }
  scan_grid(d, w, h, metric == DistanceMetric::Chessboard);
  auto image = Image::create(width, height, PixelType::Float32);
  auto* data = static_cast<unsigned char*>(image.mutable_data());
  for (std::size_t y = 0; y < height; ++y) {
    auto* row = reinterpret_cast<float*>(data + y * image.stride());
    for (std::size_t x = 0; x < width; ++x) {
      const auto value = d[(y + 1) * w + x + 1];
      row[x] = value == infinity ? INFINITY : static_cast<float>(double(value));
    }
  }
  return image;
}
Image distance_one(const Region& r, DistanceMetric metric, bool foreground, std::size_t width,
                   std::size_t height) {
  const auto w = width + 2, h = height + 2;
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  if (foreground) {
    // 矩形连通分量内每点的最近背景位于四侧之一，三种距离度量均成立。
    const auto components = connection(r, Connectivity::Eight);
    bool rectangles = true;
    for (std::size_t i = 0; i < components.size(); ++i) {
      const auto box = components[i].bounding_box();
      if (!box || box->row_min < INT32_MIN || box->row_max > INT32_MAX ||
          box->col_min < INT32_MIN || box->col_max > INT32_MAX) {
        rectangles = false;
        break;
      }
      const auto ww = std::uint64_t(box->col_max - box->col_min) + 1;
      const auto hh = std::uint64_t(box->row_max - box->row_min) + 1;
      if (hh > UINT64_MAX / ww || components[i].area() != ww * hh) {
        rectangles = false;
        break;
      }
    }
    if (rectangles) {
      auto image = Image::create(width, height, PixelType::Float32);
      auto* data = static_cast<std::byte*>(image.mutable_data());
      for (std::size_t i = 0; i < components.size(); ++i) {
        auto box = *components[i].bounding_box();
        box.row_min = std::max<std::int64_t>(box.row_min, 0);
        box.col_min = std::max<std::int64_t>(box.col_min, 0);
        box.row_max = std::min<std::int64_t>(box.row_max, height - 1);
        box.col_max = std::min<std::int64_t>(box.col_max, width - 1);
        for (auto y = box.row_min; y <= box.row_max; ++y) {
          auto* row = reinterpret_cast<float*>(data + y * image.stride());
          const auto dy = std::min(y - box.row_min + 1, box.row_max - y + 1);
          for (auto x = box.col_min; x <= box.col_max; ++x)
            row[x] = static_cast<float>(
                std::min(dy, std::min(x - box.col_min + 1, box.col_max - x + 1)));
        }
      }
      return image;
    }
  }
  if (metric != DistanceMetric::Euclidean)
    return integer_distance(r, metric, foreground, width, height, memory);
  std::pmr::vector<double> d(w * h, foreground ? 0.0 : INFINITY, memory);
  for (const auto& run : r.runs()) {
    if (run.row < 0 || run.row >= static_cast<std::int64_t>(height)) continue;
    const auto first = std::max<std::int64_t>(run.col_begin, 0);
    const auto last = std::min<std::int64_t>(run.col_end, width - 1);
    if (first <= last)
      std::fill(d.begin() + (std::int64_t(run.row) + 1) * w + first + 1, d.begin() + (std::int64_t(run.row) + 1) * w + last + 2,
                foreground ? INFINITY : 0.);
  }
  const auto n = std::max(w, h);
  std::pmr::vector<double> f(n, 0, memory), out(n, 0, memory), cuts(n, 0, memory);
  std::pmr::vector<std::size_t> sites(n, 0, memory);
  for (std::size_t y = 0; y < h; ++y) {
    if (w <= (1U << 24)) {
      binary_edt_line(std::span<double>(d.data() + y * w, w));
    } else {
      edt_line(std::span<const double>(d.data() + y * w, w), std::span<double>(out.data(), w),
               sites, cuts);
      std::copy_n(out.begin(), w, d.begin() + y * w);
    }
  }
  for (std::size_t x = 0; x < w; ++x) {
    for (std::size_t y = 0; y < h; ++y) f[y] = d[y * w + x];
    edt_line(std::span<const double>(f.data(), h), std::span<double>(out.data(), h), sites, cuts);
    for (std::size_t y = 0; y < h; ++y) d[y * w + x] = std::sqrt(out[y]);
  }

  auto image = Image::create(width, height, PixelType::Float32);
  auto* data = static_cast<unsigned char*>(image.mutable_data());
  for (std::size_t y = 0; y < height; ++y) {
    auto* row = reinterpret_cast<float*>(data + y * image.stride());
    for (std::size_t x = 0; x < width; ++x) row[x] = static_cast<float>(d[(y + 1) * w + x + 1]);
  }
  return image;
}

std::uint64_t pack_pixel(std::uint32_t y, std::uint32_t x) {
  return (static_cast<std::uint64_t>(y) << 32) | x;
}

void zhang_counts(const int p[8], int* n, int* transitions) {
  *n = 0;
  *transitions = 0;
  for (int k = 0; k < 8; ++k) {
    *n += p[k];
    *transitions += !p[k] && p[(k + 1) % 8];
  }
}

bool zhang_keep(const int p[8], int pass) {
  // 邻居为 0/1 像素，使用布尔条件避免 GCC -Wint-in-bool-context，Zhang-Suen 保留谓词不变。
  return pass == 0 ? ((p[0] && p[2] && p[4]) || (p[2] && p[4] && p[6]))
                   : ((p[0] && p[2] && p[6]) || (p[0] && p[4] && p[6]));
}

Region thin_dense(const Region& region, const RectI64& box, std::size_t width, std::size_t height) {
  dimensions(width + 2, height + 2, "skeleton");
  const auto w = width + 2, h = height + 2;
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  std::pmr::vector<unsigned char> pixels(w * h, 0, memory), remove(w * h, 0, memory);
  for (const auto& run : region.runs()) {
    const auto y =
        static_cast<std::size_t>(std::uint64_t(run.row) - std::uint64_t(box.row_min)) + 1;
    const auto x0 =
        static_cast<std::size_t>(std::uint64_t(run.col_begin) - std::uint64_t(box.col_min)) + 1;
    const auto x1 =
        static_cast<std::size_t>(std::uint64_t(run.col_end) - std::uint64_t(box.col_min)) + 1;
    std::fill(pixels.begin() + y * w + x0, pixels.begin() + y * w + x1 + 1,
              static_cast<unsigned char>(1));
  }
  auto neighbors = [&](std::size_t i, int p[8]) {
    p[0] = pixels[i - w];
    p[1] = pixels[i - w + 1];
    p[2] = pixels[i + 1];
    p[3] = pixels[i + w + 1];
    p[4] = pixels[i + w];
    p[5] = pixels[i + w - 1];
    p[6] = pixels[i - 1];
    p[7] = pixels[i - w - 1];
  };
  bool changed;
  do {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      std::fill(remove.begin(), remove.end(), static_cast<unsigned char>(0));
      for (std::size_t y = 1; y + 1 < h; ++y)
        for (std::size_t x = 1; x + 1 < w; ++x) {
          const auto i = y * w + x;
          if (!pixels[i]) continue;
          int p[8];
          neighbors(i, p);
          int n = 0, transitions = 0;
          zhang_counts(p, &n, &transitions);
          if (n < 2 || n > 6 || transitions != 1) continue;
          if (!zhang_keep(p, pass)) remove[i] = 1;
        }
      for (std::size_t i = 0; i < pixels.size(); ++i)
        if (remove[i]) {
          int p[8];
          neighbors(i, p);
          int n = 0, transitions = 0;
          zhang_counts(p, &n, &transitions);
          if (n >= 2 && transitions == 1) {
            pixels[i] = 0;
            changed = true;
          }
        }
    }
  } while (changed);
  std::pmr::vector<RegionRun> runs{memory};
  for (std::size_t y = 1; y + 1 < h; ++y)
    for (std::size_t x = 1; x + 1 < w;) {
      if (!pixels[y * w + x]) {
        ++x;
        continue;
      }
      const auto start = x;
      while (x + 1 < w && pixels[y * w + x]) ++x;
      runs.push_back({add(box.row_min, static_cast<std::int64_t>(y - 1)),
                      add(box.col_min, static_cast<std::int64_t>(start - 1)),
                      add(box.col_min, static_cast<std::int64_t>(x - 2))});
    }
  return Region::from_runs(runs);
}

Region thin_sparse(const Region& region, const RectI64& box) {
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  std::pmr::unordered_set<std::uint64_t> live{memory};
  live.reserve(static_cast<std::size_t>(
      std::min<std::uint64_t>(region.area(), std::numeric_limits<std::size_t>::max())));
  for (const auto& run : region.runs()) {
    const auto y =
        static_cast<std::uint32_t>(std::uint64_t(run.row) - std::uint64_t(box.row_min)) + 1;
    const auto x0 =
        static_cast<std::uint32_t>(std::uint64_t(run.col_begin) - std::uint64_t(box.col_min)) + 1;
    const auto x1 =
        static_cast<std::uint32_t>(std::uint64_t(run.col_end) - std::uint64_t(box.col_min)) + 1;
    for (auto x = x0; x <= x1; ++x) live.insert(pack_pixel(y, x));
  }
  auto neighbor = [&](std::uint32_t y, std::uint32_t x, int p[8]) {
    const std::uint32_t ys[8] = {static_cast<std::uint32_t>(y - 1),
                                 static_cast<std::uint32_t>(y - 1),
                                 y,
                                 static_cast<std::uint32_t>(y + 1),
                                 static_cast<std::uint32_t>(y + 1),
                                 static_cast<std::uint32_t>(y + 1),
                                 y,
                                 static_cast<std::uint32_t>(y - 1)};
    const std::uint32_t xs[8] = {x,
                                 static_cast<std::uint32_t>(x + 1),
                                 static_cast<std::uint32_t>(x + 1),
                                 static_cast<std::uint32_t>(x + 1),
                                 x,
                                 static_cast<std::uint32_t>(x - 1),
                                 static_cast<std::uint32_t>(x - 1),
                                 static_cast<std::uint32_t>(x - 1)};
    for (int k = 0; k < 8; ++k) p[k] = live.count(pack_pixel(ys[k], xs[k])) ? 1 : 0;
  };
  bool changed;
  do {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      std::pmr::vector<std::uint64_t> remove{memory};
      remove.reserve(live.size());
      for (auto key : live) {
        const auto y = static_cast<std::uint32_t>(key >> 32);
        const auto x = static_cast<std::uint32_t>(key);
        if (y == 0 || x == 0) continue;
        int p[8];
        neighbor(y, x, p);
        int n = 0, transitions = 0;
        zhang_counts(p, &n, &transitions);
        if (n < 2 || n > 6 || transitions != 1) continue;
        if (!zhang_keep(p, pass)) remove.push_back(key);
      }
      std::sort(remove.begin(), remove.end());
      for (auto key : remove) {
        if (!live.count(key)) continue;
        const auto y = static_cast<std::uint32_t>(key >> 32);
        const auto x = static_cast<std::uint32_t>(key);
        int p[8];
        neighbor(y, x, p);
        int n = 0, transitions = 0;
        zhang_counts(p, &n, &transitions);
        if (n >= 2 && transitions == 1) {
          live.erase(key);
          changed = true;
        }
      }
    }
  } while (changed);
  std::pmr::vector<std::uint64_t> keys{live.begin(), live.end(), memory};
  std::sort(keys.begin(), keys.end());
  std::pmr::vector<RegionRun> runs{memory};
  for (std::size_t i = 0; i < keys.size();) {
    const auto y = static_cast<std::uint32_t>(keys[i] >> 32);
    auto x = static_cast<std::uint32_t>(keys[i]);
    const auto start = x;
    ++i;
    while (i < keys.size() && static_cast<std::uint32_t>(keys[i] >> 32) == y &&
           static_cast<std::uint32_t>(keys[i]) == x + 1) {
      ++x;
      ++i;
    }
    runs.push_back({add(box.row_min, static_cast<std::int64_t>(y - 1)),
                    add(box.col_min, static_cast<std::int64_t>(start - 1)),
                    add(box.col_min, static_cast<std::int64_t>(x - 1))});
  }
  return Region::from_runs(runs);
}

Region thin_rectangle(const RectI64& box, std::size_t width, std::size_t height) {
  // 矩形有精确中心线段；短轴为偶数时 HALCON 栅格同值规则使用上/右与下/左对角点对。
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  auto emit = [&](std::size_t y, std::size_t x0, std::size_t x1) {
    runs.push_back({add(box.row_min, static_cast<std::int64_t>(y)),
                    add(box.col_min, static_cast<std::int64_t>(x0)),
                    add(box.col_min, static_cast<std::int64_t>(x1))});
  };
  if (height <= width) {
    const auto half = height / 2;
    if (height & 1) {
      emit(half, half, width - half - 1);
    } else {
      emit(half - 1, width - half, width - half);
      emit(half, half - 1, width - half - 1);
    }
  } else {
    const auto half = width / 2;
    const auto x = half;
    const auto first = (width & 1) ? half : half - 1;
    const auto last = height - half - 1;
    for (auto y = first; y <= last; ++y) emit(y, x, x);
    if (!(width & 1)) emit(height - half, half - 1, half - 1);
  }
  return Region::from_runs(runs);
}

Region thin_component(const Region& region) {
  const auto box = region.bounding_box();
  if (!box) return region;
  const auto width = extent(box->col_min, box->col_max, "skeleton"),
             height = extent(box->row_min, box->row_max, "skeleton");
  if (width > 1 && height > 1 && width <= std::numeric_limits<std::uint64_t>::max() / height &&
      region.area() == width * height)
    return thin_rectangle(*box, width, height);
  if (occupancy_fits(width + 2, height + 2) || region.area() > kOccupancyLimit)
    return thin_dense(region, *box, width, height);
  return thin_sparse(region, *box);
}
Region thin(const Region& region) {
  if (region.empty()) return region;
  const auto parts = connection(region, Connectivity::Eight);
  if (parts.size() == 1) return thin_component(parts[0]);
  ExecutionContext context;
  std::pmr::vector<RegionRun> runs{context.scratch_resource()};
  for (std::size_t i = 0; i < parts.size(); ++i) {
    const auto skeleton_part = thin_component(parts[i]);
    for (const auto& run : skeleton_part.runs()) runs.push_back(run);
  }
  return Region::from_runs(runs);
}
}  // namespace
mv::core::Regions skeleton(const mv::core::Regions& regions) { return map(regions, thin); }
mv::core::IconicTuple distance_transform(const mv::core::Regions& regions, DistanceMetric metric,
                                         bool foreground, std::size_t width, std::size_t height) {
  check(metric <= DistanceMetric::Chessboard, "distance_transform", "Metric", "invalid metric");
  dimensions(width, height, "distance_transform");
  dimensions(width + 2, height + 2, "distance_transform");
  ExecutionContext context;
  std::pmr::vector<Image> images(regions.size(), context.scratch_resource());
  ExecutionContext::for_each_partition(
      regions.size(),
      ExecutionContext::saturated_work(regions.size(),
                                       ExecutionContext::saturated_work(width, height)),
      32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          images[i] = distance_one(regions[i], metric, foreground, width, height);
      });
  std::vector<IconicValue> values;
  values.reserve(regions.size());
  for (auto& image : images) values.push_back(IconicValue::from_image(std::move(image)));
  return IconicTuple{std::move(values)};
}
}  // namespace mv::regionproc
