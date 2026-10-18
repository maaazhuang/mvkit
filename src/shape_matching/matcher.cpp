#include <shape_matching/operators.hpp>
#include <imageproc/gray_pyramid.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <memory_resource>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#include <emmintrin.h>
#define MVKIT_SHAPE_HAS_SSE2 1
#else
#define MVKIT_SHAPE_HAS_SSE2 0
#endif
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
#include "gradient_simd.hpp"
#endif
#include "discrete_shape_point.hpp"
#include "score_pruning.hpp"
#include "direction_evidence.hpp"
#include "coarse_angle_phase.hpp"
#include "trial_sort.hpp"

#include <core/exception.hpp>
#include <core/value.hpp>
#include <imageproc/image_view.hpp>
#include <imageproc/execution_context.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/env.hpp>
#include <runtime/execution/execution_context.hpp>
#include <runtime/memory/scratch_arena.hpp>
#include <runtime/memory/memory_resource.hpp>

namespace mv::shape_matching {

struct PreparedShapePose {
  double angle{};
  double scale{1.0};
  std::vector<ShapeModelPoint> points;
  std::vector<DiscreteShapePoint> discrete_points;
  std::pmr::vector<PackedI8ShapePoint> packed_points{mv::runtime::persist_resource()};
  std::array<DiscreteShapePoint, 4> anchor_points{};
  std::size_t anchor_count{};
  double min_row{};
  double max_row{};
  double min_col{};
  double max_col{};
};

struct ShapeSearchPlanKey {
  std::size_t top{};
  std::size_t image_width{};
  double angle_start{};
  double angle_extent{};
  double scale_min{};
  double scale_max{};
  bool scaled{};
  bool continuous_points{};
  CoarseAnglePhase phase{CoarseAnglePhase::Full};

  bool operator==(const ShapeSearchPlanKey& other) const noexcept {
    return top == other.top && image_width == other.image_width &&
           angle_start == other.angle_start && angle_extent == other.angle_extent &&
           scale_min == other.scale_min && scale_max == other.scale_max &&
           scaled == other.scaled && continuous_points == other.continuous_points && phase == other.phase;
  }
};

struct PreparedPointSoA {
  std::vector<float> row,column,nr,nc;
};

struct ShapeModelSearchCache {
  struct Entry {
    bool valid{};
    ShapeSearchPlanKey key;
    std::shared_ptr<const std::vector<PreparedShapePose>> plan;
  };
  std::mutex mutex;
  // 保留两个互补计划热态，负图像以同一模型重复搜索时失败首阶段不能反复逐出自身。
  std::array<Entry, 2> entries;
  std::vector<std::shared_ptr<const PreparedPointSoA>> point_levels;
  std::shared_ptr<const PreparedPointSoA> refinement_points;
  std::shared_ptr<const matching::Box> overlap_box;
};

std::shared_ptr<ShapeModelSearchCache> make_shape_model_search_cache() {
  return std::make_shared<ShapeModelSearchCache>();
}

namespace {
std::shared_ptr<const PreparedPointSoA> prepared_point_soa(const ShapeModel& model,std::size_t level,
                                                        bool refinement = false) {
  const bool use_refinement = refinement && level == 0 && !model.refinement_points().empty();
  auto build=[&] {
    auto out=std::make_shared<PreparedPointSoA>();
    const std::span<const ShapeModelPoint> points = use_refinement
        ? std::span<const ShapeModelPoint>{model.refinement_points()}
        : std::span<const ShapeModelPoint>{model.levels()[level].points};
    const auto n=points.size();
    out->row.resize(n);out->column.resize(n);out->nr.resize(n);out->nc.resize(n);
    for(std::size_t i=0;i<n;++i) {
      out->row[i]=static_cast<float>(points[i].row);out->column[i]=static_cast<float>(points[i].column);
      out->nr[i]=static_cast<float>(points[i].normal_row);out->nc[i]=static_cast<float>(points[i].normal_column);
    }
    return out;
  };
  if (!model.model_cache_enabled() || !model.search_cache()) return build();
  auto& cache=*model.search_cache();
  std::lock_guard lock(cache.mutex);
  if(cache.point_levels.empty())cache.point_levels.resize(model.levels().size());
  auto& out=use_refinement ? cache.refinement_points : cache.point_levels[level];
  if(!out)out=build();
  return out;
}
// 中等均匀行遍历限制分派成本；大图保持自适应划分，稀疏 ROI 已执行自身区域。
constexpr std::size_t uniform_row_partitions(std::size_t pixels) noexcept {
  return pixels < 4u * 1024u * 1024u ? 8 : 0;
}
using mv::core::Exception;
using mv::core::Image;
using mv::core::Status;
using mv::core::Tuple;
using mv::core::TupleKind;
using mv::core::ValueKind;

template <typename T>
class PmrBuffer {
 public:
  PmrBuffer() = default;
  PmrBuffer(std::size_t size, std::pmr::memory_resource* memory)
      : size_(size), memory_(memory) {
    if (size_ != 0) {
      data_ = static_cast<T*>(memory_->allocate(size_ * sizeof(T), 32));
      std::uninitialized_default_construct_n(data_, size_);
    }
  }
  ~PmrBuffer() {
    if (data_ && memory_) {
      std::destroy_n(data_, size_);
      memory_->deallocate(data_, size_ * sizeof(T), 32);
    }
  }
  PmrBuffer(const PmrBuffer&) = delete;
  PmrBuffer& operator=(const PmrBuffer&) = delete;
  PmrBuffer(PmrBuffer&& other) noexcept
      : data_(std::exchange(other.data_, nullptr)),
        size_(std::exchange(other.size_, 0)),
        memory_(other.memory_) {}
  PmrBuffer& operator=(PmrBuffer&& other) noexcept {
    if (this == &other) return *this;
    if (data_ && memory_) {
      std::destroy_n(data_, size_);
      memory_->deallocate(data_, size_ * sizeof(T), 32);
    }
    data_ = std::exchange(other.data_, nullptr);
    size_ = std::exchange(other.size_, 0);
    memory_ = other.memory_;
    return *this;
  }
  // 借用 MultiQueryCache 拥有的不可变完整查询分量。
  static PmrBuffer borrow(const PmrBuffer& source) {
    PmrBuffer view;view.data_=source.data_;view.size_=source.size_;return view;
  }
  [[nodiscard]] T* data() noexcept { return data_; }
  [[nodiscard]] const T* data() const noexcept { return data_; }
  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
  T& operator[](std::size_t index) noexcept { return data_[index]; }
  const T& operator[](std::size_t index) const noexcept {
    return data_[index];
  }

 private:
  T* data_{};
  std::size_t size_{};
  std::pmr::memory_resource* memory_{};
};

using PmrFloatBuffer = PmrBuffer<float>;

struct PyramidRect {
  std::size_t r0{}, r1{}, c0{}, c1{};
};

struct GrayLevel {
  std::size_t w{};
  std::size_t h{};
  PmrFloatBuffer pixels;
  bool limited{};
  PyramidRect valid{};

  GrayLevel(std::size_t width, std::size_t height,
            std::pmr::memory_resource* memory)
      : w(width), h(height), memory(memory) {}
  std::pmr::memory_resource* memory{};
};
bool clip_sobel_to_gray(const GrayLevel& src, std::size_t& r0, std::size_t& r1,
                        std::size_t& c0, std::size_t& c1) {
  if (src.limited) {
    const auto& v = src.valid;
    if (v.r0 >= v.r1 || v.c0 >= v.c1) return false;
    r0 = std::max(r0, v.r0 + (v.r0 != 0));
    r1 = std::min(r1, v.r1 - (v.r1 != src.h));
    c0 = std::max(c0, v.c0 + (v.c0 != 0));
    c1 = std::min(c1, v.c1 - (v.c1 != src.w));
    // 保留完整行 SIMD 晶格及标量尾部，ROI 变化不能让像素切换浮点路径。
    if (c0 != 0) c0 = 1 + ((c0 - 1 + 7) / 8) * 8;
    if (c1 != src.w && c1 != 0) c1 = 1 + ((c1 - 1) / 8) * 8;
  }
  return r0 < r1 && c0 < c1;
}

struct Grad {
  std::size_t w{};
  std::size_t h{};
  std::pmr::vector<float> gx;
  std::pmr::vector<float> gy;
  PmrBuffer<std::int16_t> gx_i16;
  PmrBuffer<std::int16_t> gy_i16;
  PmrBuffer<float> inverse_magnitude;
  PmrBuffer<std::uint32_t> packed_components;
  // 不可变完整分量的非拥有查询视图；仅多模型整图缓冲设置该指针，候选局部 band 不缓存。
  Grad* complete_owner{};
  double integer_scale{1.0};
  std::pmr::memory_resource* memory{};

  Grad(std::size_t width, std::size_t height,
       std::pmr::memory_resource* mem)
      : w(width), h(height), gx(mem), gy(mem), memory(mem) {}

  [[nodiscard]] bool has_integer_components() const noexcept {
    return gx_i16.size() != 0;
  }
};

#if MVKIT_SHAPE_HAS_AVX2_KERNEL
const float* inverse_magnitude_data(const Grad& g) noexcept {
  return g.complete_owner ? g.complete_owner->inverse_magnitude.data() : nullptr;
}
const std::uint32_t* packed_components_data(const Grad& g) noexcept {
  return g.complete_owner ? g.complete_owner->packed_components.data() : nullptr;
}
void maybe_prepare_gradient_caches(Grad& g, std::size_t point_work) {
  if (!g.complete_owner || !simd::cpu_has_avx2()) return;
  auto& complete = *g.complete_owner;
  // 整图准备比稀疏/小候选束更贵；此工作量门限不改变试探生成或评分拒绝。
  const auto pixels = complete.w * complete.h;
  if (point_work < (pixels + 1) / 2) return;
  const bool diagnostics = mv::runtime::getenv_or_null("MVKIT_SHAPE_DIAGNOSTICS") != nullptr;
  const auto* packed_policy = mv::runtime::getenv_or_null("MVKIT_SHAPE_CACHE_PACKED_GRADIENT");
  if (complete.packed_components.empty() && !(packed_policy && std::string_view(packed_policy) == "0")) {
    const auto begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    const auto n = complete.w * complete.h;
    complete.packed_components = PmrBuffer<std::uint32_t>(n, complete.memory);
    simd::avx2_kernels().prepare_i16_packed_components(complete.gx_i16.data(),
        complete.gy_i16.data(), n, complete.packed_components.data());
    if (diagnostics) std::fprintf(stderr,"mvkit_shape_packed_gradient prepare_us=%.3f pixels=%zu point_work=%zu bytes=%zu\n",
        std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count(),n,point_work,n*sizeof(std::uint32_t));
  }
  if (point_work < 2 * pixels) return;
  if (!complete.inverse_magnitude.empty()) return;
  const auto* policy = mv::runtime::getenv_or_null("MVKIT_SHAPE_CACHE_NORMALIZATION");
  if (policy && std::string_view(policy) == "0") return;
  const auto begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  complete.inverse_magnitude = PmrBuffer<float>(complete.w * complete.h, complete.memory);
  simd::avx2_kernels().prepare_i16_inverse_magnitude(complete.gx_i16.data(),
      complete.gy_i16.data(), static_cast<float>(complete.integer_scale),
      complete.w * complete.h, complete.inverse_magnitude.data());
  if (diagnostics) std::fprintf(stderr,"mvkit_shape_inverse_magnitude prepare_us=%.3f pixels=%zu point_work=%zu bytes=%zu\n",
      std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count(),
      complete.w*complete.h,point_work,complete.w*complete.h*sizeof(float));
}
double score_i16_nearest_components(const Grad& g, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  if (const auto* packed = packed_components_data(g))
    return simd::avx2_kernels().score_i16_nearest_packed_use_polarity(
        g.gx_i16.data(), g.gy_i16.data(), packed, inverse_magnitude_data(g), g.w,
        static_cast<float>(g.integer_scale), min_mag_squared, prow, pcol, pnr,
        pnc, n, row, col, ca, sa, scale, greediness, reject_below, metric);
  if (const auto* inverse = inverse_magnitude_data(g))
    return simd::avx2_kernels().score_i16_nearest_cached_use_polarity(
        g.gx_i16.data(), g.gy_i16.data(), inverse, g.w,
        static_cast<float>(g.integer_scale), min_mag_squared, prow, pcol, pnr,
        pnc, n, row, col, ca, sa, scale, greediness, reject_below, metric);
  return simd::avx2_kernels().score_i16_nearest_use_polarity(g.gx_i16.data(),
      g.gy_i16.data(), g.w, static_cast<float>(g.integer_scale), min_mag_squared,
      prow, pcol, pnr, pnc, n, row, col, ca, sa, scale, greediness, reject_below, metric);
}
double score_i16_pre_rotated_components(const Grad& g, float min_mag_squared,
    const float* off_r, const float* off_c, const float* nr, const float* nc,
    std::size_t n, float row, float col, float greediness, float reject_below,
    int metric) noexcept {
  if (const auto* packed = packed_components_data(g))
    return simd::avx2_kernels().score_i16_pre_rotated_packed_use_polarity(
        g.gx_i16.data(), g.gy_i16.data(), packed, inverse_magnitude_data(g), g.w,
        static_cast<float>(g.integer_scale), min_mag_squared, off_r, off_c, nr,
        nc, n, row, col, greediness, reject_below, metric);
  if (const auto* inverse = inverse_magnitude_data(g))
    return simd::avx2_kernels().score_i16_pre_rotated_cached_use_polarity(
        g.gx_i16.data(), g.gy_i16.data(), inverse, g.w,
        static_cast<float>(g.integer_scale), min_mag_squared, off_r, off_c, nr,
        nc, n, row, col, greediness, reject_below, metric);
  return simd::avx2_kernels().score_i16_pre_rotated_use_polarity(g.gx_i16.data(),
      g.gy_i16.data(), g.w, static_cast<float>(g.integer_scale), min_mag_squared,
      off_r, off_c, nr, nc, n, row, col, greediness, reject_below, metric);
}
double score_i16_bilinear_components(const Grad& g, float min_mag_squared,
    const float* prow, const float* pcol, const float* pnr, const float* pnc,
    std::size_t n, float row, float col, float ca, float sa, float scale,
    float greediness, float reject_below, int metric) noexcept {
  if (const auto* packed = packed_components_data(g))
    return simd::avx2_kernels().score_i16_bilinear_packed_use_polarity(
        g.gx_i16.data(), g.gy_i16.data(), packed, g.w,
        static_cast<float>(g.integer_scale), min_mag_squared, prow, pcol, pnr,
        pnc, n, row, col, ca, sa, scale, greediness, reject_below, metric);
  return simd::avx2_kernels().score_i16_bilinear_use_polarity(g.gx_i16.data(),
      g.gy_i16.data(), g.w, static_cast<float>(g.integer_scale), min_mag_squared,
      prow, pcol, pnr, pnc, n, row, col, ca, sa, scale, greediness, reject_below, metric);
}
void geometric_i16_bilinear_components(const Grad& g, float expected_row,
    float expected_col, float normal_row, float normal_col, float search_step,
    int span, float min_squared, float* responses, bool ignore_polarity) noexcept {
  if (const auto* packed = packed_components_data(g)) {
    simd::avx2_kernels().geometric_i16_bilinear_packed_line(g.gx_i16.data(),
        g.gy_i16.data(), packed, g.w, static_cast<float>(g.integer_scale),
        expected_row, expected_col, normal_row, normal_col, search_step, span,
        min_squared, responses, ignore_polarity);
    return;
  }
  simd::avx2_kernels().geometric_i16_bilinear_line(g.gx_i16.data(),
      g.gy_i16.data(), g.w, static_cast<float>(g.integer_scale), expected_row,
      expected_col, normal_row, normal_col, search_step, span, min_squared,
      responses, ignore_polarity);
}
simd::GeometricPeak geometric_i16_bilinear_peak17_components(const Grad& g,
    float expected_row, float expected_col, float normal_row, float normal_col,
    float search_step, float min_squared, bool ignore_polarity) noexcept {
  if (const auto* packed = packed_components_data(g))
    return simd::avx2_kernels().geometric_i16_bilinear_packed_peak17(
        g.gx_i16.data(), g.gy_i16.data(), packed, g.w,
        static_cast<float>(g.integer_scale), expected_row, expected_col,
        normal_row, normal_col, search_step, min_squared, ignore_polarity);
  return simd::avx2_kernels().geometric_i16_bilinear_peak17(
      g.gx_i16.data(), g.gy_i16.data(), g.w, static_cast<float>(g.integer_scale),
      expected_row, expected_col, normal_row, normal_col, search_step,
      min_squared, ignore_polarity);
}
#endif

struct UnitGrad {
  std::size_t w{};
  std::size_t h{};
  PmrFloatBuffer nx;
  PmrFloatBuffer ny;
  std::pmr::memory_resource* memory{};

  UnitGrad(std::size_t width, std::size_t height,
           std::pmr::memory_resource* mem, bool allocate = true)
      : w(width), h(height), nx(allocate ? width * height : 0, mem),
        ny(allocate ? width * height : 0, mem), memory(mem) {}

  void ensure_storage() {
    if (nx.size() != 0 || memory == nullptr || w * h == 0) return;
    nx = PmrFloatBuffer(w * h, memory);
    ny = PmrFloatBuffer(w * h, memory);
  }

  [[nodiscard]] bool has_unit() const noexcept { return nx.size() != 0; }
};

struct GradientPyramid {
  std::vector<Grad> gradients;
  std::vector<UnitGrad> unit_gradients;
  std::vector<GrayLevel> gray_levels;
  std::pmr::vector<PyramidRect> regions;
  // 粗层单位梯度在查询内不可变，多模型/互补单模型阶段使用同一量化及交错平面，
  // 与拥有源数据的金字塔共同保存。
  std::pmr::vector<std::int8_t> coarse_qnx, coarse_qny, coarse_query_pairs;
  explicit GradientPyramid(std::pmr::memory_resource* memory)
      : regions(memory), coarse_qnx(memory), coarse_qny(memory),
        coarse_query_pairs(memory) {}
};

struct TimeoutWatch {
  mv::runtime::ExecutionContext* context{nullptr};
  void poll() const {
    if (context == nullptr) return;
    switch (context->poll()) {
      case mv::runtime::StopReason::DeadlineExceeded:
        throw Exception(Status::LimitExceeded, "find_shape_model",
                        "search timeout elapsed");
      case mv::runtime::StopReason::Cancelled:
        throw Exception(Status::InvalidState, "find_shape_model",
                        "search cancelled");
      case mv::runtime::StopReason::None:
        break;
    }
  }
};

struct SubPixelSpec {
  enum class Mode { None, Interpolation, LeastSquares, LeastSquaresHigh, LeastSquaresVeryHigh };
  Mode mode{Mode::LeastSquares};
  int max_deformation{0};
};

struct SearchLevels {
  std::size_t coarsest{0};
  std::size_t finest{0};
};

struct ShapeSearchDiagnostics {
  double gradient_us{};
  double unit_gradient_us{};
  double coarse_us{};
  double refine_us{};
  double final_score_us{};
  double refine_prepare_us{};
  double refine_generate_us{};
  double refine_score_us{};
  double refine_dedup_us{};
  double output_us{};
  std::size_t coarse_evaluated{};
  std::size_t coarse_accepted{};
  std::size_t coarse_scored{};
  std::size_t refine_evaluated{};
  std::size_t refine_accepted{};
  std::size_t final_candidates{};
  std::size_t invert_generated{};
  std::size_t invert_verified{};
};

double elapsed_us(std::chrono::steady_clock::time_point begin,
                  std::chrono::steady_clock::time_point end) noexcept {
  return std::chrono::duration<double, std::micro>(end - begin).count();
}

GrayLevel to_gray(const Image& image, std::pmr::memory_resource* memory) {
  mv::imageproc::ImageView view{image};
  GrayLevel out{image.width(), image.height(), memory};
  out.pixels = mv::imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
    using T = typename decltype(tag)::type;
    PmrFloatBuffer values(image.width() * image.height(), memory);
    for (std::size_t r = 0; r < image.height(); ++r) {
      const auto* row = view.row_as<T>(r);
      for (std::size_t c = 0; c < image.width(); ++c)
        values[r * image.width() + c] = static_cast<float>(row[c]);
    }
    return values;
  });
  return out;
}

GrayLevel downsample(const GrayLevel& src,
                     std::pmr::memory_resource* memory, const PyramidRect* region = nullptr,
                     const TimeoutWatch* timeout = nullptr) {
  GrayLevel dst{std::max<std::size_t>(1, src.w / 2),
                std::max<std::size_t>(1, src.h / 2), memory};
  dst.pixels = PmrFloatBuffer(dst.w * dst.h, dst.memory);
  const PyramidRect bounds = region ? *region : PyramidRect{0, dst.h, 0, dst.w};
  dst.limited = region != nullptr; dst.valid = bounds;
  auto process_rows = [&](mv::runtime::IndexRange rows) {
    for (std::size_t r = rows.begin; r < rows.end; ++r) {
      if (timeout && (r & 31) == 0) timeout->poll();
      const std::size_t r0 = std::min(src.h - 1, 2 * r);
      const std::size_t r1 = std::min(src.h - 1, r0 + 1);
      const float* __restrict row0 = src.pixels.data() + r0 * src.w;
      const float* __restrict row1 = src.pixels.data() + r1 * src.w;
      float* __restrict out = dst.pixels.data() + r * dst.w;
      std::size_t c = bounds.c0;
#if MVKIT_SHAPE_HAS_SSE2
      // 一次读入 8 个源像素，两两相加后再与下一行平均，写出 4 个 float。
      const __m128 quarter = _mm_set1_ps(0.25F);
      for (; c + 4 <= bounds.c1 && 2 * c + 7 < src.w; c += 4) {
        const std::size_t c0 = 2 * c;
        const __m128 a0 = _mm_loadu_ps(row0 + c0);
        const __m128 a1 = _mm_loadu_ps(row0 + c0 + 4);
        const __m128 b0 = _mm_loadu_ps(row1 + c0);
        const __m128 b1 = _mm_loadu_ps(row1 + c0 + 4);
        const __m128 even = _mm_shuffle_ps(a0, a1, 0x88);
        const __m128 odd = _mm_shuffle_ps(a0, a1, 0xDD);
        const __m128 even_b = _mm_shuffle_ps(b0, b1, 0x88);
        const __m128 odd_b = _mm_shuffle_ps(b0, b1, 0xDD);
        const __m128 sum = _mm_add_ps(_mm_add_ps(even, odd),
                                      _mm_add_ps(even_b, odd_b));
        _mm_storeu_ps(out + c, _mm_mul_ps(sum, quarter));
      }
#endif
      for (; c < bounds.c1; ++c) {
        out[c] = mv::imageproc::box2_gray(row0,row1,c,src.w);
      }
    }
  };
  if (region) process_rows({bounds.r0, bounds.r1});
  else mv::imageproc::ExecutionContext::for_each_partition(
      dst.h, src.w * src.h, 256u * 1024u, process_rows,
      uniform_row_partitions(src.w * src.h));
  return dst;
}

void fill_unit_sobel_rows(const GrayLevel& src, UnitGrad& unit,
                          float min_squared, std::size_t row_begin,
                          std::size_t row_end, bool parallel,
                          std::size_t col_begin = 0,
                          std::size_t col_end = static_cast<std::size_t>(-1)) {
  if (!unit.has_unit() || src.pixels.empty() || src.w != unit.w ||
      src.h != unit.h || unit.w == 0 || unit.h == 0)
    return;
  if (row_end > unit.h) row_end = unit.h;
  if (row_begin >= row_end) return;
  if (col_end > unit.w) col_end = unit.w;
  if (col_begin >= col_end) return;
  if (!clip_sobel_to_gray(src, row_begin, row_end, col_begin, col_end)) return;
  const bool full_width = col_begin == 0 && col_end == unit.w;
  auto process_rows = [&](mv::runtime::IndexRange rows) {
    for (std::size_t r = rows.begin; r < rows.end; ++r) {
      const std::size_t previous_row = r == 0 ? 0 : r - 1;
      const std::size_t next_row = r + 1 < unit.h ? r + 1 : unit.h - 1;
      const float* top = src.pixels.data() + previous_row * unit.w;
      const float* middle = src.pixels.data() + r * unit.w;
      const float* bottom = src.pixels.data() + next_row * unit.w;
      float* __restrict nx_row = unit.nx.data() + r * unit.w;
      float* __restrict ny_row = unit.ny.data() + r * unit.w;
      auto write_unit = [&](std::size_t c) {
        const std::size_t left = c == 0 ? 0 : c - 1;
        const std::size_t right = c + 1 < unit.w ? c + 1 : unit.w - 1;
        const float sx =
            (top[right] + 2.0F * middle[right] + bottom[right]) -
            (top[left] + 2.0F * middle[left] + bottom[left]);
        const float sy =
            (bottom[left] + 2.0F * bottom[c] + bottom[right]) -
            (top[left] + 2.0F * top[c] + top[right]);
        const float gx = sx * 0.25F;
        const float gy = sy * 0.25F;
        const float squared = gx * gx + gy * gy;
        if (squared >= min_squared) {
          const float inverse = 1.0F / std::sqrt(squared);
          nx_row[c] = gx * inverse;
          ny_row[c] = gy * inverse;
        } else {
          nx_row[c] = 0.0F;
          ny_row[c] = 0.0F;
        }
      };
      if (unit.w == 1) {
        write_unit(0);
        continue;
      }
      std::size_t c = col_begin;
      if (c == 0) {
        write_unit(0);
        c = 1;
      }
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if ((full_width || src.limited) && simd::cpu_has_avx2()) {
        const auto offset = c - 1;
        c = offset + simd::avx2_kernels().unit_sobel_f32_row(
            top + offset, middle + offset, bottom + offset, nx_row + offset, ny_row + offset,
            col_end - offset + (col_end != unit.w), min_squared);
      } else
#endif
#if MVKIT_SHAPE_HAS_SSE2
      {
        const __m128 quarter = _mm_set1_ps(0.25F);
        const __m128 one = _mm_set1_ps(1.0F);
        const __m128 threshold = _mm_set1_ps(min_squared);
        const std::size_t simd_end = col_end < unit.w ? col_end : unit.w - 1;
        for (; c + 3 < simd_end; c += 4) {
          const __m128 top_left = _mm_loadu_ps(top + c - 1);
          const __m128 top_center = _mm_loadu_ps(top + c);
          const __m128 top_right = _mm_loadu_ps(top + c + 1);
          const __m128 middle_left = _mm_loadu_ps(middle + c - 1);
          const __m128 middle_right = _mm_loadu_ps(middle + c + 1);
          const __m128 bottom_left = _mm_loadu_ps(bottom + c - 1);
          const __m128 bottom_center = _mm_loadu_ps(bottom + c);
          const __m128 bottom_right = _mm_loadu_ps(bottom + c + 1);
          const __m128 sx = _mm_sub_ps(
              _mm_add_ps(_mm_add_ps(top_right, bottom_right),
                         _mm_add_ps(middle_right, middle_right)),
              _mm_add_ps(_mm_add_ps(top_left, bottom_left),
                         _mm_add_ps(middle_left, middle_left)));
          const __m128 sy = _mm_sub_ps(
              _mm_add_ps(_mm_add_ps(bottom_left, bottom_right),
                         _mm_add_ps(bottom_center, bottom_center)),
              _mm_add_ps(_mm_add_ps(top_left, top_right),
                         _mm_add_ps(top_center, top_center)));
          const __m128 gx = _mm_mul_ps(sx, quarter);
          const __m128 gy = _mm_mul_ps(sy, quarter);
          const __m128 squared =
              _mm_add_ps(_mm_mul_ps(gx, gx), _mm_mul_ps(gy, gy));
          const __m128 live = _mm_cmpge_ps(squared, threshold);
          const __m128 inverse = _mm_div_ps(one, _mm_sqrt_ps(squared));
          _mm_storeu_ps(nx_row + c, _mm_and_ps(live, _mm_mul_ps(gx, inverse)));
          _mm_storeu_ps(ny_row + c, _mm_and_ps(live, _mm_mul_ps(gy, inverse)));
        }
      }
#endif
      const std::size_t last = col_end;
      for (; c + 1 < last && c + 1 < unit.w; ++c) write_unit(c);
      if (last == unit.w) write_unit(unit.w - 1);
      else if (c < last) write_unit(c);
    }
  };
  if (parallel && full_width && row_begin == 0 && row_end == unit.h) {
    mv::imageproc::ExecutionContext::for_each_partition(
        unit.h, unit.w * unit.h, 256u * 1024u, process_rows,
        uniform_row_partitions(unit.w * unit.h));
  } else {
    process_rows({row_begin, row_end});
  }
}

void ensure_unit_for_candidates(UnitGrad& unit, const GrayLevel& gray,
                                double min_magnitude, double pyramid_scale,
                                double radius,
                                const std::vector<MatchCandidate>& cand,
                                int max_deformation) {
  if (unit.has_unit() || gray.pixels.empty() || cand.empty()) return;
  unit.ensure_storage();
  double row0 = static_cast<double>(unit.h);
  double row1 = 0.0;
  double col0 = static_cast<double>(unit.w);
  double col1 = 0.0;
  const double inv_scale = 1.0 / std::max(1.0, pyramid_scale);
  for (const auto& x : cand) {
    const double pad = std::abs(x.scale) * radius +
                       static_cast<double>(max_deformation) * inv_scale + 8.0;
    const double level_row = x.row * inv_scale;
    const double level_col = x.column * inv_scale;
    row0 = std::min(row0, level_row - pad);
    row1 = std::max(row1, level_row + pad);
    col0 = std::min(col0, level_col - pad);
    col1 = std::max(col1, level_col + pad);
  }
  auto r_begin = static_cast<std::size_t>(std::max(0.0, std::floor(row0)));
  auto r_end = static_cast<std::size_t>(
      std::min(static_cast<double>(unit.h), std::ceil(row1) + 1.0));
  auto c_begin = static_cast<std::size_t>(std::max(0.0, std::floor(col0)));
  auto c_end = static_cast<std::size_t>(
      std::min(static_cast<double>(unit.w), std::ceil(col1) + 1.0));
  const bool large =
      r_end > r_begin && c_end > c_begin &&
      (r_end - r_begin) * (c_end - c_begin) * 2 > unit.w * unit.h;
  if (large) {
    r_begin = 0;
    r_end = unit.h;
    c_begin = 0;
    c_end = unit.w;
  } else {
    if (r_begin > 0) --r_begin;
    if (r_end < unit.h) ++r_end;
    if (c_begin > 0) --c_begin;
    if (c_end < unit.w) ++c_end;
  }
  fill_unit_sobel_rows(gray, unit,
                       static_cast<float>(min_magnitude * min_magnitude),
                       r_begin, r_end, r_end - r_begin == unit.h &&
                                           c_end - c_begin == unit.w,
                       c_begin, c_end);
}

void fill_float_gray_i16_rows(const GrayLevel& src, Grad& g,
                              std::size_t row_begin, std::size_t row_end,
                              std::size_t col_begin, std::size_t col_end) {
  if (!g.has_integer_components() || src.pixels.empty() || src.w != g.w ||
      src.h != g.h || g.w == 0 || g.h == 0)
    return;
  if (row_end > g.h) row_end = g.h;
  if (row_begin >= row_end) return;
  if (col_end > g.w) col_end = g.w;
  if (col_begin >= col_end) return;
  if (!clip_sobel_to_gray(src, row_begin, row_end, col_begin, col_end)) return;
  auto write_pixel = [&](std::size_t r, std::size_t c, std::size_t left,
                         std::size_t right) {
    const std::size_t previous_row = r == 0 ? 0 : r - 1;
    const std::size_t next_row = r + 1 < g.h ? r + 1 : g.h - 1;
    const float* top = src.pixels.data() + previous_row * g.w;
    const float* middle = src.pixels.data() + r * g.w;
    const float* bottom = src.pixels.data() + next_row * g.w;
    const float sx = (top[right] + 2.0F * middle[right] + bottom[right]) -
                     (top[left] + 2.0F * middle[left] + bottom[left]);
    const float sy = (bottom[left] + 2.0F * bottom[c] + bottom[right]) -
                     (top[left] + 2.0F * top[c] + top[right]);
    const auto index = r * g.w + c;
    g.gx_i16[index] = static_cast<std::int16_t>(std::lround(sx));
    g.gy_i16[index] = static_cast<std::int16_t>(std::lround(sy));
  };
  for (std::size_t r = row_begin; r < row_end; ++r) {
    const std::size_t previous_row = r == 0 ? 0 : r - 1;
    const std::size_t next_row = r + 1 < g.h ? r + 1 : g.h - 1;
    const float* top = src.pixels.data() + previous_row * g.w;
    const float* middle = src.pixels.data() + r * g.w;
    const float* bottom = src.pixels.data() + next_row * g.w;
    std::size_t c = col_begin;
    if (c == 0) {
      write_pixel(r, 0, 0, g.w > 1 ? 1 : 0);
      c = 1;
    }
    const std::size_t last = col_end;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (simd::cpu_has_avx2() && last > c + 8) {
      c = simd::avx2_kernels().sobel_f32_i16_row(top, middle, bottom,
                                       g.gx_i16.data() + r * g.w,
                                       g.gy_i16.data() + r * g.w, g.w, c, last);
    }
#endif
    for (; c + 1 < last && c + 1 < g.w; ++c)
      write_pixel(r, c, c - 1, c + 1);
    if (last == g.w) write_pixel(r, g.w - 1, g.w > 1 ? g.w - 2 : 0, g.w - 1);
    else if (c < last) write_pixel(r, c, c == 0 ? 0 : c - 1, std::min(c + 1, g.w - 1));
  }
}

void ensure_i16_from_gray(Grad& g, const GrayLevel& gray, double pyramid_scale,
                          double radius, const std::vector<MatchCandidate>& cand,
                          int max_deformation) {
  if (g.has_integer_components() || gray.pixels.empty() || cand.empty()) return;
  if (g.w != gray.w || g.h != gray.h || g.memory == nullptr) return;
  double row0 = static_cast<double>(g.h);
  double row1 = 0.0;
  double col0 = static_cast<double>(g.w);
  double col1 = 0.0;
  const double inv_scale = 1.0 / std::max(1.0, pyramid_scale);
  for (const auto& x : cand) {
    const double pad = std::abs(x.scale) * radius +
                       static_cast<double>(max_deformation) * inv_scale + 8.0;
    const double level_row = x.row * inv_scale;
    const double level_col = x.column * inv_scale;
    row0 = std::min(row0, level_row - pad);
    row1 = std::max(row1, level_row + pad);
    col0 = std::min(col0, level_col - pad);
    col1 = std::max(col1, level_col + pad);
  }
  auto r_begin = static_cast<std::size_t>(std::max(0.0, std::floor(row0)));
  auto r_end = static_cast<std::size_t>(
      std::min(static_cast<double>(g.h), std::ceil(row1) + 1.0));
  auto c_begin = static_cast<std::size_t>(std::max(0.0, std::floor(col0)));
  auto c_end = static_cast<std::size_t>(
      std::min(static_cast<double>(g.w), std::ceil(col1) + 1.0));
  const bool large =
      r_end > r_begin && c_end > c_begin &&
      (r_end - r_begin) * (c_end - c_begin) * 2 > g.w * g.h;
  if (large) {
    r_begin = 0;
    r_end = g.h;
    c_begin = 0;
    c_end = g.w;
  } else {
    if (r_begin > 0) --r_begin;
    if (r_end < g.h) ++r_end;
    if (c_begin > 0) --c_begin;
    if (c_end < g.w) ++c_end;
  }
  g.gx_i16 = PmrBuffer<std::int16_t>(g.w * g.h, g.memory);
  g.gy_i16 = PmrBuffer<std::int16_t>(g.w * g.h, g.memory);
  g.integer_scale = 0.25;
  fill_float_gray_i16_rows(gray, g, r_begin, r_end, c_begin, c_end);
}

std::pair<Grad, UnitGrad> gradient(const GrayLevel& src,
                                   double min_magnitude,
                                   bool keep_components,
                                   bool keep_unit,
                                   std::pmr::memory_resource* memory) {
  Grad g{src.w, src.h, memory};
  UnitGrad unit{src.w, src.h, memory, keep_unit};
  if (keep_components) {
    g.gx.assign(g.w * g.h, 0.0F);
    g.gy.assign(g.w * g.h, 0.0F);
  }
  const float min_squared = static_cast<float>(min_magnitude * min_magnitude);
  if (!keep_components && keep_unit) {
    if (src.limited) {
      std::fill_n(unit.nx.data(), unit.w * unit.h, 0.0F);
      std::fill_n(unit.ny.data(), unit.w * unit.h, 0.0F);
    }
    fill_unit_sobel_rows(src, unit, min_squared, 0, src.h, true);
    return {std::move(g), std::move(unit)};
  }
  auto process_rows = [&](mv::runtime::IndexRange rows) {
    for (std::size_t r = rows.begin; r < rows.end; ++r) {
    const std::size_t previous_row = r == 0 ? 0 : r - 1;
    const std::size_t next_row = r + 1 < g.h ? r + 1 : g.h - 1;
    const float* top = src.pixels.data() + previous_row * g.w;
    const float* middle = src.pixels.data() + r * g.w;
    const float* bottom = src.pixels.data() + next_row * g.w;
    auto write_gradient = [&](std::size_t c) {
      const std::size_t left = c == 0 ? 0 : c - 1;
      const std::size_t right = c + 1 < g.w ? c + 1 : g.w - 1;
      const float sx = (top[right] + 2.0F * middle[right] + bottom[right]) -
                       (top[left] + 2.0F * middle[left] + bottom[left]);
      const float sy = (bottom[left] + 2.0F * bottom[c] + bottom[right]) -
                       (top[left] + 2.0F * top[c] + top[right]);
      const auto index = r * g.w + c;
      const float gx = sx * 0.25F;
      const float gy = sy * 0.25F;
      if (keep_components) {
        g.gx[index] = gx;
        g.gy[index] = gy;
      }
      if (keep_unit) {
        const float squared = gx * gx + gy * gy;
        if (squared >= min_squared) {
          const float inverse = 1.0F / std::sqrt(squared);
          unit.nx[index] = gx * inverse;
          unit.ny[index] = gy * inverse;
        } else {
          unit.nx[index] = 0.0;
          unit.ny[index] = 0.0;
        }
      }
    };
    if (g.w == 1) {
      write_gradient(0);
      continue;
    }
    write_gradient(0);
    std::size_t c = 1;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (!keep_components && keep_unit && simd::cpu_has_avx2()) {
      c = simd::avx2_kernels().unit_sobel_f32_row(
          top, middle, bottom, unit.nx.data() + r * g.w,
          unit.ny.data() + r * g.w, g.w, min_squared);
    } else
#endif
#if MVKIT_SHAPE_HAS_SSE2
    if (!keep_components && keep_unit) {
      const __m128 quarter = _mm_set1_ps(0.25F);
      const __m128 one = _mm_set1_ps(1.0F);
      const __m128 threshold = _mm_set1_ps(min_squared);
      for (; c + 3 < g.w - 1; c += 4) {
        const __m128 top_left = _mm_loadu_ps(top + c - 1);
        const __m128 top_center = _mm_loadu_ps(top + c);
        const __m128 top_right = _mm_loadu_ps(top + c + 1);
        const __m128 middle_left = _mm_loadu_ps(middle + c - 1);
        const __m128 middle_right = _mm_loadu_ps(middle + c + 1);
        const __m128 bottom_left = _mm_loadu_ps(bottom + c - 1);
        const __m128 bottom_center = _mm_loadu_ps(bottom + c);
        const __m128 bottom_right = _mm_loadu_ps(bottom + c + 1);
        const __m128 sx = _mm_sub_ps(
            _mm_add_ps(_mm_add_ps(top_right, bottom_right),
                       _mm_add_ps(middle_right, middle_right)),
            _mm_add_ps(_mm_add_ps(top_left, bottom_left),
                       _mm_add_ps(middle_left, middle_left)));
        const __m128 sy = _mm_sub_ps(
            _mm_add_ps(_mm_add_ps(bottom_left, bottom_right),
                       _mm_add_ps(bottom_center, bottom_center)),
            _mm_add_ps(_mm_add_ps(top_left, top_right),
                       _mm_add_ps(top_center, top_center)));
        const __m128 gx = _mm_mul_ps(sx, quarter);
        const __m128 gy = _mm_mul_ps(sy, quarter);
        const __m128 squared = _mm_add_ps(_mm_mul_ps(gx, gx),
                                          _mm_mul_ps(gy, gy));
        const __m128 live = _mm_cmpge_ps(squared, threshold);
        const __m128 inverse = _mm_div_ps(one, _mm_sqrt_ps(squared));
        _mm_storeu_ps(unit.nx.data() + r * g.w + c,
                      _mm_and_ps(live, _mm_mul_ps(gx, inverse)));
        _mm_storeu_ps(unit.ny.data() + r * g.w + c,
                      _mm_and_ps(live, _mm_mul_ps(gy, inverse)));
      }
    }
#endif
    for (; c + 1 < g.w; ++c) write_gradient(c);
    write_gradient(g.w - 1);
    }
  };
  mv::imageproc::ExecutionContext::for_each_partition(
      g.h, g.w * g.h, 256u * 1024u, process_rows,
      uniform_row_partitions(g.w * g.h));
  return {std::move(g), std::move(unit)};
}

std::pair<Grad, UnitGrad> gradient_u8(
    const Image& image, double min_magnitude,
    bool keep_components,
    bool keep_unit,
    std::pmr::memory_resource* memory) {
  mv::imageproc::ImageView view{image};
  Grad g{image.width(), image.height(), memory};
  UnitGrad unit{image.width(), image.height(), memory, keep_unit};
  if (keep_components) {
    g.gx_i16 = PmrBuffer<std::int16_t>(g.w * g.h, memory);
    g.gy_i16 = PmrBuffer<std::int16_t>(g.w * g.h, memory);
    g.integer_scale = 0.25;
  }
  // UInt8 Sobel 响应在共同 1/4 缩放前为整数；内循环留在此域，避免逐像素重复缩放浮点乘积。
  const double min_raw_squared =
      16.0 * min_magnitude * min_magnitude;
  auto process_rows = [&](mv::runtime::IndexRange rows) {
    for (std::size_t r = rows.begin; r < rows.end; ++r) {
    const std::size_t previous_row = r == 0 ? 0 : r - 1;
    const std::size_t next_row = r + 1 < g.h ? r + 1 : g.h - 1;
    const auto* top = view.row_as<std::uint8_t>(previous_row);
    const auto* middle = view.row_as<std::uint8_t>(r);
    const auto* bottom = view.row_as<std::uint8_t>(next_row);
    auto write_gradient = [&](std::size_t c, std::size_t left,
                              std::size_t right) {
      const int sx =
          (top[right] + 2 * middle[right] + bottom[right]) -
          (top[left] + 2 * middle[left] + bottom[left]);
      const int sy =
          (bottom[left] + 2 * bottom[c] + bottom[right]) -
          (top[left] + 2 * top[c] + top[right]);
      const auto index = r * g.w + c;
      if (keep_components) {
        g.gx_i16[index] = static_cast<std::int16_t>(sx);
        g.gy_i16[index] = static_cast<std::int16_t>(sy);
      }
      if (keep_unit) {
        const int raw_squared = sx * sx + sy * sy;
        if (static_cast<double>(raw_squared) >= min_raw_squared) {
          const double inverse =
              1.0 / std::sqrt(static_cast<double>(raw_squared));
          unit.nx[index] = static_cast<float>(static_cast<double>(sx) * inverse);
          unit.ny[index] = static_cast<float>(static_cast<double>(sy) * inverse);
        } else {
          unit.nx[index] = 0.0;
          unit.ny[index] = 0.0;
        }
      }
    };
    if (g.w == 1) {
      write_gradient(0, 0, 0);
      continue;
    }
    write_gradient(0, 0, 1);
    std::size_t c = 1;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (keep_components && !keep_unit && simd::cpu_has_avx2()) {
      c = simd::avx2_kernels().sobel_u8_i16_row(
          top, middle, bottom, g.gx_i16.data() + r * g.w,
          g.gy_i16.data() + r * g.w, g.w);
    } else
#endif
#if MVKIT_SHAPE_HAS_SSE2
    // 最细 UInt8 层只需带符号 Sobel 分量，用 x64 SSE2 处理八相邻像素；
    // 归一化留给确实使用单位梯度的粗层。
    if (keep_components && !keep_unit) {
      const __m128i zero = _mm_setzero_si128();
      auto load8_u16 = [&](const std::uint8_t* values) {
        return _mm_unpacklo_epi8(
            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(values)), zero);
      };
      for (; c + 7 < g.w - 1; c += 8) {
        const __m128i top_left = load8_u16(top + c - 1);
        const __m128i top_center = load8_u16(top + c);
        const __m128i top_right = load8_u16(top + c + 1);
        const __m128i middle_left = load8_u16(middle + c - 1);
        const __m128i middle_right = load8_u16(middle + c + 1);
        const __m128i bottom_left = load8_u16(bottom + c - 1);
        const __m128i bottom_center = load8_u16(bottom + c);
        const __m128i bottom_right = load8_u16(bottom + c + 1);
        const __m128i sx_right = _mm_add_epi16(
            _mm_add_epi16(top_right, bottom_right),
            _mm_slli_epi16(middle_right, 1));
        const __m128i sx_left = _mm_add_epi16(
            _mm_add_epi16(top_left, bottom_left),
            _mm_slli_epi16(middle_left, 1));
        const __m128i sy_bottom = _mm_add_epi16(
            _mm_add_epi16(bottom_left, bottom_right),
            _mm_slli_epi16(bottom_center, 1));
        const __m128i sy_top = _mm_add_epi16(
            _mm_add_epi16(top_left, top_right),
            _mm_slli_epi16(top_center, 1));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(g.gx_i16.data() + r * g.w + c),
            _mm_sub_epi16(sx_right, sx_left));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(g.gy_i16.data() + r * g.w + c),
            _mm_sub_epi16(sy_bottom, sy_top));
      }
    }
#endif
    for (; c + 1 < g.w; ++c)
      write_gradient(c, c - 1, c + 1);
    write_gradient(g.w - 1, g.w - 2, g.w - 1);
    }
  };
  mv::imageproc::ExecutionContext::for_each_partition(
      g.h, g.w * g.h,
      mv::imageproc::ExecutionContext::kFilterParallelWork, process_rows,
      uniform_row_partitions(g.w * g.h));
  return {std::move(g), std::move(unit)};
}

void fill_u8_sobel_component_rows(const Image& image, Grad& g,
                                  std::size_t row_begin, std::size_t row_end,
                                  std::size_t col_begin = 0,
                                  std::size_t col_end = static_cast<std::size_t>(-1)) {
  if (image.pixel_type() != mv::core::PixelType::UInt8 || g.w == 0 ||
      g.h == 0 || row_begin >= g.h)
    return;
  if (row_end > g.h) row_end = g.h;
  if (row_begin >= row_end) return;
  if (col_end > g.w) col_end = g.w;
  if (col_begin >= col_end) return;
  if (!g.has_integer_components()) {
    g.gx_i16 = PmrBuffer<std::int16_t>(g.w * g.h, g.memory);
    g.gy_i16 = PmrBuffer<std::int16_t>(g.w * g.h, g.memory);
    g.integer_scale = 0.25;
  }
  mv::imageproc::ImageView view{image};
  for (std::size_t r = row_begin; r < row_end; ++r) {
    const std::size_t previous_row = r == 0 ? 0 : r - 1;
    const std::size_t next_row = r + 1 < g.h ? r + 1 : g.h - 1;
    const auto* top = view.row_as<std::uint8_t>(previous_row);
    const auto* middle = view.row_as<std::uint8_t>(r);
    const auto* bottom = view.row_as<std::uint8_t>(next_row);
    auto write_gradient = [&](std::size_t c, std::size_t left,
                              std::size_t right) {
      const int sx =
          (top[right] + 2 * middle[right] + bottom[right]) -
          (top[left] + 2 * middle[left] + bottom[left]);
      const int sy =
          (bottom[left] + 2 * bottom[c] + bottom[right]) -
          (top[left] + 2 * top[c] + top[right]);
      const auto index = r * g.w + c;
      g.gx_i16[index] = static_cast<std::int16_t>(sx);
      g.gy_i16[index] = static_cast<std::int16_t>(sy);
    };
    if (g.w == 1) {
      write_gradient(0, 0, 0);
      continue;
    }
    std::size_t c = col_begin;
    if (c == 0) {
      write_gradient(0, 0, 1);
      c = 1;
    }
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (simd::cpu_has_avx2()) {
      // 内部请求区间两侧各提供一个真实邻居，行核只写 SIMD 内部；边界及短尾仍由原标量代码处理。
      const auto offset = c - 1;
      const auto end = std::min(col_end, g.w - 1);
      if (c < end)
        c = offset + simd::avx2_kernels().sobel_u8_i16_row(
            top + offset, middle + offset, bottom + offset,
            g.gx_i16.data() + r * g.w + offset,
            g.gy_i16.data() + r * g.w + offset, end - offset + 1);
    } else
#endif
#if MVKIT_SHAPE_HAS_SSE2
    {
      const __m128i zero = _mm_setzero_si128();
      auto load8_u16 = [&](const std::uint8_t* values) {
        return _mm_unpacklo_epi8(
            _mm_loadl_epi64(reinterpret_cast<const __m128i*>(values)), zero);
      };
      const std::size_t simd_end = col_end < g.w ? col_end : g.w - 1;
      for (; c + 7 < simd_end; c += 8) {
        const __m128i top_left = load8_u16(top + c - 1);
        const __m128i top_center = load8_u16(top + c);
        const __m128i top_right = load8_u16(top + c + 1);
        const __m128i middle_left = load8_u16(middle + c - 1);
        const __m128i middle_right = load8_u16(middle + c + 1);
        const __m128i bottom_left = load8_u16(bottom + c - 1);
        const __m128i bottom_center = load8_u16(bottom + c);
        const __m128i bottom_right = load8_u16(bottom + c + 1);
        const __m128i sx_right = _mm_add_epi16(
            _mm_add_epi16(top_right, bottom_right),
            _mm_slli_epi16(middle_right, 1));
        const __m128i sx_left = _mm_add_epi16(
            _mm_add_epi16(top_left, bottom_left),
            _mm_slli_epi16(middle_left, 1));
        const __m128i sy_bottom = _mm_add_epi16(
            _mm_add_epi16(bottom_left, bottom_right),
            _mm_slli_epi16(bottom_center, 1));
        const __m128i sy_top = _mm_add_epi16(
            _mm_add_epi16(top_left, top_right),
            _mm_slli_epi16(top_center, 1));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(g.gx_i16.data() + r * g.w + c),
            _mm_sub_epi16(sx_right, sx_left));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(g.gy_i16.data() + r * g.w + c),
            _mm_sub_epi16(sy_bottom, sy_top));
      }
    }
#endif
    for (; c + 1 < col_end && c + 1 < g.w; ++c)
      write_gradient(c, c - 1, c + 1);
    if (col_end == g.w) write_gradient(g.w - 1, g.w - 2, g.w - 1);
    else if (c < col_end) write_gradient(c, c - 1, std::min(c + 1, g.w - 1));
  }
}

void ensure_finest_u8_components(const Image& image, Grad& g,
                                 const ShapeModel& model,
                                 const std::vector<MatchCandidate>& cand,
                                 int max_deformation) {
  if (g.has_integer_components() ||
      image.pixel_type() != mv::core::PixelType::UInt8 ||
      g.w != image.width() || g.h != image.height())
    return;
  if (cand.empty()) return;
  const double radius = model.levels().front().radius;
  double row0 = static_cast<double>(g.h);
  double row1 = 0.0;
  double col0 = static_cast<double>(g.w);
  double col1 = 0.0;
  for (const auto& x : cand) {
    const double pad =
        std::abs(x.scale) * radius + static_cast<double>(max_deformation) + 12.0;
    row0 = std::min(row0, x.row - pad);
    row1 = std::max(row1, x.row + pad);
    col0 = std::min(col0, x.column - pad);
    col1 = std::max(col1, x.column + pad);
  }
  auto r_begin = static_cast<std::size_t>(std::max(0.0, std::floor(row0)));
  auto r_end = static_cast<std::size_t>(
      std::min(static_cast<double>(g.h), std::ceil(row1) + 1.0));
  auto c_begin = static_cast<std::size_t>(std::max(0.0, std::floor(col0)));
  auto c_end = static_cast<std::size_t>(
      std::min(static_cast<double>(g.w), std::ceil(col1) + 1.0));
  const bool large =
      r_end > r_begin && c_end > c_begin &&
      (r_end - r_begin) * (c_end - c_begin) * 2 > g.w * g.h;
  if (large) {
    r_begin = 0;
    r_end = g.h;
    c_begin = 0;
    c_end = g.w;
  } else {
    if (r_begin > 0) --r_begin;
    if (r_end < g.h) ++r_end;
    if (c_begin > 0) --c_begin;
    if (c_end < g.w) ++c_end;
  }
  fill_u8_sobel_component_rows(image, g, r_begin, r_end, c_begin, c_end);
}

GrayLevel downsample_u8(const Image& image,
                        std::pmr::memory_resource* memory, const PyramidRect* region = nullptr,
                        const TimeoutWatch* timeout = nullptr) {
  mv::imageproc::ImageView view{image};
  GrayLevel dst{std::max<std::size_t>(1, image.width() / 2),
                std::max<std::size_t>(1, image.height() / 2), memory};
  dst.pixels = PmrFloatBuffer(dst.w * dst.h, dst.memory);
  const PyramidRect bounds = region ? *region : PyramidRect{0, dst.h, 0, dst.w};
  dst.limited = region != nullptr; dst.valid = bounds;
  auto process_rows = [&](mv::runtime::IndexRange rows) {
    for (std::size_t r = rows.begin; r < rows.end; ++r) {
    if (timeout && (r & 31) == 0) timeout->poll();
    const std::size_t r0 = std::min(image.height() - 1, 2 * r);
    const std::size_t r1 = std::min(image.height() - 1, r0 + 1);
    const auto* row0 = view.row_as<std::uint8_t>(r0);
    const auto* row1 = view.row_as<std::uint8_t>(r1);
    float* __restrict out = dst.pixels.data() + r * dst.w;
    std::size_t c = bounds.c0;
#if MVKIT_SHAPE_HAS_SSE2
    // 16 个相邻 u8 扩成 u16 后按 2x2 块相加，得到 8 个平均亮度。
    const __m128i zero = _mm_setzero_si128();
    const __m128i ones = _mm_set1_epi16(1);
    const __m128 quarter = _mm_set1_ps(0.25F);
    for (; c + 8 <= bounds.c1 && 2 * c + 15 < image.width(); c += 8) {
      const std::size_t c0 = 2 * c;
      const __m128i top16 = _mm_loadu_si128(
          reinterpret_cast<const __m128i*>(row0 + c0));
      const __m128i bot16 = _mm_loadu_si128(
          reinterpret_cast<const __m128i*>(row1 + c0));
      const __m128i top_lo = _mm_unpacklo_epi8(top16, zero);
      const __m128i top_hi = _mm_unpackhi_epi8(top16, zero);
      const __m128i bot_lo = _mm_unpacklo_epi8(bot16, zero);
      const __m128i bot_hi = _mm_unpackhi_epi8(bot16, zero);
      const __m128i sum_lo = _mm_madd_epi16(_mm_add_epi16(top_lo, bot_lo), ones);
      const __m128i sum_hi = _mm_madd_epi16(_mm_add_epi16(top_hi, bot_hi), ones);
      _mm_storeu_ps(out + c, _mm_mul_ps(_mm_cvtepi32_ps(sum_lo), quarter));
      _mm_storeu_ps(out + c + 4, _mm_mul_ps(_mm_cvtepi32_ps(sum_hi), quarter));
    }
#endif
    for (; c < bounds.c1; ++c) {
      const std::size_t c0 = std::min(image.width() - 1, 2 * c);
      const std::size_t c1 = std::min(image.width() - 1, c0 + 1);
      out[c] =
          0.25F * static_cast<float>(row0[c0] + row0[c1] + row1[c0] + row1[c1]);
    }
    }
  };
  if (region) process_rows({bounds.r0, bounds.r1});
  else mv::imageproc::ExecutionContext::for_each_partition(
      dst.h, image.width() * image.height(),
      mv::imageproc::ExecutionContext::kFilterParallelWork, process_rows,
      uniform_row_partitions(image.width() * image.height()));
  return dst;
}

GradientPyramid gradient_pyramid(const Image& image, const ShapeModel& model,
                                 std::size_t levels, std::size_t finest,
                                 std::pmr::memory_resource* memory,
                                 bool use_roi = false, const TimeoutWatch* timeout = nullptr) {
  GradientPyramid out(memory);
  out.gradients.reserve(levels);
  out.unit_gradients.reserve(levels);
  out.gray_levels.reserve(levels);
  const bool direct_u8 = image.pixel_type() == mv::core::PixelType::UInt8;
  if (use_roi && direct_u8 && finest == 0 && levels > 1) {
    double r0 = static_cast<double>(image.height()), r1 = -1;
    double c0 = static_cast<double>(image.width()), c1 = -1;
    for (const auto& run : image.domain().runs()) {
      if (run.row < 0 || static_cast<std::size_t>(run.row) >= image.height()) continue;
      r0 = std::min(r0, static_cast<double>(run.row));
      r1 = std::max(r1, static_cast<double>(run.row));
      c0 = std::min(c0, static_cast<double>(run.col_begin));
      c1 = std::max(c1, static_cast<double>(run.col_end));
    }
    if (r0 <= r1 && c0 <= c1) {
      out.regions.resize(levels);
      std::size_t w = image.width(), h = image.height();
      std::pmr::vector<std::pair<std::size_t, std::size_t>> sizes(memory);
      for (std::size_t i = 0; i < levels; ++i) {
        sizes.emplace_back(w, h);
        const double scale = std::ldexp(1.0, static_cast<int>(i));
        // 刚性候选在各细化层保持域内；回传边界前包含旋转不变模型半径、
        // 既有延迟梯度 padding(8)、舍入及 Sobel 支撑。
        const double pad = model.levels()[i].radius + 12.0;
        out.regions[i] = {
            static_cast<std::size_t>(std::clamp(std::floor(r0 / scale - pad), 0.0, static_cast<double>(h))),
            static_cast<std::size_t>(std::clamp(std::ceil(r1 / scale + pad) + 1, 0.0, static_cast<double>(h))),
            static_cast<std::size_t>(std::clamp(std::floor(c0 / scale - pad), 0.0, static_cast<double>(w))),
            static_cast<std::size_t>(std::clamp(std::ceil(c1 / scale + pad) + 1, 0.0, static_cast<double>(w)))};
        w = std::max<std::size_t>(1, w / 2); h = std::max<std::size_t>(1, h / 2);
      }
      for (std::size_t i = levels - 1; i > 1; --i) {
        const auto& next = out.regions[i];
        auto& prev = out.regions[i - 1];
        prev.r0 = std::min(prev.r0, 2 * next.r0); prev.c0 = std::min(prev.c0, 2 * next.c0);
        prev.r1 = std::max(prev.r1, std::min(sizes[i-1].second, 2 * next.r1));
        prev.c1 = std::max(prev.c1, std::min(sizes[i-1].first, 2 * next.c1));
      }
      const auto& first = out.regions[1];
      if (static_cast<double>((first.r1-first.r0)*(first.c1-first.c0)) >
          0.65 * sizes[1].first * sizes[1].second) out.regions.clear();
    }
  }
  auto region = [&](std::size_t i) -> const PyramidRect* {
    return out.regions.empty() ? nullptr : &out.regions[i];
  };
  GrayLevel gray = direct_u8
                       ? (levels > 1 ? downsample_u8(image, memory, region(1), timeout)
                                     : GrayLevel{0, 0, memory})
                       : to_gray(image, memory);
  for (std::size_t i = 0; i < levels; ++i) {
    if (timeout) timeout->poll();
    const double min_magnitude =
        model.min_contrast() /
        std::max(1.0, model.levels()[i].pyramid_scale);
    const bool keep_components = i == finest;
    const bool keep_unit = i != finest || finest + 1 == levels;
    const bool coarsest_level = i + 1 == levels;
    const bool defer_unit = keep_unit && !keep_components && !coarsest_level &&
                            !(finest + 1 == levels);
    if (direct_u8 && i == 0 && keep_components && !keep_unit) {
      out.gradients.emplace_back(image.width(), image.height(), memory);
      out.unit_gradients.emplace_back(image.width(), image.height(), memory,
                                      false);
      out.gray_levels.emplace_back(0, 0, memory);
      continue;
    }
    if (defer_unit) {
      // 中间层单位梯度只在金字塔精化碰到候选时按行带补算。
      out.gradients.emplace_back(gray.w, gray.h, memory);
      out.unit_gradients.emplace_back(gray.w, gray.h, memory, false);
      out.gray_levels.push_back(std::move(gray));
      if (i + 1 < levels)
        gray = downsample(out.gray_levels.back(), memory, region(i + 1), timeout);
      continue;
    }
    auto level = direct_u8 && i == 0
                     ? gradient_u8(image, min_magnitude, keep_components,
                                   keep_unit,
                                   memory)
                     : gradient(gray, min_magnitude, keep_components,
                                keep_unit, memory);
    out.gradients.push_back(std::move(level.first));
    out.unit_gradients.push_back(std::move(level.second));
    out.gray_levels.emplace_back(0, 0, memory);
    if (i + 1 < levels && !(direct_u8 && i == 0))
      gray = downsample(gray, memory, region(i + 1), timeout);
  }
  return out;
}

bool sample_components(const Grad& g, double r, double c, double* gx, double* gy, bool bilinear = true) {
  if (r < 0 || c < 0 || r > static_cast<double>(g.h - 1) || c > static_cast<double>(g.w - 1)) return false;
  if (!bilinear) {
    const auto rr = static_cast<std::size_t>(r + 0.5);
    const auto cc = static_cast<std::size_t>(c + 0.5);
    const auto idx = rr * g.w + cc;
    if (g.has_integer_components()) {
      *gx = g.integer_scale * static_cast<double>(g.gx_i16[idx]);
      *gy = g.integer_scale * static_cast<double>(g.gy_i16[idx]);
    } else {
      *gx = g.gx[idx];
      *gy = g.gy[idx];
    }
    return true;
  }
  // 上方边界保证坐标非负，整数转换等价 floor，避免最内匹配循环两次标量取整。
  const auto r0 = static_cast<std::size_t>(r);
  const auto c0 = static_cast<std::size_t>(c);
  const auto r1 = std::min(r0 + 1, g.h - 1);
  const auto c1 = std::min(c0 + 1, g.w - 1);
  const double fr = r - static_cast<double>(r0);
  const double fc = c - static_cast<double>(c0);
  const double one_minus_fr = 1.0 - fr;
  const double one_minus_fc = 1.0 - fc;
  const auto row0 = r0 * g.w;
  const auto row1 = r1 * g.w;
  auto interp = [&](const auto& v) {
    const double top = one_minus_fc * v[row0 + c0] + fc * v[row0 + c1];
    const double bottom = one_minus_fc * v[row1 + c0] + fc * v[row1 + c1];
    return one_minus_fr * top + fr * bottom;
  };
  if (g.has_integer_components()) {
    *gx = g.integer_scale * interp(g.gx_i16);
    *gy = g.integer_scale * interp(g.gy_i16);
  } else {
    *gx = interp(g.gx);
    *gy = interp(g.gy);
  }
  return true;
}

void sample_components_interior(const Grad& g, double r, double c, double* gx, double* gy,
                                bool bilinear) noexcept {
  if (!bilinear) {
    const auto rr = static_cast<std::size_t>(r + 0.5);
    const auto cc = static_cast<std::size_t>(c + 0.5);
    const auto idx = rr * g.w + cc;
    if (g.has_integer_components()) {
      *gx = g.integer_scale * static_cast<double>(g.gx_i16[idx]);
      *gy = g.integer_scale * static_cast<double>(g.gy_i16[idx]);
    } else {
      *gx = g.gx[idx];
      *gy = g.gy[idx];
    }
    return;
  }
  const auto r0 = static_cast<std::size_t>(r);
  const auto c0 = static_cast<std::size_t>(c);
  const auto r1 = r0 + 1;
  const auto c1 = c0 + 1;
  const double fr = r - static_cast<double>(r0);
  const double fc = c - static_cast<double>(c0);
  const double one_minus_fr = 1.0 - fr;
  const double one_minus_fc = 1.0 - fc;
  const auto row0 = r0 * g.w;
  const auto row1 = r1 * g.w;
  auto interp = [&](const auto& values) {
    const double top = one_minus_fc * values[row0 + c0] + fc * values[row0 + c1];
    const double bottom = one_minus_fc * values[row1 + c0] + fc * values[row1 + c1];
    return one_minus_fr * top + fr * bottom;
  };
  if (g.has_integer_components()) {
    *gx = g.integer_scale * interp(g.gx_i16);
    *gy = g.integer_scale * interp(g.gy_i16);
  } else {
    *gx = interp(g.gx);
    *gy = interp(g.gy);
  }
}

bool sample_unit_components(const UnitGrad& g, double r, double c,
                            double* nx, double* ny) noexcept {
  if (r < 0 || c < 0 || r > static_cast<double>(g.h - 1) ||
      c > static_cast<double>(g.w - 1))
    return false;
  const auto rr = static_cast<std::size_t>(r + 0.5);
  const auto cc = static_cast<std::size_t>(c + 0.5);
  const auto index = rr * g.w + cc;
  *nx = g.nx[index];
  *ny = g.ny[index];
  return true;
}

void sample_unit_components_interior(const UnitGrad& g, double r, double c,
                                     double* nx, double* ny) noexcept {
  const auto rr = static_cast<std::size_t>(r + 0.5);
  const auto cc = static_cast<std::size_t>(c + 0.5);
  const auto index = rr * g.w + cc;
  *nx = g.nx[index];
  *ny = g.ny[index];
}

void sample_unit_components_interior(const UnitGrad& g, float r, float c,
                                     float* nx, float* ny) noexcept {
  const auto rr = static_cast<std::size_t>(r + 0.5F);
  const auto cc = static_cast<std::size_t>(c + 0.5F);
  const auto index = rr * g.w + cc;
  *nx = g.nx[index];
  *ny = g.ny[index];
}

bool sample_unit_components(const UnitGrad& g, float r, float c,
                            float* nx, float* ny) noexcept {
  if (r < 0.0F || c < 0.0F || r > static_cast<float>(g.h - 1) ||
      c > static_cast<float>(g.w - 1))
    return false;
  sample_unit_components_interior(g, r, c, nx, ny);
  return true;
}

#if MVKIT_SHAPE_HAS_SSE2
// 相邻候选中心在单位梯度图上是连续 float，用未对齐 load 代替 setr 插入。
inline __m128 load2_unit_ps(const float* __restrict values) {
  return _mm_castpd_ps(
      _mm_loadu_pd(reinterpret_cast<const double*>(values)));
}
#endif

int live_lane_count(int mask, int lanes) noexcept {
  int count = 0;
  for (int lane = 0; lane < lanes; ++lane)
    count += (mask >> lane) & 1;
  return count;
}

// 高分搜索可要求两个显著锚点；遮挡/小模型仍只需一次命中，保持 GreedySearchFindsInstance 行为。
int min_anchor_hits_for_search(double min_score, std::size_t anchor_count,
                               std::size_t point_count) noexcept {
  return (min_score >= 0.5 && anchor_count >= 4 && point_count >= 16) ? 2 : 1;
}

constexpr int kUnitQuant = 127;
constexpr int kUnitQuantSq = kUnitQuant * kUnitQuant;
constexpr int kEvidenceDotThr = 15322;  // 0.95 * 127 * 127

std::int8_t quantize_unit_normal(float value) noexcept {
  const int quantized =
      static_cast<int>(std::lround(static_cast<double>(value) * kUnitQuant));
  if (quantized > kUnitQuant) return static_cast<std::int8_t>(kUnitQuant);
  if (quantized < -kUnitQuant) return static_cast<std::int8_t>(-kUnitQuant);
  return static_cast<std::int8_t>(quantized);
}

void quantize_unit_maps(const float* __restrict nx, const float* __restrict ny,
                        std::int8_t* __restrict qnx, std::int8_t* __restrict qny,
                        std::size_t count) noexcept {
  std::size_t i = 0;
#if MVKIT_SHAPE_HAS_SSE2
  // lround(x) 半值远离 0：先乘 127，再按符号加 ±0.5 后截断。
  const __m128 scale = _mm_set1_ps(static_cast<float>(kUnitQuant));
  const __m128 half = _mm_set1_ps(0.5F);
  const __m128 sign_bit = _mm_set1_ps(-0.0F);
  auto quant16 = [&](const float* __restrict src, std::int8_t* __restrict dst) {
    auto quant4 = [&](const float* __restrict values) {
      const __m128 mag = _mm_mul_ps(_mm_loadu_ps(values), scale);
      const __m128 bias = _mm_or_ps(half, _mm_and_ps(mag, sign_bit));
      return _mm_cvttps_epi32(_mm_add_ps(mag, bias));
    };
    const __m128i packed = _mm_packs_epi16(
        _mm_packs_epi32(quant4(src), quant4(src + 4)),
        _mm_packs_epi32(quant4(src + 8), quant4(src + 12)));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), packed);
  };
  for (; i + 16 <= count; i += 16) {
    quant16(nx + i, qnx + i);
    quant16(ny + i, qny + i);
  }
#endif
  for (; i < count; ++i) {
    qnx[i] = quantize_unit_normal(nx[i]);
    qny[i] = quantize_unit_normal(ny[i]);
  }
}

int i8_dot(std::int8_t ny, std::int8_t nx, std::int8_t n_row, std::int8_t n_col,
           bool use_polarity) noexcept {
  int dot = static_cast<int>(ny) * n_row + static_cast<int>(nx) * n_col;
  if (!use_polarity && dot < 0) dot = -dot;
  return dot;
}

template <int kLanes>
int i8_group_evidence_mask(const std::int8_t* __restrict qnx,
                           const std::int8_t* __restrict qny,
                           std::span<const DiscreteShapePoint> anchors,
                           std::size_t first_center_index, int min_hits,
                           int threshold, bool use_polarity) noexcept {
  const int need = min_hits < 1 ? 1 : min_hits;
  const auto base = static_cast<std::ptrdiff_t>(first_center_index);
  const int n_anchors = static_cast<int>(anchors.size());
  int mask = 0;
  for (int lane = 0; lane < kLanes; ++lane) {
    int hits = 0;
    for (int a = 0; a < n_anchors; ++a) {
      const auto& anchor = anchors[static_cast<std::size_t>(a)];
      const auto index =
          static_cast<std::size_t>(base + anchor.image_offset + lane);
      if (i8_dot(qny[index], qnx[index], anchor.qn_row, anchor.qn_col,
                 use_polarity) > threshold)
        ++hits;
      if (hits >= need) {
        mask |= 1 << lane;
        break;
      }
      if (hits + (n_anchors - a - 1) < need) break;
    }
  }
  return mask;
}

int i8_group_evidence_mask_offsets(const std::int8_t* __restrict qnx,
                                   const std::int8_t* __restrict qny,
                                   const std::ptrdiff_t* __restrict offsets,
                                   const std::int8_t* __restrict n_row,
                                   const std::int8_t* __restrict n_col,
                                   std::size_t anchor_count,
                                   std::size_t first_center_index, int lanes,
                                   int min_hits, int threshold,
                                   bool use_polarity) noexcept {
  const int need = min_hits < 1 ? 1 : min_hits;
  const auto base = static_cast<std::ptrdiff_t>(first_center_index);
  const int n_anchors = static_cast<int>(anchor_count);
  int mask = 0;
  for (int lane = 0; lane < lanes; ++lane) {
    int hits = 0;
    for (int a = 0; a < n_anchors; ++a) {
      const auto index = static_cast<std::size_t>(
          base + offsets[static_cast<std::size_t>(a)] + lane);
      if (i8_dot(qny[index], qnx[index], n_row[a], n_col[a], use_polarity) >
          threshold)
        ++hits;
      if (hits >= need) {
        mask |= 1 << lane;
        break;
      }
      if (hits + (n_anchors - a - 1) < need) break;
    }
  }
  return mask;
}

double score_discrete_unit_points(const UnitGrad& g, const ShapeModel& model,
                                  std::span<const DiscreteShapePoint> points,
                                  std::size_t center_index, double greediness,
                                  double reject_below) {
  const double n = static_cast<double>(points.size());
  if (n <= 0) return 0;
  double pos = 0;
  double neg = 0;
  const bool ignore_local_polarity =
      model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity =
      model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const auto base = static_cast<std::ptrdiff_t>(center_index);
  const auto accumulate = [&](const DiscreteShapePoint& point) {
    const auto index = static_cast<std::size_t>(base + point.image_offset);
    const double dot = point.normal_row * g.ny[index] +
                       point.normal_column * g.nx[index];
    if (ignore_local_polarity) {
      pos += std::abs(dot);
    } else {
      pos += std::max(0.0, dot);
      if (ignore_global_polarity) neg += std::max(0.0, -dot);
    }
  };
  const auto current_score = [&] {
    return ignore_global_polarity ? std::max(pos, neg) : pos;
  };
  // 初始证据窗使用保守 remaining=1 界。
  const std::size_t greedy_begin =
      greediness > 0 && reject_below > 0
          ? static_cast<std::size_t>(greedy_prefix(n,greediness,reject_below))
          : points.size();
  const std::size_t prefix = std::min(greedy_begin, points.size());
  std::size_t done = 0;
  for (; done < prefix; ++done) {
    accumulate(points[done]);
    if (reject_below > 0 &&
        current_score() + (n - static_cast<double>(done + 1)) < reject_below * n)
      return 0;
  }
  for (; done < points.size(); ++done) {
    accumulate(points[done]);
    if (reject_below <= 0) continue;
    const double remaining = n - static_cast<double>(done + 1);
    const double current = current_score();
    if (current + remaining < reject_below * n) return 0;
    if (current + remaining * (1.0 - greediness) < reject_below * n) return 0;
  }
  const double raw = current_score() / n;
  return std::clamp(raw, 0.0, 1.0);
}

template <int kLanes>
void score_discrete_unit_group(const UnitGrad& g, const ShapeModel& model,
                               std::span<const DiscreteShapePoint> points,
                               std::size_t first_center_index, int live_mask,
                               double greediness, double reject_below,
                               double* scores) {
  constexpr int kAllLanes = (1 << kLanes) - 1;
  live_mask &= kAllLanes;
  for (int lane = 0; lane < kLanes; ++lane) scores[lane] = 0;
  if (live_mask == 0 || points.empty()) return;
#if MVKIT_SHAPE_HAS_SSE2
  const float n = static_cast<float>(points.size());
  const bool ignore_local_polarity =
      model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity =
      model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const __m128 zero = _mm_setzero_ps();
  const __m128 sign = _mm_set1_ps(-0.0F);
  __m128 pos = zero;
  __m128 neg = zero;
  int rejected_mask = kAllLanes & ~live_mask;
  float done = 0;
  const float reject_total = static_cast<float>(reject_below) * n;
  const float greedy_remaining_factor = 1.0F - static_cast<float>(greediness);
  const auto base = static_cast<std::ptrdiff_t>(first_center_index);
  const auto accumulate = [&](const DiscreteShapePoint& point) {
    const auto index = static_cast<std::size_t>(base + point.image_offset);
    // 连续中心的梯度是 SoA 行内相邻元素，kLanes=4 时一次 loadu 填满 XMM。
    const float* __restrict nx_ptr = g.nx.data() + index;
    const float* __restrict ny_ptr = g.ny.data() + index;
    const __m128 nx =
        kLanes >= 4 ? _mm_loadu_ps(nx_ptr) : load2_unit_ps(nx_ptr);
    const __m128 ny =
        kLanes >= 4 ? _mm_loadu_ps(ny_ptr) : load2_unit_ps(ny_ptr);
    const __m128 dot = _mm_add_ps(
        _mm_mul_ps(ny, _mm_set1_ps(point.normal_row)),
        _mm_mul_ps(nx, _mm_set1_ps(point.normal_column)));
    if (ignore_local_polarity) {
      pos = _mm_add_ps(pos, _mm_andnot_ps(sign, dot));
    } else {
      pos = _mm_add_ps(pos, _mm_max_ps(zero, dot));
      if (ignore_global_polarity)
        neg = _mm_add_ps(neg, _mm_max_ps(zero, _mm_sub_ps(zero, dot)));
    }
  };
  const auto reject_ordinary = [&](float remaining) {
    const __m128 current =
        ignore_global_polarity ? _mm_max_ps(pos, neg) : pos;
    const __m128 reject_limit = _mm_set1_ps(reject_total);
    rejected_mask |=
        _mm_movemask_ps(_mm_cmplt_ps(
            _mm_add_ps(current, _mm_set1_ps(remaining)), reject_limit)) &
        kAllLanes;
  };
  const auto reject_greedy = [&](float remaining) {
    const __m128 current =
        ignore_global_polarity ? _mm_max_ps(pos, neg) : pos;
    const __m128 reject_limit = _mm_set1_ps(reject_total);
    rejected_mask |=
        _mm_movemask_ps(_mm_cmplt_ps(
            _mm_add_ps(current, _mm_set1_ps(remaining * greedy_remaining_factor)),
            reject_limit)) &
        kAllLanes;
  };
  const std::size_t greedy_index =
      greediness > 0 && reject_below > 0
          ? static_cast<std::size_t>(greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below)))
          : points.size();
  const std::size_t prefix = std::min(greedy_index, points.size());
  std::size_t i = 0;
  for (; i < prefix; ++i) {
    accumulate(points[i]);
    done += 1.0F;
    if (reject_below > 0) {
      reject_ordinary(n - done);
      if ((rejected_mask & live_mask) == live_mask) return;
    }
  }
  for (; i < points.size(); ++i) {
    accumulate(points[i]);
    done += 1.0F;
    if (reject_below <= 0) continue;
    const float remaining = n - done;
    reject_ordinary(remaining);
    reject_greedy(remaining);
    if ((rejected_mask & live_mask) == live_mask) return;
  }
  alignas(16) float pos_values[4];
  alignas(16) float neg_values[4];
  _mm_store_ps(pos_values, pos);
  _mm_store_ps(neg_values, neg);
  for (int lane = 0; lane < kLanes; ++lane) {
    if ((live_mask & (1 << lane)) == 0) continue;
    if ((rejected_mask & (1 << lane)) != 0) continue;
    const double raw =
        (ignore_global_polarity ? std::max(pos_values[lane], neg_values[lane])
                                : pos_values[lane]) /
        n;
    scores[lane] = std::clamp(raw, 0.0, 1.0);
  }
#else
  for (int lane = 0; lane < kLanes; ++lane) {
    if ((live_mask & (1 << lane)) == 0) continue;
    scores[lane] = score_discrete_unit_points(
        g, model, points, first_center_index + static_cast<std::size_t>(lane),
        greediness, reject_below);
  }
#endif
}

#if MVKIT_SHAPE_HAS_SSE2
inline __m128i load_i8_as_i16x4(const std::int8_t* __restrict values,
                                int lanes) noexcept {
  if (lanes >= 4) {
    int packed = 0;
    std::memcpy(&packed, values, 4);
    const __m128i bytes = _mm_cvtsi32_si128(packed);
    return _mm_srai_epi16(_mm_unpacklo_epi8(bytes, bytes), 8);
  }
  std::int16_t packed = 0;
  std::memcpy(&packed, values, 2);
  const __m128i bytes = _mm_cvtsi32_si128(static_cast<int>(packed) & 0xFFFF);
  return _mm_srai_epi16(_mm_unpacklo_epi8(bytes, bytes), 8);
}

inline __m128i abs_epi32_sse2(__m128i value) noexcept {
  const __m128i mask = _mm_srai_epi32(value, 31);
  return _mm_sub_epi32(_mm_xor_si128(value, mask), mask);
}
#endif

double score_discrete_i8_points(const std::int8_t* __restrict qnx,
                                const std::int8_t* __restrict qny,
                                const ShapeModel& model,
                                std::span<const DiscreteShapePoint> points,
                                std::size_t center_index, double greediness,
                                double reject_below) {
  const double n = static_cast<double>(points.size());
  if (n <= 0) return 0;
  int pos = 0;
  int neg = 0;
  const bool ignore_local_polarity =
      model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity =
      model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const auto base = static_cast<std::ptrdiff_t>(center_index);
  const int reject_total =
      reject_below > 0
          ? static_cast<int>(reject_below * n * kUnitQuantSq)
          : 0;
  const auto accumulate = [&](const DiscreteShapePoint& point) {
    const auto index = static_cast<std::size_t>(base + point.image_offset);
    const int dot = static_cast<int>(qny[index]) * point.qn_row +
                    static_cast<int>(qnx[index]) * point.qn_col;
    if (ignore_local_polarity) {
      pos += dot < 0 ? -dot : dot;
    } else {
      if (dot > 0) pos += dot;
      if (ignore_global_polarity && dot < 0) neg -= dot;
    }
  };
  const auto current_score = [&] {
    return ignore_global_polarity ? (pos > neg ? pos : neg) : pos;
  };
  const std::size_t greedy_begin =
      greediness > 0 && reject_below > 0
          ? static_cast<std::size_t>(greedy_prefix(n,greediness,reject_below))
          : points.size();
  const std::size_t prefix = std::min(greedy_begin, points.size());
  std::size_t done = 0;
  for (; done < prefix; ++done) {
    accumulate(points[done]);
    if (reject_below > 0 &&
        current_score() +
                static_cast<int>(n - static_cast<double>(done + 1)) *
                    kUnitQuantSq <
            reject_total)
      return 0;
  }
  for (; done < points.size(); ++done) {
    accumulate(points[done]);
    if (reject_below <= 0) continue;
    const int remaining =
        static_cast<int>(n - static_cast<double>(done + 1)) * kUnitQuantSq;
    const int current = current_score();
    if (current + remaining < reject_total) return 0;
    if (current + static_cast<int>(static_cast<double>(remaining) *
                                   (1.0 - greediness)) <
        reject_total)
      return 0;
  }
  const double raw = static_cast<double>(current_score()) / (n * kUnitQuantSq);
  return std::clamp(raw, 0.0, 1.0);
}

template <int kLanes>
void score_discrete_i8_group(const std::int8_t* __restrict qnx,
                             const std::int8_t* __restrict qny,
                             const ShapeModel& model,
                             std::span<const DiscreteShapePoint> points,
                             std::size_t first_center_index, int live_mask,
                             double greediness, double reject_below,
                             double* scores) {
  constexpr int kAllLanes = (1 << kLanes) - 1;
  live_mask &= kAllLanes;
  for (int lane = 0; lane < kLanes; ++lane) scores[lane] = 0;
  if (live_mask == 0 || points.empty()) return;
#if MVKIT_SHAPE_HAS_SSE2
  const float n = static_cast<float>(points.size());
  const bool ignore_local_polarity =
      model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity =
      model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const __m128i zero = _mm_setzero_si128();
  __m128i pos = zero;
  __m128i neg = zero;
  int rejected_mask = kAllLanes & ~live_mask;
  float done = 0;
  const int reject_total =
      static_cast<int>(static_cast<float>(reject_below) * n * kUnitQuantSq);
  const __m128i reject_i32 = _mm_set1_epi32(reject_total);
  const float greedy_remaining_factor = 1.0F - static_cast<float>(greediness);
  const auto base = static_cast<std::ptrdiff_t>(first_center_index);
  const auto accumulate = [&](const DiscreteShapePoint& point) {
    const auto index = static_cast<std::size_t>(base + point.image_offset);
    // 相邻中心的 int8 法向是连续字节；madd 成 4 路 i32 点积。
    const __m128i nx16 = load_i8_as_i16x4(qnx + index, kLanes);
    const __m128i ny16 = load_i8_as_i16x4(qny + index, kLanes);
    const __m128i pairs = _mm_unpacklo_epi16(ny16, nx16);
    const auto nr = static_cast<std::uint16_t>(point.qn_row);
    const auto nc = static_cast<std::uint16_t>(point.qn_col);
    const __m128i coef = _mm_set1_epi32(
        (static_cast<unsigned>(nc) << 16) | static_cast<unsigned>(nr));
    __m128i dot = _mm_madd_epi16(pairs, coef);
    if (ignore_local_polarity) {
      pos = _mm_add_epi32(pos, abs_epi32_sse2(dot));
    } else {
      pos = _mm_add_epi32(pos, _mm_and_si128(dot, _mm_cmpgt_epi32(dot, zero)));
      if (ignore_global_polarity)
        neg = _mm_add_epi32(
            neg, _mm_and_si128(_mm_sub_epi32(zero, dot),
                               _mm_cmplt_epi32(dot, zero)));
    }
  };
  const auto current_i32 = [&] {
    if (!ignore_global_polarity) return pos;
    const __m128i pos_ge = _mm_cmpgt_epi32(pos, neg);
    return _mm_or_si128(_mm_and_si128(pos_ge, pos),
                        _mm_andnot_si128(pos_ge, neg));
  };
  const auto reject_limit = [&](int remaining_q) {
    rejected_mask |=
        _mm_movemask_ps(_mm_castsi128_ps(_mm_cmplt_epi32(
            _mm_add_epi32(current_i32(), _mm_set1_epi32(remaining_q)),
            reject_i32))) &
        kAllLanes;
  };
  const std::size_t greedy_index =
      greediness > 0 && reject_below > 0
          ? static_cast<std::size_t>(greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below)))
          : points.size();
  const std::size_t prefix = std::min(greedy_index, points.size());
  std::size_t i = 0;
  for (; i < prefix; ++i) {
    accumulate(points[i]);
    done += 1.0F;
    if (reject_below > 0) {
      reject_limit(static_cast<int>((n - done) * kUnitQuantSq));
      if ((rejected_mask & live_mask) == live_mask) return;
    }
  }
  for (; i < points.size(); ++i) {
    accumulate(points[i]);
    done += 1.0F;
    if (reject_below <= 0) continue;
    const int remaining = static_cast<int>((n - done) * kUnitQuantSq);
    reject_limit(remaining);
    reject_limit(static_cast<int>(static_cast<float>(remaining) *
                                  greedy_remaining_factor));
    if ((rejected_mask & live_mask) == live_mask) return;
  }
  alignas(16) int pos_values[4];
  alignas(16) int neg_values[4];
  _mm_store_si128(reinterpret_cast<__m128i*>(pos_values), pos);
  _mm_store_si128(reinterpret_cast<__m128i*>(neg_values), neg);
  const double inv_n = 1.0 / (static_cast<double>(n) * kUnitQuantSq);
  for (int lane = 0; lane < kLanes; ++lane) {
    if ((live_mask & (1 << lane)) == 0) continue;
    if ((rejected_mask & (1 << lane)) != 0) continue;
    const int raw_i =
        ignore_global_polarity ? std::max(pos_values[lane], neg_values[lane])
                               : pos_values[lane];
    scores[lane] = std::clamp(static_cast<double>(raw_i) * inv_n, 0.0, 1.0);
  }
#else
  for (int lane = 0; lane < kLanes; ++lane) {
    if ((live_mask & (1 << lane)) == 0) continue;
    scores[lane] = score_discrete_i8_points(
        qnx, qny, model, points,
        first_center_index + static_cast<std::size_t>(lane), greediness,
        reject_below);
  }
#endif
}

void score_discrete_live_interiors(const std::int8_t* __restrict qnx,
                                   const std::int8_t* __restrict qny,
                                   const ShapeModel& model,
                                   std::span<const DiscreteShapePoint> points,
                                   std::size_t index0, int lanes, int evidence,
                                   double greediness, double reject_below,
                                   double* scores, const int* remaining_bounds = nullptr,
                                   const PackedI8ShapePoint* packed_points = nullptr) {
  for (int lane = 0; lane < lanes; ++lane) scores[lane] = 0;
  if (evidence == 0 || points.empty()) return;
  if (lanes == 1) {
    scores[0] = score_discrete_i8_points(qnx, qny, model, points, index0,
                                         greediness, reject_below);
    return;
  }
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  if (simd::cpu_has_avx2()) {
    // 粗方向平面有 16 项守卫，2/4 lane 尾部可安全读八字节；仅活跃 lane 贡献或返回。
    double packed_scores[8];
    simd::avx2_kernels().score_i8_group8(qnx, qny, points.data(), points.size(), index0,
        evidence & ((1 << lanes) - 1), model.metric() == ShapeMetric::IgnoreLocalPolarity,
        model.metric() == ShapeMetric::IgnoreGlobalPolarity,
        greediness, reject_below, packed_scores, remaining_bounds, packed_points);
    std::copy_n(packed_scores, lanes, scores);
    return;
  }
#endif
  if (lanes >= 8) {
    const int low = evidence & 15;
    const int high = (evidence >> 4) & 15;
    if (low != 0)
      score_discrete_i8_group<4>(qnx, qny, model, points, index0, low,
                                 greediness, reject_below, scores);
    if (high != 0)
      score_discrete_i8_group<4>(qnx, qny, model, points, index0 + 4, high,
                                 greediness, reject_below, scores + 4);
  } else if (lanes >= 4) {
    score_discrete_i8_group<4>(qnx, qny, model, points, index0, evidence,
                               greediness, reject_below, scores);
  } else {
    score_discrete_i8_group<2>(qnx, qny, model, points, index0, evidence,
                               greediness, reject_below, scores);
  }
}

bool sample(const Grad& g, double r, double c, double* gx, double* gy, double* mag, bool bilinear = true) {
  if (!sample_components(g, r, c, gx, gy, bilinear)) return false;
  *mag = std::hypot(*gx, *gy);
  return true;
}

bool tuple_is(std::string_view s, const Tuple& t) {
  return t.size() == 1 && t.kind() == TupleKind::String && t.string_view() == s;
}

std::string_view item_string(const Tuple& t, std::size_t i) {
  if (t.at(i).kind() != ValueKind::String) {
    throw Exception(Status::InvalidArgument, "find_shape_model", "sub_pixel values must be strings");
  }
  return t.string_view(i);
}

int parse_max_deformation(std::string_view text) {
  constexpr std::string_view prefix = "max_deformation";
  if (text.size() < prefix.size() || text.substr(0, prefix.size()) != prefix) return -1;
  auto rest = text.substr(prefix.size());
  while (!rest.empty() && (rest.front() == ' ' || rest.front() == '_')) rest.remove_prefix(1);
  if (rest.empty()) return 0;
  int value = 0;
  for (char ch : rest) {
    if (ch < '0' || ch > '9') {
      throw Exception(Status::InvalidArgument, "find_shape_model", "max_deformation must be an integer 0..32");
    }
    value = value * 10 + (ch - '0');
  }
  if (value > 32) throw Exception(Status::InvalidArgument, "find_shape_model", "max_deformation must be in 0..32");
  return value;
}

SubPixelSpec::Mode parse_subpixel_mode(std::string_view value) {
  if (value == "none" || value == "false") return SubPixelSpec::Mode::None;
  if (value == "interpolation" || value == "true") return SubPixelSpec::Mode::Interpolation;
  if (value == "least_squares") return SubPixelSpec::Mode::LeastSquares;
  if (value == "least_squares_high") return SubPixelSpec::Mode::LeastSquaresHigh;
  if (value == "least_squares_very_high") return SubPixelSpec::Mode::LeastSquaresVeryHigh;
  throw Exception(Status::InvalidArgument, "find_shape_model",
                  "sub_pixel supports none/interpolation/least_squares[_high|_very_high]");
}

SubPixelSpec parse_sub_pixel(const Tuple& t) {
  SubPixelSpec spec;
  if (t.empty()) return spec;
  if (t.kind() != TupleKind::String && t.kind() != TupleKind::Mixed) {
    throw Exception(Status::InvalidArgument, "find_shape_model", "sub_pixel must be strings");
  }
  for (std::size_t i = 0; i < t.size(); ++i) {
    const auto text = item_string(t, i);
    if (const int deformation = parse_max_deformation(text); deformation >= 0) {
      spec.max_deformation = deformation;
      continue;
    }
    spec.mode = parse_subpixel_mode(text);
  }
  return spec;
}

int subpixel_iterations(SubPixelSpec::Mode mode) {
  switch (mode) {
    case SubPixelSpec::Mode::Interpolation: return 1;
    case SubPixelSpec::Mode::LeastSquares: return 2;
    case SubPixelSpec::Mode::LeastSquaresHigh: return 3;
    case SubPixelSpec::Mode::LeastSquaresVeryHigh: return 4;
    case SubPixelSpec::Mode::None: return 0;
  }
  return 0;
}

int geometric_iterations(SubPixelSpec::Mode mode) {
  switch (mode) {
    case SubPixelSpec::Mode::LeastSquares: return 1;
    case SubPixelSpec::Mode::LeastSquaresHigh: return 2;
    case SubPixelSpec::Mode::LeastSquaresVeryHigh: return 3;
    default: return 0;
  }
}

std::int64_t as_level_number(const Tuple& t, std::size_t index) {
  if (t.kind() == TupleKind::Int64) return t.as_int64(index);
  if (t.kind() == TupleKind::Real64) return static_cast<std::int64_t>(std::llround(t.as_real64(index)));
  if (t.kind() == TupleKind::Mixed) {
    const auto item = t.at(index);
    if (item.kind() == ValueKind::Int64) return item.as_int64();
    if (item.kind() == ValueKind::Real64) return static_cast<std::int64_t>(std::llround(item.as_real64()));
  }
  throw Exception(Status::InvalidArgument, "find_shape_model", "num_levels values must be integers");
}

SearchLevels parse_search_levels(const Tuple& t, std::size_t available) {
  SearchLevels levels{available - 1, 0};
  if (t.empty() || tuple_is("auto", t) || (t.kind() == TupleKind::Int64 && t.size() == 1 && t.as_int64() == 0)) {
    return levels;
  }
  if (t.size() == 1) {
    const auto n = as_level_number(t, 0);
    if (n < 0) throw Exception(Status::InvalidArgument, "find_shape_model", "num_levels must be nonnegative");
    if (n == 0) return levels;
    const auto used = std::min(available, static_cast<std::size_t>(n));
    return SearchLevels{used - 1, 0};
  }
  if (t.size() == 2) {
    const auto highest = as_level_number(t, 0);
    const auto lowest = as_level_number(t, 1);
    if (highest < 1 || lowest < 1 || highest < lowest) {
      throw Exception(Status::InvalidArgument, "find_shape_model",
                      "num_levels tuple must be [highest, lowest] with 1-based levels");
    }
    const auto coarsest = std::min(available, static_cast<std::size_t>(highest)) - 1;
    const auto finest = std::min(available, static_cast<std::size_t>(lowest)) - 1;
    if (coarsest < finest) {
      throw Exception(Status::InvalidArgument, "find_shape_model", "num_levels highest must not be finer than lowest");
    }
    return SearchLevels{coarsest, finest};
  }
  throw Exception(Status::InvalidArgument, "find_shape_model", "num_levels must be auto, 0, N or [highest, lowest]");
}

bool intersect_angle(double start, double extent, double model_start, double model_extent, double* out_start,
                     double* out_extent) {
  const double a = std::max(start, model_start);
  const double b = std::min(start + extent, model_start + model_extent);
  if (b + 1e-12 < a) return false;
  *out_start = a;
  *out_extent = std::max(0.0, b - a);
  return true;
}

bool intersect_scale(double smin, double smax, double model_min, double model_max, double* out_min, double* out_max) {
  *out_min = std::max(smin, model_min);
  *out_max = std::min(smax, model_max);
  return *out_min <= *out_max + 1e-12;
}

std::vector<double> samples(double start, double extent, double step) {
  std::vector<double> v;
  if (extent <= 1e-12) {
    v.push_back(start);
    return v;
  }
  const auto n = static_cast<std::size_t>(std::floor(extent / step)) + 1;
  for (std::size_t i = 0; i < n; ++i) v.push_back(start + static_cast<double>(i) * step);
  if (v.empty() || v.back() < start + extent - 1e-9) v.push_back(start + extent);
  return v;
}

template <class Points>
double score_rotated_points(const Grad& g, const ShapeModel& model, const Points& pts, double point_radius,
                            double pyramid_scale, double full_row, double full_col, double ca, double sa,
                            double scale, double greediness, double reject_below, int max_deformation,
                            bool bilinear = true) {
  const double row = full_row / pyramid_scale;
  const double col = full_col / pyramid_scale;
  const double min_mag = model.min_contrast() / std::max(1.0, pyramid_scale);
  const double min_mag_squared = min_mag * min_mag;
  const int deform =
      std::max(0, static_cast<int>(std::lround(static_cast<double>(max_deformation) / std::max(1.0, pyramid_scale))));
  const double n = static_cast<double>(pts.size());
  if (n <= 0) return 0;
  double pos = 0;
  double neg = 0;
  double done = 0;
  const bool ignore_local_polarity = model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity = model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const bool allow_border = model.border_shape_models();
  const double interior_margin = std::abs(scale) * point_radius + static_cast<double>(deform) + 1.0;
  const bool interior = row >= interior_margin && col >= interior_margin &&
                        row + interior_margin < static_cast<double>(g.h) &&
                        col + interior_margin < static_cast<double>(g.w);
  for (const auto& p : pts) {
    const double rr0 = row + scale * (-sa * p.column + ca * p.row);
    const double cc0 = col + scale * (ca * p.column + sa * p.row);
    // 模型法向为单位长度，旋转保范数，不为每候选每点重复开方归一化。
    const double nr = -sa * p.normal_column + ca * p.normal_row;
    const double nc = ca * p.normal_column + sa * p.normal_row;
    double best_pos = 0;
    double best_neg = 0;
    if (deform == 0) {
      double gx, gy;
      const bool sampled = interior ? (sample_components_interior(g, rr0, cc0, &gx, &gy, bilinear), true)
                                    : sample_components(g, rr0, cc0, &gx, &gy, bilinear);
      if (!sampled) {
        if (!allow_border) return 0;
      } else {
        const double mag_squared = gx * gx + gy * gy;
        if (mag_squared >= min_mag_squared) {
          const double dot = (nr * gy + nc * gx) / std::max(1e-12, std::sqrt(mag_squared));
          if (ignore_local_polarity) {
            best_pos = std::abs(dot);
          } else {
            best_pos = std::max(0.0, dot);
            if (ignore_global_polarity)
              best_neg = std::max(0.0, -dot);
          }
        }
      }
    } else {
      bool any = false;
      for (int k = -deform; k <= deform; ++k) {
        const double rr = rr0 + static_cast<double>(k) * nr;
        const double cc = cc0 + static_cast<double>(k) * nc;
        double gx, gy;
        const bool sampled = interior ? (sample_components_interior(g, rr, cc, &gx, &gy, bilinear), true)
                                      : sample_components(g, rr, cc, &gx, &gy, bilinear);
        if (!sampled) {
          if (!allow_border) return 0;
          continue;
        }
        any = true;
        const double mag_squared = gx * gx + gy * gy;
        if (mag_squared < min_mag_squared) continue;
        const double dot = (nr * gy + nc * gx) / std::max(1e-12, std::sqrt(mag_squared));
        if (ignore_local_polarity) {
          best_pos = std::max(best_pos, std::abs(dot));
        } else {
          best_pos = std::max(best_pos, std::max(0.0, dot));
          if (ignore_global_polarity)
            best_neg = std::max(best_neg, std::max(0.0, -dot));
        }
      }
      if (!any && !allow_border) return 0;
    }
    pos += best_pos;
    if (ignore_global_polarity) neg += best_neg;
    done += 1.0;
    if (reject_below > 0) {
      const double current = ignore_global_polarity ? std::max(pos, neg) : pos;
      const double remaining = n - done;
      if (current + remaining < reject_below * n) return 0;
      // greedy 上界前保留初始证据窗；预生成旋转降低粗离散噪声，但仍须容忍稀疏早期轮廓支撑。
      if (greediness > 0 && done >= greedy_prefix(n,greediness,reject_below) &&
          current + remaining * (1.0 - greediness) < reject_below * n)
        return 0;
    }
  }
  const double raw = (ignore_global_polarity ? std::max(pos, neg) : pos) / n;
  return std::clamp(raw, 0.0, 1.0);
}

template <class Points>
double score_rotated_unit_points(const UnitGrad& g, const ShapeModel& model,
                                 const Points& pts, double point_radius,
                                 double pyramid_scale, double full_row,
                                 double full_col, double ca, double sa,
                                 double scale, double greediness,
                                 double reject_below, int max_deformation) {
  const float row = static_cast<float>(full_row / pyramid_scale);
  const float col = static_cast<float>(full_col / pyramid_scale);
  const int deform = std::max(
      0, static_cast<int>(std::lround(static_cast<double>(max_deformation) /
                                     std::max(1.0, pyramid_scale))));
  const float n = static_cast<float>(pts.size());
  if (n <= 0) return 0;
  float pos = 0;
  float neg = 0;
  float done = 0;
  const bool ignore_local_polarity =
      model.metric() == ShapeMetric::IgnoreLocalPolarity;
  const bool ignore_global_polarity =
      model.metric() == ShapeMetric::IgnoreGlobalPolarity;
  const bool allow_border = model.border_shape_models();
  const float ca_f = static_cast<float>(ca);
  const float sa_f = static_cast<float>(sa);
  const float scale_f = static_cast<float>(scale);
  const float interior_margin =
      std::abs(scale_f) * static_cast<float>(point_radius) +
      static_cast<float>(deform) + 1.0F;
  const bool interior = row >= interior_margin && col >= interior_margin &&
                        row + interior_margin < static_cast<float>(g.h) &&
                        col + interior_margin < static_cast<float>(g.w);
  const float reject_total = static_cast<float>(reject_below) * n;
  const float greedy_begin = greedy_prefix(n,static_cast<float>(greediness),static_cast<float>(reject_below));
  const float greedy_remaining_factor = 1.0F - static_cast<float>(greediness);
  for (const auto& p : pts) {
    const float rr0 =
        row + scale_f * (-sa_f * static_cast<float>(p.column) +
                         ca_f * static_cast<float>(p.row));
    const float cc0 =
        col + scale_f * (ca_f * static_cast<float>(p.column) +
                         sa_f * static_cast<float>(p.row));
    const float nr = -sa_f * static_cast<float>(p.normal_column) +
                     ca_f * static_cast<float>(p.normal_row);
    const float nc = ca_f * static_cast<float>(p.normal_column) +
                     sa_f * static_cast<float>(p.normal_row);
    float best_pos = 0;
    float best_neg = 0;
    if (deform == 0) {
      float nx, ny;
      const bool sampled =
          interior
              ? (sample_unit_components_interior(g, rr0, cc0, &nx, &ny), true)
              : sample_unit_components(g, rr0, cc0, &nx, &ny);
      if (!sampled) {
        if (!allow_border) return 0;
      } else {
        const float dot = nr * ny + nc * nx;
        if (ignore_local_polarity) {
          best_pos = std::abs(dot);
        } else {
          best_pos = std::max(0.0F, dot);
          if (ignore_global_polarity) best_neg = std::max(0.0F, -dot);
        }
      }
    } else {
      bool any = false;
      for (int k = -deform; k <= deform; ++k) {
        const float rr = rr0 + static_cast<float>(k) * nr;
        const float cc = cc0 + static_cast<float>(k) * nc;
        float nx, ny;
        const bool sampled =
            interior
                ? (sample_unit_components_interior(g, rr, cc, &nx, &ny), true)
                : sample_unit_components(g, rr, cc, &nx, &ny);
        if (!sampled) {
          if (!allow_border) return 0;
          continue;
        }
        any = true;
        const float dot = nr * ny + nc * nx;
        if (ignore_local_polarity) {
          best_pos = std::max(best_pos, std::abs(dot));
        } else {
          best_pos = std::max(best_pos, std::max(0.0F, dot));
          if (ignore_global_polarity)
            best_neg = std::max(best_neg, std::max(0.0F, -dot));
        }
      }
      if (!any && !allow_border) return 0;
    }
    pos += best_pos;
    if (ignore_global_polarity) neg += best_neg;
    done += 1.0F;
    if (reject_below > 0) {
      const float current = ignore_global_polarity ? std::max(pos, neg) : pos;
      const float remaining = n - done;
      if (current + remaining < reject_total) return 0;
      if (greediness > 0 && done >= greedy_begin &&
          current + remaining * greedy_remaining_factor < reject_total)
        return 0;
    }
  }
  const float raw = (ignore_global_polarity ? std::max(pos, neg) : pos) / n;
  return std::clamp(static_cast<double>(raw), 0.0, 1.0);
}

double score_points(const Grad& g, const ShapeModel& model,
                    std::span<const ShapeModelPoint> pts,
                    double point_radius, double pyramid_scale, double full_row, double full_col,
                    double angle, double scale, double greediness, double reject_below,
                    int max_deformation, bool bilinear = true) {
  return score_rotated_points(g, model, pts, point_radius, pyramid_scale, full_row, full_col,
                              std::cos(angle), std::sin(angle), scale, greediness, reject_below,
                              max_deformation, bilinear);
}

double score_unit_points(const UnitGrad& g, const ShapeModel& model,
                         std::span<const ShapeModelPoint> pts,
                         double point_radius, double pyramid_scale,
                         double full_row, double full_col, double angle,
                         double scale, double greediness, double reject_below,
                         int max_deformation) {
  return score_rotated_unit_points(
      g, model, pts, point_radius, pyramid_scale, full_row, full_col,
      std::cos(angle), std::sin(angle), scale, greediness, reject_below,
      max_deformation);
}

double score_pose_level(const Grad& g, const ShapeModel& model, std::size_t level, double full_row, double full_col,
                        double angle, double scale, double greediness, double reject_below, int max_deformation,
                        bool bilinear = true) {
  const auto& ml = model.levels()[level];
  return score_points(g, model, ml.points, ml.radius, ml.pyramid_scale, full_row, full_col,
                      angle, scale, greediness, reject_below, max_deformation, bilinear);
}

struct Pt {
  double x, y;
};

double polygon_area(std::span<const Pt> p) {
  double a = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const auto& u = p[i];
    const auto& v = p[(i + 1) % p.size()];
    a += u.x * v.y - v.x * u.y;
  }
  return std::abs(a) * 0.5;
}

double cross(Pt a, Pt b, Pt c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
Pt intersection(Pt a, Pt b, Pt c, Pt d) {
  const double A1 = b.y - a.y, B1 = a.x - b.x, C1 = A1 * a.x + B1 * a.y;
  const double A2 = d.y - c.y, B2 = c.x - d.x, C2 = A2 * c.x + B2 * c.y;
  const double det = A1 * B2 - A2 * B1;
  if (std::abs(det) < 1e-12) return b;
  return {(B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det};
}

struct ModelBounds {
  double rmin, rmax, cmin, cmax;
};

ModelBounds model_bounds(const ShapeModel& model) {
  double rmin = 1e99, rmax = -1e99, cmin = 1e99, cmax = -1e99;
  for (const auto& p : model.levels().front().points) {
    rmin = std::min(rmin, p.row);
    rmax = std::max(rmax, p.row);
    cmin = std::min(cmin, p.column);
    cmax = std::max(cmax, p.column);
  }
  return {rmin, rmax, cmin, cmax};
}
matching::Box model_overlap_box(const ShapeModel& model) {
  const auto build=[&] {
    runtime::ScratchArena scratch;std::pmr::vector<matching::Point> points{scratch.resource()};
    points.reserve(model.levels().front().points.size());
    for(const auto& p:model.levels().front().points)points.push_back({p.column,p.row});
    return matching::minimum_rectangle(points,scratch.resource());
  };
  if(!model.model_cache_enabled() || !model.search_cache())return build();
  auto& cache=*model.search_cache();std::lock_guard lock(cache.mutex);
  if(!cache.overlap_box)cache.overlap_box=std::make_shared<const matching::Box>(build());
  return *cache.overlap_box;
}

std::array<Pt, 4> box_four(const ModelBounds& bounds, const MatchCandidate& x) {
  const auto [rmin, rmax, cmin, cmax] = bounds;
  std::array<std::pair<double, double>, 4> q{{{rmin, cmin}, {rmin, cmax}, {rmax, cmax}, {rmax, cmin}}};
  std::array<Pt, 4> out;
  const double ca = std::cos(x.angle), sa = std::sin(x.angle);
  std::size_t i = 0;
  for (auto [r, c] : q)
    out[i++] = {x.column + x.scale * (ca * c + sa * r), x.row + x.scale * (-sa * c + ca * r)};
  return out;
}

double clip_four_area(const std::array<Pt, 4>& subject, const std::array<Pt, 4>& clip) {
  // 每边对每输入顶点最多输出两个顶点；四初始顶点/四条边最多 64 槽，含 epsilon 同值。
  std::array<Pt, 64> first, second;
  Pt* input = first.data();
  Pt* output = second.data();
  std::copy(subject.begin(), subject.end(), input);
  std::size_t count = subject.size();
  double orient = 0;
  for (std::size_t i = 0; i < clip.size(); ++i) {
    const auto& a = clip[i];
    const auto& b = clip[(i + 1) % clip.size()];
    orient += a.x * b.y - b.x * a.y;
  }
  const double sign = orient >= 0 ? 1.0 : -1.0;
  for (std::size_t e = 0; e < clip.size(); ++e) {
    const Pt A = clip[e], B = clip[(e + 1) % clip.size()];
    if (count == 0) break;
    Pt S = input[count - 1];
    std::size_t next = 0;
    auto inside = [&](Pt p) { return sign * cross(A, B, p) >= -1e-9; };
    for (std::size_t i = 0; i < count; ++i) {
      const Pt E = input[i];
      if (inside(E)) {
        if (!inside(S)) output[next++] = intersection(S, E, A, B);
        output[next++] = E;
      } else if (inside(S)) {
        output[next++] = intersection(S, E, A, B);
      }
      S = E;
    }
    count = next;
    std::swap(input, output);
  }
  return polygon_area(std::span<const Pt>{input, count});
}

double overlap(const ModelBounds& bounds, const MatchCandidate& a, const MatchCandidate& b) {
  auto A = box_four(bounds, a), B = box_four(bounds, b);
  const double aa = polygon_area(std::span<const Pt>{A}), ba = polygon_area(std::span<const Pt>{B});
  if (aa <= 1e-12 || ba <= 1e-12) return 0;
  return clip_four_area(A, B) / std::min(aa, ba);
}

double parabola_offset(double fm, double f0, double fp) {
  const double d = fm - 2 * f0 + fp;
  if (std::abs(d) < 1e-12) return 0;
  return std::clamp(0.5 * (fm - fp) / d, -0.5, 0.5);
}

bool solve_linear_system(double matrix[4][4], double right[4], int dimension, double solution[4]) {
  double a[4][4]{};
  double b[4]{};
  for (int r = 0; r < dimension; ++r) {
    b[r] = right[r];
    for (int c = 0; c < dimension; ++c) a[r][c] = matrix[r][c];
  }
  for (int pivot = 0; pivot < dimension; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < dimension; ++row) {
      if (std::abs(a[row][pivot]) > std::abs(a[best][pivot])) best = row;
    }
    if (std::abs(a[best][pivot]) < 1e-12) return false;
    if (best != pivot) {
      for (int column = pivot; column < dimension; ++column) std::swap(a[pivot][column], a[best][column]);
      std::swap(b[pivot], b[best]);
    }
    for (int row = pivot + 1; row < dimension; ++row) {
      const double factor = a[row][pivot] / a[pivot][pivot];
      for (int column = pivot; column < dimension; ++column) a[row][column] -= factor * a[pivot][column];
      b[row] -= factor * b[pivot];
    }
  }
  for (int row = dimension - 1; row >= 0; --row) {
    double value = b[row];
    for (int column = row + 1; column < dimension; ++column) value -= a[row][column] * solution[column];
    solution[row] = value / a[row][row];
  }
  return true;
}

double normalize_angle(double angle) {
  constexpr double pi = 3.14159265358979323846;
  while (angle <= -pi) angle += 2.0 * pi;
  while (angle > pi) angle -= 2.0 * pi;
  return angle;
}

MatchCandidate refine_interpolation(const Grad& g, const ShapeModel& m, std::size_t level, MatchCandidate x,
                                    double astep, double sstep, bool scaled, int max_deformation) {
  const std::span<const ShapeModelPoint> pts =
      (level == 0 && !m.refinement_points().empty())
          ? std::span<const ShapeModelPoint>{m.refinement_points()}
          : std::span<const ShapeModelPoint>{m.levels()[level].points};
  const double point_radius = m.levels()[level].radius;
  const double pyr = m.levels()[level].pyramid_scale;
  auto score = [&](double r, double c, double a, double s) {
    return score_points(g, m, pts, point_radius, pyr, r, c, a, s, 0, 0, max_deformation);
  };
  auto keep_if_better = [&](MatchCandidate cand, MatchCandidate trial) {
    trial.score = score(trial.row, trial.column, trial.angle, trial.scale);
    return trial.score + 1e-15 >= cand.score ? trial : cand;
  };

  auto sequential_fit = [&](MatchCandidate cand, double tstep, double a_step, double s_step) {
    cand.score = score(cand.row, cand.column, cand.angle, cand.scale);
    double s[3][3]{};
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        s[dr + 1][dc + 1] = score(cand.row + dr * tstep, cand.column + dc * tstep, cand.angle, cand.scale);
      }
    }
    const double a = s[1][1];
    const double b = 0.5 * (s[2][1] - s[0][1]);
    const double c = 0.5 * (s[1][2] - s[1][0]);
    const double d = 0.5 * (s[2][1] + s[0][1]) - a;
    const double e = 0.25 * (s[2][2] - s[0][2] - s[2][0] + s[0][0]);
    const double f = 0.5 * (s[1][2] + s[1][0]) - a;
    MatchCandidate trial = cand;
    const double det = 4.0 * d * f - e * e;
    if (d < 0.0 && f < 0.0 && std::abs(det) > 1e-12) {
      trial.row += std::clamp((-2.0 * f * b + e * c) / det, -1.0, 1.0) * tstep;
      trial.column += std::clamp((-2.0 * d * c + e * b) / det, -1.0, 1.0) * tstep;
    } else {
      if (d < 0.0) trial.row += parabola_offset(s[0][1], a, s[2][1]) * tstep;
      if (f < 0.0) trial.column += parabola_offset(s[1][0], a, s[1][2]) * tstep;
    }
    cand = keep_if_better(cand, trial);

    const double f0 = cand.score;
    const double fm = score(cand.row, cand.column, cand.angle - a_step, cand.scale);
    const double fp = score(cand.row, cand.column, cand.angle + a_step, cand.scale);
    const double ad = fm - 2.0 * f0 + fp;
    if (ad < -1e-12) {
      MatchCandidate angled = cand;
      angled.angle += std::clamp(0.5 * (fm - fp) / ad, -0.5, 0.5) * a_step;
      cand = keep_if_better(cand, angled);
    }
    if (scaled && s_step > 0) {
      const double g0 = cand.score;
      const double gm = score(cand.row, cand.column, cand.angle, cand.scale - s_step);
      const double gp = score(cand.row, cand.column, cand.angle, cand.scale + s_step);
      const double sd = gm - 2.0 * g0 + gp;
      if (sd < -1e-12) {
        MatchCandidate scaled_c = cand;
        scaled_c.scale += std::clamp(0.5 * (gm - gp) / sd, -0.5, 0.5) * s_step;
        cand = keep_if_better(cand, scaled_c);
      }
    }
    return cand;
  };

  auto joint_fit = [&](MatchCandidate cand, double tstep, double a_step) {
    cand.score = score(cand.row, cand.column, cand.angle, cand.scale);
    double f[3][3][3]{};
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        for (int da = -1; da <= 1; ++da) {
          f[dr + 1][dc + 1][da + 1] =
              score(cand.row + dr * tstep, cand.column + dc * tstep, cand.angle + da * a_step, cand.scale);
        }
      }
    }
    auto F = [&](int dr, int dc, int da) { return f[dr + 1][dc + 1][da + 1]; };
    const double f0 = F(0, 0, 0);
    double hess[4][4]{};
    double rhs[4]{};
    hess[0][0] = F(1, 0, 0) - 2.0 * f0 + F(-1, 0, 0);
    hess[1][1] = F(0, 1, 0) - 2.0 * f0 + F(0, -1, 0);
    hess[2][2] = F(0, 0, 1) - 2.0 * f0 + F(0, 0, -1);
    if (hess[0][0] >= -1e-12 || hess[1][1] >= -1e-12 || hess[2][2] >= -1e-12) return cand;
    hess[0][1] = hess[1][0] = 0.25 * (F(1, 1, 0) - F(1, -1, 0) - F(-1, 1, 0) + F(-1, -1, 0));
    hess[0][2] = hess[2][0] = 0.25 * (F(1, 0, 1) - F(1, 0, -1) - F(-1, 0, 1) + F(-1, 0, -1));
    hess[1][2] = hess[2][1] = 0.25 * (F(0, 1, 1) - F(0, 1, -1) - F(0, -1, 1) + F(0, -1, -1));
    rhs[0] = -0.5 * (F(1, 0, 0) - F(-1, 0, 0));
    rhs[1] = -0.5 * (F(0, 1, 0) - F(0, -1, 0));
    rhs[2] = -0.5 * (F(0, 0, 1) - F(0, 0, -1));
    double sol[4]{};
    if (!solve_linear_system(hess, rhs, 3, sol)) return cand;
    MatchCandidate trial = cand;
    trial.row += std::clamp(sol[0], -1.0, 1.0) * tstep;
    trial.column += std::clamp(sol[1], -1.0, 1.0) * tstep;
    trial.angle += std::clamp(sol[2], -1.0, 1.0) * a_step;
    trial.score = score(trial.row, trial.column, trial.angle, trial.scale);
    return trial.score + 1e-15 >= f0 ? trial : cand;
  };

  x.score = score(x.row, x.column, x.angle, x.scale);
  x = sequential_fit(x, 1.0, astep, sstep);
  x = joint_fit(x, 1.0, astep);
  x = sequential_fit(x, 0.5, 0.5 * astep, 0.5 * sstep);
  x = joint_fit(x, 0.5, 0.5 * astep);
  x = sequential_fit(x, 0.25, 0.25 * astep, 0.25 * sstep);
  x = joint_fit(x, 0.25, 0.25 * astep);
  x.score = score(x.row, x.column, x.angle, x.scale);
  return x;
}

MatchCandidate refine_least_squares(const Grad& g, const ShapeModel& m, std::size_t level, MatchCandidate result,
                                    std::span<const ShapeModelPoint> features, double feature_radius,
                                    int iterations, double angle_step,
                                    double scale_step, double scale_min, double scale_max, int max_deformation) {
  double translation_step = 0.75;
  double rotation_step = std::max(angle_step * 0.75, 0.0005);
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  const bool avx_score = simd::cpu_has_avx2() && g.has_integer_components() &&

                         max_deformation == 0 && !m.border_shape_models() &&
                         !features.empty();
#else
  const bool avx_score = false;
#endif
  std::shared_ptr<const PreparedPointSoA> feature_soa;
  std::span<const float> prow, pcol, pnr, pnc;
  if (avx_score) {
    feature_soa = prepared_point_soa(m, level, true);
    prow = feature_soa->row; pcol = feature_soa->column;
    pnr = feature_soa->nr; pnc = feature_soa->nc;
  }
  const float min_mag_sq =
      static_cast<float>(m.min_contrast() * m.min_contrast());
  auto score_candidate = [&](const MatchCandidate& candidate) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (avx_score) {
      const double margin = std::abs(candidate.scale) * feature_radius + 1.0;
      if (candidate.row >= margin && candidate.column >= margin &&
          candidate.row + margin < static_cast<double>(g.h) &&
          candidate.column + margin < static_cast<double>(g.w)) {
        return score_i16_nearest_components(g, min_mag_sq, prow.data(),
            pcol.data(), pnr.data(), pnc.data(), prow.size(),
            static_cast<float>(candidate.row),
            static_cast<float>(candidate.column),
            static_cast<float>(std::cos(candidate.angle)),
            static_cast<float>(std::sin(candidate.angle)),
            static_cast<float>(candidate.scale), 0.0F, 0.0F, static_cast<int>(m.metric()));
      }
    }
#endif
    return score_points(g, m, features, feature_radius, 1.0, candidate.row, candidate.column,
                        candidate.angle, candidate.scale, 0, 0, max_deformation);
  };
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const MatchCandidate iteration_start = result;
    const bool use_scale = scale_step > 0.0 && scale_max > scale_min;
    const int dimension = use_scale ? 4 : 3;
    const double steps[4] = {translation_step, translation_step, rotation_step, use_scale ? scale_step : 1.0};
    auto displaced = [&](int axis, double delta) {
      MatchCandidate candidate = result;
      if (axis == 0) candidate.row += delta;
      else if (axis == 1) candidate.column += delta;
      else if (axis == 2) candidate.angle += delta;
      else candidate.scale = std::clamp(candidate.scale + delta, scale_min, scale_max);
      return candidate;
    };
    const double center = result.score;
    double gradient_vector[4]{};
    double hessian[4][4]{};
    double minus_scores[4]{};
    double plus_scores[4]{};
    for (int axis = 0; axis < dimension; ++axis) {
      const double minus = score_candidate(displaced(axis, -steps[axis]));
      const double plus = score_candidate(displaced(axis, steps[axis]));
      minus_scores[axis] = minus;
      plus_scores[axis] = plus;
      gradient_vector[axis] = (plus - minus) / (2.0 * steps[axis]);
      hessian[axis][axis] = (plus - 2.0 * center + minus) / (steps[axis] * steps[axis]);
    }
    // 金字塔细化后平移/旋转/尺度已近最优；对角二次拟合复用中心一维样本，
    // 省每刚性迭代十二次完整轮廓评分，后续几何正规方程仍求剩余耦合。
    double right[4]{};
    for (int axis = 0; axis < dimension; ++axis) right[axis] = -gradient_vector[axis];
    double update[4]{};
    bool solved = solve_linear_system(hessian, right, dimension, update);
    if (solved) {
      for (int axis = 0; axis < dimension; ++axis)
        update[axis] = std::clamp(update[axis], -steps[axis] * 1.5, steps[axis] * 1.5);
      bool improved = false;
      for (double factor : {1.0, 0.5, 0.25}) {
        MatchCandidate trial = result;
        trial.row += factor * update[0];
        trial.column += factor * update[1];
        trial.angle = normalize_angle(trial.angle + factor * update[2]);
        if (use_scale) trial.scale = std::clamp(trial.scale + factor * update[3], scale_min, scale_max);
        trial.score = score_candidate(trial);
        if (trial.score > center) {
          result = trial;
          improved = true;
          break;
        }
      }
      if (!improved) solved = false;
    }
    if (!solved) {
      result.row +=
          parabola_offset(minus_scores[0], center, plus_scores[0]) * steps[0];
      result.column +=
          parabola_offset(minus_scores[1], center, plus_scores[1]) * steps[1];
      result.angle = normalize_angle(
          result.angle +
          parabola_offset(minus_scores[2], center, plus_scores[2]) * steps[2]);
      if (use_scale) {
        result.scale =
            std::clamp(result.scale +
                           parabola_offset(minus_scores[3], center,
                                           plus_scores[3]) * steps[3],
                       scale_min, scale_max);
      }
      result.score = score_candidate(result);
    }
    translation_step *= 0.65;
    rotation_step *= 0.65;
    scale_step *= 0.65;
    const double translation_change =
        std::hypot(result.row - iteration_start.row,
                   result.column - iteration_start.column);
    const double angle_change =
        std::abs(normalize_angle(result.angle - iteration_start.angle));
    const double scale_change = std::abs(result.scale - iteration_start.scale);
    if (translation_change < 0.05 && angle_change < 0.0005 &&
        scale_change < 0.002)
      break;
  }
  return result;
}

MatchCandidate refine_geometric(const Grad& g, const ShapeModel& m, MatchCandidate result,
                                std::span<const ShapeModelPoint> features, double feature_radius,
                                int iterations, double search_limit, double scale_min,
                                double scale_max, int max_deformation) {
  const bool use_scale = scale_max > scale_min + 1e-12;
  const int dimension = use_scale ? 4 : 3;
  search_limit = std::max(search_limit,
                          static_cast<double>(max_deformation) + 1.0);
  constexpr double search_step = 0.25;
  const int span = static_cast<int>(std::lround(search_limit / search_step));
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  const bool avx_geo_hw = simd::cpu_has_avx2() && g.has_integer_components() &&
                          !m.is_xld_model() && span >= 0 &&
                          2 * span + 1 <= 64;
#else
  const bool avx_geo_hw = false;
#endif
  for (int iteration = 0; iteration < iterations; ++iteration) {
    double normal_matrix[4][4]{};
    double right[4]{};
    int used = 0;
    const double ca = std::cos(result.angle);
    const double sa = std::sin(result.angle);
    const double interior_margin =
        std::abs(result.scale) * feature_radius + search_limit + 1.0;
    const bool interior =
        result.row >= interior_margin && result.column >= interior_margin &&
        result.row + interior_margin < static_cast<double>(g.h) &&
        result.column + interior_margin < static_cast<double>(g.w);
    const std::size_t feature_stride = iterations == 1 ? 2u : 1u;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    float peak_left[8], peak_center[8], peak_right[8];
    int peak_index[8];
    double cached_offset_row[8],cached_offset_col[8],cached_nr[8],cached_nc[8],cached_base[8],cached_step[8];
    int cached_span[8];
    bool batch_ready=false;
    const bool xld_batch=m.is_xld_model() && simd::cpu_has_avx2() &&
        g.has_integer_components() && feature_stride==1;
#endif
    for (std::size_t feature_index = 0; feature_index < features.size();
         feature_index += feature_stride) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if (xld_batch && feature_index%8==0) {
        float er[8],ec[8],nr[8],nc[8],base[8],step[8];int spans[8],maximum_span=0;
        batch_ready=true;
        for(std::size_t lane=0;lane<8;++lane) {
          const auto& p=features[std::min(feature_index+lane,features.size()-1)];
          cached_offset_row[lane]=result.scale*(-sa*p.column+ca*p.row);
          cached_offset_col[lane]=result.scale*(ca*p.column+sa*p.row);
          const double r=result.row+cached_offset_row[lane];
          const double c=result.column+cached_offset_col[lane];
          const double nrow=-sa*p.normal_column+ca*p.normal_row, ncol=ca*p.normal_column+sa*p.normal_row;
          const bool row_axis=std::abs(nrow)>=std::abs(ncol);
          const double axis=row_axis?nrow:ncol, coordinate=row_axis?r:c;
          const double increment=1/std::abs(axis),shift=(std::round(coordinate)-coordinate)/axis;
          cached_nr[lane]=nrow;cached_nc[lane]=ncol;cached_base[lane]=shift;cached_step[lane]=increment;
          spans[lane]=static_cast<int>(std::ceil(search_limit/increment));
          cached_span[lane]=spans[lane];
          maximum_span=std::max(maximum_span,spans[lane]);
          const double reach=std::abs(shift)+spans[lane]*increment;
          // Float gather 要求真实邻居，补齐 lane 也如此。
          batch_ready=batch_ready && r-reach*std::abs(nrow)>=1 && c-reach*std::abs(ncol)>=1 &&
              r+reach*std::abs(nrow)<g.h-2 && c+reach*std::abs(ncol)<g.w-2;
          er[lane]=static_cast<float>(r);ec[lane]=static_cast<float>(c);
          nr[lane]=static_cast<float>(nrow);nc[lane]=static_cast<float>(ncol);
          base[lane]=static_cast<float>(shift);step[lane]=static_cast<float>(increment);
        }
        if(batch_ready) simd::avx2_kernels().xld_geometric_i16_peaks(
            g.gx_i16.data(),g.gy_i16.data(),g.w,static_cast<float>(g.integer_scale),
            er,ec,nr,nc,base,step,spans,maximum_span,
            static_cast<float>(m.min_contrast()*m.min_contrast()),m.metric()!=ShapeMetric::UsePolarity,
            peak_left,peak_center,peak_right,peak_index);
      }
#endif
      const auto& feature = features[feature_index];
      double offset_row = result.scale * (-sa * feature.column + ca * feature.row);
      double offset_col = result.scale * (ca * feature.column + sa * feature.row);
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if(xld_batch){offset_row=cached_offset_row[feature_index%8];offset_col=cached_offset_col[feature_index%8];}
#endif
      const double expected_row = result.row + offset_row;
      const double expected_col = result.column + offset_col;
      double normal_row = -sa * feature.normal_column + ca * feature.normal_row;
      double normal_col = ca * feature.normal_column + sa * feature.normal_row;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if(xld_batch){normal_row=cached_nr[feature_index%8];normal_col=cached_nc[feature_index%8];}
#endif
      auto response = [&](double distance) {
        double gx, gy;
        const double row = expected_row + distance * normal_row;
        const double column = expected_col + distance * normal_col;
        const bool sampled =
            interior
                ? (sample_components_interior(g, row, column, &gx, &gy,
                                              true),
                   true)
                : sample_components(g, row, column, &gx, &gy, true);
        if (!sampled)
          return 0.0;
        const double magnitude_squared = gx * gx + gy * gy;
        const double min_contrast_squared =
            m.min_contrast() * m.min_contrast();
        if (magnitude_squared < min_contrast_squared) return 0.0;
        const double magnitude = std::sqrt(magnitude_squared);
        const double orientation =
            (normal_row * gy + normal_col * gx) /
            std::max(1e-12, magnitude);
        const double aligned =
            m.metric() == ShapeMetric::UsePolarity ? std::max(0.0, orientation) : std::abs(orientation);
        return magnitude * aligned * aligned;
      };
      const bool avx_geo = avx_geo_hw && interior;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      const bool fused_peak17 = avx_geo && iterations == 1 && span == 8 && search_limit == 2.0;
      float avx_responses[64];
      if (avx_geo && !fused_peak17) {
        geometric_i16_bilinear_components(g,
            static_cast<float>(expected_row),
            static_cast<float>(expected_col),
            static_cast<float>(normal_row),
            static_cast<float>(normal_col),
            static_cast<float>(search_step), span,
            static_cast<float>(m.min_contrast() * m.min_contrast()),
            avx_responses, m.metric() != ShapeMetric::UsePolarity);
      }
#endif
      int best_index = 0;
      double best_response = 0.0;
      double best_left = 0.0;
      double best_right = 0.0;
      double previous_response = 0.0;
      const double axis = std::abs(normal_row) >= std::abs(normal_col) ? normal_row : normal_col;
      const double coordinate = std::abs(normal_row) >= std::abs(normal_col) ? expected_row : expected_col;
      double peak_step = search_step,peak_base = 0;int peak_span=span;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if(xld_batch){peak_step=cached_step[feature_index%8];peak_base=cached_base[feature_index%8];peak_span=cached_span[feature_index%8];}
      else
#endif
      if(m.is_xld_model()) {
        peak_step=1/std::abs(axis);peak_base=(std::round(coordinate)-coordinate)/axis;
        peak_span=static_cast<int>(std::ceil(search_limit/peak_step));
      }
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if (xld_batch && batch_ready) {
        const auto lane=feature_index%8;
        best_index=peak_index[lane];best_response=peak_center[lane];
        best_left=peak_left[lane];best_right=peak_right[lane];
      } else if (avx_geo && span == 8) {
        const auto peak = fused_peak17 ? geometric_i16_bilinear_peak17_components(g,
            static_cast<float>(expected_row), static_cast<float>(expected_col),
            static_cast<float>(normal_row), static_cast<float>(normal_col),
            static_cast<float>(search_step), static_cast<float>(m.min_contrast() * m.min_contrast()),
            m.metric() != ShapeMetric::UsePolarity) :
            simd::avx2_kernels().geometric_response_peak(avx_responses, span);
        best_index = peak.index; best_response = peak.center;
        best_left = peak.left; best_right = peak.right;
      } else
#endif
      for (int index = -peak_span; index <= peak_span; ++index) {
        const double value =
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
            avx_geo ? static_cast<double>(avx_responses[index + span]) :
#endif
            response(peak_base + index * peak_step);
        if (index == best_index + 1) best_right = value;
        if (value > best_response) {
          best_response = value;
          best_index = index;
          best_left = previous_response;
          best_right = 0.0;
        }
        previous_response = value;
      }
      if (best_response <= 0.0 || best_index == -peak_span || best_index == peak_span) continue;
      double residual =
          (best_index +
           parabola_offset(best_left, best_response, best_right)) *
          peak_step + peak_base;
      if (m.is_xld_model()) {
        // 双线性响应峰受像素格限制，改用主法向轴的三个独立像素样本拟合。
        if (best_left > 0 && best_right > 0) {
          residual = peak_base + peak_step * (best_index +
              parabola_offset(std::log(best_left/best_response), 0, std::log(best_right/best_response)));
        }
        if (std::abs(residual) > search_limit) continue;
      }
      double jacobian[4] = {normal_row, normal_col, normal_row * (-offset_col) + normal_col * offset_row, 0.0};
      if (use_scale && result.scale > 1e-12) {
        jacobian[3] = normal_row * (offset_row / result.scale) + normal_col * (offset_col / result.scale);
      }
      const double weight = std::clamp(best_response / 1020.0, 0.05, 1.0);
      for (int row = 0; row < dimension; ++row) {
        right[row] += weight * jacobian[row] * residual;
        for (int column = 0; column < dimension; ++column) {
          normal_matrix[row][column] += weight * jacobian[row] * jacobian[column];
        }
      }
      ++used;
    }
    if (used < 12) break;
    for (int axis = 0; axis < dimension; ++axis) {
      normal_matrix[axis][axis] += 1e-4 * std::max(1.0, normal_matrix[axis][axis]);
    }
    double update[4]{};
    if (!solve_linear_system(normal_matrix, right, dimension, update)) break;
    result.row += std::clamp(update[0], -0.6, 0.6);
    result.column += std::clamp(update[1], -0.6, 0.6);
    result.angle = normalize_angle(result.angle + std::clamp(update[2], -0.05, 0.05));
    if (use_scale) result.scale = std::clamp(result.scale + std::clamp(update[3], -0.05, 0.05), scale_min, scale_max);
    if (m.is_xld_model() && std::hypot(update[0], update[1]) < .002 &&
        std::abs(update[2]) < .0001 && (!use_scale || std::abs(update[3]) < .0001)) break;
  }
  return result;
}

MatchCandidate apply_halcon_affine(MatchCandidate candidate) {
  const double cosine = std::cos(candidate.angle);
  const double sine = -std::sin(candidate.angle);
  candidate.row = candidate.row + 0.5 - 0.5 * candidate.scale * (sine + cosine);
  candidate.column = candidate.column + 0.5 - 0.5 * candidate.scale * (cosine - sine);
  candidate.angle = normalize_angle(candidate.angle);
  return candidate;
}

MatchCandidate refine_subpixel(const Grad& g, const ShapeModel& m, std::size_t level, MatchCandidate x,
                               SubPixelSpec spec, double astep, double sstep, bool scaled) {
  if (spec.mode == SubPixelSpec::Mode::None) return x;
  if (spec.mode == SubPixelSpec::Mode::Interpolation) {
    return refine_interpolation(g, m, level, x, astep, sstep, scaled, spec.max_deformation);
  }
  const std::span<const ShapeModelPoint> features =
      (level == 0 && !m.refinement_points().empty())
          ? std::span<const ShapeModelPoint>{m.refinement_points()}
          : std::span<const ShapeModelPoint>{m.levels()[level].points};
  const double feature_radius = m.levels()[level].radius;
  const double scale_min = scaled ? m.scale_min() : 1.0;
  const double scale_max = scaled ? m.scale_max() : 1.0;
  if (!m.is_xld_model()) x = refine_least_squares(g, m, level, x, features, feature_radius, subpixel_iterations(spec.mode), astep,
                           scaled ? sstep : 0.0, scale_min,
                           scale_max, spec.max_deformation);
  const int geo = m.is_xld_model() ? std::max(3, geometric_iterations(spec.mode)) : geometric_iterations(spec.mode);
  if (geo > 0) {
    const double geometric_search_limit =
        spec.mode == SubPixelSpec::Mode::LeastSquares
            ? 2.0
            : (spec.mode == SubPixelSpec::Mode::LeastSquaresHigh ? 4.0
                                                                  : 6.0);
    x = refine_geometric(g, m, x, features, feature_radius, geo,
                         geometric_search_limit, scale_min, scale_max,
                         spec.max_deformation);
  }
  return x;
}

std::vector<MatchCandidate> coarse_to_fine(const Image& image, GradientPyramid& pyramid,
                                           const ShapeModel& model, double astart, double aextent,
                                           double smin, double smax, double min_score, SearchLevels levels,
                                           double greediness, bool scaled, int max_deformation,
                                           std::size_t candidate_limit,
                                           const TimeoutWatch& timeout,
                                           ShapeSearchDiagnostics* diagnostics,
                                           std::pmr::memory_resource* memory, CoarseAnglePhase phase, bool local_maxima = false,
                                           unsigned radix_trial_sort = 0) {
  auto& gp = pyramid.gradients;
  auto& unit_gp = pyramid.unit_gradients;
  const std::size_t top = levels.coarsest;
  const double div = model.levels()[top].pyramid_scale;
  const double astep = model.angle_step() * std::max(1.0, div);
  const double sstep = model.scale_step() * std::max(1.0, div);
  auto angles = select_angle_phase(samples(astart, aextent, astep), phase);
  if (angles.empty()) return {};
  auto scales = scaled ? samples(smin, smax - smin, std::max(sstep, 1e-6))
                       : std::vector<double>{1.0};
  const auto& top_points = model.levels()[top].points;
  // 缩减域仅约束候选中心，不使金字塔网格变成小数；
  // 除非边界/形变确需连续点，否则保留离散 SIMD 计划。
  const bool needs_continuous_points =
      model.border_shape_models() || max_deformation != 0;
  constexpr double coarse_score_factor = 0.45;
  constexpr double refinement_score_factor = 0.40;
  auto build_plan = [&] {
    auto result = std::make_shared<std::vector<PreparedShapePose>>();
    result->reserve(angles.size() * scales.size());
    for (double angle : angles) {
      const double ca = std::cos(angle);
      const double sa = std::sin(angle);
      for (double pose_scale : scales) {
        result->emplace_back();
        auto& prepared = result->back();
        prepared.angle = angle;
        prepared.scale = pose_scale;
        if (needs_continuous_points)
          prepared.points.reserve(top_points.size());
        prepared.discrete_points.reserve(top_points.size());
        for (const auto& point : top_points) {
          ShapeModelPoint rotated{-sa * point.column + ca * point.row,
                                  ca * point.column + sa * point.row,
                                  -sa * point.normal_column + ca * point.normal_row,
                                  ca * point.normal_column + sa * point.normal_row,
                                  point.response};
          if (needs_continuous_points) prepared.points.push_back(rotated);
          const double scaled_row = pose_scale * rotated.row;
          const double scaled_col = pose_scale * rotated.column;
          // 原点修改/读取模型可能超过创建范围，在浮点/整数转换及打包窄化前验证。
          if (!std::isfinite(scaled_row) || !std::isfinite(scaled_col) ||
              std::abs(scaled_row) > 32760 || std::abs(scaled_col) > 32760)
            throw Exception(Status::InvalidArgument, "find_shape_model",
                            "scaled model offsets exceed discrete plan bounds");
          const int row_offset = static_cast<int>(std::floor(scaled_row + 0.5));
          const int col_offset = static_cast<int>(std::floor(scaled_col + 0.5));
          prepared.min_row = std::min(prepared.min_row, scaled_row);
          prepared.max_row = std::max(prepared.max_row, scaled_row);
          prepared.min_col = std::min(prepared.min_col, scaled_col);
          prepared.max_col = std::max(prepared.max_col, scaled_col);
          const auto n_row = static_cast<float>(rotated.normal_row);
          const auto n_col = static_cast<float>(rotated.normal_column);
          prepared.discrete_points.push_back(
              {static_cast<std::ptrdiff_t>(row_offset) *
                       static_cast<std::ptrdiff_t>(unit_gp[top].w) +
                   col_offset,
               static_cast<std::int16_t>(row_offset),
               static_cast<std::int16_t>(col_offset), n_row, n_col,
               quantize_unit_normal(n_row), quantize_unit_normal(n_col)});
        }
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
        if (simd::cpu_has_avx2()) {
          prepared.packed_points.reserve(prepared.discrete_points.size());
          for (const auto& point : prepared.discrete_points)
            prepared.packed_points.push_back(pack_i8_shape_point(point));
        }
#endif
        // 空间稀疏化后的强点已在前，直接作为锚点，不加入最弱末点，以免纹理背景产生假证据。
        const std::size_t anchor_count =
            std::min<std::size_t>(4, prepared.discrete_points.size());
        prepared.anchor_count = anchor_count;
        for (std::size_t anchor = 0; anchor < anchor_count; ++anchor)
          prepared.anchor_points[anchor] = prepared.discrete_points[anchor];

      }
    }
    return result;
  };
  std::shared_ptr<const std::vector<PreparedShapePose>> prepared_poses;
  if (model.model_cache_enabled() && model.search_cache()) {
    const ShapeSearchPlanKey key{top, unit_gp[top].w, astart, aextent,
                                 smin, smax, scaled, needs_continuous_points, phase};
    auto& cache = *model.search_cache();
    std::lock_guard lock(cache.mutex);
    auto& entry = cache.entries[phase == CoarseAnglePhase::Complementary ? 1 : 0];
    if (!entry.valid || !(entry.key == key)) {
      auto next = build_plan();
      auto& other = cache.entries[phase == CoarseAnglePhase::Complementary ? 0 : 1];
      auto paired_key = key;
      paired_key.phase = other.key.phase;
      if (phase == CoarseAnglePhase::Full || !(paired_key == other.key)) other = {};
      entry.key = key;
      entry.plan = std::move(next);
      entry.valid = true;
    }
    prepared_poses = entry.plan;
  } else {
    prepared_poses = build_plan();
  }
  std::vector<MatchCandidate> cand;
  const auto domain = image.domain();
  const auto domain_bounds = domain.bounding_box();
  const bool rectangular_domain = domain_bounds && domain_bounds->row_min >= 0 &&
      domain_bounds->col_min >= 0 && domain_bounds->row_max < static_cast<std::int64_t>(image.height()) &&
      domain_bounds->col_max < static_cast<std::int64_t>(image.width()) &&
      domain.area() == static_cast<std::uint64_t>(domain_bounds->row_max-domain_bounds->row_min+1) *
                       static_cast<std::uint64_t>(domain_bounds->col_max-domain_bounds->col_min+1);
  const std::int64_t stride = static_cast<std::int64_t>(std::max(1.0, div));
  const bool allow_border = model.border_shape_models();
  const auto coarse_begin = diagnostics ? std::chrono::steady_clock::now()
                                        : std::chrono::steady_clock::time_point{};
  const auto runs = domain.runs();
  const auto& unit_top = unit_gp[top];
  const std::size_t coarse_n = unit_top.w * unit_top.h;
  auto& coarse_qnx = pyramid.coarse_qnx;
  auto& coarse_qny = pyramid.coarse_qny;
  if (coarse_qnx.size() != coarse_n + 16u) {
    coarse_qnx.resize(coarse_n + 16u);
    coarse_qny.resize(coarse_n + 16u);
    quantize_unit_maps(unit_top.nx.data(), unit_top.ny.data(), coarse_qnx.data(),
                       coarse_qny.data(), coarse_n);
  }
  const std::int8_t* qnx = coarse_qnx.data();
  const std::int8_t* qny = coarse_qny.data();
  const bool use_polarity = model.metric() == ShapeMetric::UsePolarity;

#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  const bool have_avx2 = simd::cpu_has_avx2();
#else
  const bool have_avx2 = false;
#endif
  std::pmr::vector<int> coarse_remaining_bounds(memory);
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  simd::I8ScoreSetup coarse_score_setup{};
  if (have_avx2 && !prepared_poses->empty()) {
    coarse_remaining_bounds.resize(prepared_poses->front().discrete_points.size());
    simd::prepare_i8_remaining_bounds(coarse_remaining_bounds.data(),
        coarse_remaining_bounds.size(), greediness, min_score * coarse_score_factor);
    coarse_score_setup = simd::prepare_i8_score_setup(coarse_remaining_bounds.size(),
        greediness, min_score * coarse_score_factor);
  }
#endif
  const bool legacy_scan = mv::runtime::getenv_or_null("MVKIT_SHAPE_LEGACY_SCAN") != nullptr;
  std::span<const std::int8_t> coarse_query_pairs;
  if (have_avx2 && !legacy_scan && max_deformation == 0 && !allow_border &&
      static_cast<double>(domain.area()) / (div * div) >= 128) {
    // 每查询交错配对一次，避免每候选每模板点反复读两平面再 unpack；
    // 七对补零使掩码 AVX2 尾部即使末模型点到最终像素，也可读八中心。
    if (pyramid.coarse_query_pairs.empty()) {
      pyramid.coarse_query_pairs.resize(2 * (coarse_n + 7));
      for (std::size_t i = 0; i < coarse_n; ++i) {
        pyramid.coarse_query_pairs[2 * i] = qny[i];
        pyramid.coarse_query_pairs[2 * i + 1] = qnx[i];
      }
    }
    coarse_query_pairs = pyramid.coarse_query_pairs;
  }
  // 足够内部工作量可摊销时才建立查询索引；几何控制分派，不按图像名称或预期结果选择。
  const char* direction_policy = mv::runtime::getenv_or_null("MVKIT_SHAPE_DIRECTION_INDEX");
  // 增量 Map 在较小搜索量即可摊销，优于旧逐法向独立位图写入。
  constexpr double direction_index_min_work = 300000.0;
  const auto possible_work = static_cast<double>(coarse_n) * prepared_poses->size();
  const bool consider_direction_index = !coarse_query_pairs.empty() &&
      (direction_policy ? std::string_view(direction_policy) == "1" : possible_work >= direction_index_min_work);
  double interior_work = 0;
  std::size_t domain_centers = 0;
  bool compatible_anchors = true;
  if (consider_direction_index) {
    for (const auto& run : runs) {
      if (run.row < 0 || static_cast<std::size_t>(run.row) >= image.height() || run.row % stride) continue;
      const auto first = (std::int64_t(run.col_begin) + stride - 1) / stride;
      const auto last = std::int64_t(run.col_end) / stride;
      if (last >= first) domain_centers += static_cast<std::size_t>(last - first + 1);
    }
    for (const auto& pose : *prepared_poses) {
      compatible_anchors &= pose.anchor_count == 4 && pose.packed_points.size() >= 4;
      interior_work += std::max(0.0, static_cast<double>(unit_top.w) - (pose.max_col - pose.min_col)) *
                       std::max(0.0, static_cast<double>(unit_top.h) - (pose.max_row - pose.min_row));
    }
  }
  const auto estimated_work = std::min(interior_work,
      static_cast<double>(domain_centers) * prepared_poses->size());
  bool use_direction_index = consider_direction_index && compatible_anchors &&
      (direction_policy ? std::string_view(direction_policy) == "1" :
       estimated_work >= direction_index_min_work && interior_work >= 0.5 * possible_work);
  const bool verify_direction_index = mv::runtime::getenv_or_null("MVKIT_SHAPE_VERIFY_DIRECTION_INDEX") != nullptr;
  DirectionEvidence direction_index(memory);
  std::pmr::vector<std::array<std::size_t, 4>> direction_ids(memory);
  std::pmr::vector<std::array<std::size_t, 2>> direction_batches(memory);
  if (use_direction_index) {
    const auto index_begin = std::chrono::steady_clock::now();
    std::pmr::vector<std::uint32_t> normals(memory);
    direction_ids.resize(prepared_poses->size());
    if (diagnostics) direction_batches.resize(prepared_poses->size());
    for (std::size_t p = 0; p < prepared_poses->size(); ++p) {
      const auto& pose = (*prepared_poses)[p];
      for (std::size_t a = 0; a < pose.anchor_count; ++a) {
        const auto normal = pose.packed_points[a].normal_pair;
        const auto found = std::find(normals.begin(), normals.end(), normal);
        const auto id = static_cast<std::size_t>(found - normals.begin());
        if (found == normals.end()) normals.push_back(normal);
        direction_ids[p][a] = id;
      }
    }
    const auto words = (coarse_n + 63) / 64 + 1;
    DirectionEvidence::BuildMaps build_maps = nullptr;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (have_avx2) build_maps = simd::build_direction_maps_avx2;
#endif
    // 每查询严格 Map 预算，强制诊断策略也受限制。
    if (normals.size() > (32u * 1024u * 1024u / sizeof(std::uint64_t)) / words)
      use_direction_index = false;
    else
      direction_index.build(coarse_qnx, coarse_qny, normals, kEvidenceDotThr, use_polarity, build_maps,
          [](const void* context) { static_cast<const TimeoutWatch*>(context)->poll(); }, &timeout);
    if (diagnostics) std::fprintf(stderr,
        "mvkit_direction_index build_us=%.3f normals=%zu anchors=%zu pixels=%zu bytes=%zu\n",
        elapsed_us(index_begin, std::chrono::steady_clock::now()), normals.size(),
        prepared_poses->size() * 4, coarse_n, normals.size() * ((coarse_n + 63) / 64 + 1) * 8);
  }
  // 查询相关位姿不变量不进入持久模型缓存。
  struct PoseScanSetup {
    std::int64_t minimum_column{}, maximum_column{};
    int min_anchor_hits{};
    std::ptrdiff_t anchor_offsets[4]{};
    std::int8_t anchor_qn_row[4]{}, anchor_qn_col[4]{};
  };
  auto process_run = [&](std::size_t run_index,
                         std::vector<MatchCandidate>& output,
                         std::size_t& run_evaluated,
                         std::size_t& run_accepted,
                         std::size_t& run_scored,
                         std::size_t pose_begin, std::size_t pose_end,
                         const PoseScanSetup& setup) {
    const auto& run = runs[run_index];
    std::size_t run_polls = 0;
    // 只有 coarse_runs 已验证并对齐的行进入此 worker。
    const double coarse_row = static_cast<double>(run.row) / div;
    const auto& unit = unit_gp[top];
    for (std::size_t pose_index = pose_begin; pose_index < pose_end;
         ++pose_index) {
      const auto& prepared = (*prepared_poses)[pose_index];
      const bool row_inside =
          max_deformation == 0 &&
          coarse_row + prepared.min_row >= 0.0 &&
          coarse_row + prepared.max_row <=
              static_cast<double>(unit.h - 1);
      if (max_deformation == 0 && !allow_border && !row_inside)
        continue;
      const int min_anchor_hits = setup.min_anchor_hits;
      const auto anchors = std::span(prepared.anchor_points.data(),
                                     prepared.anchor_count);
      const auto* anchor_offsets = setup.anchor_offsets;
      const auto* anchor_qn_row = setup.anchor_qn_row;
      const auto* anchor_qn_col = setup.anchor_qn_col;
      auto has_anchor_evidence = [&](std::size_t candidate_center) {
        const auto base = static_cast<std::ptrdiff_t>(candidate_center);
        int hits = 0;
        for (std::size_t a = 0; a < prepared.anchor_count; ++a) {
          const auto& anchor = prepared.anchor_points[a];
          const auto index =
              static_cast<std::size_t>(base + anchor.image_offset);
          if (i8_dot(qny[index], qnx[index], anchor.qn_row, anchor.qn_col,
                     use_polarity) > kEvidenceDotThr) {
            ++hits;
            if (hits >= min_anchor_hits) return true;
          }
        }
        return false;
      };
      const double reject_below = min_score * coarse_score_factor;
      auto emit_if_accepted = [&](std::int64_t full_column, double score) {
        if (score < reject_below) return;
        if (diagnostics) ++run_accepted;
        output.push_back({static_cast<double>(run.row),
                          static_cast<double>(full_column), prepared.angle,
                          prepared.scale, score});
      };
      auto score_one = [&](std::int64_t full_column,
                           std::size_t discrete_center, bool interior) {
        if (interior)
          return score_discrete_i8_points(
              qnx, qny, model, prepared.discrete_points, discrete_center,
              greediness, reject_below);
        return score_rotated_unit_points(
            unit, model, prepared.points, model.levels()[top].radius, div,
            static_cast<double>(run.row), static_cast<double>(full_column),
            1.0, 0.0, prepared.scale, greediness, reject_below,
            max_deformation);
      };
      auto score_live_interiors = [&](std::size_t index0, int lanes,
                                      int evidence, double* scores) {
        score_discrete_live_interiors(
            qnx, qny, model, prepared.discrete_points, index0, lanes, evidence,
            greediness, reject_below, scores,
            coarse_remaining_bounds.size() == prepared.discrete_points.size()
                ? coarse_remaining_bounds.data() : nullptr,
            prepared.packed_points.empty() ? nullptr : prepared.packed_points.data());
      };
      std::int64_t column_begin = run.col_begin;
      std::int64_t column_end = run.col_end;
      const bool clipped_interior = max_deformation == 0 && !allow_border;
      if (clipped_interior) {
        column_begin = std::max(column_begin, setup.minimum_column);
        column_end = std::min(column_end, setup.maximum_column);
      }
      // 行已位于全局金字塔晶格，列也须同样对齐，否则缩减域游程所有中心都走标量连续坐标路径。
      const auto remainder = column_begin % stride;
      if (remainder != 0)
        column_begin += remainder > 0 ? stride - remainder : -remainder;
      if (column_begin > column_end) continue;
      constexpr bool integral_center = true;
      const auto center_row = run.row / stride;
      auto center_col = column_begin / stride;
      const double max_center_col = static_cast<double>(unit.w - 1);
      for (std::int64_t c = column_begin; c <= column_end;) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
        if (!legacy_scan && have_avx2 && clipped_interior && prepared.anchor_count == 4 &&
            prepared.packed_points.size() >= 4 &&
            coarse_remaining_bounds.size() == prepared.packed_points.size()) {
          const auto available = static_cast<std::size_t>((column_end - c) / stride + 1);
          // 保留最终标量 lane 的 Double 阈值/归一化，仅原 2/4 lane SIMD 尾部并入 AVX2 游程。
          const auto centers = std::min<std::size_t>(64, available) & ~std::size_t{1};
          if (centers != 0) {
            timeout.poll();
            std::uint64_t evidence_bits = 0;
            if (use_direction_index) {
              const auto base = static_cast<std::ptrdiff_t>(center_row * unit.w + center_col);
              std::array<std::uint64_t, 4> bits;
              for (std::size_t a = 0; a < 4; ++a)
                bits[a] = direction_index.window(direction_ids[pose_index][a],
                    static_cast<std::size_t>(base + prepared.packed_points[a].image_offset));
              evidence_bits = DirectionEvidence::combine(bits, min_anchor_hits);
              if (centers < 64) evidence_bits &= (std::uint64_t{1} << centers) - 1;
              if (verify_direction_index) {
                for (std::size_t lane = 0; lane < centers; ++lane)
                  if (has_anchor_evidence(static_cast<std::size_t>(base) + lane) !=
                      ((evidence_bits >> lane) & 1))
                    throw std::logic_error("Direction evidence differs from anchor predicate");
              }
              if (diagnostics) {
                direction_batches[pose_index][0] += (centers + 7) / 8;
                for (auto remaining = evidence_bits; remaining;) {
                  const auto group = std::countr_zero(remaining) & ~7;
                  remaining &= ~(std::uint64_t{255} << group);
                  ++direction_batches[pose_index][1];
                }
              }
              if (!evidence_bits && reject_below > 0) {
                if (diagnostics) run_evaluated += centers;
                c += static_cast<std::int64_t>(centers) * stride;
                center_col += static_cast<std::int64_t>(centers);
                continue;
              }
            }
            double scores[64];
            std::uint64_t accepted_mask = 0;
            const auto scored = simd::scan_i8_run_avx2(qnx, qny,
                prepared.packed_points.data(), prepared.packed_points.size(),
                static_cast<std::size_t>(center_row) * unit.w + static_cast<std::size_t>(center_col),
                centers, min_anchor_hits, kEvidenceDotThr, use_polarity,
                model.metric() == ShapeMetric::IgnoreLocalPolarity,
                model.metric() == ShapeMetric::IgnoreGlobalPolarity,
                greediness, reject_below, coarse_remaining_bounds.data(), scores, &accepted_mask,
                coarse_query_pairs.empty() ? nullptr : coarse_query_pairs.data(),
                use_direction_index ? &evidence_bits : nullptr, &coarse_score_setup);
            if (diagnostics) { run_evaluated += centers; run_scored += scored; }
            while (accepted_mask) {
              const auto lane = std::countr_zero(accepted_mask);
              emit_if_accepted(c + static_cast<std::int64_t>(lane) * stride, scores[lane]);
              accepted_mask &= accepted_mask - 1;
            }
            c += static_cast<std::int64_t>(centers) * stride;
            center_col += static_cast<std::int64_t>(centers);
            continue;
          }
        }
#endif
        const bool has_eighth =
            have_avx2 && integral_center && c + 7 * stride <= column_end;
        const bool has_fourth = integral_center && c + 3 * stride <= column_end;
        const bool has_second = c + stride <= column_end;
        const auto center_index =
            integral_center
                ? static_cast<std::size_t>(center_row) * unit.w +
                      static_cast<std::size_t>(center_col)
                : 0u;
        auto center_inside = [&](double extra_col) {
          const double first_center_col = static_cast<double>(c) / div;
          return row_inside &&
                 first_center_col + extra_col + prepared.min_col >= 0.0 &&
                 first_center_col + extra_col + prepared.max_col <=
                     max_center_col;
        };
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
        const bool pack8 =
            has_eighth && (clipped_interior || (center_inside(0.0) && center_inside(7.0)));
        if (pack8) {
          run_polls += 8u;
          if ((run_polls & 63) == 0) timeout.poll();
          if (diagnostics) run_evaluated += 8u;
          const int evidence =
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
              simd::avx2_kernels().i8_group_evidence_mask(
                  qnx, qny, anchor_offsets, anchor_qn_row, anchor_qn_col,
                  prepared.anchor_count, center_index, min_anchor_hits,
                  kEvidenceDotThr, use_polarity, 8);
#else
              i8_group_evidence_mask_offsets(
                  qnx, qny, anchor_offsets, anchor_qn_row, anchor_qn_col,
                  prepared.anchor_count, center_index, 8, min_anchor_hits,
                  kEvidenceDotThr, use_polarity);
#endif
          double pack_scores8[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          if (evidence != 0) {
            if (diagnostics)
              run_scored +=
                  static_cast<std::size_t>(live_lane_count(evidence, 8));
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
            simd::avx2_zeroupper();
#endif
            score_live_interiors(center_index, 8, evidence, pack_scores8);
          }
          for (int lane = 0; lane < 8; ++lane)
            emit_if_accepted(c + lane * stride, pack_scores8[lane]);
          c += 8 * stride;
          center_col += 8;
          continue;
        }
#endif
        const bool pack4 = has_fourth && (clipped_interior ||
                           (center_inside(0.0) && center_inside(3.0)));
        if (pack4) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
          // VZEROUPPER 本身要求 AVX；编译了 AVX2 翻译单元不代表宿主或所选策略可执行。
          if (have_avx2) simd::avx2_zeroupper();
#endif
          run_polls += 4u;
          if ((run_polls & 63) == 0) timeout.poll();
          if (diagnostics) run_evaluated += 4u;
          int evidence = i8_group_evidence_mask<4>(
              qnx, qny, anchors, center_index, min_anchor_hits,
              kEvidenceDotThr, use_polarity);
          double pack_scores[4] = {0.0, 0.0, 0.0, 0.0};
          if (evidence != 0) {
            if (diagnostics)
              run_scored += static_cast<std::size_t>(
                  live_lane_count(evidence, 4));
            score_live_interiors(center_index, 4, evidence, pack_scores);
          }
          for (int lane = 0; lane < 4; ++lane)
            emit_if_accepted(c + lane * stride, pack_scores[lane]);
          c += 4 * stride;
          center_col += 4;
          continue;
        }
        run_polls += has_second ? 2u : 1u;
        if ((run_polls & 63) == 0) timeout.poll();
        if (diagnostics)
          run_evaluated += has_second ? 2u : 1u;
        const bool first_inside = clipped_interior || center_inside(0.0);
        const bool second_inside = has_second && (clipped_interior || center_inside(1.0));
        const bool first_interior = integral_center && first_inside;
        const bool second_interior =
            has_second && integral_center && second_inside;
        const bool first_rejected =
            max_deformation == 0 && !allow_border && !first_inside;
        const bool second_rejected =
            has_second && max_deformation == 0 && !allow_border &&
            !second_inside;
        double pair_scores[2] = {0.0, 0.0};
        const int pair_evidence =
            first_interior && second_interior
                ? [&] {
                    return i8_group_evidence_mask<2>(
                        qnx, qny, anchors, center_index, min_anchor_hits,
                        kEvidenceDotThr, use_polarity);
                  }()
                : 0;
        const bool first_has_evidence =
            !first_rejected &&
            (!first_interior ||
             (second_interior ? (pair_evidence & 1) != 0
                              : has_anchor_evidence(center_index)));
        const bool second_has_evidence =
            !second_rejected &&
            (!second_interior || (pair_evidence & 2) != 0);
        if (first_has_evidence || second_has_evidence) {
          if (diagnostics) {
            if (first_has_evidence) ++run_scored;
            if (has_second && second_has_evidence) ++run_scored;
          }
          if (first_interior && second_interior) {
            const int evidence =
                (first_has_evidence ? 1 : 0) | (second_has_evidence ? 2 : 0);
            score_live_interiors(center_index, 2, evidence, pair_scores);
          } else {
            if (first_has_evidence)
              pair_scores[0] =
                  score_one(c, center_index, first_interior);
            if (has_second && second_has_evidence)
              pair_scores[1] = score_one(c + stride, center_index + 1,
                                         second_interior);
          }
        }
        emit_if_accepted(c, pair_scores[0]);
        if (has_second)
          emit_if_accepted(c + stride, pair_scores[1]);
        c += has_second ? 2 * stride : stride;
        center_col += has_second ? 2 : 1;
      }
    }
  };
  // 两个执行器都按位姿分区，各位姿独占结果，合并序不受 worker 数影响；
  // 不能只为串行路径丢弃 2x2 块方向或收紧锚点要求。
  std::vector<std::size_t> coarse_runs;
  for (std::size_t i = 0; i < runs.size(); ++i) {
    if (runs[i].row >= 0 && static_cast<std::size_t>(runs[i].row) < image.height()
        && runs[i].row % stride == 0) coarse_runs.push_back(i);
  }
  struct PoseResults {
    std::vector<MatchCandidate> candidates;
    std::size_t evaluated{}, accepted{}, scored{};
  };
  std::vector<PoseResults> results(prepared_poses->size());
  auto search_poses = [&](mv::runtime::IndexRange range) {
    for (std::size_t pose = range.begin; pose < range.end; ++pose) {
      auto& result = results[pose];
      const auto& prepared = (*prepared_poses)[pose];
      PoseScanSetup setup;
      setup.min_anchor_hits = min_anchor_hits_for_search(
          min_score, prepared.anchor_count, prepared.discrete_points.size());
      if (max_deformation == 0 && !allow_border) {
        setup.minimum_column = static_cast<std::int64_t>(std::ceil(-prepared.min_col * div));
        setup.maximum_column = static_cast<std::int64_t>(std::floor(
            (static_cast<double>(unit_top.w - 1) - prepared.max_col) * div));
      }
      for (std::size_t a = 0; a < prepared.anchor_count; ++a) {
        setup.anchor_offsets[a] = prepared.anchor_points[a].image_offset;
        setup.anchor_qn_row[a] = prepared.anchor_points[a].qn_row;
        setup.anchor_qn_col[a] = prepared.anchor_points[a].qn_col;
      }
      for (auto row : coarse_runs)
        process_run(row, result.candidates, result.evaluated, result.accepted,
                    result.scored, pose, pose + 1, setup);
    }
  };
  if (image.width() * image.height() >= mv::imageproc::ExecutionContext::kFilterParallelWork)
    mv::runtime::parallel_for(0, results.size(), 1, search_poses);
  else
    search_poses({0, results.size()});
  std::pmr::vector<double> peak_scores(memory);
  if(local_maxima)peak_scores.resize(coarse_n);
  for (auto& result : results) {
    if(local_maxima && !result.candidates.empty()) {
      std::fill(peak_scores.begin(),peak_scores.end(),-std::numeric_limits<double>::infinity());
      for(const auto& p:result.candidates)peak_scores[std::size_t(p.row/div)*unit_top.w+std::size_t(p.column/div)]=p.score;
      result.candidates.erase(std::remove_if(result.candidates.begin(),result.candidates.end(),[&](const auto& p){
        const auto r=std::int64_t(p.row/div),c=std::int64_t(p.column/div);
        for(int dr=-1;dr<=1;++dr)for(int dc=-1;dc<=1;++dc) {
          if((!dr && !dc) || r+dr<0 || c+dc<0 || r+dr>=std::int64_t(unit_top.h) || c+dc>=std::int64_t(unit_top.w))continue;
          const auto other=peak_scores[std::size_t(r+dr)*unit_top.w+std::size_t(c+dc)];
          if(other>p.score || (other==p.score && (dr<0 || (!dr && dc<0))))return true;
        }
        return false;
      }),result.candidates.end());
    }
    cand.insert(cand.end(), std::make_move_iterator(result.candidates.begin()),
                std::make_move_iterator(result.candidates.end()));
    if (diagnostics) {
      diagnostics->coarse_evaluated += result.evaluated;
      diagnostics->coarse_accepted += result.accepted;
      diagnostics->coarse_scored += result.scored;
    }
  }
  if (diagnostics && use_direction_index) {
    std::size_t total = 0, live = 0;
    for (const auto& batches : direction_batches) { total += batches[0]; live += batches[1]; }
    std::fprintf(stderr, "mvkit_direction_batches total=%zu live=%zu skipped=%zu\n", total, live, total - live);
  }
  if (diagnostics)
    diagnostics->coarse_us +=
        elapsed_us(coarse_begin, std::chrono::steady_clock::now());
  const auto better = [](const auto& a, const auto& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.row != b.row) return a.row < b.row;
    if (a.column != b.column) return a.column < b.column;
    if (a.angle != b.angle) return a.angle < b.angle;
    return a.scale < b.scale;
  };
  if (cand.size() > candidate_limit) {
    std::nth_element(cand.begin(), cand.begin() + static_cast<std::ptrdiff_t>(candidate_limit), cand.end(), better);
    cand.resize(candidate_limit);
  }
  std::sort(cand.begin(), cand.end(), better);
  const auto refine_begin = diagnostics ? std::chrono::steady_clock::now()
                                        : std::chrono::steady_clock::time_point{};
  for (std::size_t l = top; l > levels.finest; --l) {
    if (cand.empty()) break;
    timeout.poll();
    const auto prepare_begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    const std::size_t finer = l - 1;
    if (finer == levels.finest)
      ensure_finest_u8_components(image, gp[finer], model, cand,
                                  max_deformation);
    else {
      const auto& ml = model.levels()[finer];
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if (have_avx2 && max_deformation == 0 &&

          !model.border_shape_models()) {
        ensure_i16_from_gray(gp[finer], pyramid.gray_levels[finer],
                             ml.pyramid_scale, ml.radius, cand,
                             max_deformation);
      }
#endif
      if (!gp[finer].has_integer_components()) {
        ensure_unit_for_candidates(
            unit_gp[finer], pyramid.gray_levels[finer],
            model.min_contrast() / std::max(1.0, ml.pyramid_scale),
            ml.pyramid_scale, ml.radius, cand, max_deformation);
      }
    }
    const auto generate_begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    if (diagnostics) diagnostics->refine_prepare_us += elapsed_us(prepare_begin, generate_begin);
    const double step = model.levels()[l].pyramid_scale;
    // 角度/尺度晶格键仍在生成集合中，有序 trial 缓冲仅需空间键。
    struct RefinementTrial {
      MatchCandidate candidate;
      std::int64_t row_key{};
      std::int64_t column_key{};
    };
    struct TrialKey {
      std::int64_t row;
      std::int64_t column;
      std::int64_t angle;
      std::int64_t scale;
    };
    const std::size_t trial_multiplier = scaled ? 81u : 27u;
    const std::size_t maximum_trials = cand.size() * trial_multiplier;
    std::vector<RefinementTrial> trials;
    trials.reserve(maximum_trials);
    // 相邻父项常产生重复晶格试探，生成时用紧凑开放寻址集合，
    // 不物化全部重复后排序完整 candidate_count*27/81 数组。
    std::size_t key_capacity = 1;
    while (key_capacity < std::max<std::size_t>(8, maximum_trials * 2u))
      key_capacity <<= 1;
    // 仅读取占用槽，不清零数 MiB 键数组，其生命周期复用查询 scratch。
    PmrBuffer<TrialKey> trial_keys(key_capacity, memory);
    std::pmr::vector<std::uint8_t> trial_key_used(key_capacity, 0u, memory);
    const auto mix_key = [](std::uint64_t value) noexcept {
      value ^= value >> 30;
      value *= UINT64_C(0xbf58476d1ce4e5b9);
      value ^= value >> 27;
      value *= UINT64_C(0x94d049bb133111eb);
      return value ^ (value >> 31);
    };
    const auto trial_slot = [&](const TrialKey& key) {
      std::uint64_t hash = mix_key(static_cast<std::uint64_t>(key.row));
      hash ^= mix_key(static_cast<std::uint64_t>(key.column) + UINT64_C(0x9e3779b97f4a7c15));
      hash ^= mix_key(static_cast<std::uint64_t>(key.angle) + UINT64_C(0x3c6ef372fe94f82a));
      hash ^= mix_key(static_cast<std::uint64_t>(key.scale) + UINT64_C(0xdaa66d2c7ddef743));
      std::size_t slot = static_cast<std::size_t>(hash) & (key_capacity - 1u);
      for (;;) {
        if (trial_key_used[slot] == 0u) {
          return slot;
        }
        const auto& existing = trial_keys[slot];
        if (existing.row == key.row && existing.column == key.column &&
            existing.angle == key.angle && existing.scale == key.scale)
          return slot;
        slot = (slot + 1u) & (key_capacity - 1u);
      }
    };
    const auto insert_trial_key = [&](const TrialKey& key) {
      const auto slot=trial_slot(key);if(trial_key_used[slot])return false;
      trial_key_used[slot]=1;trial_keys[slot]=key;return true;
    };
    const double angle_quantum = model.angle_step() * step;
    const double scale_quantum = model.scale_step() * step;
    for (const auto& base : cand) {
      std::array<double,3> rows, columns, angles, scales;
      std::array<std::int64_t,3> row_keys, column_keys, angle_keys, scale_keys, pixel_rows, pixel_columns;
      std::array<bool,3> angle_valid, scale_valid;
      // 归属与晶格舍入只依赖单轴，不依赖全部 27/81 组合；
      // 每父项各值只算一次，重放原试探序及精确键。
      for (int offset=-1; offset<=1; ++offset) {
        const auto k=static_cast<std::size_t>(offset+1);
        rows[k]=base.row+offset*step; columns[k]=base.column+offset*step;
        row_keys[k]=static_cast<std::int64_t>(std::llround(rows[k]/step));
        column_keys[k]=static_cast<std::int64_t>(std::llround(columns[k]/step));
        pixel_rows[k]=static_cast<std::int64_t>(std::llround(rows[k]));
        pixel_columns[k]=static_cast<std::int64_t>(std::llround(columns[k]));
        angles[k]=base.angle+offset*angle_quantum;
        scales[k]=base.scale+offset*scale_quantum;
        angle_valid[k]=angles[k]>=astart-1e-9 && angles[k]<=astart+aextent+1e-9;
        scale_valid[k]=scales[k]>=smin-1e-9 && scales[k]<=smax+1e-9;
        if (angle_valid[k]) angle_keys[k]=static_cast<std::int64_t>(std::llround((angles[k]-astart)/angle_quantum));
        if (scaled && scale_valid[k]) scale_keys[k]=static_cast<std::int64_t>(std::llround((scales[k]-smin)/scale_quantum));
      }
      for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
          const auto r=static_cast<std::size_t>(dr+1), c=static_cast<std::size_t>(dc+1);
          const bool inside = rectangular_domain
              ? pixel_rows[r]>=domain_bounds->row_min && pixel_rows[r]<=domain_bounds->row_max &&
                pixel_columns[c]>=domain_bounds->col_min && pixel_columns[c]<=domain_bounds->col_max
              : domain.contains(pixel_rows[r],pixel_columns[c]);
          if (!inside) continue;
          for (int da = -1; da <= 1; ++da)
            for (int ds = scaled ? -1 : 0; ds <= (scaled ? 1 : 0); ++ds) {
              const auto a=static_cast<std::size_t>(da+1), s=static_cast<std::size_t>(ds+1);
              if (!angle_valid[a] || !scale_valid[s]) continue;
              MatchCandidate x = base;
              x.row=rows[r]; x.column=columns[c]; x.angle=angles[a]; x.scale=scales[s];
              const TrialKey key{row_keys[r],column_keys[c],angle_keys[a],scaled?scale_keys[s]:0};
              if (insert_trial_key(key))
                trials.push_back({x, key.row, key.column});
            }
        }
    }
    std::vector<MatchCandidate> next;
    next.reserve(trials.size());
    const auto& ml = model.levels()[finer];
    std::shared_ptr<const PreparedPointSoA> point_soa;
    std::span<const float> prow,pcol,pnr,pnc;
    const bool avx_score =
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
        have_avx2 && max_deformation == 0 &&

        !model.border_shape_models();
#else
        false;
#endif
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    // 局部候选/基线评分不消费整图缓存，仅整数 AVX2 路径可用时准备。
    if (avx_score && gp[finer].has_integer_components())
      maybe_prepare_gradient_caches(gp[finer], trials.size() * ml.points.size());
#endif
    if (avx_score) {
      point_soa=prepared_point_soa(model,finer);
      prow=point_soa->row;pcol=point_soa->column;pnr=point_soa->nr;pnc=point_soa->nc;
      // 单刚性方向时全部 trial 已属一组，平移晶格排序无评分用途。
      if((aextent!=0 || scaled) &&
         (!radix_trial_sort || !detail::try_radix_sort_trials(trials, memory, radix_trial_sort == 2)))
        std::sort(trials.begin(), trials.end(),
                [](const RefinementTrial& a, const RefinementTrial& b) {
                  if (a.candidate.angle != b.candidate.angle)
                    return a.candidate.angle < b.candidate.angle;
                  if (a.candidate.scale != b.candidate.scale)
                    return a.candidate.scale < b.candidate.scale;
                  if (a.row_key != b.row_key) return a.row_key < b.row_key;
                  return a.column_key < b.column_key;
                });
    }
    const auto score_begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    if (diagnostics) diagnostics->refine_generate_us += elapsed_us(generate_begin, score_begin);
    const double reject_refine = min_score * refinement_score_factor;
    const bool finest_level = finer == levels.finest;
    const bool i16_score = gp[finer].has_integer_components();
    const double min_mag =
        model.min_contrast() / std::max(1.0, ml.pyramid_scale);
    const float min_mag_sq = static_cast<float>(min_mag * min_mag);
    auto score_trial = [&](RefinementTrial& trial, bool use_pre_rotated,
                           const float* pre_off_r, const float* pre_off_c,
                           const float* pre_rot_nr, const float* pre_rot_nc,
                           float pose_cosine,float pose_sine) {
      auto& x = trial.candidate;
      bool scored = false;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if (avx_score) {
        const double level_row = x.row / ml.pyramid_scale;
        const double level_col = x.column / ml.pyramid_scale;
        const double margin = std::abs(x.scale) * ml.radius + 1.0;
        const double level_h = static_cast<double>(
            i16_score ? gp[finer].h : unit_gp[finer].h);
        const double level_w = static_cast<double>(
            i16_score ? gp[finer].w : unit_gp[finer].w);
        const bool interior = level_row >= margin && level_col >= margin &&
                              level_row + margin < level_h &&
                              level_col + margin < level_w;
        if (interior) {
          if (use_pre_rotated) {
            if (i16_score) {
              x.score = score_i16_pre_rotated_components(gp[finer],
                  min_mag_sq, pre_off_r, pre_off_c, pre_rot_nr,
                  pre_rot_nc, prow.size(), static_cast<float>(level_row),
                  static_cast<float>(level_col),
                  static_cast<float>(greediness),
                  static_cast<float>(reject_refine), static_cast<int>(model.metric()));
              scored = true;
            } else if (!finest_level) {
              x.score = simd::avx2_kernels().score_unit_pre_rotated_use_polarity(
                  unit_gp[finer].nx.data(), unit_gp[finer].ny.data(),
                  unit_gp[finer].w, pre_off_r, pre_off_c, pre_rot_nr,
                  pre_rot_nc, prow.size(), static_cast<float>(level_row),
                  static_cast<float>(level_col),
                  static_cast<float>(greediness),
                  static_cast<float>(reject_refine), static_cast<int>(model.metric()));
              scored = true;
            }
          } else {
            const auto ca = pose_cosine;
            const auto sa = pose_sine;
            if (i16_score) {
              x.score = score_i16_nearest_components(gp[finer],
                  min_mag_sq, prow.data(), pcol.data(), pnr.data(),
                  pnc.data(), prow.size(), static_cast<float>(level_row),
                  static_cast<float>(level_col), ca, sa,
                  static_cast<float>(x.scale), static_cast<float>(greediness),
                  static_cast<float>(reject_refine), static_cast<int>(model.metric()));
              scored = true;
            } else if (!finest_level) {
              x.score = simd::avx2_kernels().score_unit_use_polarity(
                  unit_gp[finer].nx.data(), unit_gp[finer].ny.data(),
                  unit_gp[finer].w, prow.data(), pcol.data(), pnr.data(),
                  pnc.data(), prow.size(), static_cast<float>(level_row),
                  static_cast<float>(level_col), ca, sa,
                  static_cast<float>(x.scale), static_cast<float>(greediness),
                  static_cast<float>(reject_refine), static_cast<int>(model.metric()));
              scored = true;
            }
          }
        }
      }
#endif
      if (!scored) {
        if (finest_level || i16_score) {
          x.score = score_pose_level(
              gp[finer], model, finer, x.row, x.column, x.angle, x.scale,
              greediness, reject_refine, max_deformation, false);
        } else {
          x.score = score_unit_points(
              unit_gp[finer], model, ml.points, ml.radius, ml.pyramid_scale,
              x.row, x.column, x.angle, x.scale, greediness, reject_refine,
              max_deformation);
        }
      }
      if (finest_level && model.is_xld_model() && x.score >= reject_refine) {
        if (avx_score && i16_score && x.row >= ml.radius * std::abs(x.scale) + 1 &&
            x.column >= ml.radius * std::abs(x.scale) + 1 &&
            x.row + ml.radius * std::abs(x.scale) + 1 < gp[finer].h &&
            x.column + ml.radius * std::abs(x.scale) + 1 < gp[finer].w && ml.pyramid_scale == 1) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
          x.score = score_i16_bilinear_components(gp[finer], min_mag_sq,
              prow.data(), pcol.data(), pnr.data(), pnc.data(), prow.size(),
              static_cast<float>(x.row), static_cast<float>(x.column),
              pose_cosine,pose_sine,
              static_cast<float>(x.scale), 0.0F,
              static_cast<float>(min_score), static_cast<int>(model.metric()));
#endif
        } else {
          x.score = score_pose_level(gp[finer], model, finer, x.row, x.column, x.angle,
              x.scale, 0, min_score, max_deformation, true);
        }
        return x.score >= min_score;
      }
      return x.score >= reject_refine;
    };
    struct RefinementGroup {
      std::size_t begin{};
      std::size_t end{};
      bool pre_rotate{};
      float cosine{},sine{};
      std::vector<MatchCandidate> accepted;
    };
    std::vector<RefinementGroup> groups;
    if (avx_score) {
      for (std::size_t begin = 0; begin < trials.size();) {
        std::size_t end = begin + 1;
        while (end < trials.size() &&
               trials[end].candidate.angle == trials[begin].candidate.angle &&
               trials[end].candidate.scale == trials[begin].candidate.scale)
          ++end;
        groups.push_back({begin, end, end - begin >= 2,
            static_cast<float>(std::cos(trials[begin].candidate.angle)),
            static_cast<float>(std::sin(trials[begin].candidate.angle)),{}});
        begin = end;
      }
    } else {
      constexpr std::size_t trials_per_group = 64;
      for (std::size_t begin = 0; begin < trials.size(); begin += trials_per_group)
        groups.push_back(
            {begin, std::min(trials.size(), begin + trials_per_group), false, 0,0,{}});
    }
    auto score_groups = [&](mv::runtime::IndexRange range) {
      std::vector<float> off_r;
      std::vector<float> off_c;
      std::vector<float> rot_nr;
      std::vector<float> rot_nc;
      std::array<float,64> stack_r,stack_c,stack_nr,stack_nc;
      if (avx_score && prow.size()>64) {
        off_r.resize(prow.size());
        off_c.resize(prow.size());
        rot_nr.resize(prow.size());
        rot_nc.resize(prow.size());
      }
      auto* r=prow.size()<=64?stack_r.data():off_r.data();
      auto* c=prow.size()<=64?stack_c.data():off_c.data();
      auto* nr=prow.size()<=64?stack_nr.data():rot_nr.data();
      auto* nc=prow.size()<=64?stack_nc.data():rot_nc.data();
      for (std::size_t group_index = range.begin; group_index < range.end;
           ++group_index) {
        auto& group = groups[group_index];
        group.accepted.reserve(group.end - group.begin);
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
        if (group.pre_rotate) {
          const auto& x0 = trials[group.begin].candidate;
          simd::avx2_kernels().rotate_model_points(
              prow.data(), pcol.data(), pnr.data(), pnc.data(), prow.size(),
              group.cosine,group.sine,static_cast<float>(x0.scale),r,c,nr,nc);
        }
#endif
        for (std::size_t k = group.begin; k < group.end; ++k) {
          if (score_trial(trials[k], group.pre_rotate, r,c,nr,nc,group.cosine,group.sine))
            group.accepted.push_back(trials[k].candidate);
        }
      }
    };
    const std::size_t refinement_work = trials.size() * ml.points.size();
    // 候选逐项评分比图像滤波昂贵，采用较低工作量门限，让中等候选束受益而不调度微小细化集。
    constexpr std::size_t refinement_parallel_work = 128u * 1024u;
    if (groups.size() > 1 && refinement_work >= refinement_parallel_work)
      mv::runtime::parallel_for(0, groups.size(), 1, score_groups);
    else
      score_groups({0, groups.size()});
    for (auto& group : groups) {
      if (diagnostics) {
        diagnostics->refine_evaluated += group.end - group.begin;
        diagnostics->refine_accepted += group.accepted.size();
      }
      next.insert(next.end(), std::make_move_iterator(group.accepted.begin()),
                  std::make_move_iterator(group.accepted.end()));
    }
    if(local_maxima && finer>levels.finest) {
      PmrBuffer<double> lattice_scores(key_capacity,memory);
      for(std::size_t i=0;i<key_capacity;++i)if(trial_key_used[i])lattice_scores[i]=-1;
      const auto key_of=[&](const MatchCandidate& x){return TrialKey{std::llround(x.row/step),std::llround(x.column/step),std::llround((x.angle-astart)/angle_quantum),scaled?std::llround((x.scale-smin)/scale_quantum):0};};
      for(const auto& x:next)lattice_scores[trial_slot(key_of(x))]=x.score;
      next.erase(std::remove_if(next.begin(),next.end(),[&](const auto& x){
        const auto key=key_of(x);
        for(int dr=-1;dr<=1;++dr)for(int dc=-1;dc<=1;++dc) {
          if(!dr && !dc)continue;const auto slot=trial_slot(TrialKey{key.row+dr,key.column+dc,key.angle,key.scale});
          if(!trial_key_used[slot])continue;const auto other=lattice_scores[slot];
          if(other>x.score || (other==x.score && (dr<0 || (!dr && dc<0))))return true;
        }
        return false;
      }),next.end());
    }
    const auto dedup_begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    if (diagnostics) diagnostics->refine_score_us += elapsed_us(score_begin, dedup_begin);
    std::sort(next.begin(), next.end(), [](const auto& a, const auto& b) { return a.score > b.score; });
    std::vector<MatchCandidate> dedup;
    for (auto x : next) {
      bool close = false;
      for (const auto& k : dedup) {
        if (std::abs(k.row - x.row) < 0.5 * step && std::abs(k.column - x.column) < 0.5 * step &&
            std::abs(k.angle - x.angle) < 0.5 * model.angle_step() * step &&
            std::abs(k.scale - x.scale) < 0.5 * model.scale_step() * step) {
          close = true;
          break;
        }
      }
      if (!close) dedup.push_back(x);
      if (dedup.size() >= candidate_limit) break;
    }
    cand = std::move(dedup);
    if (diagnostics) diagnostics->refine_dedup_us += elapsed_us(dedup_begin, std::chrono::steady_clock::now());
  }
  if (diagnostics)
    diagnostics->refine_us +=
        elapsed_us(refine_begin, std::chrono::steady_clock::now());
  const auto final_begin = diagnostics ? std::chrono::steady_clock::now()
                                       : std::chrono::steady_clock::time_point{};
  if (diagnostics) diagnostics->final_candidates = cand.size();
  // XLD 最细 trial 在去重前已通过同一双线性评分，保留该精确值，不重复 gather。
  if (!(model.is_xld_model() && top > levels.finest)) {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  {
    const auto& fine_ml = model.levels()[levels.finest];
    const auto& fine_g = gp[levels.finest];
    const bool avx_final =
        have_avx2 && max_deformation == 0 &&

        !model.border_shape_models() && fine_g.has_integer_components() &&
        !cand.empty() && !fine_ml.points.empty();
    if (avx_final) {
      const auto n = fine_ml.points.size();
      const auto fine_soa=prepared_point_soa(model,levels.finest);
      const auto& frow=fine_soa->row;const auto& fcol=fine_soa->column;
      const auto& fnr=fine_soa->nr;const auto& fnc=fine_soa->nc;
      const double min_mag =
          model.min_contrast() / std::max(1.0, fine_ml.pyramid_scale);
      const float min_mag_sq = static_cast<float>(min_mag * min_mag);
      for (auto& x : cand) {
        const double level_row = x.row / fine_ml.pyramid_scale;
        const double level_col = x.column / fine_ml.pyramid_scale;
        const double margin = std::abs(x.scale) * fine_ml.radius + 1.0;
        if (level_row >= margin && level_col >= margin &&
            level_row + margin < static_cast<double>(fine_g.h) &&
            level_col + margin < static_cast<double>(fine_g.w)) {
          x.score = score_i16_bilinear_components(fine_g, min_mag_sq, frow.data(),
              fcol.data(), fnr.data(), fnc.data(), n,
              static_cast<float>(level_row), static_cast<float>(level_col),
              static_cast<float>(std::cos(x.angle)),
              static_cast<float>(std::sin(x.angle)),
              static_cast<float>(x.scale), static_cast<float>(greediness),
              0.0F, static_cast<int>(model.metric()));
        } else {
          x.score = score_pose_level(fine_g, model, levels.finest, x.row,
                                     x.column, x.angle, x.scale, greediness,
                                     0.0, max_deformation);
        }
      }
    } else {
      for (auto& x : cand) {
        x.score = score_pose_level(gp[levels.finest], model, levels.finest, x.row,
                                   x.column, x.angle, x.scale, greediness, 0.0,
                                   max_deformation);
      }
    }
  }
#else
  for (auto& x : cand) {
    x.score = score_pose_level(gp[levels.finest], model, levels.finest, x.row, x.column, x.angle, x.scale,
                               greediness, 0.0, max_deformation);
  }
#endif
  }
  if (diagnostics)
    diagnostics->final_score_us +=
        elapsed_us(final_begin, std::chrono::steady_clock::now());
  cand.erase(std::remove_if(cand.begin(), cand.end(), [&](const auto& x) { return x.score < min_score; }), cand.end());
  std::sort(cand.begin(), cand.end(), [](const auto& a, const auto& b) { return a.score > b.score; });
  return cand;
}

struct PreparedShapeQuery {
  const ShapeModel* model{};
  std::unique_ptr<mv::runtime::ExecutionContext> timed;
  std::unique_ptr<Grad> fine;
  std::vector<MatchCandidate> candidates;
  SubPixelSpec spec;
  std::size_t finest{};
  double scale_min{1},scale_max{1},min_score{};
  bool scaled{};
};
bool refine_shape_candidate(const Grad& fine_g,const ShapeModel& model,std::size_t finest,
    const SubPixelSpec& spec,double search_smin,double search_smax,bool scaled,double min_score,MatchCandidate& x) {
  const double astep=model.angle_step()*std::max(1.0,model.levels()[finest].pyramid_scale);
  const double sstep=model.scale_step()*std::max(1.0,model.levels()[finest].pyramid_scale);
    const auto& fine_ml = model.levels()[finest];
  #if MVKIT_SHAPE_HAS_AVX2_KERNEL
    std::shared_ptr<const PreparedPointSoA> fine_soa;
    std::span<const float> fine_prow,fine_pcol,fine_pnr,fine_pnc;
    const bool avx_fine = simd::cpu_has_avx2() &&
                          fine_g.has_integer_components() &&

                          spec.max_deformation == 0 &&
                          !model.border_shape_models() &&
                          !fine_ml.points.empty();
    if (avx_fine) {
      fine_soa=prepared_point_soa(model,finest);
      fine_prow=fine_soa->row;fine_pcol=fine_soa->column;fine_pnr=fine_soa->nr;fine_pnc=fine_soa->nc;
    }
    const float fine_min_mag_sq = static_cast<float>(
        (model.min_contrast() / std::max(1.0, fine_ml.pyramid_scale)) *
        (model.min_contrast() / std::max(1.0, fine_ml.pyramid_scale)));
  #endif
      auto score_now = [&](const MatchCandidate& c, bool bilinear) {
  #if MVKIT_SHAPE_HAS_AVX2_KERNEL
        if (avx_fine) {
          const double level_row = c.row / fine_ml.pyramid_scale;
          const double level_col = c.column / fine_ml.pyramid_scale;
          const double margin = std::abs(c.scale) * fine_ml.radius + 1.0;
          if (level_row >= margin && level_col >= margin &&
              level_row + margin < static_cast<double>(fine_g.h) &&
              level_col + margin < static_cast<double>(fine_g.w)) {
            const auto ca = static_cast<float>(std::cos(c.angle));
            const auto sa = static_cast<float>(std::sin(c.angle));
            if (bilinear) {
              return score_i16_bilinear_components(fine_g, fine_min_mag_sq,
                  fine_prow.data(), fine_pcol.data(), fine_pnr.data(),
                  fine_pnc.data(), fine_prow.size(),
                  static_cast<float>(level_row), static_cast<float>(level_col),
                  ca, sa, static_cast<float>(c.scale), 0.0F, 0.0F, static_cast<int>(model.metric()));
            }
            return score_i16_nearest_components(fine_g, fine_min_mag_sq,
                fine_prow.data(), fine_pcol.data(), fine_pnr.data(),
                fine_pnc.data(), fine_prow.size(),
                static_cast<float>(level_row), static_cast<float>(level_col), ca,
                sa, static_cast<float>(c.scale), 0.0F, 0.0F, static_cast<int>(model.metric()));
          }
        }
  #endif
        return score_pose_level(fine_g, model, finest, c.row, c.column,
                                c.angle, c.scale, 0, 0, spec.max_deformation,
                                bilinear);
      };
      if (spec.mode == SubPixelSpec::Mode::Interpolation) {
        constexpr double step = 1.0;
          MatchCandidate best = x;
          for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
              for (int da = -1; da <= 1; ++da) {
                for (int ds = scaled ? -1 : 0; ds <= (scaled ? 1 : 0); ++ds) {
                  MatchCandidate trial = x;
                  trial.row += dr * step;
                  trial.column += dc * step;
                  trial.angle += da * astep * step;
                  if (scaled) trial.scale = std::clamp(trial.scale + ds * sstep * step, search_smin, search_smax);
                  trial.score = score_now(trial, true);
                  if (trial.score > best.score) best = trial;
                }
              }
            }
          }
          x = best;
      }
      x = refine_subpixel(fine_g, model, finest, x, spec, astep, sstep, scaled);
      if (spec.mode != SubPixelSpec::Mode::None) {
        x.score = score_now(x, false);
      }
      const double raw_score = x.score;
      x = apply_halcon_affine(x);
      x.score = raw_score;
  return x.score>=min_score;
}

struct MultiQueryCache {
  mv::runtime::ScratchArena scratch;
  std::unique_ptr<GradientPyramid> pyramid;
  std::unique_ptr<Grad> full_components;
  std::vector<std::unique_ptr<Grad>> intermediate_components;
  bool no_directions{};
  bool share_components{true};
  std::size_t coarsest{},finest{};
  double min_contrast{};
};

unsigned shape_radix_trial_sort_requested() noexcept {
  const auto* value = mv::runtime::getenv_or_null("MVKIT_SHAPE_RADIX_TRIAL_SORT");
  if (!value) return 2;
  const std::string_view mode(value);
  if (mode == "1") return 1;
  if (mode == "rank") return 2;
  return 0;
}

std::vector<MatchCandidate> find_impl(const Image& image, const ShapeModel& model, double astart, double aextent,
                                      double smin, double smax, double min_score, std::int64_t num_matches,
                                      double max_overlap, const SubPixelSpec& spec, const SearchLevels& levels,
                                      double greediness, bool scaled, double timeout_ms, bool allow_window = true,
                                      MultiQueryCache* cache = nullptr, bool defer_selection = false,
                                      std::vector<PreparedShapeQuery>* pending = nullptr,
                                      const unsigned* prepared_radix_trial_sort = nullptr) {
  if (image.empty() || model.empty()) {
    throw Exception(Status::InvalidArgument, "find_shape_model", "image/model must not be empty");
  }
  if (min_score < 0 || min_score > 1 || num_matches < 0 || max_overlap < 0 || max_overlap > 1 || greediness < 0 ||
      greediness > 1) {
    throw Exception(Status::InvalidArgument, "find_shape_model", "invalid search parameter");
  }
  double search_start = astart;
  double search_extent = aextent;
  if (!intersect_angle(astart, aextent, model.angle_start(), model.angle_extent(), &search_start, &search_extent)) {
    return {};
  }
  double search_smin = smin;
  double search_smax = smax;
  if (scaled) {
    if (smin > smax) throw Exception(Status::InvalidArgument, "find_scaled_shape_model", "invalid scale range");
    if (!intersect_scale(smin, smax, model.scale_min(), model.scale_max(), &search_smin, &search_smax)) return {};
  }
  if (!pending && allow_window && model.is_xld_model() && image.has_reduced_domain() &&
      image.pixel_type() == mv::core::PixelType::UInt8 && levels.finest == 0 &&
      spec.max_deformation == 0 && !model.border_shape_models() &&
      mv::runtime::getenv_or_null("MVKIT_SHAPE_FULL_PYRAMID") == nullptr) {
    const auto domain=image.domain();
    const auto bounds=domain.bounding_box();
    if (bounds) {
      const auto phase=static_cast<std::int64_t>(model.levels()[levels.coarsest].pyramid_scale);
      double pad=0;
      for (std::size_t i=0;i<=levels.coarsest;++i)
        pad=std::max(pad,(model.levels()[i].radius*std::max(1.0,search_smax)+(i==0?12:3))*model.levels()[i].pyramid_scale);
      const auto start=[&](std::int64_t v){return static_cast<std::int64_t>(std::max(0.0,std::floor((v-pad)/phase)*phase));};
      const auto end=[&](std::int64_t v,std::size_t size){return static_cast<std::int64_t>(std::min(double(size),std::ceil((v+pad+1)/phase)*phase));};
      const auto r0=start(bounds->row_min),c0=start(bounds->col_min);
      const auto r1=end(bounds->row_max,image.height()),c1=end(bounds->col_max,image.width());
      if (r1>r0 && c1>c0 && double(r1-r0)*double(c1-c0)<.35*image.width()*image.height()) {
        // 只读零复制窗口原点对齐粗晶格，各层保留模型/梯度/细化 halo，
        // 人工边界不能影响可达模型样本；不使用查询数据缓存。
        auto owner=std::make_shared<Image>(image);
        auto* pixels=const_cast<std::uint8_t*>(static_cast<const std::uint8_t*>(image.data()))+
            static_cast<std::size_t>(r0)*image.stride()+static_cast<std::size_t>(c0);
        auto window=Image::wrap(static_cast<std::size_t>(c1-c0),static_cast<std::size_t>(r1-r0),
            mv::core::PixelType::UInt8,pixels,image.stride(),owner).reduce_domain(domain.shifted(-r0,-c0));
        auto matches=find_impl(window,model,astart,aextent,smin,smax,min_score,num_matches,
            max_overlap,spec,levels,greediness,scaled,timeout_ms,false);
        for(auto& match:matches){match.row+=r0;match.column+=c0;}
        return matches;
      }
    }
  }
  const unsigned radix_trial_sort = prepared_radix_trial_sort ? *prepared_radix_trial_sort : shape_radix_trial_sort_requested();
  std::unique_ptr<mv::runtime::ExecutionContext> timed;
  TimeoutWatch timeout;
  const double effective_timeout = timeout_ms > 0 ? timeout_ms : model.timeout_ms();
  if (effective_timeout > 0) {
    using Clock = std::chrono::steady_clock;
    const auto now = Clock::now();
    const auto available = Clock::time_point::max() - now;
    const double maximum_ms = std::chrono::duration<double, std::milli>(available).count();
    const double rounded_ms = std::round(effective_timeout);
    const auto deadline = rounded_ms >= maximum_ms ? Clock::time_point::max() :
        now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(rounded_ms));
    timed = std::make_unique<mv::runtime::ExecutionContext>(deadline);
    timeout.context = timed.get();
  }
  ShapeSearchDiagnostics diagnostics_storage;
  ShapeSearchDiagnostics* diagnostics =
      mv::runtime::getenv_or_null("MVKIT_SHAPE_DIAGNOSTICS") ? &diagnostics_storage : nullptr;
  mv::runtime::ScratchArena scratch;
  const auto gradient_begin = diagnostics ? std::chrono::steady_clock::now()
                                          : std::chrono::steady_clock::time_point{};
  const bool reuse=cache && !image.has_reduced_domain() && cache->pyramid && cache->coarsest==levels.coarsest &&
      cache->finest==levels.finest && cache->min_contrast==model.min_contrast();
  std::unique_ptr<GradientPyramid> local_pyramid;
  if(!reuse) {
    auto prepared=std::make_unique<GradientPyramid>(gradient_pyramid(image,model,
        levels.coarsest+1,levels.finest,cache?cache->scratch.resource():scratch.resource(),
        image.has_reduced_domain() && !scaled && spec.max_deformation==0 &&
        !model.border_shape_models() && mv::runtime::getenv_or_null("MVKIT_SHAPE_FULL_PYRAMID")==nullptr,&timeout));
    if(cache){
      cache->intermediate_components.clear();
      const auto& unit=prepared->unit_gradients[levels.coarsest];
      cache->no_directions=true;
      for(std::size_t i=0;i<unit.nx.size();++i)if(unit.nx[i]!=0 || unit.ny[i]!=0){cache->no_directions=false;break;}
      cache->pyramid=std::move(prepared);cache->coarsest=levels.coarsest;cache->finest=levels.finest;cache->min_contrast=model.min_contrast();}
    else local_pyramid=std::move(prepared);
  }
  auto& pyramid=cache?*cache->pyramid:*local_pyramid;
  if(reuse)for(std::size_t i=0;i<levels.coarsest;++i) {
    if(i==levels.finest && (i!=0 || image.pixel_type()!=mv::core::PixelType::UInt8))continue;
    // 每模型重建候选局部 band。
    const auto w=pyramid.gradients[i].w,h=pyramid.gradients[i].h;
    pyramid.gradients[i]=Grad{w,h,cache->scratch.resource()};
    pyramid.unit_gradients[i]=UnitGrad{w,h,cache->scratch.resource(),false};
  }
  if (!pyramid.regions.empty() && mv::runtime::getenv_or_null("MVKIT_SHAPE_VERIFY_ROI_PYRAMID")) {
    auto reference = gradient_pyramid(image, model, levels.coarsest + 1, levels.finest,
                                      scratch.resource(), false, &timeout);
    for (std::size_t i = 1; i < pyramid.regions.size(); ++i) {
      const auto& box = pyramid.regions[i];
      const auto& gray = pyramid.gray_levels[i];
      const auto& ref_gray = reference.gray_levels[i];
      const auto& unit = pyramid.unit_gradients[i];
      const auto& ref_unit = reference.unit_gradients[i];
      auto ur0 = box.r0, ur1 = box.r1, uc0 = box.c0, uc1 = box.c1;
      GrayLevel layout(unit.w, unit.h, scratch.resource());
      layout.limited = true; layout.valid = box;
      const bool have_unit_area = clip_sobel_to_gray(layout, ur0, ur1, uc0, uc1);
      for (auto r = box.r0; r < box.r1; ++r) {
        timeout.poll();
        for (auto c = box.c0; c < box.c1; ++c) {
          if (!gray.pixels.empty() && gray.pixels[r*gray.w+c] != ref_gray.pixels[r*gray.w+c])
            throw std::logic_error("ROI pyramid gray mismatch");
          if (unit.has_unit() && have_unit_area && r >= ur0 && r < ur1 && c >= uc0 && c < uc1 &&
              (unit.nx[r*unit.w+c] != ref_unit.nx[r*unit.w+c] || unit.ny[r*unit.w+c] != ref_unit.ny[r*unit.w+c]))
            throw std::logic_error("ROI pyramid unit mismatch");
        }
      }
    }
  }
  if (diagnostics && !pyramid.regions.empty())
    for (std::size_t i = 1; i < pyramid.regions.size(); ++i) {
      const auto& box = pyramid.regions[i];
      std::fprintf(stderr, "mvkit_roi_pyramid level=%zu pixels=%zu total=%zu\n", i,
          (box.r1-box.r0)*(box.c1-box.c0), pyramid.gradients[i].w*pyramid.gradients[i].h);
    }
  auto& gp = pyramid.gradients;
  if (diagnostics)
    diagnostics->gradient_us =
        elapsed_us(gradient_begin, std::chrono::steady_clock::now());
  if(pending && cache && cache->no_directions && min_score>0) {
    PreparedShapeQuery query;query.model=&model;pending->push_back(std::move(query));return {};
  }
  // HALCON 在所有 subpixel 模式按 NumMatches 剪金字塔候选束。
  const std::size_t candidate_multiplier = 6u;
  const std::size_t minimum_candidate_limit = model.is_xld_model() ? 32u : 6u;
  const std::size_t candidate_limit =
      num_matches > 0
          ? std::min<std::size_t>(
                300, std::max<std::size_t>(minimum_candidate_limit,
                                           num_matches * candidate_multiplier))
          : 300;
  // greedy 单实例刚性搜索可先试一个角相位；保留稠密晶格及特征/候选束规则。
  // 零 greediness、多实例或不支持的金字塔表示使用完整网格。
  const bool phased = !pending && num_matches == 1 && greediness > 0 &&
      levels.finest == 0 && levels.coarsest > 0 &&
      search_extent > model.angle_step() * model.levels()[levels.coarsest].pyramid_scale &&
      image.pixel_type() == mv::core::PixelType::UInt8 &&
      model.metric() == ShapeMetric::UsePolarity && !model.border_shape_models() &&
      spec.max_deformation == 0 && !scaled;
  for (int attempt = 0; attempt < (phased ? 2 : 1); ++attempt) {
    if (attempt != 0) {
      // 灰度金字塔及完整粗梯度可复用，较低层梯度只在先前候选周围填充。
      for (std::size_t i = 0; i < levels.coarsest; ++i) {
        const auto w = gp[i].w, h = gp[i].h;
        gp[i] = Grad{w, h, scratch.resource()};
        pyramid.unit_gradients[i] = UnitGrad{w, h, scratch.resource(), false};
      }
    }
    const auto phase = !phased ? CoarseAnglePhase::Full :
        (attempt == 0 ? CoarseAnglePhase::Primary : CoarseAnglePhase::Complementary);
    if(pending && cache && cache->share_components && levels.finest==0 && image.pixel_type()==mv::core::PixelType::UInt8) {
      if(!cache->full_components) {
        cache->full_components=std::make_unique<Grad>(image.width(),image.height(),cache->scratch.resource());
        fill_u8_sobel_component_rows(image,*cache->full_components,0,image.height(),0,image.width());
      }
      auto& g=gp[0];g.gx_i16=PmrBuffer<std::int16_t>::borrow(cache->full_components->gx_i16);
      g.gy_i16=PmrBuffer<std::int16_t>::borrow(cache->full_components->gy_i16);g.integer_scale=cache->full_components->integer_scale;
      g.complete_owner=cache->full_components.get();
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
      if(!image.has_reduced_domain() && simd::cpu_has_avx2() && spec.max_deformation==0 && !model.border_shape_models()) {
        cache->intermediate_components.resize(levels.coarsest);
        for(std::size_t i=1;i<levels.coarsest;++i) {
          auto& complete=cache->intermediate_components[i];const auto& gray=pyramid.gray_levels[i];
          if(!complete) {
            complete=std::make_unique<Grad>(gray.w,gray.h,cache->scratch.resource());
            complete->gx_i16=PmrBuffer<std::int16_t>(gray.w*gray.h,cache->scratch.resource());
            complete->gy_i16=PmrBuffer<std::int16_t>(gray.w*gray.h,cache->scratch.resource());
            complete->integer_scale=.25;
            fill_float_gray_i16_rows(gray,*complete,0,gray.h,0,gray.w);
          }
          gp[i].gx_i16=PmrBuffer<std::int16_t>::borrow(complete->gx_i16);
          gp[i].gy_i16=PmrBuffer<std::int16_t>::borrow(complete->gy_i16);gp[i].integer_scale=complete->integer_scale;
          gp[i].complete_owner=complete.get();
        }
      }
#endif
    }
    auto cand = coarse_to_fine(image, pyramid, model, search_start, search_extent,
        search_smin, search_smax, min_score, levels, greediness, scaled,
        spec.max_deformation, candidate_limit, timeout, diagnostics, scratch.resource(), phase,pending!=nullptr,radix_trial_sort);
    if(pending) {
      if(cand.empty() && phased && attempt==0)continue;
      PreparedShapeQuery query;query.model=&model;query.spec=spec;query.finest=levels.finest;
      query.scale_min=search_smin;query.scale_max=search_smax;query.scaled=scaled;query.min_score=min_score;
      query.timed=std::move(timed);query.candidates=std::move(cand);query.fine=std::make_unique<Grad>(std::move(gp[levels.finest]));
      if(diagnostics)std::fprintf(stderr,"multi stages gradient=%.3f coarse=%.3f refine=%.3f generate=%.3f score=%.3f dedup=%.3f final=%.3f candidates=%zu\n",
          diagnostics->gradient_us,diagnostics->coarse_us,diagnostics->refine_us,diagnostics->refine_generate_us,
          diagnostics->refine_score_us,diagnostics->refine_dedup_us,diagnostics->final_score_us,query.candidates.size());
      pending->push_back(std::move(query));
      if(cache && levels.finest==levels.coarsest)cache->pyramid.reset();
      return {};
    }
    std::vector<MatchCandidate> out;
    const auto output_begin = diagnostics ? std::chrono::steady_clock::now()
                                          : std::chrono::steady_clock::time_point{};
    const double astep = model.angle_step() * std::max(1.0, model.levels()[levels.finest].pyramid_scale);
    const double sstep = model.scale_step() * std::max(1.0, model.levels()[levels.finest].pyramid_scale);
    const auto& fine_g = gp[levels.finest];
    const auto& fine_ml = model.levels()[levels.finest];
  #if MVKIT_SHAPE_HAS_AVX2_KERNEL
    std::shared_ptr<const PreparedPointSoA> fine_soa;
    std::span<const float> fine_prow,fine_pcol,fine_pnr,fine_pnc;
    const bool avx_fine = simd::cpu_has_avx2() &&
                          fine_g.has_integer_components() &&

                          spec.max_deformation == 0 &&
                          !model.border_shape_models() &&
                          !fine_ml.points.empty();
    if (avx_fine) {
      fine_soa=prepared_point_soa(model,levels.finest);
      fine_prow=fine_soa->row;fine_pcol=fine_soa->column;fine_pnr=fine_soa->nr;fine_pnc=fine_soa->nc;
    }
    const float fine_min_mag_sq = static_cast<float>(
        (model.min_contrast() / std::max(1.0, fine_ml.pyramid_scale)) *
        (model.min_contrast() / std::max(1.0, fine_ml.pyramid_scale)));
  #endif
    ModelBounds overlap_bounds{};
    bool have_overlap_bounds = false;
    for (auto x : cand) {
      timeout.poll();
      bool suppress = false;
      if (!defer_selection) for (const auto& k : out) {
        // 搜索内边界不变，仅需要时计算。
        if (!have_overlap_bounds) {
          overlap_bounds = model_bounds(model);
          have_overlap_bounds = true;
        }
        if (overlap(overlap_bounds, x, k) > max_overlap) {
          suppress = true;
          break;
        }
      }
      if (suppress) continue;
      auto score_now = [&](const MatchCandidate& c, bool bilinear) {
  #if MVKIT_SHAPE_HAS_AVX2_KERNEL
        if (avx_fine) {
          const double level_row = c.row / fine_ml.pyramid_scale;
          const double level_col = c.column / fine_ml.pyramid_scale;
          const double margin = std::abs(c.scale) * fine_ml.radius + 1.0;
          if (level_row >= margin && level_col >= margin &&
              level_row + margin < static_cast<double>(fine_g.h) &&
              level_col + margin < static_cast<double>(fine_g.w)) {
            const auto ca = static_cast<float>(std::cos(c.angle));
            const auto sa = static_cast<float>(std::sin(c.angle));
            if (bilinear) {
              return score_i16_bilinear_components(fine_g, fine_min_mag_sq,
                  fine_prow.data(), fine_pcol.data(), fine_pnr.data(),
                  fine_pnc.data(), fine_prow.size(),
                  static_cast<float>(level_row), static_cast<float>(level_col),
                  ca, sa, static_cast<float>(c.scale), 0.0F, 0.0F, static_cast<int>(model.metric()));
            }
            return score_i16_nearest_components(fine_g, fine_min_mag_sq,
                fine_prow.data(), fine_pcol.data(), fine_pnr.data(),
                fine_pnc.data(), fine_prow.size(),
                static_cast<float>(level_row), static_cast<float>(level_col), ca,
                sa, static_cast<float>(c.scale), 0.0F, 0.0F, static_cast<int>(model.metric()));
          }
        }
  #endif
        return score_pose_level(fine_g, model, levels.finest, c.row, c.column,
                                c.angle, c.scale, 0, 0, spec.max_deformation,
                                bilinear);
      };
      if (spec.mode == SubPixelSpec::Mode::Interpolation) {
        constexpr double step = 1.0;
          MatchCandidate best = x;
          for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
              for (int da = -1; da <= 1; ++da) {
                for (int ds = scaled ? -1 : 0; ds <= (scaled ? 1 : 0); ++ds) {
                  MatchCandidate trial = x;
                  trial.row += dr * step;
                  trial.column += dc * step;
                  trial.angle += da * astep * step;
                  if (scaled) trial.scale = std::clamp(trial.scale + ds * sstep * step, search_smin, search_smax);
                  trial.score = score_now(trial, true);
                  if (trial.score > best.score) best = trial;
                }
              }
            }
          }
          x = best;
      }
      x = refine_subpixel(fine_g, model, levels.finest, x, spec, astep, sstep, scaled);
      if (spec.mode != SubPixelSpec::Mode::None) {
        x.score = score_now(x, false);
      }
      const double raw_score = x.score;
      x = apply_halcon_affine(x);
      x.score = raw_score;
      if (x.score < min_score) continue;
      out.push_back(x);
      if (!defer_selection && num_matches > 0 && out.size() >= static_cast<std::size_t>(num_matches)) break;
    }
    if (diagnostics)
      diagnostics->output_us += elapsed_us(output_begin, std::chrono::steady_clock::now());
    if (out.empty() && phased && attempt == 0) continue;
    if (diagnostics) {
      std::fprintf(stderr, "mvkit_shape_refine_stages prepare_us=%.3f generate_us=%.3f score_us=%.3f dedup_us=%.3f\n",
                   diagnostics->refine_prepare_us, diagnostics->refine_generate_us,
                   diagnostics->refine_score_us, diagnostics->refine_dedup_us);
      std::fprintf(
          stderr,
          "mvkit_shape_diag gradient_us=%.3f unit_us=%.3f coarse_us=%.3f "
          "refine_us=%.3f final_us=%.3f output_us=%.3f coarse_eval=%zu "
          "coarse_scored=%zu coarse_accept=%zu refine_eval=%zu refine_accept=%zu "
          "final=%zu\n",
          diagnostics->gradient_us, diagnostics->unit_gradient_us,
          diagnostics->coarse_us, diagnostics->refine_us,
          diagnostics->final_score_us, diagnostics->output_us,
          diagnostics->coarse_evaluated, diagnostics->coarse_scored,
          diagnostics->coarse_accepted,
          diagnostics->refine_evaluated, diagnostics->refine_accepted,
          diagnostics->final_candidates);
      std::fprintf(
          stderr,
          "mvkit_shape_invert generated=%zu verified095=%zu\n",
          diagnostics->invert_generated, diagnostics->invert_verified);
    }
    return out;
  }
  return {};
}
}  // namespace

std::vector<MatchCandidate> find_shape_model(const Image& image, const ShapeModel& model, double angle_start,
                                             double angle_extent, double min_score, std::int64_t num_matches,
                                             double max_overlap, const Tuple& sub_pixel, const Tuple& num_levels,
                                             double greediness, double timeout_ms) {
  if (model.kind() != ShapeModelKind::Rigid) {
    throw Exception(Status::InvalidArgument, "find_shape_model", "model is not rigid");
  }
  return find_impl(image, model, angle_start, angle_extent, 1, 1, min_score, num_matches, max_overlap, parse_sub_pixel(sub_pixel),
                   parse_search_levels(num_levels, model.levels().size()), greediness, false, timeout_ms);
}

std::vector<MatchCandidate> find_scaled_shape_model(const Image& image, const ShapeModel& model, double angle_start,
                                                    double angle_extent, double scale_min, double scale_max,
                                                    double min_score, std::int64_t num_matches, double max_overlap,
                                                    const Tuple& sub_pixel, const Tuple& num_levels, double greediness, double timeout_ms) {
  if (model.kind() != ShapeModelKind::IsotropicScale) {
    throw Exception(Status::InvalidArgument, "find_scaled_shape_model", "model is not scaled");
  }
  return find_impl(image, model, angle_start, angle_extent, scale_min, scale_max, min_score, num_matches, max_overlap,
                   parse_sub_pixel(sub_pixel), parse_search_levels(num_levels, model.levels().size()), greediness, true, timeout_ms);
}

std::vector<MatchCandidate> find_model(const Image& image,const ShapeModel& model,const SearchParameters& p) {
  return find_impl(image,model,p.angle_start,p.angle_extent,p.scale_min,p.scale_max,p.min_score,p.num_matches,p.max_overlap,
      SubPixelSpec{static_cast<SubPixelSpec::Mode>(p.subpixel),p.max_deformation},SearchLevels{p.coarsest,p.finest},p.greediness,
      (p.scale_search<0?model.kind()==ShapeModelKind::IsotropicScale:p.scale_search!=0),p.timeout_ms);
}

std::vector<matching::Match<MatchCandidate>> find_models(const Image& image,
    std::span<const ShapeModel* const> models,std::span<const SearchParameters> options,matching::Selection selection,
    std::span<const Image* const> model_images) {
  if(model_images.empty() && models.size()==1 && image.has_reduced_domain() && image.pixel_type()==mv::core::PixelType::UInt8 &&
      options[0].finest==0 && options[0].max_deformation==0 && !models[0]->border_shape_models() &&
      models[0]->kind()==ShapeModelKind::Rigid && !mv::runtime::getenv_or_null("MVKIT_SHAPE_FULL_PYRAMID")) {
    const auto domain=image.domain();const auto bounds=domain.bounding_box();
    if(bounds) {
      const auto phase=models[0]->levels()[options[0].coarsest].pyramid_scale;double pad=0;
      for(std::size_t i=0;i<=options[0].coarsest;++i)pad=std::max(pad,(models[0]->levels()[i].radius+(i==0?12:3))*models[0]->levels()[i].pyramid_scale);
      const auto start=[&](std::int64_t v){return std::int64_t(std::max(0.,std::floor((v-pad)/phase)*phase));};
      const auto end=[&](std::int64_t v,std::size_t size){return std::int64_t(std::min(double(size),std::ceil((v+pad+1)/phase)*phase));};
      const auto r0=start(bounds->row_min),c0=start(bounds->col_min),r1=end(bounds->row_max,image.height()),c1=end(bounds->col_max,image.width());
      if(r1>r0 && c1>c0 && double(r1-r0)*double(c1-c0)<.35*image.width()*image.height()) {
        auto owner=std::make_shared<Image>(image);
        auto* pixels=const_cast<std::uint8_t*>(static_cast<const std::uint8_t*>(image.data()))+std::size_t(r0)*image.stride()+std::size_t(c0);
        auto window=Image::wrap(std::size_t(c1-c0),std::size_t(r1-r0),mv::core::PixelType::UInt8,pixels,image.stride(),owner).reduce_domain(domain.shifted(-r0,-c0));
        auto result=find_models(window,models,options,selection);for(auto& m:result){m.pose.row+=r0;m.pose.column+=c0;}return result;
      }
    }
  }
  const unsigned radix_trial_sort = shape_radix_trial_sort_requested();
  MultiQueryCache cache;cache.share_components=models.size()>1;std::vector<matching::Match<MatchCandidate>> candidates;
  std::vector<std::unique_ptr<MultiQueryCache>> image_caches;
  if(!model_images.empty()) {
    if(model_images.size()!=models.size())throw Exception(Status::InvalidArgument,"find_shape_models","one image per model required");
    for(std::size_t i=0;i<models.size();++i){auto value=std::make_unique<MultiQueryCache>();value->share_components=false;image_caches.push_back(std::move(value));}
  }
  std::vector<PreparedShapeQuery> queries;queries.reserve(models.size());
  std::vector<std::size_t> owners;
  const auto same_points=[](auto a,auto b){
    return a.size()==b.size() && std::equal(a.begin(),a.end(),b.begin(),[](const auto& x,const auto& y){return x.row==y.row && x.column==y.column && x.normal_row==y.normal_row && x.normal_column==y.normal_column && x.response==y.response;});
  };
  const auto equivalent=[&](const ShapeModel& a,const ShapeModel& b){
    if(&a==&b)return true;
    if(a.kind()!=b.kind() || a.metric()!=b.metric() || a.is_xld_model()!=b.is_xld_model() ||
       a.origin_row()!=b.origin_row() || a.origin_column()!=b.origin_column() ||
       a.angle_start()!=b.angle_start() || a.angle_extent()!=b.angle_extent() || a.angle_step()!=b.angle_step() ||
       a.scale_min()!=b.scale_min() || a.scale_max()!=b.scale_max() || a.scale_step()!=b.scale_step() ||
       a.min_contrast()!=b.min_contrast() || a.border_shape_models()!=b.border_shape_models() ||
       a.levels().size()!=b.levels().size() || !same_points(a.refinement_points(),b.refinement_points()))return false;
    for(std::size_t i=0;i<a.levels().size();++i){const auto& x=a.levels()[i];const auto& y=b.levels()[i];if(x.pyramid_scale!=y.pyramid_scale || x.radius!=y.radius || !same_points(x.points,y.points))return false;}
    return true;
  };
  for(std::size_t i=0;i<models.size();++i) {
    const auto& model=*models[i];auto p=options[i];
    std::size_t owner=i;
    if(p.timeout_ms<=0 && model.timeout_ms()<=0)for(std::size_t j=0;j<i;++j)
      if((model_images.empty() || model_images[j]==model_images[i]) && options[j]==options[i] && models[j]->timeout_ms()<=0 && equivalent(*models[j],model)){owner=owners[j];break;}
    owners.push_back(owner);
    if(owner!=i){PreparedShapeQuery q;q.model=&model;queries.push_back(std::move(q));for(const auto& pose:queries[owner].candidates)candidates.push_back({apply_halcon_affine(pose),i});continue;}
    if(selection.num_matches>=0)p.num_matches=selection.num_matches;
    if(selection.max_overlap>=0 && p.num_matches==1)p.num_matches=2;
    (void)find_impl(model_images.empty()?image:*model_images[i],model,p.angle_start,p.angle_extent,p.scale_min,p.scale_max,p.min_score,
        p.num_matches,p.max_overlap,SubPixelSpec{static_cast<SubPixelSpec::Mode>(p.subpixel),p.max_deformation},
        SearchLevels{p.coarsest,p.finest},p.greediness,(p.scale_search<0?model.kind()==ShapeModelKind::IsotropicScale:p.scale_search!=0),
        p.timeout_ms,false,model_images.empty()?&cache:image_caches[i].get(),true,&queries,&radix_trial_sort);
    // 角度/尺度交集为空时在查询准备前返回。
    if(queries.size()==i){PreparedShapeQuery q;q.model=&model;queries.push_back(std::move(q));}
    for(const auto& pose:queries.back().candidates)candidates.push_back({apply_halcon_affine(pose),i});
  }
  std::vector<matching::Box> bounds;for(const auto* model:models)bounds.push_back(model_overlap_box(*model));
  const bool diagnostics = mv::runtime::getenv_or_null("MVKIT_SHAPE_DIAGNOSTICS") != nullptr;
  const auto selection_begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  const auto selection_candidates = candidates.size();
  double subpixel_us = 0;
  std::size_t subpixel_calls = 0;
  auto result = matching::select_refined(std::move(candidates),options,selection,[&](const auto& candidate){
    auto b=bounds[candidate.model_index];auto pose=candidate.pose;
    const double ca=std::cos(pose.angle),sa=std::sin(pose.angle);
    pose.row=pose.row-.5+.5*pose.scale*(ca-sa);
    pose.column=pose.column-.5+.5*pose.scale*(ca+sa);
    for(auto& p:b){const auto x=p.x,y=p.y;p={pose.column+pose.scale*(ca*x+sa*y),pose.row+pose.scale*(-sa*x+ca*y)};}
    return b;
  },[&](auto& candidate){
    auto& q=queries[owners[candidate.model_index]];TimeoutWatch{q.timed.get()}.poll();auto& pose=candidate.pose;
    const double ca=std::cos(pose.angle),sa=std::sin(pose.angle);
    pose.row=pose.row-.5+.5*pose.scale*(ca-sa);pose.column=pose.column-.5+.5*pose.scale*(ca+sa);
    const auto begin = diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    const bool accepted = refine_shape_candidate(*q.fine,*q.model,q.finest,q.spec,q.scale_min,q.scale_max,q.scaled,q.min_score,pose);
    if (diagnostics) { subpixel_us += elapsed_us(begin, std::chrono::steady_clock::now()); ++subpixel_calls; }
    return accepted;
  });
  if (diagnostics) std::fprintf(stderr,"mvkit_shape_multi_selection total_us=%.3f subpixel_us=%.3f subpixel_calls=%zu candidates=%zu output=%zu\n",
      elapsed_us(selection_begin, std::chrono::steady_clock::now()), subpixel_us, subpixel_calls, selection_candidates, result.size());
  return result;
}

}  // namespace mv::shape_matching
