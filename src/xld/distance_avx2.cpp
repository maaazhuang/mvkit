#include "distance_avx2.hpp"
#include <xld/contour_utils.hpp>
#include <immintrin.h>
#include <runtime/cpu/cpu_features.hpp>
#include <cfenv>
namespace mv::xld {
namespace {
void load4(const core::Point2d *p, __m256d &row, __m256d &col) {
  const auto a = _mm256_loadu_pd(&p[0].row), b = _mm256_loadu_pd(&p[2].row);
  row = _mm256_permute4x64_pd(_mm256_unpacklo_pd(a, b), 0xd8);
  col = _mm256_permute4x64_pd(_mm256_unpackhi_pd(a, b), 0xd8);
}
} // namespace
void validate_distance_points_avx2(std::span<const core::Point2d> points) {
  const auto bound = _mm256_set1_pd(0x1p30), sign = _mm256_set1_pd(-0.0);
  std::size_t i = 0;
  for (; i + 2 <= points.size(); i += 2) {
    const auto p = _mm256_loadu_pd(&points[i].row);
    const auto ok = _mm256_cmp_pd(_mm256_andnot_pd(sign, p), bound, _CMP_LE_OQ);
    detail::require(_mm256_movemask_pd(ok) == 15, "distance_pc",
                       "Coordinates",
                       "finite coordinates in [-2^30,2^30] required");
  }
  for (; i < points.size(); ++i) {
    detail::coordinate(points[i].row, "distance_pc");
    detail::coordinate(points[i].col, "distance_pc");
  }
}
std::array<double, 2>
distance_point_contour_avx2(std::span<const core::Point2d> points, bool closed,
                            core::Point2d p) {
  const auto pr = _mm256_set1_pd(p.row), pc = _mm256_set1_pd(p.col);
#if defined(_MSC_VER)
  const auto& cpu=runtime::cpu_features();
  const bool fused_lerp=runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(),cpu.hardware()),
      runtime::KernelRequirement{false,true,true,false}) && std::fegetround()==FE_TONEAREST;
#endif
  auto high = _mm256_set1_pd(-1), hr = _mm256_setzero_pd(), hc = hr;
  std::size_t i = 0;
  // 独立 lane 最大值避免长轮廓标量依赖链。
  for (; i + 4 <= points.size(); i += 4) {
    __m256d r, c;
    load4(points.data() + i, r, c);
    r = _mm256_sub_pd(pr, r);
    c = _mm256_sub_pd(pc, c);
    const auto d = _mm256_add_pd(_mm256_mul_pd(r, r), _mm256_mul_pd(c, c));
    auto larger = _mm256_cmp_pd(d, high, _CMP_GT_OQ);
    const int ties = _mm256_movemask_pd(_mm256_cmp_pd(d, high, _CMP_EQ_OQ));
    if (ties) {
      alignas(32) double nr[4], nc[4], oldr[4], oldc[4];
      alignas(32) std::int64_t replace[4]{};
      _mm256_store_pd(nr, r);
      _mm256_store_pd(nc, c);
      _mm256_store_pd(oldr, hr);
      _mm256_store_pd(oldc, hc);
      for (int lane = 0; lane < 4; ++lane)
        if ((ties & (1 << lane)) &&
            std::hypot(nr[lane], nc[lane]) > std::hypot(oldr[lane], oldc[lane]))
          replace[lane] = -1;
      larger = _mm256_or_pd(larger,
                            _mm256_castsi256_pd(_mm256_load_si256(
                                reinterpret_cast<const __m256i *>(replace))));
    }
    hr = _mm256_blendv_pd(hr, r, larger);
    hc = _mm256_blendv_pd(hc, c, larger);
    high = _mm256_max_pd(high, d);
  }
  alignas(32) double rows[4], cols[4], squared[4];
  _mm256_store_pd(rows, hr);
  _mm256_store_pd(cols, hc);
  _mm256_store_pd(squared, high);
  double maximum = 0;
  for (int lane = 0; lane < 4; ++lane)
    if (squared[lane] >= 0)
      maximum = std::max(maximum, std::hypot(rows[lane], cols[lane]));
  for (; i < points.size(); ++i)
    maximum = std::max(
        maximum, std::hypot(p.row - points[i].row, p.col - points[i].col));
  double low2 = INFINITY;
  core::Point2d delta{};
  const auto consider_delta = [&](core::Point2d d) {
    const double d2 = d.row * d.row + d.col * d.col;
    if (d2 < low2 || (d2 == low2 && std::hypot(d.row, d.col) <
                                        std::hypot(delta.row, delta.col))) {
      low2 = d2;delta = d;
    }
  };
  const auto consider_t = [&](core::Point2d a, core::Point2d b,double t) {
    const core::Point2d q = t == 0 ? a
                            : t == 1
                                ? b
                                : core::Point2d{std::lerp(a.row, b.row, t),
                                                std::lerp(a.col, b.col, t)};
    const core::Point2d d{p.row - q.row, p.col - q.col};
    consider_delta(d);
  };
  const auto consider = [&](core::Point2d a,core::Point2d b) {
    const double dr=b.row-a.row,dc=b.col-a.col,den=dr*dr+dc*dc;
    const double t=den==0?0:std::clamp(((p.row-a.row)*dr+(p.col-a.col)*dc)/den,0.,1.);
    consider_t(a,b,t);
  };
  i = 0;
  for (; i + 4 < points.size(); i += 4) {
    __m256d ar, ac, br, bc;
    load4(points.data() + i, ar, ac);
    load4(points.data() + i + 1, br, bc);
    const auto r =
        _mm256_max_pd(_mm256_max_pd(_mm256_sub_pd(_mm256_min_pd(ar, br), pr),
                                    _mm256_sub_pd(pr, _mm256_max_pd(ar, br))),
                      _mm256_setzero_pd());
    const auto c =
        _mm256_max_pd(_mm256_max_pd(_mm256_sub_pd(_mm256_min_pd(ac, bc), pc),
                                    _mm256_sub_pd(pc, _mm256_max_pd(ac, bc))),
                      _mm256_setzero_pd());
    const auto bound = _mm256_add_pd(_mm256_mul_pd(r, r), _mm256_mul_pd(c, c));
    const int candidates = _mm256_movemask_pd(_mm256_cmp_pd(
        bound,
        _mm256_set1_pd(low2 * (1 + 8 * std::numeric_limits<double>::epsilon())),
        _CMP_LE_OQ));
    if(!candidates)continue;
    const auto dr=_mm256_sub_pd(br,ar),dc=_mm256_sub_pd(bc,ac),zero=_mm256_setzero_pd(),one=_mm256_set1_pd(1.);
    const auto denominator=_mm256_add_pd(_mm256_mul_pd(dr,dr),_mm256_mul_pd(dc,dc));
    const auto degenerate=_mm256_cmp_pd(denominator,zero,_CMP_EQ_OQ);
    const auto numerator=_mm256_add_pd(_mm256_mul_pd(_mm256_sub_pd(pr,ar),dr),_mm256_mul_pd(_mm256_sub_pd(pc,ac),dc));
    auto t=_mm256_div_pd(numerator,_mm256_blendv_pd(denominator,one,degenerate));
    t=_mm256_blendv_pd(_mm256_min_pd(one,_mm256_max_pd(zero,t)),zero,degenerate);
#if defined(_MSC_VER)
    if(fused_lerp) {
      // Windows 标准 lerp 对同号有限端点使用融合乘加；保留该舍入语义，
      // 跨零线段采用两端权重相加。FMA 单独检查，不把它作为 AVX2 的隐含前提。
      const auto interpolate=[&](__m256d a,__m256d b) {
        const auto opposite=_mm256_or_pd(
          _mm256_and_pd(_mm256_cmp_pd(a,zero,_CMP_LE_OQ),_mm256_cmp_pd(b,zero,_CMP_GE_OQ)),
          _mm256_and_pd(_mm256_cmp_pd(a,zero,_CMP_GE_OQ),_mm256_cmp_pd(b,zero,_CMP_LE_OQ)));
        const auto weighted=_mm256_add_pd(_mm256_mul_pd(t,b),_mm256_mul_pd(_mm256_sub_pd(one,t),a));
        const auto fused=_mm256_fmadd_pd(t,_mm256_sub_pd(b,a),a);
        const auto bounded=_mm256_blendv_pd(_mm256_max_pd(fused,b),_mm256_min_pd(fused,b),_mm256_cmp_pd(b,a,_CMP_GT_OQ));
        auto q=_mm256_blendv_pd(bounded,weighted,opposite);
        q=_mm256_blendv_pd(q,a,_mm256_cmp_pd(t,zero,_CMP_EQ_OQ));
        return _mm256_blendv_pd(q,b,_mm256_cmp_pd(t,one,_CMP_EQ_OQ));
      };
      alignas(32) double residual_rows[4],residual_columns[4];
      _mm256_store_pd(residual_rows,_mm256_sub_pd(pr,interpolate(ar,br)));
      _mm256_store_pd(residual_columns,_mm256_sub_pd(pc,interpolate(ac,bc)));
      for(int lane=0;lane<4;++lane)if(candidates & (1<<lane))
        consider_delta({residual_rows[lane],residual_columns[lane]});
      continue;
    }
#endif
    alignas(32) double parameters[4];_mm256_store_pd(parameters,t);
    // 投影参数批量计算，插值仍调用标准 lerp，并按原段序处理极值和相等距离。
    for (int lane = 0; lane < 4; ++lane)
      if (candidates & (1 << lane))
        consider_t(points[i + lane], points[i + lane + 1],parameters[lane]);
  }
  for (; i + 1 < points.size(); ++i)
    consider(points[i], points[i + 1]);
  if (points.size() == 1 || (closed && points.front() != points.back()))
    consider(points.back(), points.front());
  return {std::hypot(delta.row, delta.col), maximum};
}
} // namespace mv::xld
