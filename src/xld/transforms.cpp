#include <xld/operators.hpp>
#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <runtime/execution/execution_context.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <runtime/memory/metadata_arena.hpp>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_XLD_TRANSFORM_SSE2 1
#else
#define MVKIT_XLD_TRANSFORM_SSE2 0
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <memory_resource>
#include <limits>
#include <type_traits>
#include <optional>
#include <xld/fast_paths_avx2.hpp>
#include <numbers>

namespace mv::xld {
namespace {
using namespace mv::core;
using mv::runtime::ExecutionContext;
[[noreturn]] void fail_check(const char* op, const char* parameter, const char* message) {
  throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, op, parameter, -1, message);
}
inline void check(bool valid, const char* op, const char* parameter, const char* message) {
  if (!valid) fail_check(op, parameter, message);
}

double angle(double y, double x) {
  double a = std::atan2(y, x);
  return a < 0 ? a + 2 * std::numbers::pi : a;
}
template <class F>
XldContours map(const XldContours& input, F&& fn, bool pooled = false,mv::runtime::MetadataArena* arena = nullptr) {
  if(input.empty())return {};
  if(input.contour_count()==1)
    return XldContours{IconicTuple{IconicValue::from_xld_contour(fn(input.contour(0)),pooled,arena)}};
  std::size_t work = 0;
  for (std::size_t i = 0; i < input.contour_count(); ++i)
    work += std::min(input.contour(i).size(), SIZE_MAX - work);
  // 已发布串行 arena 不得进入后续并行分区，执行器启动前进程 worker 预算改变也如此。
  if(arena || work<32768 || mv::runtime::max_parallelism()==1) {
    std::vector<IconicValue> values;values.reserve(input.contour_count());
    for(std::size_t i=0;i<input.contour_count();++i)
      values.push_back(IconicValue::from_xld_contour(fn(input.contour(i)),pooled,arena));
    return XldContours{IconicTuple{std::move(values)}};
  }
  mv::runtime::ScratchArena context;
  std::pmr::vector<XldContour> result(input.contour_count(),context.resource());
  ExecutionContext::for_each_partition(
      result.size(), work, 32768, [&](auto range) {
        for (auto i = range.begin; i < range.end; ++i)
          result[i] = fn(input.contour(i));
      });
  std::vector<IconicValue> values;
  values.reserve(result.size());
  for (auto& r : result)
    values.push_back(IconicValue::from_xld_contour(std::move(r),pooled,arena));
  return XldContours{IconicTuple{std::move(values)}};
}
std::array<double, 9> matrix_values(const Tuple& matrix, bool affine,
                                    const char* op) {
  const auto count = affine ? 6U : 9U;
  if (matrix.size() != count)
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, op,
                    "Matrix", -1,
                    "expected six affine or nine projective coefficients");
  std::array<double, 9> m{0, 0, 0, 0, 0, 0, 0, 0, 1};
  double scale = affine ? 1 : 0;
  for (std::size_t i = 0; i < count; ++i) {
    m[i] = matrix.as_real64(i);
    check(std::isfinite(m[i]), op, "Matrix", "finite coefficients required");
    scale = std::max(scale, std::abs(m[i]));
  }
  check(scale > 0, op, "Matrix", "zero matrix");
  for (auto& v : m) v /= scale;
  const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) -
                     m[1] * (m[3] * m[8] - m[5] * m[6]) +
                     m[2] * (m[3] * m[7] - m[4] * m[6]);
  check(std::isfinite(det) && det != 0, op, "Matrix",
        "singular or numerically degenerate matrix");
  if (affine) {
    for (std::size_t i = 0; i < 6; ++i) m[i] = matrix.as_real64(i);
    m[8] = 1;
  }
  return m;
}
XldContours transform(const XldContours& input, const Tuple& matrix,
                      bool affine) {
  const auto op =
      affine ? "affine_trans_contour_xld" : "projective_trans_contour_xld";
  const auto m = matrix_values(matrix, affine, op);
  const bool translation_only=affine && m[0]==1 && m[4]==1 && m[1]==0 && m[3]==0 && m[2]!=0 && m[5]!=0;
  const int translation_mode=!translation_only ? 0 :
      std::abs(m[2])<=0x1p969 && std::abs(m[5])<=0x1p969 ? 2 : 1;
  const bool pooled_metadata=affine;
  mv::runtime::MetadataArena::Owner metadata;
  if(affine && input.contour_count()>1 && input.contour_count()<=128 && mv::runtime::max_parallelism()==1)
    metadata=mv::runtime::MetadataArena::create();
  const auto& cpu=mv::runtime::cpu_features();
  const auto capabilities=mv::runtime::effective_capabilities(cpu.selection(),cpu.hardware());
#if MVKIT_XLD_HAS_AVX2
  const bool avx2=mv::runtime::kernel_allowed(capabilities,{false,false,true,false});
#endif
#if MVKIT_XLD_TRANSFORM_SSE2
  const bool sse2=mv::runtime::kernel_allowed(capabilities,{true,false,false,false});
#endif
  return map(input, [&](const XldContour& contour) {
    std::pmr::vector<Point2d> points{mv::runtime::persist_resource()};
    const auto input_points = contour.points();
    const auto old_edge =
                   contour.point_attribute(XldPointAttribute::EdgeDirection),
               old_normal =
                   contour.point_attribute(XldPointAttribute::NormalAngle);
    const bool use_inline=old_edge.empty() && old_normal.empty() && input_points.size()<=16;
    const bool use_payload=old_edge.empty() && old_normal.empty() && input_points.size()>=128;
    XldContour::OwnedPointPayload payload{mv::runtime::PlainPayloadAllocator<Point2d>{points.get_allocator().resource(),true}};
    std::array<Point2d,16> inline_points;
    if(use_payload)payload.reset_for_overwrite(input_points.size());
    else if(!use_inline)points.reserve(input_points.size());
    std::optional<ExecutionContext> context;
    if(!old_edge.empty() || !old_normal.empty())context.emplace();
    auto* memory=context ? context->scratch_resource() : mv::runtime::persist_resource();
    std::pmr::vector<double> edge{memory}, normal{memory}, denominator{memory};
    edge.reserve(old_edge.size());
    normal.reserve(old_normal.size());
    if (!affine && (!old_edge.empty() || !old_normal.empty())) denominator.reserve(contour.size());
    if(!affine && old_edge.empty() && old_normal.empty()) {
      if(!use_inline && !use_payload)points.resize(input_points.size());
      auto* destination=use_payload?payload.data():use_inline?inline_points.data():points.data();
      std::size_t i=0;bool negative=false,positive=false;
#if MVKIT_XLD_TRANSFORM_SSE2
      if(sse2) {
        const auto sign=_mm_set1_pd(-0.),infinity=_mm_set1_pd(std::numeric_limits<double>::infinity());
        const auto finite=[&](__m128d value){return _mm_cmplt_pd(_mm_andnot_pd(sign,value),infinity);};
        __m128d coefficients[9];for(int k=0;k<9;++k)coefficients[k]=_mm_set1_pd(m[k]);
        for(;input_points.size()-i>=2;i+=2) {
          const auto a=_mm_loadu_pd(&input_points[i].row),b=_mm_loadu_pd(&input_points[i+1].row);
          const auto rows=_mm_unpacklo_pd(a,b),columns=_mm_unpackhi_pd(a,b);
          if(_mm_movemask_pd(_mm_and_pd(finite(rows),finite(columns)))!=3)break;
          const auto apply=[&](int k){return _mm_add_pd(_mm_add_pd(_mm_mul_pd(coefficients[k],columns),
              _mm_mul_pd(coefficients[k+1],rows)),coefficients[k+2]);};
          const auto w=apply(6);
          if(_mm_movemask_pd(_mm_and_pd(finite(w),_mm_cmpneq_pd(w,_mm_setzero_pd())))!=3)break;
          const auto xx=_mm_div_pd(apply(0),w),yy=_mm_div_pd(apply(3),w);
          if(_mm_movemask_pd(_mm_and_pd(finite(xx),finite(yy)))!=3)break;
          const auto signs=_mm_movemask_pd(w);negative|=signs!=0;positive|=signs!=3;
          _mm_storeu_pd(&destination[i].row,_mm_unpacklo_pd(yy,xx));
          _mm_storeu_pd(&destination[i+1].row,_mm_unpackhi_pd(yy,xx));
        }
      }
#endif
      for(;i<input_points.size();++i) {
        const auto p=input_points[i];
        check(mv::core::is_finite(p.row) && mv::core::is_finite(p.col),op,"Contour","finite input points required");
        const auto w=m[6]*p.col+m[7]*p.row+m[8];
        check(mv::core::is_finite(w) && w!=0,op,"Matrix","point on projective horizon");
        const auto x=(m[0]*p.col+m[1]*p.row+m[2])/w,y=(m[3]*p.col+m[4]*p.row+m[5])/w;
        check(mv::core::is_finite(x) && mv::core::is_finite(y),op,"Matrix","nonfinite transformed point");
        destination[i]={y,x};negative|=std::signbit(w);positive|=!std::signbit(w);
      }
      // 所有点成功后才检查跨地平线，保留原输入错误优先级；无需保存每点分母。
      check(!(negative && positive),op,"Matrix","contour segment crosses projective horizon");
    } else if (affine && old_edge.empty() && old_normal.empty()) {
      std::size_t i = 0;
#if MVKIT_XLD_HAS_AVX2
      if(avx2) {
        if(use_payload)i=affine_owned_points_avx2(input_points,m.data(),{payload.data(),payload.size()},translation_mode);
        else if(use_inline)i=affine_inline_points_avx2(input_points,m.data(),
            {inline_points.data(),input_points.size()},translation_mode);
        else i=affine_points_avx2(input_points,m.data(),points,translation_mode);
      }
#endif
      // AVX2 整 tile 构造不二次初始化目标，仅尾部或失败重放需要 resize。
      if(!use_inline && !use_payload)points.resize(input_points.size());
      auto* destination=use_payload ? payload.data() : use_inline ? inline_points.data() : points.data();
#if MVKIT_XLD_TRANSFORM_SSE2
      if(sse2) {
        const auto x_coefficients = _mm_set_pd(m[0], m[3]);
        const auto y_coefficients = _mm_set_pd(m[1], m[4]);
        const auto translation = _mm_set_pd(m[2], m[5]);
        const auto sign = _mm_set1_pd(-0.0);
        const auto infinity = _mm_set1_pd(std::numeric_limits<double>::infinity());
        const auto store = [&](auto translation_only) {
          for (; input_points.size()-i >= 4; i+=4) {
            for (std::size_t lane=0;lane<4;++lane) {
            const auto p = _mm_loadu_pd(&input_points[i+lane].row);
            check(_mm_movemask_pd(_mm_cmplt_pd(_mm_andnot_pd(sign,p),infinity)) == 3,
                  op, "Contour", "finite input points required");
            const auto q = [&] {
              if constexpr (decltype(translation_only)::value) return _mm_add_pd(p,translation);
              else return _mm_add_pd(_mm_add_pd(
                  _mm_mul_pd(_mm_unpackhi_pd(p,p),x_coefficients),
                  _mm_mul_pd(_mm_unpacklo_pd(p,p),y_coefficients)),translation);
            }();
            check(_mm_movemask_pd(_mm_cmplt_pd(_mm_andnot_pd(sign,q),infinity)) == 3,
                  op, "Matrix", "nonfinite transformed point");
            _mm_storeu_pd(&destination[i+lane].row,q);
            }
          }
        };
        // 非零偏移消除省略两个零交叉项的带符号零歧义；一般仿射仍保留原算术序。
        if (translation_only)
          store(std::true_type{});
        else store(std::false_type{});
      }
#endif
      for (; i < input_points.size(); ++i) {
        const auto p = input_points[i];
        check(mv::core::is_finite(p.row) && mv::core::is_finite(p.col), op, "Contour",
              "finite input points required");
        const Point2d q=translation_only ? Point2d{p.row+m[5],p.col+m[2]} :
            Point2d{m[3]*p.col+m[4]*p.row+m[5], m[0]*p.col+m[1]*p.row+m[2]};
        check(mv::core::is_finite(q.row) && mv::core::is_finite(q.col), op, "Matrix",
              "nonfinite transformed point");
        destination[i]=q;
      }
    } else
    for (std::size_t i = 0; i < input_points.size(); ++i) {
      const auto p = input_points[i];
      check(mv::core::is_finite(p.row) && mv::core::is_finite(p.col), op, "Contour",
            "finite input points required");
      const double w = affine ? 1. : m[6] * p.col + m[7] * p.row + m[8];
      check(mv::core::is_finite(w) && w != 0, op, "Matrix",
            "point on projective horizon");
      if (!affine) denominator.push_back(w);
      const double xx = m[0] * p.col + m[1] * p.row + m[2],
                   yy = m[3] * p.col + m[4] * p.row + m[5];
      const double x = affine ? xx : xx / w, y = affine ? yy : yy / w;
      check(mv::core::is_finite(x) && mv::core::is_finite(y), op, "Matrix",
            "nonfinite transformed point");
      points.push_back({y, x});
      if (!old_edge.empty() || !old_normal.empty()) {
        double a = (m[0] - x * m[6]) / w, b = (m[1] - x * m[7]) / w,
               c = (m[3] - y * m[6]) / w, d = (m[4] - y * m[7]) / w;
        const double scale =
            std::max({std::abs(a), std::abs(b), std::abs(c), std::abs(d)});
        check(std::isfinite(scale) && scale > 0, op, "Matrix",
              "nonfinite Jacobian");
        a /= scale;
        b /= scale;
        c /= scale;
        d /= scale;
        const double determinant = a * d - b * c;
        check(std::isfinite(determinant) && determinant != 0, op, "Matrix",
              "degenerate Jacobian");
        const auto covector = [&](double nx, double ny, bool oriented) {
          double xx = d * nx - c * ny, yy = -b * nx + a * ny;
          // 梯度保留对比极性，轮廓法向保持相对遍历侧，因此反射翻转协向量符号。
          if (!oriented && determinant < 0) {
            xx = -xx;
            yy = -yy;
          }
          check(std::isfinite(xx) && std::isfinite(yy) && (xx != 0 || yy != 0),
                op, "Attributes", "degenerate transformed direction");
          return std::array<double, 2>{xx, yy};
        };
        if (!old_edge.empty()) {
          check(std::isfinite(old_edge[i]), op, "EdgeDirection",
                "finite angle required");
          auto v =
              covector(std::cos(old_edge[i]), -std::sin(old_edge[i]), false);
          edge.push_back(angle(-v[1], v[0]));
        }
        if (!old_normal.empty()) {
          check(std::isfinite(old_normal[i]), op, "NormalAngle",
                "finite angle required");
          auto v =
              covector(std::sin(old_normal[i]), std::cos(old_normal[i]), true);
          normal.push_back(angle(v[0], v[1]));
        }
      }
    }
    for (std::size_t i = 1; i < denominator.size(); ++i)
      check(std::signbit(denominator[i - 1]) == std::signbit(denominator[i]),
            op, "Matrix", "contour segment crosses projective horizon");
    if (contour.closed() && denominator.size() > 1)
      check(
          std::signbit(denominator.front()) == std::signbit(denominator.back()),
          op, "Matrix", "closing segment crosses projective horizon");
    auto approximation = contour.approximation();
    if (approximation != ContourApproximation::Line) {
      if (!affine)
        approximation = ContourApproximation::Unknown;
      else if (approximation == ContourApproximation::Circle) {
        const double u = std::hypot(m[0], m[3]), v = std::hypot(m[1], m[4]);
        const double dot = m[0] * m[1] + m[3] * m[4];
        if (std::abs(u - v) > 1e-12 * std::max(u, v) ||
            std::abs(dot) > 1e-12 * u * v)
          approximation = ContourApproximation::Ellipse;
      }
    }
    if(use_payload)return XldContour::from_owned_point_payload(
        std::move(payload),contour.closed(),approximation,
        contour.point_attribute(XldPointAttribute::Response),pooled_metadata,metadata.get());
    if(use_inline)return XldContour::from_inline_points(
        {inline_points.data(),input_points.size()},contour.closed(),approximation,
        contour.point_attribute(XldPointAttribute::Response),pooled_metadata,metadata.get());
    return XldContour::from_owned_points_and_attributes(
        std::move(points), contour.closed(), approximation,
        contour.point_attribute(XldPointAttribute::Response), edge, normal,pooled_metadata,metadata.get());
  },pooled_metadata,metadata.get());
}
}  // namespace
mv::core::XldContours affine_trans_contour_xld(
    const mv::core::XldContours& input, const mv::core::Tuple& matrix) {
  return transform(input, matrix, true);
}
mv::core::XldContours projective_trans_contour_xld(
    const mv::core::XldContours& input, const mv::core::Tuple& matrix) {
  return transform(input, matrix, false);
}
mv::core::XldContours close_contours_xld(const mv::core::XldContours& input) {
  return map(input, [](const XldContour& c) {
    if (c.empty() || c.closed()) return c;
    return XldContour::from_points_and_attributes(
        c.points(), true, ContourApproximation::Unknown,
        c.point_attribute(XldPointAttribute::Response),
        c.point_attribute(XldPointAttribute::EdgeDirection),
        c.point_attribute(XldPointAttribute::NormalAngle));
  });
}
}  // namespace mv::xld
