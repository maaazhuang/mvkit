#include <xld/operators.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(_M_X64) || defined(__x86_64__)
#include <emmintrin.h>
#endif

namespace mv::xld {
namespace {

double segment_length(const mv::core::Point2d& a,
                      const mv::core::Point2d& b) noexcept {
  const double dr = a.row - b.row;
  const double dc = a.col - b.col;
  const double squared = dr * dr + dc * dc;
  // 常见有限可表示情况仅需一次开方；溢出、下溢及非有限差保留 hypot。
  if (squared >= 1e-300 && squared < std::numeric_limits<double>::infinity())
    return std::sqrt(squared);
  if (dr == 0.0 && dc == 0.0) return 0.0;
  return std::hypot(dr, dc);
}

}  // namespace

double length_xld(const mv::core::XldContour& contour) noexcept {
  const auto points = contour.points();
  double sum = 0;
#if defined(_M_X64) || defined(__x86_64__)
  std::size_t i = 1;
  for (; i + 1 < points.size(); i += 2) {
    const auto rows = _mm_set_pd(points[i + 1].row - points[i].row,
                                 points[i].row - points[i - 1].row);
    const auto cols = _mm_set_pd(points[i + 1].col - points[i].col,
                                 points[i].col - points[i - 1].col);
    const auto squares = _mm_add_pd(_mm_mul_pd(rows, rows), _mm_mul_pd(cols, cols));
    const auto lengths = _mm_sqrt_pd(squares);
    alignas(16) double squared[2], value[2];
    _mm_store_pd(squared, squares);
    _mm_store_pd(value, lengths);
    for (std::size_t lane = 0; lane < 2; ++lane)
      sum += squared[lane] >= 1e-300 &&
                     squared[lane] < std::numeric_limits<double>::infinity()
                 ? value[lane]
                 : segment_length(points[i + lane], points[i + lane - 1]);
  }
  for (; i < points.size(); ++i)
    sum += segment_length(points[i], points[i - 1]);
#else
  for (std::size_t i = 1; i < points.size(); ++i)
    sum += segment_length(points[i], points[i - 1]);
#endif
  if (contour.closed() && points.size() > 1)
    sum += segment_length(points.front(), points.back());
  return sum;
}

std::vector<double> length_xld(const mv::core::XldContours& contours) {
  std::vector<double> lengths;
  lengths.reserve(contours.contour_count());
  for (std::size_t i = 0; i < contours.contour_count(); ++i)
    lengths.push_back(length_xld(contours.contour(i)));
  return lengths;
}

}  // namespace mv::xld
