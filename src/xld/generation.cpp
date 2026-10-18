#include <xld/operators.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <cmath>
#include <memory_resource>
#include <numbers>
#include <vector>
#include <core/exception.hpp>

namespace mv::xld {
using mv::core::XldContour; using mv::core::Exception; using mv::core::Point2d; using mv::core::Status; using mv::core::XldContours;
using mv::runtime::ExecutionContext;
XldContours gen_contour_polygon_xld(std::span<const Point2d> points,bool closed){XldContours out;out.append(XldContour::from_points(points,closed));return out;}
XldContours gen_circle_contour_xld(double row, double col, double radius,
                                    double start_phi, double end_phi,
                                    PointOrder order, double resolution) {
 constexpr auto op = "gen_circle_contour_xld";
 if (!std::isfinite(row) || !std::isfinite(col) || !std::isfinite(radius) ||
     radius <= 0 || !std::isfinite(start_phi) || !std::isfinite(end_phi) ||
     !std::isfinite(resolution) || resolution <= 0 ||
     (order != PointOrder::Positive && order != PointOrder::Negative))
   throw Exception(Status::InvalidArgument, op, "invalid circle parameters");
 constexpr double tau = 2 * std::numbers::pi_v<double>;
 const double raw = end_phi - start_phi;
 if (!std::isfinite(raw))
   throw Exception(Status::InvalidArgument, op, "angle difference overflow");
 double delta = std::fmod(raw, tau);
 if (order == PointOrder::Positive) {
   if (delta < 0) delta += tau;
   if (delta == 0 && raw > 0) delta = tau;
 } else {
   if (delta > 0) delta -= tau;
   if (delta == 0 && raw < 0) delta = -tau;
 }
 // Resolution 为弦长，保持步长直至精确末点；将残余角均分到全部段会改变每个顶点。
 const double step = 2 * std::asin(std::min(1., (resolution / radius) * .5));
 const double requested = delta == 0 ? 0 : std::ceil(std::abs(delta) / step);
 if (!std::isfinite(requested) || requested > 10000000)
   throw Exception(Status::InvalidArgument, op, "sample budget exceeds ten million points");
 const auto segments = static_cast<std::size_t>(requested);
 const bool closed = std::abs(delta) == tau;
 const double start = std::fmod(start_phi, tau);
 ExecutionContext context;
 std::pmr::vector<Point2d> points{context.scratch_resource()};
 points.reserve(segments + 1);
 for (std::size_t i = 0; i < segments; ++i) {
   const double phi = start + std::copysign(step * i, delta);
   points.push_back({row - radius * std::sin(phi), col + radius * std::cos(phi)});
 }
 const double end = start + delta;
 points.push_back(closed ? points.front()
                         : Point2d{row - radius * std::sin(end), col + radius * std::cos(end)});
 XldContours out;
 out.append(XldContour::from_points_and_attributes(points, closed,
                                                   mv::core::ContourApproximation::Circle));
 return out;
}
} // namespace mv::xld
