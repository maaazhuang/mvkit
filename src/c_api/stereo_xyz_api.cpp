#include <mvkit/c/mv_stereo.h>

#include <cmath>
#include <reconstruction/stereo/rectified_xyz.hpp>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
namespace {
constexpr auto op = "mv_disparity_image_to_xyz";
template <class F>
auto parse(const mv::c_api::TupleInput& input, const char* name, F&& f) {
  try {
    // 拥有型控制元组在解析期间保持有效，模型字段由解析结果独立持有。
    if(const auto* owned=input.owned_tuple())return f(*owned);
    return f(input.materialize());
  } catch (const mv::core::Exception& e) {
    throw mv::core::Exception(e.status(), e.error_code(), op, name, e.parameter_index(), e.message());
  }
}
}  // namespace
extern "C" ::MvStatus mv_disparity_image_to_xyz(::MvObject Disparity, ::MvObject* X, ::MvObject* Y,
                                              ::MvObject* Z, ::MvTuple CamParamRect1,
                                              ::MvTuple CamParamRect2, ::MvTuple RelPoseRect) {
  return mv::c_api::translate([&] {
    for (auto p : {X, Y, Z}) mv::c_api::require_output(p, op);
    if (X == Y || X == Z || Y == Z)
      throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue, op,
                                "Output", -1, "output pointers must be distinct");
    mv::c_api::require_iconic(Disparity, op, "Disparity");
    Disparity->value.require_kind(mv::core::IconicKind::Image, op);
    const mv::c_api::TupleInput first(CamParamRect1, op, "CamParamRect1"),
        second(CamParamRect2, op, "CamParamRect2"), relative(RelPoseRect, op, "RelPoseRect");
    std::vector<mv::core::Image> xs, ys, zs;
    if (!Disparity->value.empty()) {
      mv::reconstruction::RectifiedXyzGeometry geometry;
      geometry.first = parse(first, "CamParamRect1", mv::reconstruction::parse_rectified_camera);
      geometry.second = parse(second, "CamParamRect2", mv::reconstruction::parse_rectified_camera);
      geometry.pose = parse(relative, "RelPoseRect", [&](const mv::core::Tuple& t) {
        return mv::reconstruction::parse_rectified_pose(t, &geometry.nonfinite_rotation);
      });
      const auto t = geometry.pose.translation;
      geometry.baseline = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
      try {
        mv::reconstruction::validate_rectified_xyz_geometry(geometry);
      } catch (const mv::core::Exception& e) {
        throw mv::core::Exception(e.status(), e.error_code(), op, e.parameter(), e.parameter_index(),
                                  e.message());
      }
      xs.reserve(Disparity->value.size());
      ys.reserve(Disparity->value.size());
      zs.reserve(Disparity->value.size());
      for (std::size_t i = 0; i < Disparity->value.size(); ++i) {
        try {
          auto images =
              mv::reconstruction::rectified_disparity_xyz(Disparity->value.at(i).as_image(), geometry);
          xs.push_back(std::move(images.x));
          ys.push_back(std::move(images.y));
          zs.push_back(std::move(images.z));
        } catch (const mv::core::Exception& e) {
          throw mv::core::Exception(e.status(), e.error_code(), op, "Disparity", i, e.message());
        }
      }
    }
    auto x = mv::c_api::make_image_handle(mv::core::make_image_tuple(std::move(xs)), op),
         y = mv::c_api::make_image_handle(mv::core::make_image_tuple(std::move(ys)), op),
         z = mv::c_api::make_image_handle(mv::core::make_image_tuple(std::move(zs)), op);
    *X = x.release();
    *Y = y.release();
    *Z = z.release();
  });
}
