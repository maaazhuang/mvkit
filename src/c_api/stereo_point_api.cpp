#include <mvkit/c/mv_stereo.h>

#include <reconstruction/stereo/rectified_points.hpp>

#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "tuple_input.hpp"
namespace {
constexpr const char* op = "mv_disparity_to_point_3d";
template <class F>
auto parse(const mv::c_api::TupleInput& input, const char* parameter, F&& f) {
  try {
    // 拥有型控制元组在解析期间保持有效，模型字段由解析结果独立持有。
    if(const auto* owned=input.owned_tuple())return f(*owned);
    return f(input.materialize());
  } catch (const mv::core::Exception& e) {
    throw mv::core::Exception(e.status(), e.error_code(), op, parameter, e.parameter_index(), e.message());
  }
}
}  // namespace
extern "C" ::MvStatus mv_disparity_to_point_3d(::MvTuple CamParamRect1, ::MvTuple CamParamRect2,
                                             ::MvTuple RelPoseRect, ::MvTuple Row1, ::MvTuple Col1,
                                             ::MvTuple Disparity, ::MvTuple* X, ::MvTuple* Y,
                                             ::MvTuple* Z) {
  return mv::c_api::translate([&] {
    mv::c_api::ControlOutputs<3> output(op, {X, Y, Z});
    const mv::c_api::TupleInput a(CamParamRect1, op, "CamParamRect1"), b(CamParamRect2, op, "CamParamRect2"),
        p(RelPoseRect, op, "RelPoseRect"), r(Row1, op, "Row1"), c(Col1, op, "Col1"),
        d(Disparity, op, "Disparity");
    auto ca = parse(a, "CamParamRect1", mv::reconstruction::parse_rectified_camera);
    auto cb = parse(b, "CamParamRect2", mv::reconstruction::parse_rectified_camera);
    bool nonfinite_rotation = false;
    auto pose = parse(p, "RelPoseRect", [&](const mv::core::Tuple& v) {
      return mv::reconstruction::parse_rectified_pose(v, &nonfinite_rotation);
    });
    if (!r.size()) r.fail(mv::core::ErrorCode::InvalidCardinality, -1, "nonempty coordinate tuple required");
    if (c.size() != r.size())
      c.fail(mv::core::ErrorCode::InvalidCardinality, -1, "coordinate tuple lengths must match");
    if (d.size() != r.size())
      d.fail(mv::core::ErrorCode::InvalidCardinality, -1, "coordinate tuple lengths must match");
    for (std::size_t i = 0; i < r.size(); ++i) {
      const auto row = r.number(i), col = c.number(i), disparity = d.number(i);
      mv::Point3d point;
      try {
        point = mv::reconstruction::rectified_disparity_point(ca, cb, pose, nonfinite_rotation, row, col,
                                                              disparity);
      } catch (const mv::core::Exception& e) {
        throw mv::core::Exception(e.status(), e.error_code(), op, "Disparity", i, e.message());
      }
      output[0].append(point.x);
      output[1].append(point.y);
      output[2].append(point.z);
    }
    output.Commit();
  });
}
