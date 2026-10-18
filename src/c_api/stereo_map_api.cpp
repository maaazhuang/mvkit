#include <mvkit/c/mv_stereo.h>

#include <reconstruction/stereo/binocular_map.hpp>

#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"

extern "C" ::MvStatus mv_gen_binocular_rectification_map(MvObject* Map1, MvObject* Map2, ::MvTuple CamParam1,
                                                       ::MvTuple CamParam2, ::MvTuple RelPose,
                                                       ::MvTuple SubSampling, ::MvTuple Method,
                                                       ::MvTuple MapType, MvTuple* CamParamRect1,
                                                       MvTuple* CamParamRect2, MvTuple* CamPoseRect1,
                                                       MvTuple* CamPoseRect2, MvTuple* RelPoseRect) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_gen_binocular_rectification_map";
    const std::array<const void*, 7> outputs{Map1,         Map2,         CamParamRect1, CamParamRect2,
                                             CamPoseRect1, CamPoseRect2, RelPoseRect};
    for (std::size_t i = 0; i < outputs.size(); ++i) {
      mv::c_api::require_output(outputs[i], op);
      const auto a = reinterpret_cast<std::uintptr_t>(outputs[i]);
      for (std::size_t j = 0; j < i; ++j) {
        const auto b = reinterpret_cast<std::uintptr_t>(outputs[j]);
        if ((a < b ? b - a : a - b) < sizeof(void*))
          throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue, op,
                                    "Output", i, "output ranges must be distinct");
      }
    }
    mv::c_api::ControlOutputs<5> controls(
        op, {CamParamRect1, CamParamRect2, CamPoseRect1, CamPoseRect2, RelPoseRect});
    const mv::c_api::TupleInput a(CamParam1, op, "CamParam1"), b(CamParam2, op, "CamParam2"),
        p(RelPose, op, "RelPose"), s(SubSampling, op, "SubSampling"), m(Method, op, "Method"),
        t(MapType, op, "MapType");
    if (m.size() != 1) m.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one method required");
    if (t.size() != 1) t.fail(mv::core::ErrorCode::InvalidCardinality, -1, "exactly one map type required");
    auto result = mv::reconstruction::binocular_rectification_maps(
        a.materialize(), b.materialize(), p.materialize(), s.scalar_number(), m.string(0), t.string(0));
    auto map1 = mv::c_api::make_image_handle(std::move(result.maps[0])),
         map2 = mv::c_api::make_image_handle(std::move(result.maps[1]));
    controls[0] = std::move(result.cameras[0]);
    controls[1] = std::move(result.cameras[1]);
    controls[2] = std::move(result.camera_poses[0]);
    controls[3] = std::move(result.camera_poses[1]);
    controls[4] = std::move(result.relative_pose);
    *Map1 = map1.release();
    *Map2 = map2.release();
    controls.Commit();
  });
}
