#include <mvkit/c/mv_stereo.h>

#include <reconstruction/stereo/disparity_protocol.hpp>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"

extern "C" ::MvStatus mv_binocular_disparity_ms(
    MvObject ImageRect1, MvObject ImageRect2, MvObject* Disparity, MvObject* Score,
    ::MvTuple MinDisparity, ::MvTuple MaxDisparity, ::MvTuple SurfaceSmoothing,
    ::MvTuple EdgeSmoothing, ::MvTuple GenParamName, ::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_binocular_disparity_ms";
    mv::c_api::require_output(Disparity, op, "Disparity");
    mv::c_api::require_output(Score, op, "Score");
    const auto a = reinterpret_cast<std::uintptr_t>(Disparity);
    const auto b = reinterpret_cast<std::uintptr_t>(Score);
    if ((a < b ? b - a : a - b) < sizeof(MvObject))
      throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue,
                                op, "Output", -1, "output ranges must be distinct");
    mv::c_api::require_iconic(ImageRect1, op, "ImageRect1");
    mv::c_api::require_iconic(ImageRect2, op, "ImageRect2");
    // 任一 iconic 数组为空时 HALCON 跳过控制验证。
    mv::reconstruction::StereoDisparityImages result;
    if (ImageRect1->value.empty() || ImageRect2->value.empty()) {
      ImageRect1->value.require_kind(mv::core::IconicKind::Image, op);
      ImageRect2->value.require_kind(mv::core::IconicKind::Image, op);
    } else {
      const mv::c_api::TupleInput lo(MinDisparity, op, "MinDisparity"),
          hi(MaxDisparity, op, "MaxDisparity"), p1(SurfaceSmoothing, op, "SurfaceSmoothing"),
          p2(EdgeSmoothing, op, "EdgeSmoothing"), n(GenParamName, op, "GenParamName"),
          v(GenParamValue, op, "GenParamValue");
      result = mv::reconstruction::binocular_disparity_ms_protocol(
          ImageRect1->value, ImageRect2->value, lo.materialize(), hi.materialize(), p1.materialize(),
          p2.materialize(), n.materialize(), v.materialize());
    }
    auto disparity = mv::c_api::make_image_handle(std::move(result.disparity), op);
    auto score = mv::c_api::make_image_handle(std::move(result.score), op);
    *Disparity = disparity.release();
    *Score = score.release();
  });
}
