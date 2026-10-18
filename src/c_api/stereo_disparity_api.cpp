#include <mvkit/c/mv_stereo.h>

#include <reconstruction/stereo/classic_disparity.hpp>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"
extern "C" ::MvStatus mv_binocular_disparity(MvObject ImageRect1, MvObject ImageRect2, MvObject* Disparity,
                                           MvObject* Score, ::MvTuple Method, ::MvTuple MaskWidth,
                                           ::MvTuple MaskHeight, ::MvTuple TextureThresh,
                                           ::MvTuple MinDisparity, ::MvTuple MaxDisparity,
                                           ::MvTuple NumLevels, ::MvTuple ScoreThresh,
                                           ::MvTuple Filter, ::MvTuple SubDisparity) {
  return mv::c_api::translate([&] {
    constexpr auto op = "mv_binocular_disparity";
    mv::c_api::require_output(Disparity, op, "Disparity");
    mv::c_api::require_output(Score, op, "Score");
    const auto x = reinterpret_cast<std::uintptr_t>(Disparity), y = reinterpret_cast<std::uintptr_t>(Score);
    if ((x < y ? y - x : x - y) < sizeof(void*))
      throw mv::core::Exception(mv::core::Status::InvalidArgument, mv::core::ErrorCode::InvalidValue, op,
                                "Output", -1, "output ranges must be distinct");
    const mv::c_api::TupleInput method(Method, op, "Method"), width(MaskWidth, op, "MaskWidth"),
        height(MaskHeight, op, "MaskHeight"), texture(TextureThresh, op, "TextureThresh"),
        minimum(MinDisparity, op, "MinDisparity"), maximum(MaxDisparity, op, "MaxDisparity"),
        levels(NumLevels, op, "NumLevels"), threshold(ScoreThresh, op, "ScoreThresh"),
        filter(Filter, op, "Filter"), sub(SubDisparity, op, "SubDisparity");
    const auto p = mv::reconstruction::classic_disparity_parameters(
        method.materialize(), width.materialize(), height.materialize(), texture.materialize(),
        minimum.materialize(), maximum.materialize(), levels.materialize(), threshold.materialize(),
        filter.materialize(), sub.materialize());
    mv::c_api::require_iconic(ImageRect1, op, "ImageRect1");
    mv::c_api::require_iconic(ImageRect2, op, "ImageRect2");
    ImageRect1->value.require_kind(mv::core::IconicKind::Image, op);
    ImageRect2->value.require_kind(mv::core::IconicKind::Image, op);
    mv::core::IconicTuple disparities, scores;
    // HALCON 24.11.1.0 将此算子视为单图像对，忽略后续 iconic 对象。
    if (!ImageRect1->value.empty() && !ImageRect2->value.empty()) {
      auto result = mv::reconstruction::classic_binocular_disparity(ImageRect1->value.at(0).as_image(),
                                                                    ImageRect2->value.at(0).as_image(), p);
      disparities = mv::core::IconicTuple{mv::core::IconicValue::from_image(std::move(result.disparity))};
      scores = mv::core::IconicTuple{mv::core::IconicValue::from_image(std::move(result.score))};
    }
    auto d = mv::c_api::make_image_handle(std::move(disparities), op),
         s = mv::c_api::make_image_handle(std::move(scores), op);
    *Disparity = d.release();
    *Score = s.release();
  });
}
