#include "api_bridge.hpp"
#include <mvkit/c/mv_stereo.h>

#include <mvkit/cpp/mv_stereo.hpp>
#include <utility>

namespace mv {
void BinocularDisparity(const MvObject& ImageRect1, const MvObject& ImageRect2, MvObject* Disparity,
                        MvObject* Score, const MvTuple& Method, const MvTuple& MaskWidth,
                        const MvTuple& MaskHeight, const MvTuple& TextureThresh, const MvTuple& MinDisparity,
                        const MvTuple& MaxDisparity, const MvTuple& NumLevels, const MvTuple& ScoreThresh,
                        const MvTuple& Filter, const MvTuple& SubDisparity) {
  constexpr char op[] = "BinocularDisparity";
  cpp_api::require_output(Disparity, op);
  cpp_api::require_output(Score, op);
  if (Disparity == Score)
    throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidValue, op, "Output", -1,
                      "output pointers must be distinct");
  struct Owners {
    ::MvObject d{}, s{};
    ~Owners() {
      if (d) (void)mv_clear_obj(d);
      if (s) (void)mv_clear_obj(s);
    }
  } owners;
  cpp_api::check_status(
      mv_binocular_disparity(ImageRect1.NativeHandle(), ImageRect2.NativeHandle(), &owners.d, &owners.s,
                             Method.ControlInput(), MaskWidth.ControlInput(), MaskHeight.ControlInput(),
                             TextureThresh.ControlInput(), MinDisparity.ControlInput(),
                             MaxDisparity.ControlInput(), NumLevels.ControlInput(),
                             ScoreThresh.ControlInput(), Filter.ControlInput(), SubDisparity.ControlInput()),
      op);
  auto d = detail::MvObjectAccess::Adopt(std::exchange(owners.d, nullptr)),
       s = detail::MvObjectAccess::Adopt(std::exchange(owners.s, nullptr));
  detail::MvObjectAccess::CheckOutput(Disparity, d);
  detail::MvObjectAccess::CheckOutput(Score, s);
  detail::MvObjectAccess::Commit(Disparity, std::move(d));
  detail::MvObjectAccess::Commit(Score, std::move(s));
}
MvImage MvImage::BinocularDisparity(const MvImage& other, MvImage* score, const MvString& method,
                                    std::int64_t width, std::int64_t height, const MvTuple& texture,
                                    std::int64_t minimum, std::int64_t maximum, std::int64_t levels,
                                    const MvTuple& threshold, const MvTuple& filter,
                                    const MvString& sub) const {
  MvImage result;
  mv::BinocularDisparity(*this, other, &result, score, method, width, height, texture, minimum, maximum,
                         levels, threshold, filter, sub);
  return result;
}
MvImage MvImage::BinocularDisparity(const MvImage& other, MvImage* score, const MvString& method,
                                    std::int64_t width, std::int64_t height, double texture,
                                    std::int64_t minimum, std::int64_t maximum, std::int64_t levels,
                                    double threshold, const MvString& filter, const MvString& sub) const {
  return BinocularDisparity(other, score, method, width, height, MvTuple(texture), minimum, maximum, levels,
                            MvTuple(threshold), MvTuple(filter), sub);
}
MvImage MvImage::BinocularDisparity(const MvImage& other, MvImage* score, const char* method,
                                    std::int64_t width, std::int64_t height, double texture,
                                    std::int64_t minimum, std::int64_t maximum, std::int64_t levels,
                                    double threshold, const char* filter, const char* sub) const {
  return BinocularDisparity(other, score, MvString(method), width, height, texture, minimum, maximum, levels,
                            threshold, MvString(filter), MvString(sub));
}
MvImage MvImage::BinocularDisparity(const MvImage& other, MvImage* score, const wchar_t* method,
                                    std::int64_t width, std::int64_t height, double texture,
                                    std::int64_t minimum, std::int64_t maximum, std::int64_t levels,
                                    double threshold, const wchar_t* filter, const wchar_t* sub) const {
  return BinocularDisparity(other, score, MvString(method), width, height, texture, minimum, maximum, levels,
                            threshold, MvString(filter), MvString(sub));
}
}  // namespace mv
