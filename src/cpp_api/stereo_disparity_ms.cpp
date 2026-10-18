#include "api_bridge.hpp"
#include <mvkit/c/mv_stereo.h>
#include <mvkit/cpp/mv_stereo.hpp>

#include <utility>


namespace mv {
void BinocularDisparityMs(const MvObject& a, const MvObject& b, MvObject* disparity, MvObject* score,
                          const MvTuple& lo, const MvTuple& hi, const MvTuple& p1, const MvTuple& p2,
                          const MvTuple& names, const MvTuple& values) {
  constexpr char op[] = "BinocularDisparityMs";
  cpp_api::require_output(disparity, op);
  cpp_api::require_output(score, op);
  if (disparity == score)
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
      mv_binocular_disparity_ms(a.NativeHandle(), b.NativeHandle(), &owners.d, &owners.s,
                                lo.ControlInput(), hi.ControlInput(), p1.ControlInput(), p2.ControlInput(),
                                names.ControlInput(), values.ControlInput()),
      op);
  auto d = detail::MvObjectAccess::Adopt(std::exchange(owners.d, nullptr)),
       s = detail::MvObjectAccess::Adopt(std::exchange(owners.s, nullptr));
  detail::MvObjectAccess::CheckOutput(disparity, d);
  detail::MvObjectAccess::CheckOutput(score, s);
  detail::MvObjectAccess::Commit(disparity, std::move(d));
  detail::MvObjectAccess::Commit(score, std::move(s));
}

MvImage MvImage::BinocularDisparityMs(const MvImage& other, MvImage* score, std::int64_t lo,
                                      std::int64_t hi, std::int64_t p1, std::int64_t p2,
                                      const MvTuple& names, const MvTuple& values) const {
  MvImage disparity;
  mv::BinocularDisparityMs(*this, other, &disparity, score, lo, hi, p1, p2, names, values);
  return disparity;
}
MvImage MvImage::BinocularDisparityMs(const MvImage& other, MvImage* score, std::int64_t lo,
                                      std::int64_t hi, std::int64_t p1, std::int64_t p2,
                                      const MvString& names, const MvString& values) const {
  return BinocularDisparityMs(other, score, lo, hi, p1, p2, MvTuple(names), MvTuple(values));
}
MvImage MvImage::BinocularDisparityMs(const MvImage& other, MvImage* score, std::int64_t lo,
                                      std::int64_t hi, std::int64_t p1, std::int64_t p2,
                                      const char* names, const char* values) const {
  return BinocularDisparityMs(other, score, lo, hi, p1, p2, MvTuple(names), MvTuple(values));
}
MvImage MvImage::BinocularDisparityMs(const MvImage& other, MvImage* score, std::int64_t lo,
                                      std::int64_t hi, std::int64_t p1, std::int64_t p2,
                                      const wchar_t* names, const wchar_t* values) const {
  return BinocularDisparityMs(other, score, lo, hi, p1, p2, MvString(names), MvString(values));
}
}  // namespace mv
