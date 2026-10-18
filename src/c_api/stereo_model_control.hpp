#pragma once
#include <core/handle.hpp>
#include <reconstruction/stereo/sparse_model.hpp>
namespace mv::c_api {
class StereoModelObject final : public core::Object {
 public:
  static constexpr core::TypeId kTypeId = core::make_type_id("mvkit.control.StereoModel");
  StereoModelObject(const reconstruction::CameraSetupModel& setup, std::string_view method,
                    const core::Tuple& names, const core::Tuple& values)
      : value(setup, method, names, values) {}
  core::TypeId type_id() const noexcept override { return kTypeId; }
  bool is_valid() const noexcept override { return valid_; }
  void clear() override { value.clear(); valid_=false; }
  reconstruction::SparseStereoModel value;
 private:
  bool valid_{true};
};
}  // namespace mv::c_api
