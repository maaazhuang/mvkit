#pragma once
#include <array>
#include <mutex>
#include <reconstruction/camera_setup.hpp>
#include <vector>
namespace mv::reconstruction {
struct SparseStereoPoints {
  core::Tuple x, y, z, covariance, indices;
};
/** 相机配置快照，外部句柄副本共享状态，清理模型使全部副本失效。 */
class SparseStereoModel {
 public:
  explicit SparseStereoModel(const CameraSetupModel& setup, std::string_view method, const core::Tuple& names,
                             const core::Tuple& values);
  void set_parameters(const core::Tuple& names, const core::Tuple& values);
  void clear();
  void require_open(const char* operation) const;
  [[nodiscard]] SparseStereoPoints reconstruct(const core::Tuple& rows, const core::Tuple& columns,
                                               const core::Tuple& covariance, const core::Tuple& cameras,
                                               const core::Tuple& points) const;

 private:
  struct Camera {
    calibration::CameraParameters parameters;
    Pose3d pose;
  };
  void set_parameters_unlocked(const core::Tuple& names, const core::Tuple& values);
  std::vector<Camera> cameras_;
  std::array<double, 6> bounds_{{0, 0, 0, 0, 0, -1}};
  bool persistence_{false}, closed_{false};
  mutable std::mutex mutex_;
};
}  // namespace mv::reconstruction
