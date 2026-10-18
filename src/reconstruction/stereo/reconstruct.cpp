#include <algorithm>
#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <cstddef>
#include <limits>
#include <optional>
#include <reconstruction/geometry/triangulation.hpp>
#include <reconstruction/quality.hpp>
#include <reconstruction/stereo/disparity.hpp>
#include <reconstruction/stereo/reconstruct.hpp>

namespace mv::reconstruction {
namespace {

// 构造上下文不计算几何，全部像素无效时保持原行为。
class StereoPointContext {
 public:
  explicit StereoPointContext(const StereoRectification& rect) : rect_(rect) {}
  Point3d point(double row, double column, double disparity, double max_ray_gap) {
    if (!std::isfinite(row) || !std::isfinite(column) || !std::isfinite(disparity))
      throw core::Exception(core::Status::InvalidArgument, "disparity_to_point_3d",
                            "row, column and disparity must be finite");
    const auto ray0 = read_ray(rect_.camera(0), row, column, 0);
    const auto ray1 = read_ray(rect_.camera(1), row, column + disparity, 1);
    const auto& pose = rect_.device_1_in_device_0_pose();
    if (!rotation_) rotation_ = calibration::rotation_matrix(pose.rotation);
    const auto origin = rotate(ray1.origin);
    const Ray3d transformed{{origin.x + pose.translation.x, origin.y + pose.translation.y,
                             origin.z + pose.translation.z},
                            rotate(ray1.direction)};
    return triangulate_rays(Ray3d{ray0.origin, ray0.direction}, transformed, max_ray_gap).point;
  }

 private:
  Point3d rotate(const Point3d& p) const {
    const auto& r = *rotation_;
    return {r[0] * p.x + r[1] * p.y + r[2] * p.z, r[3] * p.x + r[4] * p.y + r[5] * p.z,
            r[6] * p.x + r[7] * p.y + r[8] * p.z};
  }
  calibration::Ray3d read_ray(const calibration::CameraParameters& params, double row, double col,
                              int camera) {
    // 类型查询延迟执行，不能让射线错误先于原 ray-zero 检查。
    if (!camera_known_[camera]) {
      division_[camera] = std::get_if<calibration::AreaScanDivisionParameters>(&params);
      camera_known_[camera] = true;
    }
    if (const auto* p = division_[camera]) {
      if (!std::isfinite(row) || !std::isfinite(col))
        throw core::Exception(core::Status::InvalidArgument, "image_point_to_ray",
                              "image point must be finite");
      const double u = (col - p->cx) * p->sx, v = (row - p->cy) * p->sy;
      if (!std::isfinite(u) || !std::isfinite(v))
        throw core::Exception(core::Status::InvalidArgument, "undistort_metric_point",
                              "metric point must be finite");
      const double r2 = u * u + v * v;
      const double denom = 1.0 + p->kappa * r2;
      if (!std::isfinite(denom) || std::abs(denom) < 1e-15)
        throw core::Exception(core::Status::InvalidState, "undistort_metric_point",
                              "division distortion is outside its valid domain");
      // kappa==0 也保留 r2 与除法，极大有限度量坐标仍可能令 r2 溢出。
      const double ideal_u = u / denom, ideal_v = v / denom;
      return {{0.0, 0.0, 0.0}, {ideal_u / p->focus, ideal_v / p->focus, 1.0}};
    }

    return calibration::image_point_to_ray(params, {row, col});
  }
  const StereoRectification& rect_;
  std::optional<std::array<double, 9>> rotation_;
  std::array<bool, 2> camera_known_{};
  std::array<const calibration::AreaScanDivisionParameters*, 2> division_{};
};

float image_float(const core::Image& image, std::size_t row, std::size_t col) {
  return reinterpret_cast<const float*>(static_cast<const std::byte*>(image.data()) +
                                        row * image.stride())[col];
}

void set_image_float(core::Image& image, std::size_t row, std::size_t col, float value) {
  reinterpret_cast<float*>(static_cast<std::byte*>(image.mutable_data()) +
                           row * image.stride())[col] = value;
}

void validate_disparity_image(const StereoRectification& rectification,
                              const core::Image& disparity, const char* operation) {
  if (disparity.pixel_type() != core::PixelType::Float32) {
    throw core::Exception(core::Status::InvalidArgument, operation,
                          "disparity image must be Float32");
  }
  const auto& common = calibration::common_parameters(rectification.camera(0));
  if (disparity.width() != static_cast<std::size_t>(common.image_width) ||
      disparity.height() != static_cast<std::size_t>(common.image_height)) {
    throw core::Exception(core::Status::InvalidArgument, operation,
                          "disparity image dimensions must match rectified camera 0");
  }
}

}  // namespace

Point3d disparity_to_point_3d(const StereoRectification& rectification, double row, double column,
                              double disparity, double max_ray_gap) {
  if (!std::isfinite(row) || !std::isfinite(column) || !std::isfinite(disparity)) {
    throw core::Exception(core::Status::InvalidArgument, "disparity_to_point_3d",
                          "row, column and disparity must be finite");
  }
  const auto ray0 = calibration::image_point_to_ray(rectification.camera(0), {row, column});
  const auto ray1_device1 =
      calibration::image_point_to_ray(rectification.camera(1), {row, column + disparity});
  const auto ray1_device0 = transform_ray(rectification.device_1_in_device_0_pose(), ray1_device1);
  return triangulate_rays(Ray3d{ray0.origin, ray0.direction}, ray1_device0, max_ray_gap).point;
}

XyzImages disparity_image_to_xyz(const StereoRectification& rectification,
                                 const core::Image& disparity, double max_ray_gap) {
  validate_disparity_image(rectification, disparity, "disparity_image_to_xyz");
  XyzImages xyz{
      core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32),
      core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32),
      core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32)};
  StereoPointContext point_context(rectification);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t r = 0; r < disparity.height(); ++r) {
    for (std::size_t c = 0; c < disparity.width(); ++c) {
      const float d = image_float(disparity, r, c);
      if (!std::isfinite(d)) {
        set_image_float(xyz.x, r, c, nan);
        set_image_float(xyz.y, r, c, nan);
        set_image_float(xyz.z, r, c, nan);
        continue;
      }
      try {
        const auto point =
            point_context.point(static_cast<double>(r), static_cast<double>(c), d, max_ray_gap);
        set_image_float(xyz.x, r, c, static_cast<float>(point.x));
        set_image_float(xyz.y, r, c, static_cast<float>(point.y));
        set_image_float(xyz.z, r, c, static_cast<float>(point.z));
      } catch (const core::Exception&) {
        set_image_float(xyz.x, r, c, nan);
        set_image_float(xyz.y, r, c, nan);
        set_image_float(xyz.z, r, c, nan);
      }
    }
  }
  return xyz;
}

ObjectModel3DData reconstruct_points_stereo(
    const core::Image& rectified_left, const core::Image& rectified_right,
    const StereoRectification& rectification, std::int64_t min_disparity,
    std::int64_t max_disparity, std::int64_t smoothness, std::int64_t edge_preservation,
    const core::Tuple& gen_param_name, const core::Tuple& gen_param_value, double max_ray_gap) {
  const auto disparity =
      binocular_disparity_ms(rectified_left, rectified_right, min_disparity, max_disparity,
                             smoothness, edge_preservation, gen_param_name, gen_param_value);
  validate_disparity_image(rectification, disparity.disparity, "reconstruct_points_stereo");

  StereoPointContext point_context(rectification);
  ObjectModel3DData model;
  model.width = disparity.disparity.width();
  model.height = disparity.disparity.height();
  const std::size_t count = model.width * model.height;
  model.points.resize(count);
  model.confidence.assign(count, 0.0f);
  model.valid.assign(count, 0U);
  const bool quality = !disparity.quality_channels.empty();
  if (quality) allocate_quality(model);
  const auto* camera0 =
      std::get_if<calibration::AreaScanDivisionParameters>(&rectification.camera(0));
  const auto* camera1 =
      std::get_if<calibration::AreaScanDivisionParameters>(&rectification.camera(1));
  const auto& pose = rectification.device_1_in_device_0_pose();
  const bool parallel_pinhole = camera0 && camera1 && camera0->kappa == 0 && camera1->kappa == 0 &&
                                camera0->focus / camera0->sx == camera1->focus / camera1->sx &&
                                camera0->focus / camera0->sy == camera1->focus / camera1->sy &&
                                camera0->cy == camera1->cy && pose.translation.y == 0 &&
                                pose.translation.z == 0 && pose.rotation.x == 0 &&
                                pose.rotation.y == 0 && pose.rotation.z == 0;

  for (std::size_t r = 0; r < model.height; ++r) {
    for (std::size_t c = 0; c < model.width; ++c) {
      const std::size_t i = r * model.width + c;
      const float d = image_float(disparity.disparity, r, c);
      if (quality)
        model.quality_reason[i] = reinterpret_cast<const std::uint16_t*>(
            static_cast<const std::byte*>(disparity.quality_channels[2].data()) +
            r * disparity.quality_channels[2].stride())[c];
      if (!std::isfinite(d)) continue;
      try {
        model.points[i] =
            point_context.point(static_cast<double>(r), static_cast<double>(c), d, max_ray_gap);
        const float score = image_float(disparity.score, r, c);
        model.confidence[i] = std::isfinite(score) ? 1.0f / (1.0f + std::max(0.0f, score)) : 0.0f;
        model.valid[i] = 1U;
        if (quality) {
          model.observed_valid[i] = 1;
          model.quality_reason[i] = 0;
          model.confidence[i] = image_float(disparity.quality_channels[0], r, c);
          if (parallel_pinhole && disparity.disparity_stddev > 0) {
            const double effective_disparity = d + camera0->cx - camera1->cx;
            const double sigma =
                std::abs(model.points[i].z / effective_disparity) * disparity.disparity_stddev;
            if (std::isfinite(sigma) && std::isfinite(static_cast<float>(sigma)))
              model.depth_uncertainty[i] = static_cast<float>(sigma);
          } else
            model.depth_uncertainty[i] = depth_sigma(
                [&](double v) { return point_context.point(double(r), double(c), v, max_ray_gap); },
                d, disparity.disparity_stddev);
        }
      } catch (const core::Exception&) {
        if (quality) model.quality_reason[i] |= GeometryFailure;
      }
    }
  }
  if (quality) fill_quality_holes(model, disparity.hole_max_width, disparity.hole_max_depth_jump);
  validate_object_model_3d(model);
  return model;
}

}  // namespace mv::reconstruction
