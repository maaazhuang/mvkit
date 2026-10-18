#include <calibration/geometry3d.hpp>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <reconstruction/stereo/rectified_xyz.hpp>
namespace mv::reconstruction {
void validate_rectified_xyz_geometry(const RectifiedXyzGeometry& g) {
  constexpr auto op = "rectified_disparity_xyz";
  if (g.first.perspective != g.second.perspective)
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "CamParamRect2",
                          -1, "both cameras must use the same projection family");
  if (!g.first.perspective) {
    const auto v = calibration::rotate_point(g.pose.rotation, {0, 0, 1});
    if (v.x == 0 && v.y == 0)
      throw core::Exception(core::Status::InvalidState, core::ErrorCode::InvalidObjectState, op,
                            "RelPoseRect", -1, "parallel telecentric viewing directions");
  }
}
XyzImages rectified_disparity_xyz(const core::Image& disparity, const RectifiedXyzGeometry& g) {
  if (disparity.channel_count() != 1 || disparity.pixel_type() != core::PixelType::Float32)
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue,
                          "rectified_disparity_xyz", "Disparity", -1,
                          "one-channel real disparity image required");
  XyzImages result{core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32),
                   core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32),
                   core::Image::create(disparity.width(), disparity.height(), core::PixelType::Float32)};
  const auto domain = disparity.domain();
  const float baseline = static_cast<float>(g.baseline);
  const float cx_difference = static_cast<float>(g.first.cx - g.second.cx);
  const float depth_scale = static_cast<float>(g.baseline * g.first.scale / g.first.sx);
  const auto second_direction = calibration::rotate_point(g.pose.rotation, {0, 0, 1});
  for (const auto& run : domain.runs()) {
    const auto* input = reinterpret_cast<const float*>(static_cast<const std::byte*>(disparity.data()) +
                                                       run.row * disparity.stride());
    auto* x = reinterpret_cast<float*>(static_cast<std::byte*>(result.x.mutable_data()) +
                                       run.row * result.x.stride());
    auto* y = reinterpret_cast<float*>(static_cast<std::byte*>(result.y.mutable_data()) +
                                       run.row * result.y.stride());
    auto* z = reinterpret_cast<float*>(static_cast<std::byte*>(result.z.mutable_data()) +
                                       run.row * result.z.stride());
    for (auto col = std::int64_t(run.col_begin); col <= run.col_end; ++col) {
      const float delta = input[col] + cx_difference;
      if (g.first.perspective) {
        if (!(delta < 0)) {
          x[col] = y[col] = z[col] = std::numeric_limits<float>::max();
        } else {
          // 校正图像转换使用基线长度及首相机内参，不同于通用双射线三角化。
          const float inverse = -1.f / delta;
          // 校正图像协议假定方形像素，保留 Float32 中间乘积并避免 cx1-cx2 相消。
          x[col] = static_cast<float>(col - g.first.cx) * baseline * inverse;
          y[col] = static_cast<float>(run.row - g.first.cy) * baseline * inverse;
          z[col] = depth_scale * inverse;
        }
      } else {
        const Point3d a{(col - g.first.cx) * g.first.sx / g.first.scale,
                        (run.row - g.first.cy) * g.first.sy / g.first.scale, 0};
        const auto b = calibration::transform_point(
            g.pose, {(col + static_cast<double>(input[col]) - g.second.cx) * g.second.sx / g.second.scale,
                     (run.row - g.second.cy) * g.second.sy / g.second.scale, 0});
        x[col] = static_cast<float>(a.x);
        y[col] = static_cast<float>(a.y);
        // 正交图像位置在视差非有限时仍有限，求解校正线的水平分量。
        const bool use_x = std::abs(second_direction.x) >= std::abs(second_direction.y);
        const auto separation = use_x ? a.x - b.x : a.y - b.y;
        const auto direction = use_x ? second_direction.x : second_direction.y;
        z[col] = static_cast<float>(b.z + separation * second_direction.z / direction);
      }
    }
  }
  if (disparity.has_reduced_domain()) {
    result.x = result.x.reduce_domain(domain);
    result.y = result.y.reduce_domain(domain);
    result.z = result.z.reduce_domain(domain);
  }
  return result;
}
}  // namespace mv::reconstruction
