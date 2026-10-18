#pragma once

#include <array>

// 标定与重建使用的私有几何值类型；不属于安装 SDK。
namespace mv {

struct Point2d {
  double row{0.0};
  double col{0.0};
};

struct Point3d {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaterniond {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose3d {
  Point3d translation{};
  Quaterniond rotation{};
};

struct HomMat3d {
  std::array<double, 16> values{};
};

}  // namespace mv
