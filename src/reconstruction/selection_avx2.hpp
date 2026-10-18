#pragma once
#include <reconstruction/object_model3d.hpp>
#include <runtime/cpu/cpu_features.hpp>
namespace mv::reconstruction {
// 内核使用 AVX2 整数及原序 Double 算术，变换和 XYZ 导出也不要求 FMA。
inline constexpr mv::runtime::KernelRequirement kSelectionKernelRequirement{false,true,false,false};
void copy_point_runs_avx2(std::span<const Point3d>,std::span<Point3d>,
                         std::span<const std::pair<std::size_t,std::size_t>>);
std::size_t transform_points_avx2(std::span<const Point3d>,std::span<Point3d>,const std::uint8_t*,const double*);
std::size_t export_xyz_avx2(std::span<const Point3d>,const std::uint8_t*,float*,float*,float*);
struct SelectionRuns { std::size_t count=0;bool indexed=true;std::vector<std::pair<std::size_t,std::size_t>> runs; };
SelectionRuns coordinate_runs_avx2(std::span<const Point3d> points,const std::uint8_t* valid,
                                 std::size_t coordinate,double lower,double upper,std::size_t max_runs,
                                 Point3d* destination=nullptr,const double* column=nullptr,
                                 const double* ranges=nullptr);
}
