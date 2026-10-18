#pragma once
#include <span>
#include <cstddef>
namespace mv::geometry {
std::size_t distance_pp_real_avx2(std::span<const double>,std::span<const double>,
    std::span<const double>,std::span<const double>,std::span<double>);
// 返回已完成数量；异常块交还逐点检查，保证原参数诊断次序。
std::size_t affine_points_real_avx2(std::span<const double> x,std::span<const double> y,
    const double* matrix,std::span<double> output_x,std::span<double> output_y);
}
