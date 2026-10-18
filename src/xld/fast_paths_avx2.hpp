#pragma once
#include <core/xld.hpp>
#include <span>
#include <cstdint>
namespace mv::xld {
std::size_t gradient_magnitudes_avx2(std::span<const double>,std::span<const double>,std::span<double>);
std::uint32_t gradient_candidates_mask32_avx2(const double*,double);
std::uint32_t ridge_candidates_mask32_avx2(const double*,const double*,const double*,double);
bool contour_cross_terms_avx2(std::span<const core::Point2d> points,core::Point2d origin,std::span<double> terms,double* row_terms=nullptr,double* col_terms=nullptr);
// 连续边要求 points.size()==4*k+1；仅用于已认证分类区间，公开面积/中心仍补偿求和。
bool contour_cross_sum_avx2(std::span<const core::Point2d>,core::Point2d,double&,double&);

void ridge_horizontal_avx2(const double*,const std::size_t*,std::size_t,std::size_t,
                          std::span<const double>,std::span<const double>,std::span<const double>,
                          double*,double*,double*,bool=false);
void ridge_vertical_avx2(const double*,const double*,const double*,const std::size_t*,std::size_t,
                        std::span<const double>,std::span<const double>,std::span<const double>,
                        double*,double*,double*,double*,double*);
void ridge_response_avx2(const double*,const double*,const double*,const std::size_t*,std::size_t,
                         std::span<const double>,std::span<const double>,std::span<const double>,double*,double*,double*,bool=false);
void ridge_gradient_tile_avx2(const double*,const double*,const std::size_t*,std::size_t,
                              std::span<const double>,std::span<const double>,double*,double*);
std::size_t affine_inline_points_avx2(std::span<const core::Point2d>,const double*,
                                      std::span<core::Point2d>,int);
std::size_t affine_points_avx2(std::span<const core::Point2d>,const double*,
                               std::pmr::vector<core::Point2d>&,int=-1);
std::size_t affine_owned_points_avx2(std::span<const core::Point2d>,const double*,
                                   std::span<core::Point2d>,int);
}
