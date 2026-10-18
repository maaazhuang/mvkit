#pragma once
#include <cstddef>
namespace mv::calibration {
std::size_t finite_pairs_avx2(const double*,const double*,const double*,const double*,std::size_t);
void affine_moments_avx2(const double*,const double*,const double*,const double*,std::size_t,double*,double*);
std::size_t checked_affine_moments_avx2(const double*,const double*,const double*,const double*,std::size_t,double*,double*);
}
