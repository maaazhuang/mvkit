#include <geometry/analytic2d_avx2.hpp>
#include <immintrin.h>
#include <cmath>
#include <cfenv>
#include <limits>

namespace mv::geometry {
std::size_t affine_points_real_avx2(std::span<const double> x,std::span<const double> y,
    const double* matrix,std::span<double> output_x,std::span<double> output_y) {
  if(std::fegetround()!=FE_TONEAREST)return 0;
  const auto sign=_mm256_set1_pd(-0.),infinity=_mm256_set1_pd(std::numeric_limits<double>::infinity());
  const auto finite=[&](__m256d value) {
    return _mm256_cmp_pd(_mm256_andnot_pd(sign,value),infinity,_CMP_LT_OQ);
  };
  const auto a=_mm256_set1_pd(matrix[0]),b=_mm256_set1_pd(matrix[1]),c=_mm256_set1_pd(matrix[2]);
  const auto d=_mm256_set1_pd(matrix[3]),e=_mm256_set1_pd(matrix[4]),f=_mm256_set1_pd(matrix[5]);
  std::size_t i=0;
  for(;i+4<=output_x.size();i+=4) {
    const auto px=x.size()==1?_mm256_set1_pd(x.front()):_mm256_loadu_pd(x.data()+i);
    const auto py=y.size()==1?_mm256_set1_pd(y.front()):_mm256_loadu_pd(y.data()+i);
    const auto qx=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(a,px),_mm256_mul_pd(b,py)),c);
    const auto qy=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(d,px),_mm256_mul_pd(e,py)),f);
    if(_mm256_movemask_pd(_mm256_and_pd(_mm256_and_pd(finite(px),finite(py)),
        _mm256_and_pd(finite(qx),finite(qy))))!=15)return i;
    _mm256_storeu_pd(output_x.data()+i,qx);_mm256_storeu_pd(output_y.data()+i,qy);
  }
  return i;
}

std::size_t distance_pp_real_avx2(std::span<const double> row1,std::span<const double> column1,
    std::span<const double> row2,std::span<const double> column2,std::span<double> output) {
  if(std::fegetround()!=FE_TONEAREST)return 0;
  const auto sign=_mm256_set1_pd(-0.0),infinity=_mm256_set1_pd(std::numeric_limits<double>::infinity());
  const auto lower=_mm256_set1_pd(1e-150),upper=_mm256_set1_pd(1e150),zero=_mm256_setzero_pd();
  const auto absolute=[&](__m256d value){return _mm256_andnot_pd(sign,value);};
  const auto load=[](std::span<const double> values,std::size_t index) {
    return values.size()==1?_mm256_set1_pd(values.front()):_mm256_loadu_pd(values.data()+index);
  };
  std::size_t i=0;
  for(;i+4<=output.size();i+=4) {
    const auto r1=load(row1,i),c1=load(column1,i),r2=load(row2,i),c2=load(column2,i);
    auto finite=_mm256_and_pd(_mm256_cmp_pd(absolute(r1),infinity,_CMP_LT_OQ),_mm256_cmp_pd(absolute(c1),infinity,_CMP_LT_OQ));
    finite=_mm256_and_pd(finite,_mm256_and_pd(_mm256_cmp_pd(absolute(r2),infinity,_CMP_LT_OQ),_mm256_cmp_pd(absolute(c2),infinity,_CMP_LT_OQ)));
    // 发现非法输入时从本块交还原逐点检查，保留点序和参数错误优先级。
    if(_mm256_movemask_pd(finite)!=15)return i;
    const auto dr=_mm256_sub_pd(r2,r1),dc=_mm256_sub_pd(c2,c1),ar=absolute(dr),ac=absolute(dc);
    const auto safe=[&](__m256d magnitude) {
      return _mm256_and_pd(_mm256_cmp_pd(magnitude,upper,_CMP_LE_OQ),
          _mm256_or_pd(_mm256_cmp_pd(magnitude,lower,_CMP_GE_OQ),_mm256_cmp_pd(magnitude,zero,_CMP_EQ_OQ)));
    };
    if(_mm256_movemask_pd(_mm256_and_pd(safe(ar),safe(ac)))==15) {
      // 常规范围保持 Double；禁用 FMA，开方路径误差由数值回归限制在少数 ULP。
      _mm256_storeu_pd(output.data()+i,_mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(dr,dr),_mm256_mul_pd(dc,dc))));
    } else {
      alignas(32) double rows[4],columns[4];
      _mm256_store_pd(rows,dr);_mm256_store_pd(columns,dc);
      for(int lane=0;lane<4;++lane)output[i+lane]=std::hypot(rows[lane],columns[lane]);
    }
  }
  return i;
}
}
