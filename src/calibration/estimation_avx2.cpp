#include <calibration/estimation_avx2.hpp>
#include <core/numeric_predicates.hpp>
#include <immintrin.h>
#include <limits>
namespace mv::calibration {
std::size_t finite_pairs_avx2(const double* x,const double* y,const double* u,const double* v,std::size_t n) {
  const auto sign=_mm256_set1_pd(-0.),limit=_mm256_set1_pd(std::numeric_limits<double>::max());
  std::size_t i=0;
  for(;n-i>=4;i+=4) {
    const auto a=_mm256_cmp_pd(_mm256_andnot_pd(sign,_mm256_loadu_pd(x+i)),limit,_CMP_LE_OQ);
    const auto b=_mm256_cmp_pd(_mm256_andnot_pd(sign,_mm256_loadu_pd(y+i)),limit,_CMP_LE_OQ);
    const auto c=_mm256_cmp_pd(_mm256_andnot_pd(sign,_mm256_loadu_pd(u+i)),limit,_CMP_LE_OQ);
    const auto d=_mm256_cmp_pd(_mm256_andnot_pd(sign,_mm256_loadu_pd(v+i)),limit,_CMP_LE_OQ);
    if(_mm256_movemask_pd(_mm256_and_pd(_mm256_and_pd(a,b),_mm256_and_pd(c,d)))!=15)break;
  }
  return i;
}
template<bool Check>
std::size_t affine_moments(const double* x,const double* y,const double* u,const double* v,
                         std::size_t n,double* a,double* b) {
  __m256d sums[11];for(auto& sum:sums)sum=_mm256_setzero_pd();
  std::size_t i=0;
  for(;n-i>=4;i+=4) {
    const auto xx=_mm256_loadu_pd(x+i),yy=_mm256_loadu_pd(y+i);
    const auto uu=_mm256_loadu_pd(u+i),vv=_mm256_loadu_pd(v+i);
    if constexpr(Check) {
      const auto mask=_mm256_castsi256_pd(_mm256_set1_epi64x(INT64_MAX));
      const auto limit=_mm256_set1_pd(std::numeric_limits<double>::max());
      if(_mm256_movemask_pd(_mm256_cmp_pd(_mm256_and_pd(xx,mask),limit,_CMP_LE_OQ))!=15 ||
         _mm256_movemask_pd(_mm256_cmp_pd(_mm256_and_pd(yy,mask),limit,_CMP_LE_OQ))!=15 ||
         _mm256_movemask_pd(_mm256_cmp_pd(_mm256_and_pd(uu,mask),limit,_CMP_LE_OQ))!=15 ||
         _mm256_movemask_pd(_mm256_cmp_pd(_mm256_and_pd(vv,mask),limit,_CMP_LE_OQ))!=15)return i;
    }
    sums[0]=_mm256_add_pd(sums[0],_mm256_mul_pd(xx,xx));
    sums[1]=_mm256_add_pd(sums[1],_mm256_mul_pd(xx,yy));
    sums[2]=_mm256_add_pd(sums[2],_mm256_mul_pd(yy,yy));
    sums[3]=_mm256_add_pd(sums[3],xx);sums[4]=_mm256_add_pd(sums[4],yy);
    sums[5]=_mm256_add_pd(sums[5],_mm256_mul_pd(xx,uu));
    sums[6]=_mm256_add_pd(sums[6],_mm256_mul_pd(yy,uu));sums[7]=_mm256_add_pd(sums[7],uu);
    sums[8]=_mm256_add_pd(sums[8],_mm256_mul_pd(xx,vv));
    sums[9]=_mm256_add_pd(sums[9],_mm256_mul_pd(yy,vv));sums[10]=_mm256_add_pd(sums[10],vv);
  }
  double values[11];
  for(int k=0;k<11;++k){double lanes[4];_mm256_storeu_pd(lanes,sums[k]);values[k]=(lanes[0]+lanes[1])+(lanes[2]+lanes[3]);}
  for(;i<n;++i) {
    if constexpr(Check) if(!core::is_finite(x[i]) || !core::is_finite(y[i]) || !core::is_finite(u[i]) || !core::is_finite(v[i]))return i;
    values[0]+=x[i]*x[i];values[1]+=x[i]*y[i];values[2]+=y[i]*y[i];values[3]+=x[i];values[4]+=y[i];
    values[5]+=x[i]*u[i];values[6]+=y[i]*u[i];values[7]+=u[i];values[8]+=x[i]*v[i];values[9]+=y[i]*v[i];values[10]+=v[i];
  }
  a[0]=values[0];a[1]=values[1];a[7]=values[2];a[2]=values[3];a[8]=values[4];
  for(int k=0;k<6;++k)b[k]=values[k+5];
  return n;
}
void affine_moments_avx2(const double* x,const double* y,const double* u,const double* v,std::size_t n,double* a,double* b) {
  (void)affine_moments<false>(x,y,u,v,n,a,b);
}
std::size_t checked_affine_moments_avx2(const double* x,const double* y,const double* u,const double* v,std::size_t n,double* a,double* b) {
  return affine_moments<true>(x,y,u,v,n,a,b);
}
}
