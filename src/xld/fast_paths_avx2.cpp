#include "fast_paths_avx2.hpp"
#include <immintrin.h>
#include <limits>
#include <algorithm>
#include <cmath>
namespace mv::xld {
std::uint32_t gradient_candidates_mask32_avx2(const double* magnitudes,double low) {
  const auto threshold=_mm256_set1_pd(low),zero=_mm256_setzero_pd();
  std::uint32_t result=0;
  for(unsigned first=0;first<32;first+=4) {
    const auto value=_mm256_loadu_pd(magnitudes+first);
    // 保留 !(m<low || m<=0)，包含标量无序比较。
    const auto pass=_mm256_and_pd(_mm256_cmp_pd(value,threshold,_CMP_NLT_UQ),
                                 _mm256_cmp_pd(value,zero,_CMP_NLE_UQ));
    result|=std::uint32_t(_mm256_movemask_pd(pass))<<first;
  }
  return result;
}
std::uint32_t ridge_candidates_mask32_avx2(const double* xx,const double* xy,const double* yy,double low) {
  const auto sign=_mm256_set1_pd(-0.0),two=_mm256_set1_pd(2.),threshold=_mm256_set1_pd(low*.5);
  std::uint32_t result=0;
  for(unsigned first=0;first<32;first+=4) {
    const auto x=_mm256_andnot_pd(sign,_mm256_loadu_pd(xx+first));
    const auto y=_mm256_andnot_pd(sign,_mm256_loadu_pd(yy+first));
    const auto cross=_mm256_andnot_pd(sign,_mm256_loadu_pd(xy+first));
    const auto bound=_mm256_add_pd(_mm256_add_pd(x,y),_mm256_mul_pd(two,cross));
    // 既有保守守卫及原评估序不变。
    result|=std::uint32_t(_mm256_movemask_pd(_mm256_cmp_pd(bound,threshold,_CMP_NLT_UQ)))<<first;
  }
  return result;
}
std::size_t gradient_magnitudes_avx2(std::span<const double> x,std::span<const double> y,std::span<double> out) {
  const auto tiny=_mm256_set1_pd(1e-300);
  const auto infinity=_mm256_set1_pd(std::numeric_limits<double>::infinity());
  const auto zero=_mm256_setzero_pd();
  std::size_t i=0;
  for(;out.size()-i>=4;i+=4) {
    const auto vx=_mm256_loadu_pd(x.data()+i),vy=_mm256_loadu_pd(y.data()+i);
    const auto squared=_mm256_add_pd(_mm256_mul_pd(vx,vx),_mm256_mul_pd(vy,vy));
    const auto safe=_mm256_and_pd(_mm256_cmp_pd(squared,tiny,_CMP_GE_OQ),
                                 _mm256_cmp_pd(squared,infinity,_CMP_LT_OQ));
    const auto both_zero=_mm256_and_pd(_mm256_cmp_pd(vx,zero,_CMP_EQ_OQ),
                                     _mm256_cmp_pd(vy,zero,_CMP_EQ_OQ));
    if(_mm256_movemask_pd(_mm256_or_pd(safe,both_zero))==15)_mm256_storeu_pd(out.data()+i,_mm256_sqrt_pd(squared));
    else {
      // 保持标量平方长度守卫，含零及 subnormal。
      for(std::size_t j=i;j<i+4;++j) {
        const double s=x[j]*x[j]+y[j]*y[j];
        out[j]=s>=1e-300 && s<std::numeric_limits<double>::infinity() ? std::sqrt(s)
             : x[j]==0 && y[j]==0 ? 0 : std::hypot(x[j],y[j]);
      }
    }
  }
  return i;
}
template<std::size_t Taps>
void ridge_horizontal_impl(const double* source,const std::size_t* columns,
    std::size_t width,std::size_t radius,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,double* o0,double* o1,double* o2) {
  std::size_t c=0;
  // 进入连续八像素 tile 前处理左镜像边界。
  for(;c<width && c<radius;++c) {
    double v0=0,v1=0,v2=0;
    for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
      const auto value=source[columns[c*k0.size()+k]];
      v0+=k0[k]*value;v1+=k1[k]*value;v2+=k2[k]*value;
    }
    o0[c]=v0;o1[c]=v1;o2[c]=v2;
  }
  for(;width-c>=8 && width-c-8>=radius;c+=8) {
    auto a0=_mm256_setzero_pd(),a1=a0,b0=a0,b1=a0,d0=a0,d1=a0;
    for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
      const auto p=_mm256_loadu_pd(source+c+k-radius),q=_mm256_loadu_pd(source+c+4+k-radius);
      const auto g0=_mm256_set1_pd(k0[k]),g1=_mm256_set1_pd(k1[k]),g2=_mm256_set1_pd(k2[k]);
      a0=_mm256_add_pd(a0,_mm256_mul_pd(g0,p));a1=_mm256_add_pd(a1,_mm256_mul_pd(g0,q));
      b0=_mm256_add_pd(b0,_mm256_mul_pd(g1,p));b1=_mm256_add_pd(b1,_mm256_mul_pd(g1,q));
      d0=_mm256_add_pd(d0,_mm256_mul_pd(g2,p));d1=_mm256_add_pd(d1,_mm256_mul_pd(g2,q));
    }
    _mm256_storeu_pd(o0+c,a0);_mm256_storeu_pd(o0+c+4,a1);
    _mm256_storeu_pd(o1+c,b0);_mm256_storeu_pd(o1+c+4,b1);
    _mm256_storeu_pd(o2+c,d0);_mm256_storeu_pd(o2+c+4,d1);
  }
  for(;c<width;) {
    if(c>=radius && width-c>=4 && width-c-4>=radius) {
      auto v0=_mm256_setzero_pd(),v1=v0,v2=v0;
      for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
        const auto value=_mm256_loadu_pd(source+c+k-radius);
        v0=_mm256_add_pd(v0,_mm256_mul_pd(_mm256_set1_pd(k0[k]),value));
        v1=_mm256_add_pd(v1,_mm256_mul_pd(_mm256_set1_pd(k1[k]),value));
        v2=_mm256_add_pd(v2,_mm256_mul_pd(_mm256_set1_pd(k2[k]),value));
      }
      _mm256_storeu_pd(o0+c,v0);_mm256_storeu_pd(o1+c,v1);_mm256_storeu_pd(o2+c,v2);c+=4;
    } else {
      double v0=0,v1=0,v2=0;
      for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
        const auto value=source[columns[c*k0.size()+k]];
        v0+=k0[k]*value;v1+=k1[k]*value;v2+=k2[k]*value;
      }
      o0[c]=v0;o1[c]=v1;o2[c]=v2;++c;
    }
  }
}
template<std::size_t Taps,bool Gradients=true>
void ridge_vertical_impl(const double* h0,const double* h1,const double* h2,
    const std::size_t* rows,std::size_t width,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,
    double* ox,double* oy,double* oxx,double* oxy,double* oyy) {
  std::size_t c=0;
  for(;width-c>=8;c+=8) {
    auto gx0=_mm256_setzero_pd(),gx1=gx0,gy0=gx0,gy1=gx0,xx0=gx0,xx1=gx0,
         xy0=gx0,xy1=gx0,yy0=gx0,yy1=gx0;
    for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
      const auto i=rows[k]+c;
      const auto g0=_mm256_set1_pd(k0[k]);
      auto a=_mm256_loadu_pd(h2+i),b=_mm256_loadu_pd(h2+i+4);
      xx0=_mm256_add_pd(xx0,_mm256_mul_pd(g0,a));xx1=_mm256_add_pd(xx1,_mm256_mul_pd(g0,b));
      a=_mm256_loadu_pd(h1+i);b=_mm256_loadu_pd(h1+i+4);
      if constexpr(Gradients){gx0=_mm256_add_pd(gx0,_mm256_mul_pd(g0,a));gx1=_mm256_add_pd(gx1,_mm256_mul_pd(g0,b));}
      const auto g1=_mm256_set1_pd(k1[k]);
      xy0=_mm256_add_pd(xy0,_mm256_mul_pd(g1,a));xy1=_mm256_add_pd(xy1,_mm256_mul_pd(g1,b));
      a=_mm256_loadu_pd(h0+i);b=_mm256_loadu_pd(h0+i+4);
      if constexpr(Gradients){gy0=_mm256_add_pd(gy0,_mm256_mul_pd(g1,a));gy1=_mm256_add_pd(gy1,_mm256_mul_pd(g1,b));}
      const auto g2=_mm256_set1_pd(k2[k]);
      yy0=_mm256_add_pd(yy0,_mm256_mul_pd(g2,a));yy1=_mm256_add_pd(yy1,_mm256_mul_pd(g2,b));
    }
    if constexpr(Gradients){_mm256_storeu_pd(ox+c,gx0);_mm256_storeu_pd(ox+c+4,gx1);}
    if constexpr(Gradients){_mm256_storeu_pd(oy+c,gy0);_mm256_storeu_pd(oy+c+4,gy1);}
    _mm256_storeu_pd(oxx+c,xx0);_mm256_storeu_pd(oxx+c+4,xx1);
    _mm256_storeu_pd(oxy+c,xy0);_mm256_storeu_pd(oxy+c+4,xy1);
    _mm256_storeu_pd(oyy+c,yy0);_mm256_storeu_pd(oyy+c+4,yy1);
  }
  for(;width-c>=4;c+=4) {
    auto gx=_mm256_setzero_pd(),gy=gx,xx=gx,xy=gx,yy=gx;
    for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
      const auto i=rows[k]+c;
      const auto v0=_mm256_loadu_pd(h0+i),v1=_mm256_loadu_pd(h1+i),v2=_mm256_loadu_pd(h2+i);
      const auto g0=_mm256_set1_pd(k0[k]),g1=_mm256_set1_pd(k1[k]),g2=_mm256_set1_pd(k2[k]);
      xx=_mm256_add_pd(xx,_mm256_mul_pd(g0,v2));xy=_mm256_add_pd(xy,_mm256_mul_pd(g1,v1));
      yy=_mm256_add_pd(yy,_mm256_mul_pd(g2,v0));if constexpr(Gradients){gx=_mm256_add_pd(gx,_mm256_mul_pd(g0,v1));}
      if constexpr(Gradients){gy=_mm256_add_pd(gy,_mm256_mul_pd(g1,v0));}
    }
    if constexpr(Gradients){_mm256_storeu_pd(ox+c,gx);_mm256_storeu_pd(oy+c,gy);}_mm256_storeu_pd(oxx+c,xx);
    _mm256_storeu_pd(oxy+c,xy);_mm256_storeu_pd(oyy+c,yy);
  }
  for(;c<width;++c) {
    double gx=0,gy=0,xx=0,xy=0,yy=0;
    for(std::size_t k=0;k<(Taps ? Taps : k0.size());++k) {
      const auto i=rows[k]+c;
      xx+=k0[k]*h2[i];xy+=k1[k]*h1[i];yy+=k2[k]*h0[i];
      if constexpr(Gradients){gx+=k0[k]*h1[i];gy+=k1[k]*h0[i];}
    }
    if constexpr(Gradients){ox[c]=gx;oy[c]=gy;}oxx[c]=xx;oxy[c]=xy;oyy[c]=yy;
  }
}
bool pairable(std::span<const double> k1,std::span<const double> k2) {
  if(k1.size()!=17 || k2.size()!=17)return false;
  for(std::size_t k=0;k<8;++k)if(k1[k]!=-k1[16-k] || k2[k]!=k2[16-k])return false;
  return true;
}
void ridge_horizontal_pairs(const double* source,const std::size_t* columns,
    std::size_t width,std::span<const double> k0,std::span<const double> k1,
    std::span<const double> k2,double* o0,double* o1,double* o2) {
  std::size_t c=0;
  const auto scalar=[&](std::size_t col) {
    double a=0,b=0,d=0;
    for(std::size_t k=0;k<17;++k){const auto value=source[columns[col*17+k]];a+=k0[k]*value;b+=k1[k]*value;d+=k2[k]*value;}
    o0[col]=a;o1[col]=b;o2[col]=d;
  };
  for(;c<width && c<8;++c)scalar(c);
  for(;width-c>=8 && width-c-8>=8;c+=8) {
    auto a=_mm256_setzero_pd(),b=a,d=a,a1=a,b1=a,d1=a;
    for(std::size_t k=0;k<8;++k) {
      const auto left=_mm256_loadu_pd(source+c+k-8),right=_mm256_loadu_pd(source+c+8-k);
      const auto left1=_mm256_loadu_pd(source+c+k-4),right1=_mm256_loadu_pd(source+c+12-k);
      a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[k]),left));
      a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[k]),left1));
      a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[16-k]),right));
      a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[16-k]),right1));
      b=_mm256_add_pd(b,_mm256_mul_pd(_mm256_set1_pd(k1[k]),_mm256_sub_pd(left,right)));
      b1=_mm256_add_pd(b1,_mm256_mul_pd(_mm256_set1_pd(k1[k]),_mm256_sub_pd(left1,right1)));
      d=_mm256_add_pd(d,_mm256_mul_pd(_mm256_set1_pd(k2[k]),_mm256_add_pd(left,right)));
      d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_set1_pd(k2[k]),_mm256_add_pd(left1,right1)));
    }
    const auto center=_mm256_loadu_pd(source+c),center1=_mm256_loadu_pd(source+c+4);
    a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[8]),center));
    a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[8]),center1));
    b=_mm256_add_pd(b,_mm256_mul_pd(_mm256_set1_pd(k1[8]),center));
    b1=_mm256_add_pd(b1,_mm256_mul_pd(_mm256_set1_pd(k1[8]),center1));
    d=_mm256_add_pd(d,_mm256_mul_pd(_mm256_set1_pd(k2[8]),center));
    d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_set1_pd(k2[8]),center1));
    _mm256_storeu_pd(o0+c,a);_mm256_storeu_pd(o1+c,b);_mm256_storeu_pd(o2+c,d);
    _mm256_storeu_pd(o0+c+4,a1);_mm256_storeu_pd(o1+c+4,b1);_mm256_storeu_pd(o2+c+4,d1);
  }
  for(;c<width;++c)scalar(c);
}
void ridge_response_pairs(const double* h0,const double* h1,const double* h2,
    const std::size_t* rows,std::size_t width,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,double* xx,double* xy,double* yy) {
  std::size_t c=0;
  for(;width-c>=8;c+=8) {
    auto a=_mm256_setzero_pd(),b=a,d=a,a1=a,b1=a,d1=a;
    for(std::size_t k=0;k<8;++k) {
      const auto left=rows[k]+c,right=rows[16-k]+c;
      a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[k]),_mm256_loadu_pd(h2+left)));
      a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[k]),_mm256_loadu_pd(h2+left+4)));
      a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[16-k]),_mm256_loadu_pd(h2+right)));
      a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[16-k]),_mm256_loadu_pd(h2+right+4)));
      b=_mm256_add_pd(b,_mm256_mul_pd(_mm256_set1_pd(k1[k]),_mm256_sub_pd(_mm256_loadu_pd(h1+left),_mm256_loadu_pd(h1+right))));
      b1=_mm256_add_pd(b1,_mm256_mul_pd(_mm256_set1_pd(k1[k]),_mm256_sub_pd(_mm256_loadu_pd(h1+left+4),_mm256_loadu_pd(h1+right+4))));
      d=_mm256_add_pd(d,_mm256_mul_pd(_mm256_set1_pd(k2[k]),_mm256_add_pd(_mm256_loadu_pd(h0+left),_mm256_loadu_pd(h0+right))));
      d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_set1_pd(k2[k]),_mm256_add_pd(_mm256_loadu_pd(h0+left+4),_mm256_loadu_pd(h0+right+4))));
    }
    a=_mm256_add_pd(a,_mm256_mul_pd(_mm256_set1_pd(k0[8]),_mm256_loadu_pd(h2+rows[8]+c)));
    a1=_mm256_add_pd(a1,_mm256_mul_pd(_mm256_set1_pd(k0[8]),_mm256_loadu_pd(h2+rows[8]+c+4)));
    b=_mm256_add_pd(b,_mm256_mul_pd(_mm256_set1_pd(k1[8]),_mm256_loadu_pd(h1+rows[8]+c)));
    b1=_mm256_add_pd(b1,_mm256_mul_pd(_mm256_set1_pd(k1[8]),_mm256_loadu_pd(h1+rows[8]+c+4)));
    d=_mm256_add_pd(d,_mm256_mul_pd(_mm256_set1_pd(k2[8]),_mm256_loadu_pd(h0+rows[8]+c)));
    d1=_mm256_add_pd(d1,_mm256_mul_pd(_mm256_set1_pd(k2[8]),_mm256_loadu_pd(h0+rows[8]+c+4)));
    _mm256_storeu_pd(xx+c,a);_mm256_storeu_pd(xy+c,b);_mm256_storeu_pd(yy+c,d);
    _mm256_storeu_pd(xx+c+4,a1);_mm256_storeu_pd(xy+c+4,b1);_mm256_storeu_pd(yy+c+4,d1);
  }
  for(;c<width;++c) {
    double a=0,b=0,d=0;
    for(std::size_t k=0;k<17;++k){a+=k0[k]*h2[rows[k]+c];b+=k1[k]*h1[rows[k]+c];d+=k2[k]*h0[rows[k]+c];}
    xx[c]=a;xy[c]=b;yy[c]=d;
  }
}
void ridge_horizontal_avx2(const double* source,const std::size_t* columns,
    std::size_t width,std::size_t radius,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,double* o0,double* o1,double* o2,bool bounded) {
  if(bounded && k0.size()==17 && pairable(k1,k2)){ridge_horizontal_pairs(source,columns,width,k0,k1,k2,o0,o1,o2);return;}
  if(k0.size()==17)ridge_horizontal_impl<17>(source,columns,width,radius,k0,k1,k2,o0,o1,o2);
  else ridge_horizontal_impl<0>(source,columns,width,radius,k0,k1,k2,o0,o1,o2);
}
void ridge_vertical_avx2(const double* h0,const double* h1,const double* h2,
    const std::size_t* rows,std::size_t width,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,
    double* ox,double* oy,double* oxx,double* oxy,double* oyy) {
  if(k0.size()==17)ridge_vertical_impl<17>(h0,h1,h2,rows,width,k0,k1,k2,ox,oy,oxx,oxy,oyy);
  else ridge_vertical_impl<0>(h0,h1,h2,rows,width,k0,k1,k2,ox,oy,oxx,oxy,oyy);
}
void ridge_response_avx2(const double* h0,const double* h1,const double* h2,
    const std::size_t* rows,std::size_t width,std::span<const double> k0,
    std::span<const double> k1,std::span<const double> k2,double* xx,double* xy,double* yy,bool bounded) {
  if(bounded && k0.size()==17 && pairable(k1,k2)){ridge_response_pairs(h0,h1,h2,rows,width,k0,k1,k2,xx,xy,yy);return;}
  if(k0.size()==17)ridge_vertical_impl<17,false>(h0,h1,h2,rows,width,k0,k1,k2,nullptr,nullptr,xx,xy,yy);
  else ridge_vertical_impl<0,false>(h0,h1,h2,rows,width,k0,k1,k2,nullptr,nullptr,xx,xy,yy);
}
void ridge_gradient_tile_avx2(const double* h0,const double* h1,const std::size_t* rows,
    std::size_t column,std::span<const double> k0,std::span<const double> k1,double* ox,double* oy) {
  auto gx=_mm256_setzero_pd(),gy=gx;
  for(std::size_t k=0;k<k0.size();++k) {
    const auto i=rows[k]+column;
    gx=_mm256_add_pd(gx,_mm256_mul_pd(_mm256_set1_pd(k0[k]),_mm256_loadu_pd(h1+i)));
    gy=_mm256_add_pd(gy,_mm256_mul_pd(_mm256_set1_pd(k1[k]),_mm256_loadu_pd(h0+i)));
  }
  _mm256_storeu_pd(ox,gx);_mm256_storeu_pd(oy,gy);
}
bool contour_cross_sum_avx2(std::span<const core::Point2d> points,core::Point2d origin,
                           double& sum,double& magnitude) {
  const auto zero=_mm256_set_pd(origin.col,origin.row,origin.col,origin.row);
  const auto sign=_mm256_set1_pd(-0.0),bound=_mm256_set1_pd(0x1p30);
  auto accumulated=_mm256_setzero_pd(),absolute=accumulated;
  const auto valid=[&](auto v){return _mm256_cmp_pd(
      _mm256_andnot_pd(sign,v),bound,_CMP_LE_OQ);};
  for(std::size_t i=0;i+4<points.size();i+=4) {
    const auto a0=_mm256_loadu_pd(&points[i].row),b0=_mm256_loadu_pd(&points[i+1].row);
    const auto a1=_mm256_loadu_pd(&points[i+2].row),b1=_mm256_loadu_pd(&points[i+3].row);
    if(_mm256_movemask_pd(_mm256_and_pd(valid(a0),_mm256_and_pd(valid(a1),valid(b1))))!=15)return false;
    const auto ar0=_mm256_sub_pd(a0,zero),br0=_mm256_sub_pd(b0,zero);
    const auto ar1=_mm256_sub_pd(a1,zero),br1=_mm256_sub_pd(b1,zero);
    const auto p0=_mm256_mul_pd(_mm256_permute_pd(ar0,5),br0);
    const auto p1=_mm256_mul_pd(_mm256_permute_pd(ar1,5),br1);
    const auto terms=_mm256_hsub_pd(p0,p1);
    accumulated=_mm256_add_pd(accumulated,terms);
    absolute=_mm256_add_pd(absolute,_mm256_andnot_pd(sign,terms));
  }
  double values[4],magnitudes[4];
  _mm256_storeu_pd(values,accumulated);_mm256_storeu_pd(magnitudes,absolute);
  sum=(values[0]+values[1])+(values[2]+values[3]);
  magnitude=(magnitudes[0]+magnitudes[1])+(magnitudes[2]+magnitudes[3]);
  return true;
}
bool contour_cross_terms_avx2(std::span<const core::Point2d> points,core::Point2d origin,
                             std::span<double> terms,double* row_terms,double* col_terms) {
  const auto zero=_mm256_set_pd(origin.col,origin.row,origin.col,origin.row);
  const auto sign=_mm256_set1_pd(-0.0),bound=_mm256_set1_pd(0x1p30);
  const auto valid=[&](auto v){return _mm256_cmp_pd(
      _mm256_andnot_pd(sign,v),bound,_CMP_LE_OQ);};
  for(std::size_t i=0;i<terms.size();i+=4) {
    const auto a0=_mm256_loadu_pd(&points[i].row),b0=_mm256_loadu_pd(&points[i+1].row);
    const auto a1=_mm256_loadu_pd(&points[i+2].row),b1=_mm256_loadu_pd(&points[i+3].row);
    // a0/a1/b1 覆盖全部五顶点，b0 只重复顶点。
    if(_mm256_movemask_pd(_mm256_and_pd(valid(a0),_mm256_and_pd(valid(a1),valid(b1))))!=15)return false;
    const auto ar0=_mm256_sub_pd(a0,zero),br0=_mm256_sub_pd(b0,zero);
    const auto ar1=_mm256_sub_pd(a1,zero),br1=_mm256_sub_pd(b1,zero);
    const auto p0=_mm256_mul_pd(_mm256_permute_pd(ar0,5),br0);
    const auto p1=_mm256_mul_pd(_mm256_permute_pd(ar1,5),br1);
    // hsub lane 序为 0/2/1/3，补偿前恢复栅格序。
    const auto w=_mm256_permute4x64_pd(_mm256_hsub_pd(p0,p1),0xd8);
    _mm256_storeu_pd(terms.data()+i,w);
    if(row_terms) {
      const auto sums0=_mm256_add_pd(ar0,br0),sums1=_mm256_add_pd(ar1,br1);
      const auto rows=_mm256_permute4x64_pd(_mm256_unpacklo_pd(sums0,sums1),0xd8);
      const auto cols=_mm256_permute4x64_pd(_mm256_unpackhi_pd(sums0,sums1),0xd8);
      _mm256_storeu_pd(row_terms+i,_mm256_mul_pd(rows,w));
      _mm256_storeu_pd(col_terms+i,_mm256_mul_pd(cols,w));
    }
  }
  return true;
}
}
