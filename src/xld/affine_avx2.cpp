#include "fast_paths_avx2.hpp"
#include <immintrin.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <iterator>
namespace mv::xld {
// 仅将已初始化前缀物化为 Point2d；标量 scratch 无默认成员初始化，不先清零整个 tile。
struct PointBlockIterator {
  using iterator_category=std::random_access_iterator_tag;
  using value_type=core::Point2d;
  using difference_type=std::ptrdiff_t;
  using pointer=void;
  using reference=value_type;
  const double* p;
  value_type operator*() const{return {p[0],p[1]};}
  value_type operator[](difference_type n) const{return *(*this+n);}
  PointBlockIterator& operator++(){p+=2;return *this;}
  PointBlockIterator operator++(int){auto old=*this;++*this;return old;}
  PointBlockIterator& operator--(){p-=2;return *this;}
  PointBlockIterator operator--(int){auto old=*this;--*this;return old;}
  PointBlockIterator& operator+=(difference_type n){p+=n*2;return *this;}
  PointBlockIterator& operator-=(difference_type n){p-=n*2;return *this;}
  friend PointBlockIterator operator+(PointBlockIterator a,difference_type n){return a+=n;}
  friend PointBlockIterator operator+(difference_type n,PointBlockIterator a){return a+=n;}
  friend PointBlockIterator operator-(PointBlockIterator a,difference_type n){return a-=n;}
  friend difference_type operator-(PointBlockIterator a,PointBlockIterator b){return (a.p-b.p)/2;}
  friend bool operator==(PointBlockIterator a,PointBlockIterator b){return a.p==b.p;}
  friend bool operator<(PointBlockIterator a,PointBlockIterator b){return a.p<b.p;}
  friend bool operator>(PointBlockIterator a,PointBlockIterator b){return b<a;}
  friend bool operator<=(PointBlockIterator a,PointBlockIterator b){return !(b<a);}
  friend bool operator>=(PointBlockIterator a,PointBlockIterator b){return !(a<b);}
};
template<std::size_t Capacity,bool Translation,bool SafeTranslation=false,class Output>
std::size_t affine_points_impl(std::span<const core::Point2d> input,const double* m,
                              Output& output) {
  const auto x=_mm256_set_pd(m[0],m[3],m[0],m[3]),y=_mm256_set_pd(m[1],m[4],m[1],m[4]);
  const auto translation=_mm256_set_pd(m[2],m[5],m[2],m[5]);
  const auto sign=_mm256_set1_pd(-0.0),infinity=_mm256_set1_pd(std::numeric_limits<double>::infinity());
  const auto finite=[&](auto p){return _mm256_cmp_pd(_mm256_andnot_pd(sign,p),infinity,_CMP_LT_OQ);};
  std::size_t i=0;
  alignas(32) double block[Capacity*2];
  const auto append=[&](std::size_t count) {
    if constexpr(std::is_same_v<Output,std::span<core::Point2d>>)
      std::copy_n(PointBlockIterator{block},count,output.data()+i);
    else output.insert(output.end(),PointBlockIterator{block},PointBlockIterator{block+count*2});
  };
  while(input.size()-i>=4) {
    const auto count=std::min<std::size_t>(Capacity,(input.size()-i)/4*4);
    std::size_t offset=0;
    if constexpr(SafeTranslation)for(;offset+8<=count;offset+=8) {
      const auto p0=_mm256_loadu_pd(&input[i+offset].row);
      const auto p1=_mm256_loadu_pd(&input[i+offset+2].row);
      const auto p2=_mm256_loadu_pd(&input[i+offset+4].row);
      const auto p3=_mm256_loadu_pd(&input[i+offset+6].row);
      const auto valid=_mm256_and_pd(_mm256_and_pd(finite(p0),finite(p1)),
                                    _mm256_and_pd(finite(p2),finite(p3)));
      if(_mm256_movemask_pd(valid)!=15) {
        append(offset);return i+offset;
      }
      _mm256_store_pd(block+(offset)*2,_mm256_add_pd(p0,translation));
      _mm256_store_pd(block+(offset+2)*2,_mm256_add_pd(p1,translation));
      _mm256_store_pd(block+(offset+4)*2,_mm256_add_pd(p2,translation));
      _mm256_store_pd(block+(offset+6)*2,_mm256_add_pd(p3,translation));
    }
    for(;offset<count;offset+=4) {
    const auto p0=_mm256_loadu_pd(&input[i+offset].row),p1=_mm256_loadu_pd(&input[i+offset+2].row);
    if(_mm256_movemask_pd(_mm256_and_pd(finite(p0),finite(p1)))!=15) {
      append(offset);return i+offset;
    }
    const auto apply=[&](auto p) {
      if constexpr(Translation)return _mm256_add_pd(p,translation);
      else
      return _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(_mm256_permute_pd(p,15),x),
          _mm256_mul_pd(_mm256_permute_pd(p,0),y)),translation);
    };
    const auto q0=apply(p0),q1=apply(p1);
    // 最近舍入下，偏移小于最高 binade 间距的一半时有限输入不可能溢出；其他情况保留原检查。
    if constexpr(!SafeTranslation) if(_mm256_movemask_pd(_mm256_and_pd(finite(q0),finite(q1)))!=15) {
      append(offset);return i+offset;
    } // 调用方按原错误序重放失败块。
    _mm256_store_pd(block+(offset)*2,q0);_mm256_store_pd(block+(offset+2)*2,q1);
    }
    append(count);
    i+=count;
  }
  return i;
}
template<bool Translation,bool SafeTranslation>
std::size_t affine_owned_impl(std::span<const core::Point2d> input,const double* m,
                             std::span<core::Point2d> output) {
  static_assert(sizeof(core::Point2d)==2*sizeof(double) && std::is_standard_layout_v<core::Point2d>);
  static_assert(offsetof(core::Point2d,col)==sizeof(double));
  const auto x=_mm256_set_pd(m[0],m[3],m[0],m[3]),y=_mm256_set_pd(m[1],m[4],m[1],m[4]);
  const auto translation=_mm256_set_pd(m[2],m[5],m[2],m[5]);
  const auto sign=_mm256_set1_pd(-0.0),infinity=_mm256_set1_pd(std::numeric_limits<double>::infinity());
  const auto finite=[&](auto p){return _mm256_cmp_pd(_mm256_andnot_pd(sign,p),infinity,_CMP_LT_OQ);};
  std::size_t i=0;
  if constexpr(SafeTranslation)for(;input.size()-i>=8;i+=8) {
    const auto p0=_mm256_loadu_pd(&input[i].row),p1=_mm256_loadu_pd(&input[i+2].row);
    const auto p2=_mm256_loadu_pd(&input[i+4].row),p3=_mm256_loadu_pd(&input[i+6].row);
    const auto valid=_mm256_and_pd(_mm256_and_pd(finite(p0),finite(p1)),
                                  _mm256_and_pd(finite(p2),finite(p3)));
    if(_mm256_movemask_pd(valid)!=15)break;
    _mm256_storeu_pd(&output[i].row,_mm256_add_pd(p0,translation));
    _mm256_storeu_pd(&output[i+2].row,_mm256_add_pd(p1,translation));
    _mm256_storeu_pd(&output[i+4].row,_mm256_add_pd(p2,translation));
    _mm256_storeu_pd(&output[i+6].row,_mm256_add_pd(p3,translation));
  }
  for(;input.size()-i>=4;i+=4) {
    const auto p0=_mm256_loadu_pd(&input[i].row),p1=_mm256_loadu_pd(&input[i+2].row);
    if(_mm256_movemask_pd(_mm256_and_pd(finite(p0),finite(p1)))!=15)break;
    const auto apply=[&](auto p) {
      if constexpr(Translation)return _mm256_add_pd(p,translation);
      else return _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(_mm256_permute_pd(p,15),x),
                    _mm256_mul_pd(_mm256_permute_pd(p,0),y)),translation);
    };
    const auto q0=apply(p0),q1=apply(p1);
    if constexpr(!SafeTranslation)
      if(_mm256_movemask_pd(_mm256_and_pd(finite(q0),finite(q1)))!=15)break;
    // 局部载荷具有隐式点生命周期及精确容量，每坐标写一次，无 tile 或 insert/copy。
    _mm256_storeu_pd(&output[i].row,q0);
    _mm256_storeu_pd(&output[i+2].row,q1);
  }
  return i;
}
std::size_t affine_owned_points_avx2(std::span<const core::Point2d> input,const double* m,
                                   std::span<core::Point2d> output,int mode) {
  const bool safe=mode==2 && (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_NEAREST;
  if(safe)return affine_owned_impl<true,true>(input,m,output);
  return mode>0 ? affine_owned_impl<true,false>(input,m,output)
                : affine_owned_impl<false,false>(input,m,output);
}
std::size_t affine_points_avx2(std::span<const core::Point2d> input,const double* m,
                              std::pmr::vector<core::Point2d>& output,int mode) {
  if(mode<0) {
    const bool translate=m[0]==1 && m[4]==1 && m[1]==0 && m[3]==0 && m[2]!=0 && m[5]!=0;
    mode=!translate ? 0 : std::abs(m[2])<=0x1p969 && std::abs(m[5])<=0x1p969 ? 2 : 1;
  }
  const bool translate=mode>0;
  // MXCSR 为线程局部，此处仍须检查 worker 舍入模式。
  const bool safe=mode==2 &&
                  (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_NEAREST;
  if(input.size()<32) {
    if(safe)return affine_points_impl<16,true,true>(input,m,output);
    return translate ? affine_points_impl<16,true>(input,m,output) : affine_points_impl<16,false>(input,m,output);
  }
  if(input.size()<128) {
    if(safe)return affine_points_impl<64,true,true>(input,m,output);
    return translate ? affine_points_impl<64,true>(input,m,output) : affine_points_impl<64,false>(input,m,output);
  }
  if(input.size()<2048) {
    if(safe)return affine_points_impl<256,true,true>(input,m,output);
    return translate ? affine_points_impl<256,true>(input,m,output) : affine_points_impl<256,false>(input,m,output);
  }
  if(safe)return affine_points_impl<512,true,true>(input,m,output);
  return translate ? affine_points_impl<512,true>(input,m,output) : affine_points_impl<512,false>(input,m,output);
}
// 小型内联存储已构造，向量 tile 直接写入。
std::size_t affine_inline_points_avx2(std::span<const core::Point2d> input,const double* m,
                                     std::span<core::Point2d> output,int mode) {
  const bool safe=mode==2 && (_mm_getcsr()&_MM_ROUND_MASK)==_MM_ROUND_NEAREST;
  if(safe)return affine_points_impl<16,true,true>(input,m,output);
  return mode>0 ? affine_points_impl<16,true>(input,m,output)
                : affine_points_impl<16,false>(input,m,output);
}

}
