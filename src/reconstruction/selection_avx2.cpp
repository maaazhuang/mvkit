#include "selection_avx2.hpp"
#include <immintrin.h>
#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <bit>
#include <cstring>
namespace mv::reconstruction {
std::size_t export_xyz_avx2(std::span<const Point3d> points,const std::uint8_t* valid,float* x,float* y,float* z) {
  std::size_t i=0;
  for(;points.size()-i>=4;i+=4) {
    if(valid && (!valid[i] || !valid[i+1] || !valid[i+2] || !valid[i+3]))break;
    _mm_storeu_ps(x+i,_mm256_cvtpd_ps(_mm256_setr_pd(points[i].x,points[i+1].x,points[i+2].x,points[i+3].x)));
    _mm_storeu_ps(y+i,_mm256_cvtpd_ps(_mm256_setr_pd(points[i].y,points[i+1].y,points[i+2].y,points[i+3].y)));
    _mm_storeu_ps(z+i,_mm256_cvtpd_ps(_mm256_setr_pd(points[i].z,points[i+1].z,points[i+2].z,points[i+3].z)));
  }
  return i;
}
// 大输出游程完整覆盖，不为写入而加载目标缓存行；小游程保留普通 memcpy。
static void copy_run_streaming(Point3d* destination,const Point3d* source,std::size_t count) {
  auto* dst=reinterpret_cast<std::byte*>(destination);
  auto* src=reinterpret_cast<const std::byte*>(source);
  std::size_t bytes=count*sizeof(Point3d);
  const auto head=std::min<std::size_t>(bytes,(64-(reinterpret_cast<std::uintptr_t>(dst)&63))&63);
  std::memcpy(dst,src,head);dst+=head;src+=head;bytes-=head;
  for(;bytes>=128;bytes-=128,dst+=128,src+=128) {
    const auto a=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src));
    const auto b=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src+32));
    const auto c=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src+64));
    const auto d=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src+96));
    _mm256_stream_si256(reinterpret_cast<__m256i*>(dst),a);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(dst+32),b);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(dst+64),c);
    _mm256_stream_si256(reinterpret_cast<__m256i*>(dst+96),d);
  }
  std::memcpy(dst,src,bytes);
}
void copy_point_runs_avx2(std::span<const Point3d> input,std::span<Point3d> output,
                         std::span<const std::pair<std::size_t,std::size_t>> runs) {
  std::size_t copied=0;bool streamed=false;
  for(const auto [first,last]:runs) {
    const auto count=last-first;
    if(output.size()>=32768 && count>=256) {
      copy_run_streaming(output.data()+copied,input.data()+first,count);streamed=true;
    } else std::memcpy(output.data()+copied,input.data()+first,count*sizeof(Point3d));
    copied+=count;
  }
  if(streamed)_mm_sfence();
}
std::size_t transform_points_avx2(std::span<const Point3d> points,std::span<Point3d> output,
                                const std::uint8_t* valid,const double* matrix) {
  static_assert(sizeof(Point3d)==3*sizeof(double) && std::is_standard_layout_v<Point3d>);
  static_assert(offsetof(Point3d,y)==sizeof(double) && offsetof(Point3d,z)==2*sizeof(double));
  // 无附加属性的大输出完整覆盖；掩码/自定义非对齐输出保留缓存写入。
  // 流式写入经 fence 后才对调用方可见，失败块按标量原序重放时也如此。
  const bool streaming=!valid && points.size()>=262144 &&
      (reinterpret_cast<std::uintptr_t>(output.data())&31)==0;
  bool streamed=false;
  const auto exponent=_mm256_set1_epi64x(0x7ff0000000000000LL);
  const auto finite=[&](__m256d x,__m256d y,__m256d z) {
    const auto invalid=[&](__m256d value) {
      return _mm256_cmpeq_epi64(_mm256_and_si256(_mm256_castpd_si256(value),exponent),exponent);
    };
    return _mm256_movemask_pd(_mm256_castsi256_pd(
        _mm256_or_si256(_mm256_or_si256(invalid(x),invalid(y)),invalid(z))))==0;
  };
  std::size_t i=0;
  for(;points.size()-i>=4;i+=4) {
    unsigned mask=15;
    if(valid)for(unsigned j=0;j<4;++j)if(!valid[i+j])mask&=~(1u<<j);
    if(!mask) {std::memcpy(output.data()+i,points.data()+i,4*sizeof(Point3d));continue;}
    // 四个 AoS 点恰占三个连续向量，无算术重排 lane，替代十二次独立标量读取。
    const auto* source=reinterpret_cast<const double*>(points.data()+i);
    const auto a=_mm256_loadu_pd(source),b=_mm256_loadu_pd(source+4),c=_mm256_loadu_pd(source+8);
    const auto xy=_mm256_permute2f128_pd(a,b,0x30);
    const auto zx=_mm256_permute2f128_pd(a,c,0x21);
    const auto yz=_mm256_permute2f128_pd(b,c,0x30);
    auto x=_mm256_shuffle_pd(xy,zx,0xa);
    auto y=_mm256_shuffle_pd(xy,yz,0x5);
    auto z=_mm256_shuffle_pd(zx,yz,0xa);
    if(mask!=15) {
      const auto active=_mm256_castsi256_pd(_mm256_set_epi64x((mask&8)?-1LL:0,(mask&4)?-1LL:0,(mask&2)?-1LL:0,(mask&1)?-1LL:0));
      x=_mm256_and_pd(x,active);y=_mm256_and_pd(y,active);z=_mm256_and_pd(z,active);
    }
    if(!finite(x,y,z))break;
    const auto apply=[&](unsigned k) {
      return _mm256_add_pd(_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(_mm256_set1_pd(matrix[k]),x),
          _mm256_mul_pd(_mm256_set1_pd(matrix[k+1]),y)),_mm256_mul_pd(_mm256_set1_pd(matrix[k+2]),z)),_mm256_set1_pd(matrix[k+3]));
    };
    const auto tx=apply(0),ty=apply(4),tz=apply(8);
    // 失败块按原标量输入序重放，无效槽不变。
    if(!finite(tx,ty,tz))break;
    if(mask==15) {
      // 将四个完整点重交错为三次连续 store；全有效路径不使用临时 XYZ 数组及逐字段写入，
      // 混合有效性保留原槽语义。
      const auto xy_low=_mm256_unpacklo_pd(tx,ty),xy_high=_mm256_unpackhi_pd(tx,ty);
      const auto zx=_mm256_unpacklo_pd(tz,xy_high),yz=_mm256_unpackhi_pd(xy_high,tz);
      auto* destination=reinterpret_cast<double*>(output.data()+i);
      const auto first=_mm256_permute2f128_pd(xy_low,zx,0x20);
      const auto second=_mm256_permute2f128_pd(yz,xy_low,0x30);
      const auto third=_mm256_permute2f128_pd(zx,yz,0x31);
      if(streaming) {
        _mm256_stream_pd(destination,first);
        _mm256_stream_pd(destination+4,second);
        _mm256_stream_pd(destination+8,third);streamed=true;
      } else {
        _mm256_storeu_pd(destination,first);
        _mm256_storeu_pd(destination+4,second);
        _mm256_storeu_pd(destination+8,third);
      }
      continue;
    }
    double xx[4],yy[4],zz[4];_mm256_storeu_pd(xx,tx);_mm256_storeu_pd(yy,ty);_mm256_storeu_pd(zz,tz);
    for(unsigned j=0;j<4;++j)output[i+j]=(mask&(1u<<j)) ? Point3d{xx[j],yy[j],zz[j]} : points[i+j];
  }
  if(streamed)_mm_sfence();
  return i;
}
template<std::size_t Axis,bool Column=false>
SelectionRuns scan(std::span<const Point3d> points,const std::uint8_t* valid,
                   double lower,double upper,std::size_t max_runs,Point3d* destination,const double* column=nullptr,
                   const double* ranges=nullptr) {
  static_assert(std::is_standard_layout_v<Point3d> && sizeof(Point3d)==3*sizeof(double));
  static_assert(offsetof(Point3d,y)==sizeof(double) && offsetof(Point3d,z)==2*sizeof(double));
  const auto lo=_mm256_set1_pd(lower),hi=_mm256_set1_pd(upper);
  const auto coordinate=[](const Point3d& p){if constexpr(Axis==0)return p.x;else if constexpr(Axis==1)return p.y;else return p.z;};
  SelectionRuns out;bool inside=false,streamed=false;std::size_t first=0,copied=0;
  out.runs.reserve(std::min<std::size_t>(max_runs,64));
  const auto close=[&](std::size_t last) {
    if(destination) {
      if(points.size()>=262144 && last-first>=64) {
        copy_run_streaming(destination+copied,points.data()+first,last-first);streamed=true;
      } else std::memcpy(destination+copied,points.data()+first,(last-first)*sizeof(Point3d));
      copied+=last-first;
    }
    if(out.runs.size()<max_runs)out.runs.emplace_back(first,last);
    else out.indexed=false;
    inside=false;
  };
  const auto accept=[&](std::size_t i,bool match) {
    if(match){++out.count;if(!inside){first=i;inside=true;}}
    else if(inside)close(i);
  };
  std::size_t i=0;
  // 坐标筛选通常有长游程，先分类十六点再更新游程；混合块保持点序递增。
  for(;points.size()-i>=16;i+=16) {
    if constexpr(Column) if(ranges && !valid && i%64==0 && points.size()-i>=64) {
      // 各对为十六个不可变坐标的精确 min/max；有序比较使非有限摘要块保留完整路径。
      const auto classify=[&](__m256d values) {
        const auto selected=_mm256_and_pd(_mm256_cmp_pd(values,lo,_CMP_GE_OQ),
                                         _mm256_cmp_pd(values,hi,_CMP_LE_OQ));
        const unsigned within=_mm256_movemask_pd(selected);
        const unsigned outside_min=_mm256_movemask_pd(_mm256_cmp_pd(values,hi,_CMP_GT_OQ));
        const unsigned outside_max=_mm256_movemask_pd(_mm256_cmp_pd(values,lo,_CMP_LT_OQ));
        return std::pair{within & (within>>1) & 5u,(outside_min | (outside_max>>1)) & 5u};
      };
      const auto a=classify(_mm256_loadu_pd(ranges+i/16*2));
      const auto b=classify(_mm256_loadu_pd(ranges+i/16*2+4));
      if(a.first==5 && b.first==5) {
        out.count+=64;if(!inside){first=i;inside=true;}i+=48;continue;
      }
      if(a.second==5 && b.second==5) {
        if(inside)close(i);i+=48;continue;
      }
    }
    unsigned mask=0;
    for(unsigned group=0;group<4;++group) {
      const auto j=i+group*4;
      const auto value=[&] {
        if constexpr(Column)return _mm256_loadu_pd(column+j);
        else return _mm256_setr_pd(coordinate(points[j]),coordinate(points[j+1]),coordinate(points[j+2]),coordinate(points[j+3]));
      }();
      mask|=static_cast<unsigned>(_mm256_movemask_pd(_mm256_and_pd(
          _mm256_cmp_pd(value,lo,_CMP_GE_OQ),_mm256_cmp_pd(value,hi,_CMP_LE_OQ))))<<(group*4);
    }
    if(valid) {
      const auto flags=_mm_loadu_si128(reinterpret_cast<const __m128i*>(valid+i));
      mask&=~static_cast<unsigned>(_mm_movemask_epi8(_mm_cmpeq_epi8(flags,_mm_setzero_si128())));
    }
    if(mask==65535){out.count+=16;if(!inside){first=i;inside=true;}}
    else if(mask==0){if(inside)close(i);}
    else {
      // 解码转换而不逐 lane 访问混合块；仅块以零结束时哨兵闭合末游程，尾部选中游程保持开放跨块。
      out.count+=std::popcount(mask);
      unsigned k=0;
      while(k<16) {
        const unsigned skipped=std::min<unsigned>(std::countr_zero(mask>>k),16-k);
        if(skipped && inside)close(i+k);
        k+=skipped;
        if(k==16)break;
        if(!inside){first=i+k;inside=true;}
        k+=std::min<unsigned>(std::countr_one(mask>>k),16-k);
        if(k<16)close(i+k);
      }
    }
  }
  for(;points.size()-i>=4;i+=4) {
    const auto value=[&] {
      if constexpr(Column)return _mm256_loadu_pd(column+i);
      else return _mm256_setr_pd(coordinate(points[i]),coordinate(points[i+1]),coordinate(points[i+2]),coordinate(points[i+3]));
    }();
    auto mask=_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(value,lo,_CMP_GE_OQ),
                                               _mm256_cmp_pd(value,hi,_CMP_LE_OQ)));
    if(valid)for(unsigned k=0;k<4;++k)if(!valid[i+k])mask&=~(1<<k);
    if(mask==15){out.count+=4;if(!inside){first=i;inside=true;}}
    else if(mask==0){if(inside)close(i);}
    else for(unsigned k=0;k<4;++k)accept(i+k,(mask&(1<<k))!=0);
  }
  for(;i<points.size();++i) {
    const double value=Column ? column[i] : coordinate(points[i]);
    accept(i,(!valid || valid[i]) && value>=lower && value<=upper);
  }
  if(inside)close(points.size());
  if(streamed)_mm_sfence();
  return out;
}
SelectionRuns coordinate_runs_avx2(std::span<const Point3d> points,const std::uint8_t* valid,
    std::size_t coordinate,double lower,double upper,std::size_t max_runs,Point3d* destination,const double* column,
    const double* ranges) {
  if(column)return scan<0,true>(points,valid,lower,upper,max_runs,destination,column,ranges);
  switch(coordinate){case 0:return scan<0>(points,valid,lower,upper,max_runs,destination);
    case 1:return scan<1>(points,valid,lower,upper,max_runs,destination);default:return scan<2>(points,valid,lower,upper,max_runs,destination);}
}
}
