#include "tuple_input.hpp"
#include <mvkit/c/mv_region.h>
#include <algorithm>
#include <cfenv>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <limits>
#include <optional>
#include <vector>
#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include "regions_cache.hpp"
#include "structured_error.hpp"
#include <core/exception.hpp>
#include <core/runtime_context.hpp>
#include <region/operators.hpp>
namespace{
using mv::c_api::Exception;using mv::core::Status;
void req(::MvObject h,const char*w){if(!h)throw Exception(Status::InvalidArgument,w,"regions handle must not be null");h->value.require_kind(mv::core::IconicKind::Region,w);}
void out(const void*p,const char*w){if(!p)throw Exception(Status::InvalidArgument,w,"output pointer must not be null");}
void outs(std::initializer_list<const void*> values,const char*w){
  for (std::size_t i=0;i<values.size();++i) {
    const auto* value=values.begin()[i];out(value,w);
    const auto a=reinterpret_cast<std::uintptr_t>(value);
    // 只比较已验证的前缀，保持空指针、重叠输出的原检查顺序。
    for(std::size_t j=0;j<i;++j) {
      const auto b=reinterpret_cast<std::uintptr_t>(values.begin()[j]);
      if((a<=b?b-a:a-b)<sizeof(::MvTuple))
        throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,w,"outputs",static_cast<int64_t>(i),"parallel Tuple output byte ranges must be distinct");
    }
  }
}


std::unique_ptr<MvTupleImpl> tuple(){return std::make_unique<MvTupleImpl>();}
std::unique_ptr<MvTupleImpl> tuple(const mv::core::Tuple& value){return std::make_unique<MvTupleImpl>(value);}
}
extern "C" ::MvStatus mv_union1(::MvObject a,::MvObject*o){return mv::c_api::translate([&]{req(a,"mv_union1");out(o,"mv_union1");*o=a->value.empty()?mv::c_api::make_regions(mv::core::Regions{}):mv::c_api::make_regions(mv::regionproc::union1(mv::c_api::regions_value(a,"mv_union1")));});}
#define BIN(NAME,FN) extern "C" ::MvStatus NAME(::MvObject a,::MvObject b,::MvObject*o){return mv::c_api::translate([&]{req(a,#NAME);req(b,#NAME);out(o,#NAME);*o=mv::c_api::make_regions(mv::regionproc::FN(mv::c_api::regions_value(a,#NAME,"a"),mv::c_api::regions_value(b,#NAME,"b")));});}
extern "C" ::MvStatus mv_union2(::MvObject a,::MvObject b,::MvObject*o){return mv::c_api::translate([&]{req(a,"mv_union2");req(b,"mv_union2");out(o,"mv_union2");const auto left=mv::c_api::regions_value(a,"mv_union2"),right=mv::c_api::regions_value(b,"mv_union2");*o=mv::c_api::make_regions(left.empty()&&!right.empty()?mv::core::Regions{mv::regionproc::union1(right)}:mv::regionproc::union2(left,right));});}
extern "C" ::MvStatus mv_intersection(::MvObject a,::MvObject b,::MvObject*o){return mv::c_api::translate([&]{req(a,"mv_intersection");req(b,"mv_intersection");out(o,"mv_intersection");*o=mv::c_api::make_regions(b->value.empty()?mv::core::Regions{}:mv::regionproc::intersection(mv::c_api::regions_value(a,"mv_intersection"),mv::c_api::regions_value(b,"mv_intersection")));});}
BIN(mv_difference,difference)
#undef BIN
extern "C" ::MvStatus mv_connection(::MvObject a,::MvObject*o){return mv::c_api::translate([&]{req(a,"mv_connection");out(o,"mv_connection");*o=mv::c_api::make_regions(mv::regionproc::connection(mv::c_api::regions_value(a,"mv_connection"),mv::core::current_runtime_context()));});}
extern "C" ::MvStatus mv_fill_up(::MvObject a,::MvObject*o){return mv::c_api::translate([&]{req(a,"mv_fill_up");out(o,"mv_fill_up");*o=mv::c_api::make_regions(mv::regionproc::fill_up(mv::c_api::regions_value(a,"mv_fill_up"),mv::core::current_runtime_context()));});}
namespace mv::c_api {
void ensure_centroid_cache(MvObjectImpl& handle) {
  auto& cache=handle.ExportCache();
  cache.ensure_feature(MvIconicExportCache::Centroid,cache.centroid_cache_once,[&] {
    const auto objects=handle.value.values();
    std::vector<std::int64_t> areas(objects.size());
    std::vector<double> real_areas(objects.size()),rows(objects.size()),columns(objects.size());
    for(std::size_t i=0;i<objects.size();++i) {
      const auto value=mv::regionproc::area_center(objects[i].as_region());
      if(value.area>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw mv::core::Exception(mv::core::Status::OutOfRange,mv::core::ErrorCode::InvalidValue,
          "area_center","Regions",static_cast<int64_t>(i),"area exceeds signed integer tuple range");
      areas[i]=static_cast<std::int64_t>(value.area);real_areas[i]=double(value.area);rows[i]=value.row;columns[i]=value.column;
    }
    cache.cached_area=mv::core::Tuple{std::move(areas)};
    cache.cached_real_area=mv::core::Tuple{std::move(real_areas)};
    cache.cached_row=mv::core::Tuple{std::move(rows)};cache.cached_column=mv::core::Tuple{std::move(columns)};
  });
}
}
extern "C" ::MvStatus mv_area_center(::MvObject a,::MvTuple* area,::MvTuple* row,::MvTuple* column) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_area_center";req(a,op);outs({area,row,column},op);
    mv::c_api::ensure_centroid_cache(*a);
    const auto& cache=a->ExportCache();
    auto aa=tuple(cache.cached_area),rr=tuple(cache.cached_row),cc=tuple(cache.cached_column);
    *area=aa.release();*row=rr.release();*column=cc.release();
  });
}
extern "C" ::MvStatus mv_smallest_rectangle1(::MvObject a,::MvTuple* row1,::MvTuple* column1,::MvTuple* row2,::MvTuple* column2) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_smallest_rectangle1";req(a,op);outs({row1,column1,row2,column2},op);
    const auto objects=a->value.values();
    if(objects.size()==1) {
      const auto box=objects[0].as_region().bounding_box();
      auto r1=tuple(),c1=tuple(),r2=tuple(),c2=tuple();
      r1->value.append(std::int64_t(box?box->row_min:0));c1->value.append(std::int64_t(box?box->col_min:0));
      r2->value.append(std::int64_t(box?box->row_max:0));c2->value.append(std::int64_t(box?box->col_max:0));
      *row1=r1.release();*column1=c1.release();*row2=r2.release();*column2=c2.release();return;
    }
    std::vector<std::int64_t> rows1,cols1,rows2,cols2;
    // 多对象仍一次预分配全部列，单对象直接使用内联标量结果。
    rows1.reserve(objects.size());cols1.reserve(objects.size());rows2.reserve(objects.size());cols2.reserve(objects.size());
    for(std::size_t i=0;i<objects.size();++i) {
      const auto box=objects[i].as_region().bounding_box();
      rows1.push_back(box?box->row_min:0);cols1.push_back(box?box->col_min:0);
      rows2.push_back(box?box->row_max:0);cols2.push_back(box?box->col_max:0);
    }
    auto r1=tuple(),c1=tuple(),r2=tuple(),c2=tuple();
    r1->value=mv::core::Tuple{std::move(rows1)};c1->value=mv::core::Tuple{std::move(cols1)};
    r2->value=mv::core::Tuple{std::move(rows2)};c2->value=mv::core::Tuple{std::move(cols2)};
    *row1=r1.release();*column1=c1.release();*row2=r2.release();*column2=c2.release();
  });
}
extern "C" ::MvStatus mv_smallest_rectangle2(::MvObject a,::MvTuple* row,::MvTuple* column,::MvTuple* phi,::MvTuple* length1,::MvTuple* length2) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_smallest_rectangle2";req(a,op);outs({row,column,phi,length1,length2},op);
    auto& cache=a->ExportCache();
    cache.ensure_feature(MvIconicExportCache::Rectangle2,cache.rectangle2_cache_once,[&] {
      const auto objects=a->value.values();
      std::array<std::vector<double>,5> values;
      for(auto& values_of_one:values)values_of_one.resize(objects.size());
      for(std::size_t i=0;i<objects.size();++i) {
        const auto v=mv::regionproc::smallest_rectangle2(objects[i].as_region());
        const double phi=-v.phi;
        values[0][i]=v.row;values[1][i]=v.column;values[2][i]=phi>=3.14159265358979323846/2?phi-3.14159265358979323846:phi;values[3][i]=v.length1;values[4][i]=v.length2;
      }
      for(std::size_t k=0;k<5;++k)cache.cached_rectangle2[k]=mv::core::Tuple{std::move(values[k])};
    });
    auto rr=tuple(cache.cached_rectangle2[0]),cc=tuple(cache.cached_rectangle2[1]),pp=tuple(cache.cached_rectangle2[2]),l1=tuple(cache.cached_rectangle2[3]),l2=tuple(cache.cached_rectangle2[4]);
    *row=rr.release();*column=cc.release();*phi=pp.release();*length1=l1.release();*length2=l2.release();
  });
}
extern "C" ::MvStatus mv_smallest_circle(::MvObject a,::MvTuple*row,::MvTuple*column,::MvTuple*radius) {
  return mv::c_api::translate([&] {
    constexpr auto op="mv_smallest_circle";req(a,op);outs({row,column,radius},op);
    auto& cache=a->ExportCache();
    // Region 句柄不可变，每句柄只发布一次精确几何；返回 Tuple 共享不可变存储并保留其生命周期。
    cache.ensure_feature(MvIconicExportCache::Circle,cache.circle_cache_once,[&] {
      const auto regions=mv::c_api::regions_value(a,op);
      const auto circles=mv::regionproc::smallest_circle(regions);
      std::array<std::vector<double>,3> values;
      for(auto& one:values)one.resize(circles.size());
      for(std::size_t i=0;i<circles.size();++i) {
        values[0][i]=circles[i].row;values[1][i]=circles[i].column;values[2][i]=circles[i].radius+(regions[i].empty()?0.:.5);
      }
      auto circles_cache=std::make_unique<std::array<mv::core::Tuple,3>>();
      for(std::size_t k=0;k<3;++k)(*circles_cache)[k]=mv::core::Tuple{std::move(values[k])};
      cache.cached_circle=std::move(circles_cache);
    });
    auto rr=tuple((*cache.cached_circle)[0]),cc=tuple((*cache.cached_circle)[1]),rad=tuple((*cache.cached_circle)[2]);
    *row=rr.release();*column=cc.release();*radius=rad.release();
  });
}



extern "C" ::MvStatus mv_circularity(::MvObject regions,::MvTuple* values) {
  return mv::c_api::translate([&] {constexpr auto op="mv_circularity";req(regions,op);out(values,op);
    auto& cache=regions->ExportCache();
    // 区域图标载荷不可变；首次完整计算，后续返回独立的拥有型 Tuple。
    cache.ensure_feature(MvIconicExportCache::Circularity,cache.circularity_once,[&] {
      const auto objects=mv::c_api::regions_value(regions,op);
      const auto original=mv::regionproc::circularity(objects);
      const auto source=original.empty()?std::span<const double>{}:original.real64_span();
      std::vector<double> output(objects.size());
      for(std::size_t i=0;i<objects.size();++i)output[i]=objects[i].area()==1?0.:source[i];
      cache.cached_circularity=mv::core::Tuple{std::move(output)};
    });
    auto result=tuple(cache.cached_circularity);
    *values=result.release();});
}
extern "C" ::MvStatus mv_orientation_region(::MvObject regions,::MvTuple* values) {
  return mv::c_api::translate([&] {constexpr auto op="mv_orientation_region";req(regions,op);out(values,op);
    const auto compute=[&] {
    const auto objects=mv::c_api::regions_value(regions,op);const auto axes=mv::regionproc::orientation_region(objects);mv::core::Tuple result;
    constexpr double pi=3.14159265358979323846;
    for(std::size_t i=0;i<objects.size();++i) {
      if(objects[i].empty()){result.append(0.);continue;}
      double phi=-axes.at(i).as_real64();const auto center=mv::regionproc::area_center(objects[i]);
      const auto box=*objects[i].bounding_box();
      // 水平线的等效椭圆退化，HALCON 保留其轴。
      if(box.row_min==box.row_max){result.append(phi);continue;}
      const double cosine=std::cos(phi),sine=std::sin(phi);
      double farthest=-1,projection=0;
      for(const auto& run:objects[i].runs())for(auto c:{run.col_begin,run.col_end}) {
        const double dr=double(run.row)-center.row,dc=double(c)-center.column,distance=dr*dr+dc*dc;
        if(distance>farthest+1e-12*std::max(1.,farthest)){farthest=distance;projection=dc*cosine-dr*sine;}
      }
      if(projection< -1e-12*std::max(1.,std::sqrt(farthest)))phi+=pi;if(phi>=pi)phi-=2*pi;if(phi< -pi)phi+=2*pi;result.append(phi);
    }
    return result;
    };
    // 不可变句柄的重复查询复用原算法结果；非默认舍入模式不读取或发布此缓存。
    // 全部计算成功才发布，失败可重试；返回 Tuple 与缓存的可变状态独立。
    if(std::fegetround()==FE_TONEAREST) {
      auto& cache=regions->ExportCache();
      cache.ensure_feature(MvIconicExportCache::Orientation,cache.orientation_once,[&] {
        cache.cached_orientation=compute();
      });
      auto result=tuple(cache.cached_orientation);*values=result.release();return;
    }
    auto result=std::make_unique<MvTupleImpl>(compute());
    *values=result.release();
  });
}
extern "C" ::MvStatus mv_eccentricity(::MvObject regions,::MvTuple* anisometry,::MvTuple* bulkiness,::MvTuple* structure_factor) {
  return mv::c_api::translate([&] {constexpr auto op="mv_eccentricity";req(regions,op);outs({anisometry,bulkiness,structure_factor},op);
    auto& cache=regions->ExportCache();
    cache.ensure_feature(MvIconicExportCache::Eccentricity,cache.eccentricity_once,[&] {
      const auto objects=mv::c_api::regions_value(regions,op);
      auto result=mv::regionproc::eccentricity(objects);
      const auto av=result.anisometry.empty()?std::span<const double>{}:result.anisometry.real64_span();
      const auto bv=result.bulkiness.empty()?std::span<const double>{}:result.bulkiness.real64_span();
      const auto cv=result.structure_factor.empty()?std::span<const double>{}:result.structure_factor.real64_span();
      std::vector<double> output(objects.size());
      for(std::size_t i=0;i<objects.size();++i)output[i]=!objects[i].empty()&&av[i]==0&&bv[i]==0?-1.:cv[i];
      cache.cached_eccentricity={std::move(result.anisometry),std::move(result.bulkiness),mv::core::Tuple{std::move(output)}};
    });
    auto a=tuple(cache.cached_eccentricity[0]),b=tuple(cache.cached_eccentricity[1]),c=tuple(cache.cached_eccentricity[2]);
    *anisometry=a.release();*bulkiness=b.release();*structure_factor=c.release();
  });
}



extern "C" ::MvStatus mv_symm_difference(::MvObject a,::MvObject b,::MvObject* o) {
 return mv::c_api::translate([&]{req(a,"mv_symm_difference");req(b,"mv_symm_difference");out(o,"mv_symm_difference");*o=mv::c_api::make_regions(mv::regionproc::symm_difference(mv::c_api::regions_value(a,"mv_symm_difference"),mv::c_api::regions_value(b,"mv_symm_difference")));});
}
extern "C" ::MvStatus mv_test_equal_region(::MvObject a,::MvObject b,::MvTuple* o) {
 return mv::c_api::translate([&]{req(a,"mv_test_equal_region");req(b,"mv_test_equal_region");out(o,"mv_test_equal_region");auto h=tuple();h->value=(a->value.empty()||b->value.empty())?mv::core::Tuple{}:mv::regionproc::test_equal_region(mv::c_api::regions_value(a,"mv_test_equal_region"),mv::c_api::regions_value(b,"mv_test_equal_region"));*o=h.release();});
}


extern "C" ::MvStatus mv_gen_region_polygon_filled(MvObject* Region,::MvTuple Rows,::MvTuple Columns) {
 return mv::c_api::translate([&]{constexpr auto op="mv_gen_region_polygon_filled";out(Region,op);
 const mv::c_api::ControlTupleView rows(Rows,op,"Rows"),columns(Columns,op,"Columns");
 *Region=mv::c_api::make_regions(mv::regionproc::gen_region_polygon_filled(rows,columns,true));});
}






extern "C" ::MvStatus mv_skeleton(::MvObject regions,::MvObject* output) {
 return mv::c_api::translate([&]{constexpr auto op="mv_skeleton";req(regions,op);out(output,op);
 *output=mv::c_api::make_regions(mv::regionproc::skeleton(mv::c_api::regions_value(regions,op)));});
}
extern "C" ::MvStatus mv_partition_rectangle(MvObject Region,MvObject* Partitioned,::MvTuple Width,::MvTuple Height) {
 return mv::c_api::translate([&]{constexpr auto op="mv_partition_rectangle";out(Partitioned,op);
 const auto regions=mv::c_api::regions_value(Region,op,"Region");
 const double width=mv::c_api::TupleInput(Width,op,"Width").scalar_number(),height=mv::c_api::TupleInput(Height,op,"Height").scalar_number();
 *Partitioned=mv::c_api::make_regions(mv::regionproc::partition_rectangle(regions,width,height,true));});
}
extern "C" ::MvStatus mv_partition_dynamic(MvObject Region,MvObject* Partitioned,::MvTuple Distance,::MvTuple Percent) {
 return mv::c_api::translate([&]{constexpr auto op="mv_partition_dynamic";out(Partitioned,op);
 const auto regions=mv::c_api::regions_value(Region,op,"Region");
 const double distance=mv::c_api::TupleInput(Distance,op,"Distance").scalar_number(),percent=mv::c_api::TupleInput(Percent,op,"Percent").scalar_number();
 *Partitioned=mv::c_api::make_regions(mv::regionproc::partition_dynamic(regions,distance,percent,true));});
}
