#include <c_api/storage_access.hpp>
#include <mvkit/c/mv_region.h>

#include <atomic>
#include <limits>
#include <vector>
#include <runtime/memory/memory_resource.hpp>
#include <cmath>
#include "control_operator_outputs.hpp"
#include "tuple_input.hpp"

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "regions_cache.hpp"
#include "structured_error.hpp"
#include <core/exception.hpp>


namespace {
using mv::c_api::Exception;
using mv::core::Status;
}

extern "C" MvStatus mv_gen_empty_region(MvObject* EmptyRegion) {
  return mv::c_api::translate([&] {
    mv::c_api::require_output(EmptyRegion,"gen_empty_region","EmptyRegion");
    *EmptyRegion=mv::c_api::make_regions(mv::core::Region{});
  });
}

extern "C" MvStatus mv_gen_region_runs(MvObject* Region,MvTuple Row,MvTuple ColumnBegin,MvTuple ColumnEnd) {
  return mv::c_api::translate([&] {
    constexpr auto op="gen_region_runs";
    mv::c_api::require_output(Region,op,"Region");
    const mv::c_api::TupleInput rows(Row,op,"Row"),begins(ColumnBegin,op,"ColumnBegin"),ends(ColumnEnd,op,"ColumnEnd");
    const auto count=rows.size();
    if(begins.size()!=count)begins.fail(mv::core::ErrorCode::InvalidCardinality,-1,"equal run coordinate lengths required");
    if(ends.size()!=count)ends.fail(mv::core::ErrorCode::InvalidCardinality,-1,"equal run coordinate lengths required");
    const auto coordinate=[](const mv::c_api::TupleInput& input,size_t i) {
      // 保留实数输入的既有 +0.5 后截断规则，再检查实际栅格坐标。
      const auto number=input.at(i).kind()==mv::core::ValueKind::Int64
          ? static_cast<double>(input.integer(i)) : std::trunc(input.number(i)+.5);
      if(!std::isfinite(number)||number<mv::core::region_coordinate_min||number>mv::core::region_coordinate_max)
        input.fail_range(i,"run coordinate exceeds int32 range");
      return static_cast<std::int32_t>(number);
    };
    std::pmr::vector<mv::core::RegionRun> runs{mv::runtime::persist_resource()};
    runs.reserve(count);
    bool canonical = true;
    const auto* row_values=rows.owned_tuple();
    const auto* begin_values=begins.owned_tuple();
    const auto* end_values=ends.owned_tuple();
    const bool integers=row_values && begin_values && end_values &&
        row_values->kind()==mv::core::TupleKind::Int64 && begin_values->kind()==mv::core::TupleKind::Int64 &&
        end_values->kind()==mv::core::TupleKind::Int64;
    const auto rr=integers?row_values->int64_span():std::span<const std::int64_t>{};
    const auto aa=integers?begin_values->int64_span():std::span<const std::int64_t>{};
    const auto bb=integers?end_values->int64_span():std::span<const std::int64_t>{};
    const auto checked=[](std::int64_t v,const mv::c_api::TupleInput& input,std::size_t i) {
      if(v<mv::core::region_coordinate_min || v>mv::core::region_coordinate_max)
        input.fail_range(i,"run coordinate exceeds int32 range");
      return static_cast<std::int32_t>(v);
    };
    // 输入调用期间不可变；长度只读取一次，端点先验证后窄化。
    for(size_t i=0;i<count;++i) {
      // 同质整数只准备一次跨度；仍先查 begin/end，再查区间顺序，最后查 row。
      const auto begin=integers?checked(aa[i],begins,i):coordinate(begins,i);
      const auto end=integers?checked(bb[i],ends,i):coordinate(ends,i);
      if(begin>end)ends.fail(mv::core::ErrorCode::InvalidValue,i,"run end must not precede its beginning");
      runs.push_back({integers?checked(rr[i],rows,i):coordinate(rows,i),begin,end});
      if (i) {
        const auto& previous = runs[i-1];
        const auto& current = runs[i];
        canonical = canonical && (current.row > previous.row ||
            (current.row == previous.row && std::int64_t(current.col_begin) > std::int64_t(previous.col_end) + 1));
      }
    }
    try {
      // 输入已经规范时直接转移，未排序、重复和相邻行程仍走原规范化路径。
      auto region = canonical ? mv::core::Region::from_canonical_runs(std::move(runs)) : mv::core::Region::from_runs(runs);
      *Region=mv::c_api::make_regions(std::move(region));
    }
    catch(const mv::core::Exception& error) {
      throw mv::core::Exception(error.status(),error.error_code(),op,"Region",-1,error.message());
    }
  });
}

extern "C" MvStatus mv_get_region_runs(MvObject Region,MvTuple* Row,MvTuple* ColumnBegin,MvTuple* ColumnEnd) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_region_runs";
    mv::c_api::ControlOutputs<3> result(op,{Row,ColumnBegin,ColumnEnd});
    mv::c_api::require_iconic(Region,op,"Region");
    Region->value.require_kind(mv::core::IconicKind::Region,op);
    if(Region->value.size()>1)throw mv::core::Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,
        op,"Region",-1,"one region or a zero-object array required");
    if(!Region->value.empty()) {
      const auto runs = Region->value.at(0).as_region().runs();
      std::vector<std::int64_t> rows(runs.size()), begins(runs.size()), ends(runs.size());
      for (std::size_t i = 0; i < runs.size(); ++i) {
        rows[i]=runs[i].row; begins[i]=runs[i].col_begin; ends[i]=runs[i].col_end;
      }
      result[0]=mv::core::Tuple{std::move(rows)};
      result[1]=mv::core::Tuple{std::move(begins)};
      result[2]=mv::core::Tuple{std::move(ends)};
    }
    result.Commit();
  });
}
