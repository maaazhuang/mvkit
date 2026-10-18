#include <geometry/analytic2d.hpp>

#include <cmath>
#include <array>
#include <algorithm>
#include <vector>
#include <utility>

#include <core/broadcast.hpp>
#include <core/exception.hpp>
#if MVKIT_GEOMETRY_HAS_AVX2
#include <geometry/analytic2d_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>
#endif

namespace mv::geometry {
namespace {

void require(bool ok, const char* op, const char* parameter, const char* message) {
  if (!ok) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op,
                          parameter, -1, message);
  }
}

double numeric(const core::Tuple& tuple, std::size_t index, const char* op, const char* parameter) {
  double v = 0;
  try {
    v = tuple.as_real64(index);
  } catch (const core::Exception&) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, parameter,
                          static_cast<std::int64_t>(index), "coordinates must be numeric");
  }
  require(std::isfinite(v), op, parameter, "finite coordinates required");
  return v;
}

// 数值类型在循环前准备；通用输入仍保留原类型、有限值和参数检查顺序。
struct NumericInput {
  const core::Tuple& tuple;
  const char* operation;
  const char* parameter;
  std::span<const double> values;
  NumericInput(const core::Tuple& t, const char* op, const char* name)
      : tuple(t), operation(op), parameter(name),
        values(t.kind()==core::TupleKind::Real64 ? t.real64_span() : std::span<const double>{}) {}
  template<bool Real> double get(std::size_t index) const {
    if constexpr (!Real) return numeric(tuple,index,operation,parameter);
    const double value=values[index];
    require(std::isfinite(value),operation,parameter,"finite coordinates required");
    return value;
  }
};

constexpr double kParallelEps = 1e-14;

}  // namespace

void distance_pp(const core::Tuple& row1, const core::Tuple& column1, const core::Tuple& row2,
                 const core::Tuple& column2, core::Tuple* distance) {
  constexpr auto op = "distance_pp";
  if (distance == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op,
                          "Distance", -1, "output must not be null");
  }
  const auto plan = core::BroadcastPlan::make({{row1.size(), "Row1", true},
                                               {column1.size(), "Column1", true},
                                               {row2.size(), "Row2", true},
                                               {column2.size(), "Column2", true}},
                                              op);
  const std::array inputs{NumericInput{row1,op,"Row1"}, NumericInput{column1,op,"Column1"}, NumericInput{row2,op,"Row2"}, NumericInput{column2,op,"Column2"}};
  const bool all_real=std::all_of(inputs.begin(),inputs.end(),[](const auto& input){return !input.values.empty();});
  std::vector<double> out(plan.size());
#if MVKIT_GEOMETRY_HAS_AVX2
  const auto& cpu=mv::runtime::cpu_features();
  const bool avx2=mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(
      cpu.selection(),cpu.hardware()),{false,true,false,false});
#endif
  const auto calculate=[&]<bool Real>() {
    std::size_t first=0;
#if MVKIT_GEOMETRY_HAS_AVX2
    if constexpr(Real) {
      if(avx2)first=distance_pp_real_avx2(inputs[0].values,inputs[1].values,inputs[2].values,inputs[3].values,out);
    }
#endif
    for (std::size_t i = first; i < plan.size(); ++i) {
      const double r1 = inputs[0].template get<Real>(plan.source_index(0, i));
      const double c1 = inputs[1].template get<Real>(plan.source_index(1, i));
      const double r2 = inputs[2].template get<Real>(plan.source_index(2, i));
      const double c2 = inputs[3].template get<Real>(plan.source_index(3, i));
      out[i] = std::hypot(r2 - r1, c2 - c1);
    }
  };
  if(all_real) calculate.template operator()<true>();
  else calculate.template operator()<false>();
  *distance = core::Tuple{std::move(out)};
}

void distance_pl(const core::Tuple& row, const core::Tuple& column, const core::Tuple& row1,
                 const core::Tuple& column1, const core::Tuple& row2, const core::Tuple& column2,
                 core::Tuple* distance) {
  constexpr auto op = "distance_pl";
  if (distance == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op,
                          "Distance", -1, "output must not be null");
  }
  const auto plan = core::BroadcastPlan::make({{row.size(), "Row", true},
                                               {column.size(), "Column", true},
                                               {row1.size(), "Row1", true},
                                               {column1.size(), "Column1", true},
                                               {row2.size(), "Row2", true},
                                               {column2.size(), "Column2", true}},
                                              op);
  const std::array inputs{NumericInput{row,op,"Row"}, NumericInput{column,op,"Column"}, NumericInput{row1,op,"Row1"}, NumericInput{column1,op,"Column1"}, NumericInput{row2,op,"Row2"}, NumericInput{column2,op,"Column2"}};
  const bool all_real=std::all_of(inputs.begin(),inputs.end(),[](const auto& input){return !input.values.empty();});
  std::vector<double> out(plan.size());
  if(all_real && plan.size() && row1.size()==1 && column1.size()==1 && row2.size()==1 && column2.size()==1) {
    // 首点按原参数顺序验证后才准备常量直线；无效直线仍先于后续点的错误。
    (void)inputs[0].get<true>(0);(void)inputs[1].get<true>(0);
    const auto r1=inputs[2].get<true>(0),c1=inputs[3].get<true>(0);
    const auto r2=inputs[4].get<true>(0),c2=inputs[5].get<true>(0);
    const auto dr=r2-r1,dc=c2-c1,len=std::hypot(dr,dc);
    require(len>0.,op,"Line","line points must not coincide");
    const bool single_row=row.size()==1,single_column=column.size()==1;
    for(std::size_t i=0;i<plan.size();++i) {
      const auto r=inputs[0].get<true>(single_row?0:i),c=inputs[1].get<true>(single_column?0:i);
      out[i]=std::abs((c-c1)*dr-(r-r1)*dc)/len;
    }
    *distance=core::Tuple{std::move(out)};
    return;
  }
  const auto calculate=[&]<bool Real>() {
    for (std::size_t i = 0; i < plan.size(); ++i) {
      const double r = inputs[0].template get<Real>(plan.source_index(0, i));
      const double c = inputs[1].template get<Real>(plan.source_index(1, i));
      const double r1 = inputs[2].template get<Real>(plan.source_index(2, i));
      const double c1 = inputs[3].template get<Real>(plan.source_index(3, i));
      const double r2 = inputs[4].template get<Real>(plan.source_index(4, i));
      const double c2 = inputs[5].template get<Real>(plan.source_index(5, i));
      const double dr = r2 - r1;
      const double dc = c2 - c1;
      const double len = std::hypot(dr, dc);
      require(len > 0.0, op, "Line", "line points must not coincide");
      out[i] = std::abs((c - c1) * dr - (r - r1) * dc) / len;
    }
  };
  if(all_real) calculate.template operator()<true>();
  else calculate.template operator()<false>();
  *distance = core::Tuple{std::move(out)};
}

void intersection_lines(const core::Tuple& row_a1, const core::Tuple& column_a1,
                        const core::Tuple& row_a2, const core::Tuple& column_a2,
                        const core::Tuple& row_b1, const core::Tuple& column_b1,
                        const core::Tuple& row_b2, const core::Tuple& column_b2, core::Tuple* row,
                        core::Tuple* column, core::Tuple* is_overlapping) {
  constexpr auto op = "intersection_lines";
  if (row == nullptr || column == nullptr || is_overlapping == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op,
                          "outputs", -1, "outputs must not be null");
  }
  if (row == column || row == is_overlapping || column == is_overlapping) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op,
                          "outputs", -1, "parallel Tuple outputs must be distinct");
  }
  const auto plan = core::BroadcastPlan::make({{row_a1.size(), "RowA1", true},
                                               {column_a1.size(), "ColumnA1", true},
                                               {row_a2.size(), "RowA2", true},
                                               {column_a2.size(), "ColumnA2", true},
                                               {row_b1.size(), "RowB1", true},
                                               {column_b1.size(), "ColumnB1", true},
                                               {row_b2.size(), "RowB2", true},
                                               {column_b2.size(), "ColumnB2", true}},
                                              op);
  core::Tuple orow;
  core::Tuple ocol;
  core::Tuple overlap;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const double ra1 = numeric(row_a1, plan.source_index(0, i), op, "RowA1");
    const double ca1 = numeric(column_a1, plan.source_index(1, i), op, "ColumnA1");
    const double ra2 = numeric(row_a2, plan.source_index(2, i), op, "RowA2");
    const double ca2 = numeric(column_a2, plan.source_index(3, i), op, "ColumnA2");
    const double rb1 = numeric(row_b1, plan.source_index(4, i), op, "RowB1");
    const double cb1 = numeric(column_b1, plan.source_index(5, i), op, "ColumnB1");
    const double rb2 = numeric(row_b2, plan.source_index(6, i), op, "RowB2");
    const double cb2 = numeric(column_b2, plan.source_index(7, i), op, "ColumnB2");
    const double dax = ca2 - ca1;
    const double day = ra2 - ra1;
    const double dbx = cb2 - cb1;
    const double dby = rb2 - rb1;
    require(std::hypot(dax, day) > 0.0 && std::hypot(dbx, dby) > 0.0, op, "Line",
            "line points must not coincide");
    const double den = dax * dby - day * dbx;
    const double dx = cb1 - ca1;
    const double dy = rb1 - ra1;
    const double collinear = dx * day - dy * dax;
    if (std::abs(den) <= kParallelEps * (std::hypot(dax, day) * std::hypot(dbx, dby) + 1.0)) {
      if (std::abs(collinear) <= kParallelEps * (std::hypot(dax, day) * std::hypot(dx, dy) + 1.0)) {
        orow.append(ra1);
        ocol.append(ca1);
        overlap.append(std::int64_t{1});
        continue;
      }
      throw core::Exception(core::Status::InvalidState, op, "infinite lines are parallel");
    }
    const double t = (dx * dby - dy * dbx) / den;
    orow.append(ra1 + t * day);
    ocol.append(ca1 + t * dax);
    overlap.append(std::int64_t{0});
  }
  *row = std::move(orow);
  *column = std::move(ocol);
  *is_overlapping = std::move(overlap);
}

void angle_ll(const core::Tuple& row_a1, const core::Tuple& column_a1, const core::Tuple& row_a2,
              const core::Tuple& column_a2, const core::Tuple& row_b1, const core::Tuple& column_b1,
              const core::Tuple& row_b2, const core::Tuple& column_b2, core::Tuple* angle) {
  constexpr auto op = "angle_ll";
  if (angle == nullptr) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::NullArgument, op, "Angle",
                          -1, "output must not be null");
  }
  const auto plan = core::BroadcastPlan::make({{row_a1.size(), "RowA1", true},
                                               {column_a1.size(), "ColumnA1", true},
                                               {row_a2.size(), "RowA2", true},
                                               {column_a2.size(), "ColumnA2", true},
                                               {row_b1.size(), "RowB1", true},
                                               {column_b1.size(), "ColumnB1", true},
                                               {row_b2.size(), "RowB2", true},
                                               {column_b2.size(), "ColumnB2", true}},
                                              op);
  core::Tuple out;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const double ra1 = numeric(row_a1, plan.source_index(0, i), op, "RowA1");
    const double ca1 = numeric(column_a1, plan.source_index(1, i), op, "ColumnA1");
    const double ra2 = numeric(row_a2, plan.source_index(2, i), op, "RowA2");
    const double ca2 = numeric(column_a2, plan.source_index(3, i), op, "ColumnA2");
    const double rb1 = numeric(row_b1, plan.source_index(4, i), op, "RowB1");
    const double cb1 = numeric(column_b1, plan.source_index(5, i), op, "ColumnB1");
    const double rb2 = numeric(row_b2, plan.source_index(6, i), op, "RowB2");
    const double cb2 = numeric(column_b2, plan.source_index(7, i), op, "ColumnB2");
    const double dax = ca2 - ca1;
    const double day = ra2 - ra1;
    const double dbx = cb2 - cb1;
    const double dby = rb2 - rb1;
    require(std::hypot(dax, day) > 0.0 && std::hypot(dbx, dby) > 0.0, op, "Line",
            "line points must not coincide");
    out.append(std::atan2(day * dbx - dax * dby, dax * dbx + day * dby));
  }
  *angle = std::move(out);
}

}  // namespace mv::geometry
