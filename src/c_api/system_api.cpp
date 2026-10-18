#include <mvkit/c/mv_system.h>

#include <core/exception.hpp>
#include <core/runtime_context.hpp>
#include <memory>
#include <string_view>
#include <string>

#include "control_operator_outputs.hpp"
#include "error_state.hpp"
#include "opaque.hpp"
#include "tuple_input.hpp"

extern "C" ::MvStatus mv_set_system(::MvTuple SystemParameter, ::MvTuple Value) {
  return mv::c_api::translate([&] {
    mv::c_api::TupleInput names(SystemParameter, "set_system", "SystemParameter");
    mv::c_api::TupleInput values(Value, "set_system", "Value");
    if (!names.size())
      names.fail(mv::core::ErrorCode::InvalidCardinality, -1, "system parameter tuple must not be empty");
    if (names.size() != values.size())
      values.fail(mv::core::ErrorCode::InvalidCardinality, -1, "one value per system parameter required");
    for (std::size_t i = 0; i < names.size(); ++i) (void)names.string(i);
    // HALCON 按顺序应用参数，后项非法不撤销先前更新；先检查名称类型及元组长度。
    for (std::size_t i = 0; i < names.size(); ++i) {
      auto name = names.string(i);
      auto value = values.at(i);
      if (name == "thread_num") {
        if (value.kind() == mv::core::ValueKind::String && value.as_string().view() == "default")
          value = mv::core::Value{std::int64_t(0)};
        else if (value.kind() == mv::core::ValueKind::Int64 && value.as_int64() <= 0)
          values.fail(mv::core::ErrorCode::InvalidValue, i, "thread_num must be positive or default");
      }
      mv::core::set_system(name, value);
    }
  });
}

extern "C" ::MvStatus mv_get_system(::MvTuple Query, ::MvTuple* Information) {
  return mv::c_api::translate([&] {
    mv::c_api::ControlOutputs<1> output("get_system", {Information});
    mv::c_api::TupleInput queries(Query, "get_system", "Query");
    for (std::size_t i = 0; i < queries.size(); ++i) (void)queries.string(i);
    for (std::size_t i = 0; i < queries.size(); ++i) {
      auto value = queries.string(i) == "version"
          ? mv::core::Tuple(std::to_string(MVKIT_VERSION_MAJOR) + "." + std::to_string(MVKIT_VERSION_MINOR) + "." + std::to_string(MVKIT_VERSION_PATCH))
          : mv::core::get_system(queries.string(i));
      for (std::size_t j = 0; j < value.size(); ++j) output[0].append(value.at(j));
    }
    output.Commit();
  });
}
