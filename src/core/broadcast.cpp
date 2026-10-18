#include <core/broadcast.hpp>

#include <algorithm>
#include <string>

#include <core/exception.hpp>

namespace mv::core {

BroadcastPlan BroadcastPlan::make(
    std::initializer_list<BroadcastInput> inputs,
    std::string_view operation) {
  if (inputs.size() == 0) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                    operation, "", -1, "broadcast requires at least one input");
  }

  BroadcastPlan result;
  result.input_count_=inputs.size();
  if(inputs.size()>result.inline_sizes_.size())result.input_sizes_.resize(inputs.size());
  std::size_t output_size = 1;
  std::int64_t argument_index = 0;
  for (const auto& input : inputs) {
    if (input.size == 0) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, input.parameter, argument_index,
                      "broadcast input must not be empty");
    }
    if(inputs.size()<=result.inline_sizes_.size())result.inline_sizes_[argument_index]=input.size;
    else result.input_sizes_[argument_index]=input.size;
    output_size = std::max(output_size, input.size);
    ++argument_index;
  }

  argument_index = 0;
  for (const auto& input : inputs) {
    if (input.size != output_size && !(input.allow_scalar && input.size == 1)) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality,
                      operation, input.parameter, argument_index,
                      "tuple length must be 1 or equal to the common broadcast length");
    }
    ++argument_index;
  }
  result.output_size_=output_size;return result;
}

[[noreturn]] void BroadcastPlan::throw_source_index(bool argument) {
  throw Exception(Status::OutOfRange, "BroadcastPlan::source_index",
                  argument ? "broadcast argument index out of range" : "broadcast output index out of range");
}

}  // namespace mv::core
