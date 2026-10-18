#include <atomic>
#include <core/exception.hpp>
#include <core/runtime_context.hpp>
#include <cstdint>
#include <limits>
#include <runtime/config/runtime_config.hpp>
#include <runtime/parallel/parallel_for.hpp>

namespace mv::core {
namespace {

std::atomic<int> default_neighborhood{8};
std::atomic<std::size_t> global_max_connection{0};
int& thread_neighborhood() noexcept {
  thread_local int value = default_neighborhood.load(std::memory_order_acquire);
  return value;
}

std::int64_t require_single_integer(const Tuple& value, std::string_view operation) {
  if (value.size() != 1) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidCardinality, operation, "Value", -1,
                    "system parameter value must contain exactly one element");
  }
  if (value.kind() != TupleKind::Int64) {
    throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation, "Value", 0,
                    "system parameter value must be an integer");
  }
  return value.as_int64();
}

std::int64_t require_single_integer(const Value& value, std::string_view operation) {
  if (value.kind() != ValueKind::Int64)
    throw Exception(Status::TypeMismatch, ErrorCode::InvalidType, operation, "Value", 0,
                    "system parameter value must be an integer");
  return value.as_int64();
}

template <class Input>
int require_neighborhood(const Input& value) {
  const auto parsed = require_single_integer(value, "set_system");
  if (parsed != 4 && parsed != 8) {
    throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, "set_system", "Value", 0,
                    "neighborhood must be 4 or 8");
  }
  return static_cast<int>(parsed);
}

}  // namespace

RuntimeContext current_runtime_context() noexcept {
  return {static_cast<Neighborhood>(thread_neighborhood()),
          global_max_connection.load(std::memory_order_acquire)};
}

void initialize_runtime_context() noexcept { (void)thread_neighborhood(); }

template <class Input>
void set_system_impl(std::string_view system_parameter, const Input& value) {
  initialize_runtime_context();
  if (system_parameter == "thread_num") {
    const auto parsed = require_single_integer(value, "set_system");
    if (parsed < 0 || static_cast<std::uint64_t>(parsed) > (std::numeric_limits<std::size_t>::max)())
      throw Exception(Status::InvalidArgument, "set_system", "thread_num must fit a nonnegative size_t");
    if (runtime::runtime_backend_config().parallel == runtime::ParallelBackend::Serial && parsed > 1)
      throw Exception(Status::Unsupported, "set_system", "serial backend accepts thread_num 0 or 1 only");
    try {
      runtime::set_parallelism(static_cast<std::size_t>(parsed));
    } catch (const std::logic_error& error) {
      throw Exception(Status::InvalidState, "set_system", error.what());
    }
    return;
  }
  if (system_parameter == "neighborhood") {
    const int parsed = require_neighborhood(value);
    default_neighborhood.store(parsed, std::memory_order_release);
    thread_neighborhood() = parsed;
    return;
  }
  if (system_parameter == "tsp_neighborhood") {
    thread_neighborhood() = require_neighborhood(value);
    return;
  }
  if (system_parameter == "max_connection") {
    const auto parsed = require_single_integer(value, "set_system");
    if (parsed < 0) {
      throw Exception(Status::InvalidArgument, ErrorCode::InvalidValue, "set_system", "Value", 0,
                      "max_connection must be non-negative");
    }
    global_max_connection.store(static_cast<std::size_t>(parsed), std::memory_order_release);
    return;
  }
  throw Exception(Status::Unsupported, ErrorCode::UnsupportedValue, "set_system", "SystemParameter", -1,
                  "unsupported system parameter");
}

void set_system(std::string_view system_parameter, const Tuple& value) {
  set_system_impl(system_parameter, value);
}
void set_system(std::string_view system_parameter, const Value& value) {
  set_system_impl(system_parameter, value);
}

Tuple get_system(std::string_view query) {
  initialize_runtime_context();
  if (query == "thread_num") return Tuple{static_cast<std::int64_t>(runtime::configured_parallelism())};
  if (query == "thread_pool_size")
    return Tuple{static_cast<std::int64_t>(runtime::started_parallel_worker_count())};
  if (query == "parallel_backend") return Tuple{runtime::parallel_backend_name()};
  if (query == "cpu_logical_count")
    return Tuple{static_cast<std::int64_t>(runtime::cpu_topology().logical_count)};
  if (query == "cpu_available_count")
    return Tuple{static_cast<std::int64_t>(runtime::cpu_topology().available_logical_count)};
  if (query == "cpu_physical_count")
    return Tuple{static_cast<std::int64_t>(runtime::cpu_topology().physical_count)};
  if (query == "cpu_performance_core_count")
    return Tuple{static_cast<std::int64_t>(runtime::cpu_topology().high_performance_cores)};
  if (query == "cpu_efficiency_core_count")
    return Tuple{static_cast<std::int64_t>(runtime::cpu_topology().efficiency_cores)};
  if (query == "neighborhood") {
    return Tuple{static_cast<std::int64_t>(default_neighborhood.load(std::memory_order_acquire))};
  }
  if (query == "tsp_neighborhood") {
    return Tuple{static_cast<std::int64_t>(thread_neighborhood())};
  }
  if (query == "max_connection") {
    const auto value = global_max_connection.load(std::memory_order_acquire);
    if (value > static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)())) {
      throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange, "get_system", "Query", -1,
                      "max_connection cannot be represented as int64");
    }
    return Tuple{static_cast<std::int64_t>(value)};
  }
  throw Exception(Status::Unsupported, ErrorCode::UnsupportedValue, "get_system", "Query", -1,
                  "unsupported system query");
}

}  // namespace mv::core
