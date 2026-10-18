#include <core/exception.hpp>

#include <string>

namespace mv::core {
namespace {
std::string make_what(std::string_view where, std::string_view message) {
  if (where.empty()) {
    return std::string(message);
  }
  std::string result;
  result.reserve(where.size() + message.size() + 2);
  result.append(where);
  result.append(": ");
  result.append(message);
  return result;
}
}  // namespace

Exception::Exception(Status status, std::string_view where, std::string_view message)
    : std::runtime_error(make_what(where, message)),
      status_(status),
      error_code_(default_error_code(status)),
      where_(where),
      message_(message) {}

Exception::Exception(Status status, ErrorCode code, std::string_view where,
                     std::string_view parameter, std::int64_t parameter_index,
                     std::string_view message)
    : std::runtime_error(make_what(where, message)),
      status_(status),
      error_code_(code),
      where_(where),
      parameter_(parameter),
      parameter_index_(parameter_index),
      message_(message) {}

}  // namespace mv::core
