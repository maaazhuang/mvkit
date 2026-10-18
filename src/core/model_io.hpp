#pragma once

#include <core/exception.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

namespace mv::core::model_io {
// UTF-8 路径和有界读取；文件与内存入口共享同一模型编解码器。
std::string read_file(std::string_view path, std::size_t limit, const char* operation);

inline void write_file(std::string_view path, std::string_view bytes, const char* operation) {
  std::ofstream output(std::filesystem::u8path(path), std::ios::binary | std::ios::trunc);
  if (!output) throw Exception(Status::IoError, operation, "cannot open model for writing");
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  output.flush();
  if (!output) throw Exception(Status::IoError, operation, "cannot write model file");
}

// 校验和用于发现损坏，不提供认证或加密。
inline std::uint64_t checksum(std::string_view bytes) noexcept {
  std::uint64_t result = 14695981039346656037ULL;
  for (unsigned char value : bytes) { result ^= value; result *= 1099511628211ULL; }
  return result;
}
}  // namespace mv::core::model_io
