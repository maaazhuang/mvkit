#include <core/model_io.hpp>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <algorithm>
#include <limits>
#endif

namespace mv::core::model_io {
std::string read_file(std::string_view path, std::size_t limit, const char* operation) {
#if defined(_WIN32)
  // 每次重新打开并读取完整文件，不用时间戳或路径缓存替代实际内容检查。
  const auto filename = std::filesystem::u8path(path);
  const auto handle = CreateFileW(filename.c_str(), GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE)
    throw Exception(Status::IoError, operation, "cannot open model for reading");
  struct FileGuard {
    HANDLE handle;
    ~FileGuard() { CloseHandle(handle); }
  } guard{handle};
  LARGE_INTEGER size;
  if (!GetFileSizeEx(handle, &size) || size.QuadPart < 0 ||
      static_cast<std::uint64_t>(size.QuadPart) > limit)
    throw Exception(Status::LimitExceeded, operation, "model file exceeds size limit");
  std::string bytes(static_cast<std::size_t>(size.QuadPart), '\0');
  for (std::size_t offset = 0; offset < bytes.size();) {
    const auto count = static_cast<DWORD>(std::min<std::size_t>(
        bytes.size() - offset, std::numeric_limits<DWORD>::max()));
    DWORD read = 0;
    if (!ReadFile(handle, bytes.data() + offset, count, &read, nullptr) || !read)
      throw Exception(Status::IoError, operation, "cannot read model file");
    offset += read;
  }
  return bytes;
#else
  std::ifstream input(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
  if (!input) throw Exception(Status::IoError, operation, "cannot open model for reading");
  const auto size = input.tellg();
  if (size < 0 || size > static_cast<std::streamoff>(limit))
    throw Exception(Status::LimitExceeded, operation, "model file exceeds size limit");
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.seekg(0);
  if (!bytes.empty()) input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!input) throw Exception(Status::IoError, operation, "cannot read model file");
  return bytes;
#endif
}
} // namespace mv::core::model_io
