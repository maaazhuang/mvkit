#include <imageio/stream.hpp>

#include <algorithm>
#include <cstring>
#include <limits>

namespace mv::imageio {

std::size_t MemoryInputStream::read(void* dst, std::size_t bytes) {
  if (bytes == 0) return 0;
  if (dst == nullptr) return 0;
  const std::size_t remaining = data_.size() - position_;
  const std::size_t count = std::min(bytes, remaining);
  if (count != 0) {
    std::memcpy(dst, data_.data() + position_, count);
    position_ += count;
  }
  return count;
}

bool MemoryInputStream::seek(std::uint64_t offset) {
  if (offset > data_.size()) return false;
  position_ = static_cast<std::size_t>(offset);
  return true;
}

std::size_t MemoryOutputStream::write(const void* src, std::size_t bytes) {
  if (bytes == 0) return 0;
  if (src == nullptr) return 0;
  if (position_ > std::numeric_limits<std::size_t>::max() - bytes) return 0;
  const std::size_t end = position_ + bytes;
  if (end > data_.size()) data_.resize(end);
  std::memcpy(data_.data() + position_, src, bytes);
  position_ = end;
  return bytes;
}

bool MemoryOutputStream::seek(std::uint64_t offset) {
  if (offset > std::numeric_limits<std::size_t>::max()) return false;
  position_ = static_cast<std::size_t>(offset);
  return true;
}

FileInputStream::FileInputStream(const std::filesystem::path& path)
    : stream_(path, std::ios::binary) {
  if (!stream_.is_open()) return;
  stream_.seekg(0, std::ios::end);
  const auto end = stream_.tellg();
  if (end < 0) {
    stream_.setstate(std::ios::badbit);
    return;
  }
  size_ = static_cast<std::uint64_t>(end);
  stream_.seekg(0, std::ios::beg);
}

std::size_t FileInputStream::read(void* dst, std::size_t bytes) {
  if (!good() || dst == nullptr || bytes == 0) return 0;
  const auto max_stream = static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max());
  const auto request = std::min(bytes, max_stream);
  stream_.read(static_cast<char*>(dst), static_cast<std::streamsize>(request));
  const auto count = stream_.gcount();
  return count > 0 ? static_cast<std::size_t>(count) : 0;
}

bool FileInputStream::seek(std::uint64_t offset) {
  if (!stream_.is_open() || offset > size_) return false;
  stream_.clear();
  stream_.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
  return !stream_.fail();
}

std::uint64_t FileInputStream::tell() const {
  if (!stream_.is_open()) return 0;
  const auto position = stream_.tellg();
  return position < 0 ? 0 : static_cast<std::uint64_t>(position);
}

FileOutputStream::FileOutputStream(const std::filesystem::path& path)
    : stream_(path, std::ios::binary | std::ios::trunc) {}

std::size_t FileOutputStream::write(const void* src, std::size_t bytes) {
  if (!good() || src == nullptr || bytes == 0) return 0;
  const auto max_stream = static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max());
  const auto request = std::min(bytes, max_stream);
  stream_.write(static_cast<const char*>(src), static_cast<std::streamsize>(request));
  if (stream_.fail()) return 0;
  const auto position = stream_.tellp();
  if (position >= 0) size_ = std::max(size_, static_cast<std::uint64_t>(position));
  return request;
}

bool FileOutputStream::seek(std::uint64_t offset) {
  if (!stream_.is_open()) return false;
  stream_.clear();
  stream_.seekp(static_cast<std::streamoff>(offset), std::ios::beg);
  return !stream_.fail();
}

std::uint64_t FileOutputStream::tell() const {
  if (!stream_.is_open()) return 0;
  const auto position = stream_.tellp();
  return position < 0 ? 0 : static_cast<std::uint64_t>(position);
}

}  // namespace mv::imageio
