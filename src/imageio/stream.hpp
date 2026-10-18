#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <vector>

namespace mv::imageio {

class InputStream {
 public:
  virtual ~InputStream() = default;
  [[nodiscard]] virtual bool good() const noexcept = 0;
  virtual std::size_t read(void* dst, std::size_t bytes) = 0;
  virtual bool seek(std::uint64_t offset) = 0;
  [[nodiscard]] virtual std::uint64_t tell() const = 0;
  [[nodiscard]] virtual std::uint64_t size() const = 0;
};

class OutputStream {
 public:
  virtual ~OutputStream() = default;
  [[nodiscard]] virtual bool good() const noexcept = 0;
  virtual std::size_t write(const void* src, std::size_t bytes) = 0;
  virtual bool seek(std::uint64_t offset) = 0;
  [[nodiscard]] virtual std::uint64_t tell() const = 0;
  [[nodiscard]] virtual std::uint64_t size() const = 0;
};

class MemoryInputStream final : public InputStream {
 public:
  explicit MemoryInputStream(std::span<const std::byte> data) noexcept : data_(data) {}

  [[nodiscard]] bool good() const noexcept override { return true; }
  std::size_t read(void* dst, std::size_t bytes) override;
  bool seek(std::uint64_t offset) override;
  [[nodiscard]] std::uint64_t tell() const override { return position_; }
  [[nodiscard]] std::uint64_t size() const override { return data_.size(); }

 private:
  std::span<const std::byte> data_;
  std::size_t position_{0};
};

class MemoryOutputStream final : public OutputStream {
 public:
  [[nodiscard]] bool good() const noexcept override { return true; }
  std::size_t write(const void* src, std::size_t bytes) override;
  bool seek(std::uint64_t offset) override;
  [[nodiscard]] std::uint64_t tell() const override { return position_; }
  [[nodiscard]] std::uint64_t size() const override { return data_.size(); }
  [[nodiscard]] std::span<const std::byte> data() const noexcept { return data_; }
  [[nodiscard]] std::vector<std::byte> release() noexcept { return std::move(data_); }

 private:
  std::vector<std::byte> data_;
  std::size_t position_{0};
};

class FileInputStream final : public InputStream {
 public:
  explicit FileInputStream(const std::filesystem::path& path);

  [[nodiscard]] bool good() const noexcept override { return stream_.is_open() && !stream_.bad(); }
  std::size_t read(void* dst, std::size_t bytes) override;
  bool seek(std::uint64_t offset) override;
  [[nodiscard]] std::uint64_t tell() const override;
  [[nodiscard]] std::uint64_t size() const override { return size_; }

 private:
  mutable std::ifstream stream_;
  std::uint64_t size_{0};
};

class FileOutputStream final : public OutputStream {
 public:
  explicit FileOutputStream(const std::filesystem::path& path);

  [[nodiscard]] bool good() const noexcept override { return stream_.is_open() && !stream_.bad(); }
  std::size_t write(const void* src, std::size_t bytes) override;
  bool seek(std::uint64_t offset) override;
  [[nodiscard]] std::uint64_t tell() const override;
  [[nodiscard]] std::uint64_t size() const override { return size_; }

 private:
  mutable std::ofstream stream_;
  std::uint64_t size_{0};
};

}  // namespace mv::imageio
