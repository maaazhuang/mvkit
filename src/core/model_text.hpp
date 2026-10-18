#pragma once

#include <charconv>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <locale>
#include <sstream>
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace mv::core::model_io {
// 维持现行十进制、classic locale 和 17 位有效数字，不改变模型版本或字段顺序。
class TextOutput {
public:
  void reserve(std::size_t size) { bytes_.reserve(size); }
  TextOutput &operator<<(char value) {
    bytes_.push_back(value);
    return *this;
  }
  TextOutput &operator<<(std::string_view value) {
    bytes_.append(value);
    return *this;
  }
  template <std::integral T> TextOutput &operator<<(T value) {
    char buffer[32];
    if constexpr (std::is_same_v<T, bool>)
      return *this << static_cast<unsigned>(value);
    else {
      // 配置枚举和掩码多为单个十进制数字，直接追加同一字节。
      if (value >= T{0} && value <= T{9}) {
        bytes_.push_back(static_cast<char>('0' + value));
        return *this;
      }
      const auto converted =
          std::to_chars(buffer, buffer + sizeof(buffer), value);
      bytes_.append(buffer, converted.ptr);
      return *this;
    }
  }
  TextOutput &operator<<(double value) {
    char buffer[64];
    if(value==0.) {
      if(std::signbit(value))bytes_.push_back('-');
      bytes_.push_back('0');return *this;
    }
    if(value==1.) {bytes_.push_back('1');return *this;}
    // 53 位内的整数可精确转换，general/17 在此范围也输出完整十进制整数。
    // 单独保留负零；非整数、极值和特殊值仍使用原浮点转换器。
    if (value >= -9007199254740992. && value <= 9007199254740992. &&
        !(value == 0. && std::signbit(value))) {
      const auto integer = static_cast<std::int64_t>(value);
      if (static_cast<double>(integer) == value) return *this << integer;
    }
    const auto converted = std::to_chars(buffer, buffer + sizeof(buffer), value,
                                         std::chars_format::general, 17);
    bytes_.append(buffer, converted.ptr);
    return *this;
  }
  std::string str() const & { return bytes_; }
  std::string str() && { return std::move(bytes_); }

private:
  std::string bytes_;
};

// 只读视图沿用标准流的头部词法，正文直接引用调用期间有效的输入。
// 流只拥有游标，既不复制大段文本，也不修改原始字节。
class TextViewInput : public std::istream {
  class Buffer : public std::streambuf {
  public:
    explicit Buffer(std::string_view bytes) {
      if (!bytes.empty()) {
        auto* begin = const_cast<char*>(bytes.data());
        setg(begin, begin, begin + bytes.size());
      }
    }
    std::string_view remaining() const {
      return gptr() ? std::string_view(gptr(), static_cast<std::size_t>(egptr() - gptr()))
                    : std::string_view{};
    }
  } buffer_;
public:
  explicit TextViewInput(std::string_view bytes) : std::istream(nullptr), buffer_(bytes) {
    rdbuf(&buffer_);
    imbue(std::locale::classic());
  }
  std::string_view remaining() const { return buffer_.remaining(); }
};

inline std::string enclose_text(std::string_view magic, std::uint64_t checksum,
                                std::string_view body) {
  TextOutput output;
  output.reserve(magic.size() + 32 + body.size());
  output << magic << checksum << '\n' << body;
  return std::move(output).str();
}

inline std::string enclose_text(std::string_view magic,std::uint64_t checksum,std::string&& body) {
  char digits[32];
  const auto converted=std::to_chars(digits,digits+sizeof(digits),checksum);
  const auto count=static_cast<std::size_t>(converted.ptr-digits);
  // 直接在现有容量中前插头部，避免另建完整正文副本；格式及校验和字节保持一致。
  body.insert(0,magic.size()+count+1,'\0');
  if(!magic.empty())std::memcpy(body.data(),magic.data(),magic.size());
  std::memcpy(body.data()+magic.size(),digits,count);
  body[magic.size()+count]='\n';
  return std::move(body);
}

class TextInput {
public:
  explicit TextInput(std::string_view bytes) : bytes_(bytes) {}
  template <typename T> TextInput &operator>>(T &value) {
    skip_space();
    const auto original = bytes_;
    if (bytes_.empty()) {
      failed_ = true;
      return *this;
    }
    // from_chars 不接受前导 +；现行流协议允许，保留该词法行为。
    const bool plus = bytes_.front() == '+';
    if (plus)
      bytes_.remove_prefix(1);
    if (bytes_.empty() ||
        (plus && (bytes_.front() == '+' || bytes_.front() == '-'))) {
      failed_ = true;
      return *this;
    }
    if constexpr (std::is_same_v<T, double>) {
      // 模型灰度常为小整数；完整整数字段无需浮点十进制解析。
      // 仅接收空白/结尾分隔且不超过 2^53 的值，其他词法交回原解析器。
      const bool negative = bytes_.front() == '-';
      std::size_t cursor = negative ? 1 : 0;
      const auto first = cursor;
      std::uint64_t integer = 0;
      while (cursor < bytes_.size() && cursor - first < 16 &&
             bytes_[cursor] >= '0' && bytes_[cursor] <= '9') {
        integer = integer * 10 + static_cast<unsigned>(bytes_[cursor++] - '0');
      }
      if (cursor > first && integer <= 9007199254740992ULL &&
          (cursor == bytes_.size() || is_space(bytes_[cursor]))) {
        value = negative ? -static_cast<double>(integer) : static_cast<double>(integer);
        bytes_.remove_prefix(cursor);
        return *this;
      }
    }
    const auto converted =
        std::from_chars(bytes_.data(), bytes_.data() + bytes_.size(), value);
    bool fallback = converted.ec != std::errc{};
    if constexpr (std::is_floating_point_v<T>) {
      // 不完整指数、浮点下溢和旧流特殊词法使用原解析器；正常字段不创建流或副本。
      fallback = fallback || (converted.ptr != bytes_.data() + bytes_.size() &&
                              (*converted.ptr == 'e' || *converted.ptr == 'E' || *converted.ptr == 'x' || *converted.ptr == 'X'));
      fallback = fallback || bytes_.front() == 'i' || bytes_.front() == 'I' ||
                 bytes_.front() == 'n' || bytes_.front() == 'N';
    }
    if (fallback) {
      std::istringstream legacy{std::string(original)};
      legacy.imbue(std::locale::classic());
      if (!(legacy >> value)) {
        if (legacy.eof())
          bytes_ = {};
        failed_ = true;
        return *this;
      }
      const auto offset = legacy.tellg();
      bytes_ = offset < 0 ? std::string_view{}
                          : original.substr(static_cast<std::size_t>(offset));
    } else
      bytes_.remove_prefix(
          static_cast<std::size_t>(converted.ptr - bytes_.data()));
    return *this;
  }
  // 编解码器只使用 std::ws；扫描与 classic locale 的六种 ASCII 空白一致。
  TextInput &operator>>(std::istream &(*)(std::istream &)) {
    skip_space();
    return *this;
  }
  explicit operator bool() const noexcept { return !failed_; }
  bool eof() const noexcept { return bytes_.empty(); }

private:
  static bool is_space(char c) noexcept {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '\v' || c == '\f';
  }
  void skip_space() {
    while (!bytes_.empty()) {
      const char c = bytes_.front();
      if (!is_space(c))
        break;
      bytes_.remove_prefix(1);
    }
  }
  std::string_view bytes_;
  bool failed_ = false;
};
} // namespace mv::core::model_io
