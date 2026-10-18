#include <calibration/targets/calplate_descr.hpp>

#include <cctype>
#include <charconv>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <core/exception.hpp>

namespace mv::calibration {
namespace {

[[noreturn]] void fail_parse(std::string_view message) {
  throw core::Exception(core::Status::InvalidArgument, "parse_calplate_cpd", message);
}

[[noreturn]] void fail_resolve(std::string_view message) {
  throw core::Exception(core::Status::NotFound, "resolve_calib_descr", message);
}

std::string_view trim(std::string_view text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) text.remove_prefix(1);
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.remove_suffix(1);
  return text;
}

std::string_view strip_comment(std::string_view text) {
  const auto hash = text.find('#');
  if (hash != std::string_view::npos) text = text.substr(0, hash);
  return trim(text);
}

std::vector<std::string> split_tokens(std::string_view text) {
  std::vector<std::string> tokens;
  std::istringstream in{std::string(text)};
  for (std::string token; in >> token;) tokens.push_back(std::move(token));
  return tokens;
}

bool parse_i64(std::string_view text, std::int64_t& out) {
  if (text.empty()) return false;
  const char* begin = text.data();
  const char* end = begin + text.size();
  std::int64_t value = 0;
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc{} || result.ptr != end) return false;
  out = value;
  return true;
}

bool parse_f64(std::string_view text, double& out) {
  if (text.empty()) return false;
  const char* begin = text.data();
  const char* end = begin + text.size();
  double value = 0.0;
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc{} || result.ptr != end) return false;
  out = value;
  return true;
}

std::string line_prefix(std::size_t line_no) { return "line " + std::to_string(line_no) + ": "; }

bool regular_file_exists(const std::filesystem::path& path) {
  std::error_code ec;
  return !path.empty() && std::filesystem::is_regular_file(path, ec);
}

}  // namespace

std::filesystem::path resolve_calib_descr(std::string_view descr) {
  const std::filesystem::path given{std::string(descr)};
  if (regular_file_exists(given)) return given;

  fail_resolve("calibration description not found: " + std::string(descr));
}

CalplateDescription parse_calplate_cpd(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw core::Exception(core::Status::NotFound, "parse_calplate_cpd",
                          "cannot open calibration plate description: " + path.string());
  }

  CalplateDescription desc;
  std::int64_t finders_left = 0;
  std::string raw;
  std::size_t line_no = 0;
  while (std::getline(in, raw)) {
    ++line_no;
    const auto body = strip_comment(raw);
    if (body.empty()) continue;
    const auto tokens = split_tokens(body);
    if (tokens.empty()) continue;

    if (finders_left > 0) {
      if (tokens.size() < 2) fail_parse(line_prefix(line_no) + "expected finder column row");
      CalplateFinder finder;
      if (!parse_i64(tokens[0], finder.column) || !parse_i64(tokens[1], finder.row)) {
        fail_parse(line_prefix(line_no) + "invalid finder column row");
      }
      desc.finders.push_back(finder);
      --finders_left;
      continue;
    }

    if (tokens[0].size() == 1 && std::isalpha(static_cast<unsigned char>(tokens[0][0]))) {
      const char cmd = tokens[0][0];
      switch (cmd) {
        case 'r':
          if (tokens.size() < 2 || !parse_i64(tokens[1], desc.rows)) {
            fail_parse(line_prefix(line_no) + "invalid r command");
          }
          break;
        case 'c':
          if (tokens.size() < 2 || !parse_i64(tokens[1], desc.columns)) {
            fail_parse(line_prefix(line_no) + "invalid c command");
          }
          break;
        case 'z':
          if (tokens.size() < 2 || !parse_f64(tokens[1], desc.z)) {
            fail_parse(line_prefix(line_no) + "invalid z command");
          }
          break;
        case 'p':
          if (tokens.size() < 2) fail_parse(line_prefix(line_no) + "invalid p command");
          if (tokens[1] == "light") {
            desc.light_on_dark = true;
          } else if (tokens[1] == "dark") {
            desc.light_on_dark = false;
          } else {
            fail_parse(line_prefix(line_no) + "p expects light or dark");
          }
          break;
        case 'f': {
          std::int64_t count = 0;
          if (tokens.size() < 2 || !parse_i64(tokens[1], count) || count < 0) {
            fail_parse(line_prefix(line_no) + "invalid f command");
          }
          finders_left = count;
          break;
        }
        case 'o':
          break;
        default:
          break;
      }
      continue;
    }

    if (tokens.size() < 3) fail_parse(line_prefix(line_no) + "expected x y radius");
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    if (!parse_f64(tokens[0], x) || !parse_f64(tokens[1], y) || !parse_f64(tokens[2], radius)) {
      fail_parse(line_prefix(line_no) + "invalid x y radius");
    }
    if (!(radius > 0.0)) fail_parse(line_prefix(line_no) + "radius must be > 0");
    desc.points.push_back(Point3d{x, y, desc.z});
    desc.radii.push_back(radius);
  }

  if (finders_left > 0) fail_parse("missing finder rows");
  if (desc.points.size() != desc.radii.size()) fail_parse("points and radii size mismatch");
  if (desc.points.empty()) fail_parse("expected at least one mark");
  if (desc.finders.empty()) fail_parse("expected at least one finder");
  if (desc.rows < 1 || desc.columns < 1) fail_parse("rows and columns must be >= 1");
  return desc;
}

}  // namespace mv::calibration
