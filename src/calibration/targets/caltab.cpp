#include <calibration/targets/caltab.hpp>
#include <calibration/targets/find_calib_object.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <core/exception.hpp>
#include <core/image.hpp>
#include <core/xld.hpp>

namespace mv::calibration {
namespace {

[[noreturn]] void fail_parse(std::string_view message) {
  throw core::Exception(core::Status::InvalidArgument, "parse_caltab_descr", message);
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
  if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(value)) return false;
  out = value;
  return true;
}

std::string line_prefix(std::size_t line_no) { return "line " + std::to_string(line_no) + ": "; }

std::array<double, 4> parse_quad(const std::vector<std::string>& tokens,
                                 std::size_t line_no, std::string_view command) {
  if (tokens.size() < 5) fail_parse(line_prefix(line_no) + "invalid " + std::string(command) + " command");
  std::array<double, 4> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!parse_f64(tokens[i + 1], values[i])) {
      fail_parse(line_prefix(line_no) + "invalid " + std::string(command) + " command");
    }
  }
  return values;
}

double corner_patch_mean(const core::Image& image, const core::Point2d& grid_center,
                         const core::Point2d& corner, double radius) {
  const double row = corner.row + 0.18 * (corner.row - grid_center.row);
  const double col = corner.col + 0.18 * (corner.col - grid_center.col);
  const int r0 = std::max(0, static_cast<int>(std::floor(row - radius)));
  const int r1 = std::min(static_cast<int>(image.height()) - 1,
                          static_cast<int>(std::ceil(row + radius)));
  const int c0 = std::max(0, static_cast<int>(std::floor(col - radius)));
  const int c1 = std::min(static_cast<int>(image.width()) - 1,
                          static_cast<int>(std::ceil(col + radius)));
  const auto* pixels = static_cast<const std::uint8_t*>(image.data());
  double sum = 0.0;
  std::size_t count = 0;
  const double radius2 = radius * radius;
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      const double dr = static_cast<double>(r) - row;
      const double dc = static_cast<double>(c) - col;
      if (dr * dr + dc * dc > radius2) continue;
      sum += pixels[static_cast<std::size_t>(r) * image.stride() + static_cast<std::size_t>(c)];
      ++count;
    }
  }
  return count == 0 ? 255.0 : sum / static_cast<double>(count);
}

std::size_t nearest_description_corner(const CaltabDescription& description) {
  if (!description.triangle) return 0;
  const double tx = 0.5 * ((*description.triangle)[0] + (*description.triangle)[2]);
  const double ty = 0.5 * ((*description.triangle)[1] + (*description.triangle)[3]);
  const std::array<std::size_t, 4> corners{
      0,
      static_cast<std::size_t>(description.columns - 1),
      static_cast<std::size_t>((description.rows - 1) * description.columns),
      description.points.size() - 1,
  };
  return *std::min_element(corners.begin(), corners.end(), [&](std::size_t a, std::size_t b) {
    const auto distance2 = [&](std::size_t i) {
      const double dx = description.points[i].x - tx;
      const double dy = description.points[i].y - ty;
      return dx * dx + dy * dy;
    };
    return distance2(a) < distance2(b);
  });
}

}  // namespace

CaltabDescription parse_caltab_descr(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw core::Exception(core::Status::NotFound, "parse_caltab_descr",
                          "cannot open calibration plate description: " + path.string());
  }

  CaltabDescription desc;
  std::string raw;
  std::size_t line_no = 0;
  while (std::getline(in, raw)) {
    ++line_no;
    const auto body = strip_comment(raw);
    if (body.empty()) continue;
    const auto tokens = split_tokens(body);
    if (tokens.empty()) continue;

    if (tokens[0].size() == 1 && std::isalpha(static_cast<unsigned char>(tokens[0][0]))) {
      switch (tokens[0][0]) {
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
        case 'o':
          desc.rim = parse_quad(tokens, line_no, "o");
          break;
        case 'i':
          desc.frame = parse_quad(tokens, line_no, "i");
          break;
        case 't':
          desc.triangle = parse_quad(tokens, line_no, "t");
          break;
        case 'w': {
          double width = 0.0;
          if (tokens.size() < 2 || !parse_f64(tokens[1], width) || !(width > 0.0)) {
            fail_parse(line_prefix(line_no) + "invalid w command");
          }
          desc.frame_width = width;
          break;
        }
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
        default:
          break;
      }
      continue;
    }

    if (tokens.size() < 3) fail_parse(line_prefix(line_no) + "expected x y radius");
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    if (!parse_f64(tokens[0], x) || !parse_f64(tokens[1], y) ||
        !parse_f64(tokens[2], radius)) {
      fail_parse(line_prefix(line_no) + "invalid x y radius");
    }
    if (!(radius > 0.0)) fail_parse(line_prefix(line_no) + "radius must be > 0");
    desc.points.push_back(Point3d{x, y, 0.0});
    desc.radii.push_back(radius);
  }

  if (desc.rows < 1 || desc.columns < 1) fail_parse("rows and columns must be >= 1");
  if (desc.rows > std::numeric_limits<std::int64_t>::max() / desc.columns) {
    fail_parse("mark count overflow");
  }
  const auto expected = static_cast<std::size_t>(desc.rows * desc.columns);
  if (desc.points.size() != expected || desc.radii.size() != expected) {
    fail_parse("mark count does not match rows * columns");
  }
  for (auto& point : desc.points) point.z = desc.z;
  return desc;
}

std::vector<core::Point2d> detect_caltab(const core::Image& image,
                                         const CaltabDescription& description) {
  if (description.rows < 2 || description.columns < 2 ||
      description.points.size() !=
          static_cast<std::size_t>(description.rows * description.columns)) {
    throw core::Exception(core::Status::InvalidArgument, "detect_caltab",
                          "invalid caltab description");
  }
  auto image_order = detect_circle_grid_points(image, description.rows, description.columns);
  const auto rows = static_cast<std::size_t>(description.rows);
  const auto columns = static_cast<std::size_t>(description.columns);

  core::Point2d center{};
  for (const auto& point : image_order) {
    center.row += point.row;
    center.col += point.col;
  }
  center.row /= static_cast<double>(image_order.size());
  center.col /= static_cast<double>(image_order.size());

  double spacing_sum = 0.0;
  std::size_t spacing_count = 0;
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 1; c < columns; ++c) {
      const auto& a = image_order[r * columns + c - 1];
      const auto& b = image_order[r * columns + c];
      spacing_sum += std::hypot(a.row - b.row, a.col - b.col);
      ++spacing_count;
    }
  }
  for (std::size_t r = 1; r < rows; ++r) {
    for (std::size_t c = 0; c < columns; ++c) {
      const auto& a = image_order[(r - 1) * columns + c];
      const auto& b = image_order[r * columns + c];
      spacing_sum += std::hypot(a.row - b.row, a.col - b.col);
      ++spacing_count;
    }
  }
  const double spacing = spacing_sum / static_cast<double>(spacing_count);
  const std::array<std::size_t, 4> image_corners{
      0, columns - 1, (rows - 1) * columns, image_order.size() - 1};
  const auto image_triangle = *std::min_element(
      image_corners.begin(), image_corners.end(), [&](std::size_t a, std::size_t b) {
        return corner_patch_mean(image, center, image_order[a], 0.22 * spacing) <
               corner_patch_mean(image, center, image_order[b], 0.22 * spacing);
      });

  const std::size_t description_triangle = nearest_description_corner(description);
  const std::size_t image_corner_row = image_triangle / columns;
  const std::size_t image_corner_col = image_triangle % columns;
  const std::size_t description_corner_row = description_triangle / columns;
  const std::size_t description_corner_col = description_triangle % columns;

  std::vector<core::Point2d> result(image_order.size());
  for (std::size_t r = 0; r < rows; ++r) {
    const auto row_offset = r > description_corner_row ? r - description_corner_row
                                                        : description_corner_row - r;
    const std::size_t image_row = image_corner_row == 0 ? row_offset : rows - 1 - row_offset;
    for (std::size_t c = 0; c < columns; ++c) {
      const auto col_offset = c > description_corner_col ? c - description_corner_col
                                                          : description_corner_col - c;
      const std::size_t image_col = image_corner_col == 0 ? col_offset : columns - 1 - col_offset;
      result[r * columns + c] = image_order[image_row * columns + image_col];
    }
  }
  return result;
}

}  // namespace mv::calibration
