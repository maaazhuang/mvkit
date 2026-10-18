#include <array>
#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <calibration/parameters.hpp>
#include <calibration/persist.hpp>
#include <cctype>
#include <charconv>
#include <cfenv>
#include <clocale>
#include <cmath>
#include <core/exception.hpp>
#include <core/model_io.hpp>
#include <core/model_text.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace mv::calibration {
namespace {

[[noreturn]] void fail(core::Status status, const char* op, const char* message) {
  throw core::Exception(status, op, message);
}

int64_t integer(std::string_view text, const char* op) {
  std::string_view token(text);
  if(!token.empty() && token.front()=='+')token.remove_prefix(1);
  std::int64_t direct;
  const auto converted=std::from_chars(token.data(),token.data()+token.size(),direct);
  if(!token.empty() && !(text.front()=='+' && token.front()=='-') &&
      converted.ec==std::errc{} && converted.ptr==token.data()+token.size())return direct;
  try {
    size_t used = 0;
    auto n = std::stoll(std::string(text), &used);
    if (used != text.size()) fail(core::Status::DecodeError, op, "invalid integer");
    return n;
  } catch (const core::Exception&) {
    throw;
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid integer");
  }
}
bool decimal_input() {
  const auto* numeric_locale=std::setlocale(LC_NUMERIC,nullptr);
  return numeric_locale && std::strcmp(numeric_locale,"C")==0 && std::fegetround()==FE_TONEAREST;
}
double number(std::string_view text, const char* op, bool finite = true, bool decimal = false) {
  // 普通 C locale 的完整十进制字段直接转换；其他 locale、舍入模式、
  // 十六进制、特殊值及下溢仍走原 stod，以保留旧词法和失败语义。
  if(decimal) {
    std::string_view token(text);
    if(!token.empty() && token.front()=='+')token.remove_prefix(1);
    if(!token.empty() && !(text.front()=='+' && token.front()=='-')) {
      double direct;
      const auto converted=std::from_chars(token.data(),token.data()+token.size(),direct);
      if(converted.ec==std::errc{} && converted.ptr==token.data()+token.size() &&
          std::isfinite(direct) && (direct==0. || std::abs(direct)>=std::numeric_limits<double>::min()) &&
          (token.front()=='-' || token.front()=='.' ||
                                   (token.front()>='0' && token.front()<='9')))return direct;
    }
  }
  try {
    size_t used = 0;
    auto n = std::stod(std::string(text), &used);
    if (used != text.size() || (finite && !std::isfinite(n)))
      fail(core::Status::DecodeError, op, "invalid finite number");
    return n;
  } catch (const core::Exception&) {
    throw;
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid finite number");
  }
}
size_t bounded_count(int64_t n, const char* op, size_t limit = 1000000) {
  if (n < 0 || static_cast<uint64_t>(n) > limit)
    fail(core::Status::LimitExceeded, op, "file count exceeds limit");
  return static_cast<size_t>(n);
}

std::string path_string(std::string_view path, const char* op) {
  if (path.empty()) fail(core::Status::InvalidArgument, op, "path must not be empty");
  return std::string(path);
}

std::ifstream open_in(const std::string& path, const char* op) {
  std::ifstream in(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
  if (!in) fail(core::Status::IoError, op, "failed to open file for reading");
  if (in.tellg() > static_cast<std::streamoff>(64 * 1024 * 1024))
    fail(core::Status::LimitExceeded, op, "file exceeds 64 MiB");
  in.seekg(0);
  return in;
}

std::ofstream open_out(const std::string& path, const char* op, bool binary = false) {
  std::ofstream out(std::filesystem::u8path(path),
                    std::ios::trunc | (binary ? std::ios::binary : std::ios::openmode{}));
  if (!out) fail(core::Status::IoError, op, "failed to open file for writing");
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(17);
  return out;
}

std::string next_line(std::istream& in, const char* op) {
  std::string line;
  if (!std::getline(in, line)) fail(core::Status::DecodeError, op, "file is truncated");
  if (line.size() > 1048576) fail(core::Status::LimitExceeded, op, "line exceeds limit");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  return line;
}

// 每次标定文件读取固定一份空白规则，避免逐行构造 locale 和查找 facet。
class LineSyntax {
public:
  LineSyntax():classic_(locale_==std::locale::classic()),
               facet_(&std::use_facet<std::ctype<char>>(locale_)) {}
  bool space(char c) const {
    const auto value=static_cast<unsigned char>(c);
    return classic_ ? (c==' ' || (value>=9 && value<=13))
                    : facet_->is(std::ctype_base::space,c);
  }
private:
  std::locale locale_;
  bool classic_;
  const std::ctype<char>* facet_;
};

class LineTokens {
public:
  // 直接构造返回值，不复制未使用的内联偏移，也不延长借用行的生存期。
  LineTokens(const LineTokens&)=delete;
  LineTokens& operator=(const LineTokens&)=delete;
  LineTokens(LineTokens&&)=delete;
  LineTokens& operator=(LineTokens&&)=delete;
  explicit LineTokens(std::string line):line_(std::move(line)) {
    tokenize(line_,LineSyntax{});
  }
  LineTokens(std::string_view line,const LineSyntax& syntax):external_(line),borrowed_(true) {
    tokenize(line,syntax);
  }
  std::size_t size() const noexcept {return count_;}
  bool empty() const noexcept {return count_==0;}
  std::string_view operator[](std::size_t i) const {
    const auto [first,length]=i<local_.size()?local_[i]:extra_[i-local_.size()];
    return (borrowed_?external_:std::string_view(line_)).substr(first,length);
  }
private:
  void tokenize(std::string_view line,const LineSyntax& syntax) {
    // 八个以内的字段使用内联偏移；长行只为超出的偏移分配空间。
    for(std::size_t cursor=0;cursor<line.size();) {
      while(cursor<line.size() && syntax.space(line[cursor]))++cursor;
      const auto first=cursor;
      while(cursor<line.size() && !syntax.space(line[cursor]))++cursor;
      if(first!=cursor) {
        const auto range=std::pair{first,cursor-first};
        if(count_<local_.size())local_[count_]=range;else extra_.push_back(range);
        ++count_;
      }
    }
  }
  // 普通流拥有整行；标定文件借用调用期间保持不变的文件缓冲区。
  // 拥有模式使用偏移，移动短字符串不会留下悬空视图。
  std::string line_;
  std::string_view external_;
  bool borrowed_=false;
  std::array<std::pair<std::size_t,std::size_t>,8> local_;
  std::vector<std::pair<std::size_t,std::size_t>> extra_;
  std::size_t count_=0;
};
LineTokens split(std::string line) {return LineTokens(std::move(line));}
struct CalibrationLine {
  std::string_view text;
  const LineSyntax* syntax;
};
LineTokens split(CalibrationLine line) {return LineTokens(line.text,*line.syntax);}

class CalibrationInput {
public:
  explicit CalibrationInput(std::string_view bytes):remaining_(bytes) {}
  CalibrationLine next(const char* op) {
    if(remaining_.empty())fail(core::Status::DecodeError,op,"file is truncated");
    const auto end=remaining_.find('\n');
    auto line=remaining_.substr(0,end);
    remaining_.remove_prefix(end==std::string_view::npos?remaining_.size():end+1);
    // 与 getline 路径保持相同的长度检查顺序，CR 也计入限制。
    if(line.size()>1048576)fail(core::Status::LimitExceeded,op,"line exceeds limit");
    if(!line.empty() && line.back()=='\r')line.remove_suffix(1);
    return {line,&syntax_};
  }
  bool only_space_remains() const {
    for(char c:remaining_)if(!syntax_.space(c))return false;
    return true;
  }
private:
  std::string_view remaining_;
  LineSyntax syntax_;
};
CalibrationLine next_line(CalibrationInput& in,const char* op) {return in.next(op);}

void write_tuple(std::ostream& out, const core::Tuple& tuple) {
  out << "TUPLE " << tuple.size() << '\n';
  for (std::size_t i = 0; i < tuple.size(); ++i) {
    const auto value = tuple.at(i);
    switch (value.kind()) {
      case core::ValueKind::Int64:
        out << "I " << value.as_int64() << '\n';
        break;
      case core::ValueKind::Real64:
        out << "R " << value.as_real64() << '\n';
        break;
      case core::ValueKind::String:
        out << "S " << value.as_string().view() << '\n';
        break;
      default:
        fail(core::Status::InvalidArgument, "write_tuple", "unsupported tuple element kind");
    }
  }
}

template<class Input>
core::Tuple read_tuple(Input& in, const char* op, bool finite = true, bool decimal = false) {
  const auto header = split(next_line(in, op));
  if (header.size() != 2 || header[0] != "TUPLE") {
    fail(core::Status::DecodeError, op, "expected TUPLE header");
  }
  std::size_t count = 0;
  try {
    count = bounded_count(integer(header[1], op), op);
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid tuple size");
  }
  core::Tuple tuple;
  for (std::size_t i = 0; i < count; ++i) {
    const auto tokens = split(next_line(in, op));
    if (tokens.size() < 2) fail(core::Status::DecodeError, op, "truncated tuple element");
    try {
      if (tokens[0] == "I")
        tuple.append(static_cast<std::int64_t>(integer(tokens[1], op)));
      else if (tokens[0] == "R")
        tuple.append(number(tokens[1], op, finite, decimal));
      else if (tokens[0] == "S") {
        std::string text(tokens[1]);
        for (std::size_t t = 2; t < tokens.size(); ++t) {
          text.push_back(' ');
          text += tokens[t];
        }
        tuple.append(std::string_view{text});
      } else {
        fail(core::Status::DecodeError, op, "unknown tuple element tag");
      }
    } catch (const core::Exception&) {
      throw;
    } catch (...) {
      fail(core::Status::DecodeError, op, "invalid tuple element");
    }
  }
  return tuple;
}

const char* setup_name(SetupKind kind) {
  switch (kind) {
    case SetupKind::CalibrationObject:
      return "calibration_object";
    case SetupKind::HandEyeMovingCamera:
      return "hand_eye_moving_cam";
    case SetupKind::HandEyeStationaryCamera:
      return "hand_eye_stationary_cam";
    case SetupKind::HandEyeScaraMovingCamera:
      return "hand_eye_scara_moving_cam";
    case SetupKind::HandEyeScaraStationaryCamera:
      return "hand_eye_scara_stationary_cam";
  }
  return "calibration_object";
}

const char* target_name(TargetKind kind) {
  switch (kind) {
    case TargetKind::Unconfigured:
      return "unconfigured";
    case TargetKind::CustomPoints:
      return "custom_points";
    case TargetKind::Checkerboard:
      return "checkerboard";
    case TargetKind::CircleGrid:
      return "circle_grid";
    case TargetKind::Calplate:
      return "calplate";
    case TargetKind::Caltab:
      return "caltab";
  }
  return "unconfigured";
}

TargetKind parse_target_kind(std::string_view name, const char* op) {
  if (name == "unconfigured") return TargetKind::Unconfigured;
  if (name == "custom_points") return TargetKind::CustomPoints;
  if (name == "checkerboard") return TargetKind::Checkerboard;
  if (name == "circle_grid") return TargetKind::CircleGrid;
  if (name == "calplate") return TargetKind::Calplate;
  if (name == "caltab") return TargetKind::Caltab;
  fail(core::Status::DecodeError, op, "unknown calibration object kind");
  return TargetKind::Unconfigured;
}

void write_points3(std::ostream& out, const std::vector<Point3d>& points) {
  out << "POINTS " << points.size() << '\n';
  for (const auto& p : points) out << p.x << ' ' << p.y << ' ' << p.z << '\n';
}

template<class Input>
std::vector<Point3d> read_points3(Input& in, const char* op, bool decimal=false) {
  const auto header = split(next_line(in, op));
  if (header.size() != 2 || header[0] != "POINTS") fail(core::Status::DecodeError, op, "expected POINTS");
  std::size_t count = 0;
  try {
    count = bounded_count(integer(header[1], op), op);
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid point count");
  }
  std::vector<Point3d> points(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto tokens = split(next_line(in, op));
    if (tokens.size() != 3) fail(core::Status::DecodeError, op, "truncated point");
    try {
      points[i] = {number(tokens[0], op, false, decimal), number(tokens[1], op, false, decimal), number(tokens[2], op, false, decimal)};
    } catch (...) {
      fail(core::Status::DecodeError, op, "invalid point");
    }
  }
  return points;
}

}  // namespace

namespace {
// HALCON 24.11 HCPR v3，仅支持此处四类相机；头标志与单位转换已由双向探针验证。
using CameraHeader = std::array<unsigned char, 15>;
CameraHeader camera_header(bool telecentric, bool polynomial) {
  return {'H',
          'C',
          'P',
          'R',
          0,
          3,
          0,
          0,
          0,
          static_cast<unsigned char>(telecentric),
          0,
          static_cast<unsigned char>(polynomial),
          0,
          0,
          static_cast<unsigned char>(polynomial ? 12 : 8)};
}
std::vector<double> camera_scales(bool telecentric, bool polynomial) {
  std::vector<double> scales{telecentric ? 1. : 1000.};
  if (polynomial)
    scales.insert(scales.end(), {1e-6, 1e-12, 1e-18, 1e-3, 1e-3});
  else
    scales.push_back(1e-6);
  scales.insert(scales.end(), {1000., 1000., 1., 1.});
  return scales;
}
void write_big(std::ostream& out, std::uint64_t bits, std::size_t size) {
  for (std::size_t i = size; i > 0; --i) out.put(static_cast<char>((bits >> ((i - 1) * 8)) & 255));
}
std::uint64_t read_big(std::istream& in, std::size_t size, const char* op) {
  std::uint64_t bits = 0;
  for (std::size_t i = 0; i < size; ++i) {
    auto c = in.get();
    if (c == std::char_traits<char>::eof()) fail(core::Status::DecodeError, op, "truncated camera file");
    bits = (bits << 8) | static_cast<unsigned char>(c);
  }
  return bits;
}
core::Tuple read_hcpr(std::istream& in, const char* op) {
  CameraHeader header{};
  in.read(reinterpret_cast<char*>(header.data()), header.size());
  if (!in) fail(core::Status::DecodeError, op, "truncated camera header");
  bool telecentric = false, polynomial = false, found = false;
  for (bool t : {false, true})
    for (bool p : {false, true})
      if (header == camera_header(t, p)) {
        telecentric = t;
        polynomial = p;
        found = true;
      }
  if (!found) fail(core::Status::Unsupported, op, "unsupported HCPR camera model or version");
  const char* type =
      telecentric ? (polynomial ? "area_scan_telecentric_polynomial" : "area_scan_telecentric_division")
                  : (polynomial ? "area_scan_polynomial" : "area_scan_division");
  core::Tuple result;
  result.append(std::string_view(type));
  for (double scale : camera_scales(telecentric, polynomial)) {
    const auto bits = read_big(in, 8, op);
    double value;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    result.append(value / scale);
  }
  result.append(static_cast<std::int64_t>(read_big(in, 4, op)));
  result.append(static_cast<std::int64_t>(read_big(in, 4, op)));
  if (in.peek() != std::char_traits<char>::eof()) fail(core::Status::DecodeError, op, "trailing camera data");
  (void)parse_camera_parameters(result);
  return result;
}
}  // namespace

void write_cam_par(std::string_view path, const core::Tuple& camera_param) {
  constexpr auto op = "write_cam_par";
  (void)parse_camera_parameters(camera_param);
  const auto type = camera_param.string_view(0);
  const bool telecentric = type.find("telecentric") != std::string_view::npos;
  const bool polynomial = type.find("polynomial") != std::string_view::npos;
  const auto scales = camera_scales(telecentric, polynomial);
  for (std::size_t i = 0; i < 2; ++i)
    if (static_cast<std::uint64_t>(camera_param.as_int64(camera_param.size() - 2 + i)) >
        std::numeric_limits<std::uint32_t>::max())
      fail(core::Status::LimitExceeded, op, "image dimension exceeds file format limit");
  auto out = open_out(path_string(path, op), op, true);
  const auto header = camera_header(telecentric, polynomial);
  out.write(reinterpret_cast<const char*>(header.data()), header.size());
  for (std::size_t i = 0; i < scales.size(); ++i) {
    double value = camera_param.as_real64(i + 1) * scales[i];
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    write_big(out, bits, 8);
  }
  write_big(out, camera_param.as_int64(camera_param.size() - 2), 4);
  write_big(out, camera_param.as_int64(camera_param.size() - 1), 4);
  out.flush();
  if (!out) fail(core::Status::IoError, op, "failed to write camera parameters");
}

core::Tuple read_cam_par(std::string_view path) {
  constexpr auto op = "read_cam_par";
  auto in = open_in(path_string(path, op), op);
  if (in.peek() == 'H') return read_hcpr(in, op);
  const auto magic = split(next_line(in, op));
  if (magic.size() != 2 || magic[0] != "MVKIT_CAM_PAR") {
    fail(core::Status::DecodeError, op, "not an MVKit camera-parameter file");
  }
  if (magic[1] != "1") fail(core::Status::DecodeError, op, "unsupported camera-parameter version");
  auto tuple = read_tuple(in, op);
  (void)parse_camera_parameters(tuple);
  return tuple;
}

void write_pose(std::string_view path, const core::Tuple& pose) {
  constexpr auto op = "write_pose";
  (void)pose_from_halcon_tuple(pose);
  auto out = open_out(path_string(path, op), op);
  out << "# 3D pose parameters\nf " << pose.as_int64(6) << "\nr " << pose.as_real64(3) << ' '
      << pose.as_real64(4) << ' ' << pose.as_real64(5) << "\nt " << pose.as_real64(0) << ' '
      << pose.as_real64(1) << ' ' << pose.as_real64(2) << '\n';
  out.flush();
  if (!out) fail(core::Status::IoError, op, "failed to write pose");
}

core::Tuple read_pose(std::string_view path) {
  constexpr auto op = "read_pose";
  auto in = open_in(path_string(path, op), op);
  const bool decimal=decimal_input();
  const auto magic = split(next_line(in, op));
  if (magic.empty() || magic[0] != "MVKIT_POSE") {
    in.clear();
    in.seekg(0);
    std::array<double, 6> values{};
    std::int64_t code = 0;
    bool f = false, r = false, t = false;
    while (in.peek() != std::char_traits<char>::eof()) {
      auto line = next_line(in, op);
      auto hash = line.find('#');
      if (hash != std::string::npos) line.resize(hash);
      const auto tokens = split(line);
      if (tokens.empty()) continue;
      if (tokens[0] == "f" && tokens.size() == 2 && !f) {
        code = integer(tokens[1], op);
        f = true;
      } else if ((tokens[0] == "r" || tokens[0] == "t") && tokens.size() == 4) {
        bool& seen = tokens[0] == "r" ? r : t;
        if (seen) fail(core::Status::DecodeError, op, "duplicate pose record");
        seen = true;
        const std::size_t offset = tokens[0] == "r" ? 3 : 0;
        for (std::size_t i = 0; i < 3; ++i) values[offset + i] = number(tokens[i + 1], op, true, decimal);
      } else
        fail(core::Status::DecodeError, op, "invalid pose record");
    }
    if (!f || !r || !t) fail(core::Status::DecodeError, op, "missing pose record");
    core::Tuple tuple;
    for (auto value : values) tuple.append(value);
    tuple.append(code);
    (void)pose_from_halcon_tuple(tuple);
    return tuple;
  }
  if (magic.size() != 2) fail(core::Status::DecodeError, op, "invalid native pose header");
  if (magic[1] != "1") fail(core::Status::DecodeError, op, "unsupported pose version");
  auto tuple = read_tuple(in, op, true, decimal);
  (void)pose_from_halcon_tuple(tuple);
  return tuple;
}

void write_calib_data(std::string_view path, const CalibrationDataModel& data) {
  constexpr auto op = "write_calib_data";
  auto out = open_out(path_string(path, op), op);
  out << "MVKIT_CALIB_DATA 3\n";
  out << "setup " << setup_name(data.setup_kind()) << '\n';
  out << "cameras " << data.num_cameras() << '\n';
  out << "objects " << data.num_calibration_objects() << '\n';
  out << "optimization_method " << data.optimization_method() << '\n';
  out << "max_iterations " << data.options().max_iterations << '\n';
  out << "function_tolerance " << data.options().function_tolerance << '\n';
  out << "gradient_tolerance " << data.options().gradient_tolerance << '\n';
  out << "parameter_tolerance " << data.options().parameter_tolerance << '\n';
  out << "initial_damping " << data.options().initial_damping << '\n';
  for (std::size_t i = 0; i < data.num_cameras(); ++i) {
    const auto& camera = data.camera(static_cast<std::int64_t>(i));
    out << "camera " << i << '\n';
    if (camera.parameters) {
      out << "has_params 1\n";
      write_tuple(out, camera_parameters_to_tuple(*camera.parameters));
    } else {
      out << "has_params 0\n";
    }
    out << "has_init_params " << (camera.initial_parameters ? 1 : 0) << '\n';
    if (camera.initial_parameters) write_tuple(out, camera_parameters_to_tuple(*camera.initial_parameters));
    out << "excluded " << camera.excluded_settings.size();
    for (const auto& name : camera.excluded_settings) out << ' ' << name;
    out << '\n';
  }
  for (std::size_t i = 0; i < data.num_calibration_objects(); ++i) {
    const auto& target = data.target(static_cast<std::int64_t>(i));
    out << "object " << i << ' ' << target_name(target.kind) << ' ' << target.rows << ' ' << target.columns
        << ' ' << target.spacing_row << ' ' << target.spacing_column << ' ' << (target.light_on_dark ? 1 : 0)
        << '\n';
    write_points3(out, target.points);
    out << "radii " << target.mark_radii.size();
    for (double r : target.mark_radii) out << ' ' << r;
    out << '\n';
    out << "finders " << target.finders.size();
    for (const auto& f : target.finders) out << ' ' << f.column << ' ' << f.row;
    out << '\n';
    for (const auto& a : {target.caltab_rim, target.caltab_frame, target.caltab_triangle}) {
      out << "optional4 " << (a ? 1 : 0);
      if (a)
        for (double v : *a) out << ' ' << v;
      out << '\n';
    }
    out << "frame_width " << (target.caltab_frame_width ? 1 : 0);
    if (target.caltab_frame_width) out << ' ' << *target.caltab_frame_width;
    out << '\n';
  }
  out << "observations " << data.observations().size() << '\n';
  for (const auto& [key, obs] : data.observations()) {
    out << "observ " << key.camera_index << ' ' << key.calib_object_index << ' ' << key.pose_index << ' '
        << obs.rows.size() << ' ' << (obs.initial_pose || !obs.reported_pose.empty() ? 1 : 0) << '\n';
    for (std::size_t i = 0; i < obs.rows.size(); ++i) {
      out << obs.rows[i] << ' ' << obs.columns[i] << ' ' << obs.indices[i] << '\n';
    }
    if (!obs.reported_pose.empty())
      write_tuple(out, obs.reported_pose);
    else if (obs.initial_pose)
      write_tuple(out, pose_to_halcon_tuple(*obs.initial_pose));
  }
  out << "tool_poses " << data.tool_in_base_poses().size() << '\n';
  for (const auto& [index, pose] : data.tool_in_base_poses()) {
    out << "tool " << index << '\n';
    write_tuple(out, pose_to_halcon_tuple(pose));
  }
  if (!out) fail(core::Status::IoError, op, "failed to write calibration data");
}

CalibrationDataModel read_calib_data(std::string_view path) {
  constexpr auto op = "read_calib_data";
  const auto bytes=core::model_io::read_file(path_string(path,op),64U*1024U*1024U,op);
  CalibrationInput in(bytes);
  // 调用期间按当前环境解析；只准备一次 C 数值 locale 和舍入模式的适用条件。
  const bool decimal=decimal_input();
  const auto read_number=[&](std::string_view text,const char* operation,bool finite=true) {
    return number(text,operation,finite,decimal);
  };
  const auto magic = split(next_line(in, op));
  if (magic.size() != 2 || magic[0] != "MVKIT_CALIB_DATA") {
    fail(core::Status::DecodeError, op, "not an MVKit calibration-data file");
  }
  if (magic[1] != "1" && magic[1] != "2" && magic[1] != "3")
    fail(core::Status::Unsupported, op, "unsupported calibration-data version");
  const bool version2 = magic[1] != "1";
  const bool version3 = magic[1] == "3";
  const auto setup = split(next_line(in, op));
  if (setup.size() != 2 || setup[0] != "setup") fail(core::Status::DecodeError, op, "missing setup");
  const auto cameras_line = split(next_line(in, op));
  const auto objects_line = split(next_line(in, op));
  if (cameras_line.size() != 2 || cameras_line[0] != "cameras" || objects_line.size() != 2 ||
      objects_line[0] != "objects") {
    fail(core::Status::DecodeError, op, "missing camera/object counts");
  }
  std::int64_t n_cameras = 0;
  std::int64_t n_objects = 0;
  try {
    n_cameras = integer(cameras_line[1], op);
    n_objects = integer(objects_line[1], op);
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid camera/object counts");
  }
  bounded_count(n_cameras, op, 1024);
  bounded_count(n_objects, op, 1024);
  auto data = CalibrationDataModel::create(setup[1], n_cameras, n_objects);
  const auto method = split(next_line(in, op));
  if (method.size() != 2 || method[0] != "optimization_method") {
    fail(core::Status::DecodeError, op, "missing optimization_method");
  }
  data.set_optimization_method(std::string(method[1]));
  auto read_option = [&](const char* key, auto assign) {
    const auto tokens = split(next_line(in, op));
    if (tokens.size() != 2 || tokens[0] != key) fail(core::Status::DecodeError, op, "missing solver option");
    try {
      assign(tokens[1]);
    } catch (...) {
      fail(core::Status::DecodeError, op, "invalid solver option");
    }
  };
  read_option("max_iterations", [&](std::string_view v) {
    data.options().max_iterations = bounded_count(integer(v, op), op);
  });
  read_option("function_tolerance",
              [&](std::string_view v) { data.options().function_tolerance = read_number(v, op); });
  read_option("gradient_tolerance",
              [&](std::string_view v) { data.options().gradient_tolerance = read_number(v, op); });
  read_option("parameter_tolerance",
              [&](std::string_view v) { data.options().parameter_tolerance = read_number(v, op); });
  read_option("initial_damping",
              [&](std::string_view v) { data.options().initial_damping = read_number(v, op); });
  const auto& options = data.options();
  if (options.max_iterations == 0 || options.function_tolerance < 0 || options.gradient_tolerance < 0 ||
      options.parameter_tolerance < 0 || options.initial_damping <= 0)
    fail(core::Status::DecodeError, op, "invalid solver options");
  if (method[1] != "lm" && method[1] != "linear" && method[1] != "nonlinear")
    fail(core::Status::Unsupported, op, "unsupported solver method");
  for (std::int64_t i = 0; i < n_cameras; ++i) {
    const auto header = split(next_line(in, op));
    if (header.size() != 2 || header[0] != "camera" || integer(header[1], op) != i) {
      fail(core::Status::DecodeError, op, "unexpected camera section");
    }
    const auto has = split(next_line(in, op));
    if (has.size() != 2 || has[0] != "has_params") fail(core::Status::DecodeError, op, "missing has_params");
    if (has[1] == "1") {
      set_camera_parameters(data, i, read_tuple(in, op, true, decimal));
    } else if (has[1] != "0") {
      fail(core::Status::DecodeError, op, "invalid has_params");
    }
    if (version3) {
      const auto initial = split(next_line(in, op));
      if (initial.size() != 2 || initial[0] != "has_init_params" || (initial[1] != "0" && initial[1] != "1"))
        fail(core::Status::DecodeError, op, "invalid has_init_params");
      if (initial[1] == "1")
        data.camera(i).initial_parameters = parse_camera_parameters(read_tuple(in, op, true, decimal));
      else
        data.camera(i).initial_parameters.reset();
    }
    const auto excluded = split(next_line(in, op));
    if (excluded.size() < 2 || excluded[0] != "excluded")
      fail(core::Status::DecodeError, op, "missing excluded");
    const auto excluded_count = bounded_count(integer(excluded[1], op), op);
    if (excluded.size() != excluded_count + 2) fail(core::Status::DecodeError, op, "invalid excluded count");
    if (has[1] == "1") {
      core::Tuple names;
      for (std::size_t t = 2; t < excluded.size(); ++t) names.append(std::string_view{excluded[t]});
      // 反序列化恢复整个集合，不追加到默认排除项。
      data.camera(i).excluded_settings.clear();
      set_calib_data(data, "camera", core::Tuple{i}, "excluded_settings", names);
    }
  }
  for (std::int64_t i = 0; i < n_objects; ++i) {
    const auto header = split(next_line(in, op));
    if (header.size() < 7 || header[0] != "object" || integer(header[1], op) != i) {
      fail(core::Status::DecodeError, op, "unexpected object section");
    }
    auto& target = data.target(i);
    target.kind = parse_target_kind(header[2], op);
    target.rows = integer(header[3], op);
    target.columns = integer(header[4], op);
    target.spacing_row = read_number(header[5], op);
    target.spacing_column = read_number(header[6], op);
    target.light_on_dark = header.size() < 8 || header[7] != "0";
    target.points = read_points3(in, op, decimal);
    if (version2) {
      auto radii = split(next_line(in, op));
      if (radii.size() < 2 || radii[0] != "radii") fail(core::Status::DecodeError, op, "missing radii");
      const auto nr = bounded_count(integer(radii[1], op), op);
      if (radii.size() != nr + 2 || (nr != 0 && nr != target.points.size()))
        fail(core::Status::DecodeError, op, "invalid radii count");
      for (size_t r = 0; r < nr; ++r) {
        double v = read_number(radii[r + 2], op);
        if (v <= 0) fail(core::Status::DecodeError, op, "invalid radius");
        target.mark_radii.push_back(v);
      }
      auto finders = split(next_line(in, op));
      if (finders.size() < 2 || finders[0] != "finders")
        fail(core::Status::DecodeError, op, "missing finders");
      const auto nf = bounded_count(integer(finders[1], op), op);
      if (finders.size() != 2 * nf + 2) fail(core::Status::DecodeError, op, "invalid finder count");
      for (size_t f = 0; f < nf; ++f)
        target.finders.push_back({integer(finders[2 + 2 * f], op), integer(finders[3 + 2 * f], op)});
      for (auto* a : {&target.caltab_rim, &target.caltab_frame, &target.caltab_triangle}) {
        auto values = split(next_line(in, op));
        if (values.size() < 2 || values[0] != "optional4")
          fail(core::Status::DecodeError, op, "missing target geometry");
        const auto present = bounded_count(integer(values[1], op), op, 1);
        if (values.size() != 2 + 4 * present) fail(core::Status::DecodeError, op, "invalid target geometry");
        if (present)
          *a = std::array<double, 4>{read_number(values[2], op), read_number(values[3], op), read_number(values[4], op),
                                     read_number(values[5], op)};
      }
      auto width = split(next_line(in, op));
      if (width.size() < 2 || width[0] != "frame_width")
        fail(core::Status::DecodeError, op, "missing frame width");
      const auto present = bounded_count(integer(width[1], op), op, 1);
      if (width.size() != 2 + present) fail(core::Status::DecodeError, op, "invalid frame width");
      if (present) target.caltab_frame_width = read_number(width[2], op);
    }
  }
  const auto observ_header = split(next_line(in, op));
  if (observ_header.size() != 2 || observ_header[0] != "observations") {
    fail(core::Status::DecodeError, op, "missing observations");
  }
  const auto n_obs = bounded_count(integer(observ_header[1], op), op);
  for (std::size_t n = 0; n < n_obs; ++n) {
    const auto header = split(next_line(in, op));
    if (header.size() != 6 || header[0] != "observ")
      fail(core::Status::DecodeError, op, "truncated observation");
    ObservationKey key{integer(header[1], op), integer(header[2], op), integer(header[3], op)};
    const auto count = bounded_count(integer(header[4], op), op);
    if (header[5] != "0" && header[5] != "1") fail(core::Status::DecodeError, op, "invalid pose flag");
    const bool has_pose = header[5] != "0";
    CalibrationObservation obs;
    obs.rows.resize(count);
    obs.columns.resize(count);
    obs.indices.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto tokens = split(next_line(in, op));
      if (tokens.size() != 3) fail(core::Status::DecodeError, op, "truncated observation point");
      obs.rows[i] = read_number(tokens[0], op, false);
      obs.columns[i] = read_number(tokens[1], op, false);
      obs.indices[i] = integer(tokens[2], op);
    }
    if (has_pose) {
      obs.reported_pose = read_tuple(in, op, false, decimal);
      if (obs.reported_pose.size() != 7) fail(core::Status::DecodeError, op, "invalid observation pose");
      const auto code = obs.reported_pose.as_int64(6);
      if (code < 0 || code > 13 || (code & 6) == 6)
        fail(core::Status::DecodeError, op, "invalid pose representation code");
      bool finite = true;
      for (std::size_t i = 0; i < 6; ++i) finite = std::isfinite(obs.reported_pose.as_real64(i)) && finite;
      if (finite) {
        obs.initial_pose = pose_from_halcon_tuple(obs.reported_pose);
        const auto standard = code == 0 ? obs.reported_pose : pose_to_halcon_tuple(*obs.initial_pose);
        core::Tuple reported;
        for (std::size_t i = 0; i < 6; ++i) {
          auto value = standard.as_real64(i);
          if (i >= 3) {
            value = std::fmod(value, 360.);
            if (value < 0) value += 360.;
          }
          reported.append(value);
        }
        reported.append(std::int64_t{0});
        obs.reported_pose = std::move(reported);
      } else if (code != 0)
        fail(core::Status::DecodeError, op, "nonfinite nonstandard stored pose unsupported");
    }
    if (data.observations().count(key)) fail(core::Status::DecodeError, op, "duplicate observation");
    (void)data.camera(key.camera_index);
    const auto& target = data.target(key.calib_object_index);
    if (key.pose_index < 0) fail(core::Status::DecodeError, op, "invalid pose index");
    for (auto index : obs.indices)
      if (index < 0 || static_cast<size_t>(index) >= target.points.size())
        fail(core::Status::DecodeError, op, "invalid point index");
    data.replace_observation(key, std::move(obs));
  }
  const auto tools = split(next_line(in, op));
  if (tools.size() != 2 || tools[0] != "tool_poses")
    fail(core::Status::DecodeError, op, "missing tool poses");
  const auto n_tools = bounded_count(integer(tools[1], op), op);
  for (std::size_t i = 0; i < n_tools; ++i) {
    const auto header = split(next_line(in, op));
    if (header.size() != 2 || header[0] != "tool") fail(core::Status::DecodeError, op, "truncated tool pose");
    data.set_tool_in_base_pose(integer(header[1], op), pose_from_halcon_tuple(read_tuple(in, op, true, decimal)));
  }
  if (!in.only_space_remains()) fail(core::Status::DecodeError, op, "trailing calibration data");
  return data;
}

}  // namespace mv::calibration
