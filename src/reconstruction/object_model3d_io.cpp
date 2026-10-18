#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <core/exception.hpp>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <reconstruction/object_model3d.hpp>
#include <runtime/execution/execution_context.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace mv::reconstruction {
namespace {

[[noreturn]] void fail(core::Status status, const char* op, const char* message) {
  throw core::Exception(status, op, message);
}

std::string path_string(std::string_view path, const char* op) {
  if (path.empty()) fail(core::Status::InvalidArgument, op, "path must not be empty");
  return std::string(path);
}

std::string ascii_lower(std::string text) {
  for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}

std::ifstream open_in(const std::string& path, const char* op) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) fail(core::Status::IoError, op, "failed to open file for reading");
  if (in.tellg() > static_cast<std::streamoff>(1024ULL * 1024ULL * 1024ULL))
    fail(core::Status::LimitExceeded, op, "PLY exceeds 1 GiB");
  in.seekg(0);
  return in;
}

std::ofstream open_out(const std::string& path, const char* op) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) fail(core::Status::IoError, op, "failed to open file for writing");
  return out;
}

std::string next_line(std::istream& in, const char* op) {
  std::string line;
  if (!std::getline(in, line)) fail(core::Status::DecodeError, op, "file is truncated");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  return line;
}

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> tokens;
  std::istringstream ss(line);
  std::string token;
  while (ss >> token) tokens.push_back(std::move(token));
  return tokens;
}

bool point_valid(const ObjectModel3DData& model, std::size_t index) {
  return model.valid.empty() || model.valid[index] != 0U;
}

enum class PlyFormat { Ascii, BinaryLe };

enum class PlyType { Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64 };

PlyType parse_ply_type(const std::string& name, const char* op) {
  if (name == "char" || name == "int8") return PlyType::Int8;
  if (name == "uchar" || name == "uint8") return PlyType::UInt8;
  if (name == "short" || name == "int16") return PlyType::Int16;
  if (name == "ushort" || name == "uint16") return PlyType::UInt16;
  if (name == "int" || name == "int32") return PlyType::Int32;
  if (name == "uint" || name == "uint32") return PlyType::UInt32;
  if (name == "float" || name == "float32") return PlyType::Float32;
  if (name == "double" || name == "float64") return PlyType::Float64;
  fail(core::Status::DecodeError, op, "unsupported PLY property type");
  return PlyType::Float32;
}

std::size_t ply_type_size(PlyType type) {
  switch (type) {
    case PlyType::Int8:
    case PlyType::UInt8:
      return 1;
    case PlyType::Int16:
    case PlyType::UInt16:
      return 2;
    case PlyType::Int32:
    case PlyType::UInt32:
    case PlyType::Float32:
      return 4;
    case PlyType::Float64:
      return 8;
  }
  return 0;
}

double ply_type_to_double(PlyType type, const std::uint8_t* bytes) {
  switch (type) {
    case PlyType::Int8: {
      std::int8_t v{};
      std::memcpy(&v, bytes, 1);
      return static_cast<double>(v);
    }
    case PlyType::UInt8:
      return static_cast<double>(bytes[0]);
    case PlyType::Int16: {
      std::int16_t v{};
      std::memcpy(&v, bytes, 2);
      return static_cast<double>(v);
    }
    case PlyType::UInt16: {
      std::uint16_t v{};
      std::memcpy(&v, bytes, 2);
      return static_cast<double>(v);
    }
    case PlyType::Int32: {
      std::int32_t v{};
      std::memcpy(&v, bytes, 4);
      return static_cast<double>(v);
    }
    case PlyType::UInt32: {
      std::uint32_t v{};
      std::memcpy(&v, bytes, 4);
      return static_cast<double>(v);
    }
    case PlyType::Float32: {
      float v{};
      std::memcpy(&v, bytes, 4);
      return static_cast<double>(v);
    }
    case PlyType::Float64: {
      double v{};
      std::memcpy(&v, bytes, 8);
      return v;
    }
  }
  return 0.0;
}

struct PlyProperty {
  std::string name;
  PlyType type{PlyType::Float32};
};

double ascii_number(const std::string& token, const char* op) {
  try {
    std::size_t consumed = 0;
    const double value = std::stod(token, &consumed);
    if (consumed != token.size()) fail(core::Status::DecodeError, op, "invalid PLY number");
    return value;
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid PLY number");
    return 0.0;
  }
}

size_t native_count(std::istream& in, const char* op, size_t limit = 10000000) {
  std::string token;
  if (!(in >> token)) fail(core::Status::DecodeError, op, "truncated count");
  try {
    size_t used = 0;
    const auto n = std::stoll(token, &used);
    if (used != token.size() || n < 0 || static_cast<uint64_t>(n) > limit)
      fail(core::Status::LimitExceeded, op, "invalid or excessive count");
    return static_cast<size_t>(n);
  } catch (const core::Exception&) {
    throw;
  } catch (...) {
    fail(core::Status::DecodeError, op, "invalid count");
  }
}
double native_real(std::istream& in, const char* op) {
  std::string token;
  if (!(in >> token)) fail(core::Status::DecodeError, op, "truncated number");
  return ascii_number(token, op);
}
void write_native(std::string_view path, const ObjectModel3DData& m) {
  constexpr auto op = "write_object_model_3d";
  if (m.points.size() > 10000000 || m.triangles.size() > 20000000 || m.lines.size() > 20000000)
    fail(core::Status::LimitExceeded, op, "native model exceeds count limits");
  auto out = open_out(path_string(path, op), op);
  out.precision(17);
  const bool quality = !m.observed_valid.empty();
  const auto attributes = m.point_attributes.Read();
  const bool rich = m.primitive_support || (attributes && !attributes->values.empty());
  const bool topology =
      rich || !m.lines.empty() || m.mapping_width || m.mapping_height || m.triangles_are_polygons;
  if ((m.primitive_support && m.primitive_support->plane_boundary.size() > 10000000) ||
      (attributes && attributes->values.size() > 10000))
    fail(core::Status::LimitExceeded, op, "native metadata exceeds count limits");
  out << "MVKIT_OBJECT_MODEL_3D "
      << (rich       ? 5
          : topology ? 4
          : quality  ? 3
                     : 2)
      << "\n"
      << m.points.size() << ' ' << m.width << ' ' << m.height << ' ' << !m.valid.empty() << ' '
      << !m.confidence.empty() << ' ' << !m.normals.empty() << '\n';
  for (size_t i = 0; i < m.points.size(); ++i) {
    const auto& p = m.points[i];
    out << p.x << ' ' << p.y << ' ' << p.z;
    if (!m.valid.empty()) out << ' ' << static_cast<unsigned>(m.valid[i]);
    if (!m.confidence.empty()) out << ' ' << m.confidence[i];
    if (!m.normals.empty()) {
      const auto& n = m.normals[i];
      out << ' ' << n.x << ' ' << n.y << ' ' << n.z;
    }
    out << '\n';
  }
  out << m.triangles.size() << '\n';
  for (const auto& t : m.triangles) out << t[0] << ' ' << t[1] << ' ' << t[2] << '\n';
  out << static_cast<int>(m.primitive_kind) << '\n';
  for (double v : m.plane) out << v << ' ';
  out << m.primitive_rms << ' ' << m.primitive_num_inliers << '\n';
  for (double v : m.primitive) out << v << ' ';
  out << m.fit_iterations << ' ' << m.fit_converged << ' ' << m.information_ratio << '\n';
  out << m.source_indices.size() << '\n';
  for (auto i : m.source_indices) out << i << ' ';
  out << '\n';
  out << m.voxel_counts.size() << '\n';
  for (auto i : m.voxel_counts) out << i << ' ';
  out << '\n';
  out << m.input_to_sample.size() << '\n';
  for (auto i : m.input_to_sample) out << i << ' ';
  out << '\n';
  if (topology) out << quality << '\n';
  if (quality) {
    const auto write_attribute = [&](const auto& values) {
      out << values.size() << '\n';
      for (auto v : values) out << static_cast<unsigned>(v) << ' ';
      out << '\n';
    };
    write_attribute(m.observed_valid);
    write_attribute(m.filled);
    write_attribute(m.quality_reason);
    out << m.depth_uncertainty.size() << '\n';
    for (float v : m.depth_uncertainty) out << v << ' ';
    out << '\n';
  }
  if (topology) {
    out << m.mapping_width << ' ' << m.mapping_height << ' ' << m.triangles_are_polygons << '\n';
    out << m.lines.size() << '\n';
    for (const auto& line : m.lines) out << line[0] << ' ' << line[1] << '\n';
  }
  if (rich) {
    out << static_cast<bool>(m.primitive_support) << '\n';
    const auto point = [&](Point3d p) { out << p.x << ' ' << p.y << ' ' << p.z << '\n'; };
    if (m.primitive_support) {
      const auto& s = *m.primitive_support;
      point(s.origin);
      point(s.axis_x);
      point(s.axis_y);
      point(s.axis_z);
      out << s.cylinder_extent[0] << ' ' << s.cylinder_extent[1] << '\n';
      out << s.plane_boundary.size() << '\n';
      for (auto p : s.plane_boundary) point(p);
    }
    out << (attributes ? attributes->values.size() : 0) << '\n';
    if (attributes)
      for (const auto& [name, values] : attributes->values) {
        if (name.size() < 2 || name.size() > 4096 || (name[0] != '&' && name != "original_point_indices") ||
            name.find('\0') != std::string::npos)
          fail(core::Status::InvalidArgument, op, "invalid extended attribute name");
        out << std::quoted(name) << ' ' << values.size() << '\n';
        for (auto value : values) out << value << ' ';
        out << '\n';
      }
  }
  out << "END\n";
  out.flush();
  if (!out) fail(core::Status::IoError, op, "failed writing native model");
}
ObjectModel3DData read_native(std::istream& in, double scale, int version) {
  constexpr auto op = "read_object_model_3d";
  ObjectModel3DData m;
  const auto n = native_count(in, op);
  m.width = native_count(in, op);
  m.height = native_count(in, op);
  const bool valid = native_count(in, op, 1) != 0, confidence = native_count(in, op, 1) != 0,
             normals = native_count(in, op, 1) != 0;
  if ((m.width == 0) != (m.height == 0) || (m.width && (m.height > n / m.width || m.width * m.height != n)))
    fail(core::Status::DecodeError, op, "invalid organized dimensions");
  // 每解码一条记录再扩容，截断文件不能强制按声明数量分配。
  for (size_t i = 0; i < n; ++i) {
    Point3d p{native_real(in, op) * scale, native_real(in, op) * scale, native_real(in, op) * scale};
    const auto v = valid ? native_count(in, op, 1) : 1;
    if (v && (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)))
      fail(core::Status::DecodeError, op, "invalid valid-point coordinates");
    m.points.push_back(p);
    if (valid) m.valid.push_back(static_cast<uint8_t>(v));
    if (confidence) {
      const double c = native_real(in, op);
      if (!std::isfinite(c) || c < 0 || c > 1) fail(core::Status::DecodeError, op, "invalid confidence");
      m.confidence.push_back(static_cast<float>(c));
    }
    if (normals) {
      Point3d normal{native_real(in, op), native_real(in, op), native_real(in, op)};
      if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z))
        fail(core::Status::DecodeError, op, "invalid normal");
      m.normals.push_back(normal);
    }
  }
  const auto triangles = native_count(in, op, 20000000);
  for (size_t i = 0; i < triangles; ++i) {
    std::array<uint32_t, 3> t{};
    for (auto& index : t) {
      const auto v = native_count(in, op);
      if (v >= n) fail(core::Status::DecodeError, op, "triangle index out of range");
      index = static_cast<uint32_t>(v);
    }
    m.triangles.push_back(t);
  }
  m.primitive_kind = static_cast<PrimitiveKind>(native_count(in, op, version >= 2 ? 3 : 1));
  for (auto& v : m.plane) {
    v = native_real(in, op);
    if (!std::isfinite(v)) fail(core::Status::DecodeError, op, "invalid primitive");
  }
  m.plane[3] *= scale;
  m.primitive_rms = native_real(in, op) * scale;
  m.primitive_num_inliers = native_count(in, op);
  if (!std::isfinite(m.primitive_rms) || m.primitive_rms < 0 || m.primitive_num_inliers > n)
    fail(core::Status::DecodeError, op, "invalid primitive quality");
  if (version >= 2) {
    for (auto& v : m.primitive) {
      v = native_real(in, op);
      if (!std::isfinite(v)) fail(core::Status::DecodeError, op, "invalid primitive");
    }
    if (m.primitive_kind == PrimitiveKind::Sphere || m.primitive_kind == PrimitiveKind::Cylinder) {
      for (size_t i = 0; i < 3; ++i) m.primitive[i] *= scale;
      m.primitive[m.primitive_kind == PrimitiveKind::Sphere ? 3 : 6] *= scale;
    }
    m.fit_iterations = native_count(in, op, 10000);
    m.fit_converged = native_count(in, op, 1) != 0;
    m.information_ratio = native_real(in, op);
    if (!std::isfinite(m.information_ratio) || m.information_ratio < 0 || m.information_ratio > 1)
      fail(core::Status::DecodeError, op, "invalid conditioning");
    auto count = native_count(in, op);
    if (count && count != n) fail(core::Status::DecodeError, op, "invalid source index count");
    for (size_t i = 0; i < count; ++i)
      m.source_indices.push_back(native_count(in, op, std::numeric_limits<size_t>::max()));
    count = native_count(in, op);
    if (count && count != n) fail(core::Status::DecodeError, op, "invalid voxel count");
    for (size_t i = 0; i < count; ++i) {
      auto v = native_count(in, op);
      if (!v) fail(core::Status::DecodeError, op, "zero voxel count");
      m.voxel_counts.push_back(v);
    }
    count = native_count(in, op);
    for (size_t i = 0; i < count; ++i) {
      const double v = native_real(in, op);
      if (!std::isfinite(v) || std::floor(v) != v || v < -1 || v >= static_cast<double>(n))
        fail(core::Status::DecodeError, op, "invalid mapping");
      m.input_to_sample.push_back(static_cast<int64_t>(v));
    }
  }
  const bool quality = version == 3 || (version >= 4 && native_count(in, op, 1) != 0);
  if (quality) {
    const auto read_attribute = [&](auto& values, size_t limit) {
      const auto count = native_count(in, op);
      if (count != n) fail(core::Status::DecodeError, op, "invalid quality count");
      for (size_t i = 0; i < count; ++i)
        values.push_back(
            static_cast<typename std::decay_t<decltype(values)>::value_type>(native_count(in, op, limit)));
    };
    read_attribute(m.observed_valid, 1);
    read_attribute(m.filled, 1);
    read_attribute(m.quality_reason, 511);
    const auto count = native_count(in, op);
    if (count && count != n) fail(core::Status::DecodeError, op, "invalid uncertainty count");
    for (size_t i = 0; i < count; ++i)
      m.depth_uncertainty.push_back(static_cast<float>(native_real(in, op) * scale));
  }
  if (version >= 4) {
    m.mapping_width = native_count(in, op);
    m.mapping_height = native_count(in, op);
    m.triangles_are_polygons = native_count(in, op, 1) != 0;
    if ((m.mapping_width == 0) != (m.mapping_height == 0))
      fail(core::Status::DecodeError, op, "invalid sparse mapping dimensions");
    const auto count = native_count(in, op, 20000000);
    for (size_t i = 0; i < count; ++i) {
      std::array<uint32_t, 2> line{};
      for (auto& index : line) {
        const auto slot = native_count(in, op);
        if (slot >= n) fail(core::Status::DecodeError, op, "line index out of range");
        index = static_cast<uint32_t>(slot);
      }
      m.lines.push_back(line);
    }
  }
  if (version >= 5) {
    if (native_count(in, op, 1)) {
      auto s = std::allocate_shared<PrimitiveSupport>(
          std::pmr::polymorphic_allocator<PrimitiveSupport>{runtime::persist_resource()});
      const auto point = [&](double factor) {
        return Point3d{native_real(in, op) * factor, native_real(in, op) * factor,
                       native_real(in, op) * factor};
      };
      s->origin = point(scale);
      s->axis_x = point(1);
      s->axis_y = point(1);
      s->axis_z = point(1);
      s->cylinder_extent = {native_real(in, op) * scale, native_real(in, op) * scale};
      const auto count = native_count(in, op);
      for (size_t i = 0; i < count; ++i) s->plane_boundary.push_back(point(scale));
      m.primitive_support = std::move(s);
    }
    const auto count = native_count(in, op, 10000);
    std::vector<std::string> names;
    std::vector<std::vector<double>> values;
    std::unordered_set<std::string> seen;
    for (size_t i = 0; i < count; ++i) {
      std::string name;
      if (!(in >> std::quoted(name)) || name.size() < 2 || name.size() > 4096 ||
          (name[0] != '&' && name != "original_point_indices") || name.find('\0') != std::string::npos ||
          !seen.insert(name).second)
        fail(core::Status::DecodeError, op, "invalid extended attribute name");
      const auto length = native_count(in, op);
      if (length != n) fail(core::Status::DecodeError, op, "extended attribute point count mismatch");
      std::vector<double> column;
      for (size_t j = 0; j < length; ++j) column.push_back(native_real(in, op));
      names.push_back(std::move(name));
      values.push_back(std::move(column));
    }
    std::vector<PointAttributes::Update> updates;
    for (size_t i = 0; i < count; ++i) updates.push_back({names[i], values[i]});
    if (count) m.point_attributes.Replace(updates);
  }
  std::string end;
  if (!(in >> end) || end != "END") fail(core::Status::DecodeError, op, "missing native model footer");
  in >> std::ws;
  if (!in.eof()) fail(core::Status::DecodeError, op, "trailing native model data");
  validate_object_model_3d(m);
  return m;
}

}  // namespace

void write_object_model_3d(std::string_view path, std::string_view file_type,
                           const ObjectModel3DData& model) {
  constexpr auto op = "write_object_model_3d";
  validate_object_model_3d(model);
  if (ascii_lower(std::string(file_type)) == "mvkit") {
    write_native(path, model);
    return;
  }
  if (ascii_lower(std::string(file_type)) != "ply") {
    fail(core::Status::Unsupported, op, "supported file types are ply and mvkit");
  }

  std::size_t count = 0;
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (point_valid(model, i)) ++count;
  }
  const bool has_confidence = !model.confidence.empty();
  auto out = open_out(path_string(path, op), op);
  out << "ply\nformat ascii 1.0\ncomment MVKit ObjectModel3D\n";
  out << "element vertex " << count << '\n';
  out << "property double x\nproperty double y\nproperty double z\n";
  if (has_confidence) out << "property float confidence\n";
  if (!model.observed_valid.empty())
    out << "property uchar observed_valid\nproperty uchar filled\nproperty ushort quality_reason\n";
  if (!model.depth_uncertainty.empty()) out << "property float depth_uncertainty\n";
  if (!model.normals.empty()) out << "property double nx\nproperty double ny\nproperty double nz\n";
  if (!model.source_indices.empty()) out << "property double source_index\n";
  if (!model.voxel_counts.empty()) out << "property double voxel_count\n";
  runtime::ExecutionContext context;
  std::pmr::vector<size_t> remap(model.points.size(), std::numeric_limits<size_t>::max(),
                                 context.scratch_resource());
  size_t next = 0;
  for (size_t i = 0; i < model.points.size(); ++i)
    if (point_valid(model, i)) remap[i] = next++;
  size_t faces = 0;
  for (const auto& t : model.triangles)
    if (point_valid(model, t[0]) && point_valid(model, t[1]) && point_valid(model, t[2])) ++faces;
  if (faces) out << "element face " << faces << "\nproperty list uchar uint vertex_indices\n";
  out << "end_header\n";
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(17);
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (!point_valid(model, i)) continue;
    out << model.points[i].x << ' ' << model.points[i].y << ' ' << model.points[i].z;
    if (has_confidence) out << ' ' << model.confidence[i];
    if (!model.observed_valid.empty())
      out << ' ' << unsigned(model.observed_valid[i]) << ' ' << unsigned(model.filled[i]) << ' '
          << model.quality_reason[i];
    if (!model.depth_uncertainty.empty()) out << ' ' << model.depth_uncertainty[i];
    if (!model.normals.empty()) {
      const auto n = model.normals[i];
      out << ' ' << n.x << ' ' << n.y << ' ' << n.z;
    }
    if (!model.source_indices.empty()) out << ' ' << model.source_indices[i];
    if (!model.voxel_counts.empty()) out << ' ' << model.voxel_counts[i];
    out << '\n';
  }
  for (const auto& t : model.triangles)
    if (point_valid(model, t[0]) && point_valid(model, t[1]) && point_valid(model, t[2]))
      out << "3 " << remap[t[0]] << ' ' << remap[t[1]] << ' ' << remap[t[2]] << '\n';
  out.flush();
  if (!out) fail(core::Status::IoError, op, "failed to write PLY");
}

ObjectModel3DData read_object_model_3d(std::string_view path, double scale) {
  constexpr auto op = "read_object_model_3d";
  if (!std::isfinite(scale) || scale <= 0.0) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, "Scale", -1,
                          "scale must be finite and positive");
  }
  auto in = open_in(path_string(path, op), op);
  const auto magic = next_line(in, op);
  if (magic == "MVKIT_OBJECT_MODEL_3D 1" || magic == "MVKIT_OBJECT_MODEL_3D 2" ||
      magic == "MVKIT_OBJECT_MODEL_3D 3" || magic == "MVKIT_OBJECT_MODEL_3D 4" ||
      magic == "MVKIT_OBJECT_MODEL_3D 5")
    return read_native(in, scale, magic.back() - '0');
  if (magic.rfind("MVKIT_OBJECT_MODEL_3D ", 0) == 0)
    fail(core::Status::Unsupported, op, "unsupported native model version");
  if (magic != "ply") fail(core::Status::DecodeError, op, "not a PLY or native MVKit model");

  PlyFormat format = PlyFormat::Ascii;
  std::vector<PlyProperty> vertex_properties;
  std::size_t vertex_count = 0;
  bool seen_vertex = false;
  bool seen_format = false;
  bool in_vertex = false;
  bool header_done = false;
  bool in_face = false, seen_face = false, face_property = false;
  size_t face_count = 0;
  PlyType face_size_type = PlyType::UInt8, face_index_type = PlyType::UInt32;
  while (!header_done) {
    const auto line = next_line(in, op);
    const auto tokens = split(line);
    if (tokens.empty()) continue;
    if (tokens[0] == "comment" || tokens[0] == "obj_info") continue;
    if (tokens[0] == "format") {
      if (tokens.size() != 3 || seen_format) fail(core::Status::DecodeError, op, "invalid PLY format line");
      seen_format = true;
      if (tokens[2] != "1.0") fail(core::Status::Unsupported, op, "unsupported PLY version");
      if (tokens[1] == "ascii")
        format = PlyFormat::Ascii;
      else if (tokens[1] == "binary_little_endian")
        format = PlyFormat::BinaryLe;
      else
        fail(core::Status::DecodeError, op, "unsupported PLY format");
      continue;
    }
    if (tokens[0] == "element") {
      if (tokens.size() < 3) fail(core::Status::DecodeError, op, "invalid PLY element");
      if (tokens[1] == "vertex") {
        if (seen_vertex) fail(core::Status::DecodeError, op, "multiple vertex elements");
        seen_vertex = true;
        in_vertex = true;
        in_face = false;
        try {
          size_t used = 0;
          const auto n = std::stoll(tokens[2], &used);
          if (used != tokens[2].size() || n < 0 || n > 10000000)
            fail(core::Status::LimitExceeded, op, "invalid or excessive vertex count");
          vertex_count = static_cast<size_t>(n);
        } catch (const core::Exception&) {
          throw;
        } catch (...) {
          fail(core::Status::DecodeError, op, "invalid vertex count");
        }
      } else if (!seen_vertex) {
        fail(core::Status::DecodeError, op, "vertex must be the first PLY element");
      } else {
        in_vertex = false;
        in_face = tokens[1] == "face";
        if (in_face) {
          if (seen_face) fail(core::Status::DecodeError, op, "duplicate face element");
          seen_face = true;
          std::istringstream count(tokens[2]);
          face_count = native_count(count, op, 20000000);
        } else if (tokens[2] != "0")
          fail(core::Status::Unsupported, op, "unsupported nonempty PLY element");
      }
      continue;
    }
    if (tokens[0] == "property") {
      if (in_face) {
        if (face_property || tokens.size() != 5 || tokens[1] != "list" ||
            (tokens[4] != "vertex_indices" && tokens[4] != "vertex_index"))
          fail(core::Status::Unsupported, op, "faces require one vertex_indices list property");
        face_size_type = parse_ply_type(tokens[2], op);
        face_index_type = parse_ply_type(tokens[3], op);
        if (face_size_type == PlyType::Float32 || face_size_type == PlyType::Float64 ||
            face_index_type == PlyType::Float32 || face_index_type == PlyType::Float64)
          fail(core::Status::DecodeError, op, "face indices must use integer types");
        face_property = true;
        continue;
      }
      if (!in_vertex) continue;
      if (tokens.size() >= 2 && tokens[1] == "list") {
        fail(core::Status::DecodeError, op, "list vertex properties are not supported");
      }
      if (tokens.size() < 3) fail(core::Status::DecodeError, op, "invalid PLY property");
      PlyProperty property;
      property.type = parse_ply_type(tokens[1], op);
      property.name = tokens[2];
      for (const auto& existing : vertex_properties)
        if (existing.name == property.name) fail(core::Status::DecodeError, op, "duplicate vertex property");
      if (vertex_properties.size() >= 64) fail(core::Status::LimitExceeded, op, "too many vertex properties");
      vertex_properties.push_back(std::move(property));
      continue;
    }
    if (tokens[0] == "end_header") {
      header_done = true;
      continue;
    }
  }
  if (!seen_format) fail(core::Status::DecodeError, op, "missing PLY format");
  if (!seen_vertex) fail(core::Status::DecodeError, op, "PLY file has no vertex element");

  int ix = -1, iy = -1, iz = -1, ic = -1, inx = -1, iny = -1, inz = -1, isource = -1, icount = -1,
      iobserved = -1, ifilled = -1, ireason = -1, isigma = -1;
  for (int i = 0; i < static_cast<int>(vertex_properties.size()); ++i) {
    const auto& name = vertex_properties[i].name;
    if (name == "x")
      ix = i;
    else if (name == "y")
      iy = i;
    else if (name == "z")
      iz = i;
    else if (name == "confidence" || name == "intensity" || name == "scalar_intensity")
      ic = i;
    else if (name == "nx")
      inx = i;
    else if (name == "ny")
      iny = i;
    else if (name == "nz")
      inz = i;
    else if (name == "source_index")
      isource = i;
    else if (name == "voxel_count")
      icount = i;
    else if (name == "observed_valid")
      iobserved = i;
    else if (name == "filled")
      ifilled = i;
    else if (name == "quality_reason")
      ireason = i;
    else if (name == "depth_uncertainty")
      isigma = i;
  }
  if (ix < 0 || iy < 0 || iz < 0) fail(core::Status::DecodeError, op, "PLY vertex must have x, y and z");
  if ((inx >= 0 || iny >= 0 || inz >= 0) && (inx < 0 || iny < 0 || inz < 0))
    fail(core::Status::DecodeError, op, "incomplete PLY normals");
  if ((iobserved >= 0 || ifilled >= 0 || ireason >= 0) && (iobserved < 0 || ifilled < 0 || ireason < 0))
    fail(core::Status::DecodeError, op, "incomplete PLY reconstruction quality");
  if (face_count && !face_property) fail(core::Status::DecodeError, op, "missing face property");
  auto binary_value = [&](PlyType type) {
    std::array<uint8_t, 8> bytes{};
    const auto size = ply_type_size(type);
    in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
    if (!in) fail(core::Status::DecodeError, op, "truncated binary PLY");
    if constexpr (std::endian::native == std::endian::big) std::reverse(bytes.begin(), bytes.begin() + size);
    return ply_type_to_double(type, bytes.data());
  };
  auto attribute_index = [&](double value, bool positive) {
    if (!std::isfinite(value) || value < (positive ? 1. : 0.) || value > 9007199254740991. ||
        std::floor(value) != value)
      fail(core::Status::DecodeError, op, "invalid integer point attribute");
    return static_cast<size_t>(value);
  };
  ObjectModel3DData model;
  std::vector<double> values(vertex_properties.size());
  for (size_t i = 0; i < vertex_count; ++i) {
    if (format == PlyFormat::Ascii) {
      const auto tokens = split(next_line(in, op));
      if (tokens.size() != values.size()) fail(core::Status::DecodeError, op, "invalid PLY vertex record");
      for (size_t j = 0; j < values.size(); ++j) values[j] = ascii_number(tokens[j], op);
    } else
      for (size_t j = 0; j < values.size(); ++j) values[j] = binary_value(vertex_properties[j].type);
    Point3d point{values[ix] * scale, values[iy] * scale, values[iz] * scale};
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      fail(core::Status::DecodeError, op, "nonfinite PLY coordinates");
    model.points.push_back(point);
    if (ic >= 0) {
      const double v = values[ic];
      if (!std::isfinite(v) || !std::isfinite(static_cast<float>(v)))
        fail(core::Status::DecodeError, op, "nonfinite confidence");
      model.confidence.push_back(static_cast<float>(v));
    }
    if (inx >= 0) {
      const Point3d n{values[inx], values[iny], values[inz]};
      if (!std::isfinite(n.x) || !std::isfinite(n.y) || !std::isfinite(n.z))
        fail(core::Status::DecodeError, op, "nonfinite normal");
      model.normals.push_back(n);
    }
    if (isource >= 0) model.source_indices.push_back(attribute_index(values[isource], false));
    if (icount >= 0) model.voxel_counts.push_back(attribute_index(values[icount], true));
    if (iobserved >= 0) {
      const auto observed = attribute_index(values[iobserved], false),
                 filled = attribute_index(values[ifilled], false),
                 reason = attribute_index(values[ireason], false);
      if (observed > 1 || filled > 1 || reason > 511)
        fail(core::Status::DecodeError, op, "invalid PLY quality flags");
      model.observed_valid.push_back(static_cast<uint8_t>(observed));
      model.filled.push_back(static_cast<uint8_t>(filled));
      model.quality_reason.push_back(static_cast<uint16_t>(reason));
    }
    if (isigma >= 0) {
      const double sigma = values[isigma] * scale;
      if (!std::isnan(sigma) &&
          (!std::isfinite(sigma) || sigma < 0 || !std::isfinite(static_cast<float>(sigma))))
        fail(core::Status::DecodeError, op, "invalid PLY depth uncertainty");
      model.depth_uncertainty.push_back(static_cast<float>(sigma));
    }
  }
  for (size_t i = 0; i < face_count; ++i) {
    double count = 0;
    std::array<double, 3> indices{};
    if (format == PlyFormat::Ascii) {
      const auto tokens = split(next_line(in, op));
      if (tokens.empty()) fail(core::Status::DecodeError, op, "truncated face");
      count = ascii_number(tokens[0], op);
      if (count != 3) fail(core::Status::Unsupported, op, "only triangle PLY faces are supported");
      if (tokens.size() != 4) fail(core::Status::DecodeError, op, "invalid triangle record");
      for (size_t j = 0; j < 3; ++j) indices[j] = ascii_number(tokens[j + 1], op);
    } else {
      count = binary_value(face_size_type);
      if (count != 3) fail(core::Status::Unsupported, op, "only triangle PLY faces are supported");
      for (auto& index : indices) index = binary_value(face_index_type);
    }
    std::array<uint32_t, 3> triangle{};
    for (size_t j = 0; j < 3; ++j) {
      const double index = indices[j];
      if (!std::isfinite(index) || std::floor(index) != index || index < 0 || index >= vertex_count)
        fail(core::Status::DecodeError, op, "triangle index out of range");
      triangle[j] = static_cast<uint32_t>(index);
    }
    model.triangles.push_back(triangle);
  }
  validate_object_model_3d(model);
  return model;
}

}  // namespace mv::reconstruction
