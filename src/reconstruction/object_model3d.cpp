#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/coordinate_selection.hpp>
#include <reconstruction/selection_avx2.hpp>
#include <runtime/cpu/cpu_features.hpp>
#include <span>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#if !defined(_M_ARM64EC) && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h>
#define MVKIT_XYZ_SSE2 1
#else
#define MVKIT_XYZ_SSE2 0
#endif

#include <calibration/hom_mat3d.hpp>
#include <core/exception.hpp>
#include <core/numeric_predicates.hpp>
#include <core/region.hpp>
#include <cstring>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/execution/execution_context.hpp>

namespace mv::reconstruction {
namespace {

[[noreturn]] void fail(core::Status status, const char* op, const char* message) {
  throw core::Exception(status, op, message);
}

void fail_value(const char* op, const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                        message);
}

bool quality_is_nan(float value) noexcept {
  if constexpr (sizeof(float) == sizeof(std::uint32_t) && std::numeric_limits<float>::is_iec559)
    return (std::bit_cast<std::uint32_t>(value) & 0x7fffffffU) > 0x7f800000U;
  else
    return std::isnan(value);
}

bool point_valid(const ObjectModel3DData& model, std::size_t index) {
  return model.valid.empty() || model.valid[index] != 0U;
}

double numeric_at(const core::Tuple& tuple, std::size_t index, const char* op, const char* parameter) {
  try {
    const double value = tuple.as_real64(index);
    if (!mv::core::is_finite(value)) fail_value(op, parameter, "coordinates must be finite");
    return value;
  } catch (const core::Exception& error) {
    if (error.status() == core::Status::TypeMismatch) {
      throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidType, op, parameter,
                            static_cast<std::int64_t>(index), "coordinates must be numeric");
    }
    throw;
  }
}

Point3d transform_point(const std::array<double, 12>& m, const Point3d& p) {
  return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
          m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
}

std::array<double, 9> inverse_transpose_linear(const std::array<double, 12>& m) {
  const double a00 = m[0];
  const double a01 = m[1];
  const double a02 = m[2];
  const double a10 = m[4];
  const double a11 = m[5];
  const double a12 = m[6];
  const double a20 = m[8];
  const double a21 = m[9];
  const double a22 = m[10];
  const double det =
      a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
  const double i00 = (a11 * a22 - a12 * a21) / det;
  const double i01 = (a02 * a21 - a01 * a22) / det;
  const double i02 = (a01 * a12 - a02 * a11) / det;
  const double i10 = (a12 * a20 - a10 * a22) / det;
  const double i11 = (a00 * a22 - a02 * a20) / det;
  const double i12 = (a02 * a10 - a00 * a12) / det;
  const double i20 = (a10 * a21 - a11 * a20) / det;
  const double i21 = (a01 * a20 - a00 * a21) / det;
  const double i22 = (a00 * a11 - a01 * a10) / det;
  return {i00, i10, i20, i01, i11, i21, i02, i12, i22};
}

Point3d transform_normal(const std::array<double, 9>& inv_t, const Point3d& n) {
  Point3d out{inv_t[0] * n.x + inv_t[1] * n.y + inv_t[2] * n.z,
              inv_t[3] * n.x + inv_t[4] * n.y + inv_t[5] * n.z,
              inv_t[6] * n.x + inv_t[7] * n.y + inv_t[8] * n.z};
  const double length = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
  if (n.x == 0 && n.y == 0 && n.z == 0) return {};
  if (!(length > 0.0) || !mv::core::is_finite(length)) {
    fail(core::Status::InvalidArgument, "affine_trans_object_model_3d", "transformed normal is degenerate");
  }
  out.x /= length;
  out.y /= length;
  out.z /= length;
  return out;
}

ObjectModel3DData transform_model(const ObjectModel3DData& model, const std::array<double, 12>& m) {
  ObjectModel3DData out{model, ObjectModel3DData::CopyMode::Transform};
  // 源相机 Z 的标量 sigma 不能按完整协方差旋转。
  out.depth_uncertainty.clear();
  decltype(out.points) coordinates{PlainPayloadAllocator<Point3d>{mv::runtime::persist_resource(), true}};
  coordinates.resize(model.points.size());
  const bool has_normals = !out.normals.empty();
  std::array<double, 9> inv_t{};
  if (has_normals || out.primitive_kind == PrimitiveKind::Plane) {
    inv_t = inverse_transpose_linear(m);
  }
  std::size_t first = 0;
#if MVKIT_RECONSTRUCTION_HAS_AVX2
  const auto& cpu = mv::runtime::cpu_features();
  if (!has_normals &&
      mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                  kSelectionKernelRequirement))
    first = transform_points_avx2(model.points, coordinates, out.valid.empty() ? nullptr : out.valid.data(),
                                  m.data());
#endif
  for (std::size_t i = first; i < model.points.size(); ++i) {
    if (!point_valid(model, i)) {
      coordinates[i] = model.points[i];
      continue;
    }
    coordinates[i] = transform_point(m, model.points[i]);
    if (!mv::core::is_finite(coordinates[i].x) || !mv::core::is_finite(coordinates[i].y) ||
        !mv::core::is_finite(coordinates[i].z)) {
      fail(core::Status::InvalidArgument, "affine_trans_object_model_3d",
           "transformed coordinates are not finite");
    }
    if (has_normals) {
      const auto& n = out.normals[i];
      if (n.x == 0.0 && n.y == 0.0 && n.z == 0.0) continue;
      out.normals[i] = transform_normal(inv_t, n);
    }
  }
  out.points = std::move(coordinates);
  if (out.primitive_kind == PrimitiveKind::Plane) {
    const Point3d old{out.plane[0], out.plane[1], out.plane[2]};
    const Point3d raw{inv_t[0] * old.x + inv_t[1] * old.y + inv_t[2] * old.z,
                      inv_t[3] * old.x + inv_t[4] * old.y + inv_t[5] * old.z,
                      inv_t[6] * old.x + inv_t[7] * old.y + inv_t[8] * old.z};
    const double normal_scale = std::hypot(raw.x, raw.y, raw.z);
    const Point3d n = transform_normal(inv_t, old);
    const Point3d t{m[3], m[7], m[11]};
    out.plane = {n.x, n.y, n.z, out.plane[3] / normal_scale - (n.x * t.x + n.y * t.y + n.z * t.z)};
    out.primitive_rms /= normal_scale;
  }
  if (out.primitive_kind == PrimitiveKind::Sphere || out.primitive_kind == PrimitiveKind::Cylinder) {
    const Point3d x{m[0], m[4], m[8]}, y{m[1], m[5], m[9]}, z{m[2], m[6], m[10]};
    const auto dot = [](Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const double scale2 = dot(x, x), tol = 1e-10 * scale2;
    if (std::abs(dot(y, y) - scale2) > tol || std::abs(dot(z, z) - scale2) > tol ||
        std::abs(dot(x, y)) > tol || std::abs(dot(x, z)) > tol || std::abs(dot(y, z)) > tol) {
      // 一般仿射像为椭球/椭圆柱，不属于此原语族。
      out.primitive_kind = PrimitiveKind::None;
      out.primitive = {};
      out.primitive_rms = 0;
      out.primitive_num_inliers = 0;
      out.fit_converged = false;
      out.fit_iterations = 0;
      out.information_ratio = 0;
      out.primitive_support.reset();
    } else {
      const double scale = std::sqrt(scale2);
      const auto p = transform_point(m, {out.primitive[0], out.primitive[1], out.primitive[2]});
      out.primitive[0] = p.x;
      out.primitive[1] = p.y;
      out.primitive[2] = p.z;
      if (out.primitive_kind == PrimitiveKind::Sphere)
        out.primitive[3] *= scale;
      else {
        const Point3d a{out.primitive[3], out.primitive[4], out.primitive[5]};
        Point3d direction{(m[0] * a.x + m[1] * a.y + m[2] * a.z) / scale,
                          (m[4] * a.x + m[5] * a.y + m[6] * a.z) / scale,
                          (m[8] * a.x + m[9] * a.y + m[10] * a.z) / scale};
        if (!out.primitive_support) {
          if (direction.z < 0 ||
              (direction.z == 0 && (direction.y < 0 || (direction.y == 0 && direction.x < 0))))
            direction = {-direction.x, -direction.y, -direction.z};
          const double t = dot(p, direction);
          out.primitive[0] -= t * direction.x;
          out.primitive[1] -= t * direction.y;
          out.primitive[2] -= t * direction.z;
        }
        out.primitive[3] = direction.x;
        out.primitive[4] = direction.y;
        out.primitive[5] = direction.z;
        out.primitive[6] *= scale;
      }
      out.primitive_rms *= scale;
    }
  }
  if (out.primitive_support) {
    auto support = std::allocate_shared<PrimitiveSupport>(
        std::pmr::polymorphic_allocator<PrimitiveSupport>{runtime::persist_resource()},
        *out.primitive_support);
    support->origin = transform_point(m, support->origin);
    const auto linear = [&](Point3d p) {
      return Point3d{m[0] * p.x + m[1] * p.y + m[2] * p.z, m[4] * p.x + m[5] * p.y + m[6] * p.z,
                     m[8] * p.x + m[9] * p.y + m[10] * p.z};
    };
    const auto normalize = [](Point3d p) {
      const double n = std::hypot(p.x, p.y, p.z);
      return Point3d{p.x / n, p.y / n, p.z / n};
    };
    const auto dot = [](Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    support->axis_z = out.primitive_kind == PrimitiveKind::Plane
                          ? Point3d{out.plane[0], out.plane[1], out.plane[2]}
                          : normalize(linear(support->axis_z));
    auto u = linear(support->axis_x);
    const double projected = dot(u, support->axis_z);
    support->axis_x = normalize({u.x - projected * support->axis_z.x, u.y - projected * support->axis_z.y,
                                 u.z - projected * support->axis_z.z});
    const auto a = support->axis_z, b = support->axis_x;
    support->axis_y = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    for (auto& p : support->plane_boundary) p = transform_point(m, p);
    const double determinant = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) +
                               m[2] * (m[4] * m[9] - m[5] * m[8]);
    if (determinant < 0) std::reverse(support->plane_boundary.begin(), support->plane_boundary.end());
    if (out.primitive_kind == PrimitiveKind::Cylinder) {
      const auto axis = model.primitive_support->axis_z;
      const auto transformed = linear(axis);
      const double scale = std::hypot(transformed.x, transformed.y, transformed.z);
      for (auto& extent : support->cylinder_extent) extent *= scale;
    }
    out.primitive_support = std::move(support);
  }
  return out;
}

struct VoxelKey {
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t z{};
  friend bool operator==(const VoxelKey&, const VoxelKey&) = default;
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    std::size_t h = std::hash<std::int64_t>{}(key.x);
    h ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

std::int64_t voxel_index(double coord, double distance, const char* op) {
  const double scaled = std::floor(coord / distance);
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int64_t>::min());
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (!mv::core::is_finite(scaled) || scaled < kMin || scaled >= kMax) {
    fail(core::Status::InvalidArgument, op, "voxel index is out of int64 range");
  }
  return static_cast<std::int64_t>(scaled);
}

}  // namespace

void validate_object_model_3d(const ObjectModel3DData& model) {
  const auto attributes = model.point_attributes.Read();
  if (attributes)
    for (const auto& [name, values] : attributes->values)
      if (values.size() != model.points.size())
        fail(core::Status::InvalidArgument, "validate_object_model_3d",
             "extended point attribute length mismatch");
  if ((!model.valid.empty() && model.valid.size() != model.points.size()) ||
      (!model.confidence.empty() && model.confidence.size() != model.points.size()))
    fail(core::Status::InvalidArgument, "validate_object_model_3d", "point attribute length mismatch");
  for (auto size : {model.observed_valid.size(), model.filled.size(), model.quality_reason.size(),
                    model.depth_uncertainty.size()})
    if (size && size != model.points.size())
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "quality attribute length mismatch");
  for (float sigma : model.depth_uncertainty)
    if (!quality_is_nan(sigma) && (!mv::core::is_finite(sigma) || sigma < 0))
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "invalid depth uncertainty");
  if (!model.observed_valid.empty() || !model.filled.empty() || !model.quality_reason.empty()) {
    if (model.observed_valid.size() != model.points.size() || model.filled.size() != model.points.size() ||
        model.quality_reason.size() != model.points.size())
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "incomplete reconstruction quality");
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      const bool valid = model.valid.empty() || model.valid[i] != 0;
      if (model.observed_valid[i] > 1 || model.filled[i] > 1 || model.quality_reason[i] > 511 ||
          (model.observed_valid[i] && (!valid || model.filled[i] || model.quality_reason[i])) ||
          (model.filled[i] && !valid) ||
          (model.filled[i] && !model.confidence.empty() && model.confidence[i] != 0) ||
          (!model.depth_uncertainty.empty() && (!valid || model.filled[i]) &&
           !quality_is_nan(model.depth_uncertainty[i])))
        fail(core::Status::InvalidArgument, "validate_object_model_3d",
             "inconsistent reconstruction quality");
    }
  }
  const bool organized = model.width != 0 || model.height != 0;
  if (organized) {
    if (model.width == 0 || model.height == 0 || model.width * model.height != model.points.size()) {
      throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                            "organized dimensions must match point count");
    }
  }
  if (!model.valid.empty() && model.valid.size() != model.points.size()) {
    throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                          "valid mask must match point count");
  }
  if (!model.confidence.empty() && model.confidence.size() != model.points.size()) {
    throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                          "confidence must match point count");
  }
  if (!model.normals.empty() && model.normals.size() != model.points.size()) {
    throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                          "normals must match point count");
  }
  if ((!model.source_indices.empty() && model.source_indices.size() != model.points.size()) ||
      (!model.voxel_counts.empty() && model.voxel_counts.size() != model.points.size()))
    fail(core::Status::InvalidArgument, "validate_object_model_3d",
         "point attributes must match point count");
  for (auto i : model.input_to_sample)
    if (i < -1 || (i >= 0 && static_cast<size_t>(i) >= model.points.size()))
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "invalid voxel mapping");
  if (model.primitive_kind == PrimitiveKind::Sphere || model.primitive_kind == PrimitiveKind::Cylinder) {
    for (double v : model.primitive)
      if (!std::isfinite(v))
        fail(core::Status::InvalidArgument, "validate_object_model_3d", "nonfinite primitive");
    const auto& p = model.primitive;
    if (!(p[model.primitive_kind == PrimitiveKind::Sphere ? 3 : 6] > 0))
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "radius must be positive");
    if (model.primitive_kind == PrimitiveKind::Cylinder &&
        std::abs(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] - 1) > 1e-8)
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "cylinder axis must be unit length");
  }
  for (const auto& triangle : model.triangles) {
    for (std::uint32_t index : triangle) {
      if (index >= model.points.size()) {
        throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                              "triangle index is out of range");
      }
    }
  }
  for (const auto& line : model.lines) {
    if (line[0] >= model.points.size() || line[1] >= model.points.size())
      throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                            "line index is out of range");
  }
  if (model.primitive_kind == PrimitiveKind::Plane) {
    const double nx = model.plane[0];
    const double ny = model.plane[1];
    const double nz = model.plane[2];
    const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!mv::core::is_finite(length) || length <= 0.0 || !mv::core::is_finite(model.plane[3])) {
      throw core::Exception(core::Status::InvalidArgument, "validate_object_model_3d",
                            "plane primitive is degenerate");
    }
  }
  if (model.primitive_support) {
    const auto& support = *model.primitive_support;
    if (model.primitive_kind == PrimitiveKind::None || !model.payloads_use_persist_resource())
      fail(core::Status::InvalidArgument, "validate_object_model_3d",
           "invalid primitive support ownership or kind");
    const auto finite = [](Point3d p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    const auto dot = [](Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    if (!finite(support.origin) || !finite(support.axis_x) || !finite(support.axis_y) ||
        !finite(support.axis_z) || std::abs(dot(support.axis_x, support.axis_x) - 1) > 1e-8 ||
        std::abs(dot(support.axis_y, support.axis_y) - 1) > 1e-8 ||
        std::abs(dot(support.axis_z, support.axis_z) - 1) > 1e-8 ||
        std::abs(dot(support.axis_x, support.axis_y)) > 1e-8 ||
        std::abs(dot(support.axis_x, support.axis_z)) > 1e-8 ||
        std::abs(dot(support.axis_y, support.axis_z)) > 1e-8)
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "invalid primitive frame");
    if (model.primitive_kind == PrimitiveKind::Plane) {
      if (support.plane_boundary.size() < 3)
        fail(core::Status::InvalidArgument, "validate_object_model_3d", "plane support needs a polygon");
      for (auto p : support.plane_boundary)
        if (!finite(p) ||
            std::abs(model.plane[0] * p.x + model.plane[1] * p.y + model.plane[2] * p.z + model.plane[3]) >
                1e-7 * (1 + std::abs(model.plane[3]) + std::hypot(p.x, p.y, p.z)))
          fail(core::Status::InvalidArgument, "validate_object_model_3d", "invalid coplanar support point");
    } else if (!support.plane_boundary.empty())
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "non-plane has polygon support");
    if (model.primitive_kind == PrimitiveKind::Cylinder &&
        (!std::isfinite(support.cylinder_extent[0]) || !std::isfinite(support.cylinder_extent[1]) ||
         support.cylinder_extent[0] > support.cylinder_extent[1]))
      fail(core::Status::InvalidArgument, "validate_object_model_3d", "invalid cylinder extent");
  }
}

ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model,
                                          std::span<const std::size_t> indices) {
  return compact_object_model_3d(model, indices, model.point_attributes.Read());
}
ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model,
                                          std::span<const std::size_t> indices,
                                          const PointAttributes::Snapshot& attributes) {
  ObjectModel3DData out;
  const bool has_confidence = !model.confidence.empty();
  out.point_attributes = PointAttributes::Select(attributes, indices);
  const bool has_normals = !model.normals.empty();
  out.mapping_width = model.mapping_width ? model.mapping_width : model.width;
  out.mapping_height = model.mapping_height ? model.mapping_height : model.height;
  out.triangles_are_polygons = model.triangles_are_polygons;
  if(!has_confidence && !has_normals && model.source_indices.empty() && model.voxel_counts.empty() &&
     model.observed_valid.empty() && model.filled.empty() && model.quality_reason.empty() &&
     model.depth_uncertainty.empty() && !out.mapping_width && model.triangles.empty() && model.lines.empty()) {
    // 纯 XYZ 载荷不逐点重复检查不存在的伴随数组；扩展属性已在上方独立选出。
    out.points.resize(indices.size());
    for(std::size_t i=0;i<indices.size();++i)out.points[i]=model.points[indices[i]];
    return out;
  }
  out.points.reserve(indices.size());
  if (has_confidence) out.confidence.reserve(indices.size());
  if (has_normals) out.normals.reserve(indices.size());
  if (!model.source_indices.empty()) out.source_indices.reserve(indices.size());
  if (!model.voxel_counts.empty()) out.voxel_counts.reserve(indices.size());
  for (const std::size_t i : indices) {
    out.points.push_back(model.points[i]);
    if (has_confidence) out.confidence.push_back(model.confidence[i]);
    if (has_normals) out.normals.push_back(model.normals[i]);
    if (!model.source_indices.empty() || !model.observed_valid.empty() || out.mapping_width)
      out.source_indices.push_back(model.source_indices.empty() ? i : model.source_indices[i]);
    if (!model.voxel_counts.empty()) out.voxel_counts.push_back(model.voxel_counts[i]);
    if (!model.observed_valid.empty()) out.observed_valid.push_back(model.observed_valid[i]);
    if (!model.filled.empty()) out.filled.push_back(model.filled[i]);
    if (!model.quality_reason.empty()) out.quality_reason.push_back(model.quality_reason[i]);
    if (!model.depth_uncertainty.empty()) out.depth_uncertainty.push_back(model.depth_uncertainty[i]);
  }
  if (!model.triangles.empty() || !model.lines.empty()) {
    std::vector<std::size_t> remap(model.points.size(), std::numeric_limits<std::size_t>::max());
    for (std::size_t i = 0; i < indices.size(); ++i) remap[indices[i]] = i;
    for (const auto& triangle : model.triangles)
      if (remap[triangle[0]] < indices.size() && remap[triangle[1]] < indices.size() &&
          remap[triangle[2]] < indices.size())
        out.triangles.push_back({static_cast<std::uint32_t>(remap[triangle[0]]),
                                 static_cast<std::uint32_t>(remap[triangle[1]]),
                                 static_cast<std::uint32_t>(remap[triangle[2]])});
    for (const auto& line : model.lines)
      if (remap[line[0]] < indices.size() && remap[line[1]] < indices.size())
        out.lines.push_back(
            {static_cast<std::uint32_t>(remap[line[0]]), static_cast<std::uint32_t>(remap[line[1]])});
  }
  return out;
}

ObjectModel3DData compact_coordinate_points(const ObjectModel3DData& model,std::size_t axis,double lower,double upper,
                                          const PointAttributes::Snapshot& attributes) {
#if MVKIT_RECONSTRUCTION_HAS_AVX2
  const auto& cpu=runtime::cpu_features();
  if(model.points.size()>=64 && !attributes && model.confidence.empty() && model.normals.empty() &&
     model.source_indices.empty() && model.voxel_counts.empty() && model.observed_valid.empty() &&
     model.filled.empty() && model.quality_reason.empty() && model.depth_uncertainty.empty() &&
     !model.width && !model.mapping_width && model.triangles.empty() && model.lines.empty() &&
     runtime::kernel_allowed(runtime::effective_capabilities(cpu.selection(),cpu.hardware()),kSelectionKernelRequirement)) {
    ObjectModel3DData result;result.triangles_are_polygons=model.triangles_are_polygons;
    decltype(result.points) points{PlainPayloadAllocator<Point3d>{runtime::persist_resource(),true}};
    points.resize(model.points.size());
    // 现有内核按连续选中游程直接复制完整点；所有实际输出均已覆盖，再发布最终长度。
    const auto selected=coordinate_runs_avx2(model.points,model.valid.empty()?nullptr:model.valid.data(),
                                            axis,lower,upper,0,points.data());
    points.resize(selected.count);result.points=std::move(points);return result;
  }
#endif
  std::vector<std::size_t> selected;selected.reserve(model.points.size());
  for(std::size_t i=0;i<model.points.size();++i) {
    const auto& point=model.points[i];const auto value=axis==0?point.x:axis==1?point.y:point.z;
    if(point_valid(model,i) && value>=lower && value<=upper)selected.push_back(i);
  }
  return compact_object_model_3d(model,selected,attributes);
}

ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model, const std::uint8_t* keep) {
  std::size_t selected_count = 0;
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (keep[i] != 0U) ++selected_count;
  }
  runtime::ExecutionContext context;
  std::pmr::vector<std::size_t> indices(selected_count, context.scratch_resource());
  std::size_t w = 0;
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (keep[i] != 0U) indices[w++] = i;
  }
  return compact_object_model_3d(model, std::span<const std::size_t>{indices.data(), indices.size()});
}

XyzImages object_model_3d_to_xyz(const ObjectModel3DData& model) {
  validate_object_model_3d(model);
  if (model.width == 0 || model.height == 0) {
    throw core::Exception(core::Status::InvalidState, "object_model_3d_to_xyz",
                          "object model is not organized");
  }

  XyzImages out{
      core::Image::create_uninitialized(model.width, model.height, std::array{core::PixelType::Float32}),
      core::Image::create_uninitialized(model.width, model.height, std::array{core::PixelType::Float32}),
      core::Image::create_uninitialized(model.width, model.height, std::array{core::PixelType::Float32}),
  };
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto* x_base = static_cast<std::byte*>(out.x.mutable_data());
  auto* y_base = static_cast<std::byte*>(out.y.mutable_data());
  auto* z_base = static_cast<std::byte*>(out.z.mutable_data());
  bool vectorized = false;
#if MVKIT_RECONSTRUCTION_HAS_AVX2
  const auto& cpu = mv::runtime::cpu_features();
  vectorized = mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()), kSelectionKernelRequirement);
#endif
  for (std::size_t row = 0; row < model.height; ++row) {
    auto* x = reinterpret_cast<float*>(x_base + row * out.x.stride());
    auto* y = reinterpret_cast<float*>(y_base + row * out.y.stride());
    auto* z = reinterpret_cast<float*>(z_base + row * out.z.stride());
    std::size_t col = 0;
#if MVKIT_RECONSTRUCTION_HAS_AVX2
    if (vectorized)
      col = export_xyz_avx2(std::span<const Point3d>{model.points.data() + row * model.width, model.width},
                            model.valid.empty() ? nullptr : model.valid.data() + row * model.width, x, y, z);
#endif
    for (; col < model.width; ++col) {
      const std::size_t i = row * model.width + col;
      const bool valid = model.valid.empty() || model.valid[i] != 0;
      if (!valid) {
        x[col] = nan;
        y[col] = nan;
        z[col] = nan;
      } else {
        x[col] = static_cast<float>(model.points[i].x);
        y[col] = static_cast<float>(model.points[i].y);
        z[col] = static_cast<float>(model.points[i].z);
      }
    }
  }
  return out;
}

ObjectModel3DData gen_object_model_3d_from_points(const core::Tuple& x, const core::Tuple& y,
                                                  const core::Tuple& z) {
  constexpr auto op = "gen_object_model_3d_from_points";
  if (x.size() != y.size() || y.size() != z.size()) {
    throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidCardinality, op, "X", -1,
                          "X, Y and Z must have equal length");
  }
  ObjectModel3DData model;
  model.points.resize(x.size());
  if (x.kind() == core::TupleKind::Real64 && y.kind() == core::TupleKind::Real64 &&
      z.kind() == core::TupleKind::Real64) {
    const auto xs = x.real64_span();
    const auto ys = y.real64_span();
    const auto zs = z.real64_span();
    for (std::size_t i = 0; i < xs.size(); ++i) {
      if (!mv::core::is_finite(xs[i])) fail_value(op, "X", "coordinates must be finite");
      if (!mv::core::is_finite(ys[i])) fail_value(op, "Y", "coordinates must be finite");
      if (!mv::core::is_finite(zs[i])) fail_value(op, "Z", "coordinates must be finite");
      model.points[i] = {xs[i], ys[i], zs[i]};
    }
    validate_object_model_3d(model);
    return model;
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    model.points[i] = {numeric_at(x, i, op, "X"), numeric_at(y, i, op, "Y"), numeric_at(z, i, op, "Z")};
  }
  validate_object_model_3d(model);
  return model;
}

ObjectModel3DData xyz_to_object_model_3d(const core::Image& x, const core::Image& y, const core::Image& z) {
  constexpr auto op = "xyz_to_object_model_3d";
  if (x.empty() || y.empty() || z.empty()) {
    fail(core::Status::InvalidArgument, op, "X, Y and Z images must not be empty");
  }
  if (x.width() != y.width() || x.height() != y.height() || x.width() != z.width() ||
      x.height() != z.height()) {
    fail(core::Status::InvalidArgument, op, "X, Y and Z images must have the same size");
  }
  if (x.channel_count() < 1 || y.channel_count() < 1 || z.channel_count() < 1) {
    fail(core::Status::InvalidArgument, op, "X, Y and Z must have at least one channel");
  }

  ObjectModel3DData model;
  model.width = x.width();
  model.height = x.height();
  const std::size_t count = model.width * model.height;
  // 仅完整域 Float32 写入能覆盖全部栅格槽，allocator 模式保持局部，无效槽显式置零。
  const bool overwrite_full_float = x.pixel_type() == core::PixelType::Float32 &&
                                    y.pixel_type() == core::PixelType::Float32 &&
                                    z.pixel_type() == core::PixelType::Float32 && !x.has_reduced_domain() &&
                                    !y.has_reduced_domain() && !z.has_reduced_domain();
  std::vector<Point3d, PlainPayloadAllocator<Point3d>> overwrite{
      PlainPayloadAllocator<Point3d>{mv::runtime::persist_resource(), true}};
  if (overwrite_full_float)
    overwrite.resize(count);
  else
    model.points.resize(count);
  auto& points = overwrite_full_float ? overwrite : model.points;
  model.valid.assign(count, 0U);
  const auto domain = x.domain().intersect(y.domain()).intersect(z.domain());
  const auto copy_plane = [&](const core::Image& image, auto store) {
    const auto stride = image.stride();
    const auto* data = static_cast<const std::byte*>(image.data());
    mv::imageproc::dispatch_scalar(image.pixel_type(), [&](auto tag) {
      using T = typename decltype(tag)::type;
      for (const auto& run : domain.runs()) {
        const auto row = static_cast<std::size_t>(run.row);
        for (auto col = std::int64_t(run.col_begin); col <= run.col_end; ++col) {
          T value;
          std::memcpy(&value, data + row * stride + col * sizeof(T), sizeof(T));
          store(points[row * model.width + col], double(value));
        }
      }
    });
  };
#if MVKIT_XYZ_SSE2
  const auto& cpu = mv::runtime::cpu_features();
  const bool sse2 = mv::runtime::kernel_allowed(
      mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()), {true, false, false, false});
  if (sse2 && x.pixel_type() == core::PixelType::Float32 && y.pixel_type() == core::PixelType::Float32 &&
      z.pixel_type() == core::PixelType::Float32) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559 &&
                  sizeof(Point3d) == 3 * sizeof(double) && std::is_standard_layout_v<Point3d>);
    const auto* xd = static_cast<const std::byte*>(x.data());
    const auto* yd = static_cast<const std::byte*>(y.data());
    const auto* zd = static_cast<const std::byte*>(z.data());
    const auto xs = x.stride(), ys = y.stride(), zs = z.stride();
    // 精确读取两个 Float，可疑 IEEE 点对重放标量路径。
    const auto finite = [&](__m128i v) {
      return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmplt_epi32(_mm_and_si128(v, _mm_set1_epi32(0x7fffffff)),
                                                               _mm_set1_epi32(0x7f800000)))) &
              3) == 3;
    };
    for (const auto& run : domain.runs()) {
      const auto* xr = reinterpret_cast<const float*>(xd + run.row * xs);
      const auto* yr = reinterpret_cast<const float*>(yd + run.row * ys);
      const auto* zr = reinterpret_cast<const float*>(zd + run.row * zs);
      auto col = std::int64_t(run.col_begin);
      for (; col + 1 <= run.col_end; col += 2) {
        const auto a = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(xr + col)),
                   b = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(yr + col)),
                   c = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(zr + col));
        const auto i = std::size_t(run.row) * model.width + col;
        if (finite(a) && finite(b) && finite(c)) {
          const auto ax = _mm_cvtps_pd(_mm_castsi128_ps(a)), by = _mm_cvtps_pd(_mm_castsi128_ps(b)),
                     cz = _mm_cvtps_pd(_mm_castsi128_ps(c));
          auto* out = reinterpret_cast<double*>(points.data() + i);
          _mm_storeu_pd(out, _mm_unpacklo_pd(ax, by));
          _mm_storeu_pd(out + 2, _mm_shuffle_pd(cz, ax, 2));
          _mm_storeu_pd(out + 4, _mm_unpackhi_pd(by, cz));
          model.valid[i] = 1U;
          model.valid[i + 1] = 1U;
        } else
          for (int k = 0; k < 2; ++k) {
            const double px = xr[col + k], py = yr[col + k], pz = zr[col + k];
            if (core::is_finite(px) && core::is_finite(py) && core::is_finite(pz)) {
              points[i + k] = {px, py, pz};
              model.valid[i + k] = 1U;
            } else
              points[i + k] = {};
          }
      }
      for (; col <= run.col_end; ++col) {
        const auto i = std::size_t(run.row) * model.width + col;
        const double a = xr[col], b = yr[col], c = zr[col];
        if (core::is_finite(a) && core::is_finite(b) && core::is_finite(c)) {
          points[i] = {a, b, c};
          model.valid[i] = 1U;
        } else
          points[i] = {};
      }
    }
  } else
#endif
      if (x.pixel_type() == y.pixel_type() && x.pixel_type() == z.pixel_type()) {
    const auto* xd = static_cast<const std::byte*>(x.data());
    const auto* yd = static_cast<const std::byte*>(y.data());
    const auto* zd = static_cast<const std::byte*>(z.data());
    const auto xs = x.stride(), ys = y.stride(), zs = z.stride();
    mv::imageproc::dispatch_scalar(x.pixel_type(), [&](auto tag) {
      using T = typename decltype(tag)::type;
      for (const auto& run : domain.runs()) {
        const auto* xr = reinterpret_cast<const T*>(xd + run.row * xs);
        const auto* yr = reinterpret_cast<const T*>(yd + run.row * ys);
        const auto* zr = reinterpret_cast<const T*>(zd + run.row * zs);
        for (auto col = std::int64_t(run.col_begin); col <= run.col_end; ++col) {
          const auto i = std::size_t(run.row) * model.width + col;
          const double a = xr[col], b = yr[col], c = zr[col];
          if (mv::core::is_finite(a) && mv::core::is_finite(b) && mv::core::is_finite(c)) {
            points[i] = {a, b, c};
            model.valid[i] = 1U;
          } else
            points[i] = {};
        }
      }
    });
  } else {
    copy_plane(x, [](auto& p, double v) { p.x = v; });
    copy_plane(y, [](auto& p, double v) { p.y = v; });
    copy_plane(z, [](auto& p, double v) { p.z = v; });
    for (const auto& run : domain.runs())
      for (auto col = std::int64_t(run.col_begin); col <= run.col_end; ++col) {
        const auto i = std::size_t(run.row) * model.width + col;
        auto& p = points[i];
        if (mv::core::is_finite(p.x) && mv::core::is_finite(p.y) && mv::core::is_finite(p.z))
          model.valid[i] = 1U;
        else
          p = {};
      }
  }
  if (overwrite_full_float) model.points = std::move(overwrite);
  validate_object_model_3d(model);
  return model;
}

ObjectModel3DData affine_trans_object_model_3d(const ObjectModel3DData& model, const core::Tuple& hom_mat3d) {
  validate_object_model_3d(model);
  const auto matrix =
      calibration::parse_hom_mat3d(hom_mat3d, "affine_trans_object_model_3d",
                                   !model.normals.empty() || model.primitive_kind == PrimitiveKind::Plane);
  return transform_model(model, matrix);
}

ObjectModel3DData rigid_trans_object_model_3d(const ObjectModel3DData& model, const core::Tuple& pose) {
  validate_object_model_3d(model);
  return transform_model(model, calibration::parse_hom_mat3d(calibration::pose_to_hom_mat3d(pose),
                                                             "rigid_trans_object_model_3d"));
}

ObjectModel3DData select_points_object_model_3d(const ObjectModel3DData& model, std::string_view attrib,
                                                double min_value, double max_value,
                                                std::span<const double> coordinate_column,
                                                std::span<const double> coordinate_ranges) {
  constexpr auto op = "select_points_object_model_3d";
  validate_object_model_3d(model);
  if (!mv::core::is_finite(min_value) || !mv::core::is_finite(max_value)) {
    fail_value(op, "Min", "Min and Max must be finite");
  }
  enum class Attr { X, Y, Z, Score, Observed, Filled, Reason, Sigma };
  Attr kind = Attr::X;
  if (attrib == "point_coord_x") {
    kind = Attr::X;
  } else if (attrib == "point_coord_y") {
    kind = Attr::Y;
  } else if (attrib == "point_coord_z") {
    kind = Attr::Z;
  } else if (attrib == "point_score") {
    kind = Attr::Score;
    if (model.confidence.empty()) {
      fail(core::Status::InvalidState, op, "object model has no confidence");
    }
  } else if (attrib == "observed_valid" || attrib == "filled" || attrib == "quality_reason" ||
             attrib == "depth_uncertainty") {
    if (model.observed_valid.empty() || (attrib == "depth_uncertainty" && model.depth_uncertainty.empty()))
      fail(core::Status::InvalidState, op, "object model has no requested reconstruction quality");
    kind = attrib == "observed_valid"   ? Attr::Observed
           : attrib == "filled"         ? Attr::Filled
           : attrib == "quality_reason" ? Attr::Reason
                                        : Attr::Sigma;
  } else {
    fail_value(op, "Attrib", "unsupported point attribute");
  }

  const auto coordinate_min = std::nextafter(min_value, -std::numeric_limits<double>::infinity());
  const auto coordinate_max = std::nextafter(max_value, std::numeric_limits<double>::infinity());
  const auto total = model.points.size();
  if (!coordinate_column.empty() && coordinate_column.size() != total)
    fail(core::Status::InvalidArgument, op, "coordinate column length mismatch");
  const auto* column = coordinate_column.empty() ? nullptr : coordinate_column.data();
  if (!coordinate_ranges.empty() && (!column || coordinate_ranges.size() != total / 16 * 2))
    fail(core::Status::InvalidArgument, op, "coordinate block range length mismatch");
  const auto* ranges = coordinate_ranges.empty() ? nullptr : coordinate_ranges.data();
  const auto* input_points = model.points.data();
  const auto* input_valid = model.valid.empty() ? nullptr : model.valid.data();
  const auto* input_confidence = model.confidence.empty() ? nullptr : model.confidence.data();
  const auto* input_normals = model.normals.empty() ? nullptr : model.normals.data();
  const auto select = [&](auto value_at, double lower, double upper) {
    const bool valid_range = min_value <= max_value;
    const auto matches = [&](std::size_t i) {
      if (!valid_range || (input_valid && !input_valid[i])) return false;
      const double value = value_at(i);
      return value >= lower && value <= upper;
    };
    if (model.point_attributes.Read()) {
      std::vector<std::size_t> indices;
      for (std::size_t i = 0; i < model.points.size(); ++i)
        if (matches(i)) indices.push_back(i);
      return compact_object_model_3d(model, indices);
    }
    std::size_t count = 0;
    std::vector<std::pair<std::size_t, std::size_t>> runs;
    bool indexed = true;
    constexpr std::size_t max_indexed_runs = 4096;
    bool vectorized = false;
#if MVKIT_RECONSTRUCTION_HAS_AVX2
    const auto& cpu = mv::runtime::cpu_features();
    const bool coordinate_attribute = kind == Attr::X || kind == Attr::Y || kind == Attr::Z;
    if (valid_range && coordinate_attribute &&
        mv::runtime::kernel_allowed(mv::runtime::effective_capabilities(cpu.selection(), cpu.hardware()),
                                    kSelectionKernelRequirement)) {
      if (total >= 262144 && !model.mapping_width && !model.width && !input_confidence && !input_normals &&
          model.source_indices.empty() && model.voxel_counts.empty() && model.observed_valid.empty() &&
          model.filled.empty() && model.quality_reason.empty() && model.depth_uncertainty.empty()) {
        // 源缓存仍热时复制完整游程；上界分配在全部选中点写完前保持私有，缩短后只暴露已初始化前缀。
        ObjectModel3DData out;
        decltype(out.points) buffer{PlainPayloadAllocator<Point3d>{mv::runtime::persist_resource(), true}};
        buffer.resize(total);
        const auto result = coordinate_runs_avx2(model.points, input_valid, static_cast<std::size_t>(kind),
                                                 lower, upper, 0, buffer.data(), column, ranges);
        buffer.resize(result.count);
        out.points = std::move(buffer);
        validate_object_model_3d(out);
        return out;
      }
      auto result = coordinate_runs_avx2(model.points, input_valid, static_cast<std::size_t>(kind), lower,
                                         upper, max_indexed_runs, nullptr, column, ranges);
      count = result.count;
      indexed = result.indexed;
      runs = std::move(result.runs);
      vectorized = true;
    }
#endif
    if (valid_range && !vectorized) {
      for (std::size_t i = 0; i < total;) {
        while (i < total && !matches(i)) ++i;
        const auto first = i;
        while (i < total && matches(i)) ++i;
        if (first == i) break;
        if (runs.size() < max_indexed_runs)
          runs.emplace_back(first, i);
        else
          indexed = false;
        count += i - first;
      }
    }
    ObjectModel3DData out;
    out.mapping_width = model.mapping_width ? model.mapping_width : model.width;
    out.mapping_height = model.mapping_height ? model.mapping_height : model.height;
    decltype(out.points) selected_points{
        PlainPayloadAllocator<Point3d>{mv::runtime::persist_resource(), true}};
    selected_points.resize(count);
#if MVKIT_RECONSTRUCTION_HAS_AVX2
    if (indexed && vectorized && count >= 32768 && !out.mapping_width && !input_confidence &&
        !input_normals && model.source_indices.empty() && model.voxel_counts.empty() &&
        model.observed_valid.empty() && model.filled.empty() && model.quality_reason.empty() &&
        model.depth_uncertainty.empty()) {
      // 已知精确输出数；中等输入避免上界构造，足够长游程避免目标 RFO。
      copy_point_runs_avx2(model.points, selected_points, runs);
      out.points = std::move(selected_points);
      validate_object_model_3d(out);
      return out;
    }
#endif
    std::size_t copied_points = 0;
    const bool confidence = input_confidence != nullptr, normals = input_normals != nullptr;
    if (confidence) out.confidence.reserve(count);
    if (normals) out.normals.reserve(count);
    if (!model.source_indices.empty() || !model.observed_valid.empty() || out.mapping_width)
      out.source_indices.reserve(count);
    if (!model.voxel_counts.empty()) out.voxel_counts.reserve(count);
    if (!model.observed_valid.empty()) out.observed_valid.reserve(count);
    if (!model.filled.empty()) out.filled.reserve(count);
    if (!model.quality_reason.empty()) out.quality_reason.reserve(count);
    if (!model.depth_uncertainty.empty()) out.depth_uncertainty.reserve(count);
    const auto copy_selected = [&](auto has_confidence, auto has_normals) {
      const auto copy_run = [&](std::size_t first, std::size_t i) {
        // 选中游程保持次序，各载荷只复制一次，不逐点检查 vector 扩容或值初始化。
        const auto length = i - first;
        std::memcpy(selected_points.data() + copied_points, input_points + first, length * sizeof(Point3d));
        copied_points += length;
        if (!model.source_indices.empty())
          out.source_indices.insert(out.source_indices.end(), model.source_indices.begin() + first,
                                    model.source_indices.begin() + i);
        else if (!model.observed_valid.empty() || out.mapping_width)
          for (std::size_t k = first; k < i; ++k) out.source_indices.push_back(k);
        if (!model.voxel_counts.empty())
          out.voxel_counts.insert(out.voxel_counts.end(), model.voxel_counts.begin() + first,
                                  model.voxel_counts.begin() + i);
        if (!model.observed_valid.empty())
          out.observed_valid.insert(out.observed_valid.end(), model.observed_valid.begin() + first,
                                    model.observed_valid.begin() + i);
        if (!model.filled.empty())
          out.filled.insert(out.filled.end(), model.filled.begin() + first, model.filled.begin() + i);
        if (!model.quality_reason.empty())
          out.quality_reason.insert(out.quality_reason.end(), model.quality_reason.begin() + first,
                                    model.quality_reason.begin() + i);
        if (!model.depth_uncertainty.empty())
          out.depth_uncertainty.insert(out.depth_uncertainty.end(), model.depth_uncertainty.begin() + first,
                                       model.depth_uncertainty.begin() + i);
        if constexpr (decltype(has_confidence)::value)
          out.confidence.insert(out.confidence.end(), input_confidence + first, input_confidence + i);
        if constexpr (decltype(has_normals)::value)
          out.normals.insert(out.normals.end(), input_normals + first, input_normals + i);
      };
      if (indexed)
        for (const auto [first, last] : runs) copy_run(first, last);
      else
        for (std::size_t i = 0; i < total;) {
          while (i < total && !matches(i)) ++i;
          const auto first = i;
          while (i < total && matches(i)) ++i;
          if (first == i) break;
          copy_run(first, i);
        }
    };
    if (confidence) {
      if (normals)
        copy_selected(std::true_type{}, std::true_type{});
      else
        copy_selected(std::true_type{}, std::false_type{});
    } else {
      if (normals)
        copy_selected(std::false_type{}, std::true_type{});
      else
        copy_selected(std::false_type{}, std::false_type{});
    }
    // 资源相同可接管完整写入缓冲；out 保留普通 allocator，含零初始化规则。
    out.points = std::move(selected_points);
    validate_object_model_3d(out);
    return out;
  };
  switch (kind) {
    case Attr::X:
      return select(
          [&](std::size_t i) { return coordinate_column.empty() ? input_points[i].x : coordinate_column[i]; },
          coordinate_min, coordinate_max);
    case Attr::Y:
      return select(
          [&](std::size_t i) { return coordinate_column.empty() ? input_points[i].y : coordinate_column[i]; },
          coordinate_min, coordinate_max);
    case Attr::Z:
      return select(
          [&](std::size_t i) { return coordinate_column.empty() ? input_points[i].z : coordinate_column[i]; },
          coordinate_min, coordinate_max);
    case Attr::Score:
      return select([&](std::size_t i) { return double(input_confidence[i]); }, min_value, max_value);
    case Attr::Observed:
      return select([&](std::size_t i) { return double(model.observed_valid[i]); }, min_value, max_value);
    case Attr::Filled:
      return select([&](std::size_t i) { return double(model.filled[i]); }, min_value, max_value);
    case Attr::Reason:
      return select([&](std::size_t i) { return double(model.quality_reason[i]); }, min_value, max_value);
    case Attr::Sigma:
      return select([&](std::size_t i) { return double(model.depth_uncertainty[i]); }, min_value, max_value);
  }
  return {};
}

ObjectModel3DData sample_object_model_3d(const ObjectModel3DData& model, std::string_view method,
                                         double sample_distance) {
  constexpr auto op = "sample_object_model_3d";
  validate_object_model_3d(model);
  if (method != "voxel" && method != "voxel_centroid") {
    fail_value(op, "Method", "supported sampling methods are voxel and voxel_centroid");
  }
  if (!mv::core::is_finite(sample_distance) || sample_distance <= 0.0) {
    fail_value(op, "SampleDistance", "sample distance must be finite and positive");
  }

  runtime::ExecutionContext context;
  if (method == "voxel_centroid") {
    std::pmr::unordered_map<VoxelKey, size_t, VoxelHash> cells{context.scratch_resource()};
    cells.reserve(model.points.size());
    ObjectModel3DData out;
    out.input_to_sample.assign(model.points.size(), -1);
    for (size_t i = 0; i < model.points.size(); ++i) {
      if (!point_valid(model, i)) continue;
      const VoxelKey key{voxel_index(model.points[i].x, sample_distance, op),
                         voxel_index(model.points[i].y, sample_distance, op),
                         voxel_index(model.points[i].z, sample_distance, op)};
      auto [it, inserted] = cells.emplace(key, out.points.size());
      const size_t j = it->second;
      out.input_to_sample[i] = static_cast<int64_t>(j);
      if (inserted) {
        out.points.push_back(model.points[i]);
        out.voxel_counts.push_back(1);
        out.source_indices.push_back(model.source_indices.empty() ? i : model.source_indices[i]);
        if (!model.confidence.empty()) out.confidence.push_back(model.confidence[i]);
        if (!model.normals.empty()) out.normals.push_back(model.normals[i]);
      } else {
        const double count = static_cast<double>(++out.voxel_counts[j]);
        auto& p = out.points[j];
        const auto q = model.points[i];
        p.x += (q.x - p.x) / count;
        p.y += (q.y - p.y) / count;
        p.z += (q.z - p.z) / count;
        if (!model.confidence.empty())
          out.confidence[j] += static_cast<float>((model.confidence[i] - out.confidence[j]) / count);
        if (!model.normals.empty()) {
          auto& n = out.normals[j];
          const auto v = model.normals[i];
          n.x += v.x;
          n.y += v.y;
          n.z += v.z;
        }
      }
    }
    for (auto& n : out.normals) {
      const double len = std::hypot(n.x, n.y, n.z);
      if (len > 1e-15) {
        n.x /= len;
        n.y /= len;
        n.z /= len;
      } else
        n = {};
    }
    validate_object_model_3d(out);
    return out;
  }
  std::pmr::unordered_set<VoxelKey, VoxelHash> first{context.scratch_resource()};
  first.reserve(model.points.size());
  std::pmr::vector<std::uint8_t> keep{model.points.size(), 0U, context.scratch_resource()};
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    if (!point_valid(model, i)) continue;
    const VoxelKey key{voxel_index(model.points[i].x, sample_distance, op),
                       voxel_index(model.points[i].y, sample_distance, op),
                       voxel_index(model.points[i].z, sample_distance, op)};
    const auto [it, inserted] = first.emplace(key);
    if (inserted) keep[i] = 1U;
    (void)it;
  }
  auto out = compact_object_model_3d(model, keep.data());
  validate_object_model_3d(out);
  return out;
}

}  // namespace mv::reconstruction
