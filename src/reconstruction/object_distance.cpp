#include "object_distance.hpp"

#include <cmath>
#include <limits>
#include <reconstruction/mesh_index.hpp>
#include <reconstruction/neighborhood_index.hpp>
#include <reconstruction/primitive_fitting.hpp>
#include <runtime/execution/execution_context.hpp>
namespace mv::reconstruction {
namespace {
constexpr auto op = "distance_object_model_3d";
[[noreturn]] void bad(const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                        message);
}
Point3d transform(Point3d p, const ObjectDistanceOptions& options) {
  if (!options.pose) return p;
  const auto& m = *options.pose;
  return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
          m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
}
double primitive_distance(Point3d p, const ObjectModel3DData& target) {
  if (target.primitive_support) return model_primitive_distance(p, target);
  if (target.primitive_kind == PrimitiveKind::Plane) {
    const auto& a = target.plane;
    return a[0] * p.x + a[1] * p.y + a[2] * p.z + a[3];
  }
  const auto& a = target.primitive;
  const auto q = mesh_detail::sub(p, {a[0], a[1], a[2]});
  if (target.primitive_kind == PrimitiveKind::Sphere) return std::hypot(q.x, q.y, q.z) - a[3];
  if (target.primitive_kind == PrimitiveKind::Cylinder) {
    const auto t = q.x * a[3] + q.y * a[4] + q.z * a[5];
    return std::hypot(q.x - t * a[3], q.y - t * a[4], q.z - t * a[5]) - a[6];
  }
  bad("ObjectModel3DTo", "target has no primitive");
}
}  // namespace
ObjectDistanceResult object_distances(const ObjectModel3DData& source, const ObjectModel3DData& target,
                                      const ObjectDistanceOptions& options, const NeighborhoodIndex* cached) {
  validate_object_model_3d(source);
  validate_object_model_3d(target);
  if (source.points.empty() ||
      (!source.valid.empty() &&
       std::none_of(source.valid.begin(), source.valid.end(), [](auto value) { return value != 0; })))
    bad("ObjectModel3DFrom", "source has no point data");
  // HALCON 将非正及非有限控制解释为不限搜索。
  const double maximum = std::isfinite(options.maximum) && options.maximum > 0 ? options.maximum : 0;
  auto type = options.distance_to;
  if (type == "auto")
    type = target.primitive_kind != PrimitiveKind::None ? "primitive"
           : !target.triangles.empty() ? (target.triangles_are_polygons ? "polygons" : "triangles")
                                       : "points";
  if (type != "points" && type != "triangles" && type != "polygons" && type != "primitive")
    bad("distance_to", "unknown target type");
  if (type == "polygons" && !target.triangles_are_polygons) bad("ObjectModel3DTo", "target has no polygons");
  if (options.method != "auto" && options.method != "linear" && options.method != "kd-tree" &&
      options.method != "voxel")
    bad("method", "unknown distance method");
  if (options.method == "voxel")
    throw core::Exception(core::Status::Unsupported, op, "voxel distance preparation not implemented");
  if (type == "primitive" && options.method == "kd-tree")
    bad("method", "primitive distances require linear method");
  if (type == "primitive" && options.store_closest_index)
    bad("store_closest_index", "closest index is unavailable for primitives");
  if (type == "primitive" && target.primitive_kind == PrimitiveKind::None)
    bad("ObjectModel3DTo", "target has no primitive");
  if (type == "points" && options.signed_distances && target.normals.empty())
    bad("ObjectModel3DTo", "point normals required for signed distances");
  const double cap = maximum > 0 ? maximum * maximum : std::numeric_limits<double>::infinity();
  const bool linear = options.method == "linear" ||
                      (options.method == "auto" && (type != "points" || target.points.size() < 100));
  std::optional<NeighborhoodIndex> points;
  std::optional<MeshIndex> mesh;
  std::pmr::vector<std::size_t> reference_indices{runtime::scratch_resource()};
  if (type == "points") {
    if (linear) {
      for (std::size_t i = 0; i < target.points.size(); ++i)
        if (mesh_detail::valid(target, i)) reference_indices.push_back(i);
      if (reference_indices.empty()) bad("ObjectModel3DTo", "target has no finite point data");
    } else {
      if (!cached) {
        points.emplace(target);
        cached = &*points;
      }
      if (!cached->size()) bad("ObjectModel3DTo", "target has no finite point data");
    }
  } else if (type == "triangles" || type == "polygons") {
    if (linear) {
      for (std::size_t i = 0; i < target.triangles.size(); ++i) {
        const auto t = target.triangles[i];
        if (mesh_detail::valid(target, t[0]) && mesh_detail::valid(target, t[1]) &&
            mesh_detail::valid(target, t[2]))
          reference_indices.push_back(i);
      }
      if (reference_indices.empty()) bad("ObjectModel3DTo", "target has no finite triangle data");
    } else {
      mesh.emplace(target);
    }
  }
  ObjectDistanceResult result;
  std::pmr::vector<std::size_t> target_ordinals{runtime::scratch_resource()};
  if (type == "points" && !target.valid.empty() &&
      std::find(target.valid.begin(), target.valid.end(), std::uint8_t{0}) != target.valid.end()) {
    target_ordinals.resize(target.points.size());
    std::size_t ordinal = 0;
    for (std::size_t i = 0; i < target.points.size(); ++i) {
      target_ordinals[i] = ordinal;
      ordinal += target.valid[i] != 0;
    }
  }
  result.distances.resize(source.points.size(), std::numeric_limits<double>::quiet_NaN());
  if (options.store_closest_index) result.closest.resize(source.points.size(), -1.);
  runtime::ExecutionContext::for_each_partition(
      source.points.size(), source.points.size(), 4096, [&](auto range) {
        for (std::size_t i = range.begin; i < range.end; ++i) {
          if (!source.valid.empty() && !source.valid[i]) continue;
          const auto p = transform(source.points[i], options);
          std::size_t closest = 0;
          double squared = cap, distance = maximum;
          bool found = false;
          if (type == "primitive") {
            distance = primitive_distance(p, target);
            found = std::isfinite(distance);
          } else if (type == "points") {
            if (linear) {
              for (auto j : reference_indices) {
                const auto delta = mesh_detail::sub(p, target.points[j]);
                const auto d = mesh_detail::dot(delta, delta);
                if (std::isfinite(d) && d <= squared && (!found || d < squared || j < closest)) {
                  closest = j;
                  squared = d;
                  found = true;
                }
              }
            } else
              found = cached->nearest(p, cap, &closest, &squared, kNeighborhoodNoSkip, true);
            if (found) {
              distance = std::sqrt(squared);
              if (options.signed_distances &&
                  mesh_detail::dot(mesh_detail::sub(p, target.points[closest]), target.normals[closest]) < 0)
                distance = -distance;
            }
          } else {
            if (linear) {
              for (auto j : reference_indices) {
                const auto t = target.triangles[j];
                const auto d =
                    mesh_detail::triangle2(p, target.points[t[0]], target.points[t[1]], target.points[t[2]]);
                if (std::isfinite(d) && d <= squared && (!found || d < squared || j < closest)) {
                  closest = j;
                  squared = d;
                  found = true;
                }
              }
            } else {
              found = mesh->Nearest(p, cap, &closest, &squared);
            }
            if (found) {
              distance = std::sqrt(squared);
              const auto triangle = target.triangles[closest];
              if (options.signed_distances &&
                  mesh_detail::dot(
                      mesh_detail::sub(p, target.points[triangle[0]]),
                      mesh_detail::cross(
                          mesh_detail::sub(target.points[triangle[1]], target.points[triangle[0]]),
                          mesh_detail::sub(target.points[triangle[2]], target.points[triangle[0]]))) < 0)
                distance = -distance;
            }
          }
          if (!found) distance = maximum > 0 ? maximum : std::numeric_limits<double>::quiet_NaN();
          const bool within = found && (!maximum || std::abs(distance) < maximum);
          if (maximum && (type == "primitive" ? (options.signed_distances ? distance : std::abs(distance))
                                              : std::abs(distance)) >= maximum)
            distance = maximum;
          result.distances[i] = options.signed_distances ? distance : std::abs(distance);
          if (options.store_closest_index && type != "primitive" && within)
            result.closest[i] = double(target_ordinals.empty() ? closest : target_ordinals[closest]);
        }
      });
  return result;
}
}  // namespace mv::reconstruction
