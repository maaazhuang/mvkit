#include "sampling.hpp"

#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <numeric>
#include <reconstruction/neighborhood_index.hpp>
#include <unordered_set>
#include <vector>

#include "normal_estimation.hpp"
namespace mv::reconstruction {
namespace {
constexpr auto op = "sample_object_model_3d";
double d2(Point3d a, Point3d b) {
  const auto x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
  return x * x + y * y + z * z;
}
double dot(Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Point3d unit(Point3d p) {
  const auto n = std::hypot(p.x, p.y, p.z);
  return n > 0 ? Point3d{p.x / n, p.y / n, p.z / n} : Point3d{};
}
bool valid(const ObjectModel3DData& m, std::size_t i) {
  const auto p = m.points[i];
  return (m.valid.empty() || m.valid[i]) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
[[noreturn]] void bad(const char* p, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, p, -1, message);
}
}  // namespace
ObjectModel3DData sample_points(const ObjectModel3DData& model, std::string_view method, double parameter,
                                const SamplingOptions& options) {
  validate_object_model_3d(model);
  if (!std::isfinite(parameter) || parameter <= 0) bad("SamplingParam", "positive finite parameter required");
  if (!options.minimum_points || !std::isfinite(options.maximum_normal_angle) ||
      options.maximum_normal_angle < 0 || options.maximum_normal_angle > 180)
    bad("GenParamValue", "invalid sampling options");
  if (!model.triangles.empty() || !model.lines.empty() || model.primitive_kind != PrimitiveKind::None)
    throw core::Exception(core::Status::Unsupported, op,
                          "face, line and primitive surface sampling not implemented");
  const bool normals = method.ends_with("_compute_normals");
  if (normals) method = method.substr(0, method.size() - 16);
  if (method != "fast" && method != "accurate" && method != "accurate_use_normals" &&
      method != "xyz_mapping" && method != "furthest_point")
    bad("Method", "unknown sampling method");
  if ((method == "xyz_mapping" && model.points.empty()) ||
      (method == "furthest_point" && (model.points.empty() || parameter < 1)))
    return {};
  ObjectModel3DData computed;
  const auto& source =
      normals && model.normals.empty() ? (computed = estimate_normals(model, "xyz_mapping")) : model;
  if (method == "accurate_use_normals" && source.normals.empty())
    bad("ObjectModel3D", "point normals required");
  std::vector<std::size_t> ids;
  ids.reserve(source.points.size());
  for (std::size_t i = 0; i < source.points.size(); ++i)
    if (valid(source, i)) ids.push_back(i);
  std::vector<std::uint8_t> keep(source.points.size(), 0);
  ObjectModel3DData result;
  if (method == "xyz_mapping") {
    if (ids.empty()) return compact_object_model_3d(source, keep.data());
    const auto width = source.mapping_width ? source.mapping_width : source.width,
               height = source.mapping_height ? source.mapping_height : source.height;
    if (!width || !height) bad("ObjectModel3D", "XYZ mapping required");
    // 超栅格范围的值只选择栅格原点；转整数前裁剪，避免越界转换 UB。
    const double step = std::min(double(std::max(width, height)), std::max(1., parameter));
    const auto columns = static_cast<std::size_t>(std::floor(double(width - 1) / step)) + 1;
    std::unordered_set<std::size_t> occupied;
    occupied.reserve(ids.size());
    for (auto i : ids) {
      const auto slot = source.source_indices.empty() ? i : source.source_indices[i];
      if (slot / width >= height) bad("ObjectModel3D", "mapping index outside image");
      const auto cell = static_cast<std::size_t>(std::floor(double(slot / width) / step)) * columns +
                        static_cast<std::size_t>(std::floor(double(slot % width) / step));
      if (occupied.insert(cell).second) keep[i] = 1;
    }
    result = compact_object_model_3d(source, keep.data());
  } else if (method == "furthest_point") {
    // HALCON 截断数值计数；转换前裁剪，有限但超机器整数范围的计数也如此。
    const auto count = static_cast<std::size_t>(std::min(double(ids.size()), std::floor(parameter)));
    if (count == ids.size()) {
      result = compact_object_model_3d(source, std::span<const std::size_t>(ids));
      if (normals)
        for (auto& n : result.normals)
          if (n.z < 0) n = {-n.x, -n.y, -n.z};
      return result;
    }
    std::vector<std::size_t> selected;
    selected.reserve(count);
    if (count) {
      Point3d low = source.points[ids[0]], high = low;
      for (auto i : ids) {
        auto p = source.points[i];
        low = {std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z)};
        high = {std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z)};
      }
      Point3d center{low.x + (high.x - low.x) / 2, low.y + (high.y - low.y) / 2,
                     low.z + (high.z - low.z) / 2};
      std::size_t next = 0;
      double far = -1;
      for (std::size_t j = 0; j < ids.size(); ++j) {
        auto d = d2(source.points[ids[j]], center);
        if (d > far) {
          far = d;
          next = j;
        }
      }
      std::vector<double> closest(ids.size(), std::numeric_limits<double>::infinity());
      for (std::size_t n = 0; n < count; ++n) {
        const auto i = ids[next];
        keep[i] = 1;
        selected.push_back(i);
        far = -1;
        for (std::size_t j = 0; j < ids.size(); ++j) {
          closest[j] = std::min(closest[j], d2(source.points[ids[j]], source.points[i]));
          if (!keep[ids[j]] && closest[j] > far) {
            far = closest[j];
            next = j;
          }
        }
      }
    }
    result = compact_object_model_3d(source, std::span<const std::size_t>(selected));
  } else {
    if (ids.empty()) bad("ObjectModel3D", "point data required for radius sampling");
    NeighborhoodIndex index(source);
    NeighborhoodVisitedState visited(index);
    std::vector<std::uint8_t> seen(source.points.size(), 0);
    std::vector<std::size_t> neighbors;
    std::vector<Point3d> centers, normal_means;
    const double cosine = static_cast<float>(
        std::cos(options.maximum_normal_angle * 3.14159265358979323846 / 180));
    for (auto i : ids) {
      if (seen[i]) continue;
      index.radius_unvisited(i, parameter, &neighbors, seen, visited);
      if (method == "accurate_use_normals") {
        // 带法向方法消耗种子，但仅插值满足条件的邻居；孤立种子不产生输出。
        seen[i] = 1;
        visited.exclude(i);
      } else {
        neighbors.push_back(i);
      }
      std::sort(neighbors.begin(), neighbors.end());
      neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
      Point3d center{}, normal{};
      std::size_t count = 0;
      for (auto j : neighbors) {
        if (method == "fast" && j != i && !(d2(source.points[i], source.points[j]) < parameter * parameter))
          continue;
        if (method == "accurate_use_normals" &&
            std::clamp(dot(unit(source.normals[i]), unit(source.normals[j])), -1., 1.) <= cosine)
          continue;
        seen[j] = 1;
        visited.exclude(j);
        auto p = source.points[j];
        center.x += p.x;
        center.y += p.y;
        center.z += p.z;
        ++count;
        if (!source.normals.empty()) {
          auto n = source.normals[j];
          normal.x += n.x;
          normal.y += n.y;
          normal.z += n.z;
        }
      }
      if (method == "fast") {
        keep[i] = 1;
      } else if (count >= options.minimum_points) {
        keep[i] = 1;
        centers.push_back({center.x / count, center.y / count, center.z / count});
        normal_means.push_back(unit(normal));
      }
    }
    result = compact_object_model_3d(source, keep.data());
    if (method != "fast") {
      if (result.points.empty()) return {};
      result.confidence.clear();
      result.observed_valid.clear();
      result.filled.clear();
      result.quality_reason.clear();
      result.depth_uncertainty.clear();
      result.voxel_counts.clear();
      result.input_to_sample.clear();
      result.point_attributes = {};
      for (std::size_t i = 0; i < centers.size(); ++i) {
        result.points[i] = centers[i];
        if (!result.normals.empty()) result.normals[i] = normal_means[i];
      }
    }
  }
  if (normals)
    for (auto& n : result.normals)
      if (n.z < 0) n = {-n.x, -n.y, -n.z};
  validate_object_model_3d(result);
  return result;
}
}  // namespace mv::reconstruction
