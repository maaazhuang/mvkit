#pragma once

#include <array>
#include <core/image.hpp>
#include <core/tuple.hpp>
#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <core/geometry_types.hpp>
#include <reconstruction/point_attributes.hpp>
#include <reconstruction/primitive_support.hpp>
#include <runtime/memory/memory_resource.hpp>
#include <runtime/memory/plain_payload_allocator.hpp>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace mv::reconstruction {

template <class T>
using PlainPayloadAllocator = mv::runtime::PlainPayloadAllocator<T>;

class NeighborhoodIndex;

enum class PrimitiveKind : std::uint8_t { None, Plane, Sphere, Cylinder };

struct ObjectModel3DData {
  ObjectModel3DData() = default;
  enum class CopyMode { All, Transform };
  ObjectModel3DData(const ObjectModel3DData& other) : ObjectModel3DData(other, CopyMode::All) {}
  // 变换覆盖坐标并清除相机 Z 不确定度，其余载荷仅复制一次，不先复制任一数组。
  ObjectModel3DData(const ObjectModel3DData& other, CopyMode mode)
      : points(mv::runtime::persist_resource()),
        confidence(other.confidence, mv::runtime::persist_resource()),
        valid(other.valid, mv::runtime::persist_resource()),
        observed_valid(other.observed_valid, mv::runtime::persist_resource()),
        filled(other.filled, mv::runtime::persist_resource()),
        quality_reason(other.quality_reason, mv::runtime::persist_resource()),
        depth_uncertainty(
            other.depth_uncertainty.begin(),
            mode == CopyMode::All ? other.depth_uncertainty.end() : other.depth_uncertainty.begin(),
            mv::runtime::persist_resource()),
        normals(other.normals, mv::runtime::persist_resource()),
        triangles(other.triangles, mv::runtime::persist_resource()),
        lines(other.lines, mv::runtime::persist_resource()),
        source_indices(other.source_indices, mv::runtime::persist_resource()),
        voxel_counts(other.voxel_counts, mv::runtime::persist_resource()),
        input_to_sample(other.input_to_sample, mv::runtime::persist_resource()),
        point_attributes(other.point_attributes),
        width(other.width),
        height(other.height),
        mapping_width(other.mapping_width ? other.mapping_width : other.width),
        mapping_height(other.mapping_height ? other.mapping_height : other.height),
        triangles_are_polygons(other.triangles_are_polygons),
        primitive_kind(other.primitive_kind),
        plane(other.plane),
        primitive(other.primitive),
        primitive_support(other.primitive_support),
        fit_iterations(other.fit_iterations),
        fit_converged(other.fit_converged),
        information_ratio(other.information_ratio),
        primitive_rms(other.primitive_rms),
        primitive_num_inliers(other.primitive_num_inliers) {
    if (mode == CopyMode::All) points.assign(other.points.begin(), other.points.end());
  }
  ObjectModel3DData& operator=(const ObjectModel3DData& other) {
    if (this == &other) return *this;
    points.assign(other.points.begin(), other.points.end());
    confidence.assign(other.confidence.begin(), other.confidence.end());
    valid.assign(other.valid.begin(), other.valid.end());
    observed_valid.assign(other.observed_valid.begin(), other.observed_valid.end());
    filled.assign(other.filled.begin(), other.filled.end());
    quality_reason.assign(other.quality_reason.begin(), other.quality_reason.end());
    depth_uncertainty.assign(other.depth_uncertainty.begin(), other.depth_uncertainty.end());
    normals.assign(other.normals.begin(), other.normals.end());
    triangles.assign(other.triangles.begin(), other.triangles.end());
    lines.assign(other.lines.begin(), other.lines.end());
    source_indices.assign(other.source_indices.begin(), other.source_indices.end());
    voxel_counts.assign(other.voxel_counts.begin(), other.voxel_counts.end());
    input_to_sample.assign(other.input_to_sample.begin(), other.input_to_sample.end());
    point_attributes = other.point_attributes;
    width = other.width;
    height = other.height;
    mapping_width = other.mapping_width;
    mapping_height = other.mapping_height;
    triangles_are_polygons = other.triangles_are_polygons;
    primitive_kind = other.primitive_kind;
    plane = other.plane;
    primitive = other.primitive;
    primitive_support = other.primitive_support;
    fit_iterations = other.fit_iterations;
    fit_converged = other.fit_converged;
    information_ratio = other.information_ratio;
    primitive_rms = other.primitive_rms;
    primitive_num_inliers = other.primitive_num_inliers;
    return *this;
  }
  ObjectModel3DData(ObjectModel3DData&&) noexcept = default;
  ObjectModel3DData& operator=(ObjectModel3DData&&) noexcept = default;

  static_assert(std::is_trivially_copyable_v<Point3d> && std::is_trivially_destructible_v<Point3d>);
  static_assert(!std::uses_allocator_v<Point3d, PlainPayloadAllocator<Point3d>>);
  std::vector<Point3d, PlainPayloadAllocator<Point3d>> points{mv::runtime::persist_resource()};
  std::pmr::vector<float> confidence{mv::runtime::persist_resource()};
  std::pmr::vector<std::uint8_t> valid{mv::runtime::persist_resource()};
  std::pmr::vector<std::uint8_t> observed_valid{mv::runtime::persist_resource()};
  std::pmr::vector<std::uint8_t> filled{mv::runtime::persist_resource()};
  std::pmr::vector<std::uint16_t> quality_reason{mv::runtime::persist_resource()};
  std::pmr::vector<float> depth_uncertainty{mv::runtime::persist_resource()};
  std::pmr::vector<Point3d> normals{mv::runtime::persist_resource()};
  std::pmr::vector<std::array<std::uint32_t, 3>> triangles{mv::runtime::persist_resource()};
  std::pmr::vector<std::array<std::uint32_t, 2>> lines{mv::runtime::persist_resource()};
  std::pmr::vector<std::size_t> source_indices{mv::runtime::persist_resource()};
  std::pmr::vector<std::size_t> voxel_counts{mv::runtime::persist_resource()};
  // 直接输入栅格槽→采样输出索引，-1 表示无效输入。
  std::pmr::vector<std::int64_t> input_to_sample{mv::runtime::persist_resource()};
  PointAttributes point_attributes;
  std::size_t width{};
  std::size_t height{};
  // 稀疏映射在 source_indices 保留原栅格索引。
  std::size_t mapping_width{};
  std::size_t mapping_height{};
  // 正式 PLY 将三角面表示为 polygon，计算内核仍使用同一带索引三角载荷。
  bool triangles_are_polygons{};
  PrimitiveKind primitive_kind{PrimitiveKind::None};
  std::array<double, 4> plane{};
  std::array<double, 7> primitive{};
  std::shared_ptr<const PrimitiveSupport> primitive_support;
  std::size_t fit_iterations{};
  bool fit_converged{};
  double information_ratio{};
  double primitive_rms{};
  std::size_t primitive_num_inliers{};

  [[nodiscard]] bool payloads_use_persist_resource() const noexcept {
    auto* persist = mv::runtime::persist_resource();
    return points.get_allocator().resource() == persist && confidence.get_allocator().resource() == persist &&
           valid.get_allocator().resource() == persist &&
           observed_valid.get_allocator().resource() == persist &&
           filled.get_allocator().resource() == persist &&
           quality_reason.get_allocator().resource() == persist &&
           depth_uncertainty.get_allocator().resource() == persist &&
           normals.get_allocator().resource() == persist && triangles.get_allocator().resource() == persist &&
           lines.get_allocator().resource() == persist &&
           source_indices.get_allocator().resource() == persist &&
           voxel_counts.get_allocator().resource() == persist &&
           input_to_sample.get_allocator().resource() == persist && point_attributes.UsesPersistResource() &&
           (!primitive_support || primitive_support->plane_boundary.get_allocator().resource() == persist);
  }
};

struct XyzImages {
  core::Image x;
  core::Image y;
  core::Image z;
};

enum class IcpWeighting { None, Huber, Tukey };
struct IcpOptions {
  double trim_ratio = 1.0, max_normal_angle = 3.141592653589793;
  IcpWeighting weighting = IcpWeighting::None;
  double clipping_factor = 2.0, pose_delta = 0.0;
  bool coarse_seed = false;
};
struct RegisterObjectModel3DResult {
  core::Tuple pose;
  std::int64_t iterations{};
  double rms{};
  double overlap{};
  bool converged{};
  std::size_t sample_count{}, inlier_count{};
};

void validate_object_model_3d(const ObjectModel3DData& model);
[[nodiscard]] ObjectModel3DData fit_curved_primitive(const ObjectModel3DData&, std::string_view, double,
                                                     const core::Tuple&, const core::Tuple&);
[[nodiscard]] core::Tuple distance_to_surface(const ObjectModel3DData&, const ObjectModel3DData&,
                                              std::string_view);
[[nodiscard]] ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model,
                                                        const std::uint8_t* keep);
[[nodiscard]] ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model,
                                                        std::span<const std::size_t> indices);
[[nodiscard]] ObjectModel3DData compact_object_model_3d(const ObjectModel3DData& model,
                                                        std::span<const std::size_t> indices,
                                                        const PointAttributes::Snapshot& attributes);
[[nodiscard]] XyzImages object_model_3d_to_xyz(const ObjectModel3DData& model);

[[nodiscard]] ObjectModel3DData gen_object_model_3d_from_points(const core::Tuple& x, const core::Tuple& y,
                                                                const core::Tuple& z);
[[nodiscard]] ObjectModel3DData xyz_to_object_model_3d(const core::Image& x, const core::Image& y,
                                                       const core::Image& z);
[[nodiscard]] ObjectModel3DData affine_trans_object_model_3d(const ObjectModel3DData& model,
                                                             const core::Tuple& hom_mat3d);
[[nodiscard]] ObjectModel3DData rigid_trans_object_model_3d(const ObjectModel3DData& model,
                                                            const core::Tuple& pose);
[[nodiscard]] ObjectModel3DData select_points_object_model_3d(const ObjectModel3DData& model,
                                                              std::string_view attrib, double min_value,
                                                              double max_value,
                                                              std::span<const double> coordinate_column = {},
                                                              std::span<const double> coordinate_ranges = {});
[[nodiscard]] ObjectModel3DData sample_object_model_3d(const ObjectModel3DData& model,
                                                       std::string_view method, double sample_distance);
void write_object_model_3d(std::string_view path, std::string_view file_type, const ObjectModel3DData& model);
[[nodiscard]] ObjectModel3DData read_object_model_3d(std::string_view path, double scale);

[[nodiscard]] ObjectModel3DData filter_object_model_3d(const ObjectModel3DData& model,
                                                       std::string_view method, std::int64_t num_neighbors,
                                                       double std_mul,
                                                       const NeighborhoodIndex* shared_index = nullptr);
[[nodiscard]] ObjectModel3DData surface_normals_object_model_3d(
    const ObjectModel3DData& model, std::string_view method, std::int64_t num_neighbors,
    const core::Tuple& gen_param_name = {}, const core::Tuple& gen_param_value = {},
    const NeighborhoodIndex* shared_index = nullptr);
[[nodiscard]] std::vector<ObjectModel3DData> connection_object_model_3d(
    const ObjectModel3DData& model, double max_distance, std::int64_t min_points,
    const NeighborhoodIndex* shared_index = nullptr);
[[nodiscard]] ObjectModel3DData fit_primitives_object_model_3d(const ObjectModel3DData& model,
                                                               std::string_view type, double max_distance,
                                                               const core::Tuple& gen_param_name = {},
                                                               const core::Tuple& gen_param_value = {});
[[nodiscard]] core::Tuple distance_object_model_3d(const ObjectModel3DData& model,
                                                   const ObjectModel3DData& reference,
                                                   std::string_view method,
                                                   const NeighborhoodIndex* reference_index = nullptr);
[[nodiscard]] RegisterObjectModel3DResult register_object_model_3d_pair(
    const ObjectModel3DData& source, const ObjectModel3DData& target, const core::Tuple& initial_pose,
    std::string_view method, std::int64_t max_iterations, double max_distance,
    const core::Tuple& gen_param_name = {}, const core::Tuple& gen_param_value = {},
    const NeighborhoodIndex* target_index = nullptr);
[[nodiscard]] RegisterObjectModel3DResult register_object_model_3d_pair(
    const ObjectModel3DData& source, const ObjectModel3DData& target, const core::Tuple& initial_pose,
    std::string_view method, std::int64_t max_iterations, double max_distance, const IcpOptions& options,
    const NeighborhoodIndex* target_index = nullptr);

[[nodiscard]] ObjectModel3DData intersect_plane_object_model_3d(const ObjectModel3DData& model, double nx,
                                                                double ny, double nz, double d);

}  // namespace mv::reconstruction
