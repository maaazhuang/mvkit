#pragma once

#include <cstdint>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <optional>
#include <span>
#include <memory_resource>
#include <vector>
#include <limits>
#include <core/exception.hpp>

namespace mv::core {

// Region 坐标以有符号 32 位存储；边界运算先提升到 64 位，验证后才窄化。
inline constexpr std::int64_t region_coordinate_min = INT32_MIN;
inline constexpr std::int64_t region_coordinate_max = INT32_MAX;
[[nodiscard]] inline constexpr std::int32_t region_coordinate(
    std::int64_t value, const char* operation = "RegionRun") {
  if (value < region_coordinate_min || value > region_coordinate_max)
    throw Exception(Status::OutOfRange, operation, "region coordinate exceeds int32 range");
  return static_cast<std::int32_t>(value);
}
// 调用方先完成自身取整规则；避免非有限/超范围浮点数直接转为整数。
[[nodiscard]] inline std::int64_t region_coordinate_rounded(double value, const char* operation) {
  if (!(value >= region_coordinate_min && value <= region_coordinate_max))
    throw Exception(Status::OutOfRange, operation, "region coordinate exceeds int32 range");
  return static_cast<std::int64_t>(value);
}

struct RegionRun {
  std::int32_t row = 0;
  std::int32_t col_begin = 0;
  std::int32_t col_end = -1;
  constexpr RegionRun() = default;
  constexpr RegionRun(std::int64_t r, std::int64_t first, std::int64_t last)
      : row(region_coordinate(r)), col_begin(region_coordinate(first)),
        col_end(region_coordinate(last)) {}
  friend bool operator==(const RegionRun&, const RegionRun&) = default;
};
static_assert(sizeof(RegionRun) == 12);

struct RectI64 {
  std::int64_t row_min = 0;
  std::int64_t col_min = 0;
  std::int64_t row_max = -1;
  std::int64_t col_max = -1;
  friend bool operator==(const RectI64&, const RectI64&) = default;
};

struct RegionInterval {
  std::int64_t first, last;
  bool operator==(const RegionInterval&) const = default;
};
struct RegionBandView {
  std::int64_t first, last;
  std::span<const RegionInterval> profile;
};

// 紧凑分块标签将 runs_per_row 项重复 row_count 次。
struct RegionPartitionBand {
  std::size_t runs_per_row, row_count;
};

/** @brief 单个私有 Region 对象，使用规范化 RLE 表示任意二维像素集合。 */
class Region {
 public:
  Region() = default;

  [[nodiscard]] static Region from_runs(std::span<const RegionRun> runs);
  // 内部生产者可转移有序、不相交且不相邻游程；仍检查规范不变量，不信任非法输入。
  [[nodiscard]] static Region from_canonical_runs(std::pmr::vector<RegionRun>&& runs);
  // 内部不可变批量存储。生产者填完拥有者后才发布切片；检查规范性及 persist 分配。
  [[nodiscard]] static Region from_shared_canonical_runs(
      std::shared_ptr<const std::pmr::vector<RegionRun>> owner,
      std::size_t offset, std::size_t count);
  // 验证紧凑行带后展开，保留精确元数据。
  [[nodiscard]] static Region from_row_bands(std::span<const RegionBandView> bands);
  // 分块规范源游程，不重新规范化每个输出；检查标签/计数。两者为空时每源游程一个分量。
  // 每所有权块包含 <=32 KiB 游程及 <=128 元数据项，大分量独立拥有块。
  [[nodiscard]] static std::vector<Region> partition_runs(
      const Region& source, std::span<const std::size_t> labels = {},
      std::span<const std::size_t> counts = {},
      std::span<const RegionPartitionBand> bands = {});
  // 每像素一位，低位优先，每个规范域游程重新开始；每游程占 ceil(length/32) 个字，尾部未用位须为零。
  [[nodiscard]] static Region from_row_masks(const Region& domain,
                                            std::span<const std::uint32_t> masks);
  [[nodiscard]] static Region from_runs(std::initializer_list<RegionRun> runs) {
    return from_runs(std::span<const RegionRun>{runs.begin(), runs.size()});
  }
  [[nodiscard]] static Region rectangle(RectI64 rect);
  // 批量矩形共享有界不可变存储；每块最多 32 KiB 游程、128 项元数据，大矩形独立拥有。
  [[nodiscard]] static std::vector<Region> rectangles(std::span<const RectI64> boxes);

  // 内部批处理只能保留同一存储拥有者的句柄。
  [[nodiscard]] bool shares_storage_owner(const Region& other) const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] // 连续导出可能物化不可变共享片段，分配失败传播而非终止；metadata/contains 保持 noexcept。
  std::span<const RegionRun> runs() const;
  [[nodiscard]] std::size_t run_count() const noexcept;
  // 将表示复制到存活且布局兼容的平凡记录；调用方检查记录尺寸/字段偏移，不使用强类型别名转换。
  void copy_run_bytes(std::span<std::byte> destination) const;
  [[nodiscard]] std::uint64_t area() const noexcept;
  [[nodiscard]] std::uint64_t max_run_length() const noexcept;
  [[nodiscard]] bool is_rectangle() const noexcept;
  // false 表示构造时未证明，不能据此判定不连通。
  [[nodiscard]] bool known_connected4() const noexcept;
  [[nodiscard]] bool known_connected8() const noexcept;
  // 仅连通分量标记器可在完成拓扑证明后记录；单调增加的派生事实支持并发读取。
  void record_connected_component(bool four_connected) const noexcept;
  [[nodiscard]] std::optional<RectI64> bounding_box() const noexcept;
  [[nodiscard]] Region intersect(const Region& other) const;
  [[nodiscard]] Region united(const Region& other) const;
  [[nodiscard]] Region difference(const Region& other) const;
  [[nodiscard]] Region shifted(std::int64_t row_delta, std::int64_t col_delta) const;
  [[nodiscard]] bool contains(std::int64_t row, std::int64_t column) const noexcept;
  [[nodiscard]] Region clipped(RectI64 rect) const;
  [[nodiscard]] bool payload_uses_persist_resource() const noexcept;

 private:
  struct Storage;
  explicit Region(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}
  std::shared_ptr<const Storage> storage_;
};

}  // namespace mv::core
