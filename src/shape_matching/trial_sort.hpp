#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <new>
#include <type_traits>
#include <vector>

#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

namespace mv::shape_matching::detail {

// 仅压缩精确位姿身份及有界有序坐标跨度；不适用时返回 false，使用原四字段 radix，不修改 trial。
inline bool make_ranked_trial_keys(
    const std::pmr::vector<std::array<std::uint64_t, 4>>& keys,
    std::pmr::vector<std::uint64_t>& packed,
    std::pmr::memory_resource* memory) {
  const auto n = keys.size();
  if (!n || n > 4096) return false;
  auto min_row = keys[0][1], max_row = min_row;
  auto min_column = keys[0][0], max_column = min_column;
  for (const auto& key : keys) {
    min_row = std::min(min_row, key[1]);
    max_row = std::max(max_row, key[1]);
    min_column = std::min(min_column, key[0]);
    max_column = std::max(max_column, key[0]);
  }
  // 带符号次序映射使用无符号值，此相减不会溢出 int64。
  if (max_row - min_row > 65535 || max_column - min_column > 65535)
    return false;

  try {
    using Pose = std::array<std::uint64_t, 2>; // 先按实际 angle，再按实际 scale。
    std::pmr::vector<Pose> poses(memory);
    std::pmr::vector<std::uint16_t> table(memory), pose_ids(memory);
    std::pmr::vector<std::uint16_t> sorted_ids(memory), rank_of_id(memory);
    std::size_t capacity = 1;
    while (capacity < 2 * n) capacity <<= 1;
    constexpr auto empty = std::numeric_limits<std::uint16_t>::max();
    table.assign(capacity, empty);
    pose_ids.resize(n);
    poses.reserve(n);
    const auto hash_pose = [](const Pose& pose) {
      auto hash = pose[0] ^ (pose[1] + 0x9e3779b97f4a7c15ULL);
      hash ^= hash >> 33;
      hash *= 0xff51afd7ed558ccdULL;
      hash ^= hash >> 33;
      hash *= 0xc4ceb9fe1a85ec53ULL;
      return hash ^ (hash >> 33);
    };
    for (std::size_t i = 0; i < n; ++i) {
      const Pose pose{keys[i][3], keys[i][2]};
      auto slot = static_cast<std::size_t>(hash_pose(pose)) & (capacity - 1);
      bool found = false;
      for (unsigned probe = 0; probe < 32; ++probe) {
        auto& id = table[slot];
        if (id == empty) {
          if (poses.size() >= 65535) return false;
          id = static_cast<std::uint16_t>(poses.size());
          poses.push_back(pose);
          pose_ids[i] = id;
          found = true;
          break;
        }
        if (poses[id] == pose) {
          pose_ids[i] = id;
          found = true;
          break;
        }
        slot = (slot + 1) & (capacity - 1);
      }
      if (!found) return false;
    }
    // 仅排序不同 pose ID，保留其完整 128 位身份。
    sorted_ids.resize(poses.size());
    rank_of_id.resize(poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i)
      sorted_ids[i] = static_cast<std::uint16_t>(i);
    std::sort(sorted_ids.begin(), sorted_ids.end(), [&](auto a, auto b) {
      if (poses[a][0] != poses[b][0]) return poses[a][0] < poses[b][0];
      return poses[a][1] < poses[b][1];
    });
    for (std::size_t rank = 0; rank < sorted_ids.size(); ++rank)
      rank_of_id[sorted_ids[rank]] = static_cast<std::uint16_t>(rank);
    packed.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const auto row_offset = keys[i][1] - min_row;
      const auto column_offset = keys[i][0] - min_column;
      packed[i] = (static_cast<std::uint64_t>(rank_of_id[pose_ids[i]]) << 32) |
                  (row_offset << 16) | column_offset;
    }
    return true;
  } catch (const std::bad_alloc&) {
    // 已准备的通用键及索引无需打包仍可使用。
    return false;
  }
}

// 匹配原 trial 比较器：实际 Double angle/scale 后接有符号 row/column 晶格键。
// 同值或不支持值要求 std::sort 时返回 false 且不改 trial。Trial 提供 candidate.angle/scale
// 和 int64 row_key/column_key；ranked 可打包精确位姿秩及有界坐标偏移，
// 秩/跨度/探针不适用时保留通用四字段 radix。
template<class Trial, class Allocator>
bool try_radix_sort_trials(std::vector<Trial, Allocator>& trials,
                           std::pmr::memory_resource* memory, bool ranked = false) {
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  static_assert(std::numeric_limits<double>::is_iec559);
  static_assert(std::is_trivially_copyable_v<Trial>);
  const auto n = trials.size();
  if (!memory || n < 128 || n > std::numeric_limits<std::uint32_t>::max())
    return false;

  constexpr std::uint64_t sign = 0x8000000000000000ULL;
  constexpr std::uint64_t exponent = 0x7ff0000000000000ULL;
#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
  const bool denormals_are_zero = (_mm_getcsr() & 0x40u) != 0;
#else
  const bool denormals_are_zero = false;
#endif
  const auto double_key = [&](double value, std::uint64_t& out) {
    auto bits = std::bit_cast<std::uint64_t>(value);
    if ((bits & exponent) == exponent) return false; // 非有限值：NaN 或 infinity。
    const auto magnitude = bits & ~sign;
    // DAZ 比较将非零 Double subnormal 视为零，此模式保留原排序，不强制 IEEE 整数序。
    if (denormals_are_zero && magnitude && !(bits & exponent)) return false;
    if (magnitude == 0) bits = 0; // 比较器将 -0 和 +0 视为相等。
    out = (bits & sign) ? ~bits : bits ^ sign;
    return true;
  };

  try {
    // LSD 字段序为 column/row/scale/angle；索引和键使用查询 scratch，
    // 物化 trial 保留原 allocator，确保最终 swap 有效且为唯一输入修改。
    using Keys = std::array<std::uint64_t, 4>;
    std::pmr::vector<Keys> keys(memory);
    std::pmr::vector<std::uint32_t> order(memory), auxiliary(memory);
    keys.resize(n);
    order.resize(n);
    auxiliary.resize(n);
    Keys active{};
    for (std::size_t i = 0; i < n; ++i) {
      auto& key = keys[i];
      const auto& trial = trials[i];
      key[0] = std::bit_cast<std::uint64_t>(trial.column_key) ^ sign;
      key[1] = std::bit_cast<std::uint64_t>(trial.row_key) ^ sign;
      if (!double_key(trial.candidate.scale, key[2]) ||
          !double_key(trial.candidate.angle, key[3])) return false;
      order[i] = static_cast<std::uint32_t>(i);
      for (std::size_t field = 0; field < active.size(); ++field)
        active[field] |= key[field] ^ keys[0][field];
    }

    std::array<std::uint32_t, 256> counts;
    std::pmr::vector<std::uint64_t> packed(memory);
    const bool packed_mode = ranked && make_ranked_trial_keys(keys, packed, memory);
    if (packed_mode) {
      std::uint64_t active_packed = 0;
      for (const auto key : packed) active_packed |= key ^ packed[0];
      // 打包优先级为位姿秩、行偏移、列偏移，共六字节。
      for (unsigned shift = 0; shift < 48; shift += 8) {
        if (((active_packed >> shift) & 0xffu) == 0) continue;
        counts.fill(0);
        for (const auto index : order) ++counts[(packed[index] >> shift) & 0xffu];
        std::uint32_t prefix = 0;
        for (auto& count : counts) {
          const auto size = count;
          count = prefix;
          prefix += size;
        }
        for (const auto index : order) {
          const auto byte = (packed[index] >> shift) & 0xffu;
          auxiliary[counts[byte]++] = index;
        }
        order.swap(auxiliary);
      }
    } else {
      for (std::size_t field = 0; field < active.size(); ++field) {
        for (unsigned shift = 0; shift < 64; shift += 8) {
          if (((active[field] >> shift) & 0xffu) == 0) continue;
          counts.fill(0);
          for (const auto index : order)
            ++counts[(keys[index][field] >> shift) & 0xffu];
          std::uint32_t prefix = 0;
          for (auto& count : counts) {
            const auto size = count;
            count = prefix;
            prefix += size;
          }
          for (const auto index : order) {
            const auto byte = (keys[index][field] >> shift) & 0xffu;
            auxiliary[counts[byte]++] = index;
          }
          order.swap(auxiliary);
        }
      }
    }
    // 仅完整比较键唯一时，稳定 radix 可替代原比较器；不只凭 TrialKey 唯一性推断。
    for (std::size_t i = 1; i < n; ++i)
      if (keys[order[i - 1]] == keys[order[i]]) return false;

    std::vector<Trial, Allocator> sorted(trials.get_allocator());
    sorted.reserve(n);
    for (const auto index : order) sorted.push_back(trials[index]);
    trials.swap(sorted);
    return true;
  } catch (const std::bad_alloc&) {
    // 可选优化不能将可行原地排序变为查询失败，全部分配及完整复制均先于修改。
    return false;
  }
}

} // namespace mv::shape_matching::detail
