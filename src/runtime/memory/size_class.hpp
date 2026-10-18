#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mv::runtime {

enum class PoolPurpose { Persist, Scratch };

inline constexpr std::size_t kSizeClassKiB = 1024;
inline constexpr std::size_t kSizeClassMiB = 1024 * 1024;
inline constexpr std::size_t kMinSizeClass = 16;
inline constexpr std::size_t kMaxPooledPayload = 4 * kSizeClassMiB;
inline constexpr std::size_t kMaxCachedPayload = 256 * kSizeClassMiB;
inline constexpr std::size_t kAlignmentClassCount = 3;
inline constexpr std::uint32_t kMaxBlocksPerSmallClass = 512;
inline constexpr std::uint32_t kMaxBlocksPerLargeClass = 16;

struct PayloadClassTable {
  std::array<std::size_t, 256> classes{};
  std::size_t count = 0;
};

[[nodiscard]] constexpr bool size_add_overflows(std::size_t left,
                                                std::size_t right) noexcept {
  return right > (std::numeric_limits<std::size_t>::max)() - left;
}

[[nodiscard]] constexpr std::size_t ceil_to(std::size_t value,
                                            std::size_t quantum) {
  if (quantum == 0) return value;
  if (value == 0) return 0;
  const std::size_t rounded = value - 1;
  if (size_add_overflows(rounded / quantum, 1))
    return (std::numeric_limits<std::size_t>::max)();
  const std::size_t count = rounded / quantum + 1;
  if (count > (std::numeric_limits<std::size_t>::max)() / quantum)
    return (std::numeric_limits<std::size_t>::max)();
  return count * quantum;
}

[[nodiscard]] constexpr std::size_t normalize_alignment(
    std::size_t alignment) noexcept {
  return alignment < 16 ? 16 : alignment;
}

[[nodiscard]] constexpr int alignment_class_index(
    std::size_t alignment) noexcept {
  const std::size_t normalized = normalize_alignment(alignment);
  if (normalized == 16) return 0;
  if (normalized == 32) return 1;
  if (normalized == 64) return 2;
  return -1;
}

[[nodiscard]] constexpr bool is_large_payload(std::size_t payload) noexcept {
  return payload > kMaxPooledPayload;
}

[[nodiscard]] constexpr std::uint32_t max_cached_blocks_for_payload(
    std::size_t payload) noexcept {
  // TLS 尺寸块按字节额度限制，不使用数量配额。
  if (payload <= 65536) return (std::numeric_limits<std::uint32_t>::max)();
  return is_large_payload(payload) ? kMaxBlocksPerLargeClass
                                   : kMaxBlocksPerSmallClass;
}

constexpr void push_payload_class(PayloadClassTable& table,
                                  std::size_t value) noexcept {
  table.classes[table.count++] = value;
}

constexpr void append_large_payload_classes(PayloadClassTable& table) noexcept {
  for (std::size_t value = 5 * kSizeClassMiB; value <= 16 * kSizeClassMiB;
       value += kSizeClassMiB)
    push_payload_class(table, value);
  for (std::size_t value = 18 * kSizeClassMiB; value <= 256 * kSizeClassMiB;
       value += 2 * kSizeClassMiB)
    push_payload_class(table, value);
}

[[nodiscard]] constexpr int first_index_at_least(const PayloadClassTable& table,
                                                 std::size_t bytes) noexcept {
  std::size_t lo = 0;
  std::size_t hi = table.count;
  while (lo < hi) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (table.classes[mid] < bytes)
      lo = mid + 1;
    else
      hi = mid;
  }
  return lo == table.count ? -1 : static_cast<int>(lo);
}

[[nodiscard]] constexpr std::size_t first_class_at_least(
    const PayloadClassTable& table, std::size_t bytes) noexcept {
  const int index = first_index_at_least(table, bytes);
  return index < 0 ? bytes : table.classes[static_cast<std::size_t>(index)];
}

[[nodiscard]] constexpr int class_index_of(const PayloadClassTable& table,
                                           std::size_t payload) noexcept {
  const int index = first_index_at_least(table, payload);
  if (index < 0) return -1;
  return table.classes[static_cast<std::size_t>(index)] == payload ? index : -1;
}

[[nodiscard]] constexpr PayloadClassTable persist_payload_class_table() {
  PayloadClassTable table;
  auto push = [&](std::size_t value) { push_payload_class(table, value); };
  push(16);
  push(32);
  push(64);
  push(128);
  for (std::size_t value = 256; value <= kSizeClassKiB; value += 128)
    push(value);
  for (std::size_t value = kSizeClassKiB + 256; value <= 4 * kSizeClassKiB;
       value += 256)
    push(value);
  for (std::size_t value = 8 * kSizeClassKiB; value <= 64 * kSizeClassKiB;
       value += 4 * kSizeClassKiB)
    push(value);
  for (std::size_t value = 128 * kSizeClassKiB; value <= kSizeClassMiB;
       value += 64 * kSizeClassKiB)
    push(value);
  for (std::size_t value = kSizeClassMiB + 256 * kSizeClassKiB;
       value <= 4 * kSizeClassMiB; value += 256 * kSizeClassKiB)
    push(value);
  append_large_payload_classes(table);
  return table;
}

[[nodiscard]] constexpr PayloadClassTable scratch_payload_class_table() {
  PayloadClassTable table;
  auto push = [&](std::size_t value) { push_payload_class(table, value); };
  push(16);
  push(32);
  push(64);
  push(128);
  for (std::size_t value = 256; value <= kSizeClassKiB; value += 128)
    push(value);
  for (std::size_t value = kSizeClassKiB + 256; value <= 4 * kSizeClassKiB;
       value += 256)
    push(value);
  push(8 * kSizeClassKiB);
  push(16 * kSizeClassKiB);
  push(32 * kSizeClassKiB);
  push(64 * kSizeClassKiB);
  push(128 * kSizeClassKiB);
  push(256 * kSizeClassKiB);
  push(512 * kSizeClassKiB);
  push(kSizeClassMiB);
  push(2 * kSizeClassMiB);
  push(4 * kSizeClassMiB);
  append_large_payload_classes(table);
  return table;
}

inline constexpr auto kPersistPayloadTable = persist_payload_class_table();
inline constexpr auto kScratchPayloadTable = scratch_payload_class_table();
inline constexpr auto kPersistPayloadClasses = kPersistPayloadTable.classes;
inline constexpr auto kScratchPayloadClasses = kScratchPayloadTable.classes;
inline constexpr std::size_t kPersistClassCount = kPersistPayloadTable.count;
inline constexpr std::size_t kScratchClassCount = kScratchPayloadTable.count;
inline constexpr std::size_t kMaxSizeClassLists =
    (kPersistClassCount > kScratchClassCount ? kPersistClassCount
                                             : kScratchClassCount) *
    kAlignmentClassCount;
static_assert(kPersistPayloadClasses.front() == 16);
static_assert(kPersistPayloadClasses[kPersistClassCount - 1] ==
              kMaxCachedPayload);
static_assert(kScratchPayloadClasses[kScratchClassCount - 1] ==
              kMaxCachedPayload);
static_assert(kMaxSizeClassLists >= kPersistClassCount * kAlignmentClassCount);

constexpr bool payload_table_sorted(const PayloadClassTable& table) noexcept {
  for (std::size_t index = 1; index < table.count; ++index) {
    if (table.classes[index] <= table.classes[index - 1]) return false;
  }
  return true;
}
static_assert(payload_table_sorted(kPersistPayloadTable));
static_assert(payload_table_sorted(kScratchPayloadTable));

// 直接以 16 字节粒度覆盖 TLS 范围，避免重复二分搜索。
constexpr auto small_class_lookup(const PayloadClassTable& table) {
  std::array<std::uint8_t, 4097> result{};
  std::size_t index = 0;
  for (std::size_t q = 0; q < result.size(); ++q) {
    while (table.classes[index] < q * 16) ++index;
    result[q] = static_cast<std::uint8_t>(index);
  }
  return result;
}
inline constexpr auto kPersistSmallIndex =
    small_class_lookup(kPersistPayloadTable);
inline constexpr auto kScratchSmallIndex =
    small_class_lookup(kScratchPayloadTable);
[[nodiscard]] constexpr int rounded_payload_index(PoolPurpose purpose,
                                                  std::size_t bytes) noexcept {
  if (bytes <= 65536) {
    const auto q = (bytes + 15) / 16;
    return purpose == PoolPurpose::Persist ? kPersistSmallIndex[q]
                                           : kScratchSmallIndex[q];
  }
  if (bytes > kMaxCachedPayload) return -1;
  if (bytes <= kMaxPooledPayload) {
    if (purpose == PoolPurpose::Scratch) {
      constexpr auto base =
          first_index_at_least(kScratchPayloadTable, 128 * kSizeClassKiB);
      return base + static_cast<int>(std::bit_width(bytes - 1)) - 17;
    }
    if (bytes <= kSizeClassMiB) {
      constexpr auto base =
          first_index_at_least(kPersistPayloadTable, 128 * kSizeClassKiB);
      return base + static_cast<int>((bytes - 1) / (64 * kSizeClassKiB)) - 1;
    }
    constexpr auto base = first_index_at_least(
        kPersistPayloadTable, kSizeClassMiB + 256 * kSizeClassKiB);
    return base + static_cast<int>((bytes - 1 - kSizeClassMiB) /
                                   (256 * kSizeClassKiB));
  }
  const auto base =
      purpose == PoolPurpose::Persist
          ? first_index_at_least(kPersistPayloadTable, 5 * kSizeClassMiB)
          : first_index_at_least(kScratchPayloadTable, 5 * kSizeClassMiB);
  if (bytes <= 16 * kSizeClassMiB)
    return base + static_cast<int>((bytes - 1) / kSizeClassMiB) - 4;
  return base + 12 +
         static_cast<int>((bytes - 1 - 16 * kSizeClassMiB) /
                          (2 * kSizeClassMiB));
}

[[nodiscard]] constexpr std::size_t payload_class(PoolPurpose purpose,
                                                  std::size_t bytes,
                                                  bool large_cache) noexcept {
  const std::size_t requested = bytes == 0 ? kMinSizeClass : bytes;
  if (requested > kMaxPooledPayload) {
    if (!large_cache || requested > kMaxCachedPayload) return requested;
  }
  const int index = rounded_payload_index(purpose, requested);
  if (index < 0) return requested;
  return purpose == PoolPurpose::Persist ? kPersistPayloadClasses[index]
                                         : kScratchPayloadClasses[index];
}

[[nodiscard]] constexpr int payload_class_index(PoolPurpose purpose,
                                                std::size_t payload) noexcept {
  const int index = rounded_payload_index(purpose, payload);
  if (index < 0) return -1;
  const auto value = purpose == PoolPurpose::Persist
                         ? kPersistPayloadClasses[index]
                         : kScratchPayloadClasses[index];
  return value == payload ? index : -1;
}

[[nodiscard]] constexpr bool size_class_cacheable(std::size_t payload,
                                                  std::size_t alignment,
                                                  bool large_cache) noexcept {
  if (alignment_class_index(alignment) < 0) return false;
  if (payload <= kMaxPooledPayload) return true;
  return large_cache && payload <= kMaxCachedPayload &&
         payload_class_index(PoolPurpose::Persist, payload) >= 0;
}

[[nodiscard]] constexpr int size_class_list_index(
    PoolPurpose purpose, std::size_t payload, std::size_t alignment) noexcept {
  const int payload_index = payload_class_index(purpose, payload);
  const int align_index = alignment_class_index(alignment);
  if (payload_index < 0 || align_index < 0) return -1;
  return payload_index * static_cast<int>(kAlignmentClassCount) + align_index;
}

[[nodiscard]] constexpr std::size_t size_class_list_count(
    PoolPurpose purpose) noexcept {
  const std::size_t classes =
      purpose == PoolPurpose::Persist ? kPersistClassCount : kScratchClassCount;
  return classes * kAlignmentClassCount;
}

[[nodiscard]] constexpr std::size_t payload_at_index(
    PoolPurpose purpose, int payload_index) noexcept {
  if (payload_index < 0) return 0;
  const auto index = static_cast<std::size_t>(payload_index);
  if (purpose == PoolPurpose::Persist) {
    return index < kPersistClassCount ? kPersistPayloadClasses[index] : 0;
  }
  return index < kScratchClassCount ? kScratchPayloadClasses[index] : 0;
}

[[nodiscard]] constexpr std::size_t alignment_at_index(
    int align_index) noexcept {
  if (align_index == 0) return 16;
  if (align_index == 1) return 32;
  if (align_index == 2) return 64;
  return 0;
}

static_assert(size_class_list_index(PoolPurpose::Persist, 8 * kSizeClassMiB,
                                    64) >= 0);
static_assert(size_class_list_index(PoolPurpose::Persist, 100 * kSizeClassMiB,
                                    64) >= 0);
static_assert(size_class_list_index(PoolPurpose::Persist, 300 * kSizeClassMiB,
                                    64) < 0);

}  // namespace mv::runtime
