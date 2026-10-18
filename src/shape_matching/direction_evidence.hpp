#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <memory_resource>
#include <span>
#include <vector>

namespace mv::shape_matching {

// 查询独占精确方向索引；零梯度不能通过正证据阈值，法向检查前按相同量化方向分组。
class DirectionEvidence {
 public:
  using BuildMaps = void (*)(const std::uint32_t*, std::size_t, const std::uint16_t*,
      std::size_t, const std::size_t*, const std::size_t*, int, bool, std::size_t,
      std::uint64_t*, void (*)(const void*), const void*);
  explicit DirectionEvidence(std::pmr::memory_resource* memory)
      : starts_(memory), positions_(memory), keys_(memory), maps_(memory) {}

  void build(std::span<const std::int8_t> nx, std::span<const std::int8_t> ny,
             std::span<const std::uint32_t> normals, int threshold, bool polarity,
             BuildMaps build_maps = nullptr, void (*poll)(const void*) = nullptr, const void* context = nullptr) {
    assert(nx.size() == ny.size() && threshold >= 0);
    words_ = (nx.size() + 63) / 64 + 1;
    starts_.assign(65537, 0);
    keys_.clear();
    auto key_at = [&](std::size_t i) {
      return static_cast<std::uint16_t>(static_cast<std::uint8_t>(ny[i])) |
          (static_cast<std::uint16_t>(static_cast<std::uint8_t>(nx[i])) << 8);
    };
    for (std::size_t i = 0; i < nx.size(); ++i) {
      if (poll && i % 16384 == 0) poll(context);
      if (key_at(i)) ++starts_[key_at(i) + 1];
    }
    for (std::size_t key = 0; key < 65536; ++key) {
      if (starts_[key + 1]) keys_.push_back(static_cast<std::uint16_t>(key));
      starts_[key + 1] += starts_[key];
    }
    positions_.resize(starts_.back());
    std::pmr::vector<std::size_t> cursor(starts_, starts_.get_allocator());
    for (std::size_t i = 0; i < nx.size(); ++i)
      if (key_at(i)) positions_[cursor[key_at(i)]++] = i;
    maps_.assign(normals.size() * words_, 0);
    if (build_maps) {
      build_maps(normals.data(), normals.size(), keys_.data(), keys_.size(),
          starts_.data(), positions_.data(), threshold, polarity, words_, maps_.data(), poll, context);
      return;
    }
    for (std::size_t n = 0; n < normals.size(); ++n) {
      if (poll) poll(context);
      const auto row = static_cast<std::int16_t>(normals[n] & 65535);
      const auto col = static_cast<std::int16_t>(normals[n] >> 16);
      auto* map = maps_.data() + n * words_;
      for (auto key : keys_) {
        const int dot = static_cast<std::int8_t>(key & 255) * row +
                        static_cast<std::int8_t>(key >> 8) * col;
        if ((polarity ? dot : (dot < 0 ? -dot : dot)) <= threshold) continue;
        for (auto j = starts_[key]; j < starts_[key + 1]; ++j) {
          const auto index = positions_[j];
          map[index / 64] |= std::uint64_t{1} << (index % 64);
        }
      }
    }
  }

  [[nodiscard]] std::uint64_t window(std::size_t normal, std::size_t index) const {
    const auto* map = maps_.data() + normal * words_;
    const auto shift = index % 64;
    auto bits = map[index / 64] >> shift;
    if (shift) bits |= map[index / 64 + 1] << (64 - shift);
    return bits;
  }

  [[nodiscard]] static std::uint64_t combine(const std::array<std::uint64_t, 4>& b, int need) {
    if (need <= 1) return b[0] | b[1] | b[2] | b[3];
    if (need == 2) return (b[0] & b[1]) | ((b[0] | b[1]) & (b[2] | b[3])) | (b[2] & b[3]);
    if (need == 3) return (b[0] & b[1] & (b[2] | b[3])) | ((b[0] | b[1]) & b[2] & b[3]);
    return need == 4 ? b[0] & b[1] & b[2] & b[3] : 0;
  }

 private:
  std::size_t words_{};
  std::pmr::vector<std::size_t> starts_, positions_;
  std::pmr::vector<std::uint16_t> keys_;
  std::pmr::vector<std::uint64_t> maps_;
};
}  // namespace mv::shape_matching
