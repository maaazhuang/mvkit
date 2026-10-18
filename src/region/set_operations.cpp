#include <region/operators.hpp>
#include <region/banded_morphology.hpp>
#include <runtime/memory/scratch_arena.hpp>

#include <map>
#include <vector>

namespace mv::regionproc {
namespace {
std::optional<mv::core::Region> rectangle_union(const mv::core::Regions& regions) {
  if (regions.size() < 8) return std::nullopt;
  struct Event { std::int64_t row, first, last; bool add; };
  std::vector<Event> events;
  std::vector<mv::core::RectI64> rectangles;
  for (std::size_t i = 0; i < regions.size(); ++i) {
    const auto bounds = regions[i].bounding_box();
    if (!bounds) continue;
    const auto width = std::uint64_t(bounds->col_max) - std::uint64_t(bounds->col_min) + 1;
    const auto height = std::uint64_t(bounds->row_max) - std::uint64_t(bounds->row_min) + 1;
    if (!width || !height || width > std::numeric_limits<std::uint64_t>::max() / height ||
        width * height != regions[i].area()) return std::nullopt;
    rectangles.push_back(*bounds);
    events.push_back({bounds->row_min, bounds->col_min, bounds->col_max, true});
    if (bounds->row_max != std::numeric_limits<std::int64_t>::max())
      events.push_back({bounds->row_max + 1, bounds->col_min, bounds->col_max, false});
  }
  if (events.empty()) return mv::core::Region{};
  // 不相交行条带无需活跃事件树，一次规范化列 profile；纵向重叠条带仍用事件扫描。
  std::sort(rectangles.begin(),rectangles.end(),[](const auto& a,const auto& b) {
    if(a.row_min!=b.row_min)return a.row_min<b.row_min;
    if(a.row_max!=b.row_max)return a.row_max<b.row_max;
    if(a.col_min!=b.col_min)return a.col_min<b.col_min;
    return a.col_max<b.col_max;
  });
  detail::RowBands strips;
  bool disjoint=true;
  for(std::size_t i=0;i<rectangles.size();) {
    const auto first=rectangles[i].row_min,last=rectangles[i].row_max;
    detail::BandProfile profile;
    do {
      detail::append_interval(profile,{rectangles[i].col_min,rectangles[i].col_max});
      ++i;
    } while(i<rectangles.size() && rectangles[i].row_min==first && rectangles[i].row_max==last);
    if(i<rectangles.size() && rectangles[i].row_min<=last) {disjoint=false;break;}
    detail::append_band(strips,first,last,std::move(profile));
  }
  if(disjoint)return detail::expand_bands(strips);
  std::sort(events.begin(), events.end(), [](const auto& a, const auto& b) {return a.row < b.row;});
  mv::runtime::ScratchArena scratch;
  std::pmr::map<std::pair<std::int64_t, std::int64_t>, std::size_t> active{scratch.resource()};
  detail::RowBands bands;
  const auto emit = [&](std::int64_t first, std::int64_t last) {
    detail::BandProfile profile;
    for (const auto& [interval, count] : active)
      detail::append_interval(profile, {interval.first, interval.second});
    detail::append_band(bands, first, last, std::move(profile));
  };
  auto previous = events.front().row;
  for (std::size_t i = 0; i < events.size();) {
    const auto row = events[i].row;
    if (previous < row) emit(previous, row - 1);
    do {
      const auto& event = events[i++];
      const auto key = std::pair{event.first, event.last};
      if (event.add) ++active[key];
      else {
        auto found = active.find(key);
        if (--found->second == 0) active.erase(found);
      }
    } while (i < events.size() && events[i].row == row);
    previous = row;
  }
  if (!active.empty()) emit(previous, std::numeric_limits<std::int64_t>::max());
  return detail::expand_bands(bands);
}

template <class Fn>
mv::core::Regions map_left(const mv::core::Regions& left, const mv::core::Region& mask, Fn&& fn) {
  std::vector<mv::core::Region> out;
  out.reserve(left.size());
  for (std::size_t index = 0; index < left.size(); ++index) out.push_back(fn(left[index], mask));
  return mv::core::Regions{std::move(out)};
}
}

mv::core::Region union1(const mv::core::Region& region) { return region; }
mv::core::Region union1(const mv::core::Regions& regions) {
  if (regions.size() == 1) return regions[0];
  if (auto result = rectangle_union(regions)) return *result;
  // 只规范化一次，不为每个对象复制不断增大的并集。
  std::vector<mv::core::RegionRun> runs;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const auto part = regions[index].runs();
    runs.insert(runs.end(), part.begin(), part.end());
  }
  return mv::core::Region::from_runs(runs);
}

mv::core::Region union2(const mv::core::Region& left, const mv::core::Region& right) { return left.united(right); }
mv::core::Region union2(const mv::core::Region& left, const mv::core::Regions& right) { return left.united(union1(right)); }
mv::core::Regions union2(const mv::core::Regions& left, const mv::core::Region& right) {
  return map_left(left, right, [](const auto& a, const auto& b) { return a.united(b); });
}
mv::core::Regions union2(const mv::core::Regions& left, const mv::core::Regions& right) {
  return union2(left, union1(right));
}

mv::core::Region intersection(const mv::core::Region& left, const mv::core::Region& right) { return left.intersect(right); }
mv::core::Region intersection(const mv::core::Region& left, const mv::core::Regions& right) { return left.intersect(union1(right)); }
mv::core::Regions intersection(const mv::core::Regions& left, const mv::core::Region& right) {
  return map_left(left, right, [](const auto& a, const auto& b) { return a.intersect(b); });
}
mv::core::Regions intersection(const mv::core::Regions& left, const mv::core::Regions& right) {
  return intersection(left, union1(right));
}

mv::core::Region difference(const mv::core::Region& left, const mv::core::Region& right) { return left.difference(right); }
mv::core::Region difference(const mv::core::Region& left, const mv::core::Regions& right) { return left.difference(union1(right)); }
mv::core::Regions difference(const mv::core::Regions& left, const mv::core::Region& right) {
  return map_left(left, right, [](const auto& a, const auto& b) { return a.difference(b); });
}
mv::core::Regions difference(const mv::core::Regions& left, const mv::core::Regions& right) {
  return difference(left, union1(right));
}

}  // namespace mv::regionproc
