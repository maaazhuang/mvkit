#include <region/extended_common.hpp>
#include <numeric>

namespace mv::regionproc {
namespace {
using namespace extended;
std::size_t count_parts(std::size_t length, double target, bool halcon_layout=false) {
  const double parts=std::floor(double(length)/target+.5);
  if(halcon_layout) {
    check(std::isfinite(parts)&&parts<=INT32_MAX,"partition_rectangle","Width/Height","partition count must fit Int32");
    return static_cast<std::size_t>(std::max(1.,parts));
  }
  return static_cast<std::size_t>(std::clamp(parts,1.,double(length)));
}
void append_part(std::vector<Region>& out, const Region& region, RectI64 box) {
  auto part = region.clipped(box);
  if (!part.empty()) out.push_back(std::move(part));
}
}  // namespace
mv::core::Regions partition_rectangle(const mv::core::Regions& regions,
                                      double width, double height, bool halcon_layout) {
  check(std::isfinite(width) && (halcon_layout?width>0:width>=1) && std::isfinite(height) &&
            (halcon_layout?height>0:height>=1),
        "partition_rectangle", "Width/Height", halcon_layout?"finite positive sizes required":"finite sizes >=1 required");
  std::vector<Region> out;
  for (std::size_t k = 0; k < regions.size(); ++k) {
    const auto& r = regions[k];
    const auto box = r.bounding_box();
    if (!box) {
      out.push_back(r);
      continue;
    }
    const auto w = extent(box->col_min, box->col_max, "partition_rectangle"),
               h = extent(box->row_min, box->row_max, "partition_rectangle");
    const auto nx = count_parts(w, width,halcon_layout), ny = count_parts(h, height,halcon_layout);
    // 框内规范覆盖证明每单元满；两跨度已满足原 Int32 守卫，w*h 在 UInt64 内。
    const bool rectangle_profile = (nx > 1 || ny > 1) && r.area() == std::uint64_t(w) * h;
    const auto boundary = [](std::size_t i, std::size_t length, std::size_t count) {
      return static_cast<std::size_t>(std::floor(double(i) * double(length) /
                                                  double(count) + .5));
    };
    // 只批量构造严格非空的满矩形网格；零宽单元及超大网格保留原裁剪和错误顺序。
    if (rectangle_profile && nx <= w && ny <= h && nx <= 4096 / ny) {
      std::vector<RectI64> boxes;
      boxes.reserve(nx * ny);
      for (std::size_t y = 0; y < ny; ++y)
        for (std::size_t x = 0; x < nx; ++x)
          boxes.push_back({add(box->row_min, boundary(y, h, ny)),
                           add(box->col_min, boundary(x, w, nx)),
                           add(box->row_min, boundary(y + 1, h, ny) - 1),
                           add(box->col_min, boundary(x + 1, w, nx) - 1)});
      auto cells = Region::rectangles(boxes);
      out.insert(out.end(), std::make_move_iterator(cells.begin()),
                 std::make_move_iterator(cells.end()));
      continue;
    }
    for (std::size_t y = 0; y < ny; ++y) {
      if(halcon_layout&&boundary(y,h,ny)>=h)break;
      for (std::size_t x = 0; x < nx; ++x) {
        const RectI64 part_box{
            add(box->row_min, boundary(y, h, ny)),
            add(box->col_min, boundary(x, w, nx)),
            add(box->row_min, boundary(y + 1, h, ny) - 1),
            add(box->col_min, boundary(x + 1, w, nx) - 1)};
        if (rectangle_profile && part_box.row_min <= part_box.row_max &&
            part_box.col_min <= part_box.col_max)
          out.push_back(Region::rectangle(part_box));
        else if(halcon_layout)out.push_back(r.clipped(part_box));
        else
          append_part(out, r, part_box);
      }
    }
  }
  return Regions{std::move(out)};
}
mv::core::Regions partition_dynamic(const mv::core::Regions& regions,
                                    double distance, double percent, bool halcon_layout) {
  check(std::isfinite(distance) && distance >= 1 && std::isfinite(percent) &&
            percent >= 0 && percent <= 100,
        "partition_dynamic", "Distance/Percent",
        "distance >=1 and percent in [0,100] required");
  std::vector<Region> out;
  for (std::size_t k = 0; k < regions.size(); ++k) {
    const auto& r = regions[k];
    const auto box = r.bounding_box();
    if (!box) {
      out.push_back(r);
      continue;
    }
    const auto w = extent(box->col_min, box->col_max, "partition_dynamic"),
               n = count_parts(w, distance);
    if (n == 1) {
      out.push_back(r);
      continue;
    }
    const auto rows=std::uint64_t(box->row_max)-std::uint64_t(box->row_min)+1;
    const bool rectangle_profile=rows && rows<=UINT64_MAX/w && r.area()==rows*w;
    ExecutionContext context;
    std::pmr::vector<std::int64_t> top(context.scratch_resource()), bottom(context.scratch_resource());
    std::pmr::vector<unsigned char> occupied(context.scratch_resource());
    std::pmr::vector<std::int64_t> column_area(context.scratch_resource());
    if (!rectangle_profile && !halcon_layout) {
      top.assign(w,INT64_MAX);bottom.assign(w,INT64_MIN);occupied.assign(w,0);
    if (r.area() / w >= 8) {
      // 规范行递增，首末覆盖精确给出旧列 min/max；后继链接使各列只访问一次。
      std::pmr::vector<std::uint32_t> next(w+1,context.scratch_resource());
      std::iota(next.begin(),next.end(),std::uint32_t{0});
      const auto find = [&](std::uint32_t x) {
        auto root=x;
        while (next[root]!=root) root=next[root];
        while (next[x]!=x) {const auto parent=next[x];next[x]=root;x=parent;}
        return root;
      };
      const auto input=r.runs();
      const auto visit = [&](const RegionRun& run,bool first) {
        const auto begin=static_cast<std::uint32_t>(std::uint64_t(run.col_begin)-std::uint64_t(box->col_min));
        const auto end=static_cast<std::uint32_t>(std::uint64_t(run.col_end)-std::uint64_t(box->col_min));
        auto x=find(begin);
        while (x<=end) {
          if (first) {top[x]=run.row;occupied[x]=1;} else bottom[x]=run.row;
          const auto successor=find(x+1);next[x]=successor;x=successor;
        }
      };
      for (const auto& run:input) visit(run,true);
      std::iota(next.begin(),next.end(),std::uint32_t{0});
      for (auto it=input.rbegin();it!=input.rend();++it) visit(*it,false);
    } else {
    for (const auto& run : r.runs())
      for (auto x = static_cast<std::size_t>(std::uint64_t(run.col_begin) -
                                             std::uint64_t(box->col_min));
           x <= std::uint64_t(run.col_end) - std::uint64_t(box->col_min); ++x) {
        top[x] = std::min<std::int64_t>(top[x], run.row);
        bottom[x] = std::max<std::int64_t>(bottom[x], run.row);
        occupied[x] = 1;
      }
    }
    }
    // HALCON 搜索实际列面积，包含孔洞及不相连游程；差分事件在 O(runs+width) 计算，无需栅格化。
    if(halcon_layout&&!rectangle_profile) {
      column_area.assign(w+1,0);
      for(const auto& run:r.runs()) {
        const auto first=static_cast<std::size_t>(std::uint64_t(run.col_begin)-std::uint64_t(box->col_min));
        const auto after=static_cast<std::size_t>(std::uint64_t(run.col_end)-std::uint64_t(box->col_min)+1);
        ++column_area[first];--column_area[after];
      }
      for(std::size_t i=1;i<w;++i)column_area[i]+=column_area[i-1];
    }
    const auto radius = static_cast<std::size_t>(
        std::min(double(w), std::floor(distance * (percent * .01) + .5)));
    std::size_t previous = 0;
    bool finished = false;
    for (std::size_t i = 1; i < n; ++i) {
      const auto initial = static_cast<std::size_t>(
          std::floor(double(i) * double(w) / double(n) + .5));
      if(halcon_layout && initial<previous)continue;
      const auto lo = std::max(
          halcon_layout?std::size_t{0}:previous, initial > radius ? initial - radius : std::size_t{0});
      // w 处虚拟列高度为零，可消费最终切点。
      const auto hi = std::min(w, initial + radius);
      std::size_t best = lo;
      double score = INFINITY;
      const auto gap = [&](std::size_t x) {
        return x > initial ? x - initial : initial - x;
      };
      for (auto x = lo; x <= hi; ++x) {
        const double value =
            x < w && rectangle_profile
                ? double(std::uint64_t(box->row_max)-std::uint64_t(box->row_min))+1
                : halcon_layout && x<w ? double(column_area[x])
                : x < w && occupied[x]
                    ? double(std::uint64_t(bottom[x])-std::uint64_t(top[x]))+1 : 0;
        if (value < score || (value == score && (gap(x) < gap(best) || (halcon_layout && gap(x)==gap(best) && x>best)))) {
          best = x;
          score = value;
        }
      }
      if(halcon_layout && best<previous)continue;
      const RectI64 part_box{box->row_min,add(box->col_min,previous),box->row_max,
                             add(box->col_min,std::min(best,w-1))};
      if (rectangle_profile) out.push_back(Region::rectangle(part_box));
      else if(halcon_layout)out.push_back(r.clipped(part_box));
      else append_part(out,r,part_box);
      if (best == w) {
        finished = true;
        break;
      }
      previous = best + 1;
      if (previous == w) {
        // 末实际列上的切点保留一个空尾对象。
        out.emplace_back();
        finished = true;
        break;
      }
    }
    if (!finished) {
      const RectI64 part_box{box->row_min,add(box->col_min,previous),box->row_max,box->col_max};
      if (rectangle_profile) out.push_back(Region::rectangle(part_box));
      else if(halcon_layout)out.push_back(r.clipped(part_box));
      else append_part(out,r,part_box);
    }
  }
  return Regions{std::move(out)};
}
mv::core::Regions sort_characters(const mv::core::Regions& regions, SortOrder order,
                                  SortAxis axis, double overlap_fraction, bool coordinate_overlap) {
  ExecutionContext context;
  auto* memory = context.scratch_resource();
  struct Box {
    std::int64_t lo, hi, secondary;
    std::size_t index;
  };
  std::pmr::vector<Box> boxes{memory};
  std::pmr::vector<std::size_t> empty{memory};
  for (std::size_t i = 0; i < regions.size(); ++i) {
    const auto b = regions[i].bounding_box();
    if (!b) {
      empty.push_back(i);
      continue;
    }
    boxes.push_back(axis == SortAxis::Row ? Box{b->row_min, b->row_max, b->col_min, i}
                                          : Box{b->col_min, b->col_max, b->row_min, i});
  }
  std::pmr::vector<std::size_t> parent(boxes.size(), 0, memory);
  std::iota(parent.begin(), parent.end(), 0);
  auto root = [&](std::size_t i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  };
  std::pmr::vector<std::size_t> by_lo(boxes.size(), 0, memory);
  std::iota(by_lo.begin(), by_lo.end(), 0);
  std::sort(by_lo.begin(), by_lo.end(), [&](auto i, auto j) {
    if (boxes[i].lo != boxes[j].lo) return boxes[i].lo < boxes[j].lo;
    if (boxes[i].hi != boxes[j].hi) return boxes[i].hi < boxes[j].hi;
    return boxes[i].index < boxes[j].index;
  });
  // 相同主区间总按较短跨度完全重叠，与其他区间连接相同；
  // 各等价类只比较一个节点，其他成员直接挂接。
  std::size_t unique_count = 0;
  for (std::size_t first = 0; first < by_lo.size();) {
    const auto representative = by_lo[first];
    auto last = first + 1;
    while (overlap_fraction < 1 && last < by_lo.size() && boxes[by_lo[last]].lo == boxes[representative].lo &&
           boxes[by_lo[last]].hi == boxes[representative].hi) {
      parent[by_lo[last]] = representative;
      ++last;
    }
    by_lo[unique_count++] = representative;
    first = last;
  }
  by_lo.resize(unique_count);
  // 取商后 root ID 可不同，但不同分量不能有相同最小 lo：同起点区间必按较短跨度完全相连。
  // 因此最终比较器的 root-ID 同值分支对不同有效分量不可达，组/次序/lo/原索引的可观测排序不变。
  for (std::size_t a = 0; a < by_lo.size(); ++a) {
    const auto i = by_lo[a];
    for (std::size_t b = a + 1; b < by_lo.size(); ++b) {
      const auto j = by_lo[b];
      if (boxes[j].lo > boxes[i].hi) break;
      // 已连接分量内增加边不会改变分组。
      if (root(i) == root(j)) continue;
      const auto lo = std::max(boxes[i].lo, boxes[j].lo), hi = std::min(boxes[i].hi, boxes[j].hi);
      if (lo > hi) continue;
      const double overlap = double(std::uint64_t(hi) - std::uint64_t(lo)) + (coordinate_overlap?0:1);
      const double span =
          std::min(double(std::uint64_t(boxes[i].hi) - std::uint64_t(boxes[i].lo)) + (coordinate_overlap?0:1),
                   double(std::uint64_t(boxes[j].hi) - std::uint64_t(boxes[j].lo)) + (coordinate_overlap?0:1));
      if (overlap > overlap_fraction * span || (coordinate_overlap&&span==0&&lo==hi&&overlap_fraction<1)) parent[root(j)] = root(i);
    }
  }
  std::pmr::vector<std::int64_t> group_min(boxes.size(), INT64_MAX, memory);
  for (std::size_t i = 0; i < boxes.size(); ++i)
    group_min[root(i)] = std::min(group_min[root(i)], boxes[i].lo);
  std::pmr::vector<std::size_t> indices(boxes.size(), 0, memory);
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](auto i, auto j) {
    const auto a = root(i), b = root(j);
    int cmp = 0;
    if (group_min[a] != group_min[b])
      cmp = group_min[a] < group_min[b] ? -1 : 1;
    else if (a != b)
      cmp = a < b ? -1 : 1;
    else if (boxes[i].secondary != boxes[j].secondary)
      cmp = boxes[i].secondary < boxes[j].secondary ? -1 : 1;
    else if (boxes[i].lo != boxes[j].lo)
      cmp = boxes[i].lo < boxes[j].lo ? -1 : 1;
    if (!cmp) return boxes[i].index < boxes[j].index;
    return order == SortOrder::Ascending ? cmp < 0 : cmp > 0;
  });
  std::vector<Region> out;
  out.reserve(regions.size());
  for (auto i : indices) out.push_back(regions[boxes[i].index]);
  for (auto i : empty) out.push_back(regions[i]);
  return Regions{std::move(out)};
}
mv::core::Regions select_rectangles(const mv::core::Regions& regions,
                                    StandardShape shape, double percent) {
  std::vector<Region> out;
  for (std::size_t i = 0; i < regions.size(); ++i) {
    const auto& r = regions[i];
    if (r.empty()) continue;
    double area;
    if (shape == StandardShape::Rectangle1) {
      const auto b = *r.bounding_box();
      area = (double(std::uint64_t(b.row_max) - std::uint64_t(b.row_min)) + 1) *
             (double(std::uint64_t(b.col_max) - std::uint64_t(b.col_min)) + 1);
    } else {
      geometry_bounds(r, "select_shape_std");
      const auto b = smallest_rectangle2(r);
      area = (2 * b.length1 + 1) * (2 * b.length2 + 1);
    }
    if (100 * std::min(1., double(r.area()) / area) > percent)
      out.push_back(r);
  }
  return Regions{std::move(out)};
}
}  // namespace mv::regionproc
