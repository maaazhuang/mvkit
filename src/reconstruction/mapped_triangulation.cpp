#include "mapped_triangulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <vector>

namespace mv::reconstruction {
namespace {
constexpr auto op = "triangulate_object_model_3d";
constexpr auto none = (std::numeric_limits<std::size_t>::max)();
using Triangle = std::array<std::size_t, 3>;
struct XY {
  long double x, y;
};
long double orient(XY a, XY b, XY c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
long double incircle(XY a, XY b, XY c, XY p) {
  a = {a.x - p.x, a.y - p.y};
  b = {b.x - p.x, b.y - p.y};
  c = {c.x - p.x, c.y - p.y};
  return (a.x * a.x + a.y * a.y) * (b.x * c.y - b.y * c.x) -
         (b.x * b.x + b.y * b.y) * (a.x * c.y - a.y * c.x) +
         (c.x * c.x + c.y * c.y) * (a.x * b.y - a.y * b.x);
}
bool valid(const ObjectModel3DData& m, std::size_t i) {
  const auto p = m.points[i];
  return (m.valid.empty() || m.valid[i]) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
[[noreturn]] void bad(const char* parameter, const char* message) {
  throw core::Exception(core::Status::InvalidArgument, core::ErrorCode::InvalidValue, op, parameter, -1,
                        message);
}

// 仅对孔洞边界三角化；完整栅格单元走下方线性路径，避免大组织点云全局二次 Delaunay 遍历。
std::vector<Triangle> delaunay(std::vector<XY> points) {
  const auto n = points.size();
  if (n < 3) return {};
  long double scale = 1;
  for (const auto p : points) scale = std::max(scale, std::max(std::abs(p.x), std::abs(p.y)) + 1);
  points.push_back({-8 * scale, -8 * scale});
  points.push_back({8 * scale, -8 * scale});
  points.push_back({0, 8 * scale});
  std::vector<Triangle> triangles{{n, n + 1, n + 2}};
  for (std::size_t i = 0; i < n; ++i) {
    std::map<std::pair<std::size_t, std::size_t>, unsigned> edges;
    std::vector<Triangle> retained;
    for (const auto t : triangles) {
      // 严格共圆同值保留较早对角线；栅格坐标为整数，不用固定尺度 epsilon。
      // MSVC long double 仅 Double 精度，大坐标谓词需独立审查。
      if (incircle(points[t[0]], points[t[1]], points[t[2]], points[i]) > 0) {
        for (unsigned k = 0; k < 3; ++k) {
          const auto a = t[k], b = t[(k + 1) % 3];
          ++edges[{std::min(a, b), std::max(a, b)}];
        }
      } else
        retained.push_back(t);
    }
    for (const auto& [edge, count] : edges)
      if (count == 1) {
        Triangle t{edge.first, edge.second, i};
        const auto direction = orient(points[t[0]], points[t[1]], points[t[2]]);
        if (direction == 0) continue;
        if (direction < 0) std::swap(t[0], t[1]);
        retained.push_back(t);
      }
    triangles = std::move(retained);
  }
  triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
                                 [n](const Triangle& t) { return t[0] >= n || t[1] >= n || t[2] >= n; }),
                  triangles.end());
  return triangles;
}

using Corner = std::pair<std::size_t, std::size_t>;
struct BoundaryEdge {
  Corner from, to;
  std::size_t slot;
};
// 追踪缺失像素边界后，用相邻有效像素中心替代各边；
// 这种对偶轮廓保留孔角周围三角形，区别于按缺失槽质心裁剪。
std::vector<std::vector<std::size_t>> hole_outlines(const std::vector<std::size_t>& component,
                                                    const std::vector<std::size_t>& slots,
                                                    std::size_t width) {
  std::vector<BoundaryEdge> edges;
  std::map<Corner, std::vector<std::size_t>> outgoing;
  auto add = [&](Corner a, Corner b, std::size_t slot) {
    outgoing[a].push_back(edges.size());
    edges.push_back({a, b, slot});
  };
  for (const auto slot : component) {
    const auto r = slot / width, c = slot % width;
    if (slots[slot - width] != none) add({r, c}, {r, c + 1}, slot - width);
    if (slots[slot + 1] != none) add({r, c + 1}, {r + 1, c + 1}, slot + 1);
    if (slots[slot + width] != none) add({r + 1, c + 1}, {r + 1, c}, slot + width);
    if (slots[slot - 1] != none) add({r + 1, c}, {r, c}, slot - 1);
  }
  std::vector<std::uint8_t> used(edges.size(), 0);
  std::vector<std::vector<std::size_t>> result;
  for (std::size_t seed = 0; seed < edges.size(); ++seed) {
    if (used[seed]) continue;
    std::vector<std::size_t> polygon;
    auto current = seed;
    for (;;) {
      if (used[current])
        throw core::Exception(core::Status::InvalidState, op, "hole contour is not a closed boundary");
      used[current] = 1;
      const auto e = edges[current];
      if (polygon.empty() || polygon.back() != e.slot) polygon.push_back(e.slot);
      if (e.to == edges[seed].from) break;
      const auto found = outgoing.find(e.to);
      if (found == outgoing.end())
        throw core::Exception(core::Status::InvalidState, op, "hole contour has an open endpoint");
      const int dy = e.to.first > e.from.first ? 1 : e.to.first < e.from.first ? -1 : 0;
      const int dx = e.to.second > e.from.second ? 1 : e.to.second < e.from.second ? -1 : 0;
      unsigned best_rank = 5;
      std::size_t best = none;
      for (auto j : found->second)
        if (!used[j]) {
          const auto f = edges[j];
          const int fy = f.to.first > f.from.first ? 1 : f.to.first < f.from.first ? -1 : 0;
          const int fx = f.to.second > f.from.second ? 1 : f.to.second < f.from.second ? -1 : 0;
          const int cross = dx * fy - dy * fx, dot = dx * fx + dy * fy;
          const unsigned rank = cross > 0 ? 0 : dot > 0 ? 1 : cross < 0 ? 2 : 3;
          if (rank < best_rank) {
            best_rank = rank;
            best = j;
          }
        }
      if (best == none) throw core::Exception(core::Status::InvalidState, op, "hole contour cannot continue");
      current = best;
    }
    if (polygon.size() > 1 && polygon.front() == polygon.back()) polygon.pop_back();
    result.push_back(std::move(polygon));
  }
  return result;
}

bool crosses(XY a, XY b, XY c, XY d) {
  const auto ab_c = orient(a, b, c), ab_d = orient(a, b, d);
  const auto cd_a = orient(c, d, a), cd_b = orient(c, d, b);
  return ((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0)) &&
         ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0));
}
bool contains(XY p, const std::vector<std::size_t>& polygon, const std::vector<XY>& points) {
  bool inside = false;
  for (std::size_t j = 0, k = polygon.size() - 1; j < polygon.size(); k = j++) {
    const auto a = points[polygon[j]], b = points[polygon[k]];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

std::vector<Triangle> ordered_polygon(const std::vector<std::size_t>& polygon,
                                      const std::vector<XY>& points) {
  auto remaining = polygon;
  std::vector<Triangle> result;
  while (remaining.size() > 3) {
    bool removed = false;
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      const auto a = remaining[(i + remaining.size() - 1) % remaining.size()];
      const auto b = remaining[i], c = remaining[(i + 1) % remaining.size()];
      if (orient(points[a], points[b], points[c]) <= 0) continue;
      bool occupied = false;
      for (auto j : remaining)
        if (j != a && j != b && j != c) {
          if (orient(points[a], points[b], points[j]) >= 0 && orient(points[b], points[c], points[j]) >= 0 &&
              orient(points[c], points[a], points[j]) >= 0) {
            occupied = true;
            break;
          }
        }
      if (occupied) continue;
      result.push_back({b, c, a});
      remaining.erase(remaining.begin() + i);
      removed = true;
      break;
    }
    if (!removed) throw core::Exception(core::Status::InvalidState, op, "hole polygon has no valid ear");
  }
  if (remaining.size() == 3 && orient(points[remaining[0]], points[remaining[1]], points[remaining[2]]) > 0)
    result.push_back({remaining[0], remaining[1], remaining[2]});
  return result;
}

void constrain_edges(std::vector<Triangle>& triangles, const std::vector<XY>& points,
                     const std::vector<std::vector<std::size_t>>& polygons) {
  using Edge = std::pair<std::size_t, std::size_t>;
  std::set<Edge> locked;
  for (const auto& polygon : polygons)
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto a = polygon[i], b = polygon[(i + 1) % polygon.size()];
      const Edge wanted{std::min(a, b), std::max(a, b)};
      if (a == b || locked.count(wanted)) continue;
      bool present = false;
      // 边界线段连接相邻栅格中心，包含角对角；翻转穿越的凸内部边以插入约束。
      for (std::size_t step = 0; step < triangles.size() * (points.size() + 1); ++step) {
        std::map<Edge, std::vector<std::size_t>> owners;
        for (std::size_t t = 0; t < triangles.size(); ++t)
          for (unsigned j = 0; j < 3; ++j) {
            auto u = triangles[t][j], v = triangles[t][(j + 1) % 3];
            owners[{std::min(u, v), std::max(u, v)}].push_back(t);
          }
        if (owners.count(wanted)) {
          present = true;
          break;
        }
        bool flipped = false;
        for (const auto& [edge, faces] : owners) {
          if (faces.size() != 2 || locked.count(edge) ||
              !crosses(points[a], points[b], points[edge.first], points[edge.second]))
            continue;
          std::size_t u = none, v = none;
          for (auto j : triangles[faces[0]])
            if (j != edge.first && j != edge.second) u = j;
          for (auto j : triangles[faces[1]])
            if (j != edge.first && j != edge.second) v = j;
          if (!crosses(points[u], points[v], points[edge.first], points[edge.second])) continue;
          Triangle first{u, v, edge.first}, second{v, u, edge.second};
          if (orient(points[first[0]], points[first[1]], points[first[2]]) < 0) std::swap(first[0], first[1]);
          if (orient(points[second[0]], points[second[1]], points[second[2]]) < 0)
            std::swap(second[0], second[1]);
          triangles[faces[0]] = first;
          triangles[faces[1]] = second;
          flipped = true;
          break;
        }
        if (!flipped) break;
      }
      if (!present)
        throw core::Exception(core::Status::InvalidState, op, "cannot insert a hole boundary constraint");
      locked.insert(wanted);
    }
}
}  // namespace

ObjectModel3DData triangulate_mapping(const ObjectModel3DData& model,
                                      const MappedTriangulationOptions& options) {
  validate_object_model_3d(model);
  constexpr double half_pi = 1.57079632679489661923;
  if (!std::isfinite(options.maximum_hole_area) || options.maximum_hole_area < 0)
    bad("xyz_mapping_max_area_holes", "finite nonnegative area required");
  if (options.maximum_view_angle < 0 || options.maximum_view_angle > half_pi)
    bad("xyz_mapping_max_view_angle", "angle in [0,pi/2] or NaN required");
  auto view = options.view_direction;
  const auto view_length = std::hypot(view.x, view.y, view.z);
  if (!(view_length > 0) || !std::isfinite(view_length))
    bad("xyz_mapping_max_view_dir", "finite nonzero direction required");
  view = {view.x / view_length, view.y / view_length, view.z / view_length};
  const auto width = model.mapping_width ? model.mapping_width : model.width;
  const auto height = model.mapping_height ? model.mapping_height : model.height;
  if (!width || !height) throw core::Exception(core::Status::InvalidState, op, "2D mapping required");
  if (height > none / width)
    throw core::Exception(core::Status::OutOfRange, op, "mapping dimensions overflow");
  const auto area = width * height;
  std::vector<std::size_t> slots(area, none), original(model.points.size(), none);
  std::vector<std::size_t> source_slot(model.points.size(), none);
  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < model.points.size(); ++i)
    if (valid(model, i)) {
      const auto slot = model.source_indices.empty() ? i : model.source_indices[i];
      if (slot >= area) bad("ObjectModel3D", "2D mapping index outside raster");
      if (slots[slot] != none) bad("ObjectModel3D", "duplicate 2D mapping coordinate");
      slots[slot] = i;
      source_slot[i] = slot;
      original[i] = valid_count++;
    }
  if (!valid_count) throw core::Exception(core::Status::InvalidState, op, "point mapping required");

  std::vector<Triangle> faces;
  faces.reserve((width - 1) * (height - 1) * 2);
  for (std::size_t r = 0; r + 1 < height; ++r)
    for (std::size_t c = 0; c + 1 < width; ++c) {
      const auto tl = slots[r * width + c], tr = slots[r * width + c + 1];
      const auto bl = slots[(r + 1) * width + c], br = slots[(r + 1) * width + c + 1];
      if (tl != none && tr != none && bl != none && br != none) {
        faces.push_back({br, bl, tl});
        faces.push_back({tr, br, tl});
      } else {
        if (tl != none && tr != none && bl != none) faces.push_back({tr, bl, tl});
        if (tl != none && tr != none && br != none) faces.push_back({tr, br, tl});
        if (tl != none && bl != none && br != none) faces.push_back({br, bl, tl});
        if (tr != none && bl != none && br != none) faces.push_back({tr, br, bl});
      }
    }

  std::vector<std::uint8_t> visited(area, 0);
  std::vector<std::size_t> component;
  std::set<Triangle> seen;
  bool built_seen = false;
  if (options.maximum_hole_area > 0)
    for (std::size_t seed = 0; seed < area; ++seed) {
      if (slots[seed] != none || visited[seed]) continue;
      component.clear();
      component.push_back(seed);
      visited[seed] = 1;
      bool touches_border = false;
      std::set<std::size_t> boundary;
      for (std::size_t next = 0; next < component.size(); ++next) {
        const auto slot = component[next], r = slot / width, c = slot % width;
        if (!r || !c || r + 1 == height || c + 1 == width) {
          touches_border = true;
          boundary.clear();
        }
        const std::array<std::size_t, 4> neighbors{r ? slot - width : none, c ? slot - 1 : none,
                                                   r + 1 < height ? slot + width : none,
                                                   c + 1 < width ? slot + 1 : none};
        for (const auto j : neighbors)
          if (j != none) {
            if (slots[j] != none) {
              if (!touches_border) boundary.insert(j);
            } else if (!visited[j]) {
              visited[j] = 1;
              component.push_back(j);
            }
          }
      }
      if (touches_border || component.size() > options.maximum_hole_area) continue;
      if (!built_seen) {
        for (auto t : faces) {
          std::sort(t.begin(), t.end());
          seen.insert(t);
        }
        built_seen = true;
      }
      std::vector<std::size_t> indices(boundary.begin(), boundary.end());
      std::vector<XY> positions;
      for (auto slot : indices)
        positions.push_back({static_cast<long double>(slot % width), static_cast<long double>(slot / width)});
      auto patch = delaunay(std::move(positions));
      const auto outlines = hole_outlines(component, slots, width);
      std::vector<std::vector<std::size_t>> polygons;
      for (const auto& outline : outlines) {
        std::vector<std::size_t> polygon;
        for (const auto slot : outline)
          polygon.push_back(std::lower_bound(indices.begin(), indices.end(), slot) - indices.begin());
        if (polygon.size() >= 3) polygons.push_back(std::move(polygon));
      }
      std::vector<XY> coordinates;
      for (auto slot : indices)
        coordinates.push_back(
            {static_cast<long double>(slot % width), static_cast<long double>(slot / width)});
      const bool ordered_patch = polygons.size() == 1 && polygons[0].size() == indices.size();
      if (ordered_patch)
        patch = ordered_polygon(polygons[0], coordinates);
      else
        constrain_edges(patch, coordinates, polygons);
      std::vector<Triangle> added;
      for (auto t : patch) {
        XY center{};
        for (auto j : t) {
          center.x += coordinates[j].x / 3;
          center.y += coordinates[j].y / 3;
        }
        bool inside = false;
        for (const auto& polygon : polygons)
          if (contains(center, polygon, coordinates)) inside = !inside;
        if (!inside) continue;
        for (auto& j : t) j = slots[indices[j]];
        auto key = t;
        std::sort(key.begin(), key.end());
        if (!seen.insert(key).second) continue;
        unsigned first = 0;
        for (unsigned j = 1; j < 3; ++j) {
          const auto a = source_slot[t[j]], b = source_slot[t[first]];
          if (a / width < b / width || (a / width == b / width && a % width > b % width)) first = j;
        }
        if (!ordered_patch) std::rotate(t.begin(), t.begin() + first, t.end());
        added.push_back(t);
      }
      if (!ordered_patch)
        std::sort(added.begin(), added.end(), [&](const Triangle& a, const Triangle& b) {
          auto aa = a, bb = b;
          std::sort(aa.begin(), aa.end());
          std::sort(bb.begin(), bb.end());
          return aa < bb;
        });
      faces.insert(faces.end(), added.begin(), added.end());
    }
  if (faces.empty())
    throw core::Exception(core::Status::InvalidState, op, "2D mapping cannot form triangles");

  if (options.filter_view_angle) {
    const auto cosine = std::cos(options.maximum_view_angle);
    faces.erase(
        std::remove_if(faces.begin(), faces.end(),
                       [&](const Triangle& t) {
                         const auto a = model.points[t[0]], b = model.points[t[1]], c = model.points[t[2]];
                         const Point3d u{b.x - a.x, b.y - a.y, b.z - a.z}, v{c.x - a.x, c.y - a.y, c.z - a.z};
                         const Point3d n{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
                         const auto length = std::hypot(n.x, n.y, n.z);
                         return !(length > 0 &&
                                  std::abs(n.x * view.x + n.y * view.y + n.z * view.z) >= cosine * length);
                       }),
        faces.end());
  }
  std::vector<std::uint8_t> keep(model.points.size(), 0);
  if (options.output_all_points) {
    for (std::size_t i = 0; i < keep.size(); ++i) keep[i] = valid(model, i);
  } else
    for (auto t : faces)
      for (auto i : t) keep[i] = 1;
  std::vector<std::size_t> ids, remap(model.points.size(), none);
  std::vector<double> original_indices;
  for (std::size_t i = 0; i < keep.size(); ++i)
    if (keep[i]) {
      remap[i] = ids.size();
      ids.push_back(i);
      original_indices.push_back(static_cast<double>(original[i]));
    }
  if (ids.size() > (std::numeric_limits<std::uint32_t>::max)())
    throw core::Exception(core::Status::LimitExceeded, op, "triangle indices exceed uint32");
  if (ids.empty()) return {};
  auto out = compact_object_model_3d(model, std::span<const std::size_t>(ids));
  out.triangles.clear();
  out.lines.clear();
  out.triangles_are_polygons = false;
  out.triangles.reserve(faces.size());
  for (const auto t : faces)
    out.triangles.push_back({static_cast<std::uint32_t>(remap[t[0]]), static_cast<std::uint32_t>(remap[t[1]]),
                             static_cast<std::uint32_t>(remap[t[2]])});
  if (!options.output_all_points) {
    const PointAttributes::Update update{"original_point_indices", original_indices};
    out.point_attributes.Replace({&update, 1});
  }
  validate_object_model_3d(out);
  return out;
}
}  // namespace mv::reconstruction
