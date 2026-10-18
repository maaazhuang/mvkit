#pragma once
#include <algorithm>
#include <cmath>
#include <core/exception.hpp>
#include <limits>
#include <numeric>
#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {
namespace mesh_detail {
inline Point3d sub(Point3d a, Point3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Point3d cross(Point3d a, Point3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double dot(Point3d a, Point3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double component(Point3d p, int axis) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; }
inline bool valid(const ObjectModel3DData& m, std::size_t i) {
  const auto p = m.points[i];
  return (m.valid.empty() || m.valid[i]) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
inline double segment2(Point3d p, Point3d a, Point3d b) {
  const auto q = sub(b, a);
  const double den = dot(q, q), t = den > 0 ? std::clamp(dot(sub(p, a), q) / den, 0., 1.) : 0.;
  const Point3d projection{a.x + t * q.x, a.y + t * q.y, a.z + t * q.z};
  const auto d = sub(p, projection);
  return dot(d, d);
}
inline double triangle2(Point3d p, Point3d a, Point3d b, Point3d c) {
  const auto ab = sub(b, a), ac = sub(c, a), n = cross(ab, ac);
  const double nn = dot(n, n), edges = std::min({segment2(p, a, b), segment2(p, b, c), segment2(p, c, a)});
  if (!(nn > 0)) return edges;
  const auto ap = sub(p, a);
  const double d = dot(ap, n), s = dot(cross(ap, ac), n) / nn, t = dot(cross(ab, ap), n) / nn;
  return s >= 0 && t >= 0 && s + t <= 1 ? std::min(edges, d * d / nn) : edges;
}
}  // namespace mesh_detail
// 精确连续三角形 BVH；旧距离及正式距离算子共享同一几何，包含退化边。
class MeshIndex {
 public:
  explicit MeshIndex(const ObjectModel3DData& model) : model_(model) {
    for (std::size_t i = 0; i < model.triangles.size(); ++i) {
      const auto t = model.triangles[i];
      if (mesh_detail::valid(model, t[0]) && mesh_detail::valid(model, t[1]) &&
          mesh_detail::valid(model, t[2]))
        ids_.push_back(i);
    }
    if (ids_.empty())
      throw core::Exception(core::Status::InvalidState, "distance_object_model_3d",
                            "reference has no valid triangles");
    nodes_.reserve(ids_.size() * 2);
    Build(0, ids_.size());
  }
  bool Nearest(Point3d p, double cap, std::size_t* triangle, double* squared) const {
    bool found = false;
    std::size_t closest = 0;
    double best = cap;
    Query(0, p, best, closest, found);
    if (!found) return false;
    *triangle = closest;
    *squared = best;
    return true;
  }
  double distance(Point3d p) const {
    std::size_t triangle = 0;
    double squared = std::numeric_limits<double>::infinity();
    Nearest(p, squared, &triangle, &squared);
    return std::sqrt(squared);
  }

 private:
  struct Node {
    Point3d lo{}, hi{};
    std::size_t begin{}, end{}, left{}, right{};
  };
  const ObjectModel3DData& model_;
  std::pmr::vector<std::size_t> ids_{runtime::scratch_resource()};
  std::pmr::vector<Node> nodes_{runtime::scratch_resource()};
  Point3d Center(std::size_t i) const {
    const auto t = model_.triangles[i];
    const auto a = model_.points[t[0]], b = model_.points[t[1]], c = model_.points[t[2]];
    return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
  }
  std::size_t Build(std::size_t first, std::size_t end) {
    const auto index = nodes_.size();
    Node node;
    node.begin = first;
    node.end = end;
    node.lo = {INFINITY, INFINITY, INFINITY};
    node.hi = {-INFINITY, -INFINITY, -INFINITY};
    for (std::size_t i = first; i < end; ++i)
      for (auto v : model_.triangles[ids_[i]]) {
        const auto p = model_.points[v];
        node.lo = {std::min(node.lo.x, p.x), std::min(node.lo.y, p.y), std::min(node.lo.z, p.z)};
        node.hi = {std::max(node.hi.x, p.x), std::max(node.hi.y, p.y), std::max(node.hi.z, p.z)};
      }
    nodes_.push_back(node);
    if (end - first > 8) {
      const auto d = mesh_detail::sub(node.hi, node.lo);
      const int axis = d.x >= d.y && d.x >= d.z ? 0 : d.y >= d.z ? 1 : 2;
      const auto middle = (first + end) / 2;
      std::nth_element(ids_.begin() + first, ids_.begin() + middle, ids_.begin() + end, [&](auto a, auto b) {
        const auto x = mesh_detail::component(Center(a), axis), y = mesh_detail::component(Center(b), axis);
        return x == y ? a < b : x < y;
      });
      const auto left = Build(first, middle), right = Build(middle, end);
      nodes_[index].left = left;
      nodes_[index].right = right;
    }
    return index;
  }
  double Box2(const Node& n, Point3d p) const {
    double d = 0;
    for (int axis = 0; axis < 3; ++axis) {
      const auto x = mesh_detail::component(p, axis),
                 delta = std::max(
                     {mesh_detail::component(n.lo, axis) - x, 0., x - mesh_detail::component(n.hi, axis)});
      d += delta * delta;
    }
    return d;
  }
  void Query(std::size_t i, Point3d p, double& best, std::size_t& closest, bool& found) const {
    const auto& n = nodes_[i];
    if (Box2(n, p) > best) return;
    if (!n.left) {
      for (std::size_t j = n.begin; j < n.end; ++j) {
        const auto id = ids_[j];
        const auto t = model_.triangles[id];
        const auto d =
            mesh_detail::triangle2(p, model_.points[t[0]], model_.points[t[1]], model_.points[t[2]]);
        if (d <= best && (!found || d < best || id < closest)) {
          best = d;
          closest = id;
          found = true;
        }
      }
      return;
    }
    const bool leftFirst = Box2(nodes_[n.left], p) <= Box2(nodes_[n.right], p);
    Query(leftFirst ? n.left : n.right, p, best, closest, found);
    Query(leftFirst ? n.right : n.left, p, best, closest, found);
  }
};
}  // namespace mv::reconstruction
