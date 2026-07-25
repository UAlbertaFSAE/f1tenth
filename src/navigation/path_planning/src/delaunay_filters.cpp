#include "delaunay_filters.hpp"

#include <cmath>
#include <set>
#include <utility>

#include "CDT.h"

namespace path_planning {
namespace {

struct CdtPoint {
  double x;
  double y;
};

bool color_filter_ok(const ConeNode& a, const ConeNode& b) { return a.is_left != b.is_left; }

bool distance_filter_ok(const Point2D& a, const Point2D& b, double max_edge_m) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy) <= max_edge_m;
}

// Drop edges whose midpoint sits behind the car (relative to its heading).
// Skipped (always passes) until a valid car pose has been received.
bool heading_filter_ok(const Point2D& midpoint, const CarPose& car_pose) {
  if (!car_pose.valid) {
    return true;
  }
  const double rel_x = midpoint.x - car_pose.x;
  const double rel_y = midpoint.y - car_pose.y;
  const double forward = rel_x * std::cos(car_pose.yaw) + rel_y * std::sin(car_pose.yaw);
  return forward >= 0.0;
}

}  // namespace

DelaunayResult build_filtered_delaunay(const std::vector<ConeNode>& cones, double max_edge_m,
                                       const CarPose& car_pose) {
  DelaunayResult result;
  if (cones.size() < 3) {
    return result;
  }

  std::vector<CdtPoint> points;
  points.reserve(cones.size());
  for (const auto& cone : cones) {
    points.push_back({cone.pos.x, cone.pos.y});
  }

  CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::AsProvided);
  try {
    cdt.insertVertices(
        points.begin(), points.end(), [](const CdtPoint& p) { return p.x; },
        [](const CdtPoint& p) { return p.y; });
    cdt.eraseSuperTriangle();  // unconstrained Delaunay: no boundary, just the raw mesh
  } catch (const std::exception&) {
    return result;
  }

  std::set<std::pair<std::size_t, std::size_t>> unique_edges;
  for (const auto& tri : cdt.triangles) {
    const std::size_t ids[3] = {static_cast<std::size_t>(tri.vertices[0]),
                                static_cast<std::size_t>(tri.vertices[1]),
                                static_cast<std::size_t>(tri.vertices[2])};
    for (int i = 0; i < 3; ++i) {
      std::size_t a = ids[i];
      std::size_t b = ids[(i + 1) % 3];
      if (a > b) {
        std::swap(a, b);
      }
      unique_edges.insert({a, b});
    }
  }

  for (const auto& edge : unique_edges) {
    const std::size_t a = edge.first;
    const std::size_t b = edge.second;
    if (a >= cones.size() || b >= cones.size()) {
      continue;
    }

    // 1) Color filter
    if (!color_filter_ok(cones[a], cones[b])) {
      continue;
    }

    // 2) Distance filter
    if (!distance_filter_ok(cones[a].pos, cones[b].pos, max_edge_m)) {
      continue;
    }

    Point2D midpoint;
    midpoint.x = (cones[a].pos.x + cones[b].pos.x) / 2.0;
    midpoint.y = (cones[a].pos.y + cones[b].pos.y) / 2.0;

    // 3) Heading / FOV filter
    if (!heading_filter_ok(midpoint, car_pose)) {
      continue;
    }

    MidpointNode node;
    node.pos = midpoint;
    node.cone_a = a;
    node.cone_b = b;
    result.midpoints.push_back(node);
  }

  return result;
}

}  // namespace path_planning
