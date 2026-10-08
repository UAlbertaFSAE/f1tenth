// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#ifndef PATH_PLANNING_FILTERING_HPP_
#define PATH_PLANNING_FILTERING_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "CDT.h"
#include "geometry_types.hpp"

namespace path_planning {
namespace filtering_detail {

using Edge = std::pair<std::size_t, std::size_t>;

/**
 * Reject invalid coordinates, limits, and poses before triangulation.
 *
 * @param cones The vector of cone positions and sides.
 * @param max_edge_m The maximum allowed edge length in metres.
 * @param car_pose The car position and heading.
 * @param filter_heading Whether a valid car pose is required for heading filtering.
 */
inline void validate_input(const std::vector<Point2D>& cones, double max_edge_m,
                           const CarPose& car_pose, bool filter_heading) {
  if (!std::isfinite(max_edge_m) || max_edge_m <= 0.0) {
    throw std::invalid_argument("Maximum Delaunay edge length must be finite and positive");
  }
  if (filter_heading &&
      (!std::isfinite(car_pose.x) || !std::isfinite(car_pose.y) || !std::isfinite(car_pose.yaw))) {
    throw std::invalid_argument("Heading filtering requires a finite car pose");
  }
  for (const auto& cone : cones) {
    if (!std::isfinite(cone.x) || !std::isfinite(cone.y)) {
      throw std::invalid_argument("Cone coordinates must be finite");
    }
  }
}

/**
 * Merge identical cone detections without losing their side.
 *
 * @param cones The vector of cone positions and sides.
 * @return Unique cones, with conflicting colors at the same position rejected.
 */
inline std::vector<Point2D> unique_cones(const std::vector<Point2D>& cones) {
  std::vector<Point2D> points = cones;
  const auto duplicates = CDT::FindDuplicates<double>(
      points.begin(), points.end(), [](const Point2D& p) { return p.x; },
      [](const Point2D& p) { return p.y; });
  // Map each compacted vertex back to its first original detection.
  std::vector<std::size_t> first_detection(points.size(), points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto& first = first_detection[duplicates.mapping[i]];
    if (first == points.size()) first = i;
    if (points[first].is_left != points[i].is_left) {
      throw std::invalid_argument("Coincident cones have conflicting colors");
    }
  }
  CDT::RemoveDuplicates(points, duplicates.duplicates);
  return points;
}

/**
 * Build the Delaunay mesh and collect its unique edges.
 *
 * @param points The vector of unique cone positions.
 * @return Unique pairs of cone indices, or an empty vector for degenerate geometry.
 */
inline std::vector<Edge> delaunay_edges(const std::vector<Point2D>& points) {
  CDT::Triangulation<double> triangulation(CDT::VertexInsertionOrder::Auto);
  try {
    triangulation.insertVertices(
        points.begin(), points.end(), [](const Point2D& p) { return p.x; },
        [](const Point2D& p) { return p.y; });
    triangulation.eraseSuperTriangle();
  } catch (const CDT::Error&) {
    return {};  // Degenerate geometry cannot provide valid gates.
  }

  // A shared triangle edge should appear only once.
  std::vector<Edge> edges;
  edges.reserve(3 * triangulation.triangles.size());
  for (const auto& triangle : triangulation.triangles) {
    for (int i = 0; i < 3; ++i) {
      const auto a = static_cast<std::size_t>(triangle.vertices[i]);
      const auto b = static_cast<std::size_t>(triangle.vertices[(i + 1) % 3]);
      edges.emplace_back(std::min(a, b), std::max(a, b));
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

  return edges;
}

/**
 * Keep opposite-color edges within range whose midpoints are ahead.
 *
 * @param points The vector of unique cone positions.
 * @param edges The unique Delaunay edges expressed as cone indices.
 * @param max_edge_m The maximum allowed edge length in metres.
 * @param car_pose The car position and heading.
 * @param filter_heading Whether to reject midpoints behind the car.
 * @return The surviving left/right cone pairs and their midpoints.
 */
inline std::vector<ConePair> filter_edges(const std::vector<Point2D>& points,
                                          const std::vector<Edge>& edges, double max_edge_m,
                                          const CarPose& car_pose, bool filter_heading) {
  const double forward_x = filter_heading ? std::cos(car_pose.yaw) : 0.0;
  const double forward_y = filter_heading ? std::sin(car_pose.yaw) : 0.0;
  std::vector<ConePair> result;
  result.reserve(edges.size());
  for (const auto& edge : edges) {
    const auto& a = points[edge.first];
    const auto& b = points[edge.second];
    if (a.is_left == b.is_left) continue;
    const double edge_length = std::hypot(a.x - b.x, a.y - b.y);
    if (edge_length > max_edge_m) continue;
    Point2D midpoint;
    midpoint.x = 0.5 * a.x + 0.5 * b.x;
    midpoint.y = 0.5 * a.y + 0.5 * b.y;
    if (filter_heading) {
      const double forward_distance =
          (midpoint.x - car_pose.x) * forward_x + (midpoint.y - car_pose.y) * forward_y;
      if (forward_distance < 0.0) continue;
    }
    const auto& left = a.is_left ? a : b;
    const auto& right = a.is_left ? b : a;
    result.push_back({left, right, midpoint});
  }
  return result;
}

}  // namespace filtering_detail

/**
 * Triangulate cones and filter edges by color, distance, and heading.
 *
 * @param cones The vector of cone positions and sides.
 * @param max_edge_m The maximum allowed edge length in metres.
 * @param car_pose The car position and heading.
 * @param filter_heading Whether to apply heading filtering; disable until odometry is available.
 * @return Filtered left/right cone pairs and their midpoints.
 */
inline std::vector<ConePair> build_filtered_delaunay(const std::vector<Point2D>& cones,
                                                     double max_edge_m, const CarPose& car_pose,
                                                     bool filter_heading = true) {
  filtering_detail::validate_input(cones, max_edge_m, car_pose, filter_heading);
  const auto points = filtering_detail::unique_cones(cones);
  if (points.size() < 3) return {};
  const auto edges = filtering_detail::delaunay_edges(points);
  return filtering_detail::filter_edges(points, edges, max_edge_m, car_pose, filter_heading);
}

}  // namespace path_planning
#endif  // PATH_PLANNING_FILTERING_HPP_
