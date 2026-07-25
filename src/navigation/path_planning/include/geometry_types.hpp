#ifndef PATH_PLANNING_GEOMETRY_TYPES_HPP_
#define PATH_PLANNING_GEOMETRY_TYPES_HPP_

#include <array>
#include <cstddef>

namespace path_planning {

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

// One cone in the raw point set fed to the Delaunay triangulation.
struct ConeNode {
  Point2D pos;
  bool is_left = false;  // true = left/blue boundary, false = right/yellow boundary
};

// A Blue-Yellow edge midpoint that survived the color/distance/heading filters.
struct MidpointNode {
  Point2D pos;
  std::size_t cone_a = 0;
  std::size_t cone_b = 0;
};

using Edge2D = std::array<Point2D, 2>;

// Car pose used by the heading/FOV filter and as the graph-search start point.
struct CarPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;
};

}  // namespace path_planning

#endif  // PATH_PLANNING_GEOMETRY_TYPES_HPP_
