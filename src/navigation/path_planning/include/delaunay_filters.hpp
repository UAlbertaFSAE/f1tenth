#ifndef PATH_PLANNING_DELAUNAY_FILTERS_HPP_
#define PATH_PLANNING_DELAUNAY_FILTERS_HPP_

#include <vector>

#include "geometry_types.hpp"

namespace path_planning {

struct DelaunayResult {
  std::vector<MidpointNode> midpoints;
};

// Runs an unconstrained Delaunay triangulation over every cone (both colors
// together), then prunes edges in three passes before computing midpoints:
//
//   1. Color filter   - drop Blue-Blue / Yellow-Yellow edges (only a
//                        Blue-Yellow edge can be a valid track cross-section).
//   2. Distance filter - drop edges longer than max_edge_m (prevents
//                        connecting across the track on straights/hairpins).
//   3. Heading filter  - drop edges whose midpoint falls behind car_pose
//                        (skipped if car_pose is not valid yet).
//
// Returns only the surviving (post-filter) midpoints -- each one still
// carries the cone indices its edge connected, so callers can recover the
// filtered edges themselves (e.g. for visualization) without the raw,
// unfiltered mesh ever leaving this module.
DelaunayResult build_filtered_delaunay(const std::vector<ConeNode>& cones, double max_edge_m,
                                       const CarPose& car_pose);

}  // namespace path_planning

#endif  // PATH_PLANNING_DELAUNAY_FILTERS_HPP_
