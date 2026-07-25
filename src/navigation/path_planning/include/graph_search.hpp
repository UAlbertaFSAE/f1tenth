#ifndef PATH_PLANNING_GRAPH_SEARCH_HPP_
#define PATH_PLANNING_GRAPH_SEARCH_HPP_

#include <vector>

#include "geometry_types.hpp"

namespace path_planning {

// Treats the filtered midpoints as graph nodes (connected when within
// connect_radius_m of each other) and walks forward from the node nearest
// the car, at each step continuing toward whichever reachable unvisited
// neighbor deviates least from the current heading -- and refusing to turn
// more than max_turn_deg. This is what keeps a fork/intersection from
// yanking the path sideways: the branch requiring a sharp turn is never a
// legal next step, so the search just continues straight through.
std::vector<Point2D> extract_ordered_path(const std::vector<MidpointNode>& midpoints,
                                          const CarPose& car_pose, double connect_radius_m,
                                          double max_turn_deg);

}  // namespace path_planning

#endif  // PATH_PLANNING_GRAPH_SEARCH_HPP_
