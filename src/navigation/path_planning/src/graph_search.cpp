#include "graph_search.hpp"

#include <cmath>
#include <limits>

namespace path_planning {
namespace {

double wrap_pi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::size_t nearest_to(const std::vector<MidpointNode>& midpoints, double x, double y) {
  std::size_t best = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < midpoints.size(); ++i) {
    const double dx = midpoints[i].pos.x - x;
    const double dy = midpoints[i].pos.y - y;
    const double dist = dx * dx + dy * dy;
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }
  return best;
}

}  // namespace

std::vector<Point2D> extract_ordered_path(const std::vector<MidpointNode>& midpoints,
                                          const CarPose& car_pose, double connect_radius_m,
                                          double max_turn_deg) {
  std::vector<Point2D> path;
  if (midpoints.empty()) {
    return path;
  }

  const double origin_x = car_pose.valid ? car_pose.x : 0.0;
  const double origin_y = car_pose.valid ? car_pose.y : 0.0;
  const double max_turn_rad = max_turn_deg * M_PI / 180.0;
  const double connect_radius_sq = connect_radius_m * connect_radius_m;

  std::vector<bool> visited(midpoints.size(), false);
  std::size_t current = nearest_to(midpoints, origin_x, origin_y);
  visited[current] = true;
  path.push_back(midpoints[current].pos);

  double heading = car_pose.valid
                       ? car_pose.yaw
                       : std::atan2(midpoints[current].pos.y - origin_y,
                                    midpoints[current].pos.x - origin_x);

  for (std::size_t step = 0; step < midpoints.size(); ++step) {
    const Point2D& here = midpoints[current].pos;

    std::size_t best_next = midpoints.size();
    double best_turn = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < midpoints.size(); ++i) {
      if (visited[i]) {
        continue;
      }
      const double dx = midpoints[i].pos.x - here.x;
      const double dy = midpoints[i].pos.y - here.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq > connect_radius_sq || dist_sq < 1e-9) {
        continue;
      }

      const double candidate_heading = std::atan2(dy, dx);
      const double turn = std::abs(wrap_pi(candidate_heading - heading));
      if (turn > max_turn_rad) {
        continue;  // reject: too sharp a deviation from current heading
      }
      if (turn < best_turn) {
        best_turn = turn;
        best_next = i;
      }
    }

    if (best_next == midpoints.size()) {
      break;  // no legal continuation -- path ends here
    }

    heading = std::atan2(midpoints[best_next].pos.y - here.y, midpoints[best_next].pos.x - here.x);
    visited[best_next] = true;
    current = best_next;
    path.push_back(midpoints[current].pos);
  }

  return path;
}

}  // namespace path_planning
