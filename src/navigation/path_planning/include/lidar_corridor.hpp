// MIT License
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#ifndef PATH_PLANNING_LIDAR_CORRIDOR_HPP_
#define PATH_PLANNING_LIDAR_CORRIDOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "geometry_types.hpp"

namespace path_planning {

struct BoundaryCone {
  double x, y;
  int side = -1;  // -1 unknown, 0 measured right, 1 measured left
  double uncertainty = 0;
  uint64_t id = 0;
};

struct LocalCorridor {
  std::vector<ConePair> gates;
  bool uses_unknown = false;
};

/**
 * Trace locally supported gates using track width and successive gate geometry.
 * Unknown sides exist only in this planning result, never in the landmark map.
 * Equally plausible branches end the supported corridor rather than guessing.
 */
inline LocalCorridor trace_lidar_corridor(const std::vector<BoundaryCone>& cones,
                                          const CarPose& car, double width, double max_gap = 2.0) {
  LocalCorridor result;
  if (cones.size() < 4 || cones.size() > 100 || !std::isfinite(width) || width <= 0.0 ||
      !std::isfinite(max_gap) || max_gap <= 0.0 || !std::isfinite(car.x) ||
      !std::isfinite(car.y) || !std::isfinite(car.yaw))
    return result;
  for (const auto& cone : cones)
    if (!std::isfinite(cone.x) || !std::isfinite(cone.y)) return result;

  struct Candidate {
    std::size_t a, b;
    double x, y, width;
  };
  std::vector<Candidate> candidates;
  for (std::size_t a = 0; a < cones.size(); ++a) {
    for (std::size_t b = a + 1; b < cones.size(); ++b) {
      if (cones[a].side >= 0 && cones[a].side == cones[b].side) continue;
      const double length = std::hypot(cones[b].x - cones[a].x, cones[b].y - cones[a].y);
      if (length < 0.7 * width || length > 1.3 * width) continue;
      candidates.push_back(
          {a, b, (cones[a].x + cones[b].x) * 0.5, (cones[a].y + cones[b].y) * 0.5, length});
    }
  }
  std::vector<bool> used(cones.size(), false);
  double x = car.x, y = car.y, heading = car.yaw;
  while (result.gates.size() < 30) {
    struct Option {
      std::size_t left, right;
      double x, y, heading, score;
    };
    std::vector<Option> options;
    const bool first = result.gates.empty();
    for (const auto& candidate : candidates) {
      if (used[candidate.a] || used[candidate.b]) continue;
      const double dx = candidate.x - x, dy = candidate.y - y;
      const double distance = std::hypot(dx, dy);
      const double forward = dx * std::cos(heading) + dy * std::sin(heading);
      const double lateral = -dx * std::sin(heading) + dy * std::cos(heading);
      if (distance > max_gap || forward < (first ? -0.1 : 0.15)) continue;
      if (first && std::abs(lateral) > 0.35 * width) continue;
      const double direction = first ? heading : std::atan2(dy, dx);
      const double turn = std::atan2(std::sin(direction - heading), std::cos(direction - heading));
      if (std::abs(turn) > 0.8) continue;
      const double tangent = first ? heading : heading + 0.5 * turn;
      const auto& a = cones[candidate.a];
      const auto& b = cones[candidate.b];
      // A gate must cross the corridor, not run down one boundary. Use the new
      // direction to follow bends instead of repeatedly using the initial heading.
      if (std::abs((b.x - a.x) * std::cos(direction) + (b.y - a.y) * std::sin(direction)) >
          0.5 * candidate.width)
        continue;
      const bool a_left =
          -std::sin(tangent) * (a.x - candidate.x) + std::cos(tangent) * (a.y - candidate.y) > 0.0;
      const std::size_t left = a_left ? candidate.a : candidate.b;
      const std::size_t right = a_left ? candidate.b : candidate.a;
      if (cones[left].side == 0 || cones[right].side == 1) continue;
      if (!first) {
        const auto& previous = result.gates.back();
        const double lx = cones[left].x - previous.start_pos.x;
        const double ly = cones[left].y - previous.start_pos.y;
        const double rx = cones[right].x - previous.end_pos.x;
        const double ry = cones[right].y - previous.end_pos.y;
        if (lx * std::cos(tangent) + ly * std::sin(tangent) < 0.1 ||
            rx * std::cos(tangent) + ry * std::sin(tangent) < 0.1 ||
            std::hypot(lx, ly) > max_gap * 1.4 || std::hypot(rx, ry) > max_gap * 1.4)
          continue;
      }
      const double score = distance + 0.5 * std::abs(turn) + std::abs(candidate.width - width) +
                           (first ? 2.0 * std::abs(lateral) : 0.0);
      options.push_back({left, right, candidate.x, candidate.y, direction, score});
    }
    if (options.empty()) break;
    std::sort(options.begin(), options.end(),
              [](const auto& a, const auto& b) { return a.score < b.score; });
    const auto& best = options.front();
    bool ambiguous = false;
    for (std::size_t i = 1; i < options.size(); ++i) {
      if (options[i].score - best.score >= 0.15) break;
      if (std::hypot(options[i].x - best.x, options[i].y - best.y) > 0.3) {
        ambiguous = true;
        break;
      }
    }
    if (ambiguous) break;
    const auto& left = cones[best.left];
    const auto& right = cones[best.right];
    result.gates.push_back(
        {{left.x, left.y, true}, {right.x, right.y, false}, {best.x, best.y, false}});
    result.uses_unknown = result.uses_unknown || left.side < 0 || right.side < 0;
    used[best.left] = used[best.right] = true;
    x = best.x;
    y = best.y;
    heading = best.heading;
  }
  return result;
}

}  // namespace path_planning
#endif
