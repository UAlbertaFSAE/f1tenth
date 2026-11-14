// Copyright (c) 2026 UAlbertaFSAE. MIT License.
#ifndef PATH_PLANNING_CORRIDOR_SEARCH_HPP_
#define PATH_PLANNING_CORRIDOR_SEARCH_HPP_
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "bspline.hpp"
#include "filtering.hpp"
#include "lidar_corridor.hpp"
namespace path_planning {
struct CorridorConfig {
  double wheelbase = .335, width = .150, front = 0, rear = 0;
  double margin = .10, cone_radius = .065, steering = 25;
  double max_gap = 2, width_tolerance = .30, switch_margin = .25;
  int beam_width = 12, max_gates = 20;
  bool heading_fallback = false;
};
struct CorridorCandidate {
  std::vector<ConePair> gates;
  std::vector<Point2D> path;
  std::vector<std::size_t> used;
  double score = 0, heading = 0, width = 0, continuity = 0;
  std::string rejection;
};
struct CorridorResult {
  std::vector<CorridorCandidate> alternatives;
  std::vector<Point2D> path;
  std::vector<ConePair> gates;
  std::string reason = "no_supported_corridor";
  std::map<std::string, int> rejections;
  int selected_index = -1;
};
inline double distance(const Point2D& a, const Point2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline double angle_difference(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
inline double segment_distance(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double u = std::clamp(
      ((p.x - a.x) * dx + (p.y - a.y) * dy) / std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
  return std::hypot(p.x - a.x - u * dx, p.y - a.y - u * dy);
}
// Sample the rear-axle path with conservative translation padding; all landmarks
// are obstacles, including cones which were not selected as a boundary.
inline std::string validate_corridor(const std::vector<Point2D>& path,
                                     const std::vector<BoundaryCone>& cones, const CarPose& car,
                                     const CorridorConfig& cfg) {
  if (path.size() < 2) return "short_path";
  const double max_curvature = std::tan(cfg.steering * M_PI / 180) / cfg.wheelbase;
  double previous_heading = car.yaw, previous_length = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& a = path[i - 1];
    const auto& b = path[i];
    const double length = distance(a, b);
    if (!std::isfinite(length)) return "invalid_path";
    if (length < 1e-8) continue;
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    const double turn = std::abs(angle_difference(heading, previous_heading));
    if (turn > max_curvature * (i == 1 ? length : .5 * (length + previous_length)) + 1e-3)
      return "steering_limit";
    const int steps = std::max(1, static_cast<int>(std::ceil(length / .025)));
    const double rotation_radius =
        std::hypot(std::max(cfg.wheelbase + cfg.front, cfg.rear), .5 * cfg.width);
    const double pad = cfg.margin + cfg.cone_radius + .025 + rotation_radius * std::sin(.5 * turn);
    for (int j = 0; j <= steps; ++j) {
      const double t = double(j) / steps, x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
      for (const auto& cone : cones) {
        const double dx = cone.x - x, dy = cone.y - y;
        const double forward = dx * std::cos(heading) + dy * std::sin(heading);
        const double lateral = -dx * std::sin(heading) + dy * std::cos(heading);
        const double ex =
            std::max({-cfg.rear - forward, forward - cfg.wheelbase - cfg.front, 0.0});
        const double ey = std::max(std::abs(lateral) - .5 * cfg.width, 0.0);
        if (std::hypot(ex, ey) < pad) return "footprint_clearance";
      }
    }
    previous_heading = heading;
    previous_length = length;
  }
  return "";
}
inline bool crossing(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& d) {
  const auto cross = [](const Point2D& p, const Point2D& q, const Point2D& r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
  };
  return cross(a, b, c) * cross(a, b, d) < -1e-10 && cross(c, d, a) * cross(c, d, b) < -1e-10;
}
inline CorridorResult search_corridor(std::vector<BoundaryCone> cones, const CarPose& car,
                                      double track_width, double spacing,
                                      const CorridorConfig& cfg,
                                      const std::vector<Point2D>& previous = {}) {
  CorridorResult result;
  if (cones.size() > 100 || cones.size() < 4) return result;
  for (const auto& c : cones)
    if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.uncertainty) ||
        c.uncertainty < 0)
      return result;
  std::sort(cones.begin(), cones.end(), [](const auto& a, const auto& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    if (a.side != b.side) return a.side > b.side;
    if (a.uncertainty != b.uncertainty) return a.uncertainty > b.uncertainty;
    return a.id < b.id;
  });
  // Exact duplicates cannot add support. Conflicting measured sides stay obstacles.
  cones.erase(std::unique(cones.begin(), cones.end(),
                          [](const auto& a, const auto& b) {
                            return a.x == b.x && a.y == b.y && a.side == b.side;
                          }),
              cones.end());
  struct Gate {
    std::size_t a, b;
    Point2D mid;
    double width;
  };
  std::vector<Point2D> measured;
  for (const auto& cone : cones)
    if (cone.side >= 0) measured.push_back({cone.x, cone.y, cone.side == 1});
  std::vector<ConePair> measured_gates;
  try {
    measured_gates =
        build_filtered_delaunay(measured, track_width * (1 + cfg.width_tolerance), car, false);
  } catch (const std::exception&) {
    result.reason = "conflicting_landmarks";
    return result;
  }
  std::vector<Gate> gates;
  for (std::size_t i = 0; i < cones.size(); ++i)
    for (std::size_t j = i + 1; j < cones.size(); ++j) {
      const auto& a = cones[i];
      const auto& b = cones[j];
      double width = std::hypot(a.x - b.x, a.y - b.y);
      if (a.side >= 0 && a.side == b.side) continue;
      if (a.side >= 0 && b.side >= 0 &&
          !std::any_of(measured_gates.begin(), measured_gates.end(), [&](const auto& g) {
            return (g.start_pos.x == a.x && g.start_pos.y == a.y && g.end_pos.x == b.x &&
                    g.end_pos.y == b.y) ||
                   (g.start_pos.x == b.x && g.start_pos.y == b.y && g.end_pos.x == a.x &&
                    g.end_pos.y == a.y);
          }))
        continue;
      if (width < track_width * (1 - cfg.width_tolerance) ||
          width > track_width * (1 + cfg.width_tolerance))
        continue;
      gates.push_back({i, j, {.5 * (a.x + b.x), .5 * (a.y + b.y)}, width});
    }
  CorridorCandidate seed;
  seed.heading = car.yaw;
  seed.width = track_width;
  std::vector<CorridorCandidate> beam{seed}, finished;
  for (int depth = 0; depth < cfg.max_gates && !beam.empty(); ++depth) {
    std::vector<CorridorCandidate> next;
    for (const auto& state : beam) {
      bool extended = false;
      Point2D origin = state.gates.empty() ? Point2D{car.x, car.y} : state.gates.back().mid_pos;
      for (const auto& gate : gates) {
        if (std::find(state.used.begin(), state.used.end(), gate.a) != state.used.end() ||
            std::find(state.used.begin(), state.used.end(), gate.b) != state.used.end())
          continue;
        const double dx = gate.mid.x - origin.x, dy = gate.mid.y - origin.y,
                     len = std::hypot(dx, dy);
        const double forward = dx * std::cos(state.heading) + dy * std::sin(state.heading);
        if (len > cfg.max_gap || forward < (depth == 0 ? -.05 : .15)) continue;
        const double heading = len < .1 ? state.heading : std::atan2(dy, dx);
        const double turn = angle_difference(heading, state.heading);
        if (std::abs(turn) > .8) continue;
        if (depth == 0 &&
            std::abs(-dx * std::sin(car.yaw) + dy * std::cos(car.yaw)) > .35 * track_width)
          continue;
        const auto& a = cones[gate.a];
        const auto& b = cones[gate.b];
        if (std::abs((a.x - b.x) * std::cos(heading) + (a.y - b.y) * std::sin(heading)) >
            .5 * gate.width)
          continue;
        const bool a_left =
            -(a.x - gate.mid.x) * std::sin(heading) + (a.y - gate.mid.y) * std::cos(heading) > 0;
        const auto& left = a_left ? a : b;
        const auto& right = a_left ? b : a;
        if (left.side == 0 || right.side == 1) continue;
        ConePair pair{{left.x, left.y, true}, {right.x, right.y, false}, gate.mid};
        if (depth) {
          const auto& old = state.gates.back();
          if (distance(old.start_pos, pair.start_pos) > cfg.max_gap * 1.4 ||
              distance(old.end_pos, pair.end_pos) > cfg.max_gap * 1.4)
            continue;
          if (crossing(old.start_pos, pair.start_pos, old.end_pos, pair.end_pos)) continue;
          if (segment_distance(pair.start_pos, old.end_pos, pair.end_pos) < .05 ||
              segment_distance(pair.end_pos, old.start_pos, pair.start_pos) < .05)
            continue;
          if ((left.x - old.start_pos.x) * std::cos(heading) +
                      (left.y - old.start_pos.y) * std::sin(heading) <=
                  0 ||
              (right.x - old.end_pos.x) * std::cos(heading) +
                      (right.y - old.end_pos.y) * std::sin(heading) <=
                  0)
            continue;
        }
        bool intersects = false;
        for (std::size_t i = 1; i < state.gates.size(); ++i) {
          const auto& a0 = state.gates[i - 1];
          const auto& b0 = state.gates[i];
          intersects = intersects || crossing(origin, gate.mid, a0.mid_pos, b0.mid_pos);
        }
        if (intersects) continue;
        double continuity = 0;
        if (previous.size() > 1) {
          continuity = INFINITY;
          for (std::size_t i = 1; i < previous.size(); ++i)
            continuity =
                std::min(continuity, segment_distance(gate.mid, previous[i - 1], previous[i]));
          continuity = std::min(continuity, 1.0);
        }
        auto child = state;
        child.gates.push_back(pair);
        child.used.push_back(gate.a);
        child.used.push_back(gate.b);
        child.heading = heading;
        child.width = gate.width;
        child.continuity += continuity;
        child.score += 2 * std::abs(gate.width - state.width) / track_width + turn * turn +
                       continuity + .05 * ((left.side < 0) + (right.side < 0)) +
                       .2 * (left.uncertainty + right.uncertainty) - .7 * std::min(len, 1.0) - .3;
        if (cfg.heading_fallback) {
          const double lateral =
              -(left.x - right.x) * std::sin(car.yaw) + (left.y - right.y) * std::cos(car.yaw);
          if (left.side < 0 && right.side < 0 && lateral < 0) child.score += .05;
        }
        child.path = {{car.x, car.y}};
        for (const auto& p : child.gates)
          if (distance(child.path.back(), p.mid_pos) > .05) child.path.push_back(p.mid_pos);
        // Reject blocked edges before they can dominate the beam.
        if (child.path.size() > 1) {
          const auto rejection = validate_corridor(child.path, cones, car, cfg);
          if (!rejection.empty()) {
            ++result.rejections[rejection];
            continue;
          }
        }
        next.push_back(std::move(child));
        extended = true;
      }
      if (!extended && state.gates.size() >= 2) finished.push_back(state);
    }
    std::stable_sort(next.begin(), next.end(),
                     [](const auto& a, const auto& b) { return a.score < b.score; });
    if (next.size() > static_cast<std::size_t>(cfg.beam_width)) next.resize(cfg.beam_width);
    beam = std::move(next);
  }
  for (auto& state : beam)
    if (state.gates.size() >= 2) finished.push_back(std::move(state));
  for (auto& state : finished) {
    // Fix the spline's initial tangent to the current vehicle heading.
    if (state.path.size() > 1) {
      const double lead = std::min(.15, .5 * distance(state.path[0], state.path[1]));
      state.path.insert(state.path.begin() + 1,
                        {car.x + lead * std::cos(car.yaw), car.y + lead * std::sin(car.yaw)});
    }
    state.path = smooth_and_resample(state.path, spacing);
    state.rejection = validate_corridor(state.path, cones, car, cfg);
    if (!state.rejection.empty()) ++result.rejections[state.rejection];
  }
  std::stable_sort(finished.begin(), finished.end(),
                   [](const auto& a, const auto& b) { return a.score < b.score; });
  if (finished.size() > static_cast<std::size_t>(cfg.beam_width)) finished.resize(cfg.beam_width);
  result.alternatives = finished;
  std::vector<std::size_t> valid;
  for (std::size_t i = 0; i < finished.size(); ++i)
    if (finished[i].rejection.empty()) valid.push_back(i);
  if (valid.empty()) return result;
  auto chosen = valid.front();
  if (!previous.empty()) {
    auto incumbent = *std::min_element(valid.begin(), valid.end(), [&](auto a, auto b) {
      return finished[a].continuity / finished[a].gates.size() <
             finished[b].continuity / finished[b].gates.size();
    });
    if (finished[incumbent].score - finished[chosen].score < cfg.switch_margin) chosen = incumbent;
  }
  for (auto index : valid) {
    if (index == chosen ||
        std::abs(finished[index].score - finished[chosen].score) >= cfg.switch_margin)
      continue;
    const auto& a = finished[index].path;
    const auto& b = finished[chosen].path;
    bool diverges = false;
    for (std::size_t i = 0; i < std::min({a.size(), b.size(), std::size_t(20)}); ++i)
      diverges = diverges || distance(a[i], b[i]) > .3;
    if (diverges && previous.empty()) {
      result.reason = "ambiguous_corridor";
      return result;
    }
  }
  result.selected_index = static_cast<int>(chosen);
  result.path = finished[chosen].path;
  result.gates = finished[chosen].gates;
  result.reason = "selected";
  return result;
}
}  // namespace path_planning
#endif
