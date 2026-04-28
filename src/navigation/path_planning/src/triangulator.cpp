// MIT License

// Copyright (c) 2026 Krupal Shah

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "triangulator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

#include "CDT.h"

namespace {
double sqr_distance(const rc_interfaces::msg::Cone& a, const rc_interfaces::msg::Cone& b) {
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return dx * dx + dy * dy;
}

double cross_2d(double ax, double ay, double bx, double by, double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool on_segment(double ax, double ay, double bx, double by, double px, double py) {
  return px >= std::min(ax, bx) && px <= std::max(ax, bx) && py >= std::min(ay, by) &&
         py <= std::max(ay, by);
}

bool segments_intersect(double ax, double ay, double bx, double by, double cx, double cy,
                        double dx, double dy) {
  constexpr double kEpsilon = 1e-9;

  const double o1 = cross_2d(ax, ay, bx, by, cx, cy);
  const double o2 = cross_2d(ax, ay, bx, by, dx, dy);
  const double o3 = cross_2d(cx, cy, dx, dy, ax, ay);
  const double o4 = cross_2d(cx, cy, dx, dy, bx, by);

  const bool proper_intersection =
      ((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon)) &&
      ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon));
  if (proper_intersection) {
    return true;
  }

  if (std::abs(o1) <= kEpsilon && on_segment(ax, ay, bx, by, cx, cy)) {
    return true;
  }
  if (std::abs(o2) <= kEpsilon && on_segment(ax, ay, bx, by, dx, dy)) {
    return true;
  }
  if (std::abs(o3) <= kEpsilon && on_segment(cx, cy, dx, dy, ax, ay)) {
    return true;
  }
  if (std::abs(o4) <= kEpsilon && on_segment(cx, cy, dx, dy, bx, by)) {
    return true;
  }

  return false;
}
}  // namespace

Triangulator::Triangulator() : Node("triangulator_node") {
  this->declare_parameter("cones_topic", "/cone_positions");
  this->declare_parameter("waypoint_topic", "/waypoints");
  this->declare_parameter("marker_topic", "/triangulation_markers");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("left_color", "blue");
  this->declare_parameter("right_color", "yellow");
  this->declare_parameter("position_tolerance", 0.2);
  this->declare_parameter("cluster_distance", 8.0);
  this->declare_parameter("min_track_width", 0.4);
  this->declare_parameter("max_track_width", 10.0);
  this->declare_parameter("qos_depth", 10);

  this->declare_parameter("gate_enabled", true);
  this->declare_parameter<std::vector<std::string>>(
      "start_colors", std::vector<std::string>{"orange", "small_orange"});
  this->declare_parameter<std::vector<std::string>>(
      "stop_colors", std::vector<std::string>{"orange", "small_orange"});
  this->declare_parameter("gate_min_cone_count", 2);
  this->declare_parameter("gate_max_distance", 5.0);
  this->declare_parameter("stop_distance", 20.0);
  this->declare_parameter("extrapolation_step", 1.0);
  this->declare_parameter("publish_markers_when_idle", true);

  std::string cone_topic = this->get_parameter("cones_topic").as_string();
  std::string waypoint_topic = this->get_parameter("waypoint_topic").as_string();
  std::string marker_topic = this->get_parameter("marker_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  left_color_ = normalize_color(this->get_parameter("left_color").as_string());
  right_color_ = normalize_color(this->get_parameter("right_color").as_string());
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();
  cluster_distance_ = this->get_parameter("cluster_distance").as_double();
  min_track_width_ = this->get_parameter("min_track_width").as_double();
  max_track_width_ = this->get_parameter("max_track_width").as_double();

  gate_enabled_ = this->get_parameter("gate_enabled").as_bool();
  start_gate_colors_ = this->get_parameter("start_colors").as_string_array();
  stop_gate_colors_ = this->get_parameter("stop_colors").as_string_array();
  gate_min_cone_count_ = this->get_parameter("gate_min_cone_count").as_int();
  gate_max_distance_ = this->get_parameter("gate_max_distance").as_double();
  stop_distance_m_ = this->get_parameter("stop_distance").as_double();
  extrapolation_step_m_ = this->get_parameter("extrapolation_step").as_double();
  publish_markers_when_idle_ = this->get_parameter("publish_markers_when_idle").as_bool();

  for (auto& color : start_gate_colors_) {
    color = normalize_color(color);
  }
  for (auto& color : stop_gate_colors_) {
    color = normalize_color(color);
  }

  if (stop_gate_colors_.empty()) {
    stop_gate_colors_ = start_gate_colors_;
  }
  if (gate_min_cone_count_ < 1) {
    gate_min_cone_count_ = 1;
  }
  if (extrapolation_step_m_ <= 0.0) {
    extrapolation_step_m_ = 1.0;
  }
  if (stop_distance_m_ < 0.0) {
    stop_distance_m_ = 0.0;
  }

  const int qos_depth = this->get_parameter("qos_depth").as_int();
  publish_enabled_ = !gate_enabled_;

  cone_subscriber = this->create_subscription<rc_interfaces::msg::Cones>(
      cone_topic, rclcpp::QoS(qos_depth), std::bind(&Triangulator::read_cones, this, _1));

  waypoint_publisher =
      create_publisher<geometry_msgs::msg::Point>(waypoint_topic, rclcpp::QoS(qos_depth));
  marker_publisher =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(qos_depth));

  RCLCPP_INFO(this->get_logger(),
              "Starting triangulator (gate_enabled=%s, publishing=%s, stop_distance=%.2fm, "
              "cone_topic=%s, waypoint_topic=%s, marker_topic=%s)",
              gate_enabled_ ? "true" : "false", publish_enabled_ ? "true" : "false",
              stop_distance_m_, cone_topic.c_str(), waypoint_topic.c_str(), marker_topic.c_str());
}

void Triangulator::read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones_msg) {
  if (cones_msg->cones.empty()) {
    return;
  }

  if (gate_enabled_) {
    const bool gate_seen = detect_gate(*cones_msg);
    if (gate_seen && !gate_latched_) {
      gate_latched_ = true;
      publish_enabled_ = !publish_enabled_;

      if (publish_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Gate detected -> START publishing waypoints");
      } else {
        RCLCPP_INFO(
            this->get_logger(),
            "Gate detected -> STOP publishing waypoints (publishing final extrapolated set)");
        if (!last_published_waypoints_.empty()) {
          const auto final_waypoints = append_stop_extrapolation(last_published_waypoints_);
          publish_waypoint_stream(final_waypoints);
        }
      }
    }

    if (!gate_seen) {
      gate_latched_ = false;
    }
  }

  if (!publish_enabled_) {
    return;
  }

  const rc_interfaces::msg::Cones filtered_frame = filter_frame_cones(*cones_msg);
  if (filtered_frame.cones.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Filtered frame has no valid cones");
    return;
  }

  std::size_t promoted_count = 0;
  for (const auto& cone : filtered_frame.cones) {
    if (!has_cone(cone)) {
      accumulated_cones_.cones.push_back(cone);
      promoted_count++;
    }
  }

  if (accumulated_cones_.cones.empty()) {
    return;
  }

  const LR pair = split(accumulated_cones_);

  std::vector<std::array<geometry_msgs::msg::Point, 3>> triangles;
  std::vector<geometry_msgs::msg::Point> waypoints;
  if (pair.left.size() >= 2 && pair.right.size() >= 2) {
    waypoints = build_waypoints_from_triangulation(pair, &triangles);
  }

  if (!waypoints.empty()) {
    last_published_waypoints_ = waypoints;
    publish_waypoint_stream(waypoints);
  }

  if (publish_markers_when_idle_ || !waypoints.empty()) {
    publish_markers(accumulated_cones_, triangles, waypoints);
  }

  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Accumulated cones: %zu (left=%zu right=%zu), triangles=%zu, waypoints=%zu, promoted=%zu",
      accumulated_cones_.cones.size(), pair.left.size(), pair.right.size(), triangles.size(),
      waypoints.size(), promoted_count);
}

bool Triangulator::is_same_cone(const rc_interfaces::msg::Cone& coneA,
                                const rc_interfaces::msg::Cone& coneB) const {
  return (std::abs(coneA.x - coneB.x) <= position_tolerance_) &&
         (std::abs(coneA.y - coneB.y) <= position_tolerance_) &&
         (normalize_color(coneA.color) == normalize_color(coneB.color));
}

std::string Triangulator::normalize_color(const std::string& raw_color) const {
  std::string color = raw_color;
  color.erase(
      std::remove_if(color.begin(), color.end(), [](unsigned char c) { return std::isspace(c); }),
      color.end());
  std::transform(color.begin(), color.end(), color.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return color;
}

bool Triangulator::detect_gate(const rc_interfaces::msg::Cones& frame_cones) const {
  if (!gate_enabled_ || frame_cones.cones.empty()) {
    return false;
  }

  int start_count = 0;
  int stop_count = 0;
  const double gate_max_dist_sq = gate_max_distance_ * gate_max_distance_;

  for (const auto& cone : frame_cones.cones) {
    const std::string color = normalize_color(cone.color);
    const double dist_sq = static_cast<double>(cone.x) * static_cast<double>(cone.x) +
                           static_cast<double>(cone.y) * static_cast<double>(cone.y);
    if (dist_sq > gate_max_dist_sq) {
      continue;
    }

    if (std::find(start_gate_colors_.begin(), start_gate_colors_.end(), color) !=
        start_gate_colors_.end()) {
      start_count++;
    }
    if (std::find(stop_gate_colors_.begin(), stop_gate_colors_.end(), color) !=
        stop_gate_colors_.end()) {
      stop_count++;
    }
  }

  return start_count >= gate_min_cone_count_ && stop_count >= gate_min_cone_count_;
}

std::vector<geometry_msgs::msg::Point> Triangulator::append_stop_extrapolation(
    const std::vector<geometry_msgs::msg::Point>& waypoints) const {
  if (waypoints.size() < 2 || stop_distance_m_ <= 0.0) {
    return waypoints;
  }

  std::vector<geometry_msgs::msg::Point> extended = waypoints;
  const auto& p_prev = waypoints[waypoints.size() - 2];
  const auto& p_last = waypoints.back();
  const double dx = p_last.x - p_prev.x;
  const double dy = p_last.y - p_prev.y;
  const double segment_len = std::sqrt(dx * dx + dy * dy);
  if (segment_len < 1e-6) {
    return extended;
  }

  const double ux = dx / segment_len;
  const double uy = dy / segment_len;
  double distance_added = 0.0;

  while (distance_added < stop_distance_m_) {
    distance_added += extrapolation_step_m_;
    geometry_msgs::msg::Point p;
    p.x = p_last.x + ux * distance_added;
    p.y = p_last.y + uy * distance_added;
    p.z = 0.0;
    extended.push_back(p);
  }

  return extended;
}

void Triangulator::publish_waypoint_stream(
    const std::vector<geometry_msgs::msg::Point>& waypoints) {
  for (const auto& wp : waypoints) {
    waypoint_publisher->publish(wp);
  }
}

rc_interfaces::msg::Cones Triangulator::filter_frame_cones(
    const rc_interfaces::msg::Cones& frame_cones) {
  rc_interfaces::msg::Cones filtered;
  if (frame_cones.cones.empty()) {
    return filtered;
  }

  std::vector<rc_interfaces::msg::Cone> valid_cones;
  valid_cones.reserve(frame_cones.cones.size());
  for (const auto& cone : frame_cones.cones) {
    rc_interfaces::msg::Cone normalized = cone;
    normalized.color = normalize_color(cone.color);
    if (normalized.color == left_color_ || normalized.color == right_color_) {
      valid_cones.push_back(normalized);
    }
  }

  if (valid_cones.empty()) {
    return filtered;
  }

  const double cluster_dist_sq = cluster_distance_ * cluster_distance_;
  std::vector<bool> visited(valid_cones.size(), false);
  std::vector<std::vector<std::size_t>> clusters;

  for (std::size_t i = 0; i < valid_cones.size(); ++i) {
    if (visited[i]) {
      continue;
    }

    std::vector<std::size_t> cluster_indices;
    std::queue<std::size_t> queue;
    queue.push(i);
    visited[i] = true;

    while (!queue.empty()) {
      const std::size_t current = queue.front();
      queue.pop();
      cluster_indices.push_back(current);

      for (std::size_t j = 0; j < valid_cones.size(); ++j) {
        if (visited[j]) {
          continue;
        }

        if (sqr_distance(valid_cones[current], valid_cones[j]) <= cluster_dist_sq) {
          visited[j] = true;
          queue.push(j);
        }
      }
    }

    clusters.push_back(cluster_indices);
  }

  if (clusters.empty()) {
    return filtered;
  }

  std::size_t selected_cluster = 0;
  if (!has_active_cluster_center_) {
    for (std::size_t i = 1; i < clusters.size(); ++i) {
      if (clusters[i].size() > clusters[selected_cluster].size()) {
        selected_cluster = i;
      }
    }
  } else {
    double best_score = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      double cx = 0.0;
      double cy = 0.0;
      for (const auto idx : clusters[i]) {
        cx += valid_cones[idx].x;
        cy += valid_cones[idx].y;
      }
      cx /= static_cast<double>(clusters[i].size());
      cy /= static_cast<double>(clusters[i].size());

      const double dx = cx - active_cluster_center_.x;
      const double dy = cy - active_cluster_center_.y;
      const double distance_score = std::sqrt(dx * dx + dy * dy);
      const double score = distance_score - 0.1 * static_cast<double>(clusters[i].size());

      if (score < best_score) {
        best_score = score;
        selected_cluster = i;
      }
    }
  }

  std::vector<rc_interfaces::msg::Cone> cluster_cones;
  cluster_cones.reserve(clusters[selected_cluster].size());
  for (const auto idx : clusters[selected_cluster]) {
    cluster_cones.push_back(valid_cones[idx]);
  }

  for (const auto& cone : cluster_cones) {
    double nearest_opposite = std::numeric_limits<double>::max();
    for (const auto& other : cluster_cones) {
      if (cone.color == other.color) {
        continue;
      }
      nearest_opposite = std::min(nearest_opposite, std::sqrt(sqr_distance(cone, other)));
    }

    if (nearest_opposite >= min_track_width_ && nearest_opposite <= max_track_width_) {
      filtered.cones.push_back(cone);
    }
  }

  if (filtered.cones.empty()) {
    filtered.cones = cluster_cones;
  }

  if (!filtered.cones.empty()) {
    active_cluster_center_.x = 0.0;
    active_cluster_center_.y = 0.0;
    active_cluster_center_.z = 0.0;
    for (const auto& cone : filtered.cones) {
      active_cluster_center_.x += cone.x;
      active_cluster_center_.y += cone.y;
    }
    const double inv_size = 1.0 / static_cast<double>(filtered.cones.size());
    active_cluster_center_.x *= inv_size;
    active_cluster_center_.y *= inv_size;
    has_active_cluster_center_ = true;
  }

  return filtered;
}

bool Triangulator::has_cone(const rc_interfaces::msg::Cone& cone) const {
  for (const auto& existing : accumulated_cones_.cones) {
    if (is_same_cone(existing, cone)) {
      return true;
    }
  }
  return false;
}

LR Triangulator::split(const rc_interfaces::msg::Cones& cones) {
  LR pair;

  for (const auto& cone : cones.cones) {
    rc_interfaces::msg::Cone normalized = cone;
    normalized.color = normalize_color(cone.color);
    if (normalized.color == left_color_) {
      pair.left.push_back(normalized);
    } else if (normalized.color == right_color_) {
      pair.right.push_back(normalized);
    } else {
      RCLCPP_INFO(this->get_logger(), "Found cone without left or right color");
    }
  }

  return pair;
}

std::vector<geometry_msgs::msg::Point> Triangulator::build_waypoints_from_triangulation(
    const LR& pair, std::vector<std::array<geometry_msgs::msg::Point, 3>>* triangles_out) {
  triangles_out->clear();

  if (pair.left.size() < 2 || pair.right.size() < 2) {
    return {};
  }

  struct Point2D {
    double x;
    double y;
  };

  struct Edge2D {
    std::pair<std::size_t, std::size_t> vertices;
  };

  std::vector<Point2D> points;
  points.reserve(pair.left.size() + pair.right.size());

  for (const auto& cone : pair.left) {
    points.push_back({cone.x, cone.y});
  }
  for (const auto& cone : pair.right) {
    points.push_back({cone.x, cone.y});
  }

  std::vector<Edge2D> boundary_edges;
  boundary_edges.reserve((pair.left.size() - 1) + (pair.right.size() - 1) + 2);

  const std::size_t left_offset = 0;
  const std::size_t right_offset = pair.left.size();

  for (std::size_t i = 0; i + 1 < pair.left.size(); ++i) {
    boundary_edges.push_back({{i + left_offset, i + 1 + left_offset}});
  }
  for (std::size_t i = 0; i + 1 < pair.right.size(); ++i) {
    boundary_edges.push_back({{i + right_offset, i + 1 + right_offset}});
  }

  boundary_edges.push_back({{left_offset, right_offset}});
  boundary_edges.push_back(
      {{left_offset + pair.left.size() - 1, right_offset + pair.right.size() - 1}});

  bool has_boundary_intersections = false;
  for (std::size_t i = 0; i < boundary_edges.size() && !has_boundary_intersections; ++i) {
    for (std::size_t j = i + 1; j < boundary_edges.size(); ++j) {
      const auto a0 = boundary_edges[i].vertices.first;
      const auto a1 = boundary_edges[i].vertices.second;
      const auto b0 = boundary_edges[j].vertices.first;
      const auto b1 = boundary_edges[j].vertices.second;

      if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) {
        continue;
      }

      const auto& p0 = points[a0];
      const auto& p1 = points[a1];
      const auto& q0 = points[b0];
      const auto& q1 = points[b1];

      if (segments_intersect(p0.x, p0.y, p1.x, p1.y, q0.x, q0.y, q1.x, q1.y)) {
        has_boundary_intersections = true;
        break;
      }
    }
  }

  if (has_boundary_intersections) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Boundary intersections detected; using pair-based waypoint fallback instead of CDT");
    return build_waypoints_from_pairs(pair);
  }

  CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::AsProvided);
  try {
    cdt.insertVertices(
        points.begin(), points.end(), [](const Point2D& p) { return p.x; },
        [](const Point2D& p) { return p.y; });
    cdt.insertEdges(
        boundary_edges.begin(), boundary_edges.end(),
        [](const Edge2D& e) { return e.vertices.first; },
        [](const Edge2D& e) { return e.vertices.second; });
    cdt.eraseOuterTrianglesAndHoles();
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "CDT triangulation failed (%s). Using pair-based waypoint fallback.",
                         ex.what());
    return build_waypoints_from_pairs(pair);
  }

  for (const auto& tri : cdt.triangles) {
    std::array<geometry_msgs::msg::Point, 3> t;

    for (int i = 0; i < 3; ++i) {
      const std::size_t idx = tri.vertices[i];
      if (idx >= cdt.vertices.size()) {
        continue;
      }

      t[i].x = cdt.vertices[idx].x;
      t[i].y = cdt.vertices[idx].y;
      t[i].z = 0.0;
    }

    triangles_out->push_back(t);
  }

  std::set<std::pair<std::size_t, std::size_t>> unique_edges;
  for (const auto& tri : cdt.triangles) {
    const std::array<std::size_t, 3> ids = {static_cast<std::size_t>(tri.vertices[0]),
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

  std::vector<geometry_msgs::msg::Point> waypoints;
  std::set<std::pair<int, int>> unique_waypoints;

  for (const auto& edge : unique_edges) {
    const std::size_t a = edge.first;
    const std::size_t b = edge.second;

    if (a >= cdt.vertices.size() || b >= cdt.vertices.size()) {
      continue;
    }

    const bool a_left = a < pair.left.size();
    const bool b_left = b < pair.left.size();
    if (a_left == b_left) {
      continue;
    }

    geometry_msgs::msg::Point mp;
    mp.x = (cdt.vertices[a].x + cdt.vertices[b].x) / 2.0;
    mp.y = (cdt.vertices[a].y + cdt.vertices[b].y) / 2.0;
    mp.z = 0.0;

    // Deduplicate numerically similar waypoints before publishing.
    const std::pair<int, int> key{static_cast<int>(std::round(mp.x * 1000.0)),
                                  static_cast<int>(std::round(mp.y * 1000.0))};
    if (unique_waypoints.insert(key).second) {
      waypoints.push_back(mp);
    }
  }

  std::sort(waypoints.begin(), waypoints.end(),
            [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
              if (a.x == b.x) {
                return a.y < b.y;
              }
              return a.x < b.x;
            });

  if (waypoints.empty()) {
    return build_waypoints_from_pairs(pair);
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Point> Triangulator::build_waypoints_from_pairs(
    const LR& pair) const {
  std::vector<geometry_msgs::msg::Point> waypoints;
  std::set<std::pair<int, int>> unique_waypoints;

  for (const auto& left : pair.left) {
    double best_dist = std::numeric_limits<double>::max();
    const rc_interfaces::msg::Cone* best_right = nullptr;

    for (const auto& right : pair.right) {
      const double dist = std::sqrt(sqr_distance(left, right));
      if (dist < min_track_width_ || dist > max_track_width_) {
        continue;
      }

      if (dist < best_dist) {
        best_dist = dist;
        best_right = &right;
      }
    }

    if (best_right == nullptr) {
      continue;
    }

    geometry_msgs::msg::Point midpoint;
    midpoint.x = (left.x + best_right->x) * 0.5;
    midpoint.y = (left.y + best_right->y) * 0.5;
    midpoint.z = 0.0;

    const std::pair<int, int> key{static_cast<int>(std::lround(midpoint.x * 1000.0)),
                                  static_cast<int>(std::lround(midpoint.y * 1000.0))};
    if (unique_waypoints.insert(key).second) {
      waypoints.push_back(midpoint);
    }
  }

  std::sort(waypoints.begin(), waypoints.end(),
            [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
              if (a.x == b.x) {
                return a.y < b.y;
              }
              return a.x < b.x;
            });

  return waypoints;
}

void Triangulator::publish_markers(
    const rc_interfaces::msg::Cones& cones,
    const std::vector<std::array<geometry_msgs::msg::Point, 3>>& triangles,
    const std::vector<geometry_msgs::msg::Point>& waypoints) {
  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time now = this->now();

  visualization_msgs::msg::Marker left_cones_marker;
  left_cones_marker.header.frame_id = frame_id_;
  left_cones_marker.header.stamp = now;
  left_cones_marker.ns = "cones_left";
  left_cones_marker.id = 0;
  left_cones_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  left_cones_marker.action = visualization_msgs::msg::Marker::ADD;
  left_cones_marker.pose.orientation.w = 1.0;
  left_cones_marker.scale.x = 0.28;
  left_cones_marker.scale.y = 0.28;
  left_cones_marker.scale.z = 0.28;
  left_cones_marker.color.r = 0.0f;
  left_cones_marker.color.g = 0.0f;
  left_cones_marker.color.b = 1.0f;
  left_cones_marker.color.a = 1.0f;

  visualization_msgs::msg::Marker right_cones_marker = left_cones_marker;
  right_cones_marker.ns = "cones_right";
  right_cones_marker.id = 1;
  right_cones_marker.color.r = 1.0f;
  right_cones_marker.color.g = 1.0f;
  right_cones_marker.color.b = 0.0f;

  for (const auto& cone : cones.cones) {
    geometry_msgs::msg::Point p;
    p.x = cone.x;
    p.y = cone.y;
    p.z = 0.0;

    const std::string normalized_color = normalize_color(cone.color);
    if (normalized_color == left_color_) {
      left_cones_marker.points.push_back(p);
    } else if (normalized_color == right_color_) {
      right_cones_marker.points.push_back(p);
    }
  }

  visualization_msgs::msg::Marker tri_marker;
  tri_marker.header.frame_id = frame_id_;
  tri_marker.header.stamp = now;
  tri_marker.ns = "triangles";
  tri_marker.id = 2;
  tri_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tri_marker.action = visualization_msgs::msg::Marker::ADD;
  tri_marker.pose.orientation.w = 1.0;
  tri_marker.scale.x = 0.04;
  tri_marker.color.r = 1.0f;
  tri_marker.color.g = 0.2f;
  tri_marker.color.b = 0.1f;
  tri_marker.color.a = 1.0f;

  for (const auto& tri : triangles) {
    tri_marker.points.push_back(tri[0]);
    tri_marker.points.push_back(tri[1]);
    tri_marker.points.push_back(tri[1]);
    tri_marker.points.push_back(tri[2]);
    tri_marker.points.push_back(tri[2]);
    tri_marker.points.push_back(tri[0]);
  }

  visualization_msgs::msg::Marker waypoint_marker;
  waypoint_marker.header.frame_id = frame_id_;
  waypoint_marker.header.stamp = now;
  waypoint_marker.ns = "triangulation_waypoints";
  waypoint_marker.id = 3;
  waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
  waypoint_marker.pose.orientation.w = 1.0;
  waypoint_marker.scale.x = 0.22;
  waypoint_marker.scale.y = 0.22;
  waypoint_marker.scale.z = 0.22;
  waypoint_marker.color.r = 0.0f;
  waypoint_marker.color.g = 1.0f;
  waypoint_marker.color.b = 0.0f;
  waypoint_marker.color.a = 1.0f;
  waypoint_marker.points = waypoints;

  marker_array.markers.push_back(left_cones_marker);
  marker_array.markers.push_back(right_cones_marker);
  marker_array.markers.push_back(tri_marker);
  marker_array.markers.push_back(waypoint_marker);
  marker_publisher->publish(marker_array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
