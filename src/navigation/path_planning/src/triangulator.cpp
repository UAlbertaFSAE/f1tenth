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
#include <utility>
#include <vector>

#include "bspline.hpp"
#include "delaunay_filters.hpp"
#include "graph_search.hpp"

namespace {
double sqr_distance(const rc_interfaces::msg::Cone& a, const rc_interfaces::msg::Cone& b) {
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return dx * dx + dy * dy;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

Triangulator::Triangulator() : Node("triangulator_node") {
  this->declare_parameter("cones_topic", "/cone_positions");
  this->declare_parameter("odom_topic", "/ego_racecar/odom");
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
  this->declare_parameter("direction_gate_behind_m", 1.0);
  this->declare_parameter("direction_gate_half_width_m", 4.0);
  this->declare_parameter("turn_penalty_weight", 5.0);
  this->declare_parameter("window_frames", 6);
  this->declare_parameter("boundary_constraint_enabled", true);
  this->declare_parameter("view_persist", false);

  // Delaunay-filter-pipeline / graph-search / spline parameters.
  this->declare_parameter("delaunay_max_edge_m", 5.0);
  this->declare_parameter("midpoint_connect_radius_m", 3.0);
  this->declare_parameter("max_turn_deg", 60.0);
  this->declare_parameter("waypoint_spacing_m", 0.5);
  this->declare_parameter("heading_smoothing_alpha", 0.05);

  std::string cone_topic = this->get_parameter("cones_topic").as_string();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
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
  direction_gate_behind_m_ = this->get_parameter("direction_gate_behind_m").as_double();
  direction_gate_half_width_m_ = this->get_parameter("direction_gate_half_width_m").as_double();
  turn_penalty_weight_ = this->get_parameter("turn_penalty_weight").as_double();
  window_frames_ = static_cast<int>(this->get_parameter("window_frames").as_int());
  boundary_constraint_enabled_ = this->get_parameter("boundary_constraint_enabled").as_bool();
  view_persist_ = this->get_parameter("view_persist").as_bool();
  delaunay_max_edge_m_ = this->get_parameter("delaunay_max_edge_m").as_double();
  midpoint_connect_radius_m_ = this->get_parameter("midpoint_connect_radius_m").as_double();
  max_turn_deg_ = this->get_parameter("max_turn_deg").as_double();
  waypoint_spacing_m_ = this->get_parameter("waypoint_spacing_m").as_double();
  heading_smoothing_alpha_ = this->get_parameter("heading_smoothing_alpha").as_double();
  if (window_frames_ < 1) {
    window_frames_ = 1;
  }

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
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(qos_depth), std::bind(&Triangulator::read_odom, this, _1));

  waypoint_publisher = create_publisher<nav_msgs::msg::Path>(waypoint_topic, rclcpp::QoS(qos_depth));
  marker_publisher =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(qos_depth));

  RCLCPP_INFO(this->get_logger(),
              "Starting triangulator (gate_enabled=%s, publishing=%s, stop_distance=%.2fm, "
              "cone_topic=%s, odom_topic=%s, waypoint_topic=%s, marker_topic=%s)",
              gate_enabled_ ? "true" : "false", publish_enabled_ ? "true" : "false",
              stop_distance_m_, cone_topic.c_str(), odom_topic.c_str(), waypoint_topic.c_str(),
              marker_topic.c_str());
}

namespace {
double wrap_pi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

void Triangulator::read_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  car_pose_.x = odom_msg->pose.pose.position.x;
  car_pose_.y = odom_msg->pose.pose.position.y;
  const double raw_yaw = yaw_from_quaternion(odom_msg->pose.pose.orientation);

  // Low-pass filter the heading used for the Delaunay heading filter/graph
  // search: raw instantaneous yaw is noisy enough (steering oscillation,
  // physics jitter) to flip which edges count as "ahead" frame to frame even
  // with a static cone set -- confirmed on track-1. But it must still track
  // the car through a REAL turn in real time, unlike freezing the reference
  // until a path already succeeded (that version regressed corner-taking
  // entirely: the filter can't accept a turn's edges until its reference
  // already points into the turn). A continuously-updated low-pass filter
  // does both: smooths the noise, keeps rotating through an actual corner.
  if (!car_pose_.valid) {
    smoothed_yaw_ = raw_yaw;
  } else {
    smoothed_yaw_ += heading_smoothing_alpha_ * wrap_pi(raw_yaw - smoothed_yaw_);
  }
  car_pose_.yaw = raw_yaw;
  car_pose_.valid = true;
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

  frame_window_.push_back(filtered_frame);
  while (static_cast<int>(frame_window_.size()) > window_frames_) {
    frame_window_.pop_front();
  }

  // A brand-new, single-frame working set is the least reliable one there
  // is -- track-1 showed this concretely: an oddly-shaped station right at
  // the start produced a misdirected first path from frame 1 alone, and the
  // car committed to it immediately. Wait for the window to actually fill
  // before publishing anything.
  if (static_cast<int>(frame_window_.size()) < window_frames_) {
    return;
  }

  const rc_interfaces::msg::Cones working_set = build_working_set();
  if (working_set.cones.empty()) {
    return;
  }

  const LR pair = split(working_set);

  std::vector<path_planning::Edge2D> filtered_edges;
  std::vector<geometry_msgs::msg::Point> waypoints = compute_path(working_set, &filtered_edges);

  if (waypoints.empty()) {
    // No path this cycle despite having cones -- pure_pursuit will keep
    // driving toward whatever it last resolved to until its own staleness
    // timeout kicks in. Loud on purpose: silent here is what made a
    // graph-search dead-end at a corner look like a mystery instead of an
    // obvious "the filters/graph search lost the thread" signal.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No path this cycle (%zu cones in working set) -- "
                         "graph search/filters likely lost connectivity",
                         working_set.cones.size());
  }

  if (boundary_constraint_enabled_ && !waypoints.empty()) {
    waypoints = constrain_to_corridor(waypoints, pair);
  }

  if (!waypoints.empty()) {
    last_published_waypoints_ = waypoints;
    publish_waypoint_stream(waypoints);

    if (waypoints.size() >= 2) {
      const auto& p0 = waypoints.front();
      const auto& p1 = waypoints.back();
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      const double len = std::sqrt(dx * dx + dy * dy);
      if (len > 1e-6) {
        active_dir_x_ = dx / len;
        active_dir_y_ = dy / len;
        has_active_path_direction_ = true;
      }
    }
  }

  if (publish_markers_when_idle_ || !waypoints.empty()) {
    publish_markers(working_set, filtered_edges, waypoints);
  }

  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Window cones: %zu (left=%zu right=%zu), filtered_edges=%zu, waypoints=%zu, frames=%zu",
      working_set.cones.size(), pair.left.size(), pair.right.size(), filtered_edges.size(),
      waypoints.size(), frame_window_.size());
}

rc_interfaces::msg::Cones Triangulator::build_working_set() const {
  rc_interfaces::msg::Cones out;
  for (const auto& frame : frame_window_) {
    for (const auto& cone : frame.cones) {
      bool duplicate = false;
      for (const auto& existing : out.cones) {
        if (is_same_cone(existing, cone)) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        out.cones.push_back(cone);
      }
    }
  }
  return out;
}

std::vector<geometry_msgs::msg::Point> Triangulator::constrain_to_corridor(
    const std::vector<geometry_msgs::msg::Point>& waypoints, const LR& pair) const {
  if (pair.left.size() < 2 || pair.right.size() < 2) {
    return waypoints;
  }

  const double max_dist = max_track_width_;
  std::vector<geometry_msgs::msg::Point> kept;
  kept.reserve(waypoints.size());

  for (const auto& wp : waypoints) {
    double nearest_left = std::numeric_limits<double>::max();
    for (const auto& cone : pair.left) {
      const double dx = wp.x - cone.x;
      const double dy = wp.y - cone.y;
      nearest_left = std::min(nearest_left, std::sqrt(dx * dx + dy * dy));
    }

    double nearest_right = std::numeric_limits<double>::max();
    for (const auto& cone : pair.right) {
      const double dx = wp.x - cone.x;
      const double dy = wp.y - cone.y;
      nearest_right = std::min(nearest_right, std::sqrt(dx * dx + dy * dy));
    }

    if (nearest_left <= max_dist && nearest_right <= max_dist) {
      kept.push_back(wp);
    }
  }

  return kept.empty() ? waypoints : kept;
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
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = frame_id_;
  path_msg.poses.reserve(waypoints.size());
  for (const auto& wp : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = wp;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }
  waypoint_publisher->publish(path_msg);
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

      double turn_penalty = 0.0;
      if (has_active_path_direction_ && distance_score > 1e-6) {
        const double alignment = (dx * active_dir_x_ + dy * active_dir_y_) / distance_score;
        turn_penalty = (1.0 - alignment) * turn_penalty_weight_;
      }

      const double score =
          distance_score - 0.1 * static_cast<double>(clusters[i].size()) + turn_penalty;

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

  if (has_active_path_direction_ && has_active_cluster_center_ && clusters.size() > 1) {
    std::vector<rc_interfaces::msg::Cone> gated_cones;
    gated_cones.reserve(cluster_cones.size());
    for (const auto& cone : cluster_cones) {
      const double rel_x = cone.x - active_cluster_center_.x;
      const double rel_y = cone.y - active_cluster_center_.y;
      const double fwd = rel_x * active_dir_x_ + rel_y * active_dir_y_;
      const double lat = rel_x * (-active_dir_y_) + rel_y * active_dir_x_;
      if (fwd >= -direction_gate_behind_m_ && std::abs(lat) <= direction_gate_half_width_m_) {
        gated_cones.push_back(cone);
      }
    }
    if (!gated_cones.empty()) {
      cluster_cones = gated_cones;
    }
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

  if (has_active_path_direction_) {
    const double dir_x = active_dir_x_;
    const double dir_y = active_dir_y_;
    auto by_arc_position = [dir_x, dir_y](const rc_interfaces::msg::Cone& a,
                                          const rc_interfaces::msg::Cone& b) {
      return (a.x * dir_x + a.y * dir_y) < (b.x * dir_x + b.y * dir_y);
    };
    std::sort(pair.left.begin(), pair.left.end(), by_arc_position);
    std::sort(pair.right.begin(), pair.right.end(), by_arc_position);
  }

  return pair;
}

std::vector<geometry_msgs::msg::Point> Triangulator::compute_path(
    const rc_interfaces::msg::Cones& working_set,
    std::vector<path_planning::Edge2D>* filtered_edges_out) {
  filtered_edges_out->clear();

  std::vector<path_planning::ConeNode> cones;
  cones.reserve(working_set.cones.size());
  for (const auto& cone : working_set.cones) {
    const std::string color = normalize_color(cone.color);
    if (color != left_color_ && color != right_color_) {
      continue;
    }
    path_planning::ConeNode node;
    node.pos.x = cone.x;
    node.pos.y = cone.y;
    node.is_left = (color == left_color_);
    cones.push_back(node);
  }

  if (cones.size() < 3) {
    return {};
  }

  // Use the low-pass-filtered heading (see read_odom) instead of raw
  // instantaneous odom yaw -- raw yaw is noisy enough on its own (steering
  // oscillation, physics jitter) to flip which edges count as "ahead" frame
  // to frame even with a completely static cone set, confirmed on track-1.
  path_planning::CarPose filter_pose = car_pose_;
  filter_pose.yaw = smoothed_yaw_;

  // Filters 1-3: color, distance, heading -- pruned directly off the raw
  // (unconstrained) Delaunay mesh over every cone in the working set.
  const path_planning::DelaunayResult delaunay =
      path_planning::build_filtered_delaunay(cones, delaunay_max_edge_m_, filter_pose);

  if (delaunay.midpoints.empty()) {
    return {};
  }

  // Recover the surviving (post-filter) edges from the midpoints for
  // visualization -- the raw, unfiltered mesh never leaves delaunay_filters.
  filtered_edges_out->reserve(delaunay.midpoints.size());
  for (const auto& mp : delaunay.midpoints) {
    path_planning::Edge2D edge;
    edge[0] = cones[mp.cone_a].pos;
    edge[1] = cones[mp.cone_b].pos;
    filtered_edges_out->push_back(edge);
  }

  // Graph search: greedy forward walk from the midpoint nearest the car,
  // rejecting any next-hop that requires turning more than max_turn_deg_.
  const std::vector<path_planning::Point2D> ordered_path = path_planning::extract_ordered_path(
      delaunay.midpoints, filter_pose, midpoint_connect_radius_m_, max_turn_deg_);

  if (ordered_path.empty()) {
    return {};
  }

  // Spline: smooth the discrete midpoint sequence and resample it into
  // uniformly arc-length-spaced waypoints for pure_pursuit.
  const std::vector<path_planning::Point2D> smoothed =
      path_planning::smooth_and_resample(ordered_path, waypoint_spacing_m_);

  std::vector<geometry_msgs::msg::Point> waypoints;
  waypoints.reserve(smoothed.size());
  for (const auto& p : smoothed) {
    geometry_msgs::msg::Point wp;
    wp.x = p.x;
    wp.y = p.y;
    wp.z = 0.0;
    waypoints.push_back(wp);
  }
  return waypoints;
}

void Triangulator::publish_markers(const rc_interfaces::msg::Cones& cones,
                                   const std::vector<path_planning::Edge2D>& filtered_edges,
                                   const std::vector<geometry_msgs::msg::Point>& waypoints) {
  visualization_msgs::msg::MarkerArray marker_array;
  const rclcpp::Time now = this->now();

  std::string ns_suffix;
  if (!view_persist_) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = now;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
  } else {
    ns_suffix = "_" + std::to_string(marker_frame_counter_++);
  }

  visualization_msgs::msg::Marker left_cones_marker;
  left_cones_marker.header.frame_id = frame_id_;
  left_cones_marker.header.stamp = now;
  left_cones_marker.ns = "cones_left" + ns_suffix;
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
  right_cones_marker.ns = "cones_right" + ns_suffix;
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
  tri_marker.ns = "filtered_edges" + ns_suffix;
  tri_marker.id = 2;
  tri_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tri_marker.action = visualization_msgs::msg::Marker::ADD;
  tri_marker.pose.orientation.w = 1.0;
  tri_marker.scale.x = 0.04;
  tri_marker.color.r = 1.0f;
  tri_marker.color.g = 0.2f;
  tri_marker.color.b = 0.1f;
  tri_marker.color.a = 1.0f;

  for (const auto& edge : filtered_edges) {
    geometry_msgs::msg::Point a;
    a.x = edge[0].x;
    a.y = edge[0].y;
    geometry_msgs::msg::Point b;
    b.x = edge[1].x;
    b.y = edge[1].y;
    tri_marker.points.push_back(a);
    tri_marker.points.push_back(b);
  }

  visualization_msgs::msg::Marker waypoint_marker;
  waypoint_marker.header.frame_id = frame_id_;
  waypoint_marker.header.stamp = now;
  waypoint_marker.ns = "triangulation_waypoints" + ns_suffix;
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

  // Track boundaries: connect each side's cones into a line so the drivable
  // corridor is visible (and matches the corridor used to constrain waypoints).
  visualization_msgs::msg::Marker left_boundary_marker;
  left_boundary_marker.header.frame_id = frame_id_;
  left_boundary_marker.header.stamp = now;
  left_boundary_marker.ns = "boundary_left" + ns_suffix;
  left_boundary_marker.id = 4;
  left_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
  left_boundary_marker.pose.orientation.w = 1.0;
  left_boundary_marker.scale.x = 0.08;
  left_boundary_marker.color.r = 0.0f;
  left_boundary_marker.color.g = 0.0f;
  left_boundary_marker.color.b = 1.0f;
  left_boundary_marker.color.a = 1.0f;
  left_boundary_marker.points = left_cones_marker.points;

  visualization_msgs::msg::Marker right_boundary_marker = left_boundary_marker;
  right_boundary_marker.ns = "boundary_right" + ns_suffix;
  right_boundary_marker.id = 5;
  right_boundary_marker.color.r = 1.0f;
  right_boundary_marker.color.g = 1.0f;
  right_boundary_marker.color.b = 0.0f;
  right_boundary_marker.points = right_cones_marker.points;

  marker_array.markers.push_back(left_cones_marker);
  marker_array.markers.push_back(right_cones_marker);
  marker_array.markers.push_back(tri_marker);
  marker_array.markers.push_back(waypoint_marker);
  marker_array.markers.push_back(left_boundary_marker);
  marker_array.markers.push_back(right_boundary_marker);
  marker_publisher->publish(marker_array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
