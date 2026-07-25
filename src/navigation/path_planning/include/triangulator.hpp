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

#ifndef PATH_PLANNING_TRIANGULATOR_HPP_
#define PATH_PLANNING_TRIANGULATOR_HPP_

#include <deque>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_types.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

// Left-Right pair of cone vectors
struct LR {
  std::vector<rc_interfaces::msg::Cone> left;
  std::vector<rc_interfaces::msg::Cone> right;
};

class Triangulator : public rclcpp::Node {
 public:
  Triangulator();

 private:
  std::string left_color_;
  std::string right_color_;
  double position_tolerance_ = 0.2;
  double cluster_distance_ = 8.0;
  double min_track_width_ = 0.4;
  double max_track_width_ = 10.0;

  bool gate_enabled_ = true;
  std::vector<std::string> start_gate_colors_;
  std::vector<std::string> stop_gate_colors_;
  int gate_min_cone_count_ = 2;
  double gate_max_distance_ = 5.0;
  bool gate_latched_ = false;
  bool publish_enabled_ = false;

  double stop_distance_m_ = 20.0;
  double extrapolation_step_m_ = 1.0;
  bool publish_markers_when_idle_ = true;

  rc_interfaces::msg::Cones accumulated_cones_;
  std::deque<rc_interfaces::msg::Cones> frame_window_;
  int window_frames_ = 6;
  bool boundary_constraint_enabled_ = true;
  bool view_persist_ = false;
  int marker_frame_counter_ = 0;
  std::vector<geometry_msgs::msg::Point> last_published_waypoints_;
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  std::string frame_id_;
  bool has_active_cluster_center_ = false;
  geometry_msgs::msg::Point active_cluster_center_;
  bool has_active_path_direction_ = false;
  double active_dir_x_ = 1.0;
  double active_dir_y_ = 0.0;
  double direction_gate_behind_m_ = 1.0;
  double direction_gate_half_width_m_ = 4.0;
  double turn_penalty_weight_ = 5.0;

  // Delaunay-filter-pipeline / graph-search / spline parameters.
  path_planning::CarPose car_pose_;
  double smoothed_yaw_ = 0.0;
  double heading_smoothing_alpha_ = 0.05;
  double delaunay_max_edge_m_ = 5.0;
  double midpoint_connect_radius_m_ = 3.0;
  double max_turn_deg_ = 60.0;
  double waypoint_spacing_m_ = 0.5;

  // call backs
  void read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones_msg);
  void read_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  bool is_same_cone(const rc_interfaces::msg::Cone& coneA,
                    const rc_interfaces::msg::Cone& coneB) const;
  std::string normalize_color(const std::string& raw_color) const;
  rc_interfaces::msg::Cones filter_frame_cones(const rc_interfaces::msg::Cones& frame_cones);
  bool has_cone(const rc_interfaces::msg::Cone& cone) const;
  rc_interfaces::msg::Cones build_working_set() const;
  std::vector<geometry_msgs::msg::Point> constrain_to_corridor(
      const std::vector<geometry_msgs::msg::Point>& waypoints, const LR& pair) const;
  bool detect_gate(const rc_interfaces::msg::Cones& frame_cones) const;
  std::vector<geometry_msgs::msg::Point> append_stop_extrapolation(
      const std::vector<geometry_msgs::msg::Point>& waypoints) const;
  void publish_waypoint_stream(const std::vector<geometry_msgs::msg::Point>& waypoints);
  LR split(const rc_interfaces::msg::Cones& cones);
  std::vector<geometry_msgs::msg::Point> compute_path(
      const rc_interfaces::msg::Cones& working_set,
      std::vector<path_planning::Edge2D>* triangles_out);
  void publish_markers(const rc_interfaces::msg::Cones& cones,
                       const std::vector<path_planning::Edge2D>& triangles,
                       const std::vector<geometry_msgs::msg::Point>& waypoints);
};

#endif  // PATH_PLANNING_TRIANGULATOR_HPP_
