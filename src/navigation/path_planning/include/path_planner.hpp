// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#ifndef PATH_PLANNING_PATH_PLANNER_HPP_
#define PATH_PLANNING_PATH_PLANNER_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "geometry_types.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * Convert cone detections into a local path in the odometry frame.
 */
class PathPlanner : public rclcpp::Node {
 public:
  PathPlanner();

 private:
  void read_parameters();
  void read_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void read_cones(rc_interfaces::msg::Cones::ConstSharedPtr msg);
  bool odometry_ready();
  bool cones_ready(const rc_interfaces::msg::Cones& msg);
  bool is_duplicate_cone(const std::vector<path_planning::Point2D>& points,
                         const path_planning::Point2D& point) const;
  std::vector<path_planning::Point2D> preprocess_cones(const rc_interfaces::msg::Cones& msg) const;
  std::vector<path_planning::Point2D> plan_path(
      const std::vector<path_planning::ConePair>& gates) const;
  std::vector<path_planning::Point2D> one_sided_path(
      const std::vector<path_planning::Point2D>& cones) const;
  std::vector<path_planning::Point2D> extend_end(std::vector<path_planning::Point2D> path) const;
  bool passed_end(const std::vector<path_planning::Point2D>& path) const;
  void update_track_width(const std::vector<path_planning::ConePair>& gates);
  void publish_held_path();
  void publish_path(const std::vector<path_planning::Point2D>& points);
  void publish_markers(const std::vector<path_planning::Point2D>& cones,
                       const std::vector<path_planning::ConePair>& gates,
                       const std::vector<path_planning::Point2D>& path);
  static std::string normalize_color(std::string color);

  std::string cones_topic_, odom_topic_, waypoint_topic_, marker_topic_, frame_id_;
  std::string left_color_, right_color_;
  double position_tolerance_, cluster_distance_, min_track_width_, max_track_width_;
  int qos_depth_;
  bool gate_enabled_;
  std::vector<std::string> start_colors_, stop_colors_;
  int gate_min_cone_count_;
  double gate_max_distance_, stop_distance_, extrapolation_step_;
  double heading_smoothing_alpha_, max_odom_age_s_, max_cone_age_s_, delaunay_max_edge_m_;
  double waypoint_spacing_m_, end_margin_m_, hold_timeout_s_, track_width_m_;

  std::vector<path_planning::Point2D> held_path_;
  rclcpp::Time held_stamp_{0, 0, RCL_ROS_TIME};

  path_planning::CarPose car_pose_;
  double smoothed_yaw_ = NAN;
  std::size_t marker_frame_ = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cones_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

#endif
