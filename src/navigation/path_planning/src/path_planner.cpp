// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#include "path_planner.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>

#include "bspline.hpp"
#include "filtering.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace {

/**
 * Normalize a quaternion and convert its orientation to yaw.
 *
 * @param orientation The quaternion representing the car orientation.
 * @return The yaw angle in radians, or NAN when the quaternion is invalid.
 */
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& orientation) {
  const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
                                 std::hypot(orientation.z, orientation.w));
  if (!std::isfinite(norm) || norm < 1e-12) return NAN;

  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

/**
 * Initialize the planner parameters, subscriptions, and publishers.
 */
PathPlanner::PathPlanner() : Node("path_planner") {
  read_parameters();
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(qos_depth_),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { read_odom(msg); });
  cones_subscriber_ = create_subscription<rc_interfaces::msg::Cones>(
      cones_topic_, rclcpp::QoS(qos_depth_),
      [this](rc_interfaces::msg::Cones::ConstSharedPtr msg) { read_cones(msg); });
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(waypoint_topic_, 1);
  marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 1);
}

/**
 * Read configuration once at startup.
 */
void PathPlanner::read_parameters() {
  cones_topic_ = declare_parameter<std::string>("cones_topic", "/cone_positions");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/waypoints");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/triangulation_markers");
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");

  left_color_ = declare_parameter<std::string>("left_color", "blue");
  right_color_ = declare_parameter<std::string>("right_color", "yellow");

  position_tolerance_ = declare_parameter<double>("position_tolerance", 0.2);
  cluster_distance_ = declare_parameter<double>("cluster_distance", 8.0);
  min_track_width_ = declare_parameter<double>("min_track_width", 0.4);
  max_track_width_ = declare_parameter<double>("max_track_width", 10.0);
  qos_depth_ = declare_parameter<int>("qos_depth", 1);

  gate_enabled_ = declare_parameter<bool>("gate_enabled", false);
  start_colors_ =
      declare_parameter<std::vector<std::string>>("start_colors", {"orange", "small_orange"});
  stop_colors_ =
      declare_parameter<std::vector<std::string>>("stop_colors", {"orange", "small_orange"});
  gate_min_cone_count_ = declare_parameter<int>("gate_min_cone_count", 2);
  gate_max_distance_ = declare_parameter<double>("gate_max_distance", 5.0);

  stop_distance_ = declare_parameter<double>("stop_distance", 20.0);
  extrapolation_step_ = declare_parameter<double>("extrapolation_step", 1.0);
  heading_smoothing_alpha_ = declare_parameter<double>("heading_smoothing_alpha", 0.05);

  max_odom_age_s_ = declare_parameter<double>("max_odom_age_s", 0.2);
  max_cone_age_s_ = declare_parameter<double>("max_cone_age_s", 0.2);
  delaunay_max_edge_m_ = declare_parameter<double>("delaunay_max_edge_m", 5.0);
  waypoint_spacing_m_ = declare_parameter<double>("waypoint_spacing_m", 0.5);
  end_margin_m_ = declare_parameter<double>("end_margin_m", 0.15);
  hold_timeout_s_ = declare_parameter<double>("hold_timeout_s", 2.0);
  track_width_m_ = declare_parameter<double>("track_width_m", 0.92);
  left_color_ = normalize_color(left_color_);
  right_color_ = normalize_color(right_color_);
  if (qos_depth_ < 1 || !std::isfinite(position_tolerance_) || position_tolerance_ < 0.0 ||
      !std::isfinite(heading_smoothing_alpha_) || heading_smoothing_alpha_ < 0.0 ||
      heading_smoothing_alpha_ > 1.0 || !std::isfinite(max_odom_age_s_) ||
      max_odom_age_s_ <= 0.0 || !std::isfinite(max_cone_age_s_) || max_cone_age_s_ <= 0.0 ||
      !std::isfinite(delaunay_max_edge_m_) || delaunay_max_edge_m_ <= 0.0 ||
      !std::isfinite(waypoint_spacing_m_) || waypoint_spacing_m_ <= 0.0 || left_color_.empty() ||
      right_color_.empty() || left_color_ == right_color_) {
    throw std::invalid_argument("Invalid path planning parameters");
  }
}

/**
 * Store the latest valid odometry and smooth its heading.
 *
 * @param msg The odometry message containing the car position and orientation.
 */
void PathPlanner::read_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const auto& p = msg->pose.pose.position;
  const double yaw = yaw_from_quaternion(msg->pose.pose.orientation);

  if (msg->header.frame_id != frame_id_ || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(yaw)) {
    car_pose_.has_odom = false;
    return;
  }

  if (!car_pose_.has_odom || !std::isfinite(smoothed_yaw_)) {
    smoothed_yaw_ = yaw;
  } else {
    const double delta = yaw - smoothed_yaw_;
    smoothed_yaw_ += heading_smoothing_alpha_ * std::atan2(std::sin(delta), std::cos(delta));
  }

  car_pose_ = {p.x, p.y, smoothed_yaw_};
  car_pose_.odom_stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  car_pose_.has_odom = true;
}

/**
 * Check that valid odometry is recent enough for planning.
 *
 * @return True when odometry is available and within the configured age limit.
 */
bool PathPlanner::odometry_ready() {
  if (!car_pose_.has_odom) return false;
  const double age = (get_clock()->now() - car_pose_.odom_stamp).seconds();
  return age >= 0.0 && age <= max_odom_age_s_;
}

/**
 * Check that cone observations are recent and in the planning frame.
 *
 * @param msg The cone message containing its capture timestamp and coordinate frame.
 * @return True when the frame matches and the timestamp is within the configured age limit.
 */
bool PathPlanner::cones_ready(const rc_interfaces::msg::Cones& msg) {
  if (msg.header.frame_id != frame_id_ || msg.header.stamp.sec < 0 ||
      msg.header.stamp.nanosec >= 1000000000u)
    return false;

  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const double age = (get_clock()->now() - stamp).seconds();
  return age >= 0.0 && age <= max_cone_age_s_;
}

/**
 * Convert known cone colors and remove repeated detections in this frame.
 *
 * @param msg The message containing the detected cones.
 * @return Finite, unique cone positions labeled by their left or right side.
 */
std::vector<path_planning::Point2D> PathPlanner::preprocess_cones(
    const rc_interfaces::msg::Cones& msg) const {
  std::vector<path_planning::Point2D> points;
  points.reserve(msg.cones.size());

  for (const auto& cone : msg.cones) {
    const auto color = normalize_color(cone.color);
    if (color != left_color_ && color != right_color_) continue;
    if (!std::isfinite(cone.x) || !std::isfinite(cone.y)) continue;

    const path_planning::Point2D point{cone.x, cone.y, color == left_color_};

    if (!is_duplicate_cone(points, point)) points.push_back(point);
  }

  return points;
}

/**
 * Check whether a cone matches an existing detection within the position tolerance.
 *
 * @param points The vector of existing cone detections.
 * @param point The cone detection to check.
 * @return True when a cone on the same side has a matching position.
 */
bool PathPlanner::is_duplicate_cone(const std::vector<path_planning::Point2D>& points,
                                    const path_planning::Point2D& point) const {
  for (const auto& existing : points) {
    if (existing.is_left != point.is_left) continue;

    const double distance =
        std::max(std::abs(existing.x - point.x), std::abs(existing.y - point.y));
    if (distance <= position_tolerance_) return true;
  }
  return false;
}

/**
 * Order gate midpoints before smoothing them into waypoints.
 *
 * @param gates The vector of gate pairs to plan a path through.
 * @return A vector of smoothed and resampled waypoints representing the planned path.
 */
std::vector<path_planning::Point2D> PathPlanner::plan_path(
    const std::vector<path_planning::ConePair>& gates) const {
  std::vector<path_planning::Point2D> ordered;
  ordered.reserve(gates.size());
  for (const auto& gate : gates) ordered.push_back(gate.mid_pos);
  const double forward_x = std::cos(car_pose_.yaw);
  const double forward_y = std::sin(car_pose_.yaw);
  std::stable_sort(ordered.begin(), ordered.end(), [&](const auto& a, const auto& b) {
    return a.x * forward_x + a.y * forward_y < b.x * forward_x + b.y * forward_y;
  });
  if (ordered.size() < 2) return {};
  return path_planning::smooth_and_resample(ordered, waypoint_spacing_m_);
}

/**
 * Build a centreline from one boundary when the other side is out of view.
 *
 * @param cones The preprocessed cones of this frame.
 * @return Waypoints offset half a track width from the larger visible side, or empty.
 */
std::vector<path_planning::Point2D> PathPlanner::one_sided_path(
    const std::vector<path_planning::Point2D>& cones) const {
  const double forward_x = std::cos(car_pose_.yaw);
  const double forward_y = std::sin(car_pose_.yaw);
  const auto forward = [&](const path_planning::Point2D& p) {
    return (p.x - car_pose_.x) * forward_x + (p.y - car_pose_.y) * forward_y;
  };
  std::vector<path_planning::Point2D> sides[2];
  for (const auto& cone : cones) {
    if (forward(cone) >= 0.0) sides[cone.is_left].push_back(cone);
  }
  auto& line = sides[1].size() >= sides[0].size() ? sides[1] : sides[0];
  if (line.size() < 2) return {};
  std::stable_sort(line.begin(), line.end(),
                   [&](const auto& a, const auto& b) { return forward(a) < forward(b); });

  // The centreline lies right of the left boundary and left of the right one.
  const double side = line.front().is_left ? 1.0 : -1.0;

  std::vector<path_planning::Point2D> centre;
  centre.reserve(line.size());
  for (std::size_t i = 0; i < line.size(); ++i) {
    const auto& a = line[i == 0 ? 0 : i - 1];
    const auto& b = line[std::min(i + 1, line.size() - 1)];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length <= 0.0) continue;
    path_planning::Point2D point;
    point.x = line[i].x + side * 0.5 * track_width_m_ * (b.y - a.y) / length;
    point.y = line[i].y - side * 0.5 * track_width_m_ * (b.x - a.x) / length;
    centre.push_back(point);
  }
  if (centre.size() < 2) return {};
  return path_planning::smooth_and_resample(centre, waypoint_spacing_m_);
}

/**
 * Extend the path past its last point by the configured end margin.
 *
 * @param path The planned waypoints.
 * @return The waypoints with one extra point along the final segment.
 */
std::vector<path_planning::Point2D> PathPlanner::extend_end(
    std::vector<path_planning::Point2D> path) const {
  if (path.size() < 2 || end_margin_m_ <= 0.0) return path;
  const auto end = path.back();
  const auto& previous = path[path.size() - 2];
  const double length = std::hypot(end.x - previous.x, end.y - previous.y);
  if (length <= 0.0) return path;
  path_planning::Point2D extra;
  extra.x = end.x + (end.x - previous.x) / length * end_margin_m_;
  extra.y = end.y + (end.y - previous.y) / length * end_margin_m_;
  path.push_back(extra);
  return path;
}

/**
 * Check whether the car has driven past the end of a path.
 *
 * @param path The held waypoints.
 * @return True when the car is beyond the last point along the final segment.
 */
bool PathPlanner::passed_end(const std::vector<path_planning::Point2D>& path) const {
  if (path.size() < 2) return true;
  const auto& end = path.back();
  const auto& previous = path[path.size() - 2];
  return (car_pose_.x - end.x) * (end.x - previous.x) +
             (car_pose_.y - end.y) * (end.y - previous.y) >=
         0.0;
}

/**
 * Track the narrowest gate seen, used as the width for one-sided paths.
 *
 * @param gates The gates of the current frame.
 */
void PathPlanner::update_track_width(const std::vector<path_planning::ConePair>& gates) {
  double width = INFINITY;
  for (const auto& gate : gates) {
    width = std::min(width, std::hypot(gate.start_pos.x - gate.end_pos.x,
                                       gate.start_pos.y - gate.end_pos.y));
  }
  if (std::isfinite(width) && width >= min_track_width_) track_width_m_ = width;
}

/**
 * Keep publishing the last valid path until the car passes its end or it expires.
 */
void PathPlanner::publish_held_path() {
  if (!held_path_.empty() &&
      ((now() - held_stamp_).seconds() > hold_timeout_s_ || passed_end(held_path_))) {
    held_path_.clear();
  }
  publish_path(held_path_);
}

/**
 * Process detected cones and publish the planned path and visualization markers.
 *
 * @param msg The message containing the detected cones.
 */
void PathPlanner::read_cones(rc_interfaces::msg::Cones::ConstSharedPtr msg) {
  if (!odometry_ready()) {
    held_path_.clear();
    publish_path({});
    return;
  }
  if (!cones_ready(*msg)) {
    publish_held_path();
    return;
  }
  try {
    const auto cones = preprocess_cones(*msg);
    const auto gates =
        path_planning::build_filtered_delaunay(cones, delaunay_max_edge_m_, car_pose_);
    auto path = plan_path(gates);
    if (path.empty()) {
      path = one_sided_path(cones);
    } else {
      update_track_width(gates);
    }
    if (path.empty()) {
      publish_held_path();
    } else {
      held_path_ = extend_end(path);
      held_stamp_ = now();
      publish_path(held_path_);
    }
    publish_markers(cones, gates, path);
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Planning failed: %s", error.what());
    held_path_.clear();
    publish_path({});
  }
}

/**
 * Publish the current path; an empty path signals no usable plan.
 * Preserve generation time on held paths so consumers can enforce their own age limits.
 *
 * @param points The vector of 2D points representing the planned path.
 */
void PathPlanner::publish_path(const std::vector<path_planning::Point2D>& points) {
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = points.empty() ? now() : held_stamp_;
  msg.poses.reserve(points.size());
  for (const auto& point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  path_publisher_->publish(msg);
}

/**
 * Publish persistent markers for cones, gates, and the planned path.
 *
 * @param cones The vector of cones to visualize.
 * @param gates The vector of gate pairs to visualize.
 * @param path The vector of planned path points to visualize.
 */
void PathPlanner::publish_markers(const std::vector<path_planning::Point2D>& cones,
                                  const std::vector<path_planning::ConePair>& gates,
                                  const std::vector<path_planning::Point2D>& path) {
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray msg;
  msg.markers.resize(4);
  for (std::size_t i = 0; i < msg.markers.size(); ++i) {
    auto& marker = msg.markers[i];
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "planning_" + std::to_string(marker_frame_);
    marker.id = static_cast<int>(i);
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
    marker.color.a = 1.0;
  }
  ++marker_frame_;
  msg.markers[0].type = msg.markers[1].type = Marker::SPHERE_LIST;
  msg.markers[0].color.b = 1.0;
  msg.markers[1].color.r = msg.markers[1].color.g = 1.0;
  msg.markers[2].type = Marker::LINE_LIST;
  msg.markers[2].color.r = 1.0;
  msg.markers[2].scale.x = 0.04;
  msg.markers[3].type = Marker::LINE_STRIP;
  msg.markers[3].color.g = 1.0;
  msg.markers[3].scale.x = 0.06;
  const auto to_point = [](const path_planning::Point2D& p) {
    geometry_msgs::msg::Point result;
    result.x = p.x;
    result.y = p.y;
    return result;
  };
  for (const auto& cone : cones)
    msg.markers[cone.is_left ? 0 : 1].points.push_back(to_point(cone));
  for (const auto& gate : gates) {
    msg.markers[2].points.push_back(to_point(gate.start_pos));
    msg.markers[2].points.push_back(to_point(gate.end_pos));
  }
  for (const auto& p : path) msg.markers[3].points.push_back(to_point(p));
  marker_publisher_->publish(msg);
}

/**
 * Normalize a color string by removing whitespace and converting to lowercase.
 *
 * @param color The input color string to normalize.
 * @return The normalized color string.
 */
std::string PathPlanner::normalize_color(std::string color) {
  color.erase(
      std::remove_if(color.begin(), color.end(), [](unsigned char c) { return std::isspace(c); }),
      color.end());
  std::transform(color.begin(), color.end(), color.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return color;
}

/**
 * Initialize ROS and run the path_planner node.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments passed to ROS.
 * @return Zero after the node shuts down successfully.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}
