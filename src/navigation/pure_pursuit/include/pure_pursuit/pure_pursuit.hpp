// Pure Pursuit header
// Copyright 2024 F1TENTH contributors

#pragma once

#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Do not export using-directives from headers; use chrono literals in source files.

class PurePursuit : public rclcpp::Node {
 public:
  PurePursuit();

 private:
  struct csvFileData {
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> V;

    int index;
    int velocity_index;

    Eigen::Vector3d lookahead_point_world;
    Eigen::Vector3d lookahead_point_car;
    Eigen::Vector3d current_point_world;
  };

  Eigen::Matrix3d rotation_m;

  double x_car_world;
  double y_car_world;

  double car_orient_w;
  double car_orient_x;
  double car_orient_y;
  double car_orient_z;

  std::string odom_topic;
  std::string waypoint_topic;
  std::string car_refFrame;
  std::string drive_topic;
  std::string global_refFrame;
  std::string rviz_current_waypoint_topic;
  std::string rviz_lookahead_waypoint_topic;

  double K_p;
  double min_lookahead;
  double max_lookahead;
  double lookahead_ratio;
  double steering_limit;
  double velocity_percentage;
  double curr_velocity = 0.0;

  bool emergency_breaking = false;
  std::string lane_number = "left";

  std::fstream csvFile_waypoints;

  csvFileData waypoints;
  int num_waypoints;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscriber;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      publisher_drive;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double to_radians(double degrees);
  double to_degrees(double radians);
  double p2pdist(double &x1, double &x2, double &y1, double &y2);
  bool point_is_behind_car(double x, double y);

  void visualize_lookahead_point(Eigen::Vector3d &point);
  void visualize_current_point(Eigen::Vector3d &point);

  void get_waypoint();

  void quat_to_rot(double q0, double q1, double q2, double q3);

  bool transformandinterp_waypoint();

  double p_controller();

  double get_velocity(double steering_angle);

  void publish_message(double steering_angle);

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
  void waypoint_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint);

  void timer_callback();
};
