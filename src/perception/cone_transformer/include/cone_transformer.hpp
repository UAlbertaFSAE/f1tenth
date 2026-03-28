#pragma once

#include <deque>   

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_msgs/msg/object.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"

using std::placeholders::_1;

class Transformer : public rclcpp::Node {
 public:
  Transformer();
  ~Transformer();

 private:
  // ✅ Separate subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr cone_subscriber_;

  rclcpp::Publisher<rc_interfaces::msg::Cones>::SharedPtr cone_publisher_;

  // ✅ Odom buffer
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  size_t buffer_size_ = 50;

  // ✅ TF
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // ✅ Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cone_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);

  // ✅ Helpers
  nav_msgs::msg::Odometry get_closest_odom(rclcpp::Time stamp);

  rc_interfaces::msg::Cone transform(double cone_x, double cone_y, double cone_z,
                                     const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);
};