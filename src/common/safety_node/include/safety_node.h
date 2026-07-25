#ifndef SAFETY_NODE_H_
#define SAFETY_NODE_H_

#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
 public:
  Safety();

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void ScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
  void ConeCallback(const rc_interfaces::msg::Cones::ConstSharedPtr cones_msg);

  // Independent of the primary path-planning stack: if the car's current
  // position is closer than min_safe_cone_distance_ to any known blue/yellow
  // cone, it is at (or past) the track boundary -- brake regardless of what
  // pure_pursuit is doing.
  void CheckConeBoundary();
  void EmergencyBrake(const char* reason);

  double speed_;
  double car_x_;
  double car_y_;
  bool has_pose_;
  rc_interfaces::msg::Cones latest_cones_;
  bool has_cones_;
  double min_safe_cone_distance_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscription_;
};

#endif  // SAFETY_NODE_H
