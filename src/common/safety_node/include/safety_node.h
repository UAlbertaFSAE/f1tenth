#ifndef SAFETY_NODE_H_
#define SAFETY_NODE_H_

#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
 public:
  Safety();

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void ScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  double speed_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

#endif  // SAFETY_NODE_H