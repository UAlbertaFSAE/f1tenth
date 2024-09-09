#ifndef ODOM_RELAY_H
#define ODOM_RELAY_H

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomRelay : public rclcpp::Node {
 public:
  OdomRelay();

 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscriber_;
  void subscriberCallback(const ackermann_msgs::msg::AckermannDriveStamped& msg);
};
#endif
