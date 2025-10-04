#ifndef ODOM_PUBLISHER_H_
#define ODOM_PUBLISHER_H_

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class OdomPublisher : public rclcpp::Node {
 public:
  OdomPublisher();

 private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

#endif
