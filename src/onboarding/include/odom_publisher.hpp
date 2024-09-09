#ifndef ODOM_PUBLISHER_H
#define ODOM_PUBLISHER_H

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomPublisher : public rclcpp::Node {
 public:
  OdomPublisher();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  void timerCallback();
};
#endif
