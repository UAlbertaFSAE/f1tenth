

#include <memory>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomRelay : public rclcpp::Node {
 public:
  OdomRelay() : Node("odom_relay") {
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive_manipulated", 10);
    // listen to the "drive" topic and relay messages to the console
    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", 10, std::bind(&OdomRelay::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    auto new_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
    new_msg->header = msg->header;
    new_msg->drive.steering_angle = (msg->drive.steering_angle * 3);
    new_msg->drive.speed = (msg->drive.speed * 3);
    publisher_->publish(*msg);
  }
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};
