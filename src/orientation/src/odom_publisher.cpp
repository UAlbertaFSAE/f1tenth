#include "odom_publisher.h"

#include <cstdio>

OdomPublisher::OdomPublisher() : Node("odom_publisher") {
  // create v and d parameter
  this->declare_parameter<double>("v", 1.0);
  this->declare_parameter<double>("d", 0.0);

  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->timerCallback(); });
}

void OdomPublisher::timerCallback() {
  auto message = ackermann_msgs::msg::AckermannDriveStamped();
  message.drive.speed = this->get_parameter("v").as_double();
  message.drive.steering_angle = this->get_parameter("d").as_double();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.drive.speed);
  publisher_->publish(message);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();

  return 0;
}
