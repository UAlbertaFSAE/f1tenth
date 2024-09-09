#include "odom_publisher.hpp"

#include <chrono>
#include <cmath>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std::chrono_literals;

OdomPublisher::OdomPublisher() : Node("odom_publisher") {
  rcl_interfaces::msg::ParameterDescriptor speed_desc;
  speed_desc.floating_point_range.resize(1);
  speed_desc.floating_point_range[0].from_value = -3.0;  // let car go backwards
  speed_desc.floating_point_range[0].to_value = 3.0;     // max speed is 3 m/s
  speed_desc.floating_point_range[0].step = 0;           // for continuous range

  rcl_interfaces::msg::ParameterDescriptor steering_angle_desc;
  steering_angle_desc.floating_point_range.resize(1);
  steering_angle_desc.floating_point_range[0].from_value = -2 * M_PI;  // -180 degrees in radians
  steering_angle_desc.floating_point_range[0].to_value = 2 * M_PI;
  steering_angle_desc.floating_point_range[0].step = 0;  // for continuous range

  this->declare_parameter("v", 0.0, speed_desc);
  this->declare_parameter("d", 0.0, steering_angle_desc);

  timer_ = this->create_wall_timer(1ms, [this] { timerCallback(); });
  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

  RCLCPP_INFO(this->get_logger(), "odom_publisher has been started");
}

void OdomPublisher::timerCallback() {
  auto msg = ackermann_msgs::msg::AckermannDriveStamped();

  double speed = this->get_parameter("v").as_double();
  double steering_angle = this->get_parameter("d").as_double();
  msg.drive.speed = speed;                    // NOLINT
  msg.drive.steering_angle = steering_angle;  // NOLINT

  msg.header.stamp = this->get_clock()->now();
  publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
