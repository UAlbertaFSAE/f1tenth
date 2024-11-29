#include "odom_publisher.h"

#include <chrono>

using namespace std::chrono_literals;

OdomPublisher::OdomPublisher() : Node("odom_publisher_node") {
  this->declare_parameter("v", 0.5);
  this->declare_parameter("d", 0.5);
  this->declare_parameter("drive", "/drive");

  v_ = this->get_parameter("v").as_double();
  d_ = this->get_parameter("d").as_double();

  drive_ = this->get_parameter("drive").as_string();

  timer_ = this->create_wall_timer(1ms, [this] { publishMessage(); });

  publisher_drive_ =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_, 25);
}

void OdomPublisher::publishMessage() {
  auto drive_msg_obj = ackermann_msgs::msg::AckermannDriveStamped();

  v_ = this->get_parameter("v").as_double();
  d_ = this->get_parameter("d").as_double();

  drive_msg_obj.drive.speed = v_;
  drive_msg_obj.drive.steering_angle = d_;

  drive_msg_obj.header.stamp = rclcpp::Clock().now();

  RCLCPP_INFO(this->get_logger(), "Speed: %.2f, Steering Angle: %.2f", drive_msg_obj.drive.speed,
              drive_msg_obj.drive.steering_angle);

  publisher_drive_->publish(drive_msg_obj);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_ptr = std::make_shared<OdomPublisher>();  // initialise node pointer
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
