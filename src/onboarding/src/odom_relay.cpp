#include "odom_relay.h"

using std::placeholders::_1;

OdomRelay::OdomRelay() : Node("odom_relay_node") {
  this->declare_parameter("drive_topic", "/drive");
  this->declare_parameter("drive_relay", "/relay");

  drive_topic_ = this->get_parameter("drive_topic").as_string();
  drive_relay_ = this->get_parameter("drive_relay").as_string();

  subscriber_drive_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_, 25, std::bind(&OdomRelay::relay, this, _1));

  publisher_relay_ =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_relay_, 25);
}

void OdomRelay::relay(const ackermann_msgs::msg::AckermannDriveStamped& drive_sub_msg_obj) {
  drive_topic_ = this->get_parameter("drive_topic").as_string();

  publishRelay(drive_sub_msg_obj.drive.speed * 3, drive_sub_msg_obj.drive.steering_angle * 3);
}

void OdomRelay::publishRelay(double speed, double steering_angle) {
  auto relay_msg_obj = ackermann_msgs::msg::AckermannDriveStamped();
  relay_msg_obj.drive.speed = speed;
  relay_msg_obj.drive.steering_angle = steering_angle;

  RCLCPP_INFO(this->get_logger(), "Speed: %.2f, Steering Angle: %.2f", relay_msg_obj.drive.speed,
              relay_msg_obj.drive.steering_angle);

  publisher_relay_->publish(relay_msg_obj);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
