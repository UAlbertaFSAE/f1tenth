#include "odom_relay.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

OdomRelay::OdomRelay() : Node("odom_relay") {
  // ensure QoS is same for subscriber as it is for the publisher on the topic it
  // is listening too (10 in this case). Also, the lambda version of this is so much
  // more unreadable than the std::bind version so I am using std::bind here
  subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10,
      std::bind(&OdomRelay::subscriberCallback, this, std::placeholders::_1));  // NOLINT
  publisher_ =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);

  RCLCPP_INFO(this->get_logger(), "odom_relay has been started");
}

void OdomRelay::subscriberCallback(const ackermann_msgs::msg::AckermannDriveStamped& msg) {
  double relay_speed = msg.drive.speed * 3;
  double relay_steering_angle = msg.drive.steering_angle * 3;

  auto relay_msg = ackermann_msgs::msg::AckermannDriveStamped();
  relay_msg.drive.speed = relay_speed;                    // NOLINT
  relay_msg.drive.steering_angle = relay_steering_angle;  // NOLINT

  relay_msg.header.stamp = this->get_clock()->now();
  publisher_->publish(relay_msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
