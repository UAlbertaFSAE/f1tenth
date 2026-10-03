#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class odom_relay : public rclcpp::Node {
 public:
  odom_relay() : Node("odom_relay") {
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    subscriber_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", 10,
        // force the first argument to be odom_relay, leave one argument for the user to use
        std::bind(&odom_relay::receive_ackermann_message, this, std::placeholders::_1));
  }

 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscriber_;

  float new_speed_;
  int new_steering_angle_;

  void receive_ackermann_message(
      const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr request) {
    new_speed_ = request->drive.speed * 3;
    new_steering_angle_ = request->drive.steering_angle * 3;

    ackermann_msgs::msg::AckermannDriveStamped response;
    response.header.stamp = this->get_clock()->now();
    publisher_->publish(response);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_relay>());
  rclcpp::shutdown();

  return 0;
}
