#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // custom messages the publisher nodes sends

using namespace std;
using namespace std::chrono_literals;

class odom_publisher : public rclcpp::Node {
 public:
  odom_publisher(float speed, float steering_angle)
      : Node("odom_publisher"), speed(speed), steering_angle(steering_angle) {
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    timer_ = this->create_wall_timer(1000ms, [this] { send_ackermann_message(); });

    this->declare_parameter("speed", this->speed);
    this->declare_parameter("steering_angle", this->steering_angle);
  }

 private:
  void send_ackermann_message() {
    ackermann_msgs::msg::AckermannDriveStamped message;  // like a custom message protocol here
    message.drive.speed = this->speed;
    message.drive.steering_angle = this->steering_angle;
    message.header.stamp = this->get_clock()->now();
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

  float speed;
  float steering_angle;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_publisher>(50, 20));
  rclcpp::shutdown();

  return 0;
}
