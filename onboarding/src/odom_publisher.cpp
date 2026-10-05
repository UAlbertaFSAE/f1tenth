#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node {
  public:
    OdomPublisher() : Node("odom_publisher") {
      // create v and d with defaults 0.0
      this->declare_parameter<double>("v", 0.0);
      this->declare_parameter<double>("d", 0.0);

      // create publisher called /drive with message type AckermannDriveStamped
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

      // create timer
      timer_ = this->create_wall_timer(1ms, std::bind(&OdomPublisher::timer_callback, this));
    }

  private:
    // publish message with current v and d at 1kHz
    void timer_callback() {
      auto message = ackermann_msgs::msg::AckermannDriveStamped();

      // add timestamp to message header
      message.header.stamp = this->now();

      // get current v and d and add to message
      message.drive.speed = this->get_parameter("v").as_double();
      message.drive.steering_angle = this->get_parameter("d").as_double();

      // publish message
      publisher_->publish(message);
    }

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
