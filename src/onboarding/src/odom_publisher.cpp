
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
  public:
  OdomPublisher() : Node("odom_publisher")
  {
    this->declare_parameter<double>("v", 0.0);
    this->declare_parameter<double>("d", 0.0);

    publisher_ = this->create_publisher<
      ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    timer_ = this->create_wall_timer(
      1ms,
      std::bind(&OdomPublisher::publish_drive, this));
  }

private:
  void publish_drive()
  {
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();

    msg.header.stamp = this->get_clock()->now();
    msg.drive.speed = this->get_parameter("v").as_double();
    msg.drive.steering_angle = this->get_parameter("d").as_double();

    publisher_->publish(msg);
  }

  rclcpp::Publisher<
    ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
