#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class OdomRelay : public rclcpp::Node
{
public:
  OdomRelay() : Node("odom_relay")
  {
    // Create subscriber
    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10, std::bind(&OdomRelay::driveCallback, this, _1));

    // Create publisher
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);

    RCLCPP_INFO(this->get_logger(), "OdomRelay node started and listening to /drive");
  }

private:
  void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    // Create new message
    auto message = ackermann_msgs::msg::AckermannDriveStamped();

    // Update header time
    message.header.stamp = this->get_clock()->now();

    // Multiply values by 3
    message.drive.speed = msg->drive.speed * 3.0;
    message.drive.steering_angle = msg->drive.steering_angle * 3.0;

    // Log for debugging
    RCLCPP_INFO(this->get_logger(),
      "Received v=%.2f, d=%.2f | Publishing v=%.2f, d=%.2f",
      msg->drive.speed, msg->drive.steering_angle,
      message.drive.speed, message.drive.steering_angle);

    // Publish new message
    publisher_->publish(message);
  }

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
