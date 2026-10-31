
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class OdomRelay : public rclcpp::Node
{
public:
  OdomRelay() : Node("odom_relay")
  {
    publisher_ = this->create_publisher<
      ackermann_msgs::msg::AckermannDriveStamped>(
        "drive_relay", 10);

    subscription_ = this->create_subscription<
      ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", 10,
        std::bind(&OdomRelay::relay_callback, this,
                  std::placeholders::_1));
  }

private:
  void relay_callback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    auto new_msg = ackermann_msgs::msg::AckermannDriveStamped();

    // Copy the original header, including its timestamp
    new_msg.header = msg->header;

    // Multiply speed and steering angle by 3
    new_msg.drive.speed = msg->drive.speed * 3.0;
    new_msg.drive.steering_angle =
      msg->drive.steering_angle * 3.0;

    // Publish the modified message
    publisher_->publish(new_msg);
  }

  rclcpp::Publisher<
    ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

  rclcpp::Subscription<
    ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
