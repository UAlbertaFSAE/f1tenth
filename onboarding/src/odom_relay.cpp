#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class OdomRelay : public rclcpp::Node {
  public:
    OdomRelay() : Node("odom_relay") {
      // subscribe to /drive
      subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10, std::bind(&OdomRelay::topic_callback, this, _1));

      // create publisher called /drive_relay with message type AckermannDriveStamped
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

  private:
    // publish relay_message with modified v and d at 1kHz
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const {
      auto relay_message = ackermann_msgs::msg::AckermannDriveStamped();

      // add timestamp to relay_message header
      relay_message.header.stamp = this->now();

      // get current v and d from subscription, modify, and add to relay_message
      relay_message.drive.speed = msg->drive.speed * 3.0;
      relay_message.drive.steering_angle = msg->drive.steering_angle * 3.0;

      // publish relay_message
      publisher_->publish(relay_message);
    }

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
