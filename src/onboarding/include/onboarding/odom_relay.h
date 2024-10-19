#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomRelay : public rclcpp::Node {
 public:
  OdomRelay();

 private:
  std::string drive_topic_;
  std::string drive_relay_;

  double speed_;
  double steering_angle_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscriber_drive_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_relay_;

  void relay(const ackermann_msgs::msg::AckermannDriveStamped& drive_sub_msg_obj);

  void publishRelay(double speed, double steering_angle);
};
