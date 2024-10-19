#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomPublisher : public rclcpp::Node {
 public:
  OdomPublisher();

 private:
  double v_;
  double d_;

  std::string drive_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive_;

  void publishMessage();
};
