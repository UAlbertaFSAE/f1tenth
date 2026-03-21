
#include "odom_publisher.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // Spin keeps the node alive to continue publishing
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
