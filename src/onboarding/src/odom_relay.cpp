
#include "odom_relay.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // Spin keeps the node alive to continue listening
  rclcpp::spin(std::make_shared<OdomRelay>());
  rclcpp::shutdown();
  return 0;
}
