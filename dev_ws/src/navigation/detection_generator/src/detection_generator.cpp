#include "detection_generator/detection_generator.hpp"
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

DetectionGenerator::DetectionGenerator() : Node("detection_generator_node") {}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectionGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
