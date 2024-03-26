#include <cstdio>
#include "rc_interfaces/msg/cone.hpp"
#include "rclcpp/rclcpp.hpp"

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();

 private:
  std::vector<rc_interfaces::msg::Cone> cones;
};
