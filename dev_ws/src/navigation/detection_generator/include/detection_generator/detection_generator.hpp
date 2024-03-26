#include "rclcpp/rclcpp.hpp"

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();

 private:
  int x;
};
