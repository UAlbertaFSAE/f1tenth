#include <fstream>
#include <string>
#include <vector>
#include "rc_interfaces/msg/cone.hpp"
#include "rclcpp/rclcpp.hpp"

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();
  void read_csv(std::string path);

 private:
  std::vector<rc_interfaces::msg::Cone> cones;
};
