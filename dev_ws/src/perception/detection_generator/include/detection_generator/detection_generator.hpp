#include <fstream>
#include <string>
#include <vector>
#include "rc_interfaces/msg/cone.hpp"
#include "rclcpp/rclcpp.hpp"

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();

 private:
  float radius;
  void publish_cones();
  std::vector<rc_interfaces::msg::Cone> read_csv(std::string path);
  std::vector<rc_interfaces::msg::Cone> cones;
  rclcpp::Publisher<rc_interfaces::msg::Cone>::SharedPtr cone_publisher;
};
