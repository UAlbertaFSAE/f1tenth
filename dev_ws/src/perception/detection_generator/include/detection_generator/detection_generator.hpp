#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class DetectionGenerator : public rclcpp::Node {
 public:
  DetectionGenerator();

 private:
  float radius;
  void publish_cones(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  std::vector<rc_interfaces::msg::Cone> read_csv(std::string path);
  std::vector<rc_interfaces::msg::Cone> cones;
  rclcpp::Publisher<rc_interfaces::msg::Cones>::SharedPtr cone_publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
};
