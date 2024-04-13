#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Triangulator : public rclcpp::Node {
 public:
  Triangulator();

 private:
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscriber;
  void read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
};
