#include "triangulator.hpp"

Triangulator::Triangulator() : Node("triangulator") {
  this->declare_parameter("cones_topic", "/cone_data");
  std::string cone_topic = this->get_parameter("cone_topic").as_string();

  cone_subscriber = this->create_subscription<rc_interfaces::msg::Cones>(
      cone_topic, rclcpp::QoS(10), std::bind(&Triangulator::read_cones, this, _1));
}

void Triangulator::read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
