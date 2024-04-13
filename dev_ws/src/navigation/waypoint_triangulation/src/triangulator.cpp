#include "triangulator.hpp"

Triangulator::Triangulator() : Node("triangulator") {
  this->declare_parameter("cones_topic", "/cone_data");
  std::string cone_topic = this->get_parameter("cones_topic").as_string();

  cone_subscriber = this->create_subscription<rc_interfaces::msg::Cones>(
      cone_topic, rclcpp::QoS(10), std::bind(&Triangulator::read_cones, this, _1));
}

void Triangulator::read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (!has_new_cones(cones)) {
    RCLCPP_INFO(this->get_logger(), "No new cones to process");
    return;
  }

  for (int i = 0; i < cones->cones.size(); i++) {
    rc_interfaces::msg::Cone cone = cones->cones[i];
    RCLCPP_INFO(this->get_logger(), "Triangulator got cone: X: %f, Y: %f, Color: %s", cone.x,
                cone.y, cone.color.c_str());
  }
}

bool Triangulator::is_same_cone(const rc_interfaces::msg::Cone coneA,
                                const rc_interfaces::msg::Cone coneB) {
  return (coneA.x == coneB.x) && (coneA.y == coneB.y) && (coneA.color == coneB.color);
}

bool Triangulator::has_new_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (cones->cones.size() != last_cones->cones.size()) {
    return false;
  }

  for (int i = 0; i < cones->cones.size(); i++) {
    if (!is_same_cone(cones->cones[i], last_cones->cones[i])) {
      return false;
    }
  }

  return true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
