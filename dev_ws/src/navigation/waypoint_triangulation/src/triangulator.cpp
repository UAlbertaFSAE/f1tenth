#include "triangulator.hpp"

Triangulator::Triangulator() : Node("triangulator_node") {
  this->declare_parameter("cones_topic", "/detection_generator/cone_data");
  std::string cone_topic = this->get_parameter("cones_topic").as_string();

  cone_subscriber = this->create_subscription<rc_interfaces::msg::Cones>(
      cone_topic, rclcpp::QoS(10), std::bind(&Triangulator::read_cones, this, _1));

  last_cones = new rc_interfaces::msg::Cones();

  RCLCPP_INFO(this->get_logger(), "Starting up triangulator");
}

Triangulator::~Triangulator() { delete last_cones; }

void Triangulator::read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (!has_new_cones(cones)) {
    RCLCPP_INFO(this->get_logger(), "No new cones to process");
    return;
  }

  last_cones->cones.clear();
  for (size_t i = 0; i < cones->cones.size(); i++) {
    rc_interfaces::msg::Cone cone = cones->cones[i];
    last_cones->cones.push_back(cone);
    RCLCPP_INFO(this->get_logger(), "Triangulator got cone: X: %f, Y: %f, Color: %s", cone.x,
                cone.y, cone.color.c_str());
  }
}

bool Triangulator::is_same_cone(const rc_interfaces::msg::Cone &coneA,
                                const rc_interfaces::msg::Cone &coneB) {
  return (coneA.x == coneB.x) && (coneA.y == coneB.y) && (coneA.color == coneB.color);
}

bool Triangulator::has_new_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (last_cones->cones.size() == 0) {
    return true;
  }

  for (size_t i = 0; i < last_cones->cones.size(); i++) {
    if (!is_same_cone(cones->cones[i], last_cones->cones[i])) {
      RCLCPP_INFO(
          this->get_logger(), "Different Cones: last=(x=%f, y=%f, c=%s). new=(x=%f, y=%f, c=%s)",
          last_cones->cones[i].x, last_cones->cones[i].y, last_cones->cones[i].color.c_str(),
          cones->cones[i].x, cones->cones[i].y, cones->cones[i].color.c_str());
      return true;
    }
  }

  return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
