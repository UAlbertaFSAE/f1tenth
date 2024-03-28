#include "detection_generator/detection_generator.hpp"

DetectionGenerator::DetectionGenerator() : Node("detection_generator_node") {}

void DetectionGenerator::read_csv(std::string path) {
  std::fstream file;
  file.open(path);
  RCLCPP_INFO(this->get_logger(), "%s", (file.is_open() ? "Opened file" : "Failed to open file"));

  std::string line, word;
  while (!file.eof()) {
    std::getline(file, line, '\n');
    std::stringstream s(line);

    int j = 0;
    rc_interfaces::msg::Cone cone;
    while (getline(s, word, ',')) {
      if (!word.empty()) {
        if (j == 0) {
          cone.x = std::stod(word);
          RCLCPP_INFO(this->get_logger(), "X: %f", cone.x);
        }

        if (j == 1) {
          cone.y = std::stod(word);
          RCLCPP_INFO(this->get_logger(), "Y: %f", cone.y);
        }

        if (j == 2) {
          cone.color = word;
          RCLCPP_INFO(this->get_logger(), "Color: %s", cone.color.c_str());
        }
      }

      j++;
    }
    cones.push_back(cone);
  }

  file.close();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionGenerator>();
  node->read_csv("/f1tenth/dev_ws/src/perception/detection_generator/data/cone_positions.csv");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
