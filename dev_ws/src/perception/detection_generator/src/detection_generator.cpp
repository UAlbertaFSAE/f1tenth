#include "detection_generator/detection_generator.hpp"

DetectionGenerator::DetectionGenerator() : Node("detection_generator_node") {
  std::string csv_path =
      "/f1tenth/dev_ws/src/perception/detection_generator/data/cone_positions.csv";
  this->declare_parameter("radius", 4.0);
  this->declare_parameter("odom_topic", "/ego_racecar/odom");

  radius = this->get_parameter("radius").as_double();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();

  cones = read_csv(csv_path);
  cone_publisher = create_publisher<rc_interfaces::msg::Cone>("cone_data", rclcpp::QoS(10));
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), std::bind(&DetectionGenerator::publish_cones, this, _1));
}

std::vector<rc_interfaces::msg::Cone> DetectionGenerator::read_csv(std::string path) {
  std::fstream file;
  file.open(path);
  RCLCPP_INFO(this->get_logger(), "%s", (file.is_open() ? "Opened file" : "Failed to open file"));

  std::string line, word;
  std::vector<rc_interfaces::msg::Cone> cones;
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
  return cones;
}

double distance(float x1, float y1, float x2, float y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void DetectionGenerator::publish_cones(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  if (cones.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "No cones to process");
    return;
  }

  // todo: get these three values from odom
  float direction = 0;  // should be in radians
  float carX = 0;
  float carY = 0;

  for (const auto& cone : cones) {
    double dist = distance(carX, carY, cone.x, cone.y);

    // calculate angle between current heading and vector pointing to cone
    double angle = atan2(cone.y - carY, cone.x - carX);
    double angleDiff = angle - direction;

    // normalize angle difference to be within [-pi, pi]
    if (angleDiff > M_PI) {
      angleDiff -= 2 * M_PI;
    } else if (angleDiff < -M_PI) {
      angleDiff += 2 * M_PI;
    }

    // check if the cone is within the radius and in front of the current position
    if (dist <= radius && angleDiff >= -M_PI / 2 && angleDiff <= M_PI / 2) {
      // TODO: accumulate good cones into array and publish entire array at the end
      RCLCPP_INFO(this->get_logger(), "Publishing cone: x=%f, y=%f, color=%s", cone.x, cone.y,
                  cone.color.c_str());
      cone_publisher->publish(cone);
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
