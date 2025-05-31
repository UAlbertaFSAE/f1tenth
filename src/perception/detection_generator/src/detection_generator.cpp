// NOLINTBEGIN
#include "detection_generator/detection_generator.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

DetectionGenerator::DetectionGenerator() : Node("detection_generator_node") {
  std::string csv_path = "/f1tenth/src/perception/detection_generator/data/cone_positions.csv";
  this->declare_parameter("radius", 5.0);
  this->declare_parameter("odom_topic", "/ego_racecar/odom");

  radius = this->get_parameter("radius").as_double();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();

  cone_viz_pub =
      create_publisher<visualization_msgs::msg::MarkerArray>("/cone_viz", rclcpp::QoS(10));

  cones = read_csv(csv_path);
  cone_publisher = create_publisher<rc_interfaces::msg::Cones>("cone_data", rclcpp::QoS(10));
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), std::bind(&DetectionGenerator::publish_cones, this, _1));
}

std::vector<rc_interfaces::msg::Cone> DetectionGenerator::read_csv(std::string path) {
  std::fstream file;
  file.open(path);
  RCLCPP_INFO(this->get_logger(), "%s", (file.is_open() ? "Opened file" : "Failed to open file"));

  visualization_msgs::msg::MarkerArray allMarkers;

  int idIndex = 0;

  std::string line, word;
  std::vector<rc_interfaces::msg::Cone> cones;
  while (!file.eof()) {
    std::getline(file, line, '\n');
    std::stringstream s(line);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time();
    marker.ns = "my_namespace";
    marker.id = idIndex;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    int j = 0;
    rc_interfaces::msg::Cone cone;
    while (getline(s, word, ',')) {
      if (!word.empty()) {
        if (j == 0) {
          cone.x = std::stod(word);
          marker.pose.position.x = cone.x;
        }

        if (j == 1) {
          cone.y = std::stod(word);
          marker.pose.position.y = cone.y;
        }

        if (j == 2) {
          cone.color = word;
          char clr[] = "yellow";
          if (word == clr) {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
          }
        }
      }

      j++;
    }
    allMarkers.markers.push_back(marker);
    cones.push_back(cone);
    ++idIndex;
  }

  cone_viz_pub->publish(allMarkers);

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

  float carX = odom->pose.pose.position.x;
  float carY = odom->pose.pose.position.y;
  Eigen::Matrix3d rotation_matrix =
      Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                         odom->pose.pose.orientation.y, odom->pose.pose.orientation.z)
          .toRotationMatrix();

  rc_interfaces::msg::Cones visible_cones;
  for (const auto& cone : cones) {
    double dist = distance(carX, carY, cone.x, cone.y);
    if (dist > radius) {
      continue;
    }

    // calculate vector from current position to cone
    Eigen::Vector3d cone_vector(cone.x - carX, cone.y - carY, 0.0);

    // transform vector to car's coordinate frame using rotation matrix
    Eigen::Vector3d transformed_vector = rotation_matrix.transpose() * cone_vector;

    // check if the cone is within the radius and in front of the current position
    if (transformed_vector.x() >= 0) {
      RCLCPP_INFO(this->get_logger(), "Found cone: x=%f, y=%f, color=%s", cone.x, cone.y,
                  cone.color.c_str());
      visible_cones.cones.push_back(cone);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Publishing %ld visible cones", visible_cones.cones.size());
  cone_publisher->publish(visible_cones);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// NOLINTEND
