#include "triangulator.hpp"

Triangulator::Triangulator() : Node("triangulator_node") {
  this->declare_parameter("cones_topic", "/detection_generator/cone_data");
  this->declare_parameter("odom_topic", "/ego_racecar/odom");
  this->declare_parameter("waypoint_topic", "/waypoints");

  std::string cone_topic = this->get_parameter("cones_topic").as_string();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  std::string waypoint_topic = this->get_parameter("waypoint_topic").as_string();

  cone_subscriber = this->create_subscription<rc_interfaces::msg::Cones>(
      cone_topic, rclcpp::QoS(10), std::bind(&Triangulator::read_cones, this, _1));

  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), std::bind(&Triangulator::odom_callback, this, _1));

  waypoint_publisher = create_publisher<geometry_msgs::msg::Point>(waypoint_topic, rclcpp::QoS(10));

  last_cones = new rc_interfaces::msg::Cones();

  RCLCPP_INFO(this->get_logger(), "Starting up triangulator");
}

Triangulator::~Triangulator() { delete last_cones; }

void Triangulator::read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (!has_new_cones(cones)) {
    RCLCPP_INFO(this->get_logger(), "No new cones to process, still has %ld cones",
                cones->cones.size());
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

void Triangulator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  if (last_cones->cones.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "No cones for odom to process");
    return;
  }

  // split into left and right
  LR pair = split(last_cones);

  // find closest left and closest right
  rc_interfaces::msg::Cone left = closest_cone(pair.left, odom->pose.pose.position);
  rc_interfaces::msg::Cone right = closest_cone(pair.right, odom->pose.pose.position);

  RCLCPP_INFO(this->get_logger(), "Closest left cone: X: %f, Y: %f, Color: %s", left.x, left.y,
              left.color.c_str());
  RCLCPP_INFO(this->get_logger(), "Closest right cone: X: %f, Y: %f, Color: %s", right.x, right.y,
              right.color.c_str());

  // publish waypoint at midpoint
  geometry_msgs::msg::Point mp = midpoint(left, right);
  waypoint_publisher->publish(mp);

  // continue finding next closest left and rights based on previous angle
}

bool Triangulator::is_same_cone(const rc_interfaces::msg::Cone &coneA,
                                const rc_interfaces::msg::Cone &coneB) {
  return (coneA.x == coneB.x) && (coneA.y == coneB.y);
}

bool Triangulator::has_new_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones) {
  if (last_cones->cones.size() != cones->cones.size()) {
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

LR Triangulator::split(rc_interfaces::msg::Cones *cones) {
  LR pair;

  for (size_t i = 0; i < cones->cones.size(); i++) {
    rc_interfaces::msg::Cone cone = cones->cones[i];
    if (cone.color == LEFT) {
      pair.left.push_back(cone);
    } else if (cone.color == RIGHT) {
      pair.right.push_back(cone);
    } else {
      RCLCPP_INFO(this->get_logger(), "Found cone without left or right color");
    }
  }

  return pair;
}

rc_interfaces::msg::Cone Triangulator::closest_cone(std::vector<rc_interfaces::msg::Cone> &cones,
                                                    const geometry_msgs::msg::Point &position) {
  double dist = std::numeric_limits<double>::max();
  rc_interfaces::msg::Cone closest;

  // find the closest cone
  for (const auto &cone : cones) {
    if (distance(position.x, position.y, cone.x, cone.y) < dist) {
      closest = cone;
    }
  }

  return closest;
}

geometry_msgs::msg::Point Triangulator::midpoint(const rc_interfaces::msg::Cone &coneA,
                                                 const rc_interfaces::msg::Cone &coneB) {
  geometry_msgs::msg::Point point;
  point.x = (coneA.x + coneB.x) / 2;
  point.y = (coneA.y + coneB.y) / 2;
  point.z = 0;

  return point;
}

double Triangulator::distance(float x1, float y1, float x2, float y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Triangulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
