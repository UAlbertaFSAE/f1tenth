#include "cone_transformer.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"

Transformer::Transformer() : Node("coneTransformerNode"),
                             tf_buffer(this->get_clock()),
                             tf_listener(tf_buffer) {
  // this->declare_parameter("cones_topic", "/detection_generator/cone_data");
  this->declare_parameter("odom_topic", "/ego_racecar/odom");

  this->declare_parameter("cones_topic", "/cone_positions");



  std::string cone_topic = this->get_parameter("cones_topic").as_string();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();

  cone_subscriber = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      cone_topic, rclcpp::QoS(10), std::bind(&Transformer::cone_callback, this, _1));

  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), std::bind(&Transformer::odom_callback, this, _1));

  cone_publisher_ = this->create_publisher<rc_interfaces::msg::Cones>(
      "/detection_generator/cone_data", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Cone Transformer initialized");
  RCLCPP_INFO(this->get_logger(), "  Cones topic: %s", cone_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Odometry topic: %s", odom_topic.c_str());
}

Transformer::~Transformer() {}

void Transformer::cone_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "\n=== Received %ld cones ===", msg->objects.size());

  rc_interfaces::msg::Cones transformed_cones;
  for (size_t i = 0; i < msg->objects.size(); i++) {
    const auto& obj = msg->objects[i];
    double cone_x = obj.position[0];
    double cone_y = obj.position[1];
    double cone_z = obj.position[2];

    std::string cone_color = obj.label;
    cone_color.erase(0, 3);

    RCLCPP_INFO(this->get_logger(), "Cone %ld [%s]: X=%.3f, Y=%.3f, Z=%.3f, Conf=%.2f color=%s", i,
                obj.label.c_str(), cone_x, cone_y, cone_z, obj.confidence, cone_color.c_str());
    if (obj.confidence > 0.5) {
      // Calculate distance to car
      // double dist = distance(car_x, car_y, car_z, cone_x, cone_y, cone_z);
      rc_interfaces::msg::Cone c = transform(car_x, car_y, car_z, cone_x, cone_y, cone_z, msg);

      c.color = cone_color;
      transformed_cones.cones.push_back(c);

      // get x and y distance from the car to cone

      // RCLCPP_INFO(this->get_logger(), "  Distance to car: %.3f m", dist);
    }
  }

  cone_publisher_->publish(transformed_cones);
}

void Transformer::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  car_x = odom->pose.pose.position.x;
  car_y = odom->pose.pose.position.y;
  car_z = odom->pose.pose.position.z;

  RCLCPP_DEBUG(this->get_logger(), "Car position updated - X: %.3f, Y: %.3f, Z: %.3f", car_x,
               car_y, car_z);
}

rc_interfaces::msg::Cone Transformer::transform(double car_x, double car_y, double car_z, double cone_x, double cone_y, double cone_z, const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg) {
  geometry_msgs::msg::PointStamped cone_point;
  cone_point.header.frame_id = msg->header.frame_id;
  cone_point.point.x = cone_x;
  cone_point.point.y = cone_y;
  cone_point.point.z = cone_z;

  try{
    cone_point = tf_buffer.transform(cone_point, "odom", tf2::durationFromSec(1.0));

    RCLCPP_INFO(this->get_logger(), "Transformed cone position - X: %.3f, Y: %.3f", cone_point.point.x,
                cone_point.point.y);

    rc_interfaces::msg::Cone c;
    c.x = cone_point.point.x;
    c.y = cone_point.point.y;
    // c.color = msg->header.label;
    return c;

    // transformed_cones.cones.push_back(c);

    // auto out_msg = rc_interfaces::msg::Cones(*msg);
    // cone_publisher_->publish(*out_msg);

  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    // return -1.0; // Return an error value
  }


}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Transformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
