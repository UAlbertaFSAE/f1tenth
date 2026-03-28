#include "cone_transformer.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"

using std::placeholders::_1;

Transformer::Transformer()
    : Node("coneTransformerNode"),
      tf_buffer(this->get_clock()),
      tf_listener(tf_buffer) {

  this->declare_parameter("cones_topic", "/cone_positions");
  std::string cone_topic = this->get_parameter("cones_topic").as_string();

  cone_subscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      cone_topic, rclcpp::QoS(10),
      std::bind(&Transformer::cone_callback, this, _1));

  cone_publisher_ = this->create_publisher<rc_interfaces::msg::Cones>(
      "/cone_transformed", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Cone Transformer initialized");
}

Transformer::~Transformer() {}

void Transformer::cone_callback(
    const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg) {

  rc_interfaces::msg::Cones transformed_cones;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    const auto& obj = msg->objects[i];

    if (obj.confidence <= 0.5) continue;

    double cone_x = obj.position[0];
    double cone_y = obj.position[1];
    double cone_z = obj.position[2];

    std::string cone_color = obj.label;
    if (cone_color.size() > 3)
      cone_color.erase(0, 3);

    rc_interfaces::msg::Cone c =
        transform(cone_x, cone_y, cone_z, msg);

    c.color = cone_color;
    transformed_cones.cones.push_back(c);
  }

  cone_publisher_->publish(transformed_cones);
}

rc_interfaces::msg::Cone Transformer::transform(
    double cone_x, double cone_y, double cone_z,
    const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg) {

  geometry_msgs::msg::PointStamped input_point;
  input_point.header = msg->header;  // includes frame_id + stamp
  input_point.point.x = cone_x;
  input_point.point.y = cone_y;
  input_point.point.z = cone_z;

  geometry_msgs::msg::PointStamped output_point;

  try {
    tf_buffer.transform(input_point, output_point, "odom",
                        tf2::durationFromSec(0.5));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    return rc_interfaces::msg::Cone();
  }

  rc_interfaces::msg::Cone c;
  c.x = output_point.point.x;
  c.y = output_point.point.y;

  RCLCPP_INFO(this->get_logger(),
              "Transformed cone -> X: %.3f Y: %.3f",
              c.x, c.y);

  return c;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Transformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}