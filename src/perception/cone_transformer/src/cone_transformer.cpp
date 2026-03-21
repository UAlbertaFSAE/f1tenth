#include "cone_transformer.hpp"

#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"

Transformer::Transformer()
    : Node("coneTransformerNode"), tf_buffer(this->get_clock()), tf_listener(tf_buffer) {
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("cones_topic", "/cone_positions");
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  std::string cone_topic = this->get_parameter("cones_topic").as_string();

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      std::bind(&Transformer::odom_callback, this, _1));

  cone_subscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      cone_topic, rclcpp::QoS(10),
      std::bind(&Transformer::cone_callback, this, _1));

  cone_publisher_ = this->create_publisher<rc_interfaces::msg::Cones>(
      "/detection_generator/cone_data", rclcpp::QoS(10));
  
  RCLCPP_INFO(this->get_logger(), "Cone Transformer initialized");
  RCLCPP_INFO(this->get_logger(), "  Cones topic: %s", cone_topic.c_str());
}

Transformer::~Transformer() {}

void Transformer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_buffer_.push_back(*msg);

  if (odom_buffer_.size() > buffer_size_) {
    odom_buffer_.pop_front();
  }
}

nav_msgs::msg::Odometry Transformer::get_closest_odom(rclcpp::Time stamp) {
  nav_msgs::msg::Odometry best;
  double best_dt = std::numeric_limits<double>::max();

  for (const auto& odom : odom_buffer_) {
    double dt = fabs((rclcpp::Time(odom.header.stamp) - stamp).seconds());
    if (dt < best_dt) {
      best_dt = dt;
      best = odom;
    }
  }
  return best;
}

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
      rc_interfaces::msg::Cone c = transform(cone_x, cone_y, cone_z, msg);

      c.color = cone_color;
      transformed_cones.cones.push_back(c);

      // RCLCPP_INFO(this->get_logger(), "  Distance to car: %.3f m", dist);
    }
  }

  cone_publisher_->publish(transformed_cones);
}

rc_interfaces::msg::Cone Transformer::transform(
    double cone_x, double cone_y, double cone_z,
    const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg) {

  geometry_msgs::msg::PointStamped cone_point, transformed_point;

  cone_point.header = msg->header;
  cone_point.point.x = cone_x;
  cone_point.point.y = cone_y;
  cone_point.point.z = cone_z;
  
  // Transform 
  try {
    transformed_point = tf_buffer.transform(
        cone_point, "odom", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
    return rc_interfaces::msg::Cone();
  }

  rc_interfaces::msg::Cone c;

  // Final ENU position of cone
  c.x = transformed_point.point.x;
  c.y = transformed_point.point.y;

  RCLCPP_INFO(this->get_logger(),
              "Cone global position: X=%.3f Y=%.3f",
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
