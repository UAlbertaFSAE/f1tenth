#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class WaypointVisualizer : public rclcpp::Node {
public:
  WaypointVisualizer()
      : Node("waypoint_visualizer") {
    this->declare_parameter("waypoint_topic", "waypoints");
    this->declare_parameter("marker_topic", "waypoint_marker");
    this->declare_parameter("frame_id", "odom");
    this->declare_parameter("scale", 0.3);
    this->declare_parameter("lifetime", 1.0);

    waypoint_topic_ = this->get_parameter("waypoint_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    scale_ = this->get_parameter("scale").as_double();
    lifetime_sec_ = this->get_parameter("lifetime").as_double();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        waypoint_topic_, rclcpp::QoS(10),
        std::bind(&WaypointVisualizer::waypoint_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint visualizer started, subscribing to: %s",
                waypoint_topic_.c_str());
  }

private:
  void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr point) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = frame_id_;
    marker.ns = "waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point->x;
    marker.pose.position.y = point->y;
    marker.pose.position.z = point->z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale_;
    marker.scale.y = scale_;
    marker.scale.z = scale_;

    // green color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // lifetime
    builtin_interfaces::msg::Duration dur;
    int sec = static_cast<int>(std::floor(lifetime_sec_));
    double frac = lifetime_sec_ - sec;
    dur.sec = sec;
    dur.nanosec = static_cast<uint32_t>(frac * 1e9);
    marker.lifetime = dur;

    marker_pub_->publish(marker);

    RCLCPP_INFO(this->get_logger(), "Published waypoint marker at (%.3f, %.3f, %.3f)", point->x,
                point->y, point->z);
  }

  std::string waypoint_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double scale_;
  double lifetime_sec_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointVisualizer>());
  rclcpp::shutdown();
  return 0;
}
