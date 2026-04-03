#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class WaypointVisualizer : public rclcpp::Node {
public:
  WaypointVisualizer()
      : Node("waypoint_visualizer") {
    this->declare_parameter("waypoint_topic", "waypoints");
    this->declare_parameter("triangulation_marker_topic", "/triangulation_markers");
    this->declare_parameter("marker_topic", "/waypoint_visualization");
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("max_waypoints", 2000);

    waypoint_topic_ = this->get_parameter("waypoint_topic").as_string();
    triangulation_marker_topic_ = this->get_parameter("triangulation_marker_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    max_waypoints_ = this->get_parameter("max_waypoints").as_int();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    triangulation_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        triangulation_marker_topic_, rclcpp::QoS(10),
        std::bind(&WaypointVisualizer::triangulation_marker_callback, this, std::placeholders::_1));
    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        waypoint_topic_, rclcpp::QoS(10),
        std::bind(&WaypointVisualizer::waypoint_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint visualizer started, subscribing to: %s",
                waypoint_topic_.c_str());
  }

private:
  void triangulation_marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr markers) {
    latest_triangulation_markers_ = *markers;
    publish_combined_markers();
  }

  void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr point) {
    waypoint_history_.push_back(*point);
    if (static_cast<int64_t>(waypoint_history_.size()) > max_waypoints_) {
      waypoint_history_.erase(waypoint_history_.begin());
    }

    publish_combined_markers();
  }

  void publish_combined_markers() {
    visualization_msgs::msg::MarkerArray combined = latest_triangulation_markers_;

    visualization_msgs::msg::Marker history;
    history.header.stamp = this->now();
    history.header.frame_id = frame_id_;
    history.ns = "waypoint_history";
    history.id = 100;
    history.type = visualization_msgs::msg::Marker::LINE_STRIP;
    history.action = visualization_msgs::msg::Marker::ADD;
    history.pose.orientation.w = 1.0;
    history.scale.x = 0.12;
    history.color.r = 0.0f;
    history.color.g = 1.0f;
    history.color.b = 0.4f;
    history.color.a = 1.0f;
    history.points = waypoint_history_;

    visualization_msgs::msg::Marker latest;
    latest.header = history.header;
    latest.ns = "waypoint_latest";
    latest.id = 101;
    latest.type = visualization_msgs::msg::Marker::SPHERE;
    latest.action = visualization_msgs::msg::Marker::ADD;
    latest.pose.orientation.w = 1.0;
    latest.scale.x = 0.28;
    latest.scale.y = 0.28;
    latest.scale.z = 0.28;
    latest.color.r = 0.0f;
    latest.color.g = 1.0f;
    latest.color.b = 0.0f;
    latest.color.a = 1.0f;
    if (!waypoint_history_.empty()) {
      latest.pose.position = waypoint_history_.back();
    }

    combined.markers.push_back(history);
    combined.markers.push_back(latest);
    marker_pub_->publish(combined);
  }

  std::string waypoint_topic_;
  std::string triangulation_marker_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  int64_t max_waypoints_;
  std::vector<geometry_msgs::msg::Point> waypoint_history_;
  visualization_msgs::msg::MarkerArray latest_triangulation_markers_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr triangulation_marker_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointVisualizer>());
  rclcpp::shutdown();
  return 0;
}
