#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <array>
#include <unordered_map>
#include <vector>

using std::placeholders::_1;

// Left-Right pair of cone vectors
struct LR {
  std::vector<rc_interfaces::msg::Cone> left;
  std::vector<rc_interfaces::msg::Cone> right;
};

/**
 * Maintain two arrays, left and right of the same size which are the positions of the left and
 * right cones Use these two arrays to interpolate points between theme and publish these points for
 * pure pursuit
 */
class Triangulator : public rclcpp::Node {
 public:
  Triangulator();

 private:
  const std::string LEFT = "blue";
  const std::string RIGHT = "yellow";
  static constexpr float kConeKeyResolution = 0.5F;
  static constexpr int kMinObservationsToAccept = 1;
  static constexpr double kClusterDistance = 8.0;
  static constexpr double kMinTrackWidth = 0.4;
  static constexpr double kMaxTrackWidth = 10.0;

  rc_interfaces::msg::Cones accumulated_cones_;
  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  std::string frame_id_;
  std::unordered_map<std::string, int> cone_observation_counts_;
  bool has_active_cluster_center_ = false;
  geometry_msgs::msg::Point active_cluster_center_;

  // call backs
  void read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones_msg);

  bool is_same_cone(const rc_interfaces::msg::Cone& coneA,
                    const rc_interfaces::msg::Cone& coneB) const;
  std::string normalize_color(const std::string& raw_color) const;
  std::string make_cone_key(const rc_interfaces::msg::Cone& cone) const;
  rc_interfaces::msg::Cones filter_frame_cones(const rc_interfaces::msg::Cones& frame_cones);
  std::vector<geometry_msgs::msg::Point> build_waypoints_from_pairs(const LR& pair) const;
  bool has_cone(const rc_interfaces::msg::Cone& cone) const;
  LR split(const rc_interfaces::msg::Cones& cones);
  std::vector<geometry_msgs::msg::Point> build_waypoints_from_triangulation(
      const LR& pair, std::vector<std::array<geometry_msgs::msg::Point, 3>>* triangles_out);
  void publish_markers(const rc_interfaces::msg::Cones& cones,
                       const std::vector<std::array<geometry_msgs::msg::Point, 3>>& triangles,
                       const std::vector<geometry_msgs::msg::Point>& waypoints);
};
