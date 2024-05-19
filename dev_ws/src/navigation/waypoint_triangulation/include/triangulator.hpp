#include "nav_msgs/msg/odometry.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"

#include <limits.h>

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
  ~Triangulator();

 private:
  const std::string LEFT = "blue";
  const std::string RIGHT = "yellow";

  rc_interfaces::msg::Cones* last_cones;
  rc_interfaces::msg::Cone last_left;
  rc_interfaces::msg::Cone last_right;
  int interp_count;

  rclcpp::Subscription<rc_interfaces::msg::Cones>::SharedPtr cone_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher;

  // call backs
  void read_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  double distance(float x1, float y1, float x2, float y2);
  void publish_midpoint(rc_interfaces::msg::Cone left, rc_interfaces::msg::Cone right);
  std::vector<rc_interfaces::msg::Cone> interpolate_points(rc_interfaces::msg::Cone coneA,
                                                           rc_interfaces::msg::Cone coneB,
                                                           int count);
  bool is_same_cone(const rc_interfaces::msg::Cone& coneA, const rc_interfaces::msg::Cone& coneB);
  bool has_new_cones(const rc_interfaces::msg::Cones::ConstSharedPtr cones);
  LR split(rc_interfaces::msg::Cones* cones);
  rc_interfaces::msg::Cone closest_cone(std::vector<rc_interfaces::msg::Cone>& cones,
                                        const geometry_msgs::msg::Point& position);
  geometry_msgs::msg::Point midpoint(const rc_interfaces::msg::Cone& coneA,
                                     const rc_interfaces::msg::Cone& coneB);
};
