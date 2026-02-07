#include <cmath>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_msgs/msg/object.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

using std::placeholders::_1;

class Transformer : public rclcpp::Node {
 public:
  Transformer();
  ~Transformer();

  // Delete copy and move operations
  Transformer(const Transformer&) = delete;
  Transformer& operator=(const Transformer&) = delete;
  Transformer(Transformer&&) = delete;
  Transformer& operator=(Transformer&&) = delete;

 private:
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr cone_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void cone_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  void transform(double car_x, double car_y, double car_z, double cone_x, double cone_y, double cone_z);

  double car_x = 0.0;
  double car_y = 0.0;
  double car_z = 0.0;
};
