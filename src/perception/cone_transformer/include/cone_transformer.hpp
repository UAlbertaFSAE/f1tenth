#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rc_interfaces/msg/cone.hpp"
#include "rc_interfaces/msg/cones.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_msgs/msg/object.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"

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
  rclcpp::Publisher<rc_interfaces::msg::Cones>::SharedPtr cone_publisher_;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void cone_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);

  rc_interfaces::msg::Cone transform(double cone_x, double cone_y, double cone_z,
                                     const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);
};
