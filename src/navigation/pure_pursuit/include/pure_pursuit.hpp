/*
Pure Pursuit Implementation in C++. Includes features such as dynamic lookahead. Does not have
waypoint interpolation yet.
*/
#include <math.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {
 public:
  PurePursuit();

 private:
  // global static (to be shared by all objects) and dynamic variables (each instance gets its own
  // copy -> managed on the stack)
  struct csvFileData {
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> V;

    int index;
    int velocity_index;

    Eigen::Vector3d lookahead_point_world;  // from world reference frame (usually `map`)
    Eigen::Vector3d lookahead_point_car;    // from car reference frame
    Eigen::Vector3d
        current_point_world;  // Locks on to the closest waypoint, which gives a velocity profile
  };

  double x_car_world = 0.0;
  double y_car_world = 0.0;

  double car_orient_w;
  double car_orient_x;
  double car_orient_y;
  double car_orient_z;

  std::string odom_topic;
  std::string waypoint_topic;
  std::string car_refFrame;
  std::string drive_topic;
  std::string global_refFrame;
  std::string rviz_current_waypoint_topic;
  std::string rviz_lookahead_waypoint_topic;

  double K_p;
  double min_lookahead;
  double max_lookahead;
  double lookahead_ratio;
  double steering_limit;
  double velocity_percentage;
  double waypoint_velocity;
  double curr_velocity = 0.0;

  // struct initialisation
  csvFileData waypoints;
  int num_waypoints;

  // Timer initialisation
  rclcpp::TimerBase::SharedPtr timer_;

  // declare subscriber sharedpointer obj
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_subscriber;

  // declare publisher sharedpointer obj
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

  // private functions
  double to_radians(double degrees);
  double to_degrees(double radians);
  double p2pdist(double &x1, double &x2, double &y1, double &y2);
  bool point_is_behind_car(double x, double y);

  void visualize_lookahead_point(Eigen::Vector3d &point);
  void visualize_current_point(Eigen::Vector3d &point);

  void get_waypoint();

  bool transformandinterp_waypoint();

  double p_controller();

  double get_velocity(double steering_angle);

  void publish_message(double steering_angle);

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
  void waypoint_callback(const nav_msgs::msg::Path::ConstSharedPtr path);

  void timer_callback();
};
