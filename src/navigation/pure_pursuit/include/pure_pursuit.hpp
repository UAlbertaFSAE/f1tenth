/*
Pure Pursuit Implementation in C++. Includes features such as dynamic lookahead. Does not have
waypoint interpolation yet.
*/
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

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

  Eigen::Matrix3d rotation_m;

  double x_car_world;
  double y_car_world;

  double car_orient_w;
  double car_orient_x;
  double car_orient_y;
  double car_orient_z;

  std::string odom_topic;
  std::string waypoint_topic;
  std::string car_refFrame;
  std::string drive_topic;
  std::string global_refFrame;

  double K_p;
  double min_lookahead;
  double max_lookahead;
  double lookahead_ratio;
  double steering_limit;
  double velocity_percentage;
  double waypoint_velocity;
  // Starting at 0 pins the very first lookahead computation to min_lookahead
  // (lookahead = max_lookahead * curr_velocity / lookahead_ratio, clamped
  // to at least min_lookahead), which is often shorter than the gap between
  // consecutive waypoints -- nothing passes the in-range test, so
  // has_valid_waypoint stays false, publish_stop() fires instead of
  // publish_message(), and curr_velocity (only ever set inside
  // publish_message()) never leaves 0: a permanent cold-start deadlock.
  // Starting at waypoint_velocity's default instead gives a real lookahead
  // distance immediately.
  double curr_velocity = 6.0;

  // Safety guards: without these, a stale or wildly-wrong waypoint stream
  // (e.g. triangulator going silent, or a degraded fallback path at a sharp
  // corner) leaves get_waypoint() driving full speed toward whatever it last
  // resolved to -- including index 0 if nothing in range was ever found --
  // with no way to detect that anything is wrong.
  double waypoint_staleness_timeout;
  double max_target_distance;
  rclcpp::Time last_waypoint_time;
  bool have_waypoint_time = false;
  // Set by get_waypoint(): false when every waypoint failed the in-range/
  // not-behind-car test, meaning there is no legitimate target this cycle
  // (as opposed to defaulting to index 0 regardless of how stale/far it is).
  bool has_valid_waypoint = false;

  bool emergency_breaking = false;
  std::string lane_number = "left";  // left or right lane

  // file object
  std::fstream csvFile_waypoints;

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

  // declare tf shared pointers
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // private functions
  double to_radians(double degrees);
  double to_degrees(double radians);
  double p2pdist(double &x1, double &x2, double &y1, double &y2);
  bool point_is_behind_car(double x, double y);

  void get_waypoint();

  void quat_to_rot(double q0, double q1, double q2, double q3);

  bool transformandinterp_waypoint();

  double p_controller();

  double get_velocity(double steering_angle);

  void publish_message(double steering_angle);
  void publish_stop();

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
  void waypoint_callback(const nav_msgs::msg::Path::ConstSharedPtr path);

  void timer_callback();
};
