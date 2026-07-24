#include "pure_pursuit.hpp"

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
#include "rclcpp/rclcpp.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
  // initialise parameters
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("waypoint_topic", "/waypoints");
  this->declare_parameter("car_refFrame", "base_link");
  this->declare_parameter("drive_topic", "/drive");
  this->declare_parameter("global_refFrame", "odom");
  this->declare_parameter("min_lookahead", 0.8);
  this->declare_parameter("max_lookahead", 4.0);
  this->declare_parameter("lookahead_ratio", 4.0);
  this->declare_parameter("K_p", 0.30);
  this->declare_parameter("steering_limit", 25.0);
  this->declare_parameter("velocity_percentage", 1.0);  // 0.6 default
  this->declare_parameter("waypoint_velocity", 6.0);  // m/s target speed along cone-derived waypoints
  // Safety guards (see pure_pursuit.hpp): stop instead of driving toward a
  // stale or absurdly-far-away target.
  this->declare_parameter("waypoint_staleness_timeout", 0.5);
  this->declare_parameter("max_target_distance", 15.0);

  // Default Values
  odom_topic = this->get_parameter("odom_topic").as_string();
  waypoint_topic = this->get_parameter("waypoint_topic").as_string();
  car_refFrame = this->get_parameter("car_refFrame").as_string();
  drive_topic = this->get_parameter("drive_topic").as_string();
  global_refFrame = this->get_parameter("global_refFrame").as_string();
  min_lookahead = this->get_parameter("min_lookahead").as_double();
  max_lookahead = this->get_parameter("max_lookahead").as_double();
  lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
  K_p = this->get_parameter("K_p").as_double();
  steering_limit = this->get_parameter("steering_limit").as_double();
  velocity_percentage = this->get_parameter("velocity_percentage").as_double();
  waypoint_velocity = this->get_parameter("waypoint_velocity").as_double();
  waypoint_staleness_timeout = this->get_parameter("waypoint_staleness_timeout").as_double();
  max_target_distance = this->get_parameter("max_target_distance").as_double();

  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));

  waypoint_subscriber = this->create_subscription<nav_msgs::msg::Path>(
      waypoint_topic, rclcpp::QoS(10), std::bind(&PurePursuit::waypoint_callback, this, _1));

  timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

  publisher_drive =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched, odom_topic: %s",
              odom_topic.c_str());

  waypoints.index = 0;
  waypoints.velocity_index = 0;
  x_car_world = 0.0;
  y_car_world = 0.0;
  car_orient_w = 1.0;
  car_orient_x = 0.0;
  car_orient_y = 0.0;
  car_orient_z = 0.0;

  num_waypoints = 0;
}

double PurePursuit::to_radians(double degrees) {
  double radians;
  return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
  double degrees;
  return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
  double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  return dist;
}

void PurePursuit::get_waypoint() {
  // Main logic: Search within the next 500 points
  double longest_distance = 0;
  int final_i = -1;
  int start = waypoints.index;
  int end = (waypoints.index + 500) % num_waypoints;

  // Lookahead needs to be between the min_lookhead and the max_lookahead
  double lookahead = std::min(
      std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

  if (end < start) {  // If we need to loop around
    for (int i = start; i < num_waypoints; i++) {
      if (point_is_behind_car(waypoints.X[i], waypoints.Y[i])) continue;

      if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
          p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
        longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
        final_i = i;
      }
    }
    for (int i = 0; i < end; i++) {
      if (point_is_behind_car(waypoints.X[i], waypoints.Y[i])) continue;

      if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
          p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
        longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
        final_i = i;
      }
    }
  } else {
    for (int i = start; i < end; i++) {
      if (point_is_behind_car(waypoints.X[i], waypoints.Y[i])) continue;

      if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
          p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
        longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
        final_i = i;
      }
    }
  }

  if (final_i == -1) {  // if we haven't found anything, search from the beginning
    for (int i = 0; i < num_waypoints; i++) {
      if (point_is_behind_car(waypoints.X[i], waypoints.Y[i])) continue;

      if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
          p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
        longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
        final_i = i;
      }
    }
  }

  // If nothing passed the not-behind-car/in-range test anywhere in the whole
  // array, there is no legitimate target this cycle -- leave waypoints.index
  // untouched (stale-but-not-wrong) and let the caller treat this as unsafe,
  // instead of silently defaulting to index 0 regardless of where that is.
  if (final_i == -1) {
    has_valid_waypoint = false;
    return;
  }

  // Find the closest point to the car, and use the velocity index for that
  double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
  int velocity_i = 0;
  for (int i = 0; i < num_waypoints; i++) {
    if (point_is_behind_car(waypoints.X[i], waypoints.Y[i])) continue;

    if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
      shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
      velocity_i = i;
    }
  }

  waypoints.index = final_i;
  waypoints.velocity_index = velocity_i;
  has_valid_waypoint = true;
}

bool PurePursuit::point_is_behind_car(double x, double y) {
  Eigen::Matrix3d rotation_matrix =
      Eigen::Quaterniond(car_orient_w, car_orient_x, car_orient_y, car_orient_z)
          .toRotationMatrix();
  Eigen::Vector3d cone_vector(x - x_car_world, y - y_car_world, 0.0);
  Eigen::Vector3d transformed_vector = rotation_matrix.transpose() * cone_vector;
  if (transformed_vector.x() >= 0) return false;
  return true;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
  double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
  double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
  double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

  double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
  double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
  double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

  double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
  double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
  double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

  rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

bool PurePursuit::transformandinterp_waypoint() {  // pass old waypoint here
  // initialise vectors
  waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index],
      0.0;
  waypoints.current_point_world << waypoints.X[waypoints.velocity_index],
      waypoints.Y[waypoints.velocity_index], 0.0;

  // look up transformation at that instant from tf_buffer_
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    // Get the transform from the base_link reference to world reference frame
    transformStamped =
        tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
    return false;
  }

  // transform points (rotate first and then translate)
  Eigen::Vector3d translation_v(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z);
  quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
              transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

  waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
  return true;
}

double PurePursuit::p_controller() {
  double r = waypoints.lookahead_point_car.norm();  // r = sqrt(x^2 + y^2)
  if (r < 1e-6) {
    return 0.0;
  }
  double y = waypoints.lookahead_point_car(1);
  double angle =
      K_p * 2 * y /
      pow(r,
          2);  // Calculated from
               // https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

  return angle;
}

double PurePursuit::get_velocity(double steering_angle) {
  double velocity = 0;

  if (waypoints.V[waypoints.velocity_index]) {
    velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
  } else {  // For waypoints loaded without velocity profiles
    if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
      velocity = 6.0 * velocity_percentage;
    } else if (abs(steering_angle) >= to_radians(10.0) &&
               abs(steering_angle) <= to_radians(20.0)) {
      velocity = 2.5 * velocity_percentage;
    } else {
      velocity = 2.0 * velocity_percentage;
    }
  }

  return velocity;
}

void PurePursuit::publish_message(double steering_angle) {
  auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msgObj.header.stamp = this->now();
  if (steering_angle < 0.0) {
    drive_msgObj.drive.steering_angle =
        std::max(steering_angle,
                 -to_radians(steering_limit));  // ensure steering angle is dynamically capable
  } else {
    drive_msgObj.drive.steering_angle =
        std::min(steering_angle,
                 to_radians(steering_limit));  // ensure steering angle is dynamically capable
  }

  curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
  drive_msgObj.drive.speed = curr_velocity;

  RCLCPP_INFO(this->get_logger(),
              "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: "
              "%.2f ... "
              "velocity_percentage: %.2f",
              waypoints.index,
              p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index],
                      y_car_world),
              drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p,
              velocity_percentage);

  publisher_drive->publish(drive_msgObj);
}

void PurePursuit::publish_stop() {
  auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msgObj.header.stamp = this->now();
  drive_msgObj.drive.steering_angle = 0.0;
  drive_msgObj.drive.speed = 0.0;
  curr_velocity = 0.0;
  publisher_drive->publish(drive_msgObj);
}

void PurePursuit::waypoint_callback(const nav_msgs::msg::Path::ConstSharedPtr path) {
  // The triangulator recomputes and republishes its whole local path every
  // cycle -- treat each message as the current path, atomically replacing the
  // old one. Streaming/appending individual points into a small FIFO let a
  // single noisy frame partially evict a good path and blend in stale points
  // from an unrelated frame, which is what caused the car to run off track.
  if (path->poses.empty()) {
    return;
  }

  waypoints.X.clear();
  waypoints.Y.clear();
  waypoints.V.clear();

  for (const auto& pose : path->poses) {
    waypoints.X.push_back(pose.pose.position.x);
    waypoints.Y.push_back(pose.pose.position.y);
    waypoints.V.push_back(waypoint_velocity);
  }
  num_waypoints = static_cast<int>(waypoints.X.size());

  // Re-anchor the search index to whichever new point is closest to the car,
  // instead of resetting to 0, so get_waypoint continues from the right spot.
  int nearest_i = 0;
  double nearest_dist = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
  for (int i = 1; i < num_waypoints; i++) {
    double dist = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_i = i;
    }
  }
  waypoints.index = nearest_i;
  waypoints.velocity_index = nearest_i;

  last_waypoint_time = this->now();
  have_waypoint_time = true;
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
  if (num_waypoints == 0) {
    return;
  }

  // Waypoints go stale if the triangulator stops publishing (e.g. the car
  // has drifted far enough off-track that detection_generator can no longer
  // see any cones) -- without this, we'd keep driving full-speed toward
  // whatever get_waypoint() last resolved to, forever.
  if (have_waypoint_time &&
      (this->now() - last_waypoint_time).seconds() > waypoint_staleness_timeout) {
    publish_stop();
    return;
  }

  x_car_world = odom_submsgObj->pose.pose.position.x;
  y_car_world = odom_submsgObj->pose.pose.position.y;

  car_orient_w = odom_submsgObj->pose.pose.orientation.w;
  car_orient_x = odom_submsgObj->pose.pose.orientation.x;
  car_orient_y = odom_submsgObj->pose.pose.orientation.y;
  car_orient_z = odom_submsgObj->pose.pose.orientation.z;

  // interpolate between different way-points
  get_waypoint();

  // Nothing passed the not-behind-car/in-range test anywhere in the array --
  // there is no legitimate target this cycle. Stop rather than coast toward
  // whatever waypoints.index was left at.
  if (!has_valid_waypoint) {
    publish_stop();
    return;
  }

  // Sanity cap: a second, independent guard in case a valid-looking target
  // is still absurdly far away (e.g. a stale-but-technically-in-range point).
  double target_distance = p2pdist(waypoints.X[waypoints.index], x_car_world,
                                    waypoints.Y[waypoints.index], y_car_world);
  if (target_distance > max_target_distance) {
    publish_stop();
    return;
  }

  // use tf2 transform the goal point
  if (!transformandinterp_waypoint()) {
    return;
  }

  // Calculate curvature/steering angle
  double steering_angle = p_controller();

  // publish object and message: AckermannDriveStamped on drive topic
  publish_message(steering_angle);
}

void PurePursuit::timer_callback() {
  // Periodically check parameters and update
  K_p = this->get_parameter("K_p").as_double();
  velocity_percentage = this->get_parameter("velocity_percentage").as_double();
  waypoint_velocity = this->get_parameter("waypoint_velocity").as_double();
  min_lookahead = this->get_parameter("min_lookahead").as_double();
  max_lookahead = this->get_parameter("max_lookahead").as_double();
  lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
  steering_limit = this->get_parameter("steering_limit").as_double();
  waypoint_staleness_timeout = this->get_parameter("waypoint_staleness_timeout").as_double();
  max_target_distance = this->get_parameter("max_target_distance").as_double();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_ptr = std::make_shared<PurePursuit>();  // initialise node pointer
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
