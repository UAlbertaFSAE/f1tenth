// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * Publish zero drive commands on a mux input above navigation whenever the
 * autonomous pipeline has no fresh valid path, odometry, or controller output.
 */
class DriveWatchdog : public rclcpp::Node {
 public:
  DriveWatchdog() : Node("drive_watchdog") {
    path_timeout_s_ = declare_parameter<double>("path_timeout_s", 0.4);
    max_plan_age_s_ = declare_parameter<double>("max_plan_age_s", 2.0);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.2);
    drive_timeout_s_ = declare_parameter<double>("drive_timeout_s", 0.2);
    const double rate_hz = declare_parameter<double>("rate_hz", 20.0);

    for (const double value :
         {path_timeout_s_, max_plan_age_s_, odom_timeout_s_, drive_timeout_s_, rate_hz}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Watchdog timeouts and rate_hz must be finite and positive");
      }
    }

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        declare_parameter<std::string>("waypoint_topic", "/waypoints"), 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          has_path_ = false;
          if (msg->poses.empty() || msg->header.stamp.sec < 0 ||
              msg->header.stamp.nanosec >= 1000000000u ||
              (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0))
            return;
          const rclcpp::Time generated(msg->header.stamp, get_clock()->get_clock_type());
          const auto received = now();
          if (generated > received) return;
          last_path_ = received;
          plan_generated_ = generated;
          has_path_ = true;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/odom"), 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
          last_odom_ = now();
          has_odom_ = true;
        });
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        declare_parameter<std::string>("drive_topic", "/drive"), 10,
        [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr) {
          last_drive_ = now();
          has_drive_ = true;
        });
    stop_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        declare_parameter<std::string>("stop_topic", "/safety_stop"), 10);
    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() { check(); });
  }

 private:
  /**
   * Publish a zero command when any input is missing or older than its timeout.
   */
  void check() {
    const auto t = now();
    const auto fresh = [&t](const rclcpp::Time& stamp, double timeout) {
      const double age = (t - stamp).seconds();
      return age >= 0.0 && age <= timeout;
    };
    const bool healthy =
        has_path_ && has_odom_ && has_drive_ && fresh(last_path_, path_timeout_s_) &&
        fresh(plan_generated_, max_plan_age_s_) && fresh(last_odom_, odom_timeout_s_) &&
        fresh(last_drive_, drive_timeout_s_);
    if (healthy) return;
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = t;
    stop_pub_->publish(stop);
  }

  double path_timeout_s_, max_plan_age_s_, odom_timeout_s_, drive_timeout_s_;
  bool has_path_ = false, has_odom_ = false, has_drive_ = false;
  rclcpp::Time plan_generated_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_drive_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveWatchdog>());
  rclcpp::shutdown();
  return 0;
}
