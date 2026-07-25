#include "safety_node.h"

#include <algorithm>
#include <limits>

Safety::Safety() : Node("safety_node") {
  speed_ = 0.0;
  car_x_ = 0.0;
  car_y_ = 0.0;
  has_pose_ = false;
  has_cones_ = false;

  this->declare_parameter("min_safe_cone_distance", 0.35);
  min_safe_cone_distance_ = this->get_parameter("min_safe_cone_distance").as_double();

  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Safety::ScanCallback, this, _1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10, std::bind(&Safety::OdomCallback, this, _1));
  cone_subscription_ = this->create_subscription<rc_interfaces::msg::Cones>(
      "/cone_positions", 10, std::bind(&Safety::ConeCallback, this, _1));
}

void Safety::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  // just grabbing forward position of vehicle for calculations later
  this->speed_ = msg->twist.twist.linear.x;
  this->car_x_ = msg->pose.pose.position.x;
  this->car_y_ = msg->pose.pose.position.y;
  this->has_pose_ = true;
  RCLCPP_INFO(this->get_logger(), "current speed: '%f'", this->speed_);

  CheckConeBoundary();
}

void Safety::ConeCallback(const rc_interfaces::msg::Cones::ConstSharedPtr cones_msg) {
  this->latest_cones_ = *cones_msg;
  this->has_cones_ = true;

  CheckConeBoundary();
}

void Safety::CheckConeBoundary() {
  if (!has_pose_ || !has_cones_ || latest_cones_.cones.empty()) {
    return;
  }

  double nearest_dist = std::numeric_limits<double>::max();
  for (const auto& cone : latest_cones_.cones) {
    const double dx = static_cast<double>(cone.x) - car_x_;
    const double dy = static_cast<double>(cone.y) - car_y_;
    const double dist = std::sqrt(dx * dx + dy * dy);
    nearest_dist = std::min(nearest_dist, dist);
  }

  if (nearest_dist < min_safe_cone_distance_) {
    RCLCPP_INFO(this->get_logger(), "nearest cone %.2fm away (limit %.2fm)", nearest_dist,
                min_safe_cone_distance_);
    EmergencyBrake("cone boundary");
  }
}

void Safety::ScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  /// calculate instantaneous time to collision
  bool emergency_breaking = false;
  for (std::size_t i = 0; i < scan_msg->ranges.size(); i++) {
    double r = scan_msg->ranges[i];
    if (std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min) {
      continue;
    }

    double threshold = 1;  // To be tuned in real vehicle
    double time_to_collision =
        r / std::max(this->speed_ *
                         std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment),
                     0.001);
    if (time_to_collision < threshold) {
      emergency_breaking = true;
      break;
    }
  }

  // publish command to brake
  if (emergency_breaking) {
    EmergencyBrake("time-to-collision");
  }
}

void Safety::EmergencyBrake(const char* reason) {
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = 0.0;
  RCLCPP_INFO(this->get_logger(), "emergency brake engaged (%s) at speed '%f'", reason,
              this->speed_);
  this->publisher_->publish(drive_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}
