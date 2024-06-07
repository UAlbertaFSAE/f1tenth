#include "safety_node.h"

Safety::Safety() : Node("safety_node") {
  speed_ = 0.0;
  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Safety::ScanCallback, this, _1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10, std::bind(&Safety::OdomCallback, this, _1));
}

void Safety::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  // just grabbing forward position of vehicle for calculations later
  this->speed_ = msg->twist.twist.linear.x;
  RCLCPP_INFO(this->get_logger(), "current speed: '%f'", this->speed_);
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
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "emergency brake engaged at speed '%f'", this->speed_);
    this->publisher_->publish(drive_msg);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}
