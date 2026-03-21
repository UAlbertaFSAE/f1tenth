
#include <chrono>
#include <memory>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#define ODOM_PUB_TIMEOUT_MS 1.0f  // 1KHz
#define ODOM_PUB_QUEUE_SIZE 10

class OdomPublisher : public rclcpp::Node {
 public:
  OdomPublisher() : Node("odom_publisher") {
    // create a publisher for the odometry message
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    // timer to publish odometry at a fixed rate
    timer_ =
        this->create_wall_timer(std::chrono::duration<double, std::milli>(ODOM_PUB_TIMEOUT_MS),
                                std::bind(&OdomPublisher::timer_callback, this));

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    this->declare_parameter("v", 0.0f);
    this->declare_parameter("d", 0.0f);

    // Set a callback for this node's integer parameter, "an_int_param"
    auto cb_v = [this](const rclcpp::Parameter& p) {
      RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter v of type %s: \"%f\"",
                  p.get_type_name().c_str(), p.as_double());
      speed = p.as_double();
    };
    auto cb_d = [this](const rclcpp::Parameter& p) {
      RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter d of type %s: \"%f\"",
                  p.get_type_name().c_str(), p.as_double());
      steering_angle = p.as_double();
    };
    param_cb_handles_.push_back(param_subscriber_->add_parameter_callback("v", cb_v));
    param_cb_handles_.push_back(param_subscriber_->add_parameter_callback("d", cb_d));
  }

 private:
  void timer_callback() {
    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    auto now = this->get_clock()->now();

    message.drive.speed = speed;
    message.drive.steering_angle = steering_angle;
    message.header.stamp = now;

    publisher_->publish(message);
  }
  double speed = 0.0f;
  double steering_angle = 0.0f;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> param_cb_handles_;
};
