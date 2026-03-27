#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher() : Node("odom_publisher")
  {
    //Declare params
    this->declare_parameter<double>("v", 0.0);
    this->declare_parameter<double>("d", 0.0);

    //create publisher
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    //1 KHz timer
    timer_ = this->create_wall_timer(1ms, std::bind(&OdomPublisher::publishDriveMsg, this));
  }

private:
  void publishDriveMsg()
  {
    double v = this->get_parameter("v").as_double();
    double d = this->get_parameter("d").as_double();

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.drive.speed = v;
    msg.drive.steering_angle = d;

    RCLCPP_INFO(this->get_logger(), "Publishing: v=%.2f, d=%.2f", v, d);

    publisher_->publish(msg);
  }
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
