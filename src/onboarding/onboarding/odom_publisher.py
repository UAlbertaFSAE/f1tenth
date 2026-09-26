import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomPublisher(Node):
    """ROS 2 node for publishing odometry data."""

    def __init__(self) -> None:
        """Declare parameters and create publisher."""
        super().__init__("odom_publisher")

        # Parameters
        self.declare_parameter("v", 0.0)
        self.declare_parameter("d", 0.0)

        # Create a publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # Set a 1KHz publish rate
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Odom Publisher initialized at 1KHz.")

    def timer_callback(self) -> None:
        """Publishes an AckermannDriveStamped message."""
        # Get parameter values
        v_param = self.get_parameter("v").get_parameter_value().double_value
        d_param = self.get_parameter("d").get_parameter_value().double_value

        # Construct the AckermannDriveStamped message
        msg = AckermannDriveStamped()

        # Set the header timestamp to the current node time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Assign the parameters to the drive fields
        msg.drive.speed = v_param
        msg.drive.steering_angle = d_param

        # Publish the message
        self.publisher_.publish(msg)


def main() -> None:
    """Run the node until shutdown."""
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
