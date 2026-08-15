"""Node for publishing odometry drive commands."""

from typing import List, Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node


class OdomPublisher(Node):
    """ROS 2 Node that publishes Ackermann drive messages on the 'drive' topic."""

    def __init__(self) -> None:
        """Initialize the OdomPublisher node, parameters, and timer."""
        super().__init__("odom_publisher")

        # Configure parameter bounds
        v_range = FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.01)
        v_desc = ParameterDescriptor(
            description="Linear velocity parameter v",
            floating_point_range=[v_range],
        )

        d_range = FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.01)
        d_desc = ParameterDescriptor(
            description="Steering angle parameter d",
            floating_point_range=[d_range],
        )

        # Declare parameters 'v' and 'd' default to 0.0
        self.declare_parameter("v", 0.0, v_desc)
        self.declare_parameter("d", 0.0, d_desc)

        # Publisher targeting topic 'drive'
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # Timer running at 1000 Hz (1 ms interval = 0.001 s)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self) -> None:
        """Publish drive commands based on current parameter values."""
        v = self.get_parameter("v").get_parameter_value().double_value
        d = self.get_parameter("d").get_parameter_value().double_value

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(d)

        self.publisher_.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Execute the main lifecycle loop for the odom_publisher node."""
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
