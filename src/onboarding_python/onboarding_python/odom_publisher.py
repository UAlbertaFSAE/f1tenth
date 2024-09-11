from math import pi
from typing import Any

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node


class OdomPublisher(Node):
    """Node for publishing spoofed odom data."""

    def __init__(self) -> None:
        """Declare parameters and initialize publisher and timer."""
        super().__init__("odom_publisher")

        # floating point ranges ensure parameters can't take absurd values
        speed_range = FloatingPointRange(from_value=-3.0, to_value=3.0, step=0.0)
        steering_angle_range = FloatingPointRange(
            from_value=-2 * pi, to_value=2 * pi, step=0.0
        )
        speed_desc = ParameterDescriptor(floating_point_range=[speed_range])
        steering_angle_desc = ParameterDescriptor(
            floating_point_range=[steering_angle_range]
        )
        self.declare_parameter(name="v", value=0.0, descriptor=speed_desc)
        self.declare_parameter(name="d", value=0.0, descriptor=steering_angle_desc)

        self._publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self._timer = self.create_timer(0.001, self._timer_callback)

        self.get_logger().info("odom publisher has been started!")

    def _timer_callback(self) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = self.get_parameter("v").get_parameter_value().double_value
        msg.drive.steering_angle = (
            self.get_parameter("d").get_parameter_value().double_value
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)


def main(args: Any = None) -> None:
    """Spin up odom_publisher node."""
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
