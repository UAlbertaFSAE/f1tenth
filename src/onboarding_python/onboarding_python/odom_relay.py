from typing import Any

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    """Node for manipulating spoofed odom data and relaying it."""

    def __init__(self) -> None:
        """Initialize subscriber and publisher for callback."""
        super().__init__("odom_publisher")

        self._subscription = self.create_subscription(
            AckermannDriveStamped, "/drive", self._subscriber_callback, 10
        )
        self._publisher = self.create_publisher(
            AckermannDriveStamped, "/drive_relay", 10
        )

        self.get_logger().info("odom relay has been started!")

    def _subscriber_callback(self, msg: AckermannDriveStamped) -> None:
        relay_msg = AckermannDriveStamped()
        relay_msg.drive.speed = msg.drive.speed * 3
        relay_msg.drive.steering_angle = msg.drive.steering_angle * 3

        relay_msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(relay_msg)


def main(args: Any = None) -> None:
    """Spin up odom_relay node."""
    rclpy.init(args=args)
    odom_relay = OdomRelay()
    rclpy.spin(odom_relay)
    odom_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
