# Copyright (c) 2026, Your Name. Subject to the MIT license.

"""Publish a drive command built from the ``v`` and ``d`` parameters at 1 kHz."""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

PUBLISH_RATE_HZ = 1000.0  # 1 kHz


class OdomPublisher(Node):
    """Publish an AckermannDriveStamped message on ``drive`` at a fixed rate."""

    def __init__(self) -> None:
        """Declare the parameters, create the publisher and start the timer."""
        super().__init__("odom_publisher")

        # Parameters: a name and a default value.
        self.declare_parameter("v", 0.0)  # speed in m/s
        self.declare_parameter("d", 0.0)  # steering angle in radians

        # Publisher: message type, topic name, queue depth
        self._publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # Timer
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_drive)

    def _publish_drive(self) -> None:
        """Build one drive message from the current parameter values and publish it."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.get_parameter("v").value
        msg.drive.steering_angle = self.get_parameter("d").value
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Start the node and keep it running until ctrl+c."""
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
