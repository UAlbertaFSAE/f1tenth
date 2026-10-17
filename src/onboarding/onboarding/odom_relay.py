# Copyright (c) 2026, Your Name. Subject to the MIT license.

"""Relay drive commands from ``drive`` to ``drive_relay``, scaled by 3."""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

SCALE = 3.0


class OdomRelay(Node):
    """Subscribe to ``drive``, scale speed and steering, publish on ``drive_relay``."""

    def __init__(self) -> None:
        """Create the publisher and the subscription."""
        super().__init__("odom_relay")

        self._publisher = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

        # Subscription: message type, topic name, callback, queue depth.
        # ROS calls _relay_drive once for every message that arrives on ``drive``.
        self._subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self._relay_drive, 10
        )

    def _relay_drive(self, msg: AckermannDriveStamped) -> None:
        """Scale the incoming speed and steering angle and republish them.

        Args:
            msg: The drive message received on the ``drive`` topic.
        """
        relay_msg = AckermannDriveStamped()
        # Keep the original header: the scaled command still describes the same moment.
        relay_msg.header = msg.header
        relay_msg.drive.speed = msg.drive.speed * SCALE
        relay_msg.drive.steering_angle = msg.drive.steering_angle * SCALE
        self._publisher.publish(relay_msg)


def main(args: list[str] | None = None) -> None:
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init(args=args)
    node = OdomRelay()
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
