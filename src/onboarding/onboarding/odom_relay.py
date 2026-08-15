"""Node for subscribing to odometry data, scaling it, and re-publishing."""

from typing import List, Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    """ROS 2 Node that subscribes to 'drive', scales commands, and publishes to 'drive_relay'."""

    def __init__(self) -> None:
        """Initialize the OdomRelay node, subscriber, and publisher."""
        super().__init__("odom_relay")

        # Subscriber listening to 'drive'
        self.subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self.drive_callback, 10
        )

        # Publisher targeting 'drive_relay'
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        """Multiply incoming speed and steering angle by 3 and publish to drive_relay."""
        relay_msg = AckermannDriveStamped()
        relay_msg.header.stamp = self.get_clock().now().to_msg()

        relay_msg.drive.speed = msg.drive.speed * 3.0
        relay_msg.drive.steering_angle = msg.drive.steering_angle * 3.0

        self.publisher_.publish(relay_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Execute the main lifecycle loop for the odom_relay node."""
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
