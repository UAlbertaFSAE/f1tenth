from typing import Any

import rclpy
import rclpy.time
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    """This class represents the subscriber/publisher node."""

    def __init__(self) -> None:
        """Initializes the node and subscriptions."""
        super().__init__("odom_relay")

        self.subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self.subscriber_callback, 10
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

    def subscriber_callback(self, msg: AckermannDriveStamped) -> None:
        """Publishes a message everytime there is a message received."""
        msg.drive.speed *= 3
        msg.drive.steering_angle *= 3
        self.publisher_.publish(msg)

        self.get_logger().info(
            "speed: "
            + str(msg.drive.speed)
            + " steering angle: "
            + str(msg.drive.steering_angle)
        )


def main(args: Any = None) -> None:
    """Main function."""
    rclpy.init(args=args)
    odom_relay = OdomRelay()
    rclpy.spin(odom_relay)

    odom_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main()
