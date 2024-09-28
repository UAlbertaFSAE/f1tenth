from typing import Any

import rclpy
import rclpy.time
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay")

        self.subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self.subscriberCallback, 10
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

    def subscriberCallback(self, msg: AckermannDriveStamped) -> None:
        # self.get_logger().info("recieved: " + str(msg.drive))
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
    rclpy.init(args=args)
    odom_relay = OdomRelay()
    rclpy.spin(odom_relay)

    odom_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main()
