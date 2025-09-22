import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("odom_relay")

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )
        self.subscriber = self.create_subscription(
            AckermannDriveStamped,
            "drive",
            self.listener_callback,
            10,
        )

        self._publish_count = 0

    def listener_callback(self, msg: AckermannDriveStamped) -> None:  # noqa: D102
        msg_updated = AckermannDriveStamped()  # This overwrites the incoming message!
        msg_updated.header.stamp = self.get_clock().now().to_msg()
        msg_updated.drive.speed = msg.drive.speed * 3.0  # msg.v doesn't exist - should be msg.drive.speed
        msg_updated.drive.steering_angle = msg.drive.steering_angle * 3.0  # msg.d doesn't exist - should be msg.drive.steering_angle
        self.publisher_.publish(msg_updated)
        self._publish_count += 1
        if self._publish_count % 1000 == 0:
            self.get_logger().info(
                f"Publishing sample v={msg_updated.drive.speed}, d={msg_updated.drive.steering_angle}"
            )


def main(args: list | None = None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
