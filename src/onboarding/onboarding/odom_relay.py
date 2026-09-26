import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    """ROS 2 node for publishing/subscribing to the odometry data."""

    def __init__(self) -> None:
        """Declare parameters and create publisher and subscriber."""
        super().__init__("odom_relay")

        # Create a subscriber to the 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self.subscriber_callback, 10
        )

        # Create a publisher to the 'drive_relay' topic
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

        self.get_logger().info(
            "Odom Relay Node initialized. Listening to /drive and multiplying values by 3..."
        )

    def subscriber_callback(self, msg: AckermannDriveStamped) -> None:
        """Multiply incoming drive parameters by 3 and republishes the scaled message."""
        # Create a new message for the relay topic
        relay_msg = AckermannDriveStamped()

        relay_msg.header.stamp = self.get_clock().now().to_msg()

        # Multiply incoming speed and steering angle fields by 3
        relay_msg.drive.speed = msg.drive.speed * 3.0
        relay_msg.drive.steering_angle = msg.drive.steering_angle * 3.0

        # Publish the scaled data
        self.publisher_.publish(relay_msg)


def main() -> None:
    """Run the node until shutdown."""
    rclpy.init()
    node = OdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
