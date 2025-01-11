import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self):
        super().__init__("odom_relay")
        self.subscription = self.create_subscription(
            AckermannDriveStamped, "/drive", self.listener_callback, 10
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "/drive_relay", 10
        )

    def listener_callback(self, msg):
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3
        new_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
