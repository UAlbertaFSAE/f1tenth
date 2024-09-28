from typing import Any

import rclpy
import rclpy.time
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher")

        self.declare_parameter("v", 0)
        self.declare_parameter("d", 0)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self) -> None:
        v = self.get_parameter("v").get_parameter_value().double_value
        d = self.get_parameter("d").get_parameter_value().double_value

        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        # string = "speed:", str(v), "steering angle:", str(d)
        self.get_logger().info(
            "speed: "
            + str(msg.drive.speed)
            + " steering angle: "
            + str(msg.drive.steering_angle)
        )


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main()
