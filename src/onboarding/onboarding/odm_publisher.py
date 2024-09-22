import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from typing import Any


class OdmPublisher(Node):

    def __init__(self) -> None:
        super().__init__('odm_publisher')

        self.declare_parameter('v', 1.0)
        self.declare_parameter('d', 1.0)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = self.get_parameter('v').value
        msg.drive.steering_angle = self.get_parameter('d').value
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info('speed info: "%s"' % msg.drive.speed)
        self.i += 1


def main(args: Any = None) -> None:
    rclpy.init(args=args)

    odm_publisher = OdmPublisher()

    rclpy.spin(odm_publisher)

    odm_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
