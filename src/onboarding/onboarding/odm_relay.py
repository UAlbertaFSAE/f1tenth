import rclpy
from rclpy.node import Node
from typing import Any
from ackermann_msgs.msg import AckermannDriveStamped


class OdmRelay(Node):

    def __init__(self) -> None:
        super().__init__('odm_relay')

        self.subscription = self.create_subscription(AckermannDriveStamped, 'drive', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('v', 1.0)
        self.declare_parameter('d', 1.0)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg:AckermannDriveStamped) -> None:
        new_v = msg.drive.speed * 3.0
        new_d = msg.drive.steering_angle * 3.0

        newMsg = AckermannDriveStamped()
        newMsg.drive.speed = new_v
        newMsg.drive.steering_angle = new_d

        self.publisher_.publish(newMsg)

        self.get_logger().info('new speed "%s"' % newMsg.drive.speed)


def main(args : Any = None) -> None:
    rclpy.init(args=args)

    odm_relay = OdmRelay()

    rclpy.spin(odm_relay)

    odm_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
