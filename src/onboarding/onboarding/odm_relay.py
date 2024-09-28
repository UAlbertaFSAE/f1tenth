from typing import Any

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdmRelay(Node):
    """odmrelay class.

    Args:
        Node (_type_): _description_
    """

    def __init__(self) -> None:
        """Odmrelay init function."""
        super().__init__("odm_relay")

        self.subscription = self.create_subscription(
            AckermannDriveStamped, "drive", self.listener_callback, 10
        )
        # self.subscription  # prevent unused variable warning

        self.declare_parameter("v", 1.0)
        self.declare_parameter("d", 1.0)
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )

    def listener_callback(self, msg: AckermannDriveStamped) -> None:
        """Listens for message from drive topic.

        Args:
            msg (AckermannDriveStamped): _description_
        """
        new_v = msg.drive.speed * 3.0
        new_d = msg.drive.steering_angle * 3.0

        newmsg = AckermannDriveStamped()
        newmsg.drive.speed = new_v
        newmsg.drive.steering_angle = new_d

        self.publisher_.publish(newmsg)

        self.get_logger().info(f'new speed "{newmsg.drive.speed}"')


def main(args: Any = None) -> None:
    """Main method for odm relay.

    Args:
        args (Any, optional): _description_. Defaults to None.
    """
    rclpy.init(args=args)

    odm_relay = OdmRelay()

    rclpy.spin(odm_relay)

    odm_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
