import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import String


class Odom_Relay(Node):
    def __init__(self):
        super().__init__("odom_relay")
        self.subscription = self.create_subscription(
            String, "/drive", self.listener_callback, 1
        )

        self.publisher_ = self.create_publisher(String, "/drive_relay", 1)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.drive.steering_angle)
        self.get_logger().info('I heard: "%s"' % msg.drive.speed)

        param_v = msg.drive.speed * 3
        param_d = msg.drive.steering_angle * 3

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = param_v
        msg.drive.speed = param_d
        self.publisher_.publish(msg)
            "speed"
            +str(msg.drive.speed)
           + "steering angle"
            +str(msg.drive.steering_angle)


def main(args=None):
    rclpy.init(args=args)

    odom_relay = Odom_Relay()

    rclpy.spin(odom_relay)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
