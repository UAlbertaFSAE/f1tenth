import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(String, "drive", self.listener_callback, 10)

        self.publisher_ = self.create_publisher(String, "drive_relay", 10)

    # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.drive.steering_angle)
        self.get_logger().info('I heard: "%s"' % msg.drive.speed)

        param_v = msg.drive.speed * 3
        param_d = msg.drive.steering_angle * 3

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = param_v
        msg.drive.speed = param_d
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
