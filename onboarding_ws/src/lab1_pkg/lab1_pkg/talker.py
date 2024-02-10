import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)
        timer_period = 0.5  # seconds
        self.declare_parameter("v", 1.0)
        self.declare_parameter("d", 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        param_v = self.get_parameter("v").value
        param_d = self.get_parameter("d").value

        # self.get_logger().info("Speed field = %s" % param_v)
        # self.get_logger().info("Steering angle = %s" % param_d)

        msg = AckermannDriveStamped()
        # # self.get_logger().info("blabla = %s" % msg)
        msg.drive.steering_angle = param_v

        msg.drive.speed = param_d

        self.publisher_.publish(msg)
        # # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
