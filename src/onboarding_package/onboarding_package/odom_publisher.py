import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import String


class odom_publisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.get_logger().info("odom_publisher node has been started")
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )  # Creates publisher object
        timer_period = 0.001  # 1 Khz publish rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Declaring parameters
        self.declare_parameter("v", 1.0)
        self.declare_parameter("d", 1.0)

    def timer_callback(self):
        speed = self.get_parameter("v").value
        steering_angle = self.get_parameter("d").value

        msg = (
            AckermannDriveStamped()
        )  # Creates empty AckermannDriveStamped type message
        # Next three lines populate the empty message
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)  # Publishes to the /drive topic
        self.get_logger().info(
            f"Published speed = {speed}, steering angle = {steering_angle}, timestamp = {msg.header.stamp}"
        )


def main(args=None):
    rclpy.init(args=args)
    Odom_publisher = odom_publisher()
    rclpy.spin(Odom_publisher)
    Odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
