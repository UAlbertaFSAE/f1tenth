#!/usr/bin/env python3

# Create a node called odom_relay for subscribing to the odometry data that odom_publisher is publishing, manipulating that data, and publishing this new data. You will create this node in the same language you wrote your previous one in
# subscriber node subscribes to the drive topic
# subscriber callback, take the speed and steering angle from the incoming message and multiply both by 3, then publish these new values via another AckermannDriveStamped message to a topic named drive_relay


import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self):
        super().__init__("odom_relay")  # name of node

        # Create subscriber to drive topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            "drive",  # topic name
            self.drive_callback,
            10,  # queue size
        )
        self.subscription  # prevent unused variable warning

        # Create publisher to drive_relay topic
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive_relay", 10)

        self.get_logger().info("odom-relay node started")

    def drive_callback(self, msg):  # Create new message - multiplied values
        relayout_msg = AckermannDriveStamped()  # relay message

        relayout_msg.header.stamp = self.get_clock().now().to_msg()  # x3 timestamp
        relayout_msg.drive.speed = msg.drive.speed * 3.0
        relayout_msg.drive.steering_angle = msg.drive.steering_angle * 3.0

        # Publish the modified data
        self.publisher.publish(relayout_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
# test code with: colcon build --packages-select onboarding --symlink-install, then source install/setup.bash, then ros2 run onboarding odom_relay
