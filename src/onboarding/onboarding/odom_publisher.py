#!/usr/bin/env python3

# publisher listens to two ROS parameters v and d.
# publisher publishes an AckermannDriveStamped message to a topic named drive with the following fields set:
# speed equal to the v parameter
# steering_angle equal to the d parameter
# stamp equal to the current time (hint: you can get this using a method on the parent Node class)
# your node publishes data at a rate of 1KHz


import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")  # name of the node

        self.declare_parameter("v", 0.0)  # v = speed
        self.declare_parameter("d", 0.0)  # d = steering angle

        # stamp = self.get_clock().now()  # current time stamp

        # publisher
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive", 10
        )  # create publisher to publish AckermannDriveStamped messages to 'drive' topic
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1KHz timer

        self.get_logger().info("OdomPublisher node initialized")

    def timer_callback(self):
        speed = self.get_parameter("v").get_parameter_value().double_value  # v = speed
        steering = (
            self.get_parameter("d").get_parameter_value().double_value
        )  # d = steering angle

        msg = AckermannDriveStamped()  # run every 0.001 seconds

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.publisher_.publish(msg)  # publish the message


def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)  # keep the node running
    node.destroy_node()  # destroy the node when done
    rclpy.shutdown()  # shutdown rclpy


if __name__ == "__main__":
    main()


# test code with: colcon build --packages-select onboarding --symlink-install, then source install/setup.bash, then ros2 run onboarding odom_publisher with ros2 run onboarding odom_publisher

# Try updating the parameters v and d through the command line (find how in ROS2 Humble documentation) and echoing the drive topic to see changes. Try running ros2 topic hz /drive in a separate terminal to see if your data is being published near the 1KHz requirement.
