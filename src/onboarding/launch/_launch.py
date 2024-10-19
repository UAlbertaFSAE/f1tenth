from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file!"""
    return LaunchDescription(
        [
            Node(package="onboarding", executable="odom_publisher", name="odom_pub"),
            Node(package="onboarding", executable="odom_relay", name="relay_sub"),
        ]
    )
