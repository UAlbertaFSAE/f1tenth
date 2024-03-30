from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lab1_pkg", namespace="minimal_publisher", executable="talker", name="sim"
            ),
            Node(
                package="lab1_pkg", namespace="minimal_subscriber", executable="relay", name="sim"
            ),
        ]
    )
