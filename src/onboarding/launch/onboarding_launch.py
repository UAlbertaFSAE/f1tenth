from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the publisher and relay nodes."""
    return LaunchDescription(
        [
            Node(
                package="onboarding",
                executable="odom_publisher",
                name="publisher",
            ),
            Node(
                package="onboarding",
                executable="odom_relay",
                name="relay",
            ),
        ]
    )
